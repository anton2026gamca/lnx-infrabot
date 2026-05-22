import argparse
import json
import logging
import struct
import sys
import time
from dataclasses import asdict, dataclass
from serial.tools import list_ports
import serial

try:
    from robot.profiling import profile_function
except ImportError:
    def profile_function(func):
        return func



logger = logging.getLogger("Teensy Communication")


DEFAULT_TEENSY_PORT = "/dev/ttyAMA0"
DEFAULT_TEENSY_BAUD = 230400
DEFAULT_TEENSY_TIMEOUT = 1

try:
    from robot.config import MOTOR_COUNT, LINE_SENSOR_COUNT
except ImportError:
    MOTOR_COUNT = 4
    LINE_SENSOR_COUNT = 12


SENSOR_DATA_MESSAGE_TYPE = 0x01
SENSOR_DATA_MESSAGE_LENGTH = 50
RUNNING_STATE_MESSAGE_TYPE = 0x02
SET_MOTORS_MESSAGE_TYPE = 0xFF


class MessageType:
    UNKNOWN = "unknown"
    SENSOR_DATA = "sensor_data"
    RUNNING_STATE = "running_state"


@dataclass(slots=True)
class CompassData:
    heading: int
    pitch: int
    roll: int

@dataclass(slots=True)
class IRData:
    angle: int
    distance: int
    sensors: list[int]
    status: int

@dataclass(slots=True)
class ParsedTeensyData:
    compass: CompassData
    ir: IRData
    line: list[int]  # 12 sensors with (min, max) values
    raw: bytes | None
    timestamp: float


@profile_function
def parse_sensor_data_binary(data: bytes) -> ParsedTeensyData:
    if len(data) != SENSOR_DATA_MESSAGE_LENGTH:
        raise ValueError(f"Sensor data length invalid: expected {SENSOR_DATA_MESSAGE_LENGTH}, got {len(data)}")

    checksum = data[12+LINE_SENSOR_COUNT*3]
    calculated_checksum = sum(data[0:12+LINE_SENSOR_COUNT*3]) % 256
    if checksum != calculated_checksum:
        raise ValueError(f"Checksum mismatch: expected {checksum}, calculated {calculated_checksum}")
    
    heading = struct.unpack('<h', data[2:4])[0]
    pitch = struct.unpack('<h', data[4:6])[0]
    roll = struct.unpack('<h', data[6:8])[0]
    
    angle = -struct.unpack('<h', data[8:10])[0] % 360
    distance = struct.unpack('<h', data[10:12])[0]
    
    line_data_bytes = data[12:12+LINE_SENSOR_COUNT*3]
    line_data = []

    for i in range(LINE_SENSOR_COUNT):
        sensor_bytes = line_data_bytes[i*3:(i+1)*3]
        sensor_min = struct.unpack('<H', sensor_bytes[0:2])[0] & 0x0FFF
        sensor_max = (struct.unpack('<H', sensor_bytes[1:3])[0] >> 4) & 0x0FFF
        line_data.append(sensor_max)
    
    parsed = ParsedTeensyData(
        CompassData(heading, pitch, roll),
        IRData(angle, distance, [], 0),
        line_data,
        data,
        time.time(),
    )
    
    return parsed

@dataclass
class RunningStateData:
    running: bool
    main_switch_value: bool
    bt_module_enabled: bool
    bt_module_value: bool

@profile_function
def parse_running_state_binary(data: bytes) -> RunningStateData:
    if len(data) != 4:
        raise ValueError(f"Running state message length invalid: expected 4, got {len(data)}")
    
    flags = data[2]
    
    return RunningStateData(
        running=(flags & 0x01) != 0,
        bt_module_enabled=(flags & 0x02) != 0,
        main_switch_value=(flags & 0x04) != 0,
        bt_module_value=(flags & 0x08) != 0,
    )


def list_serial_ports() -> list[str]:
    return [p.device for p in list_ports.comports()]


def open_serial(port: str | None = None, baud: int = DEFAULT_TEENSY_BAUD, timeout: float | None = DEFAULT_TEENSY_TIMEOUT) -> serial.Serial:
    try:
        return serial.Serial(port, baudrate=baud, timeout=timeout)
    except serial.SerialException as e:
        ports = list_serial_ports()
        hint = ""
        if ports:
            hint = f" Available ports: {', '.join(ports)}."
        raise RuntimeError(f"Failed to open serial port {port}: {e}.{hint}") from e


@profile_function
def format_message(motor_speeds: list[int], kicker_state: bool) -> bytes:
    """Format motor control message for Teensy.
    
    Message format:
    - Byte 0: '{' (0x7B)
    - Byte 1: 0xFF (SET_MOTORS_MESSAGE_TYPE)
    - Byte 2: flags (bit 0 = kicker, bits 1-4 = motor directions)
    - Bytes 3-6: motor speeds (absolute values, 0-255)
    - Byte 7: '}' (0x7D)
    
    Direction bits: 1 = forward/positive, 0 = reverse/negative
    """
    if len(motor_speeds) != MOTOR_COUNT:
        raise ValueError(f"Expected {MOTOR_COUNT} motor speeds, got {len(motor_speeds)}")
    
    msg = bytearray(8)
    msg[0] = ord('{')
    msg[1] = SET_MOTORS_MESSAGE_TYPE
    
    flags = 0x01 if kicker_state else 0x00
    
    for i, speed in enumerate(motor_speeds):
        clamped = max(-255, min(255, int(speed)))
        
        if clamped >= 0:
            flags |= (1 << (1 + i))
        
        msg[3 + i] = abs(clamped) & 0xFF
    
    msg[2] = flags
    msg[7] = ord('}')
    
    return bytes(msg)


class TeensyCommunicator:
    def __init__(self, port: str = DEFAULT_TEENSY_PORT, baud: int = DEFAULT_TEENSY_BAUD, timeout: float = DEFAULT_TEENSY_TIMEOUT, auto_connect: bool = True) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._log = logger
        self.buffer: bytearray | None = None
        self.auto_connect = auto_connect
        if auto_connect:
            self.connect()

    def connect(self) -> None:
        if self.ser is not None:
            self._log.warning("Serial port already open")
            return
        self._log.info("Opening serial port %s @ %d", self.port, self.baud)
        self.ser = open_serial(self.port, self.baud, self.timeout)

    def close(self) -> None:
        if self.ser is not None:
            self._log.info("Closing serial port %s", self.port)
            self.ser.close()
            self.ser = None
    
    def __enter__(self) -> "TeensyCommunicator":
        if not self.auto_connect:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
    
    @profile_function
    def read_messages(self) -> dict[int, bytes]:
        if self.ser is None:
            raise RuntimeError("Serial port not open. Call connect() first.")

        if self.buffer is None:
            self.buffer = bytearray()

        read_bytes = self.ser.read_all()
        if read_bytes:
            self.buffer.extend(read_bytes)

        buf = self.buffer
        n = len(buf)

        newest: dict[int, bytes] = {}

        i = n - 1
        last_used_char_index = -1

        while i >= 0:
            if buf[i] != ord('}'):
                i -= 1
                continue

            end = i

            if end < 2:
                break

            start = end - 1
            while start >= 0 and buf[start] != ord('{'):
                start -= 1

            if start < 0:
                break

            if start + 1 >= n:
                i = start - 1
                continue

            msg_type = buf[start + 1]

            if msg_type == SENSOR_DATA_MESSAGE_TYPE and SENSOR_DATA_MESSAGE_TYPE not in newest:
                expected_len = SENSOR_DATA_MESSAGE_LENGTH
            elif msg_type == RUNNING_STATE_MESSAGE_TYPE and RUNNING_STATE_MESSAGE_TYPE not in newest:
                expected_len = 4
            else:
                i = start - 1
                continue

            if end - start + 1 != expected_len:
                i = start - 1
                continue

            newest[msg_type] = bytes(buf[start:end + 1])
            last_used_char_index = max(last_used_char_index, end)

            i = start - 1

            if (SENSOR_DATA_MESSAGE_TYPE in newest and
                RUNNING_STATE_MESSAGE_TYPE in newest):
                break

        self.buffer = buf[last_used_char_index + 1:]

        return newest
    
    def send_message(self, message: bytes) -> None:
        if self.ser is None:
            raise RuntimeError("Serial port not open. Call connect() first.")
        self.ser.write(message)
    
    @profile_function
    def send_motors_message(self, motor_speeds: list, kicker_state: bool) -> None:
        message = format_message(motor_speeds, kicker_state)
        self.send_message(message)
    
    def send_stop_motors(self) -> None:
        self.send_motors_message([0, 0, 0, 0], False)


def run_shell(port: str, out_file: str | None = None, raw_mode: bool = False) -> None:
    teensy = TeensyCommunicator(port=port, auto_connect=True)
    logger.info("Listening. Press Ctrl-C to quit.")
    fh = None
    if out_file is not None:
        fh = open(out_file, "a", encoding="utf-8")
    try:
        while True:
            data = None
            if raw_mode and teensy.ser is not None:
                read_bytes = teensy.ser.read_all()
                if read_bytes:
                    data = read_bytes.decode("utf-8", errors="replace")
            else:
                messages = teensy.read_messages()
                if SENSOR_DATA_MESSAGE_TYPE in messages:
                    try:
                        parsed = parse_sensor_data_binary(messages[SENSOR_DATA_MESSAGE_TYPE])
                        data = json.dumps(asdict(parsed)) + "\n"
                    except Exception as e:
                        logger.warning(f"Failed to parse sensor data: {e}")
                if RUNNING_STATE_MESSAGE_TYPE in messages:
                    try:
                        parsed = parse_running_state_binary(messages[RUNNING_STATE_MESSAGE_TYPE])
                        data = json.dumps(asdict(parsed)) + "\n"
                    except Exception as e:
                        logger.warning(f"Failed to parse running state: {e}")
            
            if data is None:
                continue
            
            if fh is not None:
                fh.write(data)
                fh.flush()
            else:
                print(data, end="")
    except KeyboardInterrupt:
        logger.info("Interrupted by user, closing serial.")
    finally:
        teensy.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Teensy serial receiver and parser")
    ap.add_argument("--port", "-p", default=DEFAULT_TEENSY_PORT, help=f"Serial port (default {DEFAULT_TEENSY_PORT} - Pi UART TX/RX pins)")
    ap.add_argument("--out", "-o", help="Output file (jsonl)")
    ap.add_argument("--raw", action="store_true", help="Print raw incoming messages before parsing (debug)")
    ap.add_argument("--log", default="info", help="Logging level")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(message)s")

    try:
        run_shell(args.port, out_file=args.out, raw_mode=args.raw)
    except Exception as e:
        logger.exception("Fatal error: %s", e)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
