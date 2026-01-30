import argparse
from dataclasses import dataclass
import json
import logging
import re
import sys
import time
import serial
from serial.tools import list_ports


LOG = logging.getLogger("teensy_communication")

TEENSY_BAUD = 38400
EXPECTED_FIELDS = 30
MOTOR_COUNT = 4


@dataclass
class CompassData:
    heading: int
    pitch: int
    roll: int

@dataclass
class IRData:
    angle: int
    distance: int
    sensors: list[int]
    status: int

@dataclass
class ParsedTeensyData:
    compass: CompassData
    ir: IRData
    line: list[int]
    raw: str
    timestamp: float


def parse_teensy_line(line: str) -> ParsedTeensyData:
    line = line.strip()
    m = re.match(r'^\{\s*"a"\s*=\s*"(?P<data>.*)"\s*\}\s*$', line)
    if not m:
        raise ValueError("Line does not match wrapper {\"a\"=\"...\"}")

    data = m.group("data")
    parts = data.split(",")
    if len(parts) != EXPECTED_FIELDS:
        raise ValueError(f"Wrong number of fields: expected {EXPECTED_FIELDS}, got {len(parts)}")

    try:
        parsed = ParsedTeensyData(
            CompassData(
                int(parts[0]),
                int(parts[1]),
                int(parts[2]),
            ),
            IRData(
                int(parts[3]),
                int(parts[4]),
                [int(x) for x in parts[5:17]],
                int(parts[17]),
            ),
            [int(x) for x in parts[18:]],
            line,
            time.time(),
        )
    except Exception as e:
        raise ValueError(f"Failed to parse numeric fields: {e}")

    return parsed


def list_serial_ports() -> list[str]:
    return [p.device for p in list_ports.comports()]


def open_serial(port: str, baud: int = TEENSY_BAUD, timeout: float = 1.0) -> serial.Serial:
    try:
        return serial.Serial(port, baudrate=baud, timeout=timeout)
    except serial.SerialException as e:
        ports = list_serial_ports()
        hint = ""
        if ports:
            hint = f" Available ports: {', '.join(ports)}."
        raise RuntimeError(f"Failed to open serial port {port}: {e}.{hint}") from e


def format_message(motor_speeds: list[int], kicker_state: bool) -> str:
    if len(motor_speeds) != MOTOR_COUNT:
        raise ValueError(f"Expected {MOTOR_COUNT} motor speeds, got {len(motor_speeds)}")
    
    clamped_values: list[int] = []
    for i, speed in enumerate(motor_speeds):
        if not isinstance(speed, (int, float)):
            raise ValueError(f"Motor {i} speed must be numeric, got {type(speed)}")
        clamped = max(-9999, min(9999, int(speed)))
        clamped_values.append(clamped)
    clamped_values.append(1 if kicker_state else 0)
    
    formatted_values: list[str] = []
    for speed in clamped_values:
        sign = '+' if speed >= 0 else '-'
        formatted_values.append(f"{sign}{abs(speed):04d}")
    
    data = ",".join(formatted_values)
    return '{"a"="' + data + '"}'


class TeensyCommunicator:
    def __init__(self, port: str = "/dev/serial0", baud: int = TEENSY_BAUD, timeout: float = 1.0, auto_connect: bool = True) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._log = logging.getLogger(f"teensy_communication.{port}")
        self.buffer: bytearray | None = None
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
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
    
    def read_line(self) -> str:
        if self.ser is None:
            raise RuntimeError("Serial port not open. Call connect() first.")
        raw = None
        
        if self.buffer is None:
            self.buffer = bytearray()

        if b'\n' in self.buffer:
            raw = self.buffer.split(b'\n', 1)[0] + b'\n'
            self.buffer = self.buffer[len(raw):]
        
        read_bytes = self.ser.read_all()
        if read_bytes:
            self.buffer.extend(read_bytes)

        if not raw:
            return ""
        try:
            return raw.decode("utf-8", errors="replace").strip()
        except Exception:
            return raw.decode(errors="replace").strip()
    
    def read_data(self, timeout: float | None = None) -> ParsedTeensyData | None:
        if self.ser is None:
            raise RuntimeError("Serial port not open. Call connect() first.")
        
        original_timeout = None
        if timeout is not None:
            original_timeout = self.ser.timeout
            self.ser.timeout = timeout
        
        
        try:
            line = self.read_line()
            if not line:
                return None
            try:
                parsed = parse_teensy_line(line)
                self._log.debug("Received data: %s", parsed)
                return parsed
            except ValueError as e:
                self._log.warning("Failed to parse line: %s (%s)", line, e)
                return None
        finally:
            if original_timeout is not None:
                self.ser.timeout = original_timeout
    
    def send_message(self, message: str) -> None:
        if self.ser is None:
            raise RuntimeError("Serial port not open. Call connect() first.")
        
        if not message.endswith('\n'):
            message = message + '\n'
        
        self.ser.write(message.encode('utf-8'))
        self._log.debug("Sent: %s", message.strip())
    
    def set_motors(self, motor_speeds: list, kicker_state: bool) -> None:
        message = format_message(motor_speeds, kicker_state)
        self.send_message(message)
        self._log.debug("Set motors: %s, Kicker state: %s", motor_speeds, kicker_state)
    
    def stop_motors(self) -> None:
        self.set_motors([0, 0, 0, 0], False)


def run_shell(port: str, out_file: str | None = None, raw_mode: bool = False) -> None:
    teensy = TeensyCommunicator(port=port, auto_connect=True)
    LOG.info("Listening. Press Ctrl-C to quit.")
    fh = None
    if out_file is not None:
        fh = open(out_file, "a", encoding="utf-8")
    try:
        while True:
            data = None
            if raw_mode:
                data = teensy.read_line()
            else:
                parsed = teensy.read_data()
                if parsed:
                    data = json.dumps(parsed, ensure_ascii=False)
            
            if data is None:
                continue
            
            if fh is not None:
                fh.write(data + "\n")
                fh.flush()
            else:
                print(data)
    except KeyboardInterrupt:
        LOG.info("Interrupted by user, closing serial.")
    finally:
        teensy.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Teensy serial receiver and parser")
    ap.add_argument("--port", "-p", default="/dev/serial0", help="Serial port (default /dev/serial0 - Pi UART TX/RX pins)")
    ap.add_argument("--out", "-o", help="Output file (jsonl)")
    ap.add_argument("--raw", action="store_true", help="Print raw incoming lines before parsing (debug)")
    ap.add_argument("--log", default="info", help="Logging level")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(message)s")

    try:
        run_shell(args.port, out_file=args.out, raw_mode=args.raw)
    except Exception as e:
        LOG.exception("Fatal error: %s", e)
        return 2

    return 0


if __name__ == "__main__":
    sys.exit(main())
