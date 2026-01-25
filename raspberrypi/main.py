import sys
import threading
import time
import camera.camera as camera
import teensy_communication.teensy_communication as teensy_communication
import helpers.helpers as helpers
import web_server.api.api as api
import logging



LOGGING_LEVEL = logging.DEBUG

class LogFormatter(logging.Formatter):
    RESET = "\033[0m"
    BOLD = "\033[1m"

    LEVEL_STYLES = {
        logging.DEBUG:    "\033[36m",  # Cyan
        logging.INFO:     "\033[32m",  # Green
        logging.WARNING:  "\033[33m",  # Yellow
        logging.ERROR:    "\033[31m",  # Red
        logging.CRITICAL: "\033[41m",  # Red background
    }

    def format(self, record) -> str:
        color = self.LEVEL_STYLES.get(record.levelno, "")
        levelname = record.levelname

        if color:
            record.levelname = (
                f"{self.BOLD}{color}{levelname}{self.RESET}"
            )

        msg = super().format(record)
        record.levelname = levelname
        return msg

def setup_logger(name: str | None = None, level=LOGGING_LEVEL) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    formatter = LogFormatter(
        f"[%(asctime)s] "
        f"[%(levelname)s] "
        f"[%(name)s]: %(message)s",
        datefmt="%H:%M:%S"
    )
    handler.setFormatter(formatter)

    logger.addHandler(handler)
    return logger

logger = setup_logger()
camera_logger = setup_logger("Camera Thread")
logic_logger = setup_logger("Logic Thread")
hardware_logger = setup_logger("Hardware Thread")



last_frame: camera.FrameData | None = None
last_frame_lock = threading.Lock()
def set_last_frame(frame: camera.FrameData | None):
    global last_frame
    with last_frame_lock:
        last_frame = frame
        api.set_camera_frame(frame.frame if frame else None)
def get_last_frame() -> camera.FrameData | None:
    global last_frame
    with last_frame_lock:
        return last_frame

hardware_data: teensy_communication.ParsedTeensyData | None = None
hardware_data_lock = threading.Lock()
def set_hardware_data(data: teensy_communication.ParsedTeensyData | None):
    global hardware_data
    with hardware_data_lock:
        hardware_data = data
def get_hardware_data() -> teensy_communication.ParsedTeensyData | None:
    global hardware_data
    with hardware_data_lock:
        return hardware_data

motor_speeds: list[int] = [0, 0, 0, 0]
motor_speeds_lock = threading.Lock()
def set_motor_speeds(speeds: list[int]):
    global motor_speeds
    with motor_speeds_lock:
        motor_speeds = speeds
def get_motor_speeds() -> list[int]:
    global motor_speeds
    with motor_speeds_lock:
        return motor_speeds



def camera_thread(stop_event):
    while not stop_event.is_set():
        frame = camera.get_frame()
        set_last_frame(frame)
        camera_logger.debug(f"Frame {frame.timestamp} captured")

def logic_thread(stop_event):
    while not stop_event.is_set():
        pass

def hardware_communication_thread(stop_event):
    with teensy_communication.TeensyCommunicator() as communicator:
        while not stop_event.is_set():
            communicator.set_motors(get_motor_speeds())
            data = communicator.read_data(timeout=0.5)
            if data:
                set_hardware_data(data)
                hardware_logger.debug(f"Data: {data}")



def main():
    stop_event = threading.Event()

    threads = [
        threading.Thread(target=camera_thread, args=(stop_event,)),
        threading.Thread(target=logic_thread, args=(stop_event,)),
        threading.Thread(target=hardware_communication_thread, args=(stop_event,)),
    ]

    for t in threads:
        logger.info(f"Starting thread: {t.name}")
        t.start()

    api.start()

    logger.info("System running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nStopping...\n")
        stop_event.set()

    for t in threads:
        logger.debug(f"Joining thread: {t.name}")
        t.join()


if __name__ == "__main__":
    main()
