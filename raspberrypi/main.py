import logging
import sys
import threading
import time
from werkzeug.serving import make_server

import camera.camera as camera
import teensy_communication.teensy_communication as teensy_communication
import web_server.api.api as api
from helpers.helpers import LogFormatter, RobotMode




LOGGING_LEVEL = logging.INFO

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

kicker_state: bool = False
kicker_state_lock = threading.Lock()
def set_kicker_state(state: bool):
    global kicker_state
    with kicker_state_lock:
        kicker_state = state
def get_kicker_state() -> bool:
    global kicker_state
    with kicker_state_lock:
        return kicker_state


robot_mode: str = RobotMode.IDLE
robot_mode_lock = threading.Lock()
def set_robot_mode(mode: str):
    global robot_mode
    with robot_mode_lock:
        robot_mode = mode
def get_robot_mode() -> str:
    global robot_mode
    with robot_mode_lock:
        return robot_mode


def camera_thread(stop_event):
    while not stop_event.is_set():
        frame = camera.get_frame()
        set_last_frame(frame)
        camera_logger.debug(f"Frame {frame.timestamp} captured")

def logic_thread(stop_event):
    while not stop_event.is_set():
        if get_robot_mode() == RobotMode.IDLE:
            pass
        if get_robot_mode() == RobotMode.MANUAL:
            pass
        if get_robot_mode() == RobotMode.AUTONOMOUS:
            pass

def hardware_communication_thread(stop_event):
    with teensy_communication.TeensyCommunicator(port="/dev/ttyAMA0") as communicator:
        while not stop_event.is_set():
            communicator.set_motors(get_motor_speeds(), get_kicker_state())
            data = communicator.read_data(timeout=0.1)
            if data:
                set_hardware_data(data)
                hardware_logger.debug(f"Data: {data}")

def api_thread(stop_event):
    server = make_server('0.0.0.0', 5000, api.app, threaded=True)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    try:
        stop_event.wait()
    finally:
        server.shutdown()
        server_thread.join(timeout=2)

def api_get_camera_frame():
    data = get_last_frame()
    if data is None:
        return None
    return data.frame


def main():
    api.camera_frame_getter = api_get_camera_frame
    api.hardware_data_getter = get_hardware_data
    api.motor_speeds_getter = get_motor_speeds
    api.kicker_state_getter = get_kicker_state
    api.robot_mode_getter = get_robot_mode
    api.robot_mode_setter = set_robot_mode
    
    stop_event = threading.Event()

    threads = [
        threading.Thread(target=camera_thread, args=(stop_event,)),
        threading.Thread(target=logic_thread, args=(stop_event,)),
        threading.Thread(target=hardware_communication_thread, args=(stop_event,)),
        threading.Thread(target=api_thread, args=(stop_event,)),
    ]

    for t in threads:
        logger.info(f"Starting thread: {t.name}")
        t.start()

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
