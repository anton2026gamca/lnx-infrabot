import logging
import sys
import multiprocessing
import time

import camera.camera as camera
import teensy_communication.teensy_communication as teensy_communication
import web_server.api.api as api
from helpers.helpers import LogFormatter, RobotMode, Suppress200Filter



LOGGING_LEVEL = logging.DEBUG


class BufferedLogHandler(logging.Handler):
    def __init__(self, logs_dict, logs_lock, next_log_id, formatter=None):
        super().__init__()
        self.logs_dict = logs_dict
        self.logs_lock = logs_lock
        self.next_log_id = next_log_id
        if formatter:
            self.setFormatter(formatter)
        elif LogFormatter is not None:
            self.setFormatter(LogFormatter(f"[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s", datefmt="%H:%M:%S"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            formatted = self.format(record)
            entry = {
                "message": formatted,
                "level": record.levelname,
                "logger": record.name,
                "time": getattr(record, "created", None),
            }
            with self.logs_lock:
                lid = self.next_log_id.value
                self.logs_dict[lid] = entry
                self.next_log_id.value += 1
        except Exception:
            self.handleError(record)

def setup_logger(name: str | None = None, level=LOGGING_LEVEL, logs_dict=None, logs_lock=None, next_log_id=None) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    formatter = LogFormatter(
        f"[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s",
        datefmt="%H:%M:%S"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    if logs_dict is not None and logs_lock is not None and next_log_id is not None:
        buffer_handler = BufferedLogHandler(logs_dict, logs_lock, next_log_id)
        buffer_handler.setLevel(level)
        logger.addHandler(buffer_handler)
    
    return logger


manager = multiprocessing.Manager()
shared_data: dict = {
    'last_frame': manager.dict(),
    'hardware_data': manager.dict(),
    'motor_speeds': manager.list([0, 0, 0, 0]),
    'kicker_state': manager.Value('b', False),
    'robot_mode': manager.Value('c', RobotMode.IDLE.encode()),
    'logs_buffer': manager.dict(),
    'next_log_id': manager.Value('i', 1),
    'locks': {
        'last_frame': manager.Lock(),
        'hardware_data': manager.Lock(),
        'motor_speeds': manager.Lock(),
        'kicker_state': manager.Lock(),
        'robot_mode': manager.Lock(),
        'logs_buffer': manager.Lock()
    }
}

logger = setup_logger(
    level=LOGGING_LEVEL, logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)
camera_logger = setup_logger(
    "Camera Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)
logic_logger = setup_logger(
    "Logic Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)
hardware_logger = setup_logger(
    "Hardware Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)
api_logger = setup_logger(
    "API Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)

def set_last_frame(frame: camera.FrameData | None):
    with shared_data['locks']['last_frame']:
        shared_data['last_frame'].clear()
        if frame:
            shared_data['last_frame']['frame'] = frame.frame
            shared_data['last_frame']['timestamp'] = frame.timestamp

def get_last_frame() -> camera.FrameData | None:
    with shared_data['locks']['last_frame']:
        if not shared_data['last_frame']:
            return None
        return camera.FrameData(
            frame=shared_data['last_frame']['frame'],
            timestamp=shared_data['last_frame']['timestamp']
        )

def set_hardware_data(data: teensy_communication.ParsedTeensyData | None):
    with shared_data['locks']['hardware_data']:
        shared_data['hardware_data'].clear()
        if data:
            for key, value in data.__dict__.items():
                shared_data['hardware_data'][key] = value

def get_hardware_data() -> teensy_communication.ParsedTeensyData | None:
    with shared_data['locks']['hardware_data']:
        if not shared_data['hardware_data']:
            return None
        data = teensy_communication.ParsedTeensyData.__new__(teensy_communication.ParsedTeensyData)
        for key, value in shared_data['hardware_data'].items():
            setattr(data, key, value)
        return data

def set_motor_speeds(speeds: list[int]):
    with shared_data['locks']['motor_speeds']:
        shared_data['motor_speeds'][:] = speeds

def get_motor_speeds() -> list[int]:
    with shared_data['locks']['motor_speeds']:
        return list(shared_data['motor_speeds'])

def set_kicker_state(state: bool):
    with shared_data['locks']['kicker_state']:
        shared_data['kicker_state'].value = state

def get_kicker_state() -> bool:
    with shared_data['locks']['kicker_state']:
        return shared_data['kicker_state'].value

def set_robot_mode(mode: str):
    with shared_data['locks']['robot_mode']:
        shared_data['robot_mode'].value = mode.encode()

def get_robot_mode() -> str:
    with shared_data['locks']['robot_mode']:
        return shared_data['robot_mode'].value.decode()

def get_logs(since_id: int = 0) -> tuple[list[dict], int]:
    with shared_data['locks']['logs_buffer']:
        items = [
            {"id": lid, **entry}
            for lid, entry in sorted(shared_data['logs_buffer'].items())
            if lid > since_id
        ]
        last_id = max(shared_data['logs_buffer'].keys()) if shared_data['logs_buffer'] else 0
    return items, last_id


def camera_process(stop_event):
    try:
        logging.getLogger("picamera2.picamera2").setLevel(logging.INFO)

        frames = 0
        time_start = time.time()

        camera_logger.info("Initializing camera...")
        camera.init_camera()
        camera_logger.info("Camera initialized successfully")
        
        while not stop_event.is_set():
            frame = camera.get_frame()
            frames += 1
            set_last_frame(frame)
            
            if time.time() > time_start + 1:
                camera_logger.debug(f"Camera FPS: {frames}")
                frames = 0
                time_start = time.time()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        camera_logger.error(f"{e}")

def logic_process(stop_event):
    try:
        while not stop_event.is_set():
            if get_robot_mode() == RobotMode.IDLE:
                pass
            if get_robot_mode() == RobotMode.MANUAL:
                pass
            if get_robot_mode() == RobotMode.AUTONOMOUS:
                pass
            time.sleep(1.0 / 60)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logic_logger.error(f"{e}")

def hardware_communication_process(stop_event):
    try:
        messages = 0
        time_start = time.time()

        with teensy_communication.TeensyCommunicator(port="/dev/ttyAMA0") as communicator:
            while not stop_event.is_set():
                communicator.set_motors(get_motor_speeds(), get_kicker_state())
                data = communicator.read_data(timeout=0.1)
                if data:
                    messages += 1
                    set_hardware_data(data)
                if time.time() > time_start + 1:
                    hardware_logger.debug(f"Messages per second: {messages}")
                    messages = 0
                    time_start = time.time()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        hardware_logger.error(f"{e}")

def api_process():
    try:
        api.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        api_logger.error(f"{e}")

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
    api.logs_getter = get_logs
    
    
    for logger_name in ('werkzeug', 'werkzeug.serving'):
        wl = logging.getLogger(logger_name)
        wl.setLevel(logging.INFO)
        wl.addFilter(Suppress200Filter())
        
        buffer_handler = BufferedLogHandler(
            shared_data['logs_buffer'],
            shared_data['locks']['logs_buffer'],
            shared_data['next_log_id']
        )
        buffer_handler.setLevel(logging.INFO)
        wl.addHandler(buffer_handler)
    
    stop_event = multiprocessing.Event()

    processes = [
        multiprocessing.Process(target=camera_process, args=(stop_event,), name="Camera Process"),
        multiprocessing.Process(target=logic_process, args=(stop_event,), name="Logic Process"),
        multiprocessing.Process(target=hardware_communication_process, args=(stop_event,), name="Hardware Communication Process"),
        multiprocessing.Process(target=api_process, name="API Process"),
    ]

    for p in processes:
        logger.info(f"Starting process: {p.name}")
        p.start()

    logger.info("System running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nStopping...\n")
        stop_event.set()

    for p in processes:
        logger.debug(f"Joining process: {p.name}")
        p.join(timeout=5)
        if p.is_alive():
            logger.warning(f"Process {p.name} did not terminate, forcing...")
            p.terminate()
            p.join()


if __name__ == "__main__":
    main()
