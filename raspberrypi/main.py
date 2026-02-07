import json
import logging
import math
import multiprocessing
import multiprocessing.shared_memory
import numpy as np
import os
import time

import helpers.camera as camera
import helpers.teensy_communication as teensy_communication
import web_server.api.api as api
from helpers.helpers import RobotMode, Suppress200Filter, setup_logger, BufferedLogHandler, RobotManualControl, calculate_motors_speeds
from config import (
    LOG_LEVEL, FRAME_SIZE_B, FRAME_HEIGHT, FRAME_WIDTH,
    CAMERA_MIN_FRAME_INTERVAL, LOGIC_LOOP_PERIOD, IDLE_SLEEP_DURATION,
    TEENSY_PORT, TEENSY_BAUD, TEENSY_TIMEOUT,
    LINE_SENSOR_COUNT, LINE_DETECTION_THRESHOLDS, LINE_DETECTION_DARK_LINE,
    CALIBRATION_FILE_PATH
)



manager = multiprocessing.Manager()

shared_data: dict = {
    'frame_buffer': multiprocessing.shared_memory.SharedMemory(create=True, size=FRAME_SIZE_B),
    'frame_timestamp': multiprocessing.Value('d', 0.0),
    'frame_ready': multiprocessing.Value('b', False),
    'hardware_data': manager.dict(),
    'motor_speeds': multiprocessing.Array('i', [0, 0, 0, 0]),
    'kicker_state': multiprocessing.Value('b', False),
    'robot_mode': multiprocessing.Array('b', 20),
    'logs_buffer': manager.dict(),
    'next_log_id': multiprocessing.Value('i', 1),
    'manual_control': multiprocessing.Array('d', [0.0, 0.0, 0.0]),
    'compass_reset': multiprocessing.Value('b', False),
    'line_detection_thresholds': multiprocessing.Array('i', LINE_DETECTION_THRESHOLDS),
    'line_detected': multiprocessing.Array('b', [False] * LINE_SENSOR_COUNT),
    'line_calibration_min': multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT),
    'line_calibration_max': multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT),
    'line_calibration_active': multiprocessing.Value('b', False),
    'line_detection_dark': multiprocessing.Value('b', LINE_DETECTION_DARK_LINE),
    'running_state': manager.dict(),
    'locks': {
        'frame': multiprocessing.Lock(),
        'hardware_data': multiprocessing.Lock(),
        'robot_mode': multiprocessing.Lock(),
        'logs_buffer': multiprocessing.Lock(),
        'line_calibration': multiprocessing.Lock(),
        'running_state': multiprocessing.Lock(),
        'calibration_storage': multiprocessing.Lock(),
    }
}


idle_bytes = RobotMode.IDLE.encode()[:20]
for i in range(len(idle_bytes)):
    shared_data['robot_mode'][i] = idle_bytes[i]

def get_logs(since_id: int = 0) -> tuple[list[dict], int]:
    with shared_data['locks']['logs_buffer']:
        items = [
            {"id": lid, **entry}
            for lid, entry in sorted(shared_data['logs_buffer'].items())
            if lid > since_id
        ]
        last_id = max(shared_data['logs_buffer'].keys()) if shared_data['logs_buffer'] else 0
    return items, last_id


LOGGING_LEVEL = getattr(logging, LOG_LEVEL)

logger = setup_logger(
    "Shared Data Manager", level=LOGGING_LEVEL, logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)


#region Camera

camera_logger = setup_logger(
    "Camera Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)

def camera_process(stop_event):
    try:
        if LOGGING_LEVEL == logging.DEBUG:
            logging.getLogger("picamera2.picamera2").setLevel(logging.INFO)

        frames = 0
        frames_processed = 0
        time_start = time.time()
        last_frame_time = 0

        camera_logger.info("Initializing camera...")
        camera.init_camera()
        camera_logger.info("Camera initialized successfully")
        
        while not stop_event.is_set():
            current_time = time.time()
            
            time_elapsed = current_time - last_frame_time
            if time_elapsed < CAMERA_MIN_FRAME_INTERVAL:
                time.sleep(max(CAMERA_MIN_FRAME_INTERVAL - time_elapsed - 0.001, 0))
                continue
            
            frame = camera.get_frame()
            frames += 1
            
            set_camera_frame(frame)
            frames_processed += 1
            last_frame_time = current_time
            
            if current_time > time_start + 1:
                camera_logger.debug(f"Camera FPS: {frames} (processed: {frames_processed})")
                frames = 0
                frames_processed = 0
                time_start = current_time
    except KeyboardInterrupt:
        pass
    except Exception as e:
        camera_logger.error(f"{e}")

def set_camera_frame(frame: camera.FrameData | None):
    if frame is None:
        shared_data['frame_ready'].value = False
        return
    
    frame_flat = frame.frame.flatten()
    if len(frame_flat) <= FRAME_SIZE_B:
        np_array = np.ndarray(frame_flat.shape, dtype=np.uint8, buffer=shared_data['frame_buffer'].buf)
        np_array[:] = frame_flat[:]
        shared_data['frame_timestamp'].value = frame.timestamp
        shared_data['frame_ready'].value = True

def get_camera_frame() -> camera.FrameData | None:
    if not shared_data['frame_ready'].value:
        return None
    
    try:
        np_array = np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8, buffer=shared_data['frame_buffer'].buf)
        frame_copy = np.copy(np_array)
        return camera.FrameData(
            frame=frame_copy,
            timestamp=shared_data['frame_timestamp'].value
        )
    except Exception:
        return None

#endregion
#region Logic

logic_logger = setup_logger(
    "Logic Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)

def logic_process(stop_event):
    try:
        move_x = 0.0
        move_y = 0.0
        rotate = 0.0
        acceleration = 1.0 / 0.15
        
        target_heading = None
        rotation_correction_gain = 1.3
        rotation_deadzone = 15 # degrees

        while not stop_event.is_set():
            start_time = time.time()

            mode = get_robot_mode()
            if mode == RobotMode.IDLE:
                set_motor_speeds([0, 0, 0, 0])
                time.sleep(IDLE_SLEEP_DURATION)
                target_heading = None
                continue
            elif mode == RobotMode.MANUAL:
                control = get_manual_control()
                hardware_data = get_hardware_data()
                
                move_x += (control.move_speed * math.cos(math.radians(control.move_angle)) - move_x) * acceleration * LOGIC_LOOP_PERIOD
                move_y += (control.move_speed * math.sin(math.radians(control.move_angle)) - move_y) * acceleration * LOGIC_LOOP_PERIOD
                rotate += (control.rotate - rotate) * acceleration * LOGIC_LOOP_PERIOD
                
                if abs(rotate) > 0.01:
                    target_heading = None
                elif target_heading is None and hardware_data:
                    target_heading = hardware_data.compass.heading
                
                rotation_correction = 0.0
                if hardware_data and target_heading is not None:
                    current_heading = hardware_data.compass.heading
                    heading_error = (target_heading - current_heading + 180) % 360 - 180
                    if abs(heading_error) > rotation_deadzone:
                        rotation_correction = (heading_error / 180.0) * rotation_correction_gain
                
                total_rotation = rotate * 0.4 + rotation_correction
                total_rotation = max(-1.0, min(1.0, total_rotation))

                angle = math.atan2(move_y, move_x)
                speed = math.sqrt(move_x**2 + move_y**2)
                motors = calculate_motors_speeds(angle, speed * 255, total_rotation * 255)
                
                set_motor_speeds(motors)
            elif mode == RobotMode.AUTONOMOUS:
                pass

            elapsed = time.time() - start_time
            sleep_duration = max(0.0, LOGIC_LOOP_PERIOD - elapsed - 0.001)
            if sleep_duration > 0:
                time.sleep(sleep_duration)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logic_logger.error(f"{e}")

def set_motor_speeds(speeds: list[int]):
    for i in range(min(4, len(speeds))):
        shared_data['motor_speeds'][i] = speeds[i]

def get_motor_speeds() -> list[int]:
    return list(shared_data['motor_speeds'])

def set_kicker_state(state: bool):
    shared_data['kicker_state'].value = state

def get_kicker_state() -> bool:
    return shared_data['kicker_state'].value

def set_robot_mode(mode: str):
    with shared_data['locks']['robot_mode']:
        encoded = mode.encode()[:20]
        for i in range(len(encoded)):
            shared_data['robot_mode'][i] = encoded[i]
        if len(encoded) < 20:
            shared_data['robot_mode'][len(encoded)] = 0

def get_robot_mode() -> str:
    with shared_data['locks']['robot_mode']:
        parts_int = []
        parts_bytes = []
        for c in shared_data['robot_mode']:
            if c == 0 or c == b'\x00':
                break
            if isinstance(c, int):
                parts_int.append(c)
            else:
                parts_bytes.append(c)
        if parts_bytes and not parts_int:
            return b''.join(parts_bytes).decode()
        else:
            return bytes(parts_int).decode()

def set_manual_control(control: RobotManualControl) -> None:
    shared_data['manual_control'][0] = control.move_angle
    shared_data['manual_control'][1] = control.move_speed
    shared_data['manual_control'][2] = control.rotate

def get_manual_control() -> RobotManualControl:
    return RobotManualControl(
        move_angle=shared_data['manual_control'][0],
        move_speed=shared_data['manual_control'][1],
        rotate=shared_data['manual_control'][2]
    )

#endregion
#region Hardware

hardware_logger = setup_logger(
    "Hardware Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)

def hardware_communication_process(stop_event):
    try:
        messages_received = 0
        messages_sent = 0
        time_start = time.time()

        compass_offset: dict[str, int] = {
            "heading": 0,
            "pitch": 0,
            "roll": 0
        }

        with teensy_communication.TeensyCommunicator(port=TEENSY_PORT, baud=TEENSY_BAUD, timeout=TEENSY_TIMEOUT) as communicator:
            while not stop_event.is_set():
                line = communicator.read_line()
                data = None
                if line:
                    messages_received += 1

                    line_type = teensy_communication.get_line_type(line)

                    if line_type == teensy_communication.LineType.SENSOR_DATA:
                        data = teensy_communication.parse_sensor_data_line(line)
                    elif line_type == teensy_communication.LineType.RUNNING_STATE:
                        state = teensy_communication.parse_running_state_line(line)
                        set_running_state(state)
                    
                    communicator.send_motors_message(get_motor_speeds(), get_kicker_state())
                    messages_sent += 1
                
                if data:
                    data.compass.heading -= compass_offset["heading"]
                    data.compass.pitch -= compass_offset["pitch"]
                    data.compass.roll -= compass_offset["roll"]
                    
                    update_line_calibration(data)
                    update_line_detected(data)

                    set_hardware_data(data)
                
                if check_and_clear_compass_reset():
                    if not data:
                        data = get_hardware_data()
                    if data:
                        hardware_logger.info("Resetting compass position")
                        compass_offset["heading"] += data.compass.heading
                        compass_offset["pitch"] += data.compass.pitch
                        compass_offset["roll"] += data.compass.roll
                
                if time.time() > time_start + 1:
                    hardware_logger.debug(f"Messages received per second: {messages_received}")
                    hardware_logger.debug(f"Messages sent per second: {messages_sent}")
                    messages_received = 0
                    messages_sent = 0
                    time_start = time.time()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        hardware_logger.error(f"{e}")


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


def request_compass_reset() -> None:
    shared_data['compass_reset'].value = True

def check_and_clear_compass_reset() -> bool:
    if shared_data['compass_reset'].value:
        shared_data['compass_reset'].value = False
        return True
    return False

def get_line_detection_thresholds() -> list[int]:
    with shared_data['locks']['line_calibration']:
        return list(shared_data['line_detection_thresholds'])

def set_line_detection_thresholds(thresholds: list[int], save: bool = True) -> None:
    with shared_data['locks']['line_calibration']:
        for i in range(min(LINE_SENSOR_COUNT, len(thresholds))):
            shared_data['line_detection_thresholds'][i] = thresholds[i]
    if save:
        save_calibration_data()

def get_line_detected() -> list[bool]:
    with shared_data['locks']['line_calibration']:
        return list(shared_data['line_detected'])

def get_line_detection_dark() -> bool:
    with shared_data['locks']['line_calibration']:
        return bool(shared_data['line_detection_dark'].value)

def set_line_detection_dark(is_dark_line: bool, save: bool = True) -> None:
    with shared_data['locks']['line_calibration']:
        shared_data['line_detection_dark'].value = bool(is_dark_line)
    if save:
        save_calibration_data()

def update_line_detected(data: teensy_communication.ParsedTeensyData):
    with shared_data['locks']['line_calibration']:
        detect_dark = bool(shared_data['line_detection_dark'].value)
        for i, value in enumerate(data.line):
            if i < LINE_SENSOR_COUNT:
                threshold = shared_data['line_detection_thresholds'][i]
                detected = value < threshold if detect_dark else value > threshold
                shared_data['line_detected'][i] = detected

def start_line_calibration() -> None:
    with shared_data['locks']['line_calibration']:
        shared_data['line_calibration_active'].value = True
        for i in range(LINE_SENSOR_COUNT):
            shared_data['line_calibration_min'][i] = float('inf')
            shared_data['line_calibration_max'][i] = float('-inf')

def stop_line_calibration(cancel: bool = False) -> tuple[list[int], list[float], list[float]]:
    with shared_data['locks']['line_calibration']:
        shared_data['line_calibration_active'].value = False
        
        if cancel:
            return list(shared_data['line_detection_thresholds']), list(shared_data['line_calibration_min']), list(shared_data['line_calibration_max'])

        thresholds = []
        min_values = list(shared_data['line_calibration_min'])
        max_values = list(shared_data['line_calibration_max'])
        
        for i in range(LINE_SENSOR_COUNT):
            if min_values[i] != float('inf') and max_values[i] != float('-inf'):
                threshold = int((min_values[i] + max_values[i]) / 2)
                shared_data['line_detection_thresholds'][i] = threshold
                thresholds.append(threshold)
            else:
                thresholds.append(shared_data['line_detection_thresholds'][i])

    save_calibration_data()
    return thresholds, min_values, max_values

def get_line_calibration_status() -> dict:
    with shared_data['locks']['line_calibration']:
        return {
            'active': shared_data['line_calibration_active'].value,
            'current_thresholds': list(shared_data['line_detection_thresholds']),
            'dark_line': bool(shared_data['line_detection_dark'].value),
            'calibration_min': list(shared_data['line_calibration_min']) if shared_data['line_calibration_active'].value else None,
            'calibration_max': list(shared_data['line_calibration_max']) if shared_data['line_calibration_active'].value else None,
        }

def update_line_calibration(data: teensy_communication.ParsedTeensyData):
    with shared_data['locks']['line_calibration']:
        if shared_data['line_calibration_active'].value:
            for i, value in enumerate(data.line):
                if i < LINE_SENSOR_COUNT:
                    shared_data['line_calibration_min'][i] = min(shared_data['line_calibration_min'][i], value)
                    shared_data['line_calibration_max'][i] = max(shared_data['line_calibration_max'][i], value)

CALIBRATION_SCHEMA_VERSION = 1

def _get_calibration_storage_path() -> str:
    if os.path.isabs(CALIBRATION_FILE_PATH):
        return CALIBRATION_FILE_PATH
    return os.path.join(os.path.dirname(__file__), CALIBRATION_FILE_PATH)

def _create_calibration_data() -> dict:
    with shared_data['locks']['line_calibration']:
        return {
            "version": CALIBRATION_SCHEMA_VERSION,
            "calibrations": {
                "line_detection": {
                    "thresholds": list(shared_data['line_detection_thresholds']),
                    "dark_line": bool(shared_data['line_detection_dark'].value),
                }
            }
        }

def save_calibration_data() -> None:
    path = _get_calibration_storage_path()
    data = _create_calibration_data()

    with shared_data['locks']['calibration_storage']:
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            temp_path = f"{path}.tmp"
            with open(temp_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(temp_path, path)
        except Exception as e:
            hardware_logger.error(f"Failed to save calibration data: {e}")

def load_calibration_data() -> None:
    path = _get_calibration_storage_path()
    if not os.path.exists(path):
        hardware_logger.info("No calibration data file found, using defaults")
        return

    with shared_data['locks']['calibration_storage']:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            hardware_logger.warning(f"Failed to read calibration data: {e}")
            return

    try:
        calibrations = data.get("calibrations", {}) if isinstance(data, dict) else {}
        line_data = calibrations.get("line_detection", {}) if isinstance(calibrations, dict) else {}
        thresholds = line_data.get("thresholds", None)
        dark_line = line_data.get("dark_line", None)

        if isinstance(thresholds, list) and len(thresholds) == LINE_SENSOR_COUNT:
            set_line_detection_thresholds([int(t) for t in thresholds], save=False)
        if isinstance(dark_line, bool):
            set_line_detection_dark(dark_line, save=False)

        hardware_logger.info("Calibration data loaded")
    except Exception as e:
        hardware_logger.warning(f"Invalid calibration data format: {e}")

def set_running_state(state: teensy_communication.RunningStateData | None):
    with shared_data['locks']['running_state']:
        shared_data['running_state'].clear()
        if state:
            shared_data['running_state']['running'] = state.running
            shared_data['running_state']['bt_module_enabled'] = state.bt_module_enabled
            shared_data['running_state']['bt_module_state'] = state.bt_module_state
            shared_data['running_state']['switch_state'] = state.switch_state

def get_running_state() -> teensy_communication.RunningStateData | None:
    with shared_data['locks']['running_state']:
        if not shared_data['running_state']:
            return None
        return teensy_communication.RunningStateData(
            running=shared_data['running_state'].get('running', False),
            bt_module_enabled=shared_data['running_state'].get('bt_module_enabled', False),
            bt_module_state=shared_data['running_state'].get('bt_module_state', False),
            switch_state=shared_data['running_state'].get('switch_state', False),
        )

#endregion
#region API

api_logger = setup_logger(
    "API Process", level=LOGGING_LEVEL, 
    logs_dict=shared_data['logs_buffer'], 
    logs_lock=shared_data['locks']['logs_buffer'], 
    next_log_id=shared_data['next_log_id']
)

def api_process():
    try:
        init_api()

        api.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        api_logger.error(f"{e}")

def init_api():
    api.logger = api_logger
    api.camera_frame_getter = api_get_camera_frame
    api.hardware_data_getter = get_hardware_data
    api.motor_speeds_getter = get_motor_speeds
    api.kicker_state_getter = get_kicker_state
    api.robot_mode_getter = get_robot_mode
    api.robot_mode_setter = set_robot_mode
    api.manual_control_getter = get_manual_control
    api.manual_control_setter = set_manual_control
    api.compass_reset_requester = request_compass_reset
    api.logs_getter = get_logs
    api.line_detection_thresholds_getter = get_line_detection_thresholds
    api.line_detection_thresholds_setter = set_line_detection_thresholds
    api.line_detected_getter = get_line_detected
    api.line_calibration_starter = start_line_calibration
    api.line_calibration_stopper = stop_line_calibration
    api.line_calibration_status_getter = get_line_calibration_status
    api.line_detection_dark_getter = get_line_detection_dark
    api.line_detection_dark_setter = set_line_detection_dark
    api.running_state_getter = get_running_state

def api_get_camera_frame():
    data = get_camera_frame()
    if data is None:
        return None
    return data.frame

#endregion


def main():
    load_calibration_data()

    for logger_name in ('werkzeug',):
        l = logging.getLogger(logger_name)
        l.setLevel(logging.INFO)
        l.addFilter(Suppress200Filter())
        
        buffer_handler = BufferedLogHandler(
            shared_data['logs_buffer'],
            shared_data['locks']['logs_buffer'],
            shared_data['next_log_id']
        )
        buffer_handler.setLevel(logging.INFO)
        l.addHandler(buffer_handler)
    
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
    
    try:
        shared_data['frame_buffer'].close()
        shared_data['frame_buffer'].unlink()
    except Exception as e:
        logger.error(f"Error closing/unlinking frame buffer: {e}")


if __name__ == "__main__":
    main()
