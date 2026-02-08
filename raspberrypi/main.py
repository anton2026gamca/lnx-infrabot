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
    TEENSY_PORT, TEENSY_BAUD, TEENSY_TIMEOUT, COMMUNICATION_LOOP_PERIOD,
    LINE_SENSOR_COUNT, LINE_SENSOR_LOCATIONS, LINE_DETECTION_THRESHOLDS,
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
    'line_detection_thresholds': multiprocessing.Array('i', [val for pair in LINE_DETECTION_THRESHOLDS for val in pair]),
    'line_detected': multiprocessing.Array('b', [False] * LINE_SENSOR_COUNT),
    'line_calibration_min': multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT),
    'line_calibration_max': multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT),
    'line_calibration_phase': multiprocessing.Value('i', 0),  # 0=inactive, 1=field, 2=lines
    'line_calibration_phase1_min': multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT),
    'line_calibration_phase1_max': multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT),
    'line_calibration_phase2_min': multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT),
    'line_calibration_phase2_max': multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT),
    'running_state': manager.dict(),
    'rotation_correction_enabled': multiprocessing.Value('b', True),
    'line_avoiding_enabled': multiprocessing.Value('b', True),
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
        motors_controller = SmartMotorsController()

        while not stop_event.is_set():
            start_time = time.time()

            mode = get_robot_mode()
            if mode == RobotMode.IDLE:
                motors_controller.reset()
                time.sleep(IDLE_SLEEP_DURATION)
                continue
            elif mode == RobotMode.MANUAL:
                control = get_manual_control()
                motors_controller.set_motors(control.move_angle, control.move_speed, control.rotate)
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
    
class MotorsController:
    def __init__(self):
        self.move_x = 0.0
        self.move_y = 0.0
        self.rotate = 0.0
        self.acceleration = 1.0 / 0.15

    def set_motors(self, angle: float, speed: float, rotate: float):
        self.move_x += (speed * math.cos(math.radians(angle)) - self.move_x) * self.acceleration * LOGIC_LOOP_PERIOD
        self.move_y += (speed * math.sin(math.radians(angle)) - self.move_y) * self.acceleration * LOGIC_LOOP_PERIOD
        self.rotate += (rotate - self.rotate) * self.acceleration * LOGIC_LOOP_PERIOD

        new_angle = math.atan2(self.move_y, self.move_x)
        speed = math.sqrt(self.move_x**2 + self.move_y**2)

        motors = calculate_motors_speeds(new_angle, speed * 255, self.rotate * 255)
        set_motor_speeds(motors)
    
    def reset(self):
        self.move_x = 0.0
        self.move_y = 0.0
        self.rotate = 0.0
        set_motor_speeds([0, 0, 0, 0])

class SmartMotorsController(MotorsController):
    def __init__(self):
        super().__init__()
        self.target_heading = None
        self.rotation_deadzone = 15.0
        self.rotation_correction_gain = 1.0

    def set_motors(self, angle: float, speed: float, rotate: float):
        self.move_x += (speed * math.cos(math.radians(angle)) - self.move_x) * self.acceleration * LOGIC_LOOP_PERIOD
        self.move_y += (speed * math.sin(math.radians(angle)) - self.move_y) * self.acceleration * LOGIC_LOOP_PERIOD
        self.rotate += (rotate - self.rotate) * self.acceleration * LOGIC_LOOP_PERIOD
        move_angle = math.atan2(self.move_y, self.move_x)
        move_speed = math.sqrt(self.move_x**2 + self.move_y**2)

        hardware_data = get_hardware_data()
        current_heading = hardware_data.compass.heading if hardware_data else None

        rotation_correction_enabled = get_rotation_correction_enabled()
        rotation_correction = 0.0
        if rotation_correction_enabled:
            if abs(self.rotate) > 0.01:
                self.target_heading = None
            elif self.target_heading is None and current_heading is not None:
                self.target_heading = current_heading
            if current_heading is not None and self.target_heading is not None:
                heading_error = (self.target_heading - current_heading + 180) % 360 - 180
                if abs(heading_error) > self.rotation_deadzone:
                    rotation_correction = (heading_error / 180.0) * self.rotation_correction_gain

        total_rotation = self.rotate * 0.4 + rotation_correction
        total_rotation = max(-1.0, min(1.0, total_rotation))

        line_avoiding_enabled = get_line_avoiding_enabled()
        if line_avoiding_enabled:
            lines_detected = get_line_detected()
            sum = 0
            count = 0
            for i, line in enumerate(lines_detected):
                if line:
                    sum += LINE_SENSOR_LOCATIONS[i]
                    count += 1
            if count > 0:
                avg = sum / count
                move_angle = min(max(move_angle - avg, 135), 225) + avg

        motors = calculate_motors_speeds(move_angle, move_speed * 255, total_rotation * 255)
        set_motor_speeds(motors)
    
    def reset(self):
        super().reset()
        self.target_heading = None


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

def set_rotation_correction_enabled(enabled: bool) -> None:
    shared_data['rotation_correction_enabled'].value = enabled

def get_rotation_correction_enabled() -> bool:
    return shared_data['rotation_correction_enabled'].value

def set_line_avoiding_enabled(enabled: bool) -> None:
    shared_data['line_avoiding_enabled'].value = enabled

def get_line_avoiding_enabled() -> bool:
    return shared_data['line_avoiding_enabled'].value

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
        last_log_time = time.time()

        compass_offset: dict[str, int] = {
            "heading": 0,
            "pitch": 0,
            "roll": 0
        }

        with teensy_communication.TeensyCommunicator(port=TEENSY_PORT, baud=TEENSY_BAUD, timeout=TEENSY_TIMEOUT) as communicator:
            while not stop_event.is_set():
                start_time = time.time()

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
                
                if time.time() > last_log_time + 1:
                    hardware_logger.debug(f"Messages received per second: {messages_received}")
                    hardware_logger.debug(f"Messages sent per second: {messages_sent}")
                    messages_received = 0
                    messages_sent = 0
                    last_log_time = time.time()
                
                time_elapsed = time.time() - start_time
                if time_elapsed < COMMUNICATION_LOOP_PERIOD:
                    time.sleep(max(0.0, COMMUNICATION_LOOP_PERIOD - time_elapsed - 0.001))
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

def get_line_detection_thresholds() -> list[list[int]]:
    with shared_data['locks']['line_calibration']:
        flat = list(shared_data['line_detection_thresholds'])
        return [[flat[i*2], flat[i*2+1]] for i in range(LINE_SENSOR_COUNT)]

def set_line_detection_thresholds(thresholds: list[list[int]], save: bool = True) -> None:
    with shared_data['locks']['line_calibration']:
        for i in range(min(LINE_SENSOR_COUNT, len(thresholds))):
            shared_data['line_detection_thresholds'][i * 2] = thresholds[i][0]  # min
            shared_data['line_detection_thresholds'][i * 2 + 1] = thresholds[i][1]  # max
    if save:
        save_calibration_data()

def get_line_detected() -> list[bool]:
    with shared_data['locks']['line_calibration']:
        return list(shared_data['line_detected'])

def update_line_detected(data: teensy_communication.ParsedTeensyData):
    with shared_data['locks']['line_calibration']:
        for i, value in enumerate(data.line):
            if i >= LINE_SENSOR_COUNT:
                break
            threshold_min = shared_data['line_detection_thresholds'][i * 2]
            threshold_max = shared_data['line_detection_thresholds'][i * 2 + 1]
            shared_data['line_detected'][i] = value < threshold_min or value > threshold_max

def start_line_calibration(phase: int = 1) -> None:
    with shared_data['locks']['line_calibration']:
        if phase == 1:
            shared_data['line_calibration_phase'].value = 1
            for i in range(LINE_SENSOR_COUNT):
                shared_data['line_calibration_min'][i] = float('inf')
                shared_data['line_calibration_max'][i] = float('-inf')
                shared_data['line_calibration_phase1_min'][i] = float('inf')
                shared_data['line_calibration_phase1_max'][i] = float('-inf')
                shared_data['line_calibration_phase2_min'][i] = float('inf')
                shared_data['line_calibration_phase2_max'][i] = float('-inf')
        elif phase == 2:
            for i in range(LINE_SENSOR_COUNT):
                shared_data['line_calibration_phase1_min'][i] = shared_data['line_calibration_min'][i]
                shared_data['line_calibration_phase1_max'][i] = shared_data['line_calibration_max'][i]
                shared_data['line_calibration_min'][i] = float('inf')
                shared_data['line_calibration_max'][i] = float('-inf')
            shared_data['line_calibration_phase'].value = 2

def stop_line_calibration(cancel: bool = False) -> tuple[list[list[int]], list[float], list[float], int]:
    with shared_data['locks']['line_calibration']:
        phase = shared_data['line_calibration_phase'].value
        
        thresholds = []
        min_values = list(shared_data['line_calibration_min'])
        max_values = list(shared_data['line_calibration_max'])
        
        if phase == 1:
            for i in range(LINE_SENSOR_COUNT):
                if min_values[i] != float('inf') and max_values[i] != float('-inf'):
                    thresholds.append([int(min_values[i]), int(max_values[i])])
                else:
                    existing_min = shared_data['line_detection_thresholds'][i * 2]
                    existing_max = shared_data['line_detection_thresholds'][i * 2 + 1]
                    thresholds.append([existing_min, existing_max])
            shared_data['line_calibration_phase'].value = 0
        elif phase == 2:
            for i in range(LINE_SENSOR_COUNT):
                shared_data['line_calibration_phase2_min'][i] = shared_data['line_calibration_min'][i]
                shared_data['line_calibration_phase2_max'][i] = shared_data['line_calibration_max'][i]
            
            for i in range(LINE_SENSOR_COUNT):
                field_min = shared_data['line_calibration_phase1_min'][i]
                field_max = shared_data['line_calibration_phase1_max'][i]
                line_min = shared_data['line_calibration_phase2_min'][i]
                line_max = shared_data['line_calibration_phase2_max'][i]
                
                if field_min != float('inf') and field_max != float('-inf'):
                    if line_min != float('inf') and line_max != float('-inf'):
                        field_center = (field_min + field_max) / 2
                        field_range = field_max - field_min
                        margin = max(10, field_range * 0.1)
                        
                        new_min = max(0, int(field_min - margin))
                        new_max = int(field_max + margin)
                        
                        if line_min < field_center < line_max:
                            pass
                        elif line_max < field_min:
                            new_min = max(new_min, int(line_max + 5))
                        elif line_min > field_max:
                            new_max = min(new_max, int(line_min - 5))
                        
                        thresholds.append([new_min, new_max])
                    else:
                        thresholds.append([int(field_min), int(field_max)])
                else:
                    existing_min = shared_data['line_detection_thresholds'][i * 2]
                    existing_max = shared_data['line_detection_thresholds'][i * 2 + 1]
                    thresholds.append([existing_min, existing_max])
            
            shared_data['line_calibration_phase'].value = 0

    if cancel:
        shared_data['line_calibration_phase'].value = 0
    elif phase > 0:
        for i in range(LINE_SENSOR_COUNT):
            shared_data['line_detection_thresholds'][i * 2] = thresholds[i][0]
            shared_data['line_detection_thresholds'][i * 2 + 1] = thresholds[i][1]
        save_calibration_data()
    
    return thresholds, min_values, max_values, phase

def get_line_calibration_status() -> dict:
    with shared_data['locks']['line_calibration']:
        flat = list(shared_data['line_detection_thresholds'])
        current_thresholds = [[flat[i*2], flat[i*2+1]] for i in range(LINE_SENSOR_COUNT)]
        phase = shared_data['line_calibration_phase'].value
        
        def sanitize_values(values):
            if values is None:
                return None
            return [None if (v == float('inf') or v == float('-inf')) else v for v in values]
        
        calibration_min = list(shared_data['line_calibration_min']) if phase > 0 else None
        calibration_max = list(shared_data['line_calibration_max']) if phase > 0 else None
        
        return {
            'active': phase > 0,
            'phase': phase,
            'current_thresholds': current_thresholds,
            'calibration_min': sanitize_values(calibration_min),
            'calibration_max': sanitize_values(calibration_max),
            'phase1_complete': list(shared_data['line_calibration_phase1_min'])[0] != float('inf'),
            'phase1_min': sanitize_values(list(shared_data['line_calibration_phase1_min'])) if list(shared_data['line_calibration_phase1_min'])[0] != float('inf') else None,
            'phase1_max': sanitize_values(list(shared_data['line_calibration_phase1_max'])) if list(shared_data['line_calibration_phase1_max'])[0] != float('inf') else None,
        }

def update_line_calibration(data: teensy_communication.ParsedTeensyData):
    with shared_data['locks']['line_calibration']:
        if shared_data['line_calibration_phase'].value > 0:
            for i, value in enumerate(data.line):
                if i < LINE_SENSOR_COUNT:
                    shared_data['line_calibration_min'][i] = min(shared_data['line_calibration_min'][i], value)
                    shared_data['line_calibration_max'][i] = max(shared_data['line_calibration_max'][i], value)

CALIBRATION_SCHEMA_VERSION = 2

def _get_calibration_storage_path() -> str:
    if os.path.isabs(CALIBRATION_FILE_PATH):
        return CALIBRATION_FILE_PATH
    return os.path.join(os.path.dirname(__file__), CALIBRATION_FILE_PATH)

def _create_calibration_data() -> dict:
    with shared_data['locks']['line_calibration']:
        flat = list(shared_data['line_detection_thresholds'])
        thresholds = [[flat[i*2], flat[i*2+1]] for i in range(LINE_SENSOR_COUNT)]
        return {
            "version": CALIBRATION_SCHEMA_VERSION,
            "calibrations": {
                "line_detection": {
                    "thresholds": thresholds,
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

        if isinstance(thresholds, list) and len(thresholds) == LINE_SENSOR_COUNT:
            if isinstance(thresholds[0], list):
                set_line_detection_thresholds([[int(t[0]), int(t[1])] for t in thresholds], save=False)
            else:
                set_line_detection_thresholds([[int(t), int(t)] for t in thresholds], save=False)

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
    api.running_state_getter = get_running_state
    api.rotation_correction_enabled_getter = get_rotation_correction_enabled
    api.rotation_correction_enabled_setter = set_rotation_correction_enabled
    api.line_avoiding_enabled_getter = get_line_avoiding_enabled
    api.line_avoiding_enabled_setter = set_line_avoiding_enabled

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
