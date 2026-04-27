import multiprocessing
import multiprocessing.shared_memory
import time
import numpy as np
from robot.config import *


_manager = multiprocessing.Manager()

# define logging stuff before we import any robot stuff
logs_buffer = multiprocessing.Queue(maxsize=LOG_BUFFER_MAX_ENTRIES)


from robot import utils
from robot.config import *
from robot.hardware.teensy import ParsedTeensyData, IRData, CompassData, RunningStateData
from robot.robot import RobotManualControl
from robot.vision import CameraBallData, DetectedObject, GoalDetectionResult
from robot.vision.camera import FrameData



_logger = utils.get_logger("Shared Data Manager")


# Running state
running_state = _manager.dict()
running_state_lock = multiprocessing.Lock()
def set_running_state(state: RunningStateData | None):
    with running_state_lock:
        running_state.clear()
        if state:
            running_state['running'] = state.running
            running_state['bt_module_enabled'] = state.bt_module_enabled
            running_state['bt_module_state'] = state.bt_module_state
            running_state['switch_state'] = state.switch_state
def get_running_state() -> RunningStateData | None:
    with running_state_lock:
        if not running_state:
            return None
        return RunningStateData(
            running=running_state.get('running', False),
            bt_module_enabled=running_state.get('bt_module_enabled', False),
            bt_module_state=running_state.get('bt_module_state', False),
            switch_state=running_state.get('switch_state', False),
        )


# Hardware data
hardware_data = _manager.dict()
hardware_data_lock = multiprocessing.Lock()
def set_hardware_data(data: ParsedTeensyData | None):
    with hardware_data_lock:
        dict_data = {
            'compass': {
                'heading': data.compass.heading if data else None,
                'pitch': data.compass.pitch if data else None,
                'roll': data.compass.roll if data else None,
            },
            'ir': {
                'angle': data.ir.angle if data else None,
                'distance': data.ir.distance if data else None,
                'sensors': data.ir.sensors if data else None,
                'status': data.ir.status if data else None,
            },
            'line_sensors': data.line if data else None,
            'raw': data.raw if data else None,
            'timestamp': data.timestamp if data else None,
        }
        hardware_data.clear()
        hardware_data.update(dict_data)

def get_hardware_data() -> ParsedTeensyData | None:
    with hardware_data_lock:
        if not hardware_data:
            return None
        data = ParsedTeensyData(
            compass=CompassData(
                heading=hardware_data['compass']['heading'],
                pitch=hardware_data['compass']['pitch'],
                roll=hardware_data['compass']['roll'],
            ),
            ir=IRData(
                angle=hardware_data['ir']['angle'],
                distance=hardware_data['ir']['distance'],
                sensors=hardware_data['ir']['sensors'],
                status=hardware_data['ir']['status'],
            ),
            line=hardware_data['line_sensors'],
            raw=hardware_data['raw'],
            timestamp=hardware_data['timestamp'],
        )
        return data


# Motor and kicker state
motor_speeds = multiprocessing.Array('i', [0, 0, 0, 0])
def set_motor_speeds(speeds: list[int]):
    with motor_speeds.get_lock():
        for i in range(min(4, len(speeds))):
            motor_speeds[i] = speeds[i]
def get_motor_speeds() -> list[int]:
    with motor_speeds.get_lock():
        return list(motor_speeds[:])

kicker_state = multiprocessing.Value('b', False)
def set_kicker_state(state: bool):
    kicker_state.value = state
def get_kicker_state() -> bool:
    return kicker_state.value


# Compass
compass_reset = multiprocessing.Value('b', False)
def request_compass_reset() -> None:
    compass_reset.value = True
def check_and_clear_compass_reset() -> bool:
    if compass_reset.value:
        compass_reset.value = False
        return True
    return False


# Robot mode
robot_mode = multiprocessing.Value('i', 0)
def set_robot_mode(mode: int):
    robot_mode.value = mode
def get_robot_mode() -> int:
    return robot_mode.value


# Manual control
manual_control = multiprocessing.Array('d', [0.0, 0.0, 0.0])
def set_manual_control(control: RobotManualControl) -> None:
    with manual_control.get_lock():
        manual_control[0] = control.move_angle
        manual_control[1] = control.move_speed
        manual_control[2] = control.rotate
def get_manual_control() -> RobotManualControl:
    with manual_control.get_lock():
        return RobotManualControl(
            move_angle=manual_control[0],
            move_speed=manual_control[1],
            rotate=manual_control[2]
        )


# Autonomous state
state_machine_change_request = multiprocessing.Array('c', b''.ljust(50))
def request_state_machine_change(name: str) -> None:
    name_bytes = name.encode()[:50].ljust(50)
    with state_machine_change_request.get_lock():
        for i in range(50):
            state_machine_change_request[i] = name_bytes[i:i+1]
def check_state_machine_change_request() -> str:
    with state_machine_change_request.get_lock():
        req = bytes(state_machine_change_request[:]).decode().strip()
        state_machine_change_request[:] = b''.ljust(50)
        return req

current_state_machine_name = multiprocessing.Array('c', b''.ljust(50))
def set_current_state_machine_name(name: str) -> None:
    name_bytes = name.encode()[:50].ljust(50)
    with current_state_machine_name.get_lock():
        for i in range(50):
            current_state_machine_name[i] = name_bytes[i:i+1]
def get_current_state_machine_name() -> str:
    with current_state_machine_name.get_lock():
        return bytes(current_state_machine_name[:]).decode().strip()


# Shared memory for frame data
frame_buffer = multiprocessing.shared_memory.SharedMemory(create=True, size=FRAME_SIZE_B)
frame_timestamp = multiprocessing.Value('d', 0.0)
frame_ready = multiprocessing.Value('b', False)
frame_lock = multiprocessing.Lock()
def set_camera_frame(frame: FrameData | None):
    if frame is None:
        frame_ready.value = False
        return
    
    frame_flat = frame.frame.flatten()
    if len(frame_flat) <= FRAME_SIZE_B:
        np_array = np.ndarray(frame_flat.shape, dtype=np.uint8, buffer=frame_buffer.buf)
        np_array[:] = frame_flat[:]
        frame_timestamp.value = frame.timestamp
        frame_ready.value = True

def get_camera_frame() -> FrameData | None:
    if not frame_ready.value:
        return None
    
    try:
        np_array = np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8, buffer=frame_buffer.buf)
        frame_copy = np.copy(np_array)
        return FrameData(
            frame=frame_copy,
            timestamp=frame_timestamp.value
        )
    except Exception:
        return None


# Detected objects by camera
detected_objects = _manager.list()
detected_objects_lock = multiprocessing.Lock()
def get_detected_objects_raw() -> list[dict]:
    with detected_objects_lock:
        return detected_objects[:]
def get_detected_objects() -> list[DetectedObject]:
    detections_list = get_detected_objects_raw()
    result = []
    for det_dict in detections_list:
        obj = DetectedObject(
            object_type=det_dict['object_type'],
            x=det_dict['x'],
            y=det_dict['y'],
            width=det_dict['width'],
            height=det_dict['height'],
            confidence=det_dict['confidence'],
            color=tuple(det_dict.get('color', (255, 255, 255)))
        )
        result.append(obj)
    return result
def set_detected_objects(detections: list[DetectedObject]) -> None:
    with detected_objects_lock:
        detected_objects[:] = []
        for det in detections:
            det_dict = {
                'object_type': det.object_type,
                'x': det.x,
                'y': det.y,
                'width': det.width,
                'height': det.height,
                'confidence': det.confidence,
                'color': det.color
            }
            detected_objects.append(det_dict)


# Line detection and calibration
line_calibration_lock = multiprocessing.Lock()
line_detection_thresholds = multiprocessing.Array('i', [val for pair in DEFAULT_LINE_DETECTION_THRESHOLDS for val in pair])
def get_line_detection_thresholds() -> list[list[int]]:
    with line_calibration_lock:
        flat = line_detection_thresholds[:]
        return [[flat[i*2], flat[i*2+1]] for i in range(LINE_SENSOR_COUNT)]
def set_line_detection_thresholds(thresholds: list[list[int]]) -> None:
    with line_calibration_lock:
        for i in range(min(LINE_SENSOR_COUNT, len(thresholds))):
            line_detection_thresholds[i * 2] = thresholds[i][0]  # min
            line_detection_thresholds[i * 2 + 1] = thresholds[i][1]  # max

line_detected_lock = multiprocessing.Lock()
line_detected = multiprocessing.Array('b', [False] * LINE_SENSOR_COUNT)
line_calibration_min = multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT)
line_calibration_max = multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT)
line_calibration_phase = multiprocessing.Value('i', 0)  # 0=inactive, 1=field, 2=lines
line_calibration_phase1_min = multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT)
line_calibration_phase1_max = multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT)
line_calibration_phase2_min = multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT)
line_calibration_phase2_max = multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT)


# Goal detection and calibration
goal_detection_lock = multiprocessing.Lock()

goal_color = multiprocessing.Array('c', b'yellow'.ljust(10))
def set_goal_color(color: str) -> None:
    with goal_detection_lock:
        color_bytes = color.encode()[:10].ljust(10)
        for i in range(10):
            goal_color[i] = color_bytes[i:i+1]
def get_goal_color() -> str:
    with goal_detection_lock:
        return bytes(goal_color[:]).decode().strip()

goal_calibration_yellow = _manager.dict()
goal_calibration_blue = _manager.dict()

goal_calibration_yellow['count'] = 1
goal_calibration_yellow['range_0_lower'] = [20, 100, 100]
goal_calibration_yellow['range_0_upper'] = [30, 255, 255]

goal_calibration_blue['count'] = 1
goal_calibration_blue['range_0_lower'] = [100, 100, 100]
goal_calibration_blue['range_0_upper'] = [130, 255, 255]

def set_goal_calibration(color: str, ranges: list[tuple[tuple[int, int, int], tuple[int, int, int]]]) -> None:
    """Set goal calibration for a color with multiple HSV ranges.
    
    Args:
        color: 'yellow' or 'blue'
        ranges: List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    with goal_detection_lock:
        cal_dict = goal_calibration_yellow if color.lower() == 'yellow' else goal_calibration_blue
        cal_dict.clear()
        cal_dict['count'] = len(ranges)
        for idx, (lower, upper) in enumerate(ranges):
            cal_dict[f'range_{idx}_lower'] = [int(lower[0]), int(lower[1]), int(lower[2])]
            cal_dict[f'range_{idx}_upper'] = [int(upper[0]), int(upper[1]), int(upper[2])]

def get_goal_calibration(color: str) -> list[tuple[tuple[int, int, int], tuple[int, int, int]]]:
    """Get goal calibration ranges for a color.
    
    Returns:
        List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    with goal_detection_lock:
        cal_dict = goal_calibration_yellow if color.lower() == 'yellow' else goal_calibration_blue
        
        if not cal_dict or 'count' not in cal_dict:
            if color.lower() == 'yellow':
                return [((20, 100, 100), (30, 255, 255))]
            else:
                return [((100, 100, 100), (130, 255, 255))]
        
        count = cal_dict['count']
        ranges = []
        for idx in range(count):
            lower = cal_dict.get(f'range_{idx}_lower', [0, 0, 0])
            upper = cal_dict.get(f'range_{idx}_upper', [0, 0, 0])
            ranges.append((lower, upper))
        return ranges

goal_detection_result = _manager.dict()
def set_goal_detection_result(result: GoalDetectionResult | None) -> None:
    with goal_detection_lock:
        goal_detection_result.clear()
        if result:
            goal_detection_result['alignment'] = result.alignment
            goal_detection_result['goal_detected'] = result.detected
            goal_detection_result['goal_center_x'] = result.center_x
            goal_detection_result['goal_area'] = result.area
            goal_detection_result['distance_mm'] = result.distance_mm
            goal_detection_result['goal_height_pixels'] = result.height_pixels
def get_goal_detection_result() -> GoalDetectionResult | None:
    with goal_detection_lock:
        if not goal_detection_result:
            return None
        return GoalDetectionResult(
            alignment=goal_detection_result.get('alignment', 0.0),
            detected=goal_detection_result.get('goal_detected', False),
            center_x=goal_detection_result.get('goal_center_x', None),
            area=goal_detection_result.get('goal_area', 0.0),
            distance_mm=goal_detection_result.get('distance_mm', None),
            height_pixels=goal_detection_result.get('goal_height_pixels', 0.0)
        )

goal_focal_length = multiprocessing.Value('d', DEFAULT_FOCAL_LENGTH_PIXELS)
def get_goal_focal_length() -> float:
    return goal_focal_length.value
def set_goal_focal_length(focal_length: float) -> None:
    goal_focal_length.value = focal_length

goal_distance_calibration_active = multiprocessing.Value('b', False)
goal_distance_calibration_data = _manager.dict()
goal_distance_calibration_lock = multiprocessing.Lock()

# Ball detection and calibration
ball_calibration = _manager.dict()

# Initialize with default single range
ball_calibration['count'] = 1
ball_calibration['range_0_lower'] = [DEFAULT_BALL_CALIBRATION_HSV[0], DEFAULT_BALL_CALIBRATION_HSV[1], DEFAULT_BALL_CALIBRATION_HSV[2]]
ball_calibration['range_0_upper'] = [DEFAULT_BALL_CALIBRATION_HSV[3], DEFAULT_BALL_CALIBRATION_HSV[4], DEFAULT_BALL_CALIBRATION_HSV[5]]

def set_ball_calibration(ranges: list[tuple[list[int], list[int]]]) -> None:
    """Set ball calibration with multiple HSV ranges.
    
    Args:
        ranges: List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    with goal_detection_lock:
        ball_calibration.clear()
        ball_calibration['count'] = len(ranges)
        for idx, (lower, upper) in enumerate(ranges):
            ball_calibration[f'range_{idx}_lower'] = [int(lower[0]), int(lower[1]), int(lower[2])]
            ball_calibration[f'range_{idx}_upper'] = [int(upper[0]), int(upper[1]), int(upper[2])]

def get_ball_calibration() -> list[tuple[list[int], list[int]]]:
    """Get ball calibration ranges.
    
    Returns:
        List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    with goal_detection_lock:
        if not ball_calibration or 'count' not in ball_calibration:
            # Return default single range
            return [([DEFAULT_BALL_CALIBRATION_HSV[0], DEFAULT_BALL_CALIBRATION_HSV[1], DEFAULT_BALL_CALIBRATION_HSV[2]], 
                     [DEFAULT_BALL_CALIBRATION_HSV[3], DEFAULT_BALL_CALIBRATION_HSV[4], DEFAULT_BALL_CALIBRATION_HSV[5]])]
        
        count = ball_calibration['count']
        ranges = []
        for idx in range(count):
            lower = ball_calibration.get(f'range_{idx}_lower', [0, 0, 0])
            upper = ball_calibration.get(f'range_{idx}_upper', [0, 0, 0])
            ranges.append((lower, upper))
        return ranges

camera_ball_possession = multiprocessing.Value('b', False)
def set_camera_ball_possession(possessed: bool) -> None:
    camera_ball_possession.value = possessed
def get_camera_ball_possession() -> bool:
    return bool(camera_ball_possession.value)

camera_ball_position_lock = multiprocessing.Lock()
camera_ball_angle = multiprocessing.Value('d', 999.0)  # 999 = not detected
camera_ball_distance = multiprocessing.Value('d', 999.0)
camera_ball_detected = multiprocessing.Value('b', False)
camera_ball_area_pixels = multiprocessing.Value('d', 0.0)

def set_camera_ball_data(ball_data: CameraBallData) -> None:
    with camera_ball_position_lock:
        camera_ball_angle.value = ball_data.angle
        camera_ball_distance.value = ball_data.distance
        camera_ball_detected.value = ball_data.detected
        camera_ball_area_pixels.value = ball_data.area_pixels

def get_camera_ball_data() -> CameraBallData:
    with camera_ball_position_lock:
        return CameraBallData(
            angle=camera_ball_angle.value,
            distance=camera_ball_distance.value,
            detected=camera_ball_detected.value,
            area_pixels=camera_ball_area_pixels.value
        )


camera_ball_distance_calibration_constant = multiprocessing.Value('d', 10000.0)  # Default constant
def get_camera_ball_calibration_constant() -> float:
    return camera_ball_distance_calibration_constant.value
def set_camera_ball_calibration_constant(constant: float) -> None:
    camera_ball_distance_calibration_constant.value = float(constant)

camera_ball_usage_enabled = multiprocessing.Value('b', AUTO_CAMERA_BALL_TRACKING_ENABLED)
def set_camera_ball_usage_enabled(enabled: bool) -> None:
    camera_ball_usage_enabled.value = enabled
def get_camera_ball_usage_enabled() -> bool:
    return camera_ball_usage_enabled.value


# Feature toggles
rotation_correction_enabled = multiprocessing.Value('b', DEFAULT_ROTATION_CORRECTION_ENABLED)
def set_rotation_correction_enabled(enabled: bool) -> None:
    rotation_correction_enabled.value = enabled
def get_rotation_correction_enabled() -> bool:
    return rotation_correction_enabled.value

line_avoiding_enabled = multiprocessing.Value('b', DEFAULT_LINE_AVOIDING_ENABLED)
def set_line_avoiding_enabled(enabled: bool) -> None:
    line_avoiding_enabled.value = enabled

def get_line_avoiding_enabled() -> bool:
    return line_avoiding_enabled.value

position_based_speed_enabled = multiprocessing.Value('b', DEFAULT_POSITION_BASED_SPEED_ENABLED)
def set_position_based_speed_enabled(enabled: bool) -> None:
    position_based_speed_enabled.value = enabled

def get_position_based_speed_enabled() -> bool:
    return position_based_speed_enabled.value

always_facing_goal_enabled = multiprocessing.Value('b', True)  # True = always face goal, False = face north
def set_always_facing_goal_enabled(enabled: bool) -> None:
    always_facing_goal_enabled.value = enabled

def get_always_facing_goal_enabled() -> bool:
    return always_facing_goal_enabled.value



# Bluetooth Communication
bt_device_info = _manager.dict()  # Current device info: device_id, hostname, ip_address
bt_devices_info = _manager.list()  # Connected devices info
bt_paired_devices_info = _manager.list()  # All known paired devices
bt_received_messages = _manager.list()  # Received messages history
bt_sent_messages = _manager.list()  # Sent messages history
bt_commands = _manager.list()  # Commands queued for bluetooth process
bt_command_results = _manager.dict()  # command_id -> execution result
bt_next_command_id = multiprocessing.Value('i', 1)
bt_process_alive = multiprocessing.Value('b', False)
bt_other_robot_info = _manager.dict()  # Selected peer robot metadata (mac/name/...)
bt_lock = multiprocessing.Lock()


def set_bluetooth_process_alive(alive: bool) -> None:
    bt_process_alive.value = alive


def get_bluetooth_process_alive() -> bool:
    return bool(bt_process_alive.value)


def set_bluetooth_device_info(info: dict) -> None:
    """Set information about this robot's Bluetooth device"""
    with bt_lock:
        bt_device_info.clear()
        bt_device_info.update(info or {})


def set_bluetooth_other_robot_info(info: dict) -> None:
    """Set metadata for the selected peer robot"""
    with bt_lock:
        bt_other_robot_info.clear()
        bt_other_robot_info.update(info or {})


def get_bluetooth_other_robot_info() -> dict:
    """Get metadata for the selected peer robot"""
    with bt_lock:
        return dict(bt_other_robot_info) if bt_other_robot_info else {}


def clear_bluetooth_other_robot_info() -> None:
    with bt_lock:
        bt_other_robot_info.clear()


def get_bluetooth_device_info() -> dict:
    """Get information about this robot's Bluetooth device"""
    with bt_lock:
        return dict(bt_device_info) if bt_device_info else {}


def set_bluetooth_devices_info(devices: list[dict]) -> None:
    """Set list of currently connected remote devices"""
    with bt_lock:
        del bt_devices_info[:]
        bt_devices_info.extend(devices or [])


def get_bluetooth_devices_info() -> list[dict]:
    """Get list of currently connected remote devices"""
    with bt_lock:
        return list(bt_devices_info) if bt_devices_info else []


def set_bluetooth_paired_devices_info(devices: list[dict]) -> None:
    """Set list of all paired/known remote devices"""
    with bt_lock:
        del bt_paired_devices_info[:]
        bt_paired_devices_info.extend(devices or [])


def get_bluetooth_paired_devices_info() -> list[dict]:
    """Get list of all paired/known remote devices"""
    with bt_lock:
        return list(bt_paired_devices_info) if bt_paired_devices_info else []


def add_bluetooth_received_message(message: dict) -> None:
    """Append a received Bluetooth message to shared history"""
    with bt_lock:
        bt_received_messages.append(message or {})


def get_bluetooth_received_messages(clear: bool = False, limit: int | None = None) -> list[dict]:
    """Get received Bluetooth messages history"""
    with bt_lock:
        messages = list(bt_received_messages)
        if limit is not None and limit > 0:
            messages = messages[-limit:]
        if clear:
            bt_received_messages[:] = []
        return messages


def clear_bluetooth_received_messages() -> None:
    with bt_lock:
        bt_received_messages[:] = []


def add_bluetooth_sent_message(message: dict) -> None:
    """Append a sent Bluetooth message to shared history"""
    with bt_lock:
        bt_sent_messages.append(message or {})


def get_bluetooth_sent_messages(clear: bool = False, limit: int | None = None) -> list[dict]:
    """Get sent Bluetooth messages history"""
    with bt_lock:
        messages = list(bt_sent_messages)
        if limit is not None and limit > 0:
            messages = messages[-limit:]
        if clear:
            bt_sent_messages[:] = []
        return messages


def clear_bluetooth_sent_messages() -> None:
    with bt_lock:
        bt_sent_messages[:] = []


def enqueue_bluetooth_command(command_type: str, payload: dict | None = None) -> int:
    """Queue a low-level command for the bluetooth process and return command id"""
    with bt_next_command_id.get_lock():
        command_id = bt_next_command_id.value
        bt_next_command_id.value += 1

    with bt_lock:
        bt_commands.append({
            'id': command_id,
            'type': command_type,
            'payload': payload or {},
            'timestamp': time.time(),
        })

    return command_id


def pop_bluetooth_commands() -> list[dict]:
    """Pop all queued bluetooth commands (used by bluetooth process)"""
    with bt_lock:
        commands = list(bt_commands)
        bt_commands[:] = []
        return commands


def set_bluetooth_command_result(command_id: int, success: bool, data: dict | None = None, error: str | None = None) -> None:
    """Store command execution result (used by bluetooth process)"""
    with bt_lock:
        bt_command_results[command_id] = {
            'command_id': command_id,
            'success': bool(success),
            'data': data or {},
            'error': error,
            'timestamp': time.time(),
        }


def get_bluetooth_command_result(command_id: int, pop: bool = False) -> dict | None:
    """Read command execution result"""
    with bt_lock:
        result = bt_command_results.get(command_id)
        if pop and result is not None:
            try:
                del bt_command_results[command_id]
            except Exception:
                pass
        return dict(result) if result else None


def clear_bluetooth_command_result(command_id: int) -> None:
    with bt_lock:
        if command_id in bt_command_results:
            del bt_command_results[command_id]


def cleanup() -> None:
    try:
        frame_buffer.close()
        frame_buffer.unlink()
    except Exception as e:
        _logger.error(f"Error closing/unlinking frame buffer: {e}")

