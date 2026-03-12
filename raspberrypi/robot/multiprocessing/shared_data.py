import multiprocessing
import multiprocessing.shared_memory
import numpy as np
from robot.config import *


_manager = multiprocessing.Manager()

# define logging stuff before we import any robot stuff
logs_buffer = multiprocessing.Queue(maxsize=LOG_BUFFER_MAX_ENTRIES)
logs_buffer_lock = multiprocessing.Lock()


import robot.utils as utils
from robot.config import *
from robot.hardware.teensy import ParsedTeensyData, IRData, CompassData, RunningStateData
from robot.robot import RobotManualControl
from robot.vision import CameraBallPosition, DetectedObject, GoalDetectionResult
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
autonomous_state = _manager.dict()
autonomous_state_lock = multiprocessing.Lock()

def set_autonomous_state(state: str, ball_angle: float, ball_distance: float, goal: GoalDetectionResult, have_ball: bool):
    state_info = {
        'state': state,
        'ball_angle': int(ball_angle) if ball_angle != 999 else None,
        'ball_distance': int(ball_distance) if ball_distance != 999 else None,
        'have_ball': have_ball,
        'goal_detected': goal.goal_detected if goal else False,
        'goal_alignment': round(goal.alignment, 3) if goal else None,
        'goal_distance_mm': int(round(goal.distance_mm)) if goal and goal.distance_mm else None,
    }
    with autonomous_state_lock:
        autonomous_state.clear()
        autonomous_state.update(state_info)

def get_autonomous_state() -> dict:
    with autonomous_state_lock:
        return dict(autonomous_state)


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

goal_calibration_yellow = multiprocessing.Array('i', [20, 100, 100, 30, 255, 255])
goal_calibration_blue = multiprocessing.Array('i', [100, 100, 100, 130, 255, 255])
def set_goal_calibration(color: str, lower: list[int], upper: list[int]) -> None:
    with goal_detection_lock:
        if color.lower() == 'yellow':
            for i in range(3):
                goal_calibration_yellow[i] = lower[i]
                goal_calibration_yellow[i + 3] = upper[i]
        elif color.lower() == 'blue':
            for i in range(3):
                goal_calibration_blue[i] = lower[i]
                goal_calibration_blue[i + 3] = upper[i]
def get_goal_calibration(color: str) -> tuple[list[int], list[int]]:
    with goal_detection_lock:
        if color.lower() == 'yellow':
            arr = goal_calibration_yellow[:]
            return arr[:3], arr[3:]
        elif color.lower() == 'blue':
            arr = goal_calibration_blue[:]
            return arr[:3], arr[3:]
        return [0, 0, 0], [0, 0, 0]

goal_detection_result = _manager.dict()
def set_goal_detection_result(result: GoalDetectionResult | None) -> None:
    with goal_detection_lock:
        goal_detection_result.clear()
        if result:
            goal_detection_result['alignment'] = result.alignment
            goal_detection_result['goal_detected'] = result.goal_detected
            goal_detection_result['goal_center_x'] = result.goal_center_x
            goal_detection_result['goal_area'] = result.goal_area
            goal_detection_result['distance_mm'] = result.distance_mm
            goal_detection_result['goal_height_pixels'] = result.goal_height_pixels
def get_goal_detection_result() -> GoalDetectionResult | None:
    with goal_detection_lock:
        if not goal_detection_result:
            return None
        return GoalDetectionResult(
            alignment=goal_detection_result.get('alignment', 0.0),
            goal_detected=goal_detection_result.get('goal_detected', False),
            goal_center_x=goal_detection_result.get('goal_center_x', None),
            goal_area=goal_detection_result.get('goal_area', 0.0),
            distance_mm=goal_detection_result.get('distance_mm', None),
            goal_height_pixels=goal_detection_result.get('goal_height_pixels', 0.0)
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
ball_calibration_hsv = multiprocessing.Array('i', DEFAULT_BALL_CALIBRATION_HSV)
def set_ball_calibration(lower: list[int], upper: list[int]) -> None:
    with goal_detection_lock:
        for i in range(3):
            ball_calibration_hsv[i] = lower[i]
            ball_calibration_hsv[i + 3] = upper[i]
def get_ball_calibration() -> tuple[list[int], list[int]]:
    with goal_detection_lock:
        arr = ball_calibration_hsv[:]
        return arr[:3], arr[3:]

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

def set_camera_ball_data(ball_data: CameraBallPosition) -> None:
    with camera_ball_position_lock:
        camera_ball_angle.value = ball_data.angle
        camera_ball_distance.value = ball_data.distance
        camera_ball_detected.value = ball_data.detected
        camera_ball_area_pixels.value = ball_data.area_pixels

def get_camera_ball_data() -> CameraBallPosition:
    with camera_ball_position_lock:
        return CameraBallPosition(
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



def cleanup() -> None:
    try:
        frame_buffer.close()
        frame_buffer.unlink()
    except Exception as e:
        _logger.error(f"Error closing/unlinking frame buffer: {e}")

