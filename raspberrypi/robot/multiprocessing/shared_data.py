import multiprocessing
import multiprocessing.shared_memory
import time
import numpy as np
from multiprocessing.managers import DictProxy
from robot.config import *


_manager = multiprocessing.Manager()

# define logging stuff before we import any robot stuff
logs_buffer = multiprocessing.Queue(maxsize=LOG_BUFFER_MAX_ENTRIES)


from robot import profiling, utils
from robot.config import *
from robot.hardware.teensy import ParsedTeensyData, IRData, CompassData, RunningStateData
from robot.robot import RobotManualControl
from robot.vision import CameraBallData, DetectedObject, GoalDetectionResult
from robot.vision.camera import FrameData
from robot.profiling import profile_function


_logger = utils.get_logger("Shared Data Manager")


@profile_function
def _normalize_camera_name(camera: str | None, allow_both: bool = False) -> str:
    camera_name = (camera or "front").lower()
    allowed = ("front", "back", "both") if allow_both else ("front", "back")
    if camera_name not in allowed:
        return "both" if allow_both else "front"
    return camera_name


# Running state
running_state = _manager.dict()
running_state_lock = profiling.create_profiled_lock("running_state_lock")
@profile_function
def set_running_state(state: RunningStateData | None):
    with running_state_lock:
        running_state.clear()
        if state:
            running_state['running'] = state.running
            running_state['bt_module_enabled'] = state.bt_module_enabled
            running_state['main_switch_enabled'] = state.main_switch_value
            running_state['module_value'] = state.bt_module_value
@profile_function
def get_running_state() -> RunningStateData | None:
    with running_state_lock:
        if not running_state:
            return None
        running = running_state.get('running', False)
        bt_module_enabled = running_state.get('bt_module_enabled', False)
        main_switch_value = running_state.get('main_switch_enabled', False)
        bt_module_value = running_state.get('module_value', False)
    return RunningStateData(
        running=running,
        bt_module_enabled=bt_module_enabled,
        main_switch_value=main_switch_value,
        bt_module_value=bt_module_value,
    )


# Hardware data
hardware_data = _manager.dict()
hardware_data_lock = profiling.create_profiled_lock("hardware_data_lock")
hardware_compass_heading = multiprocessing.Value('d', 999.0)
hardware_ir_angle = multiprocessing.Value('d', 999.0)
hardware_ir_distance = multiprocessing.Value('d', 0.0)
@profile_function
def set_hardware_data(data: ParsedTeensyData | None):
    compass_heading = data.compass.heading if data and data.compass else None
    ir_angle = data.ir.angle if data and data.ir else None
    ir_distance = data.ir.distance if data and data.ir else None
    hardware_compass_heading.value = float(compass_heading) if compass_heading is not None else 999.0
    hardware_ir_angle.value = float(ir_angle) if ir_angle is not None else 999.0
    hardware_ir_distance.value = float(ir_distance) if ir_distance is not None else 0.0

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
    with hardware_data_lock:
        hardware_data.clear()
        hardware_data.update(dict_data)

@profile_function
def get_hardware_data() -> ParsedTeensyData | None:
    with hardware_data_lock:
        if not hardware_data:
            return None
        compass_data = hardware_data.get('compass') or {}
        ir_data = hardware_data.get('ir') or {}
        line_sensors = hardware_data.get('line_sensors')
        raw = hardware_data.get('raw')
        timestamp = hardware_data.get('timestamp')
    return ParsedTeensyData(
        compass=CompassData(
            heading=compass_data.get('heading', 999.0),
            pitch=compass_data.get('pitch', 999.0),
            roll=compass_data.get('roll', 999.0),
        ),
        ir=IRData(
            angle=ir_data.get('angle', 999.0),
            distance=ir_data.get('distance', 0.0),
            sensors=ir_data.get('sensors', []),
            status=ir_data.get('status', -1),
        ),
        line=line_sensors or [],
        raw=raw,
        timestamp=timestamp or 0.0,
    )

@profile_function
def get_hardware_compass_ir() -> tuple[float, float, float]:
    return (
        float(hardware_compass_heading.value),
        float(hardware_ir_angle.value),
        float(hardware_ir_distance.value),
    )


# Motor and kicker state
motor_speeds = multiprocessing.Array('i', [0, 0, 0, 0])
@profile_function
def set_motor_speeds(speeds: list[int]):
    with motor_speeds.get_lock():
        for i in range(min(4, len(speeds))):
            motor_speeds[i] = speeds[i]
@profile_function
def get_motor_speeds() -> list[int]:
    with motor_speeds.get_lock():
        return list(motor_speeds[:])

kicker_state = multiprocessing.Value('b', False)
@profile_function
def set_kicker_state(state: bool):
    kicker_state.value = state
@profile_function
def get_kicker_state() -> bool:
    return kicker_state.value


# Compass
compass_reset = multiprocessing.Value('b', False)
@profile_function
def request_compass_reset() -> None:
    compass_reset.value = True
@profile_function
def check_and_clear_compass_reset() -> bool:
    if compass_reset.value:
        compass_reset.value = False
        return True
    return False


# Robot mode
robot_mode = multiprocessing.Value('i', 0)
@profile_function
def set_robot_mode(mode: int):
    robot_mode.value = mode
@profile_function
def get_robot_mode() -> int:
    return robot_mode.value


# Manual control
manual_control = multiprocessing.Array('d', [0.0, 0.0, 0.0])
@profile_function
def set_manual_control(control: RobotManualControl) -> None:
    with manual_control.get_lock():
        manual_control[0] = control.move_angle
        manual_control[1] = control.move_speed
        manual_control[2] = control.rotate
@profile_function
def get_manual_control() -> RobotManualControl:
    with manual_control.get_lock():
        return RobotManualControl(
            move_angle=manual_control[0],
            move_speed=manual_control[1],
            rotate=manual_control[2]
        )


# Autonomous state
state_machine_change_request = multiprocessing.Array('c', b''.ljust(50))
@profile_function
def request_state_machine_change(name: str) -> None:
    name_bytes = name.encode()[:50].ljust(50)
    with state_machine_change_request.get_lock():
        for i in range(50):
            state_machine_change_request[i] = name_bytes[i:i+1]
@profile_function
def check_state_machine_change_request() -> str:
    with state_machine_change_request.get_lock():
        req = bytes(state_machine_change_request[:]).decode().strip()
        state_machine_change_request[:] = b''.ljust(50)
        return req

current_state_machine_name = multiprocessing.Array('c', b''.ljust(50))
@profile_function
def set_current_state_machine_name(name: str) -> None:
    name_bytes = name.encode()[:50].ljust(50)
    with current_state_machine_name.get_lock():
        for i in range(50):
            current_state_machine_name[i] = name_bytes[i:i+1]
@profile_function
def get_current_state_machine_name() -> str:
    with current_state_machine_name.get_lock():
        return bytes(current_state_machine_name[:]).decode().strip()


# Shared memory for frame data
front_frame_buffer = multiprocessing.shared_memory.SharedMemory(create=True, size=FRAME_SIZE_B)
back_frame_buffer = multiprocessing.shared_memory.SharedMemory(create=True, size=FRAME_SIZE_B)
frame_timestamps = {
    "front": multiprocessing.Value('d', 0.0),
    "back": multiprocessing.Value('d', 0.0),
}
frame_ready = {
    "front": multiprocessing.Value('b', False),
    "back": multiprocessing.Value('b', False),
}
frame_buffers = {
    "front": front_frame_buffer,
    "back": back_frame_buffer,
}
frame_lock = profiling.create_profiled_lock("frame_lock")

@profile_function
def set_camera_frame(frame: FrameData | None, camera_name: str = "front"):
    camera_name = camera_name if camera_name in frame_buffers else "front"
    if frame is None:
        frame_ready[camera_name].value = False
        return
    
    frame_flat = frame.frame.flatten()
    if len(frame_flat) <= FRAME_SIZE_B:
        np_array = np.ndarray(frame_flat.shape, dtype=np.uint8, buffer=frame_buffers[camera_name].buf)
        np_array[:] = frame_flat[:]
        frame_timestamps[camera_name].value = frame.timestamp
        frame_ready[camera_name].value = True

@profile_function
def get_camera_frame(camera_name: str = "front") -> FrameData | None:
    camera_name = camera_name if camera_name in frame_buffers else "front"
    if not frame_ready[camera_name].value:
        return None
    
    try:
        np_array = np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8, buffer=frame_buffers[camera_name].buf)
        frame_copy = np.copy(np_array)
        return FrameData(
            frame=frame_copy,
            timestamp=frame_timestamps[camera_name].value
        )
    except Exception:
        return None


# Camera auto calibration (handled by camera_capture_process)
camera_auto_calibration_lock = profiling.create_profiled_lock("camera_auto_calibration_lock")
camera_auto_calibration_request = _manager.dict()
camera_auto_calibration_result = _manager.dict()
camera_auto_calibration_next_request_id = multiprocessing.Value('i', 1)

@profile_function
def request_camera_auto_calibration(camera: str = "front", settle_time_s: float = 2.0) -> int | None:
    with camera_auto_calibration_lock:
        if camera_auto_calibration_request.get("active", False):
            return None
        request_id = int(camera_auto_calibration_next_request_id.value)
        camera_auto_calibration_next_request_id.value += 1

        camera_name = _normalize_camera_name(camera, allow_both=True)
        camera_auto_calibration_request.clear()
        camera_auto_calibration_request.update({
            "active": True,
            "request_id": request_id,
            "camera": camera_name,
            "settle_time_s": float(settle_time_s),
            "requested_at": time.time(),
        })

        camera_auto_calibration_result.clear()
        camera_auto_calibration_result.update({
            "request_id": request_id,
            "done": False,
            "success": False,
        })
        return request_id

@profile_function
def claim_camera_auto_calibration_request() -> dict | None:
    with camera_auto_calibration_lock:
        if not camera_auto_calibration_request.get("active", False):
            return None
        request = dict(camera_auto_calibration_request)
        camera_auto_calibration_request["active"] = False
        return request

@profile_function
def set_camera_auto_calibration_result(
    request_id: int,
    success: bool,
    result: dict | None = None,
    error: str | None = None,
) -> None:
    with camera_auto_calibration_lock:
        camera_auto_calibration_result.clear()
        camera_auto_calibration_result.update({
            "request_id": int(request_id),
            "done": True,
            "success": bool(success),
            "result": dict(result or {}),
            "error": error,
            "completed_at": time.time(),
        })

@profile_function
def get_camera_auto_calibration_result(request_id: int) -> dict | None:
    with camera_auto_calibration_lock:
        if not camera_auto_calibration_result:
            return None
        if int(camera_auto_calibration_result.get("request_id", -1)) != int(request_id):
            return None
        return dict(camera_auto_calibration_result)


# Detected objects by camera
detected_objects = _manager.list()
@profile_function
def get_detected_objects_raw() -> list[dict]:
    return detected_objects[:]
@profile_function
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
            color=tuple(det_dict.get('color', (255, 255, 255))),
            camera=det_dict.get('camera')
        )
        result.append(obj)
    return result
@profile_function
def set_detected_objects(detections: list[DetectedObject]) -> None:
    payload = []
    for det in detections:
        payload.append({
            'object_type': det.object_type,
            'x': det.x,
            'y': det.y,
            'width': det.width,
            'height': det.height,
            'confidence': det.confidence,
            'color': det.color,
            'camera': det.camera
        })
    detected_objects[:] = payload


# Line detection and calibration
line_calibration_lock = profiling.create_profiled_lock("line_calibration_lock")
line_detection_thresholds = multiprocessing.Array('i', [val for pair in DEFAULT_LINE_DETECTION_THRESHOLDS for val in pair])
@profile_function
def get_line_detection_thresholds() -> list[list[int]]:
    with line_calibration_lock:
        flat = line_detection_thresholds[:]
        return [[flat[i*2], flat[i*2+1]] for i in range(LINE_SENSOR_COUNT)]
@profile_function
def set_line_detection_thresholds(thresholds: list[list[int]]) -> None:
    with line_calibration_lock:
        for i in range(min(LINE_SENSOR_COUNT, len(thresholds))):
            line_detection_thresholds[i * 2] = thresholds[i][0]  # min
            line_detection_thresholds[i * 2 + 1] = thresholds[i][1]  # max

line_detected_lock = profiling.create_profiled_lock("line_detected_lock")
line_detected = multiprocessing.Array('b', [False] * LINE_SENSOR_COUNT)
line_calibration_min = multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT)
line_calibration_max = multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT)
line_calibration_phase = multiprocessing.Value('i', 0)  # 0=inactive, 1=field, 2=lines
line_calibration_phase1_min = multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT)
line_calibration_phase1_max = multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT)
line_calibration_phase2_min = multiprocessing.Array('d', [float('inf')] * LINE_SENSOR_COUNT)
line_calibration_phase2_max = multiprocessing.Array('d', [float('-inf')] * LINE_SENSOR_COUNT)


# Goal detection and calibration

goal_color = multiprocessing.Value('b', True)  # True = yellow, False = blue
@profile_function
def set_goal_color(color: str) -> None:
    goal_color.value = True if color.lower() == 'yellow' else False
@profile_function
def get_goal_color() -> str:
    return 'yellow' if goal_color.value else 'blue'

goal_calibration_yellow_front = _manager.dict()
goal_calibration_yellow_back = _manager.dict()
goal_calibration_blue_front = _manager.dict()
goal_calibration_blue_back = _manager.dict()

@profile_function
def _get_goal_calibration_store(color: str, camera: str) -> DictProxy:
    if color.lower() == 'yellow':
        return goal_calibration_yellow_front if camera == "front" else goal_calibration_yellow_back
    return goal_calibration_blue_front if camera == "front" else goal_calibration_blue_back

@profile_function
def _init_goal_calibration_store(cal_dict: DictProxy, color: str) -> None:
    if color.lower() == 'yellow':
        cal_dict['ranges'] = [([20, 100, 100], [30, 255, 255])]
    else:
        cal_dict['ranges'] = [([100, 100, 100], [130, 255, 255])]

for _store, _color in [
    (goal_calibration_yellow_front, "yellow"),
    (goal_calibration_yellow_back, "yellow"),
    (goal_calibration_blue_front, "blue"),
    (goal_calibration_blue_back, "blue"),
]:
    _init_goal_calibration_store(_store, _color)

@profile_function
def set_goal_calibration(
    color: str,
    ranges: list[tuple[tuple[int, int, int], tuple[int, int, int]]],
    camera: str = "both",
) -> None:
    """Set goal calibration for a color with multiple HSV ranges.
    
    Args:
        color: 'yellow' or 'blue'
        ranges: List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    camera_name = _normalize_camera_name(camera, allow_both=True)
    target_cameras = ("front", "back") if camera_name == "both" else (camera_name,)
    payload = [
        ([int(lower[0]), int(lower[1]), int(lower[2])], [int(upper[0]), int(upper[1]), int(upper[2])])
        for lower, upper in ranges
    ]
    for target_camera in target_cameras:
        cal_dict = _get_goal_calibration_store(color, target_camera)
        cal_dict['ranges'] = payload

@profile_function
def get_goal_calibration(color: str, camera: str = "front") -> list[tuple[tuple[int, int, int], tuple[int, int, int]]]:
    """Get goal calibration ranges for a color.
    
    Returns:
        List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    camera_name = _normalize_camera_name(camera)
    cal_dict = _get_goal_calibration_store(color, camera_name)
    ranges = cal_dict.get('ranges') if cal_dict else None

    if not ranges:
        if color.lower() == 'yellow':
            return [((20, 100, 100), (30, 255, 255))]
        else:
            return [((100, 100, 100), (130, 255, 255))]
    return list(ranges)

goal_detection_result = _manager.dict({'data': None})
goal_detection_result_yellow = _manager.dict({'data': None})
goal_detection_result_blue = _manager.dict({'data': None})

@profile_function
def _goal_result_to_tuple(result: GoalDetectionResult | None) -> tuple | None:
    if result is None:
        return None
    return (
        float(result.alignment),
        bool(result.detected),
        result.center_x,
        float(result.area),
        result.distance_mm,
        float(result.height_pixels),
        float(result.camera_yaw_deg),
    )

@profile_function
def _goal_result_from_tuple(data: tuple | None) -> GoalDetectionResult | None:
    if not data:
        return None
    alignment, detected, center_x, area, distance_mm, height_pixels, camera_yaw_deg = data
    return GoalDetectionResult(
        alignment=alignment,
        detected=detected,
        center_x=center_x,
        area=area,
        distance_mm=distance_mm,
        height_pixels=height_pixels,
        camera_yaw_deg=camera_yaw_deg,
    )

@profile_function
def _set_goal_detection_result_to_store(store: DictProxy, result: GoalDetectionResult | None) -> None:
    store['data'] = _goal_result_to_tuple(result)

@profile_function
def _get_goal_detection_result_from_store(store: DictProxy) -> GoalDetectionResult | None:
    return _goal_result_from_tuple(store.get('data'))

@profile_function
def set_goal_detection_result_for_color(color: str, result: GoalDetectionResult | None) -> None:
    store = goal_detection_result_yellow if color.lower() == 'yellow' else goal_detection_result_blue
    _set_goal_detection_result_to_store(store, result)

@profile_function
def get_goal_detection_result_for_color(color: str) -> GoalDetectionResult | None:
    store = goal_detection_result_yellow if color.lower() == 'yellow' else goal_detection_result_blue
    return _get_goal_detection_result_from_store(store)

@profile_function
def set_goal_detection_result(result: GoalDetectionResult | None) -> None:
    _set_goal_detection_result_to_store(goal_detection_result, result)
@profile_function
def get_goal_detection_result() -> GoalDetectionResult | None:
    return _get_goal_detection_result_from_store(goal_detection_result)

goal_focal_length_front = multiprocessing.Value('d', DEFAULT_FOCAL_LENGTH_PIXELS)
goal_focal_length_back = multiprocessing.Value('d', DEFAULT_FOCAL_LENGTH_PIXELS)
@profile_function
def get_goal_focal_length(camera: str = "front") -> float:
    camera_name = _normalize_camera_name(camera)
    return goal_focal_length_front.value if camera_name == "front" else goal_focal_length_back.value
@profile_function
def set_goal_focal_length(focal_length: float, camera: str = "both") -> None:
    camera_name = _normalize_camera_name(camera, allow_both=True)
    if camera_name in ("front", "both"):
        goal_focal_length_front.value = focal_length
    if camera_name in ("back", "both"):
        goal_focal_length_back.value = focal_length

goal_distance_calibration_active = multiprocessing.Value('b', False)
goal_distance_calibration_data = _manager.dict()
goal_distance_calibration_lock = profiling.create_profiled_lock("goal_distance_calibration_lock")

# Ball detection and calibration
ball_calibration_front = _manager.dict()
ball_calibration_back = _manager.dict()
@profile_function
def _init_ball_calibration_store(cal_dict: DictProxy) -> None:
    cal_dict['ranges'] = [
        (
            [DEFAULT_BALL_CALIBRATION_HSV[0], DEFAULT_BALL_CALIBRATION_HSV[1], DEFAULT_BALL_CALIBRATION_HSV[2]],
            [DEFAULT_BALL_CALIBRATION_HSV[3], DEFAULT_BALL_CALIBRATION_HSV[4], DEFAULT_BALL_CALIBRATION_HSV[5]],
        )
    ]
for _store in [ball_calibration_front, ball_calibration_back]:
    _init_ball_calibration_store(_store)

@profile_function
def _get_ball_calibration_store(camera: str) -> DictProxy:
    return ball_calibration_front if camera == "front" else ball_calibration_back

@profile_function
def set_ball_calibration(ranges: list[tuple[list[int], list[int]]], camera: str = "both") -> None:
    """Set ball calibration with multiple HSV ranges.
    
    Args:
        ranges: List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    camera_name = _normalize_camera_name(camera, allow_both=True)
    target_cameras = ("front", "back") if camera_name == "both" else (camera_name,)
    payload = [
        ([int(lower[0]), int(lower[1]), int(lower[2])], [int(upper[0]), int(upper[1]), int(upper[2])])
        for lower, upper in ranges
    ]
    for target_camera in target_cameras:
        cal_dict = _get_ball_calibration_store(target_camera)
        cal_dict['ranges'] = payload

@profile_function
def get_ball_calibration(camera: str = "front") -> list[tuple[list[int], list[int]]]:
    """Get ball calibration ranges.
    
    Returns:
        List of tuples, each tuple is (lower_hsv, upper_hsv) where each is [h, s, v]
    """
    camera_name = _normalize_camera_name(camera)
    ball_calibration = _get_ball_calibration_store(camera_name)
    ranges = ball_calibration.get('ranges') if ball_calibration else None

    if not ranges:
        # Return default single range
        return [([DEFAULT_BALL_CALIBRATION_HSV[0], DEFAULT_BALL_CALIBRATION_HSV[1], DEFAULT_BALL_CALIBRATION_HSV[2]],
                 [DEFAULT_BALL_CALIBRATION_HSV[3], DEFAULT_BALL_CALIBRATION_HSV[4], DEFAULT_BALL_CALIBRATION_HSV[5]])]
    return list(ranges)

camera_ball_possession = multiprocessing.Value('b', False)
@profile_function
def set_camera_ball_possession(possessed: bool) -> None:
    camera_ball_possession.value = possessed
@profile_function
def get_camera_ball_possession() -> bool:
    return bool(camera_ball_possession.value)

camera_ball_position_lock = profiling.create_profiled_lock("camera_ball_position_lock")
camera_ball_angle = multiprocessing.Value('d', 999.0)  # 999 = not detected
camera_ball_distance = multiprocessing.Value('d', 999.0)
camera_ball_detected = multiprocessing.Value('b', False)
camera_ball_area_pixels = multiprocessing.Value('d', 0.0)
camera_ball_front = _manager.dict({'data': (999.0, 999.0, False, 0.0)})
camera_ball_back = _manager.dict({'data': (999.0, 999.0, False, 0.0)})

@profile_function
def _camera_ball_to_tuple(ball_data: CameraBallData) -> tuple:
    return (float(ball_data.angle), float(ball_data.distance), bool(ball_data.detected), float(ball_data.area_pixels))

@profile_function
def _camera_ball_from_tuple(data: tuple | None) -> CameraBallData:
    if not data:
        return CameraBallData(999.0, 999.0, False, 0.0)
    angle, distance, detected, area_pixels = data
    return CameraBallData(float(angle), float(distance), bool(detected), float(area_pixels))

@profile_function
def _update_camera_ball_store(store: DictProxy, ball_data: CameraBallData) -> None:
    store['data'] = _camera_ball_to_tuple(ball_data)

@profile_function
def set_camera_ball_data(ball_data: CameraBallData) -> None:
    with camera_ball_position_lock:
        camera_ball_angle.value = ball_data.angle
        camera_ball_distance.value = ball_data.distance
        camera_ball_detected.value = ball_data.detected
        camera_ball_area_pixels.value = ball_data.area_pixels

@profile_function
def set_camera_ball_data_for_camera(camera: str, ball_data: CameraBallData) -> None:
    target = camera_ball_front if _normalize_camera_name(camera) == "front" else camera_ball_back
    _update_camera_ball_store(target, ball_data)

@profile_function
def set_camera_ball_data_for_cameras(ball_data_by_camera: dict[str, CameraBallData]) -> None:
    if not ball_data_by_camera:
        return

    for camera_name, ball_data in ball_data_by_camera.items():
        normalized = _normalize_camera_name(camera_name)
        target = camera_ball_front if normalized == "front" else camera_ball_back
        _update_camera_ball_store(target, ball_data)

@profile_function
def get_camera_ball_data() -> CameraBallData:
    with camera_ball_position_lock:
        return CameraBallData(
            angle=camera_ball_angle.value,
            distance=camera_ball_distance.value,
            detected=camera_ball_detected.value,
            area_pixels=camera_ball_area_pixels.value
        )

@profile_function
def get_camera_ball_data_for_camera(camera: str = "front") -> CameraBallData:
    source = camera_ball_front if _normalize_camera_name(camera) == "front" else camera_ball_back
    data = source.get('data') if source else None
    return _camera_ball_from_tuple(data)


camera_ball_distance_calibration_constant_front = multiprocessing.Value('d', 10000.0)  # Default constant
camera_ball_distance_calibration_constant_back = multiprocessing.Value('d', 10000.0)   # Default constant
@profile_function
def get_camera_ball_calibration_constant(camera: str = "front") -> float:
    camera_name = _normalize_camera_name(camera)
    return (
        camera_ball_distance_calibration_constant_front.value
        if camera_name == "front"
        else camera_ball_distance_calibration_constant_back.value
    )
@profile_function
def set_camera_ball_calibration_constant(constant: float, camera: str = "both") -> None:
    camera_name = _normalize_camera_name(camera, allow_both=True)
    if camera_name in ("front", "both"):
        camera_ball_distance_calibration_constant_front.value = float(constant)
    if camera_name in ("back", "both"):
        camera_ball_distance_calibration_constant_back.value = float(constant)

camera_ball_usage_enabled = multiprocessing.Value('b', AUTO_CAMERA_BALL_TRACKING_ENABLED)
@profile_function
def set_camera_ball_usage_enabled(enabled: bool) -> None:
    camera_ball_usage_enabled.value = enabled
@profile_function
def get_camera_ball_usage_enabled() -> bool:
    return camera_ball_usage_enabled.value


# Feature toggles
rotation_correction_enabled = multiprocessing.Value('b', DEFAULT_ROTATION_CORRECTION_ENABLED)
@profile_function
def set_rotation_correction_enabled(enabled: bool) -> None:
    rotation_correction_enabled.value = enabled
@profile_function
def get_rotation_correction_enabled() -> bool:
    return rotation_correction_enabled.value

line_avoiding_enabled = multiprocessing.Value('b', DEFAULT_LINE_AVOIDING_ENABLED)
@profile_function
def set_line_avoiding_enabled(enabled: bool) -> None:
    line_avoiding_enabled.value = enabled

@profile_function
def get_line_avoiding_enabled() -> bool:
    return line_avoiding_enabled.value

position_based_speed_enabled = multiprocessing.Value('b', DEFAULT_POSITION_BASED_SPEED_ENABLED)
@profile_function
def set_position_based_speed_enabled(enabled: bool) -> None:
    position_based_speed_enabled.value = enabled

@profile_function
def get_position_based_speed_enabled() -> bool:
    return position_based_speed_enabled.value

position_estimate_lock = profiling.create_profiled_lock("position_estimate_lock")
last_position_estimate = _manager.dict()
@profile_function
def set_last_position_estimate(x_mm: float, y_mm: float, confidence: float) -> None:
    with position_estimate_lock:
        last_position_estimate.clear()
        last_position_estimate["x_mm"] = float(x_mm)
        last_position_estimate["y_mm"] = float(y_mm)
        last_position_estimate["confidence"] = float(confidence)
        last_position_estimate["timestamp"] = time.time()
@profile_function
def get_last_position_estimate() -> dict | None:
    with position_estimate_lock:
        return dict(last_position_estimate) if last_position_estimate else None

always_facing_goal_enabled = multiprocessing.Value('b', True)  # True = always face goal, False = face north
@profile_function
def set_always_facing_goal_enabled(enabled: bool) -> None:
    always_facing_goal_enabled.value = enabled

@profile_function
def get_always_facing_goal_enabled() -> bool:
    return always_facing_goal_enabled.value



# Bluetooth Communication
bt_device_info = _manager.dict()
bt_devices_info = _manager.list()
bt_paired_devices_info = _manager.list()
bt_received_messages = _manager.list()
bt_sent_messages = _manager.list()
bt_commands = _manager.list()
bt_command_results = _manager.dict()
bt_next_command_id = multiprocessing.Value('i', 1)
bt_process_alive = multiprocessing.Value('b', False)
bt_other_robot_info = _manager.dict()
bt_lock = profiling.create_profiled_lock("bt_lock")


@profile_function
def set_bluetooth_process_alive(alive: bool) -> None:
    bt_process_alive.value = alive


@profile_function
def get_bluetooth_process_alive() -> bool:
    return bool(bt_process_alive.value)


@profile_function
def set_bluetooth_device_info(info: dict) -> None:
    with bt_lock:
        bt_device_info.clear()
        bt_device_info.update(info or {})


@profile_function
def set_bluetooth_other_robot_info(info: dict) -> None:
    with bt_lock:
        bt_other_robot_info.clear()
        bt_other_robot_info.update(info or {})


@profile_function
def get_bluetooth_other_robot_info() -> dict:
    with bt_lock:
        return dict(bt_other_robot_info) if bt_other_robot_info else {}


@profile_function
def clear_bluetooth_other_robot_info() -> None:
    with bt_lock:
        bt_other_robot_info.clear()


@profile_function
def get_bluetooth_device_info() -> dict:
    with bt_lock:
        return dict(bt_device_info) if bt_device_info else {}


@profile_function
def set_bluetooth_devices_info(devices: list[dict]) -> None:
    with bt_lock:
        del bt_devices_info[:]
        bt_devices_info.extend(devices or [])


@profile_function
def get_bluetooth_devices_info() -> list[dict]:
    with bt_lock:
        return list(bt_devices_info) if bt_devices_info else []


@profile_function
def set_bluetooth_paired_devices_info(devices: list[dict]) -> None:
    with bt_lock:
        del bt_paired_devices_info[:]
        bt_paired_devices_info.extend(devices or [])


@profile_function
def get_bluetooth_paired_devices_info() -> list[dict]:
    with bt_lock:
        return list(bt_paired_devices_info) if bt_paired_devices_info else []


@profile_function
def add_bluetooth_received_message(message: dict) -> None:
    with bt_lock:
        bt_received_messages.append(message or {})


@profile_function
def get_bluetooth_received_messages(clear: bool = False, limit: int | None = None) -> list[dict]:
    with bt_lock:
        messages = list(bt_received_messages)
        if limit is not None and limit > 0:
            messages = messages[-limit:]
        if clear:
            bt_received_messages[:] = []
        return messages


@profile_function
def clear_bluetooth_received_messages() -> None:
    with bt_lock:
        bt_received_messages[:] = []


@profile_function
def add_bluetooth_sent_message(message: dict) -> None:
    with bt_lock:
        bt_sent_messages.append(message or {})


@profile_function
def get_bluetooth_sent_messages(clear: bool = False, limit: int | None = None) -> list[dict]:
    with bt_lock:
        messages = list(bt_sent_messages)
        if limit is not None and limit > 0:
            messages = messages[-limit:]
        if clear:
            bt_sent_messages[:] = []
        return messages


@profile_function
def clear_bluetooth_sent_messages() -> None:
    with bt_lock:
        bt_sent_messages[:] = []


@profile_function
def enqueue_bluetooth_command(command_type: str, payload: dict | None = None) -> int:
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


@profile_function
def pop_bluetooth_commands() -> list[dict]:
    with bt_lock:
        commands = list(bt_commands)
        bt_commands[:] = []
        return commands


@profile_function
def set_bluetooth_command_result(command_id: int, success: bool, data: dict | None = None, error: str | None = None) -> None:
    with bt_lock:
        bt_command_results[command_id] = {
            'command_id': command_id,
            'success': bool(success),
            'data': data or {},
            'error': error,
            'timestamp': time.time(),
        }


@profile_function
def get_bluetooth_command_result(command_id: int, pop: bool = False) -> dict | None:
    with bt_lock:
        result = bt_command_results.get(command_id)
        if pop and result is not None:
            try:
                del bt_command_results[command_id]
            except Exception:
                pass
        return dict(result) if result else None


@profile_function
def clear_bluetooth_command_result(command_id: int) -> None:
    with bt_lock:
        if command_id in bt_command_results:
            del bt_command_results[command_id]


@profile_function
def cleanup() -> None:
    try:
        front_frame_buffer.close()
        front_frame_buffer.unlink()
        back_frame_buffer.close()
        back_frame_buffer.unlink()
    except Exception as e:
        _logger.error(f"Error closing/unlinking frame buffer: {e}")
    
    try:
        _manager.shutdown()
        _logger.debug("Manager shut down successfully")
    except Exception as e:
        _logger.warning(f"Error shutting down manager: {e}")
