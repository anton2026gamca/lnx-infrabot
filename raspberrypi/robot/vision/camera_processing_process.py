import cv2
import logging
import multiprocessing.synchronize
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor

from robot import calibration, utils, vision
from robot.multiprocessing import shared_data
from robot.profiling import profile_function, sleep

from robot.vision import GoalColorCalibration, GoalDetectionResult, DetectedObject
from robot.vision.color_mask import HSV_LUT_RANGE_THRESHOLD, build_hsv_range_lut
from robot.config import *


CAMERA_CONFIG = [
    ("front", CAMERA_FRONT_YAW_DEG),
    ("back", CAMERA_BACK_YAW_DEG),
]

CALIBRATION_REFRESH_INTERVAL_S = 0.25
USE_CAMERA_PROCESSING_THREADS = True
NO_BALL_DATA = vision.CameraBallData(999.0, 999.0, False, 0.0)


def _empty_goal_result(camera_yaw: float) -> GoalDetectionResult:
    return GoalDetectionResult(0.0, False, None, 0.0, None, 0.0, camera_yaw_deg=camera_yaw)


def _target_goal_color_for_camera(
    camera_yaw: float,
    robot_heading_deg: float | None,
    enemy_goal_color: str,
    own_goal_color: str,
) -> str:
    reference_heading_deg = robot_heading_deg if robot_heading_deg is not None else 0.0
    camera_world_yaw_deg = utils.normalize_angle_deg(reference_heading_deg + camera_yaw)
    enemy_error = abs(utils.normalize_angle_deg(camera_world_yaw_deg - 0.0))
    own_error = abs(utils.normalize_angle_deg(camera_world_yaw_deg - 180.0))
    return enemy_goal_color if enemy_error <= own_error else own_goal_color


def _build_possession_detection(camera_name: str, ball_possessed: bool) -> DetectedObject:
    area_width = int(DETECTION_FRAME_WIDTH * (AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT / 100.0))
    area_height = int(DETECTION_FRAME_HEIGHT * (AUTO_BALL_POSSESSION_AREA_HEIGHT_PERCENT / 100.0))
    area_x = (DETECTION_FRAME_WIDTH - area_width) // 2
    area_y = DETECTION_FRAME_HEIGHT - area_height
    return DetectedObject(
        object_type="ball_possession_area",
        x=area_x,
        y=area_y,
        width=area_width,
        height=area_height,
        color=(0, 255, 0) if ball_possessed else (0, 0, 255),
        camera=camera_name,
    )


def _ranges_signature(
    ranges: list[tuple[tuple[int, int, int], tuple[int, int, int]]] | list[tuple[list[int], list[int]]]
) -> tuple[tuple[tuple[int, int, int], tuple[int, int, int]], ...]:
    return tuple(
        (
            (int(lower[0]), int(lower[1]), int(lower[2])),
            (int(upper[0]), int(upper[1]), int(upper[2])),
        )
        for lower, upper in ranges
    )


def _build_camera_runtime_state() -> dict[str, dict]:
    return {
        camera_name: {
            "next_refresh_at": 0.0,
            "yellow_sig": (),
            "blue_sig": (),
            "ball_sig": (),
            "yellow_ranges_np": [],
            "blue_ranges_np": [],
            "yellow_lut": None,
            "blue_lut": None,
            "ball_ranges_np": [],
            "ball_lut": None,
            "goal_calibration": GoalColorCalibration(yellow_ranges=[], blue_ranges=[]),
            "focal_length": DEFAULT_FOCAL_LENGTH_PIXELS,
            "ball_calibration_constant": 10000.0,
            "last_ball_possessed": False,
            "last_goal_results": {
                "yellow": _empty_goal_result(CAMERA_FRONT_YAW_DEG if camera_name == "front" else CAMERA_BACK_YAW_DEG),
                "blue": _empty_goal_result(CAMERA_FRONT_YAW_DEG if camera_name == "front" else CAMERA_BACK_YAW_DEG),
            },
            "last_camera_ball_data": NO_BALL_DATA,
        }
        for camera_name, _ in CAMERA_CONFIG
    }


@profile_function
def _refresh_camera_runtime_state(
    camera_name: str,
    camera_state: dict,
    now: float,
) -> None:
    if now < camera_state["next_refresh_at"]:
        return
    camera_state["next_refresh_at"] = now + CALIBRATION_REFRESH_INTERVAL_S

    yellow_ranges = shared_data.get_goal_calibration("yellow", camera_name)
    blue_ranges = shared_data.get_goal_calibration("blue", camera_name)
    ball_ranges = shared_data.get_ball_calibration(camera_name)

    yellow_sig = _ranges_signature(yellow_ranges)
    blue_sig = _ranges_signature(blue_ranges)
    ball_sig = _ranges_signature(ball_ranges)

    goal_ranges_changed = False
    if yellow_sig != camera_state["yellow_sig"]:
        camera_state["yellow_sig"] = yellow_sig
        camera_state["yellow_ranges_np"] = [
            (np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            for lower, upper in yellow_sig
        ]
        camera_state["yellow_lut"] = (
            build_hsv_range_lut(camera_state["yellow_ranges_np"])
            if len(camera_state["yellow_ranges_np"]) >= HSV_LUT_RANGE_THRESHOLD
            else None
        )
        goal_ranges_changed = True
    if blue_sig != camera_state["blue_sig"]:
        camera_state["blue_sig"] = blue_sig
        camera_state["blue_ranges_np"] = [
            (np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            for lower, upper in blue_sig
        ]
        camera_state["blue_lut"] = (
            build_hsv_range_lut(camera_state["blue_ranges_np"])
            if len(camera_state["blue_ranges_np"]) >= HSV_LUT_RANGE_THRESHOLD
            else None
        )
        goal_ranges_changed = True
    if goal_ranges_changed:
        camera_state["goal_calibration"] = GoalColorCalibration(
            yellow_ranges=camera_state["yellow_ranges_np"],
            blue_ranges=camera_state["blue_ranges_np"],
            yellow_lut=camera_state["yellow_lut"],
            blue_lut=camera_state["blue_lut"],
        )

    if ball_sig != camera_state["ball_sig"]:
        camera_state["ball_sig"] = ball_sig
        camera_state["ball_ranges_np"] = [
            (np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            for lower, upper in ball_sig
        ]
        camera_state["ball_lut"] = (
            build_hsv_range_lut(camera_state["ball_ranges_np"])
            if len(camera_state["ball_ranges_np"]) >= HSV_LUT_RANGE_THRESHOLD
            else None
        )

    camera_state["focal_length"] = shared_data.get_goal_focal_length(camera_name)
    camera_state["ball_calibration_constant"] = shared_data.get_camera_ball_calibration_constant(camera_name)


@profile_function
def _process_camera_frame(
    frame_entry: tuple[str, float, np.ndarray],
    camera_state: dict,
    enemy_goal_color: str,
    own_goal_color: str,
    robot_heading_deg: float | None,
) -> dict:
    camera_name, camera_yaw, bgr_frame = frame_entry
    if DETECTION_FRAME_SIZE_SCALE < 1.0:
        resized_frame = cv2.resize(bgr_frame, (DETECTION_FRAME_WIDTH, DETECTION_FRAME_HEIGHT))
    else:
        resized_frame = bgr_frame

    hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

    detections: list[DetectedObject] = []

    target_goal_color = _target_goal_color_for_camera(
        camera_yaw=camera_yaw,
        robot_heading_deg=robot_heading_deg,
        enemy_goal_color=enemy_goal_color,
        own_goal_color=own_goal_color,
    )
    result, goal_detections = vision.detect_goal_alignment_with_rect(
        hsv_frame,
        target_goal_color,
        camera_state["goal_calibration"],
        focal_length_pixels=camera_state["focal_length"],
        real_goal_height_mm=GOAL_HEIGHT_MM,
    )
    result.camera_yaw_deg = camera_yaw
    for det in goal_detections:
        det.camera = camera_name
    detections.extend(goal_detections)

    ball_detections, _ = vision.detect_ball(
        hsv_frame,
        camera_state["ball_ranges_np"],
        range_lut=camera_state["ball_lut"],
    )
    camera_ball_data = NO_BALL_DATA
    if ball_detections:
        camera_ball_data = vision.calculate_ball_data(
            ball_detections,
            DETECTION_FRAME_WIDTH,
            CAMERA_FOV_DEG,
            camera_state["ball_calibration_constant"],
        )
        camera_ball_data.angle = utils.normalize_angle_deg(camera_ball_data.angle + camera_yaw)
        for det in ball_detections:
            det.camera = camera_name
        detections.extend(ball_detections)

    ball_possessed = False
    if camera_name in BALL_POSSESSION_CAMERAS:
        ball_center_x = ball_detections[0].x + ball_detections[0].width / 2 if ball_detections else None
        ball_center_y = ball_detections[0].y + ball_detections[0].height / 2 if ball_detections else None
        ball_possessed, _ = vision.detect_ball_possession(
            ball_center_x,
            ball_center_y,
            DETECTION_FRAME_WIDTH,
            DETECTION_FRAME_HEIGHT,
            AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT,
            AUTO_BALL_POSSESSION_AREA_HEIGHT_PERCENT,
        )

    return {
        "camera_name": camera_name,
        "goal_color": target_goal_color,
        "goal_result": result,
        "detections": detections,
        "camera_ball_data": camera_ball_data,
        "ball_possessed": ball_possessed,
    }


@profile_function
def _fuse_goal_results(results: list[GoalDetectionResult]) -> GoalDetectionResult:
    if not results:
        return GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    best = min(
        (r for r in results if r.detected),
        key=lambda obs: (
            obs.distance_mm if obs.distance_mm is not None else float("inf"),
            -obs.area,
        ),
        default=None,
    )
    if best is None:
        return GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    return best


@profile_function
def _fuse_ball_data(
    data: list[vision.CameraBallData],
    ir_ball_angle: float | None,
    ir_ball_detected: bool,
) -> vision.CameraBallData:
    if not data:
        return NO_BALL_DATA

    if ir_ball_detected and ir_ball_angle is not None:
        best = min(
            data,
            key=lambda d: abs(utils.normalize_angle_deg(d.angle - ir_ball_angle)),
            default=None,
        )
        if best is not None:
            return best

    return max(data, key=lambda item: item.area_pixels)


def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    frames_processed = 0
    last_debug_msg_time = time.perf_counter()
    last_frame_timestamps: dict[str, float | None] = {"front": None, "back": None}
    frame_skip_count = 0
    camera_runtime_state = _build_camera_runtime_state()

    target_period = 1.0 / CAMERA_MAX_FPS
    last_process_time = time.perf_counter()
    last_processed_frames: dict[str, tuple[float, np.ndarray] | None] = {"front": None, "back": None}

    with ThreadPoolExecutor(max_workers=len(CAMERA_CONFIG), thread_name_prefix="camera-detector") as detector_pool:
        while not stop_event.is_set():
            elapsed = time.perf_counter() - last_process_time
            if elapsed < target_period * 0.95:
                sleep(max(0.0, target_period - elapsed - 0.0005))
                continue

            now = time.perf_counter()
            last_process_time = now

            frame_entries: dict[str, tuple[str, float, np.ndarray]] = {}
            for camera_name, camera_yaw in CAMERA_CONFIG:
                frame = shared_data.get_camera_frame(camera_name=camera_name)
                if frame is None:
                    last_processed_frame = last_processed_frames[camera_name]
                    if last_processed_frame is not None:
                        timestamp, bgr_frame = last_processed_frame
                        frame_entries[camera_name] = (camera_name, camera_yaw, bgr_frame)
                    continue
                if (
                    last_frame_timestamps[camera_name] is not None
                    and frame.timestamp == last_frame_timestamps[camera_name]
                ):
                    last_processed_frame = last_processed_frames[camera_name]
                    if last_processed_frame is not None:
                        timestamp, bgr_frame = last_processed_frame
                        frame_entries[camera_name] = (camera_name, camera_yaw, bgr_frame)
                    continue
                last_frame_timestamps[camera_name] = frame.timestamp
                last_processed_frames[camera_name] = (frame.timestamp, frame.frame)
                _refresh_camera_runtime_state(camera_name, camera_runtime_state[camera_name], now)
                frame_entries[camera_name] = (camera_name, camera_yaw, frame.frame)

            if not frame_entries:
                frame_skip_count += 1
                if frame_skip_count > 2:
                    sleep(0.001)
                continue

            frame_skip_count = 0
            enemy_goal_color = shared_data.get_goal_color().lower()
            own_goal_color = "blue" if enemy_goal_color == "yellow" else "yellow"
            heading_deg, ir_angle, ir_distance = shared_data.get_hardware_compass_ir()
            robot_heading_deg = None
            if heading_deg != 999.0:
                robot_heading_deg = utils.normalize_angle_deg(heading_deg)

            all_detections: list[DetectedObject] = []
            goal_results_by_camera: dict[str, dict[str, GoalDetectionResult]] = {
                camera_name: {
                    "yellow": camera_runtime_state[camera_name]["last_goal_results"]["yellow"],
                    "blue": camera_runtime_state[camera_name]["last_goal_results"]["blue"],
                }
                for camera_name, _ in CAMERA_CONFIG
            }
            camera_ball_data_by_camera: dict[str, vision.CameraBallData] = {
                camera_name: camera_runtime_state[camera_name]["last_camera_ball_data"]
                for camera_name, _ in CAMERA_CONFIG
            }

            frame_list = [frame_entries[cam_name] for cam_name, _ in CAMERA_CONFIG if cam_name in frame_entries]
            if len(frame_list) == 1 or not USE_CAMERA_PROCESSING_THREADS:
                camera_results = [
                    _process_camera_frame(
                        entry,
                        camera_runtime_state[entry[0]],
                        enemy_goal_color,
                        own_goal_color,
                        robot_heading_deg,
                    )
                    for entry in frame_list
                ]
            else:
                front_future = detector_pool.submit(
                    _process_camera_frame,
                    frame_list[0],
                    camera_runtime_state[frame_list[0][0]],
                    enemy_goal_color,
                    own_goal_color,
                    robot_heading_deg,
                )
                back_future = detector_pool.submit(
                    _process_camera_frame,
                    frame_list[1],
                    camera_runtime_state[frame_list[1][0]],
                    enemy_goal_color,
                    own_goal_color,
                    robot_heading_deg,
                )
                camera_results = [front_future.result(), back_future.result()]

            for camera_result in camera_results:
                camera_name = camera_result["camera_name"]
                goal_color = camera_result["goal_color"]
                goal_result = camera_result["goal_result"]
                goal_results_by_camera[camera_name][goal_color] = goal_result
                camera_runtime_state[camera_name]["last_goal_results"][goal_color] = goal_result
                if goal_color == enemy_goal_color:
                    calibration.update_goal_distance_calibration(goal_result, camera_name)

                camera_ball_data = camera_result["camera_ball_data"]
                camera_ball_data_by_camera[camera_name] = camera_ball_data
                camera_runtime_state[camera_name]["last_camera_ball_data"] = camera_ball_data
                if camera_name in BALL_POSSESSION_CAMERAS:
                    camera_runtime_state[camera_name]["last_ball_possessed"] = bool(camera_result["ball_possessed"])

                all_detections.extend(camera_result["detections"])

            # Keep per-camera shared state fresh even if only one camera delivered a new frame this cycle.
            shared_data.set_camera_ball_data_for_cameras(camera_ball_data_by_camera)

            goals_by_color: dict[str, list[GoalDetectionResult]] = {"yellow": [], "blue": []}
            for camera_name, _ in CAMERA_CONFIG:
                goals_by_color["yellow"].append(goal_results_by_camera[camera_name]["yellow"])
                goals_by_color["blue"].append(goal_results_by_camera[camera_name]["blue"])

            yellow_result = _fuse_goal_results(goals_by_color["yellow"])
            blue_result = _fuse_goal_results(goals_by_color["blue"])
            shared_data.set_goal_detection_result_for_color("yellow", yellow_result)
            shared_data.set_goal_detection_result_for_color("blue", blue_result)

            enemy_result = yellow_result if enemy_goal_color == "yellow" else blue_result
            own_result = blue_result if own_goal_color == "blue" else yellow_result

            ir_ball_angle = None
            ir_ball_detected = False
            if ir_angle != 999.0 and ir_distance != 0:
                ir_ball_angle = utils.normalize_angle_deg(ir_angle)
                ir_ball_detected = True

            ball_candidates = [
                camera_ball_data_by_camera[camera_name]
                for camera_name, _ in CAMERA_CONFIG
                if camera_ball_data_by_camera[camera_name].detected
            ]
            fused_ball_data = _fuse_ball_data(ball_candidates, ir_ball_angle, ir_ball_detected)
            ball_possession_active = any(
                camera_runtime_state[camera_name]["last_ball_possessed"]
                for camera_name in BALL_POSSESSION_CAMERAS
            )
            shared_data.set_camera_ball_data(fused_ball_data)
            shared_data.set_camera_ball_possession(ball_possession_active)

            for camera_name in BALL_POSSESSION_CAMERAS:
                all_detections.append(
                    _build_possession_detection(
                        camera_name=camera_name,
                        ball_possessed=camera_runtime_state[camera_name]["last_ball_possessed"],
                    )
                )

            if ball_possession_active:
                for det in all_detections:
                    if det.object_type == "ball":
                        det.color = (0, 255, 0)

            if DETECTION_FRAME_SIZE_SCALE < 1.0:
                inv_scale = 1.0 / DETECTION_FRAME_SIZE_SCALE
                for det in all_detections:
                    det.x = int(det.x * inv_scale)
                    det.y = int(det.y * inv_scale)
                    det.width = int(det.width * inv_scale)
                    det.height = int(det.height * inv_scale)

            shared_data.set_detected_objects(all_detections)

            frames_processed += 1
            if time.perf_counter() > last_debug_msg_time + 1:
                logger.debug(
                    f"Camera Processing FPS: {frames_processed} "
                    f"(enemy_goal={enemy_result.detected}, own_goal={own_result.detected}, ball={fused_ball_data.detected})"
                )
                frames_processed = 0
                last_debug_msg_time = time.perf_counter()
