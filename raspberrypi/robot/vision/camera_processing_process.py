import cv2
import logging
import multiprocessing.synchronize
import numpy as np
import time

from robot import calibration, utils, vision
from robot.multiprocessing import shared_data

from robot.vision import GoalColorCalibration, GoalDetectionResult, DetectedObject
from robot.config import *


CAMERA_CONFIG = [
    ("front", CAMERA_FRONT_YAW_DEG),
    ("back", CAMERA_BACK_YAW_DEG),
]


def _fuse_goal_results(results: list[GoalDetectionResult]) -> GoalDetectionResult:
    if not results:
        return GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    detected = [r for r in results if r.detected]
    if not detected:
        return GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    detected.sort(
        key=lambda obs: (
            obs.distance_mm if obs.distance_mm is not None else float("inf"),
            -obs.area,
        )
    )
    return detected[0]


def _fuse_ball_data(
    data: list[vision.CameraBallData],
    ir_ball_angle: float | None,
    ir_ball_detected: bool,
) -> vision.CameraBallData:
    if not data:
        return vision.CameraBallData(999.0, 999.0, False, 0.0)

    if ir_ball_detected and ir_ball_angle is not None:
        best = None
        best_error = float("inf")
        for d in data:
            error = abs(utils.normalize_angle_deg(d.angle - ir_ball_angle))
            if error < best_error:
                best_error = error
                best = d
        if best is not None:
            return best

    data.sort(key=lambda data: data.area_pixels, reverse=True)
    return data[0]


def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    frames_processed = 0
    last_debug_msg_time = time.perf_counter()
    last_frame_timestamps: dict[str, float | None] = {"front": None, "back": None}
    frame_skip_count = 0

    target_period = 1.0 / CAMERA_MAX_FPS
    last_process_time = time.perf_counter()

    while not stop_event.is_set():
        elapsed = time.perf_counter() - last_process_time
        if elapsed < target_period * 0.95:
            time.sleep(max(0.0, target_period - elapsed - 0.0005))
            continue

        last_process_time = time.perf_counter()

        frame_entries: list[tuple[str, float, np.ndarray]] = []
        for camera_name, camera_yaw in CAMERA_CONFIG:
            frame = shared_data.get_camera_frame(camera_name=camera_name)
            if frame is None:
                continue
            if (
                last_frame_timestamps[camera_name] is not None
                and frame.timestamp == last_frame_timestamps[camera_name]
            ):
                continue
            last_frame_timestamps[camera_name] = frame.timestamp
            frame_entries.append((camera_name, camera_yaw, frame.frame))

        if not frame_entries:
            frame_skip_count += 1
            if frame_skip_count > 2:
                time.sleep(0.001)
            continue

        frame_skip_count = 0
        enemy_goal_color = shared_data.get_goal_color().lower()
        own_goal_color = "blue" if enemy_goal_color == "yellow" else "yellow"

        all_detections: list[DetectedObject] = []
        goals_by_color: dict[str, list[GoalDetectionResult]] = {"yellow": [], "blue": []}
        ball_candidates: list[vision.CameraBallData] = []
        ball_possessions: list[bool] = []

        for camera_name, camera_yaw, bgr_frame in frame_entries:
            if DETECTION_FRAME_SIZE_SCALE < 1.0:
                hsv_frame = cv2.resize(bgr_frame, (DETECTION_FRAME_WIDTH, DETECTION_FRAME_HEIGHT))
            else:
                hsv_frame = bgr_frame
            hsv_frame = cv2.cvtColor(hsv_frame, cv2.COLOR_BGR2HSV)
            focal_length = shared_data.get_goal_focal_length(camera_name)
            yellow_ranges = shared_data.get_goal_calibration("yellow", camera_name)
            blue_ranges = shared_data.get_goal_calibration("blue", camera_name)
            goal_calibration = GoalColorCalibration(
                yellow_ranges=[(np.array(lower), np.array(upper)) for lower, upper in yellow_ranges],
                blue_ranges=[(np.array(lower), np.array(upper)) for lower, upper in blue_ranges],
            )
            ball_ranges = shared_data.get_ball_calibration(camera_name)
            ball_lower_arrays = [np.array(lower) for lower, _ in ball_ranges]
            ball_upper_arrays = [np.array(upper) for _, upper in ball_ranges]
            ball_calibration_constant = shared_data.get_camera_ball_calibration_constant(camera_name)

            for goal_color in ("yellow", "blue"):
                result, goal_detections = vision.detect_goal_alignment_with_rect(
                    hsv_frame,
                    goal_color,
                    goal_calibration,
                    focal_length_pixels=focal_length,
                    real_goal_height_mm=GOAL_HEIGHT_MM,
                )
                result.camera_yaw_deg = camera_yaw
                goals_by_color[goal_color].append(result)
                for det in goal_detections:
                    det.camera = camera_name
                all_detections.extend(goal_detections)
                if goal_color == enemy_goal_color:
                    calibration.update_goal_distance_calibration(result, camera_name)

            ball_detections, _ = vision.detect_ball(hsv_frame, ball_lower_arrays, ball_upper_arrays)
            if ball_detections:
                ball_data = vision.calculate_ball_data(
                    ball_detections,
                    DETECTION_FRAME_WIDTH,
                    CAMERA_FOV_DEG,
                    ball_calibration_constant,
                )
                ball_data.angle = utils.normalize_angle_deg(ball_data.angle + camera_yaw)
                ball_candidates.append(ball_data)
                shared_data.set_camera_ball_data_for_camera(camera_name, ball_data)
                for det in ball_detections:
                    det.camera = camera_name
                all_detections.extend(ball_detections)
            else:
                shared_data.set_camera_ball_data_for_camera(camera_name, vision.CameraBallData(999.0, 999.0, False, 0.0))

            ball_center_x = ball_detections[0].x + ball_detections[0].width / 2 if ball_detections else None
            ball_center_y = ball_detections[0].y + ball_detections[0].height / 2 if ball_detections else None
            if camera_name in BALL_POSSESSION_CAMERAS:
                ball_possessed, possession_area = vision.detect_ball_possession(
                    ball_center_x, ball_center_y,
                    DETECTION_FRAME_WIDTH, DETECTION_FRAME_HEIGHT,
                    AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT,
                    AUTO_BALL_POSSESSION_AREA_HEIGHT_PERCENT,
                )
                ball_possessions.append(ball_possessed)

                possession_area_color = (0, 255, 0) if ball_possessed else (0, 0, 255)
                all_detections.append(DetectedObject(
                    object_type="ball_possession_area",
                    x=possession_area.x,
                    y=possession_area.y,
                    width=possession_area.width,
                    height=possession_area.height,
                    color=possession_area_color,
                    camera=camera_name,
                ))

        yellow_result = _fuse_goal_results(goals_by_color["yellow"])
        blue_result = _fuse_goal_results(goals_by_color["blue"])
        shared_data.set_goal_detection_result_for_color("yellow", yellow_result)
        shared_data.set_goal_detection_result_for_color("blue", blue_result)

        enemy_result = yellow_result if enemy_goal_color == "yellow" else blue_result
        own_result = blue_result if own_goal_color == "blue" else yellow_result
        shared_data.set_goal_detection_result(enemy_result)

        hardware = shared_data.get_hardware_data()
        ir_ball_angle = None
        ir_ball_detected = False
        if hardware is not None and hardware.ir.angle is not None and hardware.ir.distance is not None:
            ir_ball_angle = utils.normalize_angle_deg(hardware.ir.angle)
            ir_ball_detected = ir_ball_angle != 999 and hardware.ir.distance != 0

        fused_ball_data = _fuse_ball_data(ball_candidates, ir_ball_angle, ir_ball_detected)
        shared_data.set_camera_ball_data(fused_ball_data)
        shared_data.set_camera_ball_possession(any(ball_possessions))

        if shared_data.get_camera_ball_possession():
            for det in all_detections:
                if det.object_type == "ball":
                    det.color = (0, 255, 0)

        for det in all_detections:
            det.x = int(det.x / DETECTION_FRAME_SIZE_SCALE)
            det.y = int(det.y / DETECTION_FRAME_SIZE_SCALE)
            det.width = int(det.width / DETECTION_FRAME_SIZE_SCALE)
            det.height = int(det.height / DETECTION_FRAME_SIZE_SCALE)

        shared_data.set_detected_objects(all_detections)

        frames_processed += 1
        if time.perf_counter() > last_debug_msg_time + 1:
            logger.debug(
                f"Camera Processing FPS: {frames_processed} "
                f"(enemy_goal={enemy_result.detected}, own_goal={own_result.detected}, ball={fused_ball_data.detected})"
            )
            frames_processed = 0
            last_debug_msg_time = time.perf_counter()
