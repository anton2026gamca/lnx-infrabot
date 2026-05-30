import cv2
import math
import numpy as np
from dataclasses import dataclass, field

from robot.multiprocessing import shared_data
from robot.profiling import profile_function
from robot.vision.color_mask import mask_from_ranges
from robot.vision.visualizer import DetectedObject
from robot.config import *


MORPH_OPEN_KERNEL_3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))



@dataclass(slots=True)
class GoalColorCalibration:
    """HSV color ranges for goal detection - supports multiple ranges per color"""
    yellow_ranges: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=lambda: [(np.array([20, 100, 100]), np.array([30, 255, 255]))])
    blue_ranges: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=lambda: [(np.array([100, 100, 100]), np.array([130, 255, 255]))])
    yellow_lut: np.ndarray | None = field(default=None, repr=False)
    blue_lut: np.ndarray | None = field(default=None, repr=False)

@dataclass(slots=True)
class GoalDetectionResult:
    alignment: float            # -1.0 (too far left) to 1.0 (too far right), 0.0 is centered
    detected: bool
    center_x: int | None        # X coordinate of goal center in frame
    area: float                 # Area of detected goal in pixels
    distance_mm: float | None   # Distance to goal in millimeters
    height_pixels: float        # Height of detected goal in pixels
    camera_yaw_deg: float = 0.0 # 0=front camera, 180=back camera
    _rect: tuple[int, int, int, int, float] | None = None  # Cached bounding rect (x, y, w, h) for visualization

@dataclass(slots=True)
class PositionEstimate:
    x_mm: float
    y_mm: float
    confidence: float


@profile_function
def detect_goal_alignment_with_rect(
    hsv_frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500,
    focal_length_pixels: float | None = None,
    real_goal_height_mm: float = 100.0
) -> tuple[GoalDetectionResult, list[DetectedObject]]:
    """Detect goal alignment and return both the result and detection rectangles."""
    result = _detect_goal_alignment_internal(
        hsv_frame, goal_color, calibration, min_area, focal_length_pixels, real_goal_height_mm
    )
    
    detections = []
    if result.detected and result.center_x is not None:
        rect_data = result._rect if result._rect is not None else _get_goal_bounding_rect(hsv_frame, goal_color, calibration, min_area)
        x, y, w, h, goal_height = rect_data
        if w > 0 and h > 0:
            object_type = f"goal_{goal_color.lower()}"
            detections.append(DetectedObject(
                object_type=object_type,
                x=x,
                y=y,
                width=w,
                height=h,
                confidence=min(1.0, result.area / 50000.0)
            ))
    
    return result, detections


@profile_function
def _detect_goal_alignment_internal(
    hsv_frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500,
    focal_length_pixels: float | None = None,
    real_goal_height_mm: float = 100.0
) -> GoalDetectionResult:
    if calibration is None:
        calibration = GoalColorCalibration()

    x, y, w, h, goal_height = _get_goal_bounding_rect(
        hsv_frame=hsv_frame,
        goal_color=goal_color,
        calibration=calibration,
        min_area=min_area
    )

    if (x, y, w, h, goal_height) == (0, 0, 0, 0, 0.0):
        return GoalDetectionResult(
            alignment=0.0,
            detected=False,
            center_x=None,
            area=0.0,
            distance_mm=None,
            height_pixels=0.0
        )

    goal_area = w * h

    goal_center_x = x + w // 2
    goal_height_pixels = goal_height

    distance_mm = None
    if focal_length_pixels is not None and goal_height_pixels > 0:
        distance_mm = (real_goal_height_mm * focal_length_pixels) / goal_height_pixels

    frame_center_x = hsv_frame.shape[1] // 2
    max_offset = hsv_frame.shape[1] // 2
    alignment = (goal_center_x - frame_center_x) / max_offset

    alignment = max(-1.0, min(1.0, alignment))

    result = GoalDetectionResult(
        alignment=alignment,
        detected=True,
        center_x=goal_center_x,
        area=goal_area,
        distance_mm=distance_mm,
        height_pixels=float(goal_height_pixels)
    )
    result._rect = (x, y, w, h, goal_height)
    return result


@profile_function
def _get_goal_bounding_rect(
    hsv_frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500
) -> tuple[int, int, int, int, float]:
    """
    Get the bounding rectangle for a detected goal.
    NOTE: This function is typically called from detect_goal_alignment_with_rect
    which caches the result. Use the cached value from GoalDetectionResult._rect when available.

    Returns:
        Tuple of (x, y, width, height, goal_height) or (0, 0, 0, 0, 0.0) if not detected
        where goal_height is the corrected height from minAreaRect
    """
    if calibration is None:
        calibration = GoalColorCalibration()
    
    if goal_color.lower() == "yellow":
        ranges = calibration.yellow_ranges
        lut = calibration.yellow_lut
    elif goal_color.lower() == "blue":
        ranges = calibration.blue_ranges
        lut = calibration.blue_lut
    else:
        return 0, 0, 0, 0, 0.0
    
    if not ranges:
        return 0, 0, 0, 0, 0.0

    mask = mask_from_ranges(hsv_frame, ranges, lut)
    if mask is None:
        return 0, 0, 0, 0, 0.0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_OPEN_KERNEL_3, iterations=1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return 0, 0, 0, 0, 0.0
    
    largest_contour = None
    largest_area = 0.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area:
            largest_area = area
            largest_contour = contour
    
    if largest_area < min_area or largest_contour is None:
        return 0, 0, 0, 0, 0.0
    
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    rect = cv2.minAreaRect(largest_contour)
    center, (width, height), angle = rect
    
    if width < height:
        width, height = height, width
    
    goal_height = float(int(round(height)))
    
    return x, y, w, h, goal_height


@profile_function
def get_position_estimate() -> PositionEstimate | None:
    hardware_data = shared_data.get_hardware_compass_ir()
    heading = hardware_data[0]

    if heading == 999.0:
        return None

    field_length_mm = 2190.0
    enemy_goal_color = shared_data.get_goal_color().lower()
    own_goal_color = "blue" if enemy_goal_color == "yellow" else "yellow"
    enemy_goal = shared_data.get_goal_detection_result_for_color(enemy_goal_color)
    own_goal = shared_data.get_goal_detection_result_for_color(own_goal_color)

    def _candidate_from_goal(goal: GoalDetectionResult | None, goal_x: float, goal_y: float) -> tuple[float, float, float] | None:
        if goal is None or not goal.detected or goal.distance_mm is None:
            return None
        distance_mm = goal.distance_mm
        angle_to_goal_deg = (
            heading
            + goal.camera_yaw_deg
            + goal.alignment * (CAMERA_FOV_DEG / 2.0)
        )
        angle_to_goal_rad = math.radians(angle_to_goal_deg)
        x_mm = goal_x - distance_mm * math.sin(angle_to_goal_rad)
        y_mm = goal_y + distance_mm * math.cos(angle_to_goal_rad)

        area_confidence = min(1.0, goal.area / 50000.0)
        alignment_confidence = 1.0 - abs(goal.alignment)
        distance_confidence = max(0.05, min(1.0, 2000.0 / max(distance_mm, 1.0)))
        confidence = (area_confidence * 0.45) + (alignment_confidence * 0.25) + (distance_confidence * 0.30)
        weight = confidence * (1.0 / max(distance_mm, 1.0))
        return x_mm, y_mm, weight

    candidates = []
    enemy_candidate = _candidate_from_goal(enemy_goal, goal_x=0.0, goal_y=0.0)
    own_candidate = _candidate_from_goal(own_goal, goal_x=0.0, goal_y=field_length_mm)
    if enemy_candidate is not None:
        candidates.append(enemy_candidate)
    if own_candidate is not None:
        candidates.append(own_candidate)

    if not candidates:
        return None

    sum_weights = sum(weight for _, _, weight in candidates)
    if sum_weights <= 0:
        return None

    x_mm = sum(x * weight for x, _, weight in candidates) / sum_weights
    y_mm = sum(y * weight for _, y, weight in candidates) / sum_weights
    y_mm = max(0.0, min(field_length_mm, y_mm))

    confidence = min(1.0, sum_weights / len(candidates))
    previous = shared_data.get_last_position_estimate()
    if previous is not None:
        prev_x = float(previous.get("x_mm", x_mm))
        prev_y = float(previous.get("y_mm", y_mm))
        distance_delta = math.sqrt((x_mm - prev_x) ** 2 + (y_mm - prev_y) ** 2)
        smoothing_alpha = 0.62 if distance_delta < 450.0 else 0.36
        x_mm = prev_x + (x_mm - prev_x) * smoothing_alpha
        y_mm = prev_y + (y_mm - prev_y) * smoothing_alpha
        confidence = min(1.0, (confidence * 0.75) + (float(previous.get("confidence", 0.0)) * 0.25))

    shared_data.set_last_position_estimate(x_mm, y_mm, confidence)
    return PositionEstimate(x_mm=x_mm, y_mm=y_mm, confidence=confidence)
