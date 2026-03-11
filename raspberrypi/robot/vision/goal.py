import cv2
import math
import numpy as np
from dataclasses import dataclass, field

import robot.multiprocessing.shared_data as shared_data
from robot.config import *
from robot.vision.visualizer import DetectedObject



@dataclass
class GoalColorCalibration:
    """HSV color ranges for goal detection"""
    yellow_lower: np.ndarray = field(default_factory=lambda: np.array([20, 100, 100]))
    yellow_upper: np.ndarray = field(default_factory=lambda: np.array([30, 255, 255]))
    
    blue_lower: np.ndarray = field(default_factory=lambda: np.array([100, 100, 100]))
    blue_upper: np.ndarray = field(default_factory=lambda: np.array([130, 255, 255]))

@dataclass
class GoalDetectionResult:
    alignment: float            # -1.0 (too far left) to 1.0 (too far right), 0.0 is centered
    goal_detected: bool
    goal_center_x: int | None   # X coordinate of goal center in frame
    goal_area: float            # Area of detected goal in pixels
    distance_mm: float | None   # Distance to goal in millimeters
    goal_height_pixels: float   # Height of detected goal in pixels
    _rect: tuple[int, int, int, int] | None = None  # Cached bounding rect (x, y, w, h) for visualization

@dataclass
class PositionEstimate:
    x_mm: float
    y_mm: float
    confidence: float


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
    if result.goal_detected and result.goal_center_x is not None:
        x, y, w, h = result._rect if result._rect is not None else _get_goal_bounding_rect(hsv_frame, goal_color, calibration, min_area)
        if w > 0 and h > 0:
            object_type = f"goal_{goal_color.lower()}"
            detections.append(DetectedObject(
                object_type=object_type,
                x=x,
                y=y,
                width=w,
                height=h,
                confidence=min(1.0, result.goal_area / 50000.0)
            ))
    
    return result, detections


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

    x, y, w, h = _get_goal_bounding_rect(
        hsv_frame=hsv_frame,
        goal_color=goal_color,
        calibration=calibration,
        min_area=min_area
    )

    if (x, y, w, h) == (0, 0, 0, 0):
        return GoalDetectionResult(
            alignment=0.0,
            goal_detected=False,
            goal_center_x=None,
            goal_area=0.0,
            distance_mm=None,
            goal_height_pixels=0.0
        )

    goal_area = w * h

    goal_center_x = x + w // 2
    goal_height_pixels = h

    distance_mm = None
    if focal_length_pixels is not None and goal_height_pixels > 0:
        distance_mm = (real_goal_height_mm * focal_length_pixels) / goal_height_pixels

    frame_center_x = hsv_frame.shape[1] // 2
    max_offset = hsv_frame.shape[1] // 2
    alignment = (goal_center_x - frame_center_x) / max_offset

    alignment = max(-1.0, min(1.0, alignment))

    result = GoalDetectionResult(
        alignment=alignment,
        goal_detected=True,
        goal_center_x=goal_center_x,
        goal_area=goal_area,
        distance_mm=distance_mm,
        goal_height_pixels=float(goal_height_pixels)
    )
    result._rect = (x, y, w, h)
    return result


def _get_goal_bounding_rect(
    hsv_frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500
) -> tuple[int, int, int, int]:
    """
    Get the bounding rectangle for a detected goal.
    NOTE: This function is typically called from detect_goal_alignment_with_rect
    which caches the result. Use the cached value from GoalDetectionResult._rect when available.

    Returns:
        Tuple of (x, y, width, height) or (0, 0, 0, 0) if not detected
    """
    if calibration is None:
        calibration = GoalColorCalibration()
    
    if goal_color.lower() == "yellow":
        lower = calibration.yellow_lower
        upper = calibration.yellow_upper
    elif goal_color.lower() == "blue":
        lower = calibration.blue_lower
        upper = calibration.blue_upper
    else:
        return 0, 0, 0, 0
    
    mask = cv2.inRange(hsv_frame, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return 0, 0, 0, 0
    
    largest_contour = max(contours, key=cv2.contourArea)
    goal_area = cv2.contourArea(largest_contour)
    
    if goal_area < min_area:
        return 0, 0, 0, 0
    
    x, y, w, h = cv2.boundingRect(largest_contour)
    return x, y, w, h


def get_position_estimate() -> PositionEstimate | None:
    goal_result = shared_data.get_goal_detection_result()
    hardware_data = shared_data.get_hardware_data()
    
    if not goal_result or not goal_result.goal_detected or goal_result.distance_mm is None:
        return None
    
    if not hardware_data or hardware_data.compass.heading is None:
        return None
    
    distance_mm = goal_result.distance_mm
    alignment = goal_result.alignment
    
    robot_heading_deg = hardware_data.compass.heading
    angle_offset_deg = alignment * (CAMERA_FOV_DEG / 2.0)
    angle_to_goal_deg = robot_heading_deg + angle_offset_deg
    angle_to_goal_rad = math.radians(angle_to_goal_deg)
    
    x_mm = distance_mm * math.sin(angle_to_goal_rad) * -1
    y_mm = distance_mm * math.cos(angle_to_goal_rad)
    
    area_confidence = min(1.0, goal_result.goal_area / 50000.0)
    alignment_confidence = 1.0 - abs(alignment)
    confidence = (area_confidence * 0.6) + (alignment_confidence * 0.4)
    
    return PositionEstimate(x_mm=x_mm, y_mm=y_mm, confidence=confidence)

