import time
import numpy as np
import cv2
from dataclasses import dataclass, field
from typing import List
from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]

from config import FRAME_WIDTH, FRAME_HEIGHT, CAMERA_BUFFER_COUNT
from helpers.detection_visualizer import DetectedObject


@dataclass
class FrameData:
    frame: np.ndarray
    timestamp: float


@dataclass
class GoalColorCalibration:
    """HSV color ranges for goal detection"""
    yellow_lower: np.ndarray = field(default_factory=lambda: np.array([20, 100, 100]))
    yellow_upper: np.ndarray = field(default_factory=lambda: np.array([30, 255, 255]))
    
    blue_lower: np.ndarray = field(default_factory=lambda: np.array([100, 100, 100]))
    blue_upper: np.ndarray = field(default_factory=lambda: np.array([130, 255, 255]))


@dataclass
class BallPossessionArea:
    """Represents the ball possession detection area and status."""
    x: int                  # Top-left x coordinate
    y: int                  # Top-left y coordinate
    width: int              # Area width in pixels
    height: int             # Area height in pixels
    possessed: bool         # Whether ball is possessed (enough orange pixels detected)
    orange_ratio: float     # Ratio of orange pixels in area (0.0 to 1.0)


@dataclass
class GoalDetectionResult:
    alignment: float            # -1.0 (too far left) to 1.0 (too far right), 0.0 is centered
    goal_detected: bool
    goal_center_x: int | None   # X coordinate of goal center in frame
    goal_area: float            # Area of detected goal in pixels
    distance_mm: float | None   # Distance to goal in millimeters
    goal_height_pixels: float   # Height of detected goal in pixels


picam = None


def init_camera():
    """Must be called from within the process that will use it."""
    if Picamera2 is None:
        raise ImportError("Picamera2 library not found.")
    global picam
    picam = Picamera2()
    camera_config = picam.create_preview_configuration(
        main = {"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)},
        # buffer_count = CAMERA_BUFFER_COUNT,
        controls={
            "FrameDurationLimits": (8333, 8333),  # 120 fps
            "ExposureTime": 8000
        }
    )
    picam.configure(camera_config)
    picam.start()



def detect_ball_position(
    frame: np.ndarray,
    ball_lower: np.ndarray,
    ball_upper: np.ndarray,
    min_area: int = 50
) -> tuple[List[DetectedObject], bool]:
    """
    Detect the ball's position and return bounding rectangles.
    
    Args:
        frame: RGB image array.
        ball_lower: HSV lower bound for the ball color.
        ball_upper: HSV upper bound for the ball color.
        min_area: Minimum area to consider as a valid ball detection.
    
    Returns:
        Tuple of (list of DetectedObject instances, confidence_score)
    """
    if frame is None or frame.size == 0:
        return [], False
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, ball_lower, ball_upper)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    if contours:
        # Find the largest contour (most likely to be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area >= min_area:
            x, y, w, h = cv2.boundingRect(largest_contour)
            confidence = min(1.0, area / 5000.0)
            detections.append(DetectedObject(
                object_type="ball",
                x=x,
                y=y,
                width=w,
                height=h,
                confidence=confidence
            ))
    
    return detections, len(detections) > 0


def detect_ball_possession(
    frame: np.ndarray,
    ball_lower: np.ndarray,
    ball_upper: np.ndarray,
    area_width_percent: float = 40.0,
    area_height_percent: float = 25.0,
    min_orange_ratio: float = 0.03
) -> tuple[bool, 'BallPossessionArea']:
    """
    Check if the robot has the ball by looking for ball-colored pixels
    in the possession area (centered horizontally, bottom of frame).

    Args:
        frame: RGB image array.
        ball_lower: HSV lower bound for the ball color.
        ball_upper: HSV upper bound for the ball color.
        area_width_percent: Width of possession area as % of frame width (centered).
        area_height_percent: Height of possession area as % of frame height (from bottom).
        min_orange_ratio: Minimum fraction of pixels that must match to
                          consider the ball possessed.

    Returns:
        Tuple of (ball_possessed: bool, possession_area: BallPossessionArea)
    """
    if frame is None or frame.size == 0:
        return False, BallPossessionArea(x=0, y=0, width=0, height=0, possessed=False, orange_ratio=0.0)

    frame_height, frame_width = frame.shape[:2]
    
    # Calculate possession area dimensions
    area_width = int(frame_width * (area_width_percent / 100.0))
    area_height = int(frame_height * (area_height_percent / 100.0))
    
    # Position: centered horizontally, at bottom of frame
    area_x = (frame_width - area_width) // 2
    area_y = frame_height - area_height
    
    # Extract possession area
    possession_region = frame[area_y:frame_height, area_x:area_x + area_width]
    
    if possession_region.size == 0:
        return False, BallPossessionArea(
            x=area_x, y=area_y, width=area_width, height=area_height,
            possessed=False, orange_ratio=0.0
        )
    
    # Convert to HSV and find orange pixels
    hsv_region = cv2.cvtColor(possession_region, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_region, ball_lower, ball_upper)
    
    # Calculate ratio of orange pixels
    orange_ratio = float(np.count_nonzero(mask)) / mask.size
    possessed = orange_ratio >= min_orange_ratio
    
    return possessed, BallPossessionArea(
        x=area_x, y=area_y, width=area_width, height=area_height,
        possessed=possessed, orange_ratio=orange_ratio
    )


def get_frame() -> FrameData:
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    return FrameData(frame=picam.capture_array(), timestamp=time.time())


def detect_goal_alignment_with_rect(
    frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500,
    focal_length_pixels: float | None = None,
    real_goal_height_mm: float = 100.0
) -> tuple[GoalDetectionResult, List[DetectedObject]]:
    """
    Detect goal alignment and return both the result and detection rectangles.

    Returns:
        Tuple of (GoalDetectionResult, List[DetectedObject])
    """
    result = _detect_goal_alignment_internal(
        frame, goal_color, calibration, min_area, focal_length_pixels, real_goal_height_mm
    )
    
    detections = []
    if result.goal_detected and result.goal_center_x is not None:
        x, y, w, h = _get_goal_bounding_rect(frame, goal_color, calibration, min_area)
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
    frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500,
    focal_length_pixels: float | None = None,
    real_goal_height_mm: float = 100.0
) -> GoalDetectionResult:
    if calibration is None:
        calibration = GoalColorCalibration()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    
    if goal_color.lower() == "yellow":
        lower = calibration.yellow_lower
        upper = calibration.yellow_upper
    elif goal_color.lower() == "blue":
        lower = calibration.blue_lower
        upper = calibration.blue_upper
    else:
        raise ValueError(f"Invalid goal_color: {goal_color}. Must be 'yellow' or 'blue'")
    
    mask = cv2.inRange(hsv, lower, upper)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return GoalDetectionResult(
            alignment=0.0,
            goal_detected=False,
            goal_center_x=None,
            goal_area=0.0,
            distance_mm=None,
            goal_height_pixels=0.0
        )
    
    largest_contour = max(contours, key=cv2.contourArea)
    goal_area = cv2.contourArea(largest_contour)
    
    if goal_area < min_area:
        return GoalDetectionResult(
            alignment=0.0,
            goal_detected=False,
            goal_center_x=None,
            goal_area=goal_area,
            distance_mm=None,
            goal_height_pixels=0.0
        )
    
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return GoalDetectionResult(
            alignment=0.0,
            goal_detected=False,
            goal_center_x=None,
            goal_area=goal_area,
            distance_mm=None,
            goal_height_pixels=0.0
        )
    
    goal_center_x = int(M["m10"] / M["m00"])
    
    _, _, _, goal_height_pixels = cv2.boundingRect(largest_contour)
    
    distance_mm = None
    if focal_length_pixels is not None and goal_height_pixels > 0:
        distance_mm = (real_goal_height_mm * focal_length_pixels) / goal_height_pixels
    
    frame_center_x = frame.shape[1] // 2
    max_offset = frame.shape[1] // 2
    alignment = (goal_center_x - frame_center_x) / max_offset
    
    alignment = max(-1.0, min(1.0, alignment))
    
    return GoalDetectionResult(
        alignment=alignment,
        goal_detected=True,
        goal_center_x=goal_center_x,
        goal_area=goal_area,
        distance_mm=distance_mm,
        goal_height_pixels=float(goal_height_pixels)
    )


def _get_goal_bounding_rect(
    frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500
) -> tuple[int, int, int, int]:
    """
    Get the bounding rectangle for a detected goal.

    Returns:
        Tuple of (x, y, width, height) or (0, 0, 0, 0) if not detected
    """
    if calibration is None:
        calibration = GoalColorCalibration()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    
    if goal_color.lower() == "yellow":
        lower = calibration.yellow_lower
        upper = calibration.yellow_upper
    elif goal_color.lower() == "blue":
        lower = calibration.blue_lower
        upper = calibration.blue_upper
    else:
        return 0, 0, 0, 0
    
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return 0, 0, 0, 0
    
    largest_contour = max(contours, key=cv2.contourArea)
    goal_area = cv2.contourArea(largest_contour)
    
    if goal_area < min_area:
        return 0, 0, 0, 0
    
    x, y, w, h = cv2.boundingRect(largest_contour)
    return x, y, w, h


def detect_goal_alignment(
    frame: np.ndarray,
    goal_color: str = "yellow",
    calibration: GoalColorCalibration | None = None,
    min_area: int = 500,
    focal_length_pixels: float | None = None,
    real_goal_height_mm: float = 100.0
) -> GoalDetectionResult:
    """Legacy function - use detect_goal_alignment_with_rect for visualizations."""
    return _detect_goal_alignment_internal(frame, goal_color, calibration, min_area, focal_length_pixels, real_goal_height_mm)
