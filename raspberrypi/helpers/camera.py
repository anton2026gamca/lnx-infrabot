import time
import numpy as np
import cv2
from dataclasses import dataclass, field

try:
    from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
except ImportError:
    Picamera2 = None

from config import FRAME_WIDTH, FRAME_HEIGHT, CAMERA_BUFFER_COUNT


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
        main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam.configure(camera_config)
    picam.start()



def detect_ball_possession(
    frame: np.ndarray,
    ball_lower: np.ndarray,
    ball_upper: np.ndarray,
    bottom_strip_height: int = 30,
    min_orange_ratio: float = 0.03
) -> bool:
    """
    Check if the robot has the ball by looking for ball-coloured pixels
    in the bottom strip of the camera frame.

    Args:
        frame: RGB image array.
        ball_lower: HSV lower bound for the ball colour.
        ball_upper: HSV upper bound for the ball colour.
        bottom_strip_height: Height (px) of the bottom strip to examine.
        min_orange_ratio: Minimum fraction of pixels that must match to
                          consider the ball possessed.

    Returns:
        True if the ball appears to be in the robot's front pocket.
    """
    if frame is None or frame.size == 0:
        return False

    h = frame.shape[0]
    strip = frame[max(0, h - bottom_strip_height):h, :]

    if strip.size == 0:
        return False

    hsv_strip = cv2.cvtColor(strip, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_strip, ball_lower, ball_upper)

    ratio = float(np.count_nonzero(mask)) / mask.size
    return ratio >= min_orange_ratio


def get_frame() -> FrameData:
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    return FrameData(frame=picam.capture_array(), timestamp=time.time())


def detect_goal_alignment(
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
