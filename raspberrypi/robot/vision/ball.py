import cv2
import math
import numpy as np
from dataclasses import dataclass

from robot.vision.visualizer import DetectedObject



@dataclass
class CameraBallData:
    angle: float
    distance: float
    detected: bool
    area_pixels: float

@dataclass
class BallPossessionArea:
    """Represents the ball possession detection area and status."""
    x: int                  # Top-left x coordinate
    y: int                  # Top-left y coordinate
    width: int              # Area width in pixels
    height: int             # Area height in pixels
    possessed: bool         # Whether ball is possessed (enough orange pixels detected)


def detect_ball(
    hsv_frame: np.ndarray,
    ball_lower: np.ndarray | list[np.ndarray],
    ball_upper: np.ndarray | list[np.ndarray],
    min_area: int = 50
) -> tuple[list[DetectedObject], bool]:
    """
    Detect the ball and return bounding rectangles.
    
    Args:
        hsv_frame: HSV image array.
        ball_lower: HSV lower bound(s) for the ball color. Can be a single array or list of arrays for multiple ranges.
        ball_upper: HSV upper bound(s) for the ball color. Can be a single array or list of arrays for multiple ranges.
        min_area: Minimum area to consider as a valid ball detection.
    
    Returns:
        Tuple of (list of DetectedObject instances, confidence_score)
    """
    if hsv_frame is None or hsv_frame.size == 0:
        return [], False
    
    mask = None
    if isinstance(ball_lower, list):
        for lower, upper in zip(ball_lower, ball_upper):
            range_mask = cv2.inRange(hsv_frame, lower, upper)
            mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)
    elif isinstance(ball_lower, np.ndarray) and isinstance(ball_upper, np.ndarray):
        mask = cv2.inRange(hsv_frame, ball_lower, ball_upper)

    if mask is None:
        return [], False
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    if contours:
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

def calculate_ball_data(ball_detections, frame_width: float, camera_fov: float, calibration_constant: float) -> CameraBallData:
    camera_ball_angle = 999.0
    camera_ball_distance = 0.0
    camera_ball_detected = False
    camera_ball_area = 0.0

    if ball_detections:
        det = ball_detections[0]
        det_center_x = det.x + det.width / 2.0
        frame_center_x = frame_width / 2.0
        pixel_offset = det_center_x - frame_center_x
        angle_fraction = pixel_offset / frame_center_x  # -1.0 to 1.0
        camera_ball_angle = angle_fraction * (camera_fov / 2.0)
        ball_area_pixels = det.width * det.height
        camera_ball_area = ball_area_pixels

        camera_ball_distance = (calibration_constant if calibration_constant is not None else 10000.0) / math.sqrt(max(ball_area_pixels, 1.0))
        camera_ball_detected = True

    return CameraBallData(
        camera_ball_angle,
        camera_ball_distance,
        camera_ball_detected,
        camera_ball_area,
    )

def detect_ball_possession(
    ball_center_x: float | None,
    ball_center_y: float | None,
    frame_width: int,
    frame_height: int,
    area_width_percent: float = 40.0,
    area_height_percent: float = 25.0,
) -> tuple[bool, BallPossessionArea]:
    """
    Detect if the ball is possessed based on its position in the frame.

    Args:
        ball_center_x: X coordinate of the ball center in the frame (or None if not detected)
        ball_center_y: Y coordinate of the ball center in the frame (or None if not detected)
        frame_width: Width of the camera frame
        frame_height: Height of the camera frame
        area_width_percent: Width of the possession area as a percentage of frame width
        area_height_percent: Height of the possession area as a percentage of frame height

    Returns:
        Tuple of (possessed: bool, BallPossessionArea)
    """
    if frame_width <= 0 or frame_height <= 0:
        return False, BallPossessionArea(x=0, y=0, width=0, height=0, possessed=False)

    area_width = int(frame_width * (area_width_percent / 100.0))
    area_height = int(frame_height * (area_height_percent / 100.0))

    area_x = (frame_width - area_width) // 2
    area_y = frame_height - area_height

    if ball_center_x is None or ball_center_y is None:
        return False, BallPossessionArea(
            x=area_x, y=area_y, width=area_width, height=area_height,
            possessed=False
        )

    possessed = (
        area_x <= ball_center_x <= area_x + area_width and
        area_y <= ball_center_y <= area_y + area_height
    )

    return possessed, BallPossessionArea(
        x=area_x, y=area_y, width=area_width, height=area_height,
        possessed=possessed
    )

