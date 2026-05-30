from __future__ import annotations

import cv2
import math
import numpy as np
from dataclasses import dataclass

from robot.profiling import profile_function
from robot.vision.color_mask import mask_from_ranges
from robot.vision.visualizer import DetectedObject


MORPH_OPEN_KERNEL_3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))



@dataclass(slots=True)
class CameraBallData:
    angle: float
    distance: float
    detected: bool
    area_pixels: float

@dataclass(slots=True)
class BallPossessionArea:
    """Represents the ball possession detection area and status."""
    x: int                  # Top-left x coordinate
    y: int                  # Top-left y coordinate
    width: int              # Area width in pixels
    height: int             # Area height in pixels
    possessed: bool         # Whether ball is possessed (enough orange pixels detected)

@profile_function
def detect_ball(
    hsv_frame: np.ndarray,
    ball_ranges: list[tuple[np.ndarray, np.ndarray]],
    min_area: int = 50,
    range_lut: np.ndarray | None = None,
) -> tuple[list[DetectedObject], bool]:
    """
    Detect the ball and return bounding rectangles.
    
    Args:
        hsv_frame: HSV image array.
        ball_ranges: HSV lower/upper bounds for the ball color.
        min_area: Minimum area to consider as a valid ball detection.
        range_lut: Optional HSV lookup table for fast multi-range masking.
    
    Returns:
        Tuple of (list of DetectedObject instances, confidence_score)
    """
    if hsv_frame is None or hsv_frame.size == 0:
        return [], False

    mask = mask_from_ranges(hsv_frame, ball_ranges, range_lut)
    if mask is None:
        return [], False

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_OPEN_KERNEL_3, iterations=1)
    num_labels, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return [], False

    component_stats = stats[1:]
    largest_idx_rel = int(np.argmax(component_stats[:, cv2.CC_STAT_AREA]))
    largest_area = int(component_stats[largest_idx_rel, cv2.CC_STAT_AREA])
    if largest_area < min_area:
        return [], False

    largest_idx = largest_idx_rel + 1
    x = int(stats[largest_idx, cv2.CC_STAT_LEFT])
    y = int(stats[largest_idx, cv2.CC_STAT_TOP])
    w = int(stats[largest_idx, cv2.CC_STAT_WIDTH])
    h = int(stats[largest_idx, cv2.CC_STAT_HEIGHT])
    confidence = min(1.0, largest_area / 5000.0)
    detection = DetectedObject(
        object_type="ball",
        x=x,
        y=y,
        width=w,
        height=h,
        confidence=confidence,
    )
    return [detection], True

@profile_function
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

@profile_function
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
