import math

from robot import utils
from robot.multiprocessing import shared_data
from robot.calibration.data_manager import save_calibration_data



logger = utils.get_logger("Ball Calibration")


def set_ball_color_range(lower_hsv: tuple, upper_hsv: tuple) -> None:
    """Set a single ball color range (replaces all existing ranges with a single one)."""
    shared_data.set_ball_calibration([(list(lower_hsv), list(upper_hsv))])
    save_calibration_data()
    logger.info(f"Ball calibration set: lower={lower_hsv} upper={upper_hsv}")

def add_ball_color_range(lower_hsv: tuple, upper_hsv: tuple) -> None:
    """Add an additional ball color range to the existing ranges."""
    existing_ranges = shared_data.get_ball_calibration()
    new_ranges = existing_ranges + [(list(lower_hsv), list(upper_hsv))]
    shared_data.set_ball_calibration(new_ranges)
    save_calibration_data()
    logger.info(f"Ball color range added: lower={lower_hsv} upper={upper_hsv}")

def set_ball_color_ranges(ranges: list[tuple[tuple, tuple]]) -> None:
    """Set multiple ball color ranges at once.
    
    Args:
        ranges: List of tuples, each tuple is (lower_hsv, upper_hsv)
    """
    formatted_ranges = [(list(lower), list(upper)) for lower, upper in ranges]
    shared_data.set_ball_calibration(formatted_ranges)
    save_calibration_data()
    logger.info(f"Ball calibration set with {len(ranges)} ranges")

def get_ball_color_ranges() -> list[tuple[tuple, tuple]]:
    """Get all ball color ranges."""
    ranges = shared_data.get_ball_calibration()
    return [(tuple(lower), tuple(upper)) for lower, upper in ranges]

def remove_ball_color_range(index: int) -> bool:
    """Remove a ball color range by index.
    
    Returns:
        True if range was removed, False if index was invalid
    """
    ranges = shared_data.get_ball_calibration()
    if 0 <= index < len(ranges):
        ranges.pop(index)
        shared_data.set_ball_calibration(ranges)
        save_calibration_data()
        logger.info(f"Ball color range {index} removed")
        return True
    return False

def calibrate_ball_distance(current_ball_distance_mm: float) -> float:
    """
    Calibrate the distance to the ball based on the area of the detected ball
    in pixels from the current camera frame and then save the calibration data.
    """
    ball_data = shared_data.get_camera_ball_data()
    if not ball_data or not ball_data.detected or ball_data.area_pixels <= 0:
        raise RuntimeError('No ball detected by camera')
    calibration_constant = current_ball_distance_mm * math.sqrt(ball_data.area_pixels)
    shared_data.set_camera_ball_calibration_constant(calibration_constant)
    save_calibration_data()
    return calibration_constant

