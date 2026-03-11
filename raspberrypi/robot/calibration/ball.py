import math

import robot.multiprocessing.shared_data as shared_data
import robot.utils as utils
from robot.calibration.data_manager import save_calibration_data



logger = utils.get_logger("Ball Calibration")


def set_ball_color_range(lower_hsv: tuple, upper_hsv: tuple) -> None:
    shared_data.set_ball_calibration(list(lower_hsv), list(upper_hsv))
    save_calibration_data()
    logger.info(f"Ball calibration set: lower={lower_hsv} upper={upper_hsv}")

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

