import robot.hardware.line_sensors as line_sensors
import robot.multiprocessing.shared_data as shared_data
import robot.utils as utils
from robot.calibration.data_manager import save_calibration_data
from robot.config import *
from robot.vision import GoalDetectionResult




logger = utils.get_logger("Goal Calibration")


def set_enemy_goal_color(color: str) -> None:
    if color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {color}. Must be 'yellow' or 'blue'.")
        return
    shared_data.set_goal_color(color.lower())
    save_calibration_data()
    logger.info(f"Enemy goal color set to: {color.lower()}")

def set_goal_color_ranges(goal_color: str, lower_hsv: tuple[int, int, int], upper_hsv: tuple[int, int, int]) -> None:
    if goal_color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {goal_color}. Must be 'yellow' or 'blue'.")
        return
    shared_data.set_goal_calibration(goal_color.lower(), list(lower_hsv), list(upper_hsv))
    save_calibration_data()
    logger.info(f"Updated {goal_color.lower()} goal HSV ranges: lower={lower_hsv}, upper={upper_hsv}")

def set_goal_focal_length(focal_length_pixels: float) -> None:
    if focal_length_pixels <= 0:
        logger.error(f"Invalid focal length: {focal_length_pixels}. Must be a positive number.")
        return
    shared_data.set_goal_focal_length(focal_length_pixels)
    save_calibration_data()
    logger.info(f"Goal focal length set to: {focal_length_pixels:.2f} pixels")

def start_goal_distance_calibration(initial_distance_mm: float, line_distance_mm: float) -> None:
    with shared_data.goal_distance_calibration_lock:
        shared_data.goal_distance_calibration_active.value = True
        shared_data.goal_distance_calibration_data.clear()
        shared_data.goal_distance_calibration_data['initial_distance_mm'] = initial_distance_mm
        shared_data.goal_distance_calibration_data['line_distance_mm'] = line_distance_mm
        shared_data.goal_distance_calibration_data['initial_height_pixels'] = None
        shared_data.goal_distance_calibration_data['line_height_pixels'] = None
        shared_data.goal_distance_calibration_data['phase'] = 'initial'  # 'initial' or 'driving'
    logger.info(f"Started goal distance calibration: initial={initial_distance_mm}mm, line={line_distance_mm}mm")

def stop_goal_distance_calibration() -> dict:
    with shared_data.goal_distance_calibration_lock:
        if not shared_data.goal_distance_calibration_active.value:
            return {'success': False, 'error': 'No calibration in progress'}
        
        data = dict(shared_data.goal_distance_calibration_data)
        initial_height = data.get('initial_height_pixels')
        line_height = data.get('line_height_pixels')
        initial_distance = data.get('initial_distance_mm')
        line_distance = data.get('line_distance_mm')
        
        if initial_height is None or line_height is None:
            shared_data.goal_distance_calibration_active.value = False
            return {'success': False, 'error': 'Calibration incomplete: goal not detected at both positions'}
        
        if initial_height <= 0 or line_height <= 0:
            shared_data.goal_distance_calibration_active.value = False
            return {'success': False, 'error': 'Invalid goal height detected'}
        
        focal1 = (initial_height * initial_distance) / GOAL_HEIGHT_MM
        focal2 = (line_height * line_distance) / GOAL_HEIGHT_MM
        focal_length = (focal1 + focal2) / 2.0
        
        shared_data.set_goal_focal_length(focal_length)
        shared_data.goal_distance_calibration_active.value = False
        
        save_calibration_data()
        logger.info(f"Goal distance calibration completed: focal_length={focal_length:.2f} pixels")
        
        return {
            'success': True,
            'focal_length_pixels': focal_length,
            'initial_height_pixels': initial_height,
            'line_height_pixels': line_height,
            'calculated_focal1': focal1,
            'calculated_focal2': focal2
        }

def cancel_goal_distance_calibration() -> None:
    with shared_data.goal_distance_calibration_lock:
        shared_data.goal_distance_calibration_active.value = False
        shared_data.goal_distance_calibration_data.clear()
    logger.info("Goal distance calibration cancelled")

def get_goal_distance_calibration_status() -> dict:
    with shared_data.goal_distance_calibration_lock:
        if not shared_data.goal_distance_calibration_active.value:
            return {'active': False}
        
        data = dict(shared_data.goal_distance_calibration_data)
        return {
            'active': True,
            'phase': data.get('phase', 'initial'),
            'initial_distance_mm': data.get('initial_distance_mm'),
            'line_distance_mm': data.get('line_distance_mm'),
            'initial_height_pixels': data.get('initial_height_pixels'),
            'line_height_pixels': data.get('line_height_pixels')
        }

def update_goal_distance_calibration(goal_result: GoalDetectionResult) -> None:
    with shared_data.goal_distance_calibration_lock:
        if not shared_data.goal_distance_calibration_active.value:
            return
        
        phase = shared_data.goal_distance_calibration_data.get('phase')
        initial_height = shared_data.goal_distance_calibration_data.get('initial_height_pixels')
        
        if phase == 'initial' and goal_result.goal_detected and goal_result.goal_height_pixels > 0:
            shared_data.goal_distance_calibration_data['initial_height_pixels'] = goal_result.goal_height_pixels
            logger.debug(f"Recording initial goal height: {goal_result.goal_height_pixels:.2f} pixels")
            initial_height = goal_result.goal_height_pixels
        
        if phase == 'initial' and initial_height is not None and initial_height > 0:
            shared_data.goal_distance_calibration_data['phase'] = 'driving'
            logger.info(f"Initial goal height recorded: {initial_height:.2f} pixels. Now drive toward the goal.")
        
        if phase == 'driving':
            line_detected = line_sensors.get_line_detected()
            if any(line_detected) and goal_result.goal_detected and goal_result.goal_height_pixels > 0:
                shared_data.goal_distance_calibration_data['line_height_pixels'] = goal_result.goal_height_pixels
                logger.info(f"Line detected! Recorded goal height: {goal_result.goal_height_pixels:.2f} pixels.")

