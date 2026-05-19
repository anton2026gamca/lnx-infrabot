from robot import utils
from robot.calibration.data_manager import save_calibration_data
from robot.hardware import line_sensors
from robot.multiprocessing import shared_data

from robot.vision import GoalDetectionResult
from robot.config import *




logger = utils.get_logger("Goal Calibration")


def set_enemy_goal_color(color: str) -> None:
    if color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {color}. Must be 'yellow' or 'blue'.")
        return
    shared_data.set_goal_color(color.lower())
    save_calibration_data()
    logger.info(f"Enemy goal color set to: {color.lower()}")

def add_goal_color_range(goal_color: str, lower_hsv: tuple[int, int, int], upper_hsv: tuple[int, int, int], camera: str = "both") -> None:
    """Add an additional goal color range to the existing ranges."""
    if goal_color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {goal_color}. Must be 'yellow' or 'blue'.")
        return
    existing_ranges = shared_data.get_goal_calibration(goal_color.lower(), camera)
    new_ranges = existing_ranges + [(lower_hsv, upper_hsv)]
    shared_data.set_goal_calibration(goal_color.lower(), new_ranges, camera=camera)
    save_calibration_data()
    logger.info(f"Added {goal_color.lower()} goal color range ({camera}): lower={lower_hsv}, upper={upper_hsv}")

def set_goal_color_ranges(goal_color: str, ranges: list[tuple[tuple[int, int, int], tuple[int, int, int]]], camera: str = "both") -> None:
    """Set multiple goal color ranges at once.
    
    Args:
        goal_color: 'yellow' or 'blue'
        ranges: List of tuples, each tuple is (lower_hsv, upper_hsv)
    """
    if goal_color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {goal_color}. Must be 'yellow' or 'blue'.")
        return
    shared_data.set_goal_calibration(goal_color.lower(), ranges, camera=camera)
    save_calibration_data()
    logger.info(f"Set {len(ranges)} ranges for {goal_color.lower()} goal ({camera})")

def get_goal_color_ranges(goal_color: str, camera: str = "front") -> list[tuple[tuple[int, int, int], tuple[int, int, int]]]:
    """Get all goal color ranges for a specific color."""
    if goal_color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {goal_color}. Must be 'yellow' or 'blue'.")
        return []
    ranges = shared_data.get_goal_calibration(goal_color.lower(), camera)
    return ranges if ranges is not None else []

def remove_goal_color_range(goal_color: str, index: int, camera: str = "both") -> bool:
    """Remove a goal color range by index.
    
    Returns:
        True if range was removed, False if index was invalid
    """
    if goal_color.lower() not in ['yellow', 'blue']:
        logger.error(f"Invalid goal color: {goal_color}. Must be 'yellow' or 'blue'.")
        return False
    ranges = shared_data.get_goal_calibration(goal_color.lower(), camera)
    if 0 <= index < len(ranges):
        ranges.pop(index)
        shared_data.set_goal_calibration(goal_color.lower(), ranges, camera=camera)
        save_calibration_data()
        logger.info(f"Removed range {index} from {goal_color.lower()} goal ({camera})")
        return True
    return False

def set_goal_focal_length(focal_length_pixels: float, camera: str = "both") -> None:
    if focal_length_pixels <= 0:
        logger.error(f"Invalid focal length: {focal_length_pixels}. Must be a positive number.")
        return
    shared_data.set_goal_focal_length(focal_length_pixels, camera=camera)
    save_calibration_data()
    logger.info(f"Goal focal length ({camera}) set to: {focal_length_pixels:.2f} pixels")

def start_goal_distance_calibration(initial_distance_mm: float, line_distance_mm: float, camera: str = "front") -> None:
    with shared_data.goal_distance_calibration_lock:
        shared_data.goal_distance_calibration_active.value = True
        shared_data.goal_distance_calibration_data.clear()
        shared_data.goal_distance_calibration_data['initial_distance_mm'] = initial_distance_mm
        shared_data.goal_distance_calibration_data['line_distance_mm'] = line_distance_mm
        shared_data.goal_distance_calibration_data['initial_height_pixels'] = None
        shared_data.goal_distance_calibration_data['line_height_pixels'] = None
        shared_data.goal_distance_calibration_data['phase'] = 'initial'  # 'initial' or 'driving'
        shared_data.goal_distance_calibration_data['camera'] = camera
    logger.info(f"Started goal distance calibration ({camera}): initial={initial_distance_mm}mm, line={line_distance_mm}mm")

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
        
        camera = data.get('camera', 'front')
        shared_data.set_goal_focal_length(focal_length, camera=camera)
        shared_data.goal_distance_calibration_active.value = False
        
        save_calibration_data()
        logger.info(f"Goal distance calibration completed: focal_length={focal_length:.2f} pixels")
        
        return {
            'success': True,
            'focal_length_pixels': focal_length,
            'initial_height_pixels': initial_height,
            'line_height_pixels': line_height,
            'calculated_focal1': focal1,
            'calculated_focal2': focal2,
            'camera': camera,
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
            'line_height_pixels': data.get('line_height_pixels'),
            'camera': data.get('camera', 'front'),
        }

def update_goal_distance_calibration(goal_result: GoalDetectionResult, camera: str) -> None:
    with shared_data.goal_distance_calibration_lock:
        if not shared_data.goal_distance_calibration_active.value:
            return
        if shared_data.goal_distance_calibration_data.get('camera', 'front') != camera:
            return
        
        phase = shared_data.goal_distance_calibration_data.get('phase')
        initial_height = shared_data.goal_distance_calibration_data.get('initial_height_pixels')
        
        if phase == 'initial' and goal_result.detected and goal_result.height_pixels > 0:
            shared_data.goal_distance_calibration_data['initial_height_pixels'] = goal_result.height_pixels
            logger.debug(f"Recording initial goal height: {goal_result.height_pixels:.2f} pixels")
            initial_height = goal_result.height_pixels
        
        if phase == 'initial' and initial_height is not None and initial_height > 0:
            shared_data.goal_distance_calibration_data['phase'] = 'driving'
            logger.info(f"Initial goal height recorded: {initial_height:.2f} pixels. Now drive toward the goal.")
        
        if phase == 'driving':
            line_detected = line_sensors.get_line_detected()
            if any(line_detected) and goal_result.detected and goal_result.height_pixels > 0:
                shared_data.goal_distance_calibration_data['line_height_pixels'] = goal_result.height_pixels
                logger.info(f"Line detected! Recorded goal height: {goal_result.height_pixels:.2f} pixels.")
