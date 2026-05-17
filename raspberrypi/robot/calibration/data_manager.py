import json
import multiprocessing
import os

from robot import utils
from robot.multiprocessing import shared_data

from robot.config import *



CALIBRATION_SCHEMA_VERSION = 1


lock = multiprocessing.Lock()
logger = utils.get_logger("Calibration Data Manager")


def _get_calibration_storage_path() -> str:
    if os.path.isabs(CALIBRATION_FILE_PATH):
        return CALIBRATION_FILE_PATH
    return os.path.join(os.path.dirname(__file__), CALIBRATION_FILE_PATH)

def _create_calibration_data() -> dict:
    thresholds = shared_data.get_line_detection_thresholds()

    goal_color = shared_data.get_goal_color()
    goal_detection_by_camera = {}
    ball_detection_by_camera = {}
    ball_distance_by_camera = {}
    focal_length_by_camera = {}
    for camera in ("front", "back"):
        yellow_ranges = shared_data.get_goal_calibration("yellow", camera)
        blue_ranges = shared_data.get_goal_calibration("blue", camera)
        ball_ranges = shared_data.get_ball_calibration(camera)
        goal_detection_by_camera[camera] = {
            "yellow_ranges": [{'lower': list(lower), 'upper': list(upper)} for lower, upper in yellow_ranges],
            "blue_ranges": [{'lower': list(lower), 'upper': list(upper)} for lower, upper in blue_ranges],
        }
        ball_detection_by_camera[camera] = {
            "ball_ranges": [{'lower': list(lower), 'upper': list(upper)} for lower, upper in ball_ranges],
        }
        ball_distance_by_camera[camera] = {
            "calibration_constant": shared_data.get_camera_ball_calibration_constant(camera),
        }
        focal_length_by_camera[camera] = shared_data.get_goal_focal_length(camera)
    
    return {
        "version": CALIBRATION_SCHEMA_VERSION,
        "calibrations": {
            "line_detection": {
                "thresholds": thresholds,
            },
            "goal_detection": {
                "goal_color": goal_color,
                "yellow_ranges": goal_detection_by_camera["front"]["yellow_ranges"],  # backward compatibility
                "blue_ranges": goal_detection_by_camera["front"]["blue_ranges"],      # backward compatibility
                "focal_length_pixels": focal_length_by_camera["front"],               # backward compatibility
                "by_camera": goal_detection_by_camera,
                "focal_length_pixels_by_camera": focal_length_by_camera,
            },
            "ball_detection": {
                "ball_ranges": ball_detection_by_camera["front"]["ball_ranges"],      # backward compatibility
                "by_camera": ball_detection_by_camera,
            },
            "ball_distance": {
                "calibration_constant": ball_distance_by_camera["front"]["calibration_constant"],  # backward compatibility
                "by_camera": ball_distance_by_camera,
            },
            "bluetooth": {
                "other_robot": shared_data.get_bluetooth_other_robot_info(),
            },
        }
    }

def save_calibration_data() -> None:
    path = _get_calibration_storage_path()
    data = _create_calibration_data()

    with lock:
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            temp_path = f"{path}.tmp"
            with open(temp_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(temp_path, path)
        except Exception as e:
            logger.error(f"Failed to save calibration data: {e}")

def load_calibration_data() -> None:
    path = _get_calibration_storage_path()
    if not os.path.exists(path):
        logger.info("No calibration data file found, using defaults")
        return

    with lock:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            logger.warning(f"Failed to read calibration data: {e}")
            return

    try:
        calibrations = data.get("calibrations", {}) if isinstance(data, dict) else {}
        line_data = calibrations.get("line_detection", {}) if isinstance(calibrations, dict) else {}
        thresholds = line_data.get("thresholds", None)

        if isinstance(thresholds, list) and len(thresholds) == LINE_SENSOR_COUNT:
            if isinstance(thresholds[0], list):
                shared_data.set_line_detection_thresholds([[int(t[0]), int(t[1])] for t in thresholds])
            else:
                shared_data.set_line_detection_thresholds([[int(t), int(t)] for t in thresholds])
        
        goal_data = calibrations.get("goal_detection", {}) if isinstance(calibrations, dict) else {}
        if goal_data:
            goal_color = goal_data.get("goal_color")
            if goal_color in ['yellow', 'blue']:
                with shared_data.goal_detection_lock:
                    color_bytes = goal_color.encode()[:10].ljust(10)
                    for i in range(10):
                        shared_data.goal_color[i] = color_bytes[i:i+1]

            goal_by_camera = goal_data.get("by_camera", {}) if isinstance(goal_data.get("by_camera"), dict) else {}
            focal_by_camera = goal_data.get("focal_length_pixels_by_camera", {}) if isinstance(goal_data.get("focal_length_pixels_by_camera"), dict) else {}
            for camera in ("front", "back"):
                camera_goal = goal_by_camera.get(camera, {}) if isinstance(goal_by_camera.get(camera), dict) else {}
                yellow_ranges = camera_goal.get("yellow_ranges")
                blue_ranges = camera_goal.get("blue_ranges")
                if yellow_ranges and isinstance(yellow_ranges, list):
                    ranges_list = [(r.get('lower', [20, 100, 100]), r.get('upper', [30, 255, 255])) for r in yellow_ranges if isinstance(r, dict)]
                    shared_data.set_goal_calibration('yellow', ranges_list, camera=camera)
                if blue_ranges and isinstance(blue_ranges, list):
                    ranges_list = [(r.get('lower', [100, 100, 100]), r.get('upper', [130, 255, 255])) for r in blue_ranges if isinstance(r, dict)]
                    shared_data.set_goal_calibration('blue', ranges_list, camera=camera)
                focal_length = focal_by_camera.get(camera)
                if focal_length is not None and isinstance(focal_length, (int, float)) and focal_length > 0:
                    shared_data.set_goal_focal_length(float(focal_length), camera=camera)

            if not goal_by_camera:
                # Backward compatibility with single-camera format
                yellow_ranges = goal_data.get("yellow_ranges")
                if yellow_ranges and isinstance(yellow_ranges, list):
                    ranges_list = [(r.get('lower', [20, 100, 100]), r.get('upper', [30, 255, 255])) for r in yellow_ranges if isinstance(r, dict)]
                    shared_data.set_goal_calibration('yellow', ranges_list, camera="both")
                else:
                    yellow_lower = goal_data.get("yellow_lower")
                    yellow_upper = goal_data.get("yellow_upper")
                    if yellow_lower and yellow_upper and len(yellow_lower) == 3 and len(yellow_upper) == 3:
                        shared_data.set_goal_calibration('yellow', [(yellow_lower, yellow_upper)], camera="both")

                blue_ranges = goal_data.get("blue_ranges")
                if blue_ranges and isinstance(blue_ranges, list):
                    ranges_list = [(r.get('lower', [100, 100, 100]), r.get('upper', [130, 255, 255])) for r in blue_ranges if isinstance(r, dict)]
                    shared_data.set_goal_calibration('blue', ranges_list, camera="both")
                else:
                    blue_lower = goal_data.get("blue_lower")
                    blue_upper = goal_data.get("blue_upper")
                    if blue_lower and blue_upper and len(blue_lower) == 3 and len(blue_upper) == 3:
                        shared_data.set_goal_calibration('blue', [(blue_lower, blue_upper)], camera="both")

                focal_length = goal_data.get("focal_length_pixels")
                if focal_length is not None and isinstance(focal_length, (int, float)) and focal_length > 0:
                    shared_data.set_goal_focal_length(float(focal_length), camera="both")

        ball_data = calibrations.get("ball_detection", {}) if isinstance(calibrations, dict) else {}
        if ball_data:
            ball_by_camera = ball_data.get("by_camera", {}) if isinstance(ball_data.get("by_camera"), dict) else {}
            for camera in ("front", "back"):
                camera_ball = ball_by_camera.get(camera, {}) if isinstance(ball_by_camera.get(camera), dict) else {}
                ball_ranges = camera_ball.get("ball_ranges")
                if ball_ranges and isinstance(ball_ranges, list):
                    ranges_list = [(r.get('lower', [0, 0, 0]), r.get('upper', [0, 0, 0])) for r in ball_ranges if isinstance(r, dict)]
                    shared_data.set_ball_calibration(ranges_list, camera=camera)

            if not ball_by_camera:
                ball_ranges = ball_data.get("ball_ranges")
                if ball_ranges and isinstance(ball_ranges, list):
                    ranges_list = [(r.get('lower', [0, 0, 0]), r.get('upper', [0, 0, 0])) for r in ball_ranges if isinstance(r, dict)]
                    shared_data.set_ball_calibration(ranges_list, camera="both")
                else:
                    ball_lower = ball_data.get("ball_lower")
                    ball_upper = ball_data.get("ball_upper")
                    if ball_lower and ball_upper and len(ball_lower) == 3 and len(ball_upper) == 3:
                        shared_data.set_ball_calibration([(ball_lower, ball_upper)], camera="both")

        ball_distance_data = calibrations.get("ball_distance", {}) if isinstance(calibrations, dict) else {}
        if ball_distance_data:
            by_camera = ball_distance_data.get("by_camera", {}) if isinstance(ball_distance_data.get("by_camera"), dict) else {}
            for camera in ("front", "back"):
                camera_data = by_camera.get(camera, {}) if isinstance(by_camera.get(camera), dict) else {}
                calibration_constant = camera_data.get("calibration_constant")
                if calibration_constant is not None and isinstance(calibration_constant, (int, float)) and calibration_constant > 0:
                    shared_data.set_camera_ball_calibration_constant(float(calibration_constant), camera=camera)
            if not by_camera:
                calibration_constant = ball_distance_data.get("calibration_constant")
                if calibration_constant is not None and isinstance(calibration_constant, (int, float)) and calibration_constant > 0:
                    shared_data.set_camera_ball_calibration_constant(float(calibration_constant), camera="both")

        bluetooth_data = calibrations.get("bluetooth", {}) if isinstance(calibrations, dict) else {}
        if bluetooth_data:
            other_robot = bluetooth_data.get("other_robot")
            if isinstance(other_robot, dict):
                shared_data.set_bluetooth_other_robot_info(other_robot)

        logger.info("Calibration data loaded")
    except Exception as e:
        logger.warning(f"Invalid calibration data format: {e}")
