import cv2
import json
import multiprocessing.synchronize
import threading
import time
import numpy as np
from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server

from robot import calibration, utils, vision
from robot.hardware import line_sensors
from robot.logic import autonomous_mode
from robot.multiprocessing import shared_data
from robot.robot import RobotManualControl
from robot.utils import Suppress200Filter
from robot.vision import DetectedObject, PositionEstimate
from robot.config import *


app = Flask(__name__)

app.config.update(DEBUG=False, ENV="production")

logger = utils.get_logger("API Process")

_werkzeug_logger = utils.get_logger("werkzeug")
_werkzeug_logger.addFilter(Suppress200Filter())



def gen_frames(fps=None, show_detections=True):
    if fps is not None:
        try:
            fps = float(fps)
            if fps <= 0:
                fps = API_VIDEO_TARGET_FPS
        except Exception:
            fps = API_VIDEO_TARGET_FPS
    else:
        fps = API_VIDEO_TARGET_FPS
    sleep_time = 1.0 / fps if fps > 0 else 0
    last_frame_data = None
    frame_count = 0
    
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), API_VIDEO_JPEG_QUALITY]

    while True:
        if sleep_time:
            time.sleep(sleep_time)
        
        frame_data = shared_data.get_camera_frame()
        if frame_data is None or frame_data.frame is None:
            if last_frame_data:
                yield last_frame_data
            continue

        frame = frame_data.frame
        
        if show_detections:
            try:
                detections = shared_data.get_detected_objects()
                if not isinstance(detections, list):
                    detections = []

                detected_objects = []
                for det in detections:
                    if isinstance(det, dict):
                        color = tuple(det.get('color', (255, 255, 255))) if det.get('color') else (255, 255, 255)
                        detected_objects.append(DetectedObject(
                            object_type=det.get('object_type', 'unknown'),
                            x=det.get('x', 0),
                            y=det.get('y', 0),
                            width=det.get('width', 0),
                            height=det.get('height', 0),
                            confidence=det.get('confidence', 0.0),
                            color=color
                        ))
                    else:
                        detected_objects.append(det)
                
                if detected_objects:
                    frame = vision.draw_detections_on_frame(frame, detected_objects, draw_labels=True)
            except Exception as e:
                logger.debug(f"Error drawing detections: {e}")
        
        ret, buffer = cv2.imencode('.jpg', frame, encode_params)
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        last_frame_data = (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        yield last_frame_data
        frame_count += 1


@app.route('/api/video_feed')
def video_feed():
    try:
        fps = request.args.get('fps', None)
        show_detections = request.args.get('show_detections', 'true').lower() in ('true', '1', 'yes')
        response = Response(gen_frames(fps=fps, show_detections=show_detections), mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/camera_ball_distance_calibration', methods=['POST'])
def camera_ball_distance_calibration():
    try:
        data = request.get_json()
        
        if not data or 'known_distance_mm' not in data:
            return jsonify({'Missing known_distance_mm'}), 400

        known_distance = float(data['known_distance_mm'])
        
        if not isinstance(known_distance, (int, float)) or known_distance <= 0:
            return jsonify({"known_distance_mm must be a positive number"}), 400
        
        calibration_constant = calibration.calibrate_ball_distance(known_distance)
        
        return jsonify({'status': 'ok', 'calibration_constant': calibration_constant})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return '', 500


@app.route('/api/get_sensor_data')
def get_sensor_data():
    try:
        data = shared_data.get_hardware_data()
        if data is None:
            return jsonify({"error": "No data available yet"}), 503
        
        running_state = shared_data.get_running_state()
        
        camera_ball_position = shared_data.get_camera_ball_data()
        camera_ball_angle = camera_ball_position.angle if camera_ball_position else None
        camera_ball_distance = camera_ball_position.distance if camera_ball_position else None
        camera_ball_detected = camera_ball_position.detected if camera_ball_position else None


        response = {
            "compass": {
                "heading": data.compass.heading,
                "pitch": data.compass.pitch,
                "roll": data.compass.roll
            },
            "ir": {
                "angle": data.ir.angle,
                "distance": data.ir.distance,
                "sensors": data.ir.sensors,
                "status": data.ir.status
            },
            "camera_ball": {
                "angle": camera_ball_angle,
                "distance": camera_ball_distance,
                "detected": camera_ball_detected
            },
            "line": {
                "raw": data.line,
                "detected": line_sensors.get_line_detected(),
                "thresholds": shared_data.get_line_detection_thresholds(),
            },
            "motors": shared_data.get_motor_speeds(),
            "kicker": shared_data.get_kicker_state(),
            "running_state": {
                "running": running_state.running if running_state else False,
                "bt_module_enabled": running_state.bt_module_enabled if running_state else False,
                "bt_module_state": running_state.bt_module_state if running_state else False,
                "switch_state": running_state.switch_state if running_state else False
            } if running_state else None,
            "timestamp": data.timestamp
        }
        return jsonify(response)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_logs')
def get_logs():
    try:
        since = request.args.get('since', '0')
        try:
            since_id = int(since)
        except ValueError:
            since_id = 0

        items, last_id = utils.get_logs(since_id)
        return jsonify({"logs": items, "last_id": last_id})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_mode')
def get_mode():
    try:
        mode = shared_data.get_robot_mode()
        if not mode in [0, 1, 2]:
            raise ValueError(f"Invalid robot mode: {mode}")
        return jsonify({"mode": ["idle", "manual", "autonomous"][mode]})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    try:
        mode = request.args.get('mode', None)
        if mode is None:
            mode = (request.get_json() or {}).get('mode', None)
        if mode is None or not mode in ["idle", "manual", "autonomous"]:
            return jsonify({"error": "Invalid mode"}), 400
        shared_data.set_robot_mode(["idle", "manual", "autonomous"].index(mode))
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_manual_control', methods=['POST'])
def set_manual_control():
    try:
        data_str = request.data
        if data_str is None:
            return jsonify({"error": f"Invalid request"}), 400
        data = json.loads(data_str)
        if data.get('move', None) is None or \
        not isinstance(data['move'].get('angle', None), (int, float)) or \
        not isinstance(data['move'].get('speed', None), (int, float)) or \
        not isinstance(data.get('rotate', None), (int, float)):
            return jsonify({"error": f"Invalid request data \"{data_str}\""}), 400

        control = RobotManualControl(
            move_angle=data['move']['angle'],
            move_speed=data['move']['speed'],
            rotate=data['rotate']
        )
        shared_data.set_manual_control(control)
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/reset_compass', methods=['POST'])
def reset_compass():
    try:
        shared_data.request_compass_reset()
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/start', methods=['POST'])
def start_line_calibration():
    try:
        data = request.get_json(silent=True) or {}
        phase = data.get('phase', 1)
        
        if phase not in [1, 2]:
            return jsonify({"error": "Bad request (phase)"}), 400
        
        calibration.start_line_calibration(phase)

        return jsonify({
            "status": "ok",
            "phase": phase,
            "message": f"Phase {phase} started"
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/stop', methods=['POST'])
def stop_line_calibration():
    try:
        status = calibration.get_line_calibration_status()
        if not status['phase'] > 0:
            return jsonify({"error": "Calibration is not active"}), 400
        
        thresholds, min_values, max_values, phase = calibration.stop_line_calibration()
        
        can_start_phase2 = (phase == 1 and any(min_values[i] != float('inf') for i in range(len(min_values))))
        
        return jsonify({
            "status": "ok",
            "phase": phase,
            "thresholds": thresholds,
            "min_values": min_values,
            "max_values": max_values,
            "can_start_phase2": can_start_phase2
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/cancel', methods=['POST'])
def cancel_line_calibration():
    try:
        status = calibration.get_line_calibration_status()
        if not status['phase'] > 0:
            return jsonify({"error": "Calibration is not active"}), 400
        
        phase = status.get('phase', 0)
        calibration.stop_line_calibration(cancel=True)
        
        return jsonify({
            "status": "ok",
            "phase": phase,
            "message": "Calibration cancelled."
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/status')
def get_line_calibration_status():
    try:
        status = calibration.get_line_calibration_status()
        return jsonify(status)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/set_thresholds', methods=['POST'])
def set_line_thresholds():
    try:
        data = request.get_json()
        if not data or 'thresholds' not in data:
            return jsonify({"error": "Missing 'thresholds' in request body"}), 400
        
        thresholds = data['thresholds']
        if not isinstance(thresholds, list) or len(thresholds) != LINE_SENSOR_COUNT:
            return jsonify({"error": f"Thresholds must be a list of {LINE_SENSOR_COUNT} [min, max] pairs"}), 400
        
        try:
            thresholds_ranges = []
            for t in thresholds:
                if isinstance(t, list) and len(t) == 2:
                    thresholds_ranges.append([int(t[0]), int(t[1])])
                else:
                    return jsonify({"error": f"Thresholds must be a list of {LINE_SENSOR_COUNT} [min, max] pairs"}), 400
            
            calibration.set_line_detection_thresholds(thresholds_ranges)
            return jsonify({"status": "ok", "thresholds": thresholds_ranges})
        except (ValueError, TypeError) as e:
            return jsonify({"error": f"Invalid threshold values: {e}"}) , 400
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_motor_settings')
def get_motor_settings():
    try:
        return jsonify({
            "rotation_correction_enabled": shared_data.get_rotation_correction_enabled(),
            "line_avoiding_enabled": shared_data.get_line_avoiding_enabled(),
            "position_based_speed_enabled": shared_data.get_position_based_speed_enabled(),
            "camera_ball_usage_enabled": shared_data.get_camera_ball_usage_enabled(),
            "always_facing_goal_enabled": shared_data.get_always_facing_goal_enabled(),
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_motor_settings', methods=['POST'])
def set_motor_settings():
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        
        if 'rotation_correction_enabled' in data:
            if not isinstance(data['rotation_correction_enabled'], bool):
                return jsonify({"error": "rotation_correction_enabled must be a boolean"}), 400
            shared_data.set_rotation_correction_enabled(data['rotation_correction_enabled'])
            logger.info(f"Rotation correction {'enabled' if data['rotation_correction_enabled'] else 'disabled'}")
        
        if 'line_avoiding_enabled' in data:
            if not isinstance(data['line_avoiding_enabled'], bool):
                return jsonify({"error": "line_avoiding_enabled must be a boolean"}), 400
            shared_data.set_line_avoiding_enabled(data['line_avoiding_enabled'])
            logger.info(f"Line avoiding {'enabled' if data['line_avoiding_enabled'] else 'disabled'}")
        
        if 'position_based_speed_enabled' in data:
            if not isinstance(data['position_based_speed_enabled'], bool):
                return jsonify({"error": "position_based_speed_enabled must be a boolean"}), 400
            shared_data.set_position_based_speed_enabled(data['position_based_speed_enabled'])
            logger.info(f"Position-based speed {'enabled' if data['position_based_speed_enabled'] else 'disabled'}")
        
        if 'camera_ball_usage_enabled' in data:
            if not isinstance(data['camera_ball_usage_enabled'], bool):
                return jsonify({"error": "camera_ball_usage_enabled must be a boolean"}), 400
            shared_data.set_camera_ball_usage_enabled(data['camera_ball_usage_enabled'])
            logger.info(f"Camera ball usage {'enabled' if data['camera_ball_usage_enabled'] else 'disabled'}")
        
        if 'always_facing_goal_enabled' in data:
            if not isinstance(data['always_facing_goal_enabled'], bool):
                return jsonify({"error": "always_facing_goal_enabled must be a boolean"}), 400
            shared_data.set_always_facing_goal_enabled(data['always_facing_goal_enabled'])
            logger.info(f"Always facing goal {'enabled' if data['always_facing_goal_enabled'] else 'disabled'}")
        
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_goal_settings')
def get_goal_settings():
    try:
        goal_color = shared_data.get_goal_color()
        yellow_lower, yellow_upper = shared_data.get_goal_calibration('yellow')
        blue_lower, blue_upper = shared_data.get_goal_calibration('blue')
        
        return jsonify({
            "goal_color": goal_color,
            "calibration": {
                "yellow": {
                    "lower": yellow_lower,
                    "upper": yellow_upper
                },
                "blue": {
                    "lower": blue_lower,
                    "upper": blue_upper
                }
            }
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_goal_settings', methods=['POST'])
def set_goal_settings():
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        
        if 'goal_color' in data:
            if data['goal_color'] not in ['yellow', 'blue']:
                return jsonify({"error": "goal_color must be 'yellow' or 'blue'"}), 400
            calibration.set_enemy_goal_color(data['goal_color'])
        
        if 'calibration' in data:
            cal = data['calibration']
            if 'yellow' in cal:
                yellow = cal['yellow']
                if 'lower' in yellow and 'upper' in yellow:
                    if yellow['lower'] and len(yellow['lower']) == 3 and yellow['upper'] and len(yellow['upper']) == 3:
                        calibration.set_goal_color_range('yellow', tuple(yellow['lower']), tuple(yellow['upper']))
            if 'blue' in cal:
                blue = cal['blue']
                if 'lower' in blue and 'upper' in blue:
                    if blue['lower'] and len(blue['lower']) == 3 and blue['upper'] and len(blue['upper']) == 3:
                        calibration.set_goal_color_range('blue', tuple(blue['lower']), tuple(blue['upper']))
        
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_goal_detection')
def get_goal_detection():
    try:
        result = shared_data.get_goal_detection_result()
        if result is None:
            return jsonify({
                "goal_detected": False,
                "alignment": 0.0,
                "goal_center_x": None,
                "goal_area": 0.0
            })
        
        return jsonify({
            "goal_detected": result.detected,
            "alignment": result.alignment,
            "goal_center_x": result.center_x,
            "goal_area": result.area,
            "distance_mm": result.distance_mm,
            "goal_height_pixels": result.height_pixels
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/compute_hsv_from_regions', methods=['POST'])
def compute_hsv_from_regions():
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        
        regions = data.get('regions', [])
        if not regions:
            return jsonify({"error": "Missing 'regions' in request body"}), 400

        h_min, s_min, v_min = 179, 255, 255
        h_max, s_max, v_max = 0, 0, 0

        for region in regions:
            x = int(region.get('x', 0))
            y = int(region.get('y', 0))
            width = int(region.get('width', 0))
            height = int(region.get('height', 0))
            
            if width <= 0 or height <= 0:
                return jsonify({"error": "Invalid region dimensions"}), 400
            
            frame_data = shared_data.get_camera_frame()
            if frame_data is None or frame_data.frame is None:
                return jsonify({"error": "No camera frame available"}), 503
            frame = frame_data.frame
            
            frame_height, frame_width = frame.shape[:2]
            
            x = max(0, min(x, frame_width - 1))
            y = max(0, min(y, frame_height - 1))
            x2 = max(0, min(x + width, frame_width))
            y2 = max(0, min(y + height, frame_height))
            
            region = frame[y:y2, x:x2]
            
            if region.size == 0:
                return jsonify({"error": "Empty region"}), 400
            
            hsv_region = cv2.cvtColor(region, cv2.COLOR_RGB2HSV)
            
            h_values = hsv_region[:, :, 0].flatten()
            s_values = hsv_region[:, :, 1].flatten()
            v_values = hsv_region[:, :, 2].flatten()
            
            new_h_min = int(np.percentile(h_values, 5))
            new_h_max = int(np.percentile(h_values, 95))
            new_s_min = int(np.percentile(s_values, 5))
            new_s_max = int(np.percentile(s_values, 95))
            new_v_min = int(np.percentile(v_values, 5))
            new_v_max = int(np.percentile(v_values, 95))
            
            new_h_min = max(0, new_h_min - 5)
            new_h_max = min(179, new_h_max + 5)
            new_s_min = max(0, new_s_min - 20)
            new_s_max = min(255, new_s_max + 20)
            new_v_min = max(0, new_v_min - 20)
            new_v_max = min(255, new_v_max + 20)

            h_min = min(h_min, new_h_min)
            h_max = max(h_max, new_h_max)
            s_min = min(s_min, new_s_min)
            s_max = max(s_max, new_s_max)
            v_min = min(v_min, new_v_min)
            v_max = max(v_max, new_v_max)
        
        return jsonify({
            "lower": [h_min, s_min, v_min],
            "upper": [h_max, s_max, v_max]
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_position_estimate')
def get_position_estimate():
    try:
        position: PositionEstimate | None = vision.get_position_estimate()
        if position is None:
            return jsonify({
                "x_mm": None,
                "y_mm": None,
                "confidence": 0.0
            })
        
        return jsonify({
            "x_mm": position.x_mm,
            "y_mm": position.y_mm,
            "confidence": position.confidence
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance/get_focal_length')
def get_goal_focal_length():
    try:
        focal_length = shared_data.get_goal_focal_length()
        return jsonify({"focal_length_pixels": focal_length})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance/set_focal_length', methods=['POST'])
def set_goal_focal_length():
    try:
        data = request.get_json()
        if not data or 'focal_length_pixels' not in data:
            return jsonify({"error": "Missing 'focal_length_pixels' in request body"}), 400
        
        focal_length = data['focal_length_pixels']
        if not isinstance(focal_length, (int, float)) or focal_length <= 0:
            return jsonify({"error": "focal_length_pixels must be a positive number"}), 400
        
        calibration.set_goal_focal_length(float(focal_length))
        return jsonify({"status": "ok", "focal_length_pixels": focal_length})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance_calibration/start', methods=['POST'])
def start_goal_distance_calibration():
    try:
        data = request.get_json(silent=True) or {}
        initial_distance = data.get('initial_distance', 200.0)
        line_distance = data.get('line_distance', 200.0)
        
        if not isinstance(initial_distance, (int, float)) or initial_distance <= 0:
            return jsonify({"error": "initial_distance must be a positive number"}), 400
        
        if not isinstance(line_distance, (int, float)) or line_distance <= 0:
            return jsonify({"error": "line_distance must be a positive number"}), 400
        
        calibration.start_goal_distance_calibration(float(initial_distance), float(line_distance))
        return jsonify({
            "status": "ok",
            "message": "Drive the robot toward the enemy goal until it detects the line, then stop calibration"
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance_calibration/stop', methods=['POST'])
def stop_goal_distance_calibration():
    try:
        result = calibration.stop_goal_distance_calibration()
        
        if not result.get('success'):
            return jsonify(result), 400
        
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500


@app.route('/api/goal_distance_calibration/cancel', methods=['POST'])
def cancel_goal_distance_calibration():
    try:
        calibration.cancel_goal_distance_calibration()
        return jsonify({"status": "ok", "message": "Calibration cancelled"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500


@app.route('/api/goal_distance_calibration/status')
def get_goal_distance_calibration_status():
    try:
        status = calibration.get_goal_distance_calibration_status()
        return jsonify(status)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500

@app.route('/api/get_autonomous_state')
def get_autonomous_state():
    try:
        state = shared_data.get_autonomous_state()
        return jsonify(state if state else {'state': 'idle'})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({'error': 'Internal server error'}), 500


@app.route('/api/get_detections')
def get_detections():
    try:
        detections = shared_data.get_detected_objects_raw()
        return jsonify({"detections": detections})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500

@app.route('/api/get_ball_calibration')
def get_ball_calibration():
    try:
        lower, upper = shared_data.get_ball_calibration()
        return jsonify({"lower": lower, "upper": upper})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_ball_calibration', methods=['POST'])
def set_ball_calibration():
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        lower = data.get('lower')
        upper = data.get('upper')
        if not lower or not upper or len(lower) != 3 or len(upper) != 3:
            return jsonify({"error": "'lower' and 'upper' must be lists of 3 values"}), 400
        calibration.set_ball_color_range(tuple([int(v) for v in lower]), tuple([int(v) for v in upper]))
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500

@app.route('/api/get_all_state_machines')
def get_all_state_machines():
    try:
        state_machines = autonomous_mode.get_available_state_machines()
        return jsonify({"state_machines": list(state_machines.keys())})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500

@app.route('/api/get_state_machine')
def get_state_machine():
    try:
        current = autonomous_mode.get_current_state_machine()
        return jsonify({"current_state_machine": current.name if current else None})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500

@app.route('/api/set_state_machine', methods=['POST'])
def set_state_machine():
    try:
        data = request.get_json()
        if not data or 'name' not in data:
            return jsonify({"error": "Missing 'name' in request body"}), 400
        name = data['name']
        state_machine = autonomous_mode.find_state_machine_by_name(name)
        if state_machine is None:
            return jsonify({"error": f"State machine with name '{name}' not found"}), 404
        autonomous_mode.set_current_state_machine(state_machine)
        return jsonify({"status": "ok", "current_state_machine": name})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/')
def get_index_html():
    try:
        logger.warning("Accessed root endpoint, which serves index.html. In production, this should be served by nginx directly. This may indicate a misconfiguration.")
        with open('web_server/static/index.html', 'r', encoding='utf-8') as f:
            index_html = f.read()
            return index_html
    except Exception:
        return "", 404



def start(host: str = API_HOST, port: int = API_PORT, stop_event: multiprocessing.synchronize.Event | None = None):
    server = make_server(host=host, port=port, app=app, threaded=True)
    logger.info(f"API server started on {host}:{port}")

    thread = threading.Thread(target=server.serve_forever)
    try:
        thread.start()

        if stop_event:
            stop_event.wait()
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        thread.join()


if __name__ == "__main__":
    start()

