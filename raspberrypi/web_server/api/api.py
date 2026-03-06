import cv2
import json
import logging
import time
from flask import Flask, Response, request, jsonify
from werkzeug.serving import make_server

from helpers.helpers import RobotMode, RobotManualControl, PositionEstimate
from config import API_VIDEO_JPEG_QUALITY, API_VIDEO_TARGET_FPS, API_HOST, API_PORT, LINE_SENSOR_COUNT



app = Flask(__name__)
app.config.update(DEBUG=False, ENV="production")

camera_frame_getter = None
hardware_data_getter = None
motor_speeds_getter = None
kicker_state_getter = None

robot_mode_getter = None
robot_mode_setter = None
manual_control_getter = None
manual_control_setter = None
compass_reset_requester = None

logs_getter = None

line_detection_thresholds_getter = None
line_detection_thresholds_setter = None
line_detected_getter = None
line_calibration_starter = None
line_calibration_stopper = None
line_calibration_status_getter = None

running_state_getter = None

rotation_correction_enabled_getter = None
rotation_correction_enabled_setter = None
line_avoiding_enabled_getter = None
line_avoiding_enabled_setter = None
position_based_speed_enabled_getter = None
position_based_speed_enabled_setter = None


goal_color_getter = None
goal_color_setter = None
goal_calibration_getter = None
goal_calibration_setter = None
ball_calibration_getter = None
ball_calibration_setter = None
goal_detection_result_getter = None
goal_focal_length_getter = None
goal_focal_length_setter = None
goal_distance_calibration_starter = None
goal_distance_calibration_stopper = None
goal_distance_calibration_canceler = None
goal_distance_calibration_status_getter = None
position_estimate_getter = None

autonomous_state_getter = None

logger = logging.getLogger("API Process")


def gen_frames():
    sleep_time = 1.0 / API_VIDEO_TARGET_FPS if API_VIDEO_TARGET_FPS > 0 else 0
    last_frame_data = None
    frame_count = 0
    
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), API_VIDEO_JPEG_QUALITY]

    while True:
        if sleep_time:
            time.sleep(sleep_time)
        
        if camera_frame_getter is None:
            continue
        
        frame = camera_frame_getter()
        if frame is None:
            if last_frame_data:
                yield last_frame_data
            continue
        
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
        response = Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_sensor_data')
def get_sensor_data():
    try:
        if hardware_data_getter is None:
            raise RuntimeError("api.hardware_data_getter is None")
        
        data = hardware_data_getter()
        if data is None:
            return jsonify({"error": "No data available yet"}), 503
        
        running_state = running_state_getter() if running_state_getter else None
        
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
            "line": {
                "raw": data.line,
                "detected": line_detected_getter() if line_detected_getter else [False] * LINE_SENSOR_COUNT,
                "thresholds": line_detection_thresholds_getter() if line_detection_thresholds_getter else [],
            },
            "motors": motor_speeds_getter() if motor_speeds_getter else [0, 0, 0, 0],
            "kicker": kicker_state_getter() if kicker_state_getter else False,
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
        if logs_getter is None:
            raise RuntimeError("api.logs_getter is None")
        
        since = request.args.get('since', '0')
        try:
            since_id = int(since)
        except ValueError:
            since_id = 0

        items, last_id = logs_getter(since_id) # pyright: ignore[reportGeneralTypeIssues]
        return jsonify({"logs": items, "last_id": last_id})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_mode')
def get_mode():
    try:
        if robot_mode_getter is None:
            raise RuntimeError("api.robot_mode_getter is None")
        
        mode = robot_mode_getter()
        return jsonify({"mode": mode})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    try:
        if robot_mode_setter is None:
            raise RuntimeError("api.robot_mode_setter is None")
        mode = request.args.get('mode', None)
        if mode is None:
            mode = request.json.get('mode', None)
        if mode is None or not mode in [RobotMode.IDLE, RobotMode.MANUAL, RobotMode.AUTONOMOUS]:
            return jsonify({"error": "Invalid mode"}), 401
        robot_mode_setter(mode)
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_manual_control', methods=['POST'])
def set_manual_control():
    try:
        if manual_control_setter is None:
            raise RuntimeError("api.manual_control_setter is None")
        
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
        manual_control_setter(control)
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/reset_compass', methods=['POST'])
def reset_compass():
    try:
        if compass_reset_requester is None:
            raise RuntimeError("api.compass_reset_requester is None")
        
        compass_reset_requester()
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/start', methods=['POST'])
def start_line_calibration():
    try:
        if line_calibration_starter is None:
            raise RuntimeError("api.line_calibration_starter is None")
        
        data = request.get_json(silent=True) or {}
        phase = data.get('phase', 1)
        
        if phase not in [1, 2]:
            return jsonify({"error": "Bad request (phase)"}), 400
        
        line_calibration_starter(phase)
        logger.info(f"Line sensor calibration phase {phase} started")
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
        if line_calibration_stopper is None or line_calibration_status_getter is None:
            raise RuntimeError("api.line_calibration_stopper or api.line_calibration_status_getter is None")
        
        status = line_calibration_status_getter()
        if not status['phase'] > 0:
            return jsonify({"error": "Calibration is not active"}), 400
        
        thresholds, min_values, max_values, phase = line_calibration_stopper() # pyright: ignore[reportGeneralTypeIssues]
        logger.info(f"Line sensor calibration phase {phase} completed and applied. Thresholds: {thresholds}")
        
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
        if line_calibration_stopper is None:
            raise RuntimeError("api.line_calibration_stopper is None")
        if line_calibration_status_getter is None:
            raise RuntimeError("api.line_calibration_status_getter is None")
        
        status = line_calibration_status_getter()
        if not status['phase'] > 0:
            return jsonify({"error": "Calibration is not active"}), 400
        
        phase = status.get('phase', 0)
        line_calibration_stopper(cancel=True)
        
        logger.info(f"Line sensor calibration phase {phase} cancelled")
        
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
        if line_calibration_status_getter is None:
            raise RuntimeError("api.line_calibration_status_getter is None")
        
        status = line_calibration_status_getter()
        return jsonify(status)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/line_calibration/set_thresholds', methods=['POST'])
def set_line_thresholds():
    try:
        if line_detection_thresholds_setter is None:
            raise RuntimeError("api.line_detection_thresholds_setter is None")
        
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
            
            line_detection_thresholds_setter(thresholds_ranges)  # pyright: ignore[reportGeneralTypeIssues]
            logger.info(f"Line detection thresholds manually set to: {thresholds_ranges}")
            return jsonify({"status": "ok", "thresholds": thresholds_ranges})
        except (ValueError, TypeError) as e:
            return jsonify({"error": f"Invalid threshold values: {e}"}) , 400
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_motor_settings')
def get_motor_settings():
    try:
        if rotation_correction_enabled_getter is None:
            raise RuntimeError("api.rotation_correction_enabled_getter is None")
        if line_avoiding_enabled_getter is None:
            raise RuntimeError("api.line_avoiding_enabled_getter is None")
        
        return jsonify({
            "rotation_correction_enabled": rotation_correction_enabled_getter(),
            "line_avoiding_enabled": line_avoiding_enabled_getter(),
            "position_based_speed_enabled": position_based_speed_enabled_getter() if position_based_speed_enabled_getter else True,
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_motor_settings', methods=['POST'])
def set_motor_settings():
    try:
        if rotation_correction_enabled_setter is None:
            raise RuntimeError("api.rotation_correction_enabled_setter is None")
        if line_avoiding_enabled_setter is None:
            raise RuntimeError("api.line_avoiding_enabled_setter is None")
        
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        
        if 'rotation_correction_enabled' in data:
            if not isinstance(data['rotation_correction_enabled'], bool):
                return jsonify({"error": "rotation_correction_enabled must be a boolean"}), 400
            rotation_correction_enabled_setter(data['rotation_correction_enabled'])
            logger.info(f"Rotation correction {'enabled' if data['rotation_correction_enabled'] else 'disabled'}")
        
        if 'line_avoiding_enabled' in data:
            if not isinstance(data['line_avoiding_enabled'], bool):
                return jsonify({"error": "line_avoiding_enabled must be a boolean"}), 400
            line_avoiding_enabled_setter(data['line_avoiding_enabled'])
            logger.info(f"Line avoiding {'enabled' if data['line_avoiding_enabled'] else 'disabled'}")
        
        if 'position_based_speed_enabled' in data:
            if not isinstance(data['position_based_speed_enabled'], bool):
                return jsonify({"error": "position_based_speed_enabled must be a boolean"}), 400
            if position_based_speed_enabled_setter:
                position_based_speed_enabled_setter(data['position_based_speed_enabled'])
                logger.info(f"Position-based speed {'enabled' if data['position_based_speed_enabled'] else 'disabled'}")
        
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_goal_settings')
def get_goal_settings():
    try:
        if goal_color_getter is None:
            raise RuntimeError("api.goal_color_getter is None")
        if goal_calibration_getter is None:
            raise RuntimeError("api.goal_calibration_getter is None")
        
        goal_color = goal_color_getter()
        yellow_lower, yellow_upper = goal_calibration_getter('yellow') # pyright: ignore[reportGeneralTypeIssues]
        blue_lower, blue_upper = goal_calibration_getter('blue') # pyright: ignore[reportGeneralTypeIssues]
        
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
        if goal_color_setter is None:
            raise RuntimeError("api.goal_color_setter is None")
        if goal_calibration_setter is None:
            raise RuntimeError("api.goal_calibration_setter is None")
        
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        
        if 'goal_color' in data:
            if data['goal_color'] not in ['yellow', 'blue']:
                return jsonify({"error": "goal_color must be 'yellow' or 'blue'"}), 400
            goal_color_setter(data['goal_color'])
            logger.info(f"Goal color set to {data['goal_color']}")
        
        if 'calibration' in data:
            cal = data['calibration']
            if 'yellow' in cal:
                yellow = cal['yellow']
                if 'lower' in yellow and 'upper' in yellow:
                    if yellow['lower'] and len(yellow['lower']) == 3 and yellow['upper'] and len(yellow['upper']) == 3:
                        logger.info(yellow)
                        goal_calibration_setter('yellow', yellow['lower'], yellow['upper'])
                        logger.info(f"Yellow goal calibration set: {yellow['lower']} - {yellow['upper']}")
            if 'blue' in cal:
                blue = cal['blue']
                if 'lower' in blue and 'upper' in blue:
                    if blue['lower'] and len(blue['lower']) == 3 and blue['upper'] and len(blue['upper']) == 3:
                        goal_calibration_setter('blue', blue['lower'], blue['upper'])
                        logger.info(f"Blue goal calibration set: {blue['lower']} - {blue['upper']}")
        
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/get_goal_detection')
def get_goal_detection():
    try:
        if goal_detection_result_getter is None:
            raise RuntimeError("api.goal_detection_result_getter is None")
        
        result = goal_detection_result_getter()
        if result is None:
            return jsonify({
                "goal_detected": False,
                "alignment": 0.0,
                "goal_center_x": None,
                "goal_area": 0.0
            })
        
        return jsonify({
            "goal_detected": result.goal_detected,
            "alignment": result.alignment,
            "goal_center_x": result.goal_center_x,
            "goal_area": result.goal_area,
            "distance_mm": result.distance_mm,
            "goal_height_pixels": result.goal_height_pixels
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/compute_hsv_from_regions', methods=['POST'])
def compute_hsv_from_regions():
    try:
        if camera_frame_getter is None:
            raise RuntimeError("Camera frame getter is not set")
        
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
            
            frame = camera_frame_getter()
            if frame is None:
                return jsonify({"error": "No camera frame available"}), 503
            
            frame_height, frame_width = frame.shape[:2] # pyright: ignore[reportGeneralTypeIssues]
            
            x = max(0, min(x, frame_width - 1))
            y = max(0, min(y, frame_height - 1))
            x2 = max(0, min(x + width, frame_width))
            y2 = max(0, min(y + height, frame_height))
            
            region = frame[y:y2, x:x2]
            
            if region.size == 0:
                return jsonify({"error": "Empty region"}), 400
            
            import cv2
            import numpy as np
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
        if position_estimate_getter is None:
            raise RuntimeError("api.position_estimate_getter is None")
        
        position: PositionEstimate = position_estimate_getter()
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
def get_goal_focal_length_api():
    try:
        if goal_focal_length_getter is None:
            raise RuntimeError("api.goal_focal_length_getter is None")
        
        focal_length = goal_focal_length_getter()
        return jsonify({"focal_length_pixels": focal_length})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance/set_focal_length', methods=['POST'])
def set_goal_focal_length_api():
    try:
        if goal_focal_length_setter is None:
            raise RuntimeError("api.goal_focal_length_setter is None")
        
        data = request.get_json()
        if not data or 'focal_length_pixels' not in data:
            return jsonify({"error": "Missing 'focal_length_pixels' in request body"}), 400
        
        focal_length = data['focal_length_pixels']
        if not isinstance(focal_length, (int, float)) or focal_length <= 0:
            return jsonify({"error": "focal_length_pixels must be a positive number"}), 400
        
        goal_focal_length_setter(float(focal_length))
        logger.info(f"Goal focal length set to {focal_length}")
        return jsonify({"status": "ok", "focal_length_pixels": focal_length})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance_calibration/start', methods=['POST'])
def start_goal_distance_calibration_api():
    try:
        if goal_distance_calibration_starter is None:
            raise RuntimeError("api.goal_distance_calibration_starter is None")
        
        data = request.get_json(silent=True) or {}
        initial_distance = data.get('initial_distance', 200.0)
        line_distance = data.get('line_distance', 200.0)
        
        if not isinstance(initial_distance, (int, float)) or initial_distance <= 0:
            return jsonify({"error": "initial_distance must be a positive number"}), 400
        
        if not isinstance(line_distance, (int, float)) or line_distance <= 0:
            return jsonify({"error": "line_distance must be a positive number"}), 400
        
        goal_distance_calibration_starter(float(initial_distance), float(line_distance))
        logger.info(f"Goal distance calibration started: initial={initial_distance}mm, line={line_distance}mm")
        return jsonify({
            "status": "ok",
            "message": "Drive the robot toward the enemy goal until it detects the line, then stop calibration"
        })
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/goal_distance_calibration/stop', methods=['POST'])
def stop_goal_distance_calibration_api():
    try:
        if goal_distance_calibration_stopper is None:
            raise RuntimeError("api.goal_distance_calibration_stopper is None")
        
        result = goal_distance_calibration_stopper()
        
        if not result.get('success'):
            return jsonify(result), 400
        
        logger.info(f"Goal distance calibration completed: focal_length={result['focal_length_pixels']:.2f} pixels")
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500


@app.route('/api/goal_distance_calibration/cancel', methods=['POST'])
def cancel_goal_distance_calibration_api():
    try:
        if goal_distance_calibration_canceler is None:
            raise RuntimeError("api.goal_distance_calibration_canceler is None")
        
        goal_distance_calibration_canceler()
        logger.info("Goal distance calibration cancelled")
        return jsonify({"status": "ok", "message": "Calibration cancelled"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500


@app.route('/api/goal_distance_calibration/status')
def get_goal_distance_calibration_status_api():
    try:
        if goal_distance_calibration_status_getter is None:
            raise RuntimeError("api.goal_distance_calibration_status_getter is None")
        
        status = goal_distance_calibration_status_getter()
        return jsonify(status)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500

@app.route('/api/get_autonomous_state')
def get_autonomous_state_api():
    try:
        if autonomous_state_getter is None:
            return jsonify({'state': 'unavailable'})
        state = autonomous_state_getter()
        return jsonify(state if state else {'state': 'idle'})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({'error': 'Internal server error'}), 500


@app.route('/api/get_ball_calibration')
def get_ball_calibration_api():
    try:
        if ball_calibration_getter is None:
            raise RuntimeError("api.ball_calibration_getter is None")
        lower, upper = ball_calibration_getter()
        return jsonify({"lower": lower, "upper": upper})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/api/set_ball_calibration', methods=['POST'])
def set_ball_calibration_api():
    try:
        if ball_calibration_setter is None:
            raise RuntimeError("api.ball_calibration_setter is None")
        data = request.get_json()
        if not data:
            return jsonify({"error": "Missing request body"}), 400
        lower = data.get('lower')
        upper = data.get('upper')
        if not lower or not upper or len(lower) != 3 or len(upper) != 3:
            return jsonify({"error": "'lower' and 'upper' must be lists of 3 values"}), 400
        ball_calibration_setter([int(v) for v in lower], [int(v) for v in upper])
        logger.info(f"Ball calibration set: lower={lower} upper={upper}")
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return "", 500


@app.route('/')
def get_index_html():
    try:
        logger.warning("Accessed root endpoint, which serves index.html. In production, this should be served by nginx directly. This may indicate a misconfiguration.")
        with open('web_server/site/index.html', 'r', encoding='utf-8') as f:
            index_html = f.read()
            return index_html
    except Exception:
        return "", 404

def start(host: str = API_HOST, port: int = API_PORT):
    server = make_server(host=host, port=port, app=app, threaded=True)
    logger.info(f"API server started on {host}:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    start()
