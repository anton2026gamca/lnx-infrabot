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

goal_color_getter = None
goal_color_setter = None
goal_calibration_getter = None
goal_calibration_setter = None
goal_detection_result_getter = None
goal_focal_length_getter = None
goal_focal_length_setter = None
goal_distance_calibration_starter = None
goal_distance_calibration_stopper = None
goal_distance_calibration_canceler = None
goal_distance_calibration_status_getter = None
position_estimate_getter = None

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
    response = Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response


@app.route('/api/get_sensor_data')
def get_sensor_data():
    if hardware_data_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/get_logs')
def get_logs():
    if logs_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    since = request.args.get('since', '0')
    try:
        since_id = int(since)
    except Exception:
        since_id = 0

    try:
        items, last_id = logs_getter(since_id) # pyright: ignore[reportGeneralTypeIssues]
        return jsonify({"logs": items, "last_id": last_id})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/get_mode')
def get_mode():
    if robot_mode_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    mode = robot_mode_getter()
    return jsonify({"mode": mode})

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    mode = request.args.get('mode', None)
    if mode is None:
        mode = request.json.get('mode', None)
    if mode is None or not mode in [RobotMode.IDLE, RobotMode.MANUAL, RobotMode.AUTONOMOUS]:
        return jsonify({"error": "Invalid mode"}), 401
    if robot_mode_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    robot_mode_setter(mode)
    return jsonify({"status": "ok"})


@app.route('/api/set_manual_control', methods=['POST'])
def set_manual_control():
    data_str = request.data
    if data_str is None:
        return jsonify({"error": f"Invalid request"}), 400
    data = json.loads(data_str)
    if data.get('move', None) is None or \
       not isinstance(data['move'].get('angle', None), (int, float)) or \
       not isinstance(data['move'].get('speed', None), (int, float)) or \
       not isinstance(data.get('rotate', None), (int, float)):
        return jsonify({"error": f"Invalid request data \"{data_str}\""}), 400

    if manual_control_setter is None:
        return jsonify({"error": "Internal server error"}), 503

    control = RobotManualControl(
        move_angle=data['move']['angle'],
        move_speed=data['move']['speed'],
        rotate=data['rotate']
    )
    manual_control_setter(control)
    return jsonify({"status": "ok"})


@app.route('/api/reset_compass', methods=['POST'])
def reset_compass():
    if compass_reset_requester is None:
        return jsonify({"error": "Internal server error"}), 503
    
    compass_reset_requester()
    return jsonify({"status": "ok"})


@app.route('/api/line_calibration/start', methods=['POST'])
def start_line_calibration():
    if line_calibration_starter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/line_calibration/stop', methods=['POST'])
def stop_line_calibration():
    if line_calibration_stopper is None or line_calibration_status_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/line_calibration/cancel', methods=['POST'])
def cancel_line_calibration():
    if line_calibration_stopper is None or line_calibration_status_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/line_calibration/status')
def get_line_calibration_status():
    if line_calibration_status_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    status = line_calibration_status_getter()
    return jsonify(status)


@app.route('/api/line_calibration/set_thresholds', methods=['POST'])
def set_line_thresholds():
    if line_detection_thresholds_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/get_motor_settings')
def get_motor_settings():
    if rotation_correction_enabled_getter is None or line_avoiding_enabled_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    return jsonify({
        "rotation_correction_enabled": rotation_correction_enabled_getter(),
        "line_avoiding_enabled": line_avoiding_enabled_getter()
    })


@app.route('/api/set_motor_settings', methods=['POST'])
def set_motor_settings():
    if rotation_correction_enabled_setter is None or line_avoiding_enabled_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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
    
    return jsonify({"status": "ok"})


@app.route('/api/get_goal_settings')
def get_goal_settings():
    if goal_color_getter is None or goal_calibration_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/set_goal_settings', methods=['POST'])
def set_goal_settings():
    if goal_color_setter is None or goal_calibration_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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
                if len(yellow['lower']) == 3 and len(yellow['upper']) == 3:
                    goal_calibration_setter('yellow', yellow['lower'], yellow['upper'])
                    logger.info(f"Yellow goal calibration set: {yellow['lower']} - {yellow['upper']}")
        if 'blue' in cal:
            blue = cal['blue']
            if 'lower' in blue and 'upper' in blue:
                if len(blue['lower']) == 3 and len(blue['upper']) == 3:
                    goal_calibration_setter('blue', blue['lower'], blue['upper'])
                    logger.info(f"Blue goal calibration set: {blue['lower']} - {blue['upper']}")
    
    return jsonify({"status": "ok"})


@app.route('/api/get_goal_detection')
def get_goal_detection():
    if goal_detection_result_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/compute_hsv_from_region', methods=['POST'])
def compute_hsv_from_region():
    if camera_frame_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    data = request.get_json()
    if not data:
        return jsonify({"error": "Missing request body"}), 400
    
    x = data.get('x', 0)
    y = data.get('y', 0)
    width = data.get('width', 0)
    height = data.get('height', 0)
    
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
    
    h_min = int(np.percentile(h_values, 5))
    h_max = int(np.percentile(h_values, 95))
    s_min = int(np.percentile(s_values, 5))
    s_max = int(np.percentile(s_values, 95))
    v_min = int(np.percentile(v_values, 5))
    v_max = int(np.percentile(v_values, 95))
    
    h_min = max(0, h_min - 5)
    h_max = min(179, h_max + 5)
    s_min = max(0, s_min - 20)
    s_max = min(255, s_max + 20)
    v_min = max(0, v_min - 20)
    v_max = min(255, v_max + 20)
    
    logger.info(f"Computed HSV from region ({x},{y})-({x2},{y2}): H[{h_min}-{h_max}] S[{s_min}-{s_max}] V[{v_min}-{v_max}]")
    
    return jsonify({
        "lower": [h_min, s_min, v_min],
        "upper": [h_max, s_max, v_max]
    })


@app.route('/api/get_position_estimate')
def get_position_estimate():
    if position_estimate_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/goal_distance/get_focal_length')
def get_goal_focal_length_api():
    if goal_focal_length_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    focal_length = goal_focal_length_getter()
    return jsonify({"focal_length_pixels": focal_length})


@app.route('/api/goal_distance/set_focal_length', methods=['POST'])
def set_goal_focal_length_api():
    if goal_focal_length_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    data = request.get_json()
    if not data or 'focal_length_pixels' not in data:
        return jsonify({"error": "Missing 'focal_length_pixels' in request body"}), 400
    
    focal_length = data['focal_length_pixels']
    if not isinstance(focal_length, (int, float)) or focal_length <= 0:
        return jsonify({"error": "focal_length_pixels must be a positive number"}), 400
    
    goal_focal_length_setter(float(focal_length))
    logger.info(f"Goal focal length set to {focal_length}")
    return jsonify({"status": "ok", "focal_length_pixels": focal_length})


@app.route('/api/goal_distance_calibration/start', methods=['POST'])
def start_goal_distance_calibration_api():
    if goal_distance_calibration_starter is None:
        return jsonify({"error": "Internal server error"}), 503
    
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


@app.route('/api/goal_distance_calibration/stop', methods=['POST'])
def stop_goal_distance_calibration_api():
    if goal_distance_calibration_stopper is None:
        return jsonify({"error": "Internal server error"}), 503
    
    result = goal_distance_calibration_stopper()
    
    if not result.get('success'):
        return jsonify(result), 400
    
    logger.info(f"Goal distance calibration completed: focal_length={result['focal_length_pixels']:.2f} pixels")
    return jsonify(result)


@app.route('/api/goal_distance_calibration/cancel', methods=['POST'])
def cancel_goal_distance_calibration_api():
    if goal_distance_calibration_canceler is None:
        return jsonify({"error": "Internal server error"}), 503
    
    goal_distance_calibration_canceler()
    logger.info("Goal distance calibration cancelled")
    return jsonify({"status": "ok", "message": "Calibration cancelled"})


@app.route('/api/goal_distance_calibration/status')
def get_goal_distance_calibration_status_api():
    if goal_distance_calibration_status_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    status = goal_distance_calibration_status_getter()
    return jsonify(status)

@app.route('/')
def get_index_html():
    logger.warning("Accessed root endpoint, which serves index.html. In production, this should be served by nginx directly. This may indicate a misconfiguration.")
    try:
        with open('web_server/site/index.html', 'r', encoding='utf-8') as f:
            index_html = f.read()
            return index_html
    except Exception as e:
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
