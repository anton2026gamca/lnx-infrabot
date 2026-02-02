import cv2
import json
import logging
import time
from flask import Flask, Response, request, jsonify
from werkzeug.serving import make_server

from helpers.helpers import RobotMode, RobotManualControl
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
            "thresholds": line_detection_thresholds_getter() if line_detection_thresholds_getter else [500] * LINE_SENSOR_COUNT
        },
        "motors": motor_speeds_getter() if motor_speeds_getter else [0, 0, 0, 0],
        "kicker": kicker_state_getter() if kicker_state_getter else False,
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
    
    line_calibration_starter()
    logger.info("Line sensor calibration started")
    return jsonify({
        "status": "ok",
        "message": "Calibration started. Move robot over white and black surfaces."
    })


@app.route('/api/line_calibration/stop', methods=['POST'])
def stop_line_calibration():
    if line_calibration_stopper is None or line_calibration_status_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    status = line_calibration_status_getter()
    if not status['active']:
        return jsonify({"error": "Calibration is not active"}), 400
    
    thresholds, min_values, max_values = line_calibration_stopper() # pyright: ignore[reportGeneralTypeIssues]
    
    for i in range(LINE_SENSOR_COUNT):
        if min_values[i] == float('inf') or max_values[i] == float('-inf'):
            logger.warning(f"Sensor {i} has no calibration data, keeping default threshold")
    
    logger.info(f"Line sensor calibration completed. Thresholds: {thresholds}")
    
    return jsonify({
        "status": "ok",
        "thresholds": thresholds,
        "min_values": min_values,
        "max_values": max_values
    })


@app.route('/api/line_calibration/cancel', methods=['POST'])
def cancel_line_calibration():
    if line_calibration_stopper is None or line_calibration_status_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    status = line_calibration_status_getter()
    if not status['active']:
        return jsonify({"error": "Calibration is not active"}), 400
    
    line_calibration_stopper(cancel=True)
    
    logger.info("Line sensor calibration cancelled")
    
    return jsonify({
        "status": "ok",
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
        return jsonify({"error": f"Thresholds must be a list of {LINE_SENSOR_COUNT} values"}), 400
    
    try:
        thresholds_int = [int(t) for t in thresholds]
        line_detection_thresholds_setter(thresholds_int)
        logger.info(f"Line detection thresholds manually set to: {thresholds_int}")
        return jsonify({"status": "ok", "thresholds": thresholds_int})
    except (ValueError, TypeError) as e:
        return jsonify({"error": f"Invalid threshold values: {e}"}), 400


def start(host: str = API_HOST, port: int = API_PORT):
    server = make_server(host=host, port=port, app=app, threaded=True)
    logger.info(f"API server started on {host}:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    start()
