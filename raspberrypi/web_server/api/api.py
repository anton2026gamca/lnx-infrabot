import cv2
import os
import logging
import threading
import time
from flask import Flask, Response, request, jsonify, send_from_directory
from helpers.helpers import LogFormatter, RobotMode

app = Flask(__name__)

camera_frame_getter = None
hardware_data_getter = None
motor_speeds_getter = None
kicker_state_getter = None

robot_mode_getter = None
robot_mode_setter = None


logs_buffer: dict[int, dict] = {}
logs_buffer_lock = threading.Lock()
_next_log_id = 1
_log_handler_installed = False


def gen_frames():
    target_fps = 30
    sleep_time = None
    if target_fps and target_fps > 0:
        sleep_time = 1.0 / float(target_fps)

    while True:
        if sleep_time:
            time.sleep(sleep_time)
        
        if camera_frame_getter is None or camera_frame_getter() is None:
            continue
            
        ret, buffer = cv2.imencode('.jpg', camera_frame_getter())
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


class BufferedLogHandler(logging.Handler):
    def __init__(self, formatter=None):
        super().__init__()
        if formatter:
            self.setFormatter(formatter)
        elif LogFormatter is not None:
            self.setFormatter(LogFormatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            formatted = self.format(record)
            entry = {
                "message": formatted,
                "level": record.levelname,
                "logger": record.name,
                "time": getattr(record, "created", None),
            }
            global _next_log_id
            with logs_buffer_lock:
                lid = _next_log_id
                logs_buffer[lid] = entry
                _next_log_id += 1
        except Exception:
            self.handleError(record)

def _ensure_log_buffering_installed():
    global _log_handler_installed
    if _log_handler_installed:
        return
    try:
        handler = BufferedLogHandler()
        root = logging.getLogger()
        root.addHandler(handler)
        _log_handler_installed = True
    except Exception:
        pass
_ensure_log_buffering_installed()


@app.route('/api/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/get_sensor_data')
def sensor_data():
    if hardware_data_getter is None:
        return jsonify({"error": "Hardware data not available"}), 503
    
    data = hardware_data_getter()
    if data is None:
        return jsonify({"error": "No data received yet"}), 503
    
    return jsonify({
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
        "line": data.line,
        "motors": motor_speeds_getter() if motor_speeds_getter else [0, 0, 0, 0],
        "kicker": kicker_state_getter() if kicker_state_getter else False,
        "timestamp": data.timestamp
    })

@app.route('/api/get_logs')
def get_logs():
    since = request.args.get('since', '0')
    try:
        since_id = int(since)
    except Exception:
        since_id = 0

    with logs_buffer_lock:
        items = [
            {"id": lid, **entry}
            for lid, entry in sorted(logs_buffer.items())
            if lid > since_id
        ]

    last_id = max(logs_buffer.keys()) if logs_buffer else 0
    return jsonify({"logs": items, "last_id": last_id})

@app.route('/api/get_mode')
def get_state():
    if robot_mode_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    mode = robot_mode_getter()
    return jsonify({"mode": mode})

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    mode = request.json.get('mode', None)
    if mode is None or not isinstance(mode, str) or not mode in RobotMode.__dict__.values():
        return jsonify({"error": "Invalid mode"}), 400
    if robot_mode_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    robot_mode_setter(mode)
    return jsonify({"status": "ok"})


def start():
    app.run(host='0.0.0.0', port=5000, threaded=True)
