import cv2
import time
from flask import Flask, Response, request, jsonify
from helpers.helpers import RobotMode
from werkzeug.serving import make_server



app = Flask(__name__)
app.config.update(DEBUG=False, ENV="production")

camera_frame_getter = None
hardware_data_getter = None
motor_speeds_getter = None
kicker_state_getter = None

robot_mode_getter = None
robot_mode_setter = None
logs_getter = None


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
    if logs_getter is None:
        return jsonify({"error": "Logs not available"}), 503
    
    since = request.args.get('since', '0')
    try:
        since_id = int(since)
    except Exception:
        since_id = 0

    try:
        items, last_id = logs_getter(since_id)
        return jsonify({"logs": items, "last_id": last_id})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/get_mode')
def get_state():
    if robot_mode_getter is None:
        return jsonify({"error": "Internal server error"}), 503
    
    mode = robot_mode_getter()
    return jsonify({"mode": mode})

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    mode = request.json.get('mode', None)
    if not mode or not mode in RobotMode.__dict__.values():
        return jsonify({"error": "Invalid mode"}), 400
    if robot_mode_setter is None:
        return jsonify({"error": "Internal server error"}), 503
    robot_mode_setter(mode)
    return jsonify({"status": "ok"})


def start(host: str = '0.0.0.0', port: int = 5000):
    server = make_server(host=host, port=port, app=app, threaded=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    start()
