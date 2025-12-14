from flask import Flask, Response, request
from picamera2 import Picamera2
import cv2
import time

app = Flask(__name__)


picam = Picamera2()

SENSOR_W = 4608
SENSOR_H = 2592
DEFAULT_QUALITY = 4


camera_config = picam.create_preview_configuration(
    main={"format": "XRGB8888", "size": (SENSOR_W, SENSOR_H)}
)
picam.configure(camera_config)
picam.start()


def gen_frames(quality=DEFAULT_QUALITY, target_fps=None):
    try:
        q = int(quality)
    except Exception:
        q = DEFAULT_QUALITY
    if q < 1: q = 1
    if q > 5: q = 5

    target_w = max(1, SENSOR_W // q)
    target_h = max(1, SENSOR_H // q)

    sleep_time = None
    if target_fps and target_fps > 0:
        sleep_time = 1.0 / float(target_fps)

    while True:
        if sleep_time:
            time.sleep(sleep_time)
        frame = picam.capture_array()
        if (frame.shape[1], frame.shape[0]) != (target_w, target_h):
            frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/video_feed')
def video_feed():
    q = request.args.get('q', default=DEFAULT_QUALITY, type=int)
    if q is None: q = DEFAULT_QUALITY
    if q < 1 or q > 5: q = DEFAULT_QUALITY

    if q == 1: fps = 5
    elif q == 2: fps = 10
    else: fps = None

    return Response(gen_frames(quality=q, target_fps=fps), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5000)
