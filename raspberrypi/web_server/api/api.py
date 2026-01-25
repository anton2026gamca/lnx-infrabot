from flask import Flask, Response, request
import cv2
import time

app = Flask(__name__)

camera_frame = None


def set_camera_frame(frame):
    global camera_frame
    camera_frame = frame

def gen_frames():
    target_fps = 30
    sleep_time = None
    if target_fps and target_fps > 0:
        sleep_time = 1.0 / float(target_fps)

    while True:
        if sleep_time:
            time.sleep(sleep_time)
        ret, buffer = cv2.imencode('.jpg', camera_frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')



def start():
    app.run(host='127.0.0.1', port=5000)
