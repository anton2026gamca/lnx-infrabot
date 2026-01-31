from dataclasses import dataclass
from picamera2 import Picamera2
import time



@dataclass
class FrameData:
    frame: bytes
    timestamp: float


picam = None

SENSOR_W = 4608
SENSOR_H = 2592
RESOLUTION_SCALE = 0.25


def init_camera():
    """Must be called from within the process that will use it."""
    global picam
    picam = Picamera2()
    camera_config = picam.create_preview_configuration(
        main={"format": "RGB888", "size": (int(SENSOR_W * RESOLUTION_SCALE), int(SENSOR_H * RESOLUTION_SCALE))}
    )
    picam.configure(camera_config)
    picam.start()


def get_frame() -> FrameData:
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    return FrameData(frame=picam.capture_array(), timestamp=time.time())
