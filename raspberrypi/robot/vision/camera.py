import cv2
import time
import numpy as np
from dataclasses import dataclass
from sys import exception

try:
    from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
except ImportError:
    Picamera2 = None

from robot.config import *
from robot.vision.visualizer import DetectedObject



@dataclass
class FrameData:
    frame: np.ndarray
    timestamp: float

picam = None


def init():
    """Must be called from within the process that will use it."""
    if Picamera2 is None:
        raise ImportError("Picamera2 library not found.")
    global picam
    picam = Picamera2()
    camera_config = picam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "YUV420"},
        lores={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "YUV420"},
        raw={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "SRGGB10_CSI2P"},
        controls={"FrameRate": CAMERA_MAX_FPS},
        buffer_count = CAMERA_BUFFER_COUNT,
        queue = False,
    )
    picam.configure(camera_config)
    picam.start()

def capture_frame() -> FrameData:
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    frame_yuv = picam.capture_array("lores")
    frame_rgb = _yuv420_to_rgb(frame_yuv, FRAME_WIDTH, FRAME_HEIGHT)
    return FrameData(frame=frame_rgb, timestamp=time.time())

def _yuv420_to_rgb(yuv_frame, width, height):
    yuv = yuv_frame.reshape((height * 3 // 2, width))
    rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
    return rgb

