from dataclasses import dataclass
from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
import time
import numpy as np
from config import FRAME_WIDTH, FRAME_HEIGHT, CAMERA_BUFFER_COUNT


@dataclass
class FrameData:
    frame: np.ndarray
    timestamp: float


picam = None


def init_camera():
    """Must be called from within the process that will use it."""
    global picam
    picam = Picamera2()
    camera_config = picam.create_preview_configuration(
        main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam.configure(camera_config)
    picam.start()


def get_frame() -> FrameData:
    """Get frame from camera. Returns numpy array for efficient memory operations."""
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    return FrameData(frame=picam.capture_array(), timestamp=time.time())
