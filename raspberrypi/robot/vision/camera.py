import time
import numpy as np
from dataclasses import dataclass

try:
    from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
except ImportError:
    Picamera2 = None

from robot import utils
from robot.config import *



_logger = utils.get_logger("camera")

@dataclass
class FrameData:
    frame: np.ndarray
    timestamp: float

_picams: dict = {}


def init(camera_name: str = "front", camera_index: int | None = None):
    """Must be called from within the process that will use it."""
    if Picamera2 is None:
        raise ImportError("Picamera2 library not found.")
    if camera_index is None:
        camera_index = CAMERA_FRONT_INDEX
    picam = Picamera2(camera_num=int(camera_index))
    camera_config = picam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": CAMERA_MAX_FPS},
        buffer_count = CAMERA_BUFFER_COUNT,
        queue = False,
    )
    picam.configure(camera_config)
    picam.set_controls({
        "AwbEnable": False,
        "AeEnable": False,
        "ColourGains": (1.84, 2.05),
        "ExposureTime": 10000,
        "AnalogueGain": 3.0
    })
    picam.start()
    _picams[camera_name] = picam


def capture_frame(camera_name: str = "front") -> FrameData:
    picam = _picams.get(camera_name)
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    frame_rgb = picam.capture_array()
    return FrameData(frame=frame_rgb, timestamp=time.time())


def calibrate_color_gains(camera_name: str = "front") -> tuple[int, int] | None:
    """
    Enables AWB momentarily to find the best ratios for current lighting,
    then locks them in. Best used when pointing at a white/gray card.
    """
    picam = _picams.get(camera_name)
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")

    _logger.info(f"({camera_name.title()} Camera) Starting color calibration (AWB enabled)...")
    
    picam.set_controls({"AwbEnable": True})
    
    time.sleep(2.0)
    
    metadata = picam.capture_metadata()
    gains = metadata.get("ColourGains")
    
    if gains:
        red_gain, blue_gain = gains[0], gains[1]
        picam.set_controls({
            "AwbEnable": False,
            "ColourGains": (red_gain, blue_gain)
        })
        _logger.info(f"({camera_name.title()} Camera) Calibration complete. Locked Gains -> Red: {red_gain:.3f}, Blue: {blue_gain:.3f}")
        return (red_gain, blue_gain)
    else:
        _logger.warning(f"({camera_name.title()} Camera) Calibration failed: Could not retrieve ColourGains metadata.")
        return None

