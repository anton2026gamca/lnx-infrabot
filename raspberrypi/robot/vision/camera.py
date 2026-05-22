import time
import numpy as np
from dataclasses import dataclass

try:
    from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
except ImportError:
    Picamera2 = None

from robot import utils
from robot.config import *
from robot.profiling import profile_function, sleep



_logger = utils.get_logger("camera")

@dataclass
class FrameData:
    frame: np.ndarray
    timestamp: float

_picams: dict = {}


@profile_function
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


@profile_function
def capture_frame(camera_name: str = "front") -> FrameData:
    picam = _picams.get(camera_name)
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    frame_rgb = picam.capture_array()
    return FrameData(frame=frame_rgb, timestamp=time.time())


@profile_function
def calibrate_auto_controls(camera_name: str = "front", settle_time_s: float = 2.0) -> dict:
    """
    Temporarily enables AWB and AE to adapt to current lighting,
    then locks the discovered values by disabling both again.
    """
    picam = _picams.get(camera_name)
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")
    if settle_time_s <= 0:
        raise ValueError("settle_time_s must be > 0")

    _logger.info(
        f"({camera_name.title()} Camera) Starting camera auto calibration "
        f"(AWB+AE enabled for {settle_time_s:.2f}s)..."
    )
    
    picam.set_controls({"AwbEnable": True, "AeEnable": True})
    
    sleep(settle_time_s)
    
    metadata = picam.capture_metadata()
    gains = metadata.get("ColourGains")
    exposure_time = metadata.get("ExposureTime")
    analogue_gain = metadata.get("AnalogueGain")

    controls: dict = {
        "AwbEnable": False,
        "AeEnable": False,
    }
    if gains and len(gains) >= 2:
        controls["ColourGains"] = (float(gains[0]), float(gains[1]))
    if exposure_time is not None:
        controls["ExposureTime"] = int(exposure_time)
    if analogue_gain is not None:
        controls["AnalogueGain"] = float(analogue_gain)
    picam.set_controls(controls)
    
    if gains:
        red_gain, blue_gain = float(gains[0]), float(gains[1])
        _logger.info(
            f"({camera_name.title()} Camera) Calibration complete. "
            f"Locked Gains -> Red: {red_gain:.3f}, Blue: {blue_gain:.3f}, "
            f"ExposureTime: {exposure_time}, AnalogueGain: {analogue_gain}"
        )
        return {
            "color_gains": [red_gain, blue_gain],
            "exposure_time": int(exposure_time) if exposure_time is not None else None,
            "analogue_gain": float(analogue_gain) if analogue_gain is not None else None,
            "settle_time_s": float(settle_time_s),
        }
    raise RuntimeError(f"({camera_name.title()} Camera) Calibration failed: Could not retrieve ColourGains metadata.")


@profile_function
def apply_auto_calibration_result(camera_name: str, calibration_result: dict) -> dict:
    picam = _picams.get(camera_name)
    if picam is None:
        raise RuntimeError("Camera not initialized. Call init_camera() first.")

    gains = calibration_result.get("color_gains")
    exposure_time = calibration_result.get("exposure_time")
    analogue_gain = calibration_result.get("analogue_gain")
    settle_time_s = calibration_result.get("settle_time_s")

    if not isinstance(gains, list) or len(gains) != 2:
        raise ValueError("calibration_result.color_gains must contain exactly 2 values")

    controls = {
        "AwbEnable": False,
        "AeEnable": False,
        "ColourGains": (float(gains[0]), float(gains[1])),
    }
    if exposure_time is not None:
        controls["ExposureTime"] = int(exposure_time)
    if analogue_gain is not None:
        controls["AnalogueGain"] = float(analogue_gain)

    picam.set_controls(controls)
    _logger.info(
        f"({camera_name.title()} Camera) Applied calibration result from reference camera. "
        f"Gains={controls['ColourGains']}, ExposureTime={controls.get('ExposureTime')}, "
        f"AnalogueGain={controls.get('AnalogueGain')}, settle_time_s={settle_time_s}"
    )
    return {
        "color_gains": [float(gains[0]), float(gains[1])],
        "exposure_time": int(exposure_time) if exposure_time is not None else None,
        "analogue_gain": float(analogue_gain) if analogue_gain is not None else None,
        "settle_time_s": float(settle_time_s) if settle_time_s is not None else None,
    }


@profile_function
def calibrate_color_gains(camera_name: str = "front") -> tuple[float, float] | None:
    result = calibrate_auto_controls(camera_name=camera_name, settle_time_s=2.0)
    if not result:
        return None
    gains = result.get("color_gains")
    if not isinstance(gains, list) or len(gains) != 2:
        return None
    return (float(gains[0]), float(gains[1]))
