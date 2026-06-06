from __future__ import annotations

from robot import utils
from robot.calibration.data_manager import save_calibration_data
from robot.multiprocessing import shared_data


logger = utils.get_logger("Camera Calibration")


def set_camera_settings(
    color_gains: tuple[float, float] | list[float] | None = None,
    exposure_time: int | float | None = None,
    analogue_gain: float | None = None,
    camera: str = "both",
) -> dict:
    if color_gains is not None:
        if not isinstance(color_gains, (list, tuple)) or len(color_gains) != 2:
            raise ValueError("color_gains must contain exactly 2 values")
        if not all(isinstance(v, (int, float)) and v > 0 for v in color_gains):
            raise ValueError("color_gains values must be positive numbers")

    if exposure_time is not None:
        if not isinstance(exposure_time, (int, float)) or exposure_time <= 0:
            raise ValueError("exposure_time must be a positive number")

    if analogue_gain is not None:
        if not isinstance(analogue_gain, (int, float)) or analogue_gain <= 0:
            raise ValueError("analogue_gain must be a positive number")

    shared_data.set_camera_settings(
        color_gains=[float(color_gains[0]), float(color_gains[1])] if color_gains is not None else None,
        exposure_time=float(exposure_time) if exposure_time is not None else None,
        analogue_gain=float(analogue_gain) if analogue_gain is not None else None,
        camera=camera,
    )
    save_calibration_data()

    if camera == "both":
        settings = {
            "front": shared_data.get_camera_settings("front"),
            "back": shared_data.get_camera_settings("back"),
        }
    else:
        settings = shared_data.get_camera_settings(camera)

    logger.info(f"Camera settings updated for {camera}")
    return settings


def get_camera_settings(camera: str = "front") -> dict:
    if camera == "both":
        return {
            "front": shared_data.get_camera_settings("front"),
            "back": shared_data.get_camera_settings("back"),
        }
    return shared_data.get_camera_settings(camera)
