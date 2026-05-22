import logging
import multiprocessing.synchronize
import numpy as np
import threading
import time

from robot import utils
from robot import calibration
from robot.multiprocessing import shared_data
from robot.profiling import profile_function, sleep
from robot.vision import camera

from robot.vision.camera import FrameData
from robot.config import *



@profile_function
def _initialize_cameras(logger: logging.Logger) -> list[str]:
    available_cameras: list[str] = []
    for camera_name, camera_index in [("front", CAMERA_FRONT_INDEX), ("back", CAMERA_BACK_INDEX)]:
        try:
            logger.info(f"Initializing {camera_name} camera (index {camera_index})...")
            camera.init(camera_name=camera_name, camera_index=camera_index)
            available_cameras.append(camera_name)
            logger.info(f"{camera_name.capitalize()} camera initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize {camera_name} camera: {e}", exc_info=True)
    return available_cameras


@profile_function
def _handle_auto_calibration(
    calibration_request: dict,
    available_cameras: list[str],
    logger: logging.Logger,
) -> None:
    request_id = int(calibration_request.get("request_id", 0))
    target_camera = str(calibration_request.get("camera", "front")).lower()
    settle_time_s = float(calibration_request.get("settle_time_s", 2.0))

    try:
        if target_camera not in available_cameras:
            raise RuntimeError(f"{target_camera} camera is not available")

        calibration_result = camera.calibrate_auto_controls(target_camera, settle_time_s)

        for camera_name in available_cameras:
            if camera_name == target_camera:
                continue
            camera.apply_auto_calibration_result(camera_name, calibration_result)

        for camera_name in available_cameras:
            shared_data.set_camera_settings(
                color_gains=calibration_result.get("color_gains"),
                exposure_time=calibration_result.get("exposure_time"),
                analogue_gain=calibration_result.get("analogue_gain"),
                camera=camera_name,
            )
        calibration.save_calibration_data()

        shared_data.set_camera_auto_calibration_result(
            request_id=request_id,
            success=True,
            result={"camera": target_camera, "result": calibration_result},
        )
    except Exception as e:
        logger.error(f"Camera auto calibration failed: {e}", exc_info=True)
        shared_data.set_camera_auto_calibration_result(
            request_id=request_id,
            success=False,
            error=str(e),
        )


@profile_function
def _handle_manual_camera_settings_update(
    request: dict,
    available_cameras: list[str],
    logger: logging.Logger,
) -> None:
    request_id = int(request.get("request_id", 0))
    target_camera = str(request.get("camera", "both")).lower()
    target_cameras = available_cameras if target_camera == "both" else [target_camera]
    color_gains = request.get("color_gains")
    exposure_time = request.get("exposure_time")
    analogue_gain = request.get("analogue_gain")

    try:
        for camera_name in target_cameras:
            if camera_name not in available_cameras:
                raise RuntimeError(f"{camera_name} camera is not available")
            camera.set_manual_controls(
                camera_name=camera_name,
                color_gains=color_gains,
                exposure_time=exposure_time,
                analogue_gain=analogue_gain,
            )

        calibration.save_calibration_data()
        applied_settings = {
            camera_name: shared_data.get_camera_settings(camera_name)
            for camera_name in target_cameras
        }
        shared_data.set_camera_settings_update_result(
            request_id=request_id,
            success=True,
            settings=applied_settings,
        )
    except Exception as e:
        logger.error(f"Manual camera settings update failed: {e}", exc_info=True)
        shared_data.set_camera_settings_update_result(
            request_id=request_id,
            success=False,
            error=str(e),
        )


@profile_function
def _capture_camera_loop(
    camera_name: str,
    stop_event: multiprocessing.synchronize.Event,
    shutdown_event: threading.Event,
    pause_event: threading.Event,
    capture_counts: dict[str, int],
    capture_counts_lock: threading.Lock,
    logger: logging.Logger,
) -> None:
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            if pause_event.is_set():
                sleep(0.001)
                continue
            start_t = time.perf_counter()
            frame = camera.capture_frame(camera_name=camera_name)
            shared_data.set_camera_frame(frame, camera_name=camera_name)
            with capture_counts_lock:
                capture_counts[camera_name] = capture_counts.get(camera_name, 0) + 1
            end_t = time.perf_counter()
            elapsed = end_t - start_t
            sleep_t = 1.0 / CAMERA_MAX_FPS - elapsed
            if sleep_t > 0:
                sleep(sleep_t)
        except Exception as e:
            logger.error(f"Error capturing {camera_name} frame: {e}", exc_info=True)
            shared_data.set_camera_frame(None, camera_name=camera_name)
            sleep(0.001)


def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    shutdown_event = threading.Event()
    pause_event = threading.Event()
    capture_threads: list[threading.Thread] = []
    try:
        if utils.get_default_logger_level() == logging.DEBUG:
            logging.getLogger("picamera2.picamera2").setLevel(logging.INFO)

        last_debug_msg_time = time.perf_counter()

        available_cameras = _initialize_cameras(logger)

        if not available_cameras:
            raise RuntimeError("No cameras available")

        capture_counts: dict[str, int] = {camera_name: 0 for camera_name in available_cameras}
        capture_counts_lock = threading.Lock()

        for camera_name in available_cameras:
            capture_thread = threading.Thread(
                target=_capture_camera_loop,
                name=f"camera-capture-{camera_name}",
                args=(
                    camera_name,
                    stop_event,
                    shutdown_event,
                    pause_event,
                    capture_counts,
                    capture_counts_lock,
                    logger,
                ),
                daemon=True,
            )
            capture_thread.start()
            capture_threads.append(capture_thread)
        
        while not stop_event.is_set():
            camera_settings_request = shared_data.claim_camera_settings_update_request()
            if camera_settings_request:
                pause_event.set()
                try:
                    _handle_manual_camera_settings_update(camera_settings_request, available_cameras, logger)
                finally:
                    pause_event.clear()

            calibration_request = shared_data.claim_camera_auto_calibration_request()
            if calibration_request:
                pause_event.set()
                try:
                    _handle_auto_calibration(calibration_request, available_cameras, logger)
                finally:
                    pause_event.clear()

            if time.perf_counter() > last_debug_msg_time + 1:
                with capture_counts_lock:
                    camera_fps = {
                        camera_name: capture_counts.get(camera_name, 0)
                        for camera_name in available_cameras
                    }
                    for camera_name in available_cameras:
                        capture_counts[camera_name] = 0
                camera_fps_msg = ", ".join(
                    f"{camera_name}={camera_fps.get(camera_name, 0)}"
                    for camera_name in available_cameras
                )
                logger.debug(f"Camera Capture FPS: {camera_fps_msg}")
                last_debug_msg_time = time.perf_counter()
            sleep(0.001)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"{e}", exc_info=True)
        try:
            black_frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
            for camera_name in ["front", "back"]:
                shared_data.set_camera_frame(FrameData(frame=black_frame, timestamp=time.time()), camera_name=camera_name)
        except Exception | KeyboardInterrupt:
            pass
    finally:
        shutdown_event.set()
        for capture_thread in capture_threads:
            capture_thread.join(timeout=1.0)
