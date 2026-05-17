import logging
import multiprocessing.synchronize
import numpy as np
import time

from robot import utils
from robot.multiprocessing import shared_data
from robot.vision import camera

from robot.vision.camera import FrameData
from robot.config import *



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    try:
        if utils.get_default_logger_level() == logging.DEBUG:
            logging.getLogger("picamera2.picamera2").setLevel(logging.INFO)

        captured_frames = 0
        last_debug_msg_time = time.perf_counter()

        available_cameras: list[str] = []
        for camera_name, camera_index in [("front", CAMERA_FRONT_INDEX), ("back", CAMERA_BACK_INDEX)]:
            try:
                logger.info(f"Initializing {camera_name} camera (index {camera_index})...")
                camera.init(camera_name=camera_name, camera_index=camera_index)
                available_cameras.append(camera_name)
                logger.info(f"{camera_name.capitalize()} camera initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize {camera_name} camera: {e}", exc_info=True)

        if not available_cameras:
            raise RuntimeError("No cameras available")
        
        while not stop_event.is_set():
            had_capture_error = False
            for camera_name in available_cameras:
                try:
                    frame = camera.capture_frame(camera_name=camera_name)
                    shared_data.set_camera_frame(frame, camera_name=camera_name)
                except Exception as e:
                    had_capture_error = True
                    logger.error(f"Error capturing {camera_name} frame: {e}", exc_info=True)
                    shared_data.set_camera_frame(None, camera_name=camera_name)
            captured_frames += 1
            if had_capture_error:
                time.sleep(0.001)
            
            if time.perf_counter() > last_debug_msg_time + 1:
                logger.debug(f"Camera Capture FPS: {captured_frames}")
                captured_frames = 0
                last_debug_msg_time = time.perf_counter()
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

