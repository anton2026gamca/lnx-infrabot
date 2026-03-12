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
        last_debug_msg_time = time.time()

        logger.info("Initializing camera...")
        camera.init()
        logger.info("Camera initialized successfully")
        
        while not stop_event.is_set():
            try:
                frame = camera.capture_frame()
                shared_data.set_camera_frame(frame)
                captured_frames += 1
            except Exception as e:
                logger.error(f"Error capturing frame: {e}", exc_info=True)
                time.sleep(0.001)
                continue
            
            if time.time() > last_debug_msg_time + 1:
                logger.debug(f"Camera Capture FPS: {captured_frames}")
                captured_frames = 0
                last_debug_msg_time += 1
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"{e}", exc_info=True)
        try:
            black_frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
            shared_data.set_camera_frame(FrameData(frame=black_frame, timestamp=time.time()))
        except Exception | KeyboardInterrupt:
            pass


