import time
from dataclasses import dataclass


# Define classes and data structures before importing other robot modules to avoid circular import errors

class RobotMode:
    IDLE       = 0
    MANUAL     = 1
    AUTONOMOUS = 2

@dataclass
class RobotManualControl:
    move_angle: float = 0.0
    move_speed: float = 0.0
    rotate: float = 0.0


# Then import multiprocessing.shared_data to ensure it's initialized before we import utils.logging
from robot.multiprocessing import shared_data

from robot import utils, calibration
from robot.multiprocessing import process_manager
from robot.config import *


logger = utils.get_logger()


def main():
    shared_data.set_robot_mode(RobotMode.IDLE)
    calibration.load_calibration_data()

    process_manager.start_all_processes()
    
    logger.info("System running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    logger.info("Stopping...")

    process_manager.stop_all_processes()

    shared_data.cleanup()

if __name__ == "__main__":
    main()

