import time
from dataclasses import dataclass



class RobotMode:
    IDLE       = 0
    MANUAL     = 1
    AUTONOMOUS = 2

@dataclass
class RobotManualControl:
    move_angle: float = 0.0
    move_speed: float = 0.0
    rotate: float = 0.0

import robot.calibration as calibration
import robot.multiprocessing.process_manager as process_manager
import robot.multiprocessing.shared_data as shared_data
import robot.utils as utils
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
        logger.info("Stopping...")

    process_manager.stop_all_processes()

    shared_data.cleanup()

if __name__ == "__main__":
    main()

