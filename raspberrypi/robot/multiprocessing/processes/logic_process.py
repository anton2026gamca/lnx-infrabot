import logging
import multiprocessing.synchronize
import time
import robot.multiprocessing.shared_data as shared_data
import robot.hardware.motors as motors
from robot.brain import AutonomousController
from robot.robot import RobotMode
from robot.config import *



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    motors_controller = motors.SmartMotorsController()
    autonomous_controller = AutonomousController()

    while not stop_event.is_set():
        start_time = time.time()

        mode = shared_data.get_robot_mode()
        if mode == RobotMode.IDLE:
            motors_controller.reset()
            autonomous_controller.reset()
            time.sleep(IDLE_SLEEP_DURATION)
        elif mode == RobotMode.MANUAL:
            autonomous_controller.reset()
            control = shared_data.get_manual_control()
            motors_controller.set_motors(control.move_angle, control.move_speed, control.rotate)
        elif mode == RobotMode.AUTONOMOUS:
            autonomous_controller.tick(motors_controller)

        elapsed = time.time() - start_time
        sleep_duration = max(0.0, LOGIC_LOOP_PERIOD - elapsed - 0.001)
        if sleep_duration > 0:
            time.sleep(sleep_duration)

