from __future__ import annotations

import logging
import multiprocessing.synchronize
import time

from . import autonomous_mode
from robot.hardware.motors import SmartMotorsController
from robot.multiprocessing import shared_data
from robot.profiling import profile_function, sleep
from robot.robot import RobotMode
from robot.config import *



@profile_function
def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    motors_controller = SmartMotorsController()
    prev_mode = shared_data.get_robot_mode()

    ticks = 0
    last_update_time = time.perf_counter()

    while not stop_event.is_set():
        start_time = time.perf_counter()

        mode = shared_data.get_robot_mode()

        if mode == RobotMode.AUTONOMOUS and prev_mode != RobotMode.AUTONOMOUS:
            autonomous_mode.reset_current_state_machine()
    
        if mode == RobotMode.IDLE:
            motors_controller.reset()
            sleep(IDLE_SLEEP_DURATION)
        elif mode == RobotMode.MANUAL:
            control = shared_data.get_manual_control()
            motors_controller.set_motors(control.move_angle, control.move_speed, control.rotate)
        elif mode == RobotMode.AUTONOMOUS:
            autonomous_mode.tick()

        shared_data.get_bluetooth_new_received_messages()
        autonomous_mode.check_state_machine_change_request()

        ticks += 1

        if time.perf_counter() > last_update_time + 1:
            shared_data.set_process_fps(shared_data.ProfilingProcesses.LOGIC, ticks)
            ticks = 0
            last_update_time = time.perf_counter()

        elapsed = time.perf_counter() - start_time
        sleep_duration = max(0.0, LOGIC_LOOP_PERIOD - elapsed - 0.001)
        if sleep_duration > 0:
            sleep(sleep_duration)

        prev_mode = mode
