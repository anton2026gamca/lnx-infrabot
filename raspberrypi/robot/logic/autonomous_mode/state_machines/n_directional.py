from dataclasses import dataclass

from robot import utils
from robot.hardware.motors import SmartMotorsController
from robot.logic.autonomous_mode.state_machine import State, StateMachine, CrossStateData
from robot.config import *


logger = utils.get_logger("Square State Machine")

PAUSE_TIME = 1.0
DRIVE_TIME = 0.5

NUM_DIRS = 8


@dataclass
class SquareCrossStateData(CrossStateData):
    direction: int

class SquareState(State):
    def tick(self, state_machine: StateMachine) -> None:
        if not isinstance(state_machine.cross_state_data, SquareCrossStateData):
            state_machine.cross_state_data = SquareCrossStateData(direction=0)

        if state_machine.time_in_current_state() < PAUSE_TIME:
            state_machine.motors.set_motors(angle=0, speed=0, rotate=0)
            return

        if state_machine.time_in_current_state() >= PAUSE_TIME + DRIVE_TIME:
            state_machine.cross_state_data.direction = (state_machine.cross_state_data.direction + 1) % NUM_DIRS
            state_machine.transition(SquareState)
            return

        angle = (state_machine.cross_state_data.direction // 2) * (360 / NUM_DIRS)
        if state_machine.cross_state_data.direction % 2 == 1:
            angle += 180
        speed = AUTO_SPEED_MULTIPLIER

        state_machine.motors.set_motors(angle=angle, speed=speed, rotate=0)

        logger.info(f"Direction: {state_machine.cross_state_data.direction}, Angle: {angle}, Speed: {speed}")


_state_machine = StateMachine(name=f"{NUM_DIRS}-directional State Machine", initial_state=SquareState, motors=SmartMotorsController())

def get_state_machine() -> StateMachine:
    return _state_machine

