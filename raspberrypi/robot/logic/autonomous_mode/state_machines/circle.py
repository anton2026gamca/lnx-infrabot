from dataclasses import dataclass

from robot import utils
from robot.hardware.motors import SmartMotorsController
from robot.logic.autonomous_mode.state_machine import State, StateMachine
from robot.config import *



@dataclass
class CircleState(State):
    def tick(self, state_machine: StateMachine) -> None:
        revolution_time = 2
        angle = utils.normalize_angle_deg((state_machine.time_in_current_state() / revolution_time) * 360)
        speed = AUTO_SPEED_MULTIPLIER
        state_machine.motors.set_motors(angle=angle, speed=speed, rotate=0)


_state_machine = StateMachine(name="Circle State Machine", initial_state=CircleState, motors=SmartMotorsController())

def get_state_machine() -> StateMachine:
    return _state_machine
