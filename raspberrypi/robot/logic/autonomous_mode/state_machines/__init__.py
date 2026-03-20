from . import soccer, circle, square, n_directional
from robot.logic.autonomous_mode.state_machine import StateMachine


def list() -> list[StateMachine]:
    return [
        soccer.get_state_machine(),
        circle.get_state_machine(),
        square.get_state_machine(),
        n_directional.get_state_machine(),
    ]

