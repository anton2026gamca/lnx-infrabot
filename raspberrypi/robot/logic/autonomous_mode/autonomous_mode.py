from __future__ import annotations

from robot.calibration.data_manager import save_calibration_data
from robot.multiprocessing import shared_data
from robot.multiprocessing.shared_data import AutonomousStatusText
from robot.profiling import profile_function
from . import state_machines
from .state_machine import StateMachine
from .state_machines import soccer
from robot import utils
import time


logger = utils.get_logger("Autonomous Mode")


current_state_machine: StateMachine | None = state_machines.list()[0]

shared_data.set_current_state_machine_name(current_state_machine.name if current_state_machine else "")


@profile_function
def tick() -> None:
    if current_state_machine is not None:
        current_state_machine.tick()

@profile_function
def check_state_machine_change_request() -> None:
    req = shared_data.check_state_machine_change_request()
    if req != "":
        _set_current_state_machine_internal(req)

@profile_function
def set_current_state_machine(state_machine: StateMachine | str) -> None:
    name = state_machine.name if isinstance(state_machine, StateMachine) else state_machine
    shared_data.request_state_machine_change(name)

@profile_function
def get_current_state_machine() -> StateMachine | None:
    name = shared_data.get_current_state_machine_name()
    return find_state_machine_by_name(name)

@profile_function
def get_current_state_machine_name() -> str:
    name = shared_data.get_current_state_machine_name()
    return name if name else ""

@profile_function
def get_available_state_machines() -> dict[str, StateMachine]:
    return {
        state_machine.name: state_machine
        for state_machine in state_machines.list()
    }

@profile_function
def find_state_machine_by_name(name: str) -> StateMachine | None:
    return get_available_state_machines().get(name)

@profile_function
def _set_current_state_machine_internal(name: str) -> None:
    global current_state_machine
    state_machine = find_state_machine_by_name(name)
    if state_machine is not None:
        current_state_machine = state_machine
        shared_data.set_current_state_machine_name(name)
        _reset_state_machine_runtime(state_machine)
        save_calibration_data()
    else:
        logger.warning(f"State machine with name '{name}' not found")


@profile_function
def reset_current_state_machine() -> None:
    if current_state_machine is not None:
        _reset_state_machine_runtime(current_state_machine)


@profile_function
def _reset_state_machine_runtime(state_machine: StateMachine) -> None:
    if state_machine.current_state is not None:
        state_machine.current_state.on_exit(state_machine)

    state_machine.queued_transition = None
    state_machine.state_start_time = time.time()
    state_machine.current_state = state_machine.initial_state()
    state_machine.current_state.on_enter(state_machine)

    if state_machine.name in (soccer.ATTACKER_STATE_MACHINE_NAME, soccer.GOALKEEPER_STATE_MACHINE_NAME):
        soccer.reset_role_to_preset(state_machine, force_state_transition=False)
    else:
        shared_data.set_autonomous_status_text(AutonomousStatusText.DEFAULT)
