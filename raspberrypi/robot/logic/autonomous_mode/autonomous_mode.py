from robot.multiprocessing import shared_data
from . import state_machines
from .state_machine import StateMachine
from robot import utils


logger = utils.get_logger("Autonomous Mode")


current_state_machine: StateMachine | None = state_machines.list()[0]

shared_data.set_current_state_machine_name(current_state_machine.name if current_state_machine else "")


def tick() -> None:
    if current_state_machine is not None:
        current_state_machine.tick()

def check_state_machine_change_request() -> None:
    req = shared_data.check_state_machine_change_request()
    if req != "":
        _set_current_state_machine_internal(req)

def set_current_state_machine(state_machine: StateMachine | str) -> None:
    name = state_machine.name if isinstance(state_machine, StateMachine) else state_machine
    shared_data.request_state_machine_change(name)

def get_current_state_machine() -> StateMachine | None:
    name = shared_data.get_current_state_machine_name()
    return find_state_machine_by_name(name)

def get_current_state_machine_name() -> str:
    name = shared_data.get_current_state_machine_name()
    return name if name else ""

def get_available_state_machines() -> dict[str, StateMachine]:
    return {
        state_machine.name: state_machine
        for state_machine in state_machines.list()
    }

def find_state_machine_by_name(name: str) -> StateMachine | None:
    return get_available_state_machines().get(name)

def _set_current_state_machine_internal(name: str) -> None:
    global current_state_machine
    state_machine = find_state_machine_by_name(name)
    if state_machine is not None:
        current_state_machine = state_machine
        shared_data.set_current_state_machine_name(name)
    else:
        logger.warning(f"State machine with name '{name}' not found")

