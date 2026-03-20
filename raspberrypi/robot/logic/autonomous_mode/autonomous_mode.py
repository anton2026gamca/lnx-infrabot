from robot.multiprocessing import shared_data
from . import state_machines
from .state_machine import StateMachine
from robot import utils


logger = utils.get_logger("Autonomous Mode")


current_state_machine: StateMachine | None = state_machines.list()[0]


def tick() -> None:
    req = shared_data.check_state_machine_change_request()
    if req != "":
        _set_current_state_machine_internal(req)
    if current_state_machine is not None:
        current_state_machine.tick()

def set_current_state_machine(state_machine: StateMachine | str) -> None:
    name = state_machine.name if isinstance(state_machine, StateMachine) else state_machine
    shared_data.request_state_machine_change(name)

def get_current_state_machine() -> StateMachine | None:
    name = shared_data.get_current_state_machine_name()
    return find_state_machine_by_name(name)

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
    else:
        logger.warning(f"State machine with name '{name}' not found")

