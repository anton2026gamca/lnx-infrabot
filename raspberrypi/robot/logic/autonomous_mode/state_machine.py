from dataclasses import dataclass
import time

from robot import utils
from robot.hardware.motors import SmartMotorsController
from robot.profiling.function_profiler import profile_function


logger = utils.get_logger("State Machine")


class State:
    def on_enter(self, state_machine: "StateMachine") -> None:
        pass

    def on_exit(self, state_machine: "StateMachine") -> None:
        pass

    def tick(self, state_machine: "StateMachine") -> None:
        pass


@dataclass
class CrossStateData:
    pass


class StateMachine:
    name: str
    initial_state: type[State]
    current_state: State
    state_start_time: float
    queued_transition: type[State] | None
    cross_state_data: CrossStateData | None

    def __init__(self, name: str, initial_state: type[State], motors: SmartMotorsController, cross_state_data: CrossStateData | None = None):
        self.name = name

        self.queued_transition = None
        self.cross_state_data = cross_state_data
        
        self.initial_state = initial_state
        self.motors = motors

        self.state_start_time = time.time()
        self.current_state = initial_state()
        self.current_state.on_enter(self)

    @profile_function
    def tick(self):
        if self.current_state is not None:
            self.current_state.tick(self)
        if self.queued_transition is not None:
            self._do_queued_transition()

    @profile_function
    def transition(self, new_state: type[State]):
        self.queued_transition = new_state

    def reset(self):
        if self.current_state is not None:
            self.current_state.on_exit(self)
        self.__init__(self.name, self.initial_state, self.motors)

    def time_in_current_state(self) -> float:
        return time.time() - self.state_start_time

    def _do_queued_transition(self) -> None:
        if self.queued_transition is None:
            return
        if self.current_state is not None:
            self.current_state.on_exit(self)
        self.state_start_time = time.time()
        self.current_state = self.queued_transition()
        self.current_state.on_enter(self)
        self.queued_transition = None

