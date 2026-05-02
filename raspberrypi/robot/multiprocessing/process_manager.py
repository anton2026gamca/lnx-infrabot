import logging
import multiprocessing

from robot import utils
from robot.api import api_process
from robot.bluetooth import bluetooth_process
from robot.hardware import hardware_process
from robot.logic import logic_process
from robot.vision import camera_capture_process, camera_processing_process



logger = utils.get_logger("Process Manager")


class Process:
    def __init__(self, name: str, function, logger: logging.Logger | None = None):
        self.name = name
        self.function = function
        self.stop_event = multiprocessing.Event()
        self.logger = logger

    def run(self):
        if self.logger is None:
            self.logger = utils.get_logger(self.name)
        try:
            self.function(stop_event=self.stop_event, logger=self.logger)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.logger.error(f"{e}", exc_info=True)

    def stop(self):
        self.stop_event.set()




processes: list[Process] = [
    Process("Hardware Process", hardware_process.run),
    Process("Camera Capture Process", camera_capture_process.run),
    Process("Camera Processing Process", camera_processing_process.run),
    Process("Logic Process", logic_process.run),
    Process("Bluetooth Process", bluetooth_process.run),
    Process("API Process", api_process.run),
]


running_processes: dict[Process, multiprocessing.Process] = {}

def get_processes_list() -> list[multiprocessing.Process]:
    process_list = []

    for process in processes:
        p = multiprocessing.Process(target=process.run, name=process.name)
        process_list.append(p)

    return process_list

def start_process(process: Process) -> multiprocessing.Process:
    logger.info(f"Starting Process \"{process.name}\"")
    p = multiprocessing.Process(target=process.run, name=process.name)
    p.start()
    running_processes[process] = p
    return p

def stop_process(process: Process) -> None:
    p = running_processes.get(process)
    if p is None:
        logger.warning(f"Process {process.name} is not running.")
        return
    logger.debug(f"Stopping process: {p.name}")
    process.stop()
    p.join(timeout=5)
    if p.is_alive():
        logger.warning(f"Process {p.name} did not terminate, forcing...")
        p.terminate()
        p.join(timeout=2)

def start_all_processes():
    for process in processes:
        if process is None:
            continue
        start_process(process)

def stop_all_processes():
    processes.reverse()
    for process in processes:
        if process is None:
            continue
        stop_process(process)
    processes.reverse()

