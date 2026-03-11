import logging
import multiprocessing

import robot.multiprocessing.processes.api_process as api_process
import robot.multiprocessing.processes.camera_capture_process as camera_capture_process
import robot.multiprocessing.processes.camera_processing_process as camera_processing_process
import robot.multiprocessing.processes.hardware_process as hardware_process
import robot.multiprocessing.processes.logic_process as logic_process
import robot.utils as utils



logger = utils.get_logger("Process Manager")


class Process:
    def __init__(self, name: str, function, logger: logging.Logger | None = None):
        self.name = name
        self.function = function
        self.stop_event = multiprocessing.Event()
        self.stop_event.is_set()
        self.logger = utils.get_logger(f"{name}") if logger is None else logger

    def run(self):
        try:
            self.function(stop_event=self.stop_event, logger=self.logger)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.logger.error(f"{e}", exc_info=True)

    def stop(self):
        self.stop_event.set()




processes: list[Process] = [
    Process("API Process", api_process.run),
    Process("Camera Capture Process", camera_capture_process.run),
    Process("Camera Processing Process", camera_processing_process.run),
    Process("Hardware Process", hardware_process.run),
    Process("Logic Process", logic_process.run),
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
        p.join()

def start_all_processes():
    for process in processes:
        if process is None:
            continue
        start_process(process)

def stop_all_processes():
    for process in processes:
        if process is None:
            continue
        stop_process(process)

