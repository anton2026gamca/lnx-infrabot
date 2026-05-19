"""
Process tracker for registering process lifecycle events.
"""

import os
from .collector import get_collector


def register_process(name: str, process_id: int | None = None):
    """
    Register a process with the profiler.

    Should be called at the start of each process.

    Args:
        name: Name of the process
        process_id: Process ID (uses current PID if not provided)
    """
    if process_id is None:
        process_id = os.getpid()

    collector = get_collector()
    if collector:
        collector.register_process(name, process_id)


def unregister_process(name: str, process_id: int | None = None):
    """
    Unregister a process from the profiler.

    Should be called at the end of each process.

    Args:
        name: Name of the process
        process_id: Process ID (uses current PID if not provided)
    """
    if process_id is None:
        process_id = os.getpid()

    collector = get_collector()
    if collector:
        collector.unregister_process(name, process_id)
