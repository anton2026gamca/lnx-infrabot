"""
Lock wrapper for profiling lock contention and acquisition times.
Tracks which processes are waiting on which locks and for how long.
"""

import multiprocessing
import time
import os
from .collector import get_collector


class ProfiledLock:
    """
    Wrapper around multiprocessing.Lock that tracks contention and timing.
    Replaces multiprocessing.Lock() calls to enable profiling.
    """

    def __init__(self, name: str | None = None):
        self._lock = multiprocessing.Lock()
        self._name = name or f"lock_{id(self)}"
        self._acquire_time: float | None = None

    def acquire(self, block: bool = True, timeout: float | None = None) -> bool:
        collector = get_collector()
        if not collector or not collector.is_collecting:
            acquired = self._lock.acquire(block, timeout)
            if acquired:
                self._acquire_time = time.time()
            return acquired

        start_time = time.time()
        acquire_time = None

        if timeout == -1:
            acquired = self._lock.acquire(block)
        else:
            acquired = self._lock.acquire(block, timeout)

        if acquired:
            acquire_time = time.time() - start_time
            current_pid = os.getpid()

            if collector and acquire_time >= 0.00002:
                collector.collect_lock_event(
                    lock_name=self._name,
                    event_type="acquire",
                    timestamp=start_time,
                    duration=acquire_time,
                    process_id=current_pid,
                )

            self._acquire_time = time.time()
        elif collector and block:
            wait_time = time.time() - start_time
            if wait_time > 0.001:
                current_pid = os.getpid()
                collector.collect_lock_event(
                    lock_name=self._name,
                    event_type="blocked",
                    timestamp=start_time,
                    duration=wait_time,
                    process_id=current_pid,
                )

        return acquired

    def release(self):
        collector = get_collector()
        if not collector or not collector.is_collecting:
            self._lock.release()
            self._acquire_time = None
            return

        if collector and self._acquire_time is not None:
            hold_time = time.time() - self._acquire_time
            if hold_time > 0.001:
                current_pid = os.getpid()
                collector.collect_lock_event(
                    lock_name=self._name,
                    event_type="release",
                    timestamp=self._acquire_time,
                    duration=hold_time,
                    process_id=current_pid,
                )

        self._lock.release()
        self._acquire_time = None

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
        return False

    def get_lock(self):
        return self._lock


def create_profiled_lock(name: str | None = None) -> ProfiledLock:
    return ProfiledLock(name)

