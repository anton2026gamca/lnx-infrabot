"""
Decorator for selective function profiling.
Use @profile_function on functions you want to profile.
"""

from collections.abc import Callable
import asyncio
import functools
import time
import os
from .collector import get_collector


_MIN_PROFILE_DURATION_S = 0.00002


def _record_function_event(collector, name: str, module: str, duration: float) -> None:
    if not collector or not collector.is_collecting:
        return
    if duration < _MIN_PROFILE_DURATION_S:
        return
    pid = os.getpid()
    collector.collect_function_event(
        name=name,
        module=module,
        duration=duration,
        process_id=pid,
    )


def profile_function(func: Callable) -> Callable:
    """
    Decorator to profile a function's execution time.

    Usage:
        @profile_function
        def my_function():
            ...
    """

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        collector = get_collector()
        if not collector or not collector.is_collecting:
            return func(*args, **kwargs)

        start_time = time.time()

        try:
            result = func(*args, **kwargs)
            return result
        finally:
            duration = time.time() - start_time
            _record_function_event(collector, func.__qualname__, func.__module__, duration)

    return wrapper


def sleep(duration: float) -> None:
    collector = get_collector()
    if not collector or not collector.is_collecting:
        time.sleep(duration)
        return

    start_time = time.time()
    try:
        time.sleep(duration)
    finally:
        _record_function_event(collector, "sleep", "time", time.time() - start_time)


async def async_sleep(duration: float) -> None:
    collector = get_collector()
    if not collector or not collector.is_collecting:
        await asyncio.sleep(duration)
        return

    start_time = time.time()
    try:
        await asyncio.sleep(duration)
    finally:
        _record_function_event(collector, "sleep", "asyncio", time.time() - start_time)
