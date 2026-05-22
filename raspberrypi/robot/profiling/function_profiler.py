"""
Decorator for selective function profiling.
Use @profile_function on functions you want to profile.
"""

from collections.abc import Awaitable, Callable
import asyncio
import functools
import inspect
import time
import os
from typing import Any, ParamSpec, TypeVar, cast
from .collector import get_collector


_MIN_PROFILE_DURATION_S = 0.00002
P = ParamSpec("P")
R = TypeVar("R")


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


def profile_function(func: Callable[P, R]) -> Callable[P, R]:
    """
    Decorator to profile a function's execution time.

    Usage:
        @profile_function
        def my_function():
            ...
    """

    if inspect.iscoroutinefunction(func):
        async_func = cast(Callable[P, Awaitable[Any]], func)

        @functools.wraps(func)
        async def async_wrapper(*args: P.args, **kwargs: P.kwargs) -> Any:
            collector = get_collector()
            if not collector or not collector.is_collecting:
                return await async_func(*args, **kwargs)

            start_time = time.time()

            try:
                return await async_func(*args, **kwargs)
            finally:
                duration = time.time() - start_time
                _record_function_event(collector, func.__qualname__, func.__module__, duration)

        return cast(Callable[P, R], async_wrapper)

    sync_func = cast(Callable[P, R], func)

    @functools.wraps(func)
    def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
        collector = get_collector()
        if not collector or not collector.is_collecting:
            return sync_func(*args, **kwargs)

        start_time = time.time()

        try:
            return sync_func(*args, **kwargs)
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
