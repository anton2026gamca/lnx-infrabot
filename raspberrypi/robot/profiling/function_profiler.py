"""
Decorator for selective function profiling.
Use @profile_function on functions you want to profile.
"""

from collections.abc import Callable
import functools
import time
import os
from .collector import get_collector


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

            if collector and duration > 0.001:
                module = func.__module__
                name = func.__qualname__
                pid = os.getpid()

                collector.collect_function_event(
                    name=name,
                    module=module,
                    duration=duration,
                    process_id=pid,
                )

    return wrapper

