"""Queue-based cross-process collector for profiling events."""

import json
import multiprocessing
import os
import queue
import threading
import time
from collections import defaultdict, deque



MAX_TIMELINE_EVENTS = 5000
EVENT_QUEUE_MAXSIZE = 4000
REPORT_CACHE_TTL_S = 0.2


class Collector:
    def __init__(self):
        self._queue = multiprocessing.Queue(maxsize=EVENT_QUEUE_MAXSIZE)
        self._is_collecting = multiprocessing.Value("b", False)
        self._start_time = multiprocessing.Value("d", time.time())
        self._dropped_events = multiprocessing.Value("L", 0)

        self._local_lock = threading.Lock()
        self._function_events: deque[dict] = deque(maxlen=MAX_TIMELINE_EVENTS)
        self._lock_events: deque[dict] = deque(maxlen=MAX_TIMELINE_EVENTS)
        self._process_events: deque[dict] = deque(maxlen=MAX_TIMELINE_EVENTS)
        self._process_stats: dict[str, dict] = {}
        self._pid_to_name: dict[int, str] = {}
        self._total_function_events = 0
        self._total_lock_events = 0

        self._last_report_cache: dict | None = None
        self._last_report_cache_at = 0.0
        self._writer_pid = 0

    @property
    def start_time(self) -> float:
        return float(self._start_time.value)

    @start_time.setter
    def start_time(self, value: float) -> None:
        self._start_time.value = float(value)

    @property
    def is_collecting(self) -> bool:
        return bool(self._is_collecting.value)

    @is_collecting.setter
    def is_collecting(self, value: bool) -> None:
        self._is_collecting.value = bool(value)

    def _invalidate_cache(self) -> None:
        self._last_report_cache = None
        self._last_report_cache_at = 0.0

    def _enqueue_event(self, event: dict) -> bool:
        current_pid = os.getpid()
        if self._writer_pid != current_pid:
            try:
                self._queue.cancel_join_thread()
            except Exception:
                pass
            self._writer_pid = current_pid

        try:
            self._queue.put_nowait(event)
            return True
        except queue.Full:
            with self._dropped_events.get_lock():
                self._dropped_events.value += 1
            return False

    def register_process(self, process_name: str, process_id: int):
        self._enqueue_event(
            {
                "kind": "process_start",
                "process_name": process_name,
                "process_id": process_id,
                "timestamp": time.time(),
            }
        )

    def unregister_process(self, process_name: str, process_id: int):
        self._enqueue_event(
            {
                "kind": "process_stop",
                "process_name": process_name,
                "process_id": process_id,
                "timestamp": time.time(),
            }
        )

    def collect_function_event(
        self,
        name: str,
        module: str,
        duration: float,
        process_id: int,
        stack_trace: str = "",
    ):
        if not self.is_collecting:
            return
        self._enqueue_event(
            {
                "kind": "function",
                "name": name,
                "module": module,
                "start_time": time.time() - duration,
                "duration": duration,
                "process_id": process_id,
                "process_name": self._pid_to_name.get(process_id) or multiprocessing.current_process().name,
                "stack_trace": stack_trace,
            }
        )

    def collect_lock_event(
        self,
        lock_name: str,
        event_type: str,
        timestamp: float,
        duration: float,
        process_id: int,
        file_location: str = "",
        stack_trace: str = "",
    ):
        if not self.is_collecting:
            return
        self._enqueue_event(
            {
                "kind": "lock",
                "lock_name": lock_name,
                "event_type": event_type,
                "timestamp": timestamp,
                "duration": duration,
                "process_id": process_id,
                "process_name": self._pid_to_name.get(process_id) or multiprocessing.current_process().name,
                "file_location": file_location,
                "stack_trace": stack_trace,
            }
        )

    def _process_stats_for(self, process_name: str, process_id: int) -> dict:
        stats = self._process_stats.get(process_name)
        if stats is None:
            stats = {
                "process_name": process_name,
                "process_id": process_id,
                "start_time": 0.0,
                "stop_time": 0.0,
                "function_count": 0,
                "lock_events_count": 0,
                "top_functions": [],
            }
            self._process_stats[process_name] = stats
        return stats

    def _drain_events(self) -> None:
        drained_any = False
        while True:
            try:
                event = self._queue.get_nowait()
            except queue.Empty:
                break

            drained_any = True
            kind = event.get("kind")
            process_id = int(event.get("process_id", 0))
            process_name = str(event.get("process_name", "unknown"))

            if kind == "process_start":
                self._pid_to_name[process_id] = process_name
                stats = self._process_stats_for(process_name, process_id)
                stats["process_id"] = process_id
                stats["start_time"] = float(event.get("timestamp", time.time()))
                stats["stop_time"] = 0.0
                if self.is_collecting:
                    self._process_events.append(
                        {
                            "process_name": process_name,
                            "event_type": "start",
                            "timestamp": float(event.get("timestamp", time.time())),
                            "process_id": process_id,
                        }
                    )
                continue

            if kind == "process_stop":
                self._pid_to_name[process_id] = process_name
                stats = self._process_stats_for(process_name, process_id)
                stats["stop_time"] = float(event.get("timestamp", time.time()))
                if self.is_collecting:
                    self._process_events.append(
                        {
                            "process_name": process_name,
                            "event_type": "stop",
                            "timestamp": float(event.get("timestamp", time.time())),
                            "process_id": process_id,
                        }
                    )
                continue

            if kind == "function":
                self._function_events.append(
                    {
                        "name": str(event.get("name", "")),
                        "module": str(event.get("module", "")),
                        "start_time": float(event.get("start_time", 0.0)),
                        "duration": float(event.get("duration", 0.0)),
                        "process_id": process_id,
                        "process_name": process_name,
                        "stack_trace": str(event.get("stack_trace", "")),
                    }
                )
                self._total_function_events += 1
                stats = self._process_stats_for(process_name, process_id)
                stats["function_count"] = int(stats.get("function_count", 0)) + 1
                continue

            if kind == "lock":
                self._lock_events.append(
                    {
                        "lock_name": str(event.get("lock_name", "")),
                        "event_type": str(event.get("event_type", "")),
                        "timestamp": float(event.get("timestamp", 0.0)),
                        "duration": float(event.get("duration", 0.0)),
                        "process_id": process_id,
                        "process_name": process_name,
                        "file_location": str(event.get("file_location", "")),
                        "stack_trace": str(event.get("stack_trace", "")),
                    }
                )
                self._total_lock_events += 1
                stats = self._process_stats_for(process_name, process_id)
                stats["lock_events_count"] = int(stats.get("lock_events_count", 0)) + 1

        if drained_any:
            self._invalidate_cache()

    def get_report(self, force_refresh: bool = False) -> dict:
        now = time.time()
        if (
            not force_refresh
            and self._last_report_cache is not None
            and now - self._last_report_cache_at < REPORT_CACHE_TTL_S
        ):
            return self._last_report_cache

        if not self._local_lock.acquire(blocking=False):
            return self._last_report_cache or {
                "metadata": {
                    "collection_duration": now - self.start_time,
                    "start_time": self.start_time,
                    "end_time": now,
                    "total_function_events": self._total_function_events,
                    "total_lock_events": self._total_lock_events,
                    "total_processes": len(self._process_stats),
                    "is_collecting": self.is_collecting,
                    "stale": True,
                },
                "processes": {},
                "functions": {"by_name": {}, "sorted_by_total_time": []},
                "locks": {"by_name": {}, "sorted_by_contention": []},
                "timeline": {"processes": [], "functions": [], "locks": []},
            }

        try:
            self._drain_events()
            function_events = list(self._function_events)
            lock_events = list(self._lock_events)
            process_events = list(self._process_events)
            process_stats = {name: dict(stats) for name, stats in self._process_stats.items()}
            total_function_events = self._total_function_events
            total_lock_events = self._total_lock_events
            dropped_events = int(self._dropped_events.value)
        finally:
            self._local_lock.release()

        function_stats = self._calculate_function_stats(function_events)
        lock_stats = self._calculate_lock_stats(lock_events)

        report = {
            "metadata": {
                "collection_duration": now - self.start_time,
                "start_time": self.start_time,
                "end_time": now,
                "total_function_events": total_function_events,
                "total_lock_events": total_lock_events,
                "total_processes": len(process_stats),
                "dropped_events": dropped_events,
                "is_collecting": self.is_collecting,
            },
            "processes": process_stats,
            "functions": function_stats,
            "locks": lock_stats,
            "timeline": {
                "processes": process_events,
                "functions": function_events,
                "locks": lock_events,
            },
        }
        self._last_report_cache = report
        self._last_report_cache_at = now
        return report

    def start_collection(self):
        if not self._local_lock.acquire(blocking=False):
            return
        try:
            self._drain_events()
            self.is_collecting = True
            self.start_time = time.time()
            self._function_events.clear()
            self._lock_events.clear()
            self._process_events.clear()
            self._total_function_events = 0
            self._total_lock_events = 0
            with self._dropped_events.get_lock():
                self._dropped_events.value = 0
            for stats in self._process_stats.values():
                stats["function_count"] = 0
                stats["lock_events_count"] = 0
                stats["start_time"] = time.time()
                stats["stop_time"] = 0.0
            self._invalidate_cache()
        finally:
            self._local_lock.release()

    def stop_collection(self):
        if not self._local_lock.acquire(blocking=False):
            return
        try:
            self._drain_events()
            self.is_collecting = False
            stop_time = time.time()
            for stats in self._process_stats.values():
                if float(stats.get("stop_time", 0.0)) == 0.0:
                    stats["stop_time"] = stop_time
            self._invalidate_cache()
        finally:
            self._local_lock.release()

    def clear_collection(self):
        if not self._local_lock.acquire(blocking=False):
            return
        try:
            self._drain_events()
            self._function_events.clear()
            self._lock_events.clear()
            self._process_events.clear()
            self._total_function_events = 0
            self._total_lock_events = 0
            with self._dropped_events.get_lock():
                self._dropped_events.value = 0
            for stats in self._process_stats.values():
                stats["function_count"] = 0
                stats["lock_events_count"] = 0
                stats["start_time"] = time.time()
                stats["stop_time"] = 0.0
            self.start_time = time.time()
            self._invalidate_cache()
        finally:
            self._local_lock.release()

    def get_status(self) -> dict:
        if self._local_lock.acquire(blocking=False):
            try:
                self._drain_events()
                total_processes = len(self._process_stats)
                total_function_events = self._total_function_events
                total_lock_events = self._total_lock_events
            finally:
                self._local_lock.release()
        else:
            total_processes = len(self._process_stats)
            total_function_events = self._total_function_events
            total_lock_events = self._total_lock_events

        return {
            "is_collecting": self.is_collecting,
            "total_function_events": total_function_events,
            "total_lock_events": total_lock_events,
            "total_processes": total_processes,
            "collection_duration": time.time() - self.start_time,
            "dropped_events": int(self._dropped_events.value),
        }

    def _calculate_function_stats(self, function_events: list[dict]) -> dict:
        stats_by_name = defaultdict(
            lambda: {
                "count": 0,
                "total_time": 0.0,
                "min_time": float("inf"),
                "max_time": 0.0,
            }
        )

        for event in function_events:
            key = f"{event['module']}.{event['name']}"
            stats = stats_by_name[key]
            stats["count"] += 1
            stats["total_time"] += event["duration"]
            stats["min_time"] = min(stats["min_time"], event["duration"])
            stats["max_time"] = max(stats["max_time"], event["duration"])

        for key, stats in stats_by_name.items():
            stats["avg_time"] = stats["total_time"] / stats["count"]
            stats["name"] = key

        sorted_stats = sorted(stats_by_name.values(), key=lambda x: x["total_time"], reverse=True)
        return {"by_name": dict(stats_by_name), "sorted_by_total_time": sorted_stats[:50]}

    def _calculate_lock_stats(self, lock_events: list[dict]) -> dict:
        stats_by_name = defaultdict(
            lambda: {
                "acquire_count": 0,
                "total_wait_time": 0.0,
                "max_wait_time": 0.0,
                "contentions": 0,
            }
        )

        for event in lock_events:
            stats = stats_by_name[event["lock_name"]]
            if event["event_type"] == "acquire":
                stats["acquire_count"] += 1
            elif event["event_type"] == "blocked":
                stats["contentions"] += 1
                stats["total_wait_time"] += event["duration"]
                stats["max_wait_time"] = max(stats["max_wait_time"], event["duration"])

        sorted_stats = sorted(
            [{"name": k, **v} for k, v in stats_by_name.items()],
            key=lambda x: x["contentions"],
            reverse=True,
        )
        return {"by_name": dict(stats_by_name), "sorted_by_contention": sorted_stats}

    def dump_to_file(self, path: str = "profiling_report.json"):
        report = self.get_report(force_refresh=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, default=str)
        print(f"Profiling report saved to {path}")


_collector: Collector | None = None


def get_collector() -> Collector | None:
    global _collector
    if _collector is None:
        _collector = Collector()
    return _collector


def dump_report(path: str = "profiling_report.json"):
    collector = get_collector()
    if collector:
        collector.dump_to_file(path)
