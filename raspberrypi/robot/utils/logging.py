import logging
import sys
import queue
from multiprocessing import Queue
from multiprocessing.synchronize import Lock

from robot.multiprocessing import shared_data
from robot.config import *



_default_logger_level = logging.DEBUG
_known_loggers: dict[str | None, logging.Logger] = {}


class LogFormatter(logging.Formatter):
    RESET = "\033[0m"
    BOLD = "\033[1m"

    LEVEL_STYLES = {
        logging.DEBUG:    "\033[36m",  # Cyan
        logging.INFO:     "\033[32m",  # Green
        logging.WARNING:  "\033[33m",  # Yellow
        logging.ERROR:    "\033[31m",  # Red
        logging.CRITICAL: "\033[41m",  # Red background
    }

    def format(self, record) -> str:
        color = self.LEVEL_STYLES.get(record.levelno, "")
        levelname = record.levelname

        if color:
            record.levelname = (
                f"{self.BOLD}{color}{levelname}{self.RESET}"
            )

        msg = super().format(record)
        record.levelname = levelname
        return msg

class Suppress200Filter(logging.Filter):
    def filter(self, record):
        try:
            msg = record.getMessage()
            import re
            m = re.search(r'"\s*(\d{3})\s', msg)
            if m and m.group(1) == '200':
                return False
        except Exception:
            pass
        return True

class BufferedLogHandler(logging.Handler):
    def __init__(self, logs_queue: Queue, formatter=None):
        super().__init__()
        self.logs_queue = logs_queue
        if formatter:
            self.setFormatter(formatter)
        elif LogFormatter is not None:
            self.setFormatter(LogFormatter(f"[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s", datefmt="%H:%M:%S"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            formatted = self.format(record)
            entry = {
                "message": formatted,
                "level": record.levelname,
                "logger": record.name,
                "time": getattr(record, "created", None),
            }
            try:
                self.logs_queue.put_nowait(entry)
            except queue.Full:
                pass
        except Exception as e:
            self.handleError(record)

def set_default_logger_level(level: int) -> None:
    global _default_logger_level
    _default_logger_level = level

def get_default_logger_level() -> int:
    return _default_logger_level

def setup_logger(name: str | None = None, level=_default_logger_level) -> logging.Logger:
    if _default_logger_level is None:
        logger_level = getattr(logging, LOG_LEVEL.upper(), logging.INFO)
        set_default_logger_level(logger_level)
        level = _default_logger_level

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(level)
    formatter = LogFormatter(
        f"[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s",
        datefmt="%H:%M:%S"
    )
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)
    
    buffer_handler = BufferedLogHandler(shared_data.logs_buffer, formatter=formatter)
    buffer_handler.setLevel(level)
    logger.addHandler(buffer_handler)
    
    return logger

def get_logger(name: str | None = None, level=_default_logger_level) -> logging.Logger:
    if name is None:
        name = logging.getLogger().name
    if name in _known_loggers:
        return _known_loggers[name]
    logger = setup_logger(name, level)
    _known_loggers[logger.name] = logger
    return logger

_next_log_id = 1
_logs_buffer: list[dict] = []
def _drain_logs():
    global _next_log_id

    while True:
        try:
            entry = shared_data.logs_buffer.get_nowait()
        except Exception:
            break

        entry["id"] = _next_log_id
        _next_log_id += 1
        _logs_buffer.append(entry)

def get_logs(since_id: int = 0) -> tuple[list[dict], int]:
    _drain_logs()

    items = [log for log in _logs_buffer if log["id"] > since_id]
    last_id = _logs_buffer[-1]["id"] if _logs_buffer else 0

    return items, last_id

