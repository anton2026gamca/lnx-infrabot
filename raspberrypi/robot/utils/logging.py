import logging
import sys

import robot.multiprocessing.shared_data as shared_data
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
    def __init__(self, logs_dict, logs_lock, next_log_id, formatter=None, max_entries: int = 100):
        super().__init__()
        self.logs_dict = logs_dict
        self.logs_lock = logs_lock
        self.next_log_id = next_log_id
        self.max_entries = max_entries
        self._cleanup_threshold = int(max_entries * 1.2)
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
            with self.logs_lock:
                lid = self.next_log_id.value
                self.logs_dict[lid] = entry
                self.next_log_id.value += 1

                if len(self.logs_dict) > self._cleanup_threshold:
                    sorted_keys = sorted(self.logs_dict.keys())
                    to_remove = len(self.logs_dict) - self.max_entries
                    for key in sorted_keys[:to_remove]:
                        try:
                            del self.logs_dict[key]
                        except KeyError:
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

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    formatter = LogFormatter(
        f"[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s",
        datefmt="%H:%M:%S"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    buffer_handler = BufferedLogHandler(shared_data.logs_buffer, shared_data.logs_buffer_lock, shared_data.next_log_id, max_entries=LOG_BUFFER_MAX_ENTRIES)
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

def get_logs(since_id: int = 0) -> tuple[list[dict], int]:
    with shared_data.logs_buffer_lock:
        items = [
            {"id": lid, **entry}
            for lid, entry in sorted(shared_data.logs_buffer.items())
            if lid > since_id
        ]
        last_id = max(shared_data.logs_buffer.keys()) if shared_data.logs_buffer else 0
    return items, last_id

