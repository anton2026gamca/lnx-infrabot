import logging
import math
import sys
from dataclasses import dataclass

from config import MOTOR_LOCATIONS, LOG_BUFFER_MAX_ENTRIES



# ==================== LOGGING HELPERS ====================

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
        except Exception:
            self.handleError(record)

def setup_logger(name: str | None = None, level=logging.DEBUG, logs_dict=None, logs_lock=None, next_log_id=None) -> logging.Logger:
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
    
    if logs_dict is not None and logs_lock is not None and next_log_id is not None:
        buffer_handler = BufferedLogHandler(logs_dict, logs_lock, next_log_id, max_entries=LOG_BUFFER_MAX_ENTRIES)
        buffer_handler.setLevel(level)
        logger.addHandler(buffer_handler)
    
    return logger


# ==================== ROBOT CONTROL HELPERS ====================

class RobotMode:
    IDLE = "idle"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"

@dataclass
class RobotManualControl:
    move_angle: float = 0.0
    move_speed: float = 0.0
    rotate: float = 0.0

@dataclass
class PositionEstimate:
    x_mm: float
    y_mm: float
    confidence: float

def calculate_motors_speeds(move_angle_rad: float, move_speed: float, rotate: float) -> list[int]:
    speeds = []
    for motor_angle in MOTOR_LOCATIONS:
        motor_rad = math.radians(motor_angle)
        motor_speed = move_speed * math.cos(move_angle_rad - motor_rad) + rotate
        speeds.append(int(motor_speed))
    return speeds
    