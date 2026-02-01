import logging
import math
import sys



def calculate_motors_speeds(move_angle: float, move_speed: float, rotate: float) -> list[int]:
    MOTOR_LOCATIONS = [45, 135, 225, 315]
    rad = math.radians(move_angle)
    speeds = []
    for motor_angle in MOTOR_LOCATIONS:
        motor_rad = math.radians(motor_angle)
        motor_speed = move_speed * math.cos(rad - motor_rad) + rotate
        speeds.append(int(motor_speed))
    return speeds


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


class RobotMode:
    IDLE = "idle"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"


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
        self.oldest_log_id = 0
        self.max_entries = max_entries
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

                try:
                    if self.max_entries is not None:
                        while len(self.logs_dict) > self.max_entries:
                            del self.logs_dict[self.oldest_log_id]
                            self.oldest_log_id += 1
                except Exception:
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
        buffer_handler = BufferedLogHandler(logs_dict, logs_lock, next_log_id)
        buffer_handler.setLevel(level)
        logger.addHandler(buffer_handler)
    
    return logger


class RobotManualControl:
    def __init__(self, move_angle: float = 0.0, move_speed: float = 0.0, rotate: float = 0.0):
        self.move_angle = move_angle
        self.move_speed = move_speed
        self.rotate = rotate