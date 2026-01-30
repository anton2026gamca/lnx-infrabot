import logging
import math



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
