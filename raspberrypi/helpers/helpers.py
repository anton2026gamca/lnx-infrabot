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
