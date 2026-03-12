import math
import time

from robot import utils
from robot.hardware import line_sensors
from robot.multiprocessing import shared_data

from robot.config import *



logger = utils.get_logger("Motors Controller")


def calculate_speeds(move_angle_rad: float, move_speed: float, rotate: float) -> list[int]:
    speeds = []
    for motor_angle in MOTOR_LOCATIONS:
        motor_rad = math.radians(motor_angle)
        motor_speed = move_speed * math.cos(move_angle_rad - motor_rad) + rotate
        speeds.append(int(motor_speed))
    return speeds


class MotorsController:
    """Basic motors controller with acceleration smoothing. Takes desired movement commands"""

    def __init__(self):
        self.move_x = 0.0
        self.move_y = 0.0
        self.rotate = 0.0
        self.acceleration = LOGIC_LOOP_FREQUENCY / MOTOR_ACCELERATION_LOGIC_LOOPS

    def set_motors(self, angle: float, speed: float, rotate: float):
        self.move_x += (speed * math.cos(math.radians(angle)) - self.move_x) * self.acceleration * LOGIC_LOOP_PERIOD
        self.move_y += (speed * math.sin(math.radians(angle)) - self.move_y) * self.acceleration * LOGIC_LOOP_PERIOD
        self.rotate += (rotate - self.rotate) * self.acceleration * LOGIC_LOOP_PERIOD

        new_angle = math.atan2(self.move_y, self.move_x)
        speed = math.sqrt(self.move_x**2 + self.move_y**2)

        motors = calculate_speeds(new_angle, speed * 255, self.rotate * 255)
        shared_data.set_motor_speeds(motors)
    
    def reset(self):
        self.move_x = 0.0
        self.move_y = 0.0
        self.rotate = 0.0
        shared_data.set_motor_speeds([0, 0, 0, 0])


class SmartMotorsController(MotorsController):
    """
    Enhanced motors controller with:
        - Rotation correction: when enabled, automatically applies a rotation correction to maintain the robot's heading when no manual rotation input is given.
        - Line avoidance: when enabled, detects line sensor triggers and automatically steers the robot away from the line for a short duration, with cooldown to prevent oscillation.
    """

    def __init__(self):
        super().__init__()
        self.target_heading = None
        self.rotation_deadzone = 15.0
        self.rotation_correction_gain = 1.0
        
        self.line_avoidance_active = False
        self.line_avoidance_direction = 0.0
        self.line_avoidance_start_time = 0.0
        self.line_avoidance_min_duration = 0.5
        self.line_avoidance_cooldown_duration = 0.5
        self.last_line_avoid_end_time = 0.0
        self.recently_crossed_angles = []

    def set_motors(self, angle: float, speed: float, rotate: float):
        hardware_data = shared_data.get_hardware_data()
        current_heading = hardware_data.compass.heading if hardware_data else None

        rotation_correction_enabled = shared_data.get_rotation_correction_enabled()

        absolute_angle = (angle + self.target_heading - current_heading) % 360 if rotation_correction_enabled and self.target_heading is not None and current_heading is not None else angle
        self.move_x += (speed * math.cos(math.radians(absolute_angle)) - self.move_x) * self.acceleration * LOGIC_LOOP_PERIOD
        self.move_y += (speed * math.sin(math.radians(absolute_angle)) - self.move_y) * self.acceleration * LOGIC_LOOP_PERIOD
        self.rotate += (rotate - self.rotate) * self.acceleration * LOGIC_LOOP_PERIOD
        move_angle = math.atan2(self.move_y, self.move_x)
        move_speed = math.sqrt(self.move_x**2 + self.move_y**2)

        rotation_correction = 0.0
        if rotation_correction_enabled:
            if abs(self.rotate) > 0.01:
                self.target_heading = None
            elif self.target_heading is None and current_heading is not None:
                self.target_heading = current_heading
            if current_heading is not None and self.target_heading is not None:
                heading_error = (self.target_heading - current_heading + 180) % 360 - 180
                if abs(heading_error) > self.rotation_deadzone:
                    rotation_correction = (heading_error / 180.0) * self.rotation_correction_gain

        total_rotation = self.rotate * 0.4 + rotation_correction
        total_rotation = max(-1.0, min(1.0, total_rotation))

        line_avoiding_enabled = shared_data.get_line_avoiding_enabled()
        current_time = time.time()
        
        self.recently_crossed_angles = [
            (angle, t) for angle, t in self.recently_crossed_angles 
            if current_time - t < self.line_avoidance_cooldown_duration
        ]
        
        if line_avoiding_enabled:
            lines_detected = line_sensors.get_line_detected()
            if any(lines_detected):
                logger.info(f"Line detected: {lines_detected}")
            detected_angles = []
            for i, detected in enumerate(lines_detected):
                if detected and i < len(LINE_SENSOR_LOCATIONS):
                    sensor_angle = LINE_SENSOR_LOCATIONS[i]
                    detected_angles.append(sensor_angle)
            
            in_cooldown = (current_time - self.last_line_avoid_end_time) < self.line_avoidance_cooldown_duration
            
            new_detected_angles = []
            for angle in detected_angles:
                is_recent = False
                for crossed_angle, _ in self.recently_crossed_angles:
                    angle_diff = abs(((angle - crossed_angle + 180) % 360) - 180)
                    if angle_diff < 45:
                        is_recent = True
                        break
                if not is_recent:
                    new_detected_angles.append(angle)
            
            if self.line_avoidance_active:
                if current_time - self.line_avoidance_start_time < self.line_avoidance_min_duration:
                    move_speed = 1.0
                    self.move_x = move_speed * math.cos(self.line_avoidance_direction)
                    self.move_y = move_speed * math.sin(self.line_avoidance_direction)
                elif not detected_angles:
                    self.line_avoidance_active = False
                    self.last_line_avoid_end_time = current_time
                    
                    if hasattr(self, '_avoiding_angles'):
                        for angle in self._avoiding_angles:
                            self.recently_crossed_angles.append((angle, current_time))
                else:
                    angles_rad = [math.radians(a) for a in detected_angles]
                    x = sum(math.cos(a) for a in angles_rad)
                    y = sum(math.sin(a) for a in angles_rad)
                    avg_angle = (math.degrees(math.atan2(y, x)) + 360) % 360
                    avoid_angle = math.radians(avg_angle + 180)
                    move_speed = 1.0
                    self.move_x = move_speed * math.cos(avoid_angle)
                    self.move_y = move_speed * math.sin(avoid_angle)
                    self.line_avoidance_direction = avoid_angle
            elif new_detected_angles and not in_cooldown:
                self.line_avoidance_active = True
                self.line_avoidance_start_time = current_time
                self._avoiding_angles = detected_angles.copy()
                
                angles_rad = [math.radians(a) for a in detected_angles]
                x = sum(math.cos(a) for a in angles_rad)
                y = sum(math.sin(a) for a in angles_rad)
                avg_angle = (math.degrees(math.atan2(y, x)) + 360) % 360
                avoid_angle = math.radians(avg_angle + 180)
                move_speed = 1.0
                self.move_x = move_speed * math.cos(avoid_angle)
                self.move_y = move_speed * math.sin(avoid_angle)
                self.line_avoidance_direction = avoid_angle
        
        move_angle = math.atan2(self.move_y, self.move_x)
        move_speed = math.sqrt(self.move_x**2 + self.move_y**2)

        motors = calculate_speeds(move_angle, move_speed * 255, total_rotation * 255)
        shared_data.set_motor_speeds(motors)
    
    def reset(self):
        super().reset()
        self.target_heading = None
        self.line_avoidance_active = False
        self.line_avoidance_direction = 0.0
        self.recently_crossed_angles = []

