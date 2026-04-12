import math
import time

from robot import utils, vision
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
        - Position-based speed: when enabled, uses the robot's position estimate to reduce speed as it approaches lines, to help prevent crossing them.
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

        self.rotation_correction_enabled = True
        self.line_avoiding_enabled = True
        self.position_based_speed_enabled = True

    def set_motors(self, angle: float, speed: float, rotate: float):
        hardware_data = shared_data.get_hardware_data()
        current_heading = hardware_data.compass.heading if hardware_data else None

        rotation_correction_enabled = shared_data.get_rotation_correction_enabled() and self.rotation_correction_enabled

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

        if shared_data.get_line_avoiding_enabled() and self.line_avoiding_enabled:
            current_time = time.time()
            
            self.recently_crossed_angles = [
                (angle, t) for angle, t in self.recently_crossed_angles 
                if current_time - t < self.line_avoidance_cooldown_duration
            ]
            
            lines_detected = line_sensors.get_line_detected()
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
        
        pos_speed_factor = 1.0
        if shared_data.get_position_based_speed_enabled() and self.position_based_speed_enabled:
            pos = vision.get_position_estimate()
            if pos is not None:
                distances = [
                    pos.x_mm - AUTO_POSITION_SLOW_START_DISTANCE_X_MM,
                    AUTO_POSITION_SLOW_START_DISTANCE_Y_MIN_MM - pos.y_mm,
                    pos.y_mm - AUTO_POSITION_SLOW_START_DISTANCE_Y_MAX_MM
                ]
                highest = max(*distances)
                highest = max(0.0, min(AUTO_POSITION_SLOW_END_DISTANCE_MM, highest))
                pos_speed_factor = 1.0 - (highest / AUTO_POSITION_SLOW_END_DISTANCE_MM) * AUTO_POSITION_SLOW_MIN_SPEED

        move_angle = math.atan2(self.move_y, self.move_x)
        move_speed = math.sqrt(self.move_x**2 + self.move_y**2) * pos_speed_factor
        
        motors = calculate_speeds(move_angle, move_speed * 255, total_rotation * 255)
        shared_data.set_motor_speeds(motors)

    def set_functions_enabled(self, rotation_correction_enabled: bool | None = None, line_avoiding_enabled: bool | None = None, position_based_speed_enabled: bool | None = None):
        """
        Enable or disable smart functions. If a parameter is None, it will not change the current state of that function.
        NOTE: Enabling any of the functions may not actually enable them, if they are disabled globally (via the api)
        """
        if rotation_correction_enabled is not None:
            self.rotation_correction_enabled = rotation_correction_enabled
        if line_avoiding_enabled is not None:
            self.line_avoiding_enabled = line_avoiding_enabled
        if position_based_speed_enabled is not None:
            self.position_based_speed_enabled = position_based_speed_enabled
    
    def reset(self):
        super().reset()
        self.target_heading = None
        self.line_avoidance_active = False
        self.line_avoidance_direction = 0.0
        self.recently_crossed_angles = []

