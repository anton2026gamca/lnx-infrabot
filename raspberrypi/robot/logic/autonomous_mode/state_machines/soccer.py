import time
import math
from dataclasses import dataclass, field

from robot import utils, vision
from robot.hardware import line_sensors
from robot.logic.autonomous_mode.state_machine import State, StateMachine, CrossStateData
from robot.multiprocessing import shared_data

from robot.vision import GoalDetectionResult, ball
from robot.hardware.motors import SmartMotorsController
from robot.config import *


# ===================== AUTONOMOUS SOCCER SETTINGS =====================

# --- General ---
# The field dimensions in millimeters
FIELD_WIDTH_MM = 1580.0
FIELD_LENGTH_MM = 2190.0

# --- Approach ---
# Forward speed component while approaching the ball
AUTO_APPROACH_SPEED = 1.0
# The ratio between the ball angle & distance and the angle that the robot should move in
# while approaching the ball.
AUTO_IR_BALL_APPROACH_ANGLE_RATIO = 0.001
# Similar ratio for camera-based ball tracking, using the camera ball angle instead of IR.
AUTO_CAM_BALL_APPROACH_ANGLE_RATIO = 1.3
# The threshold distance to consider the ball "close enough" to initiate pushing (3000 nearest, 0 farthest)
AUTO_BALL_CLOSE_THRESHOLD = 2500
# Angular window around 0° where ball is considered "in front" of the robot
AUTO_BALL_FRONT_THRESHOLD_DEG = 15.0

# --- Pushing ---
# Speed when pushing the ball toward the goal
AUTO_PUSH_SPEED = 1.0
# Maximum angle to apply for steering while pushing (deg, when ball is at edge of possession area)
AUTO_PUSH_STEERING_MAX_ANGLE_DEG = 45.0
# Distance at which we consider the goal "scored" (stop pushing)
AUTO_GOAL_SCORED_DISTANCE_MM = 600.0

# --- Ball possession camera check ---
AUTO_BALL_POSSESSION_AREA_WIDTH_DEG = CAMERA_FOV_DEG * AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT / 100.0
# IR angle range considered "inside" the robot if the ball was possessed by camera in previous frame (deg)
AUTO_BALL_INSIDE_ROBOT_IR_ANGLE_RANGE_DEG = 40.0

# --- Camera-based ball tracking (for approach state) ---
# Use camera detection as primary up to this distance (mm), then fall back to IR
AUTO_CAMERA_BALL_TRACKING_MAX_DISTANCE_MM = 1000.0
# Max allowed angle difference between camera and IR ball angles to use camera tracking
AUTO_CAMERA_BALL_TRACKING_MAX_IR_DIFF_DEG = 45.0

# --- Goal-tracking rotation ---
# Gain applied to goal alignment error to produce rotation command while approaching/pushing
AUTO_GOAL_TRACK_ROTATE_GAIN = 0.7
# Rotation speed used while searching for the goal (no goal visible)
AUTO_GOAL_SEARCH_ROTATE_SPEED = 1.0


# Toggle between standard attacker behaviour and goalkeeper behaviour.
# When enabled, the robot will prioritize defending our goal and only
# switch to attacking states when a nearby ball can be grabbed.
GOALKEEPER_MODE_ENABLED = True

# Goalkeeper positioning/behaviour tuning
GOALKEEPER_TARGET_Y_FROM_OUR_GOAL_MM = 320.0
GOALKEEPER_POSITION_TOLERANCE_X_MM = 130.0
GOALKEEPER_POSITION_TOLERANCE_Y_MM = 120.0
GOALKEEPER_APPROACH_MAX_SPEED = 1.0
GOALKEEPER_DEFEND_MAX_SPEED = 0.65
GOALKEEPER_DEFEND_STOP_RADIUS_MM = 80.0
GOALKEEPER_NEAR_BALL_CAMERA_DISTANCE_MM = 300.0
GOALKEEPER_NEAR_BALL_IR_THRESHOLD = AUTO_BALL_CLOSE_THRESHOLD - 150
GOALKEEPER_NEAR_BALL_ANGLE_THRESHOLD_DEG = 35.0
GOALKEEPER_BALL_TRACK_X_MM_PER_DEG = 10.0
GOALKEEPER_BALL_TRACK_MAX_X_MM = 550.0
GOALKEEPER_NO_POSITION_RETREAT_SPEED = 0.35



def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def _neutral_state() -> type[State]:
    return GoalkeeperApproachState if GOALKEEPER_MODE_ENABLED else IdleState


def _goalkeeper_should_grab_ball(data: "SoccerStateMachineData") -> bool:
    if data.sensors.ball_possession or data.sensors.ball_likely_inside_robot:
        return True

    if (
        data.sensors.ir_ball_detected
        and abs(data.sensors.ir_ball_angle) <= GOALKEEPER_NEAR_BALL_ANGLE_THRESHOLD_DEG
        and data.sensors.ir_ball_distance >= GOALKEEPER_NEAR_BALL_IR_THRESHOLD
    ):
        return True

    if (
        data.sensors.cam_ball_detected
        and data.sensors.cam_ball_distance <= GOALKEEPER_NEAR_BALL_CAMERA_DISTANCE_MM
        and abs(data.sensors.cam_ball_angle) <= GOALKEEPER_NEAR_BALL_ANGLE_THRESHOLD_DEG
    ):
        return True

    return False


def _set_goal_tracking_rotation(state_machine: StateMachine, data: "SoccerStateMachineData") -> float:
    if not data.sensors.goal.detected or not shared_data.get_always_facing_goal_enabled():
        state_machine.motors.set_functions_enabled(rotation_correction_enabled=True)
        state_machine.motors.target_heading = 0
        return 0.0

    state_machine.motors.set_functions_enabled(rotation_correction_enabled=False)
    rotate = data.sensors.goal.alignment * AUTO_GOAL_TRACK_ROTATE_GAIN * AUTO_SPEED_MULTIPLIER
    return _clamp(rotate, -1.0, 1.0)


def _field_delta_to_global_angle_deg(delta_x_mm: float, delta_y_mm: float) -> float:
    return (math.degrees(math.atan2(delta_x_mm, -delta_y_mm)) + 360.0) % 360.0


def _global_to_local_angle_deg(global_angle_deg: float, heading_deg: float) -> float:
    return utils.normalize_angle_deg(global_angle_deg - heading_deg)


@dataclass
class SensorsData:
    heading: float
    goal: GoalDetectionResult
    
    cam_ball_angle: float
    cam_ball_distance: float
    cam_ball_detected: bool
    cam_ball_valid: bool
    use_cam_ball: bool
    cam_possession: bool
    _last_cam_possession_elapsed_ticks: int
    
    ir_ball_angle: float
    ir_ball_distance: float
    ir_ball_detected: bool
    ir_possession: bool

    ball_possession: bool
    _ball_possession_ticks: int
    ball_likely_inside_robot: bool

@dataclass
class LinesData:
    detected: list[bool]
    enter_avoiding_state: bool
    detection_history: list[tuple[list[bool], float]] = field(default_factory=list)  # History of recent detections with timestamps


@dataclass
class SoccerStateMachineData(CrossStateData):
    sensors: SensorsData
    lines: LinesData


def _update_sensors_data(state_machine: StateMachine) -> SensorsData:
    prev_data        = state_machine.cross_state_data.sensors if isinstance(state_machine.cross_state_data, SoccerStateMachineData) else None

    goal             = shared_data.get_goal_detection_result() or GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    hardware         = shared_data.get_hardware_data()
    heading          = utils.normalize_angle_deg(hardware.compass.heading) if hardware else 0

    ir_ball_angle    = utils.normalize_angle_deg(hardware.ir.angle) if hardware else 999
    ir_ball_distance = hardware.ir.distance if hardware else 0
    ir_ball_detected = ir_ball_angle != 999 and ir_ball_distance != 0
    ir_possession    = (
        ir_ball_detected
        and abs(ir_ball_angle) < AUTO_BALL_FRONT_THRESHOLD_DEG
        and ir_ball_distance > AUTO_BALL_CLOSE_THRESHOLD # The IR distance value is flipped (higher means closer)
    )

    cam_ball         = shared_data.get_camera_ball_data()
    cam_ball.angle   = utils.normalize_angle_deg(cam_ball.angle) if cam_ball and cam_ball.detected else 999
    cam_ball_valid   = (
        cam_ball.detected and cam_ball.distance <= AUTO_CAMERA_BALL_TRACKING_MAX_DISTANCE_MM
        and (not ir_ball_detected or abs(ir_ball_angle - cam_ball.angle) < AUTO_CAMERA_BALL_TRACKING_MAX_IR_DIFF_DEG)
    )
    use_cam_ball     = shared_data.get_camera_ball_usage_enabled() and cam_ball_valid
    cam_possession   = shared_data.get_camera_ball_possession()
    _last_cam_possession_elapsed_ticks = prev_data._last_cam_possession_elapsed_ticks + 1 if prev_data and not cam_possession else 0

    _ball_possession_ticks = prev_data._ball_possession_ticks if prev_data else 0
    _ball_possession_raw = cam_possession if use_cam_ball else ir_possession
    if _ball_possession_raw:
        _ball_possession_ticks = min(_ball_possession_ticks + 1, 3)
    else:
        _ball_possession_ticks = max(_ball_possession_ticks - 1, 0)
    ball_possession  = _ball_possession_ticks >= 3

    ball_likely_inside_robot = (
        _last_cam_possession_elapsed_ticks <= 3 and not cam_ball.detected
        and (not ir_ball_detected or abs(ir_ball_angle) > AUTO_BALL_INSIDE_ROBOT_IR_ANGLE_RANGE_DEG)
    )

    return SensorsData(
        heading,
        goal,
        
        cam_ball.angle,
        cam_ball.distance,
        cam_ball.detected,
        cam_ball_valid,
        use_cam_ball,
        cam_possession,
        _last_cam_possession_elapsed_ticks,
        
        ir_ball_angle,
        ir_ball_distance,
        ir_ball_detected,
        ir_possession,
        
        ball_possession,
        _ball_possession_ticks,
        ball_likely_inside_robot
    )


def _update_lines_data(state_machine: StateMachine) -> LinesData:
    prev_data = state_machine.cross_state_data.lines if isinstance(state_machine.cross_state_data, SoccerStateMachineData) else None

    detected = line_sensors.get_line_detected()
    enter_avoiding_state = any(detected) and shared_data.get_line_avoiding_enabled()
    
    current_time = time.time()
    history = prev_data.detection_history if prev_data and prev_data.detection_history else []
    history.append((detected.copy(), current_time))
    
    history_max_age = 0.5
    history = [(d, t) for d, t in history if current_time - t < history_max_age]
    
    return LinesData(
        detected=detected,
        enter_avoiding_state=enter_avoiding_state,
        detection_history=history
    )


def _update_cross_state_data(state_machine: StateMachine) -> SoccerStateMachineData:
    data = SoccerStateMachineData(
        sensors=_update_sensors_data(state_machine),
        lines=_update_lines_data(state_machine)
    )
    state_machine.cross_state_data = data
    return data


# -------------------------------------------------------------------
# State: LINE AVOIDING
# -------------------------------------------------------------------
class LineAvoidingState(State):
    def on_enter(self, state_machine: StateMachine) -> None:
        self.min_clear_time = 0.5
        self.clear_time_start = None
        self.line_exit_time = None
        self.slow_duration = 1.0
        self.absolute_avoid_direction = None
        
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)
        position = vision.get_position_estimate()
        current_time = time.time()

        rotate = _set_goal_tracking_rotation(state_machine, data)
        
        currently_detected = any(data.lines.detected)
        
        if currently_detected:
            self.absolute_avoid_direction = self._calculate_absolute_avoid_direction(data, position)
            relative_avoid_direction = (self.absolute_avoid_direction - data.sensors.heading + 360) % 360
            self.clear_time_start = None
            self.line_exit_time = None
            state_machine.motors.set_motors(angle=relative_avoid_direction, speed=1.0, rotate=rotate)
        else:
            if self.clear_time_start is None:
                self.clear_time_start = current_time
                self.line_exit_time = current_time
            
            time_without_detection = current_time - self.clear_time_start
            
            recent_detections_exist = any(
                any(detections) for detections, timestamp in data.lines.detection_history
                if current_time - timestamp < 0.2
            ) if data.lines.detection_history else False
            
            if time_without_detection > self.min_clear_time and not recent_detections_exist:
                state_machine.transition(_neutral_state())
                return
            
            time_since_exit = current_time - self.line_exit_time if self.line_exit_time else 0
            if time_since_exit < self.slow_duration and self.absolute_avoid_direction is not None:
                relative_avoid_direction = (self.absolute_avoid_direction - data.sensors.heading + 360) % 360
                state_machine.motors.set_motors(angle=relative_avoid_direction, speed=0.7, rotate=rotate)
            else:
                state_machine.motors.set_motors(angle=0.0, speed=0.0, rotate=0.0)

    def _calculate_absolute_avoid_direction(self, data: SoccerStateMachineData, position) -> float:
        detected_angles = []

        hardware_data = shared_data.get_hardware_data()
        robot_heading = hardware_data.compass.heading if hardware_data else 0

        for i, detected in enumerate(data.lines.detected):
            if detected and i < len(LINE_SENSOR_LOCATIONS):
                detected_angles.append((LINE_SENSOR_LOCATIONS[i] + robot_heading) % 360)

        current_time = time.time()
        for detections, timestamp in data.lines.detection_history:
            if current_time - timestamp < 0.3:
                for i, detected in enumerate(detections):
                    if detected and i < len(LINE_SENSOR_LOCATIONS):
                        detected_angles.append((LINE_SENSOR_LOCATIONS[i] + robot_heading) % 360)

        detected_angles = list(set(detected_angles))

        if not detected_angles:
            return 0.0

        if position:
            too_close_to_enemy_goal_line = position.y_mm < 400
            too_close_to_our_goal_line = (FIELD_LENGTH_MM - position.y_mm) < 400

            too_far_left  = position.x_mm < 400 - FIELD_WIDTH_MM / 2
            too_far_right = position.x_mm > FIELD_WIDTH_MM / 2 - 400

            left_detected = any(
                (angle >= 45 and angle <= 135)
                for angle in detected_angles
            )
            right_detected = any(
                (angle >= 225 and angle <= 315)
                for angle in detected_angles
            )

            if too_close_to_enemy_goal_line:
                if too_far_left and left_detected:
                    return 135
                if too_far_right and right_detected:
                    return 225
                return 180

            elif too_close_to_our_goal_line:
                if too_far_left:
                    return 45 if left_detected else 0
                if too_far_right:
                    return -45 if right_detected else 0
                ball_angle = data.sensors.cam_ball_angle if data.sensors.use_cam_ball else data.sensors.ir_ball_angle
                ball_angle_global = utils.normalize_angle_deg(ball_angle + data.sensors.heading)
                if ball_angle_global >= 30 and ball_angle_global <= 100:
                    return 60
                if ball_angle_global <= -30 and ball_angle_global >= -100:
                    return -60
                return 0

            elif too_far_left:
                return 90

            elif too_far_right:
                return -90

        angles_rad = [math.radians(a) for a in detected_angles]
        x = sum(math.cos(a) for a in angles_rad)
        y = sum(math.sin(a) for a in angles_rad)
        avg_angle = (math.degrees(math.atan2(y, x)) + 360) % 360
        
        avoid_angle = (avg_angle + 180) % 360
        return avoid_angle


# ------------------------------------------------------------------
# State: IDLE
# No ball detected - hold still.
# ------------------------------------------------------------------
class IdleState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)
        if data.lines.enter_avoiding_state:
            state_machine.transition(LineAvoidingState)
            return

        if GOALKEEPER_MODE_ENABLED:
            state_machine.transition(GoalkeeperApproachState)
            return

        if data.sensors.ir_ball_detected or data.sensors.cam_ball_detected or data.sensors.ball_likely_inside_robot:
            state_machine.transition(AttackerApproachState)
            return
        state_machine.motors.set_motors(angle=0.0, speed=0.0, rotate=0.0)


# ------------------------------------------------------------------
# State: APPROACH
# Drive toward the ball while rotating to keep the goal centred.
#
# Movement: Prefer camera detection if available (more reliable).
#           Fall back to IR if camera ball is outside tracking range.
#           If ball was recently visible by camera but lost, assume
#           it's inside robot and continue forward slowly to secure
#           possession.
# Rotation: use goal camera alignment to rotate the whole robot so
#           the goal stays centred.
# ------------------------------------------------------------------
class AttackerApproachState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)

        if data.lines.enter_avoiding_state:
            state_machine.transition(LineAvoidingState)
            return

        if data.sensors.ball_possession or data.sensors.ball_likely_inside_robot:
            state_machine.transition(AttackerPushState)
            return

        if not data.sensors.ir_ball_detected and (not data.sensors.cam_ball_detected or not data.sensors.use_cam_ball):
            state_machine.transition(_neutral_state())
            return

        move_angle = 0.0
        move_speed = AUTO_APPROACH_SPEED
        rotate = _set_goal_tracking_rotation(state_machine, data)

        if data.sensors.use_cam_ball:
            if abs(data.sensors.cam_ball_angle) > AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0:
                move_angle = data.sensors.cam_ball_angle * AUTO_CAM_BALL_APPROACH_ANGLE_RATIO
                move_angle = max(-180, min(180, move_angle))
        else:
            move_angle = utils.normalize_angle_deg(data.sensors.ir_ball_angle + data.sensors.heading) * data.sensors.ir_ball_distance * AUTO_IR_BALL_APPROACH_ANGLE_RATIO
            move_angle = max(-180, min(180, move_angle))

        position = vision.get_position_estimate()
        if position:
            y_distance = 400 - position.y_mm
            if y_distance > 0:
                left_x_boundry = -FIELD_WIDTH_MM / 2 + y_distance / 2
                right_x_boundry = FIELD_WIDTH_MM / 2 - y_distance / 2
                
                in_left_corner = position.x_mm < left_x_boundry
                in_right_corner = position.x_mm > right_x_boundry

                absolute_move_angle = (move_angle + data.sensors.heading) % 360
                
                if in_left_corner and (absolute_move_angle > 225 or absolute_move_angle < 45):
                    move_speed = 0
                if in_right_corner and (absolute_move_angle > 315 or absolute_move_angle < 135):
                    move_speed = 0

        move_speed *= AUTO_SPEED_MULTIPLIER
        state_machine.motors.set_motors(angle=move_angle, speed=move_speed, rotate=rotate)


# ------------------------------------------------------------------
# State: PUSH
# Ball is in the front pocket. Drive straight forward while keeping
# the goal centred via rotation. If the ball is lost, re-approach.
# ------------------------------------------------------------------
class AttackerPushState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)

        if data.lines.enter_avoiding_state:
            state_machine.transition(LineAvoidingState)
            return

        if not data.sensors.ball_possession and not data.sensors.ball_likely_inside_robot:
            state_machine.transition(GoalkeeperApproachState if GOALKEEPER_MODE_ENABLED else AttackerApproachState)
            return

        move_speed = AUTO_PUSH_SPEED
        move_angle = 0.0

        if data.sensors.goal.distance_mm is not None and data.sensors.goal.distance_mm < AUTO_GOAL_SCORED_DISTANCE_MM:
            state_machine.transition(_neutral_state())
            return

        rotate = _set_goal_tracking_rotation(state_machine, data)

        if data.sensors.use_cam_ball:
            move_angle = -data.sensors.cam_ball_angle / (AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0) * AUTO_PUSH_STEERING_MAX_ANGLE_DEG
            move_angle = max(-90, min(90, move_angle))

        move_speed *= AUTO_SPEED_MULTIPLIER
        state_machine.motors.set_motors(angle=move_angle, speed=move_speed, rotate=rotate)

# ------------------------------------------------------------------
# State: GOALKEEPER APPROACH
# Drive toward our goal to prepare for a defensive play.
# ------------------------------------------------------------------
class GoalkeeperApproachState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)

        if data.lines.enter_avoiding_state:
            state_machine.transition(LineAvoidingState)
            return

        if _goalkeeper_should_grab_ball(data):
            state_machine.transition(AttackerApproachState)
            return

        rotate = _set_goal_tracking_rotation(state_machine, data)
        position = vision.get_position_estimate()

        if position is None:
            move_speed = GOALKEEPER_NO_POSITION_RETREAT_SPEED * AUTO_SPEED_MULTIPLIER
            state_machine.motors.set_motors(angle=180.0, speed=move_speed, rotate=rotate)
            return

        target_x_mm = 0.0
        target_y_mm = FIELD_LENGTH_MM - GOALKEEPER_TARGET_Y_FROM_OUR_GOAL_MM

        delta_x = target_x_mm - position.x_mm
        delta_y = target_y_mm - position.y_mm

        if (
            abs(delta_x) <= GOALKEEPER_POSITION_TOLERANCE_X_MM
            and abs(delta_y) <= GOALKEEPER_POSITION_TOLERANCE_Y_MM
        ):
            state_machine.transition(GoalkeeperDefendState)
            return

        global_angle = _field_delta_to_global_angle_deg(delta_x, delta_y)
        local_move_angle = _global_to_local_angle_deg(global_angle, data.sensors.heading)

        distance_mm = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        move_speed = _clamp(distance_mm / 700.0, 0.28, GOALKEEPER_APPROACH_MAX_SPEED)
        move_speed *= AUTO_SPEED_MULTIPLIER

        state_machine.motors.set_motors(angle=local_move_angle, speed=move_speed, rotate=rotate)


# ------------------------------------------------------------------
# State: GOALKEEPER DEFEND
# Stay in front of the goal and react to the ball's position to block
# shots. If we get the ball, switch to AttackerPushState and signal
# to teammate that we're taking possession so they can adjust their
# behaviour accordingly.
# ------------------------------------------------------------------
class GoalkeeperDefendState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)

        if data.lines.enter_avoiding_state:
            state_machine.transition(LineAvoidingState)
            return

        if _goalkeeper_should_grab_ball(data):
            state_machine.transition(AttackerApproachState)
            return

        rotate = _set_goal_tracking_rotation(state_machine, data)
        position = vision.get_position_estimate()

        if position is None:
            move_speed = GOALKEEPER_NO_POSITION_RETREAT_SPEED * AUTO_SPEED_MULTIPLIER
            state_machine.motors.set_motors(angle=180.0, speed=move_speed, rotate=rotate)
            return

        target_y_mm = FIELD_LENGTH_MM - GOALKEEPER_TARGET_Y_FROM_OUR_GOAL_MM

        ball_angle = None
        if data.sensors.use_cam_ball:
            ball_angle = data.sensors.cam_ball_angle
        elif data.sensors.ir_ball_detected:
            ball_angle = data.sensors.ir_ball_angle

        target_x_mm = 0.0
        if ball_angle is not None:
            target_x_mm = position.x_mm + ball_angle * GOALKEEPER_BALL_TRACK_X_MM_PER_DEG
            target_x_mm = _clamp(target_x_mm, -GOALKEEPER_BALL_TRACK_MAX_X_MM, GOALKEEPER_BALL_TRACK_MAX_X_MM)

        delta_x = target_x_mm - position.x_mm
        delta_y = target_y_mm - position.y_mm

        if (
            abs(delta_x) > GOALKEEPER_POSITION_TOLERANCE_X_MM * 2.0
            or abs(delta_y) > GOALKEEPER_POSITION_TOLERANCE_Y_MM * 2.0
        ):
            state_machine.transition(GoalkeeperApproachState)
            return

        distance_mm = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        if distance_mm < GOALKEEPER_DEFEND_STOP_RADIUS_MM:
            state_machine.motors.set_motors(angle=0.0, speed=0.0, rotate=rotate)
            return

        if ball_angle is not None and abs(ball_angle) > 90:
            pass

        global_angle = _field_delta_to_global_angle_deg(delta_x, delta_y)
        local_move_angle = _global_to_local_angle_deg(global_angle, data.sensors.heading)

        move_speed = _clamp(distance_mm / 500.0, 0.18, GOALKEEPER_DEFEND_MAX_SPEED)
        move_speed *= AUTO_SPEED_MULTIPLIER

        state_machine.motors.set_motors(angle=local_move_angle, speed=move_speed, rotate=rotate)


_motors = SmartMotorsController()
_motors.set_functions_enabled(line_avoiding_enabled=False)
_state_machine = StateMachine(
    name="Soccer State Machine",
    initial_state=GoalkeeperApproachState if GOALKEEPER_MODE_ENABLED else IdleState,
    motors=_motors
)

def get_state_machine() -> StateMachine:
    return _state_machine

