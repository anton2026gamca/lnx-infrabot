from __future__ import annotations

import time
import math
from dataclasses import dataclass, field
from typing import overload

from robot import utils, vision
from robot.bluetooth import utils as bluetooth
from robot.hardware import line_sensors
from robot.logic.autonomous_mode.state_machine import State, StateMachine, CrossStateData
from robot.multiprocessing import shared_data
from robot.multiprocessing.shared_data import AutonomousStatusText
from robot.profiling import profile_function

from robot.vision import GoalDetectionResult
from robot.hardware.motors import SmartMotorsController
from robot.config import *


logger = utils.get_logger("Soccer State Machine")

ATTACKER_STATE_MACHINE_NAME = "Attacker State Machine"
GOALKEEPER_STATE_MACHINE_NAME = "Goalkeeper State Machine"

# ===================== AUTONOMOUS SOCCER SETTINGS =====================

# --- General ---
# The field dimensions in millimeters
FIELD_WIDTH_MM = 1580.0
FIELD_LENGTH_MM = 2190.0

# --- Logging ---
LOG_ATTACKER_STATE = True
LOG_GOALKEEPER_STATE = True
LOG_ROLE_CHANGES = True

# --- Approach ---
# Forward speed component while approaching the ball
APPROACH_SPEED = 1.0
# The ratio between the ball angle & distance and the angle that the robot should move in
# while approaching the ball.
IR_BALL_APPROACH_ANGLE_RATIO = 0.001
# Similar ratio for camera-based ball tracking, using the camera ball angle instead of IR.
CAM_BALL_APPROACH_ANGLE_RATIO = 1.3
# The threshold distance to consider the ball "close enough" to initiate pushing (3000 nearest, 0 farthest)
IR_BALL_CLOSE_THRESHOLD = 2600
# Angular window around 0° where ball is considered "in front" of the robot
BALL_FRONT_THRESHOLD_DEG = 15.0

# --- Pushing ---
# Speed when pushing the ball toward the goal
PUSH_SPEED = 1.0
# Maximum angle to apply for steering while pushing (deg, when ball is at edge of possession area)
PUSH_STEERING_MAX_ANGLE_DEG = 45.0
# Distance at which we consider the goal "scored" (stop pushing)
GOAL_SCORED_DISTANCE_MM = 600.0

# --- Ball possession camera check ---
BALL_POSSESSION_AREA_WIDTH_DEG = CAMERA_FOV_DEG * AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT / 100.0
# IR angle range considered "inside" the robot if the ball was possessed by camera in previous frame (deg)
BALL_INSIDE_ROBOT_IR_ANGLE_RANGE_DEG = 40.0

# --- Camera-based ball tracking (for approach state) ---
# Use camera detection as primary up to this distance (mm), then fall back to IR
CAMERA_BALL_TRACKING_MAX_DISTANCE_MM = 2000.0
# Max allowed angle difference between camera and IR ball angles to use camera tracking
CAMERA_BALL_TRACKING_MAX_IR_DIFF_DEG = 45.0

# --- Goal-tracking rotation ---
# Gain applied to goal alignment error to produce rotation command while approaching/pushing
GOAL_TRACK_ROTATE_GAIN = 0.7
# Rotation speed used while searching for the goal (no goal visible)
GOAL_SEARCH_ROTATE_SPEED = 1.0


# ===================== GOALKEEPER SETTINGS =====================

# Switch between straight line and arc movement
GOALKEEPER_ARC_START_ANGLE_DEG = 45.0

# Distance from the goal line when tracking in a straight line
GOALKEEPER_DEFEND_LINE_DISTANCE_MM = 450.0

# Distance from our goal center while defending
GOALKEEPER_DEFEND_ARC_RADIUS_MM = 450.0

# Maximum sideways movement angle on the defend arc
GOALKEEPER_MAX_ARC_ANGLE_DEG = 70.0

# How strongly the robot follows the ball angle on the arc
GOALKEEPER_BALL_ANGLE_TO_ARC_RATIO = 0.7

# Position tolerance before entering defend state
GOALKEEPER_POSITION_TOLERANCE_MM = 120.0

# Goal distance limit before re-entering approach
GOALKEEPER_MAX_DEFEND_GOAL_DISTANCE_MM = 800.0

# How long we may lose the goal before recovering
GOALKEEPER_GOAL_LOST_TOLERANCE_TICKS = 15

# Movement speeds
GOALKEEPER_APPROACH_MAX_SPEED = 1.0
GOALKEEPER_DEFEND_MAX_SPEED = 0.7
GOALKEEPER_RECOVER_SPEED = 0.35

# Goal-line protection
GOALKEEPER_GOAL_LINE_SENSORS_IDX = [10, 11, 0, 1, 2]
GOALKEEPER_GOAL_LINE_PUSHOFF_TICKS = 12
GOALKEEPER_GOAL_LINE_PUSHOFF_SPEED = 0.6

# When to attack the ball
GOALKEEPER_NEAR_BALL_ANGLE_THRESHOLD_DEG = 35.0
GOALKEEPER_NEAR_BALL_CAMERA_DISTANCE_MM = 300.0
GOALKEEPER_NEAR_BALL_IR_THRESHOLD = IR_BALL_CLOSE_THRESHOLD

# ===================== BLUETOOTH SETTINGS =====================
ROLE_SYNC_SAME_TIME_TOLERANCE = 0.5


@profile_function
def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@overload
def _neutral_state(state_machine: StateMachine) -> type[State]:
    ...

@overload
def _neutral_state(is_goalkeeper: bool) -> type[State]:
    ...

@profile_function
def _neutral_state(state_machine_or_is_goalkeeper: StateMachine | bool) -> type[State]:
    if isinstance(state_machine_or_is_goalkeeper, StateMachine):
        is_goalkeeper = (
            isinstance(state_machine_or_is_goalkeeper.cross_state_data, SoccerStateMachineData)
            and state_machine_or_is_goalkeeper.cross_state_data.is_goalkeeper
        )
    else:
        is_goalkeeper = state_machine_or_is_goalkeeper

    return GoalkeeperApproachState if is_goalkeeper else IdleState


@profile_function
def _goalkeeper_should_grab_ball(data: SoccerStateMachineData) -> bool:
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


@profile_function
def _get_goal_tracking_rotation(
    state_machine: StateMachine,
    data: SoccerStateMachineData,
    own_goal: bool = False,
) -> float:
    target_camera_yaw = 180 if own_goal else 0
    goal = data.sensors.own_goal if own_goal else data.sensors.enemy_goal

    if goal.detected and goal.camera_yaw_deg == target_camera_yaw:
        rotate = goal.alignment * GOAL_TRACK_ROTATE_GAIN
        return _clamp(rotate, -1.0, 1.0)
    else:
        state_machine.motors.target_heading = 0.0
        return 0.0


@profile_function
def _field_delta_to_global_angle_deg(delta_x_mm: float, delta_y_mm: float) -> float:
    return (math.degrees(math.atan2(delta_x_mm, -delta_y_mm)) + 360.0) % 360.0


@profile_function
def _global_to_local_angle_deg(global_angle_deg: float, heading_deg: float) -> float:
    return utils.normalize_angle_deg(global_angle_deg - heading_deg)


@profile_function
def _goalkeeper_goal_line_sensor_fired(lines_detected: list[bool]) -> bool:
    for idx in GOALKEEPER_GOAL_LINE_SENSORS_IDX:
        if idx < len(lines_detected) and lines_detected[idx]:
            return True
    return False



@dataclass
class SensorsData:
    heading: float
    enemy_goal: GoalDetectionResult
    own_goal: GoalDetectionResult
    
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

    ball_angle: float | None
    ball_possession: bool
    _ball_possession_ticks: int
    ball_likely_inside_robot: bool

@dataclass
class LinesData:
    detected: list[bool]
    enter_avoiding_state: bool
    detection_history: list[tuple[list[bool], float]] = field(default_factory=list)  # History of recent detections with timestamps
    raw_detected: list[bool] = field(default_factory=list)

@dataclass
class BluetoothSyncData:
    last_teammate_penalty_exit: float | None = None
    last_our_penalty_exit: float | None = None


@dataclass
class SoccerStateMachineData(CrossStateData):
    sensors: SensorsData
    lines: LinesData
    bluetooth: BluetoothSyncData
    is_goalkeeper: bool = False
    motors_running: bool = True
    motors_running_prev: bool = True
    penalty_exited: bool = False
    penalty_entered: bool = False


@profile_function
def _update_sensors_data(state_machine: StateMachine) -> SensorsData:
    prev_data        = state_machine.cross_state_data.sensors if isinstance(state_machine.cross_state_data, SoccerStateMachineData) else None

    enemy_goal_color = shared_data.get_goal_color().lower()
    own_goal_color   = "blue" if enemy_goal_color == "yellow" else "yellow"
    enemy_goal       = shared_data.get_goal_detection_result_for_color(enemy_goal_color)
    own_goal         = shared_data.get_goal_detection_result_for_color(own_goal_color)
    if enemy_goal is None: enemy_goal = GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    if own_goal   is None: own_goal   = GoalDetectionResult(0.0, False, None, 0.0, None, 0.0)
    hardware         = shared_data.get_hardware_compass_ir()
    heading          = utils.normalize_angle_deg(hardware[0]) if hardware[0] != 999.0 else 999.0

    ir_ball_angle    = utils.normalize_angle_deg(hardware[1]) if hardware[1] != 999.0 else 999.0
    ir_ball_distance = hardware[2]
    ir_ball_detected = ir_ball_angle != 999 and ir_ball_distance != 0
    ir_possession    = (
        ir_ball_detected
        and abs(ir_ball_angle) < BALL_FRONT_THRESHOLD_DEG
        and ir_ball_distance > IR_BALL_CLOSE_THRESHOLD # The IR distance value is flipped (higher means closer)
    )

    cam_ball         = shared_data.get_camera_ball_data()
    cam_ball.angle   = utils.normalize_angle_deg(cam_ball.angle) if cam_ball and cam_ball.detected else 999
    cam_ball_valid   = (
        cam_ball.detected and cam_ball.distance <= CAMERA_BALL_TRACKING_MAX_DISTANCE_MM
        and (not ir_ball_detected or abs(ir_ball_angle - cam_ball.angle) < CAMERA_BALL_TRACKING_MAX_IR_DIFF_DEG)
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
        and (not ir_ball_detected or abs(ir_ball_angle) > BALL_INSIDE_ROBOT_IR_ANGLE_RANGE_DEG)
    )

    return SensorsData(
        heading,
        enemy_goal,
        own_goal,
        
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
        
        cam_ball.angle if use_cam_ball else ir_ball_angle if ir_ball_detected else None,
        ball_possession,
        _ball_possession_ticks,
        ball_likely_inside_robot
    )


@profile_function
def _update_lines_data(state_machine: StateMachine) -> LinesData:
    prev_data = state_machine.cross_state_data.lines if isinstance(state_machine.cross_state_data, SoccerStateMachineData) else None

    raw_detected = line_sensors.get_line_detected()
    detected = raw_detected

    enter_avoiding_state = any(detected) and shared_data.get_line_avoiding_enabled()
    
    current_time = time.time()
    history = prev_data.detection_history if prev_data and prev_data.detection_history else []
    history.append((detected.copy(), current_time))
    
    history_max_age = 0.5
    history = [(d, t) for d, t in history if current_time - t < history_max_age]
    
    return LinesData(
        detected=detected,
        enter_avoiding_state=enter_avoiding_state,
        detection_history=history,
        raw_detected=raw_detected
    )

@profile_function
def _update_soccer_data(state_machine: StateMachine, is_goalkeeper: bool | None = None) -> SoccerStateMachineData:
    prev_data = state_machine.cross_state_data if isinstance(state_machine.cross_state_data, SoccerStateMachineData) else None
    is_goalkeeper = is_goalkeeper if is_goalkeeper is not None else (prev_data.is_goalkeeper if prev_data else False)

    sensors = _update_sensors_data(state_machine)
    lines = _update_lines_data(state_machine)

    running_state = shared_data.get_running_state()
    motors_running = running_state.running if running_state is not None else True
    motors_running_prev = prev_data.motors_running if prev_data is not None else motors_running

    penalty_entered = not motors_running and motors_running_prev
    penalty_exited = motors_running and not motors_running_prev

    bluetooth = prev_data.bluetooth if prev_data is not None else BluetoothSyncData()

    data = SoccerStateMachineData(
        sensors=sensors,
        lines=lines,
        bluetooth=bluetooth,
        is_goalkeeper=is_goalkeeper,
        motors_running=motors_running,
        motors_running_prev=motors_running_prev,
        penalty_entered=penalty_entered,
        penalty_exited=penalty_exited,
    )

    state_machine.cross_state_data = data
    return data


@profile_function
def _check_role_update(state_machine: StateMachine) -> bool:
    if not isinstance(state_machine.cross_state_data, SoccerStateMachineData):
        return False

    data = state_machine.cross_state_data
    preset_goalkeeper = _get_preset_is_goalkeeper(state_machine)

    bluetooth_enabled = bluetooth.get_bluetooth_enabled()
    teammate_mac = get_teammate_mac_addr()
    teammate_connected = is_teammate_connected() if teammate_mac else False
    can_sync_roles = bluetooth_enabled and bool(teammate_mac) and teammate_connected

    if not can_sync_roles:
        return False

    role_changed = False

    if data.penalty_entered:
        if change_role(
            state_machine,
            is_goalkeeper=True,
            state=_neutral_state(True),
            bt_message="penalty_entered",
        ):
            role_changed = True

    if data.penalty_exited:
        data.bluetooth.last_our_penalty_exit = time.time()

        same_time = time.time() - (data.bluetooth.last_teammate_penalty_exit or 0) < ROLE_SYNC_SAME_TIME_TOLERANCE
        reset_roles = not preset_goalkeeper and same_time
        if change_role(
            state_machine,
            is_goalkeeper=False,
            state=_neutral_state(False),
            bt_message="reset_roles" if reset_roles else "penalty_exited",
        ):
            role_changed = True

    for msg in shared_data.get_bluetooth_new_received_messages():
        if msg.message_type == "role_update":
            if LOG_ROLE_CHANGES:
                logger.info(f"Received role update: {msg.content}")
            match msg.content:
                case "goalkeeper_has_ball":
                    if change_role(
                        state_machine,
                        is_goalkeeper=True,
                        state=_neutral_state(True),
                    ):
                        role_changed = True

                case "penalty_entered":
                    if change_role(
                        state_machine,
                        is_goalkeeper=False,
                        state=_neutral_state(False),
                    ):
                        role_changed = True

                case "penalty_exited":
                    data.bluetooth.last_teammate_penalty_exit = time.time()

                    same_time = time.time() - (data.bluetooth.last_our_penalty_exit or 0) < ROLE_SYNC_SAME_TIME_TOLERANCE
                    reset_roles = not preset_goalkeeper and same_time
                    is_goalkeeper = False if reset_roles else True
                    if change_role(
                        state_machine,
                        is_goalkeeper=is_goalkeeper,
                        state=_neutral_state(is_goalkeeper),
                        bt_message="reset_roles" if reset_roles else None,
                    ):
                        role_changed = True

                case "reset_roles":
                    if change_role(
                        state_machine,
                        is_goalkeeper=preset_goalkeeper,
                        state=_neutral_state(preset_goalkeeper),
                    ):
                        role_changed = True

    if role_changed:
        shared_data.set_autonomous_status_text(AutonomousStatusText.GOALKEEPER if data.is_goalkeeper else AutonomousStatusText.ATTACKER)

    return role_changed


@profile_function
def get_teammate_mac_addr() -> str:
    other_robot = bluetooth.get_other_robot_info()
    return other_robot.mac_address if other_robot else ""


@profile_function
def is_teammate_connected() -> bool:
    teammate_mac = get_teammate_mac_addr()
    if not teammate_mac:
        return False

    for device in bluetooth.get_connected_devices():
        if device.mac_address == teammate_mac:
            return True
    return False


@profile_function
def _get_preset_is_goalkeeper(state_machine: StateMachine) -> bool:
    return state_machine.name == GOALKEEPER_STATE_MACHINE_NAME


@profile_function
def reset_role_to_preset(state_machine: StateMachine, force_state_transition: bool = False):
    preset_goalkeeper = _get_preset_is_goalkeeper(state_machine)
    change_role(
        state_machine,
        is_goalkeeper=preset_goalkeeper,
        state=_neutral_state(preset_goalkeeper),
        force_state_transition=force_state_transition,
    )


@profile_function
def change_role(
    state_machine: StateMachine,
    is_goalkeeper: bool,
    state: type[State] | None = None,
    bt_message: str | None = None,
    require_sync_for_role_change: bool = False,
    force_state_transition: bool = False,
) -> bool:
    if not isinstance(state_machine.cross_state_data, SoccerStateMachineData):
        return False

    bluetooth_enabled = bluetooth.get_bluetooth_enabled()
    teammate_mac = get_teammate_mac_addr()
    teammate_connected = is_teammate_connected() if teammate_mac else False
    can_sync_roles = bluetooth_enabled and bool(teammate_mac) and teammate_connected

    prev_is_goalkeeper = state_machine.cross_state_data.is_goalkeeper

    if require_sync_for_role_change and not can_sync_roles:
        if state:
            state_machine.transition(state)
            if LOG_ROLE_CHANGES:
                logger.info(f"Cannot sync roles, transitioning to: {state}")
    else:
        state_machine.cross_state_data.is_goalkeeper = is_goalkeeper

        role_changed = prev_is_goalkeeper != is_goalkeeper
        can_transition = force_state_transition or (role_changed and type(state_machine.current_state) not in [LineAvoidingState, AttackerPushState])
        if state is not None and type(state_machine.current_state) != state and can_transition:
            state_machine.transition(state)

        shared_data.set_autonomous_status_text(AutonomousStatusText.GOALKEEPER if is_goalkeeper else AutonomousStatusText.ATTACKER)

        if LOG_ROLE_CHANGES:
            logger.info(f"Role changed: {"goalkeeper" if prev_is_goalkeeper else "attacker"} -> {"goalkeeper" if is_goalkeeper else "attacker"} ({state() if state is not None else "None"})")

    if bt_message is not None and can_sync_roles:
        bluetooth.send_message_nowait(teammate_mac, "role_update", bt_message)
        if LOG_ROLE_CHANGES:
            logger.info(f"Sent role update: {bt_message}")

    return True


# -------------------------------------------------------------------
# State: LINE AVOIDING
# -------------------------------------------------------------------
class LineAvoidingState(State):
    def __str__(self) -> str:
        return "Line Avoiding State"

    @profile_function
    def on_enter(self, state_machine: StateMachine) -> None:
        self.min_clear_time = 0.5
        self.clear_time_start = None
        self.line_exit_time = None

        self.safe_target = None

        self.safe_margin = 450
        self.target_tolerance = 120

        state_machine.motors.set_functions_enabled(position_based_speed_enabled=False)

    @profile_function
    def on_exit(self, state_machine: StateMachine) -> None:
        state_machine.motors.set_functions_enabled(position_based_speed_enabled=True)

    @profile_function
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_soccer_data(state_machine)

        if _check_role_update(state_machine):
            return

        current_time = time.time()

        heading = data.sensors.heading if data.sensors.heading != 999.0 else 0.0
        position = vision.get_position_estimate()
        rotate = _get_goal_tracking_rotation(state_machine, data)

        line_detected = any(data.lines.detected)

        if line_detected:
            self.clear_time_start = None
            self.line_exit_time = None

            if position is not None:
                self.safe_target = self._calculate_safe_target(position.x_mm, position.y_mm)
        else:
            if self.clear_time_start is None:
                self.clear_time_start = current_time
                self.line_exit_time = current_time

        if position is not None and self.safe_target is not None:
            target_x, target_y = self.safe_target

            delta_x = target_x - position.x_mm
            delta_y = target_y - position.y_mm

            distance = math.hypot(delta_x, delta_y)

            global_angle = _field_delta_to_global_angle_deg(delta_x, delta_y)
            local_angle = _global_to_local_angle_deg(global_angle, heading)

            if (
                not line_detected
                and self.clear_time_start is not None
                and current_time - self.clear_time_start > self.min_clear_time
                and distance < self.target_tolerance
            ):
                state_machine.transition(_neutral_state(state_machine))
                return

            speed = _clamp(distance / 250.0, 0.4, 1.0)

            state_machine.motors.set_motors(
                angle=local_angle,
                speed=speed,
                rotate=rotate,
            )
        else:
            avoid_direction = self._fallback_sensor_avoid(data)
            relative_direction = (avoid_direction - heading) % 360

            state_machine.motors.set_motors(
                angle=relative_direction,
                speed=0.8,
                rotate=rotate,
            )

    @profile_function
    def _calculate_safe_target(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        min_x = -FIELD_WIDTH_MM / 2 + self.safe_margin
        max_x = FIELD_WIDTH_MM / 2 - self.safe_margin

        min_y = self.safe_margin
        max_y = FIELD_LENGTH_MM - self.safe_margin

        left_dist = x_mm + FIELD_WIDTH_MM / 2
        right_dist = FIELD_WIDTH_MM / 2 - x_mm

        enemy_dist = y_mm
        own_dist = FIELD_LENGTH_MM - y_mm

        nearest = min(
            left_dist,
            right_dist,
            enemy_dist,
            own_dist,
        )

        if nearest == left_dist:
            return (min_x, _clamp(y_mm, min_y, max_y))

        elif nearest == right_dist:
            return (max_x,_clamp(y_mm, min_y, max_y))

        elif nearest == enemy_dist:
            return (_clamp(x_mm, min_x, max_x), min_y)

        else:
            return (_clamp(x_mm, min_x, max_x), max_y)

    @profile_function
    def _fallback_sensor_avoid(
        self,
        data: SoccerStateMachineData,
    ) -> float:

        detected_angles = []

        heading = data.sensors.heading if data.sensors.heading != 999.0 else 0.0

        for i, detected in enumerate(data.lines.detected):
            if detected and i < len(LINE_SENSOR_LOCATIONS):
                detected_angles.append((LINE_SENSOR_LOCATIONS[i] + heading) % 360)

        if not detected_angles:
            return 0.0

        angles_rad = [
            math.radians(a)
            for a in detected_angles
        ]

        x = sum(math.cos(a) for a in angles_rad)
        y = sum(math.sin(a) for a in angles_rad)

        avg_angle = (math.degrees(math.atan2(y, x)) + 360) % 360

        return (avg_angle + 180) % 360


# ------------------------------------------------------------------
# State: IDLE
# No ball detected - hold still.
# ------------------------------------------------------------------
class IdleState(State):
    def __str__(self) -> str:
        return "Idle State"

    @profile_function
    def on_enter(self, state_machine: StateMachine) -> None:
        if LOG_ATTACKER_STATE:
            logger.info("Attacker idle: entering")

    @profile_function
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_soccer_data(state_machine)
        if _check_role_update(state_machine):
            return

        if data.lines.enter_avoiding_state:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker idle: line detected, avoiding")
            state_machine.transition(LineAvoidingState)
            return

        if data.is_goalkeeper:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker idle: role is goalkeeper, switching to approach")
            state_machine.transition(GoalkeeperApproachState)
            return

        if data.sensors.ir_ball_detected or data.sensors.cam_ball_detected or data.sensors.ball_likely_inside_robot:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker idle: ball detected, switching to approach")
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
    def __str__(self) -> str:
        return "Attacker Approach State"

    @profile_function
    def on_enter(self, state_machine: StateMachine) -> None:
        if LOG_ATTACKER_STATE:
            logger.info("Attacker approach: entering")

    @profile_function
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_soccer_data(state_machine)
        if _check_role_update(state_machine):
            return

        if data.lines.enter_avoiding_state:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker approach: line detected, avoiding")
            state_machine.transition(LineAvoidingState)
            return

        if data.sensors.ball_possession or data.sensors.ball_likely_inside_robot:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker approach: ball possessed, switching to push")
            state_machine.transition(AttackerPushState)
            return

        if not data.sensors.ir_ball_detected and (not data.sensors.cam_ball_detected or not data.sensors.use_cam_ball):
            if LOG_ATTACKER_STATE:
                logger.info("Attacker approach: ball lost, switching to neutral")
            state_machine.transition(_neutral_state(state_machine))
            return

        move_angle = 0.0
        move_speed = APPROACH_SPEED
        rotate = _get_goal_tracking_rotation(state_machine, data)

        if data.sensors.use_cam_ball:
            if abs(data.sensors.cam_ball_angle) > BALL_POSSESSION_AREA_WIDTH_DEG / 2.0:
                move_angle = data.sensors.cam_ball_angle * CAM_BALL_APPROACH_ANGLE_RATIO
                move_angle = max(-180, min(180, move_angle))
        else:
            move_angle = utils.normalize_angle_deg(data.sensors.ir_ball_angle + data.sensors.heading) * data.sensors.ir_ball_distance * IR_BALL_APPROACH_ANGLE_RATIO
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
    def __str__(self) -> str:
        return "Attacker Push State"

    @profile_function
    def on_enter(self, state_machine: StateMachine) -> None:
        if LOG_ATTACKER_STATE:
            logger.info("Attacker push: entering")

    @profile_function
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_soccer_data(state_machine)
        if _check_role_update(state_machine):
            return

        if data.lines.enter_avoiding_state:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker push: line detected, avoiding")
            state_machine.transition(LineAvoidingState)
            return

        if not data.sensors.ball_possession and not data.sensors.ball_likely_inside_robot:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker push: ball lost, switching to approach")
            state_machine.transition(GoalkeeperApproachState if data.is_goalkeeper else AttackerApproachState)
            return

        move_speed = PUSH_SPEED
        move_angle = 0.0

        if data.sensors.enemy_goal.distance_mm is not None and data.sensors.enemy_goal.distance_mm < GOAL_SCORED_DISTANCE_MM:
            if LOG_ATTACKER_STATE:
                logger.info("Attacker push: goal reached, switching to neutral")
            state_machine.transition(_neutral_state(state_machine))
            return

        rotate = _get_goal_tracking_rotation(state_machine, data)

        if data.sensors.use_cam_ball:
            move_angle = -data.sensors.cam_ball_angle / (BALL_POSSESSION_AREA_WIDTH_DEG / 2.0) * PUSH_STEERING_MAX_ANGLE_DEG
            move_angle = max(-90, min(90, move_angle))

        move_speed *= AUTO_SPEED_MULTIPLIER
        state_machine.motors.set_motors(angle=move_angle, speed=move_speed, rotate=rotate)


# ------------------------------------------------------------------
# State: GOALKEEPER APPROACH
# Navigate back to the defensive position in front of our goal.
# ------------------------------------------------------------------
class GoalkeeperApproachState(State):
    def __str__(self) -> str:
        return "Goalkeeper Approach State"

    @profile_function
    def on_enter(self, state_machine: StateMachine) -> None:
        self._goal_line_pushoff_ticks = 0
        if LOG_GOALKEEPER_STATE:
            logger.info("Goalkeeper approach: entering")
        state_machine.motors.set_functions_enabled(position_based_speed_enabled=False)

    @profile_function
    def on_exit(self, state_machine: StateMachine) -> None:
        state_machine.motors.set_functions_enabled(position_based_speed_enabled=True)

    @profile_function
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_soccer_data(state_machine)
        if _check_role_update(state_machine):
            return


        if _goalkeeper_goal_line_sensor_fired(data.lines.raw_detected):
            if self._goal_line_pushoff_ticks == 0:
                if LOG_GOALKEEPER_STATE:
                    logger.info("Goalkeeper approach: goal line detected, pushing off")
            self._goal_line_pushoff_ticks = GOALKEEPER_GOAL_LINE_PUSHOFF_TICKS

        if self._goal_line_pushoff_ticks > 0:
            self._goal_line_pushoff_ticks -= 1

            state_machine.motors.set_motors(
                angle=0.0,
                speed=GOALKEEPER_GOAL_LINE_PUSHOFF_SPEED * AUTO_SPEED_MULTIPLIER,
                rotate=0.0,
            )
            return


        if _goalkeeper_should_grab_ball(data):
            if LOG_GOALKEEPER_STATE:
                logger.info("Goalkeeper approach: ball close, switching to attacker")
            state_machine.transition(AttackerApproachState)
            return


        position = vision.get_position_estimate()

        if position is None:
            if LOG_GOALKEEPER_STATE:
                logger.info("Goalkeeper approach: no position estimate, recovering")
            state_machine.motors.set_motors(
                angle=180.0,
                speed=GOALKEEPER_RECOVER_SPEED * AUTO_SPEED_MULTIPLIER,
                rotate=0.0,
            )
            return

        target_x = 0.0
        target_y = FIELD_LENGTH_MM - GOALKEEPER_DEFEND_ARC_RADIUS_MM

        delta_x = target_x - position.x_mm
        delta_y = target_y - position.y_mm

        distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)


        if (
            data.sensors.own_goal.detected
            and distance <= GOALKEEPER_POSITION_TOLERANCE_MM
        ):
            if LOG_GOALKEEPER_STATE:
                logger.info(f"Goalkeeper approach: reached defend position (distance={distance:.2f}), switching to defend")
            state_machine.transition(GoalkeeperDefendState)
            return


        global_angle = _field_delta_to_global_angle_deg(delta_x, delta_y)

        heading = data.sensors.heading if data.sensors.heading != 999.0 else 0.0

        local_angle = _global_to_local_angle_deg(global_angle, heading)

        speed = _clamp(
            distance / 500.0,
            0.2,
            GOALKEEPER_APPROACH_MAX_SPEED
        )

        speed *= AUTO_SPEED_MULTIPLIER


        rotate = _get_goal_tracking_rotation(
            state_machine,
            data,
            own_goal=True,
        )

        state_machine.motors.set_motors(
            angle=local_angle,
            speed=speed,
            rotate=rotate,
        )


# ------------------------------------------------------------------
# State: GOALKEEPER DEFEND
# ------------------------------------------------------------------
class GoalkeeperDefendState(State):
    def __str__(self) -> str:
        return "Goalkeeper Defend State"

    @profile_function
    def on_enter(self, state_machine: StateMachine) -> None:
        self._goal_lost_ticks = 0
        self._goal_line_pushoff_ticks = 0
        if LOG_GOALKEEPER_STATE:
            logger.info("Goalkeeper defend: entering")
        state_machine.motors.set_functions_enabled(position_based_speed_enabled=False)

    @profile_function
    def on_exit(self, state_machine: StateMachine) -> None:
        state_machine.motors.set_functions_enabled(position_based_speed_enabled=True)

    @profile_function
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_soccer_data(state_machine)
        if _check_role_update(state_machine):
            return


        if _goalkeeper_goal_line_sensor_fired(data.lines.raw_detected):
            if self._goal_line_pushoff_ticks == 0:
                if LOG_GOALKEEPER_STATE:
                    logger.info("Goalkeeper defend: goal line detected, pushing off")
            self._goal_line_pushoff_ticks = GOALKEEPER_GOAL_LINE_PUSHOFF_TICKS

        if self._goal_line_pushoff_ticks > 0:
            self._goal_line_pushoff_ticks -= 1

            rotate = _get_goal_tracking_rotation(state_machine, data, own_goal=True)

            state_machine.motors.set_motors(
                angle=0.0,
                speed=GOALKEEPER_GOAL_LINE_PUSHOFF_SPEED * AUTO_SPEED_MULTIPLIER,
                rotate=rotate,
            )
            return


        if _goalkeeper_should_grab_ball(data):
            if LOG_GOALKEEPER_STATE:
                logger.info("Goalkeeper defend: ball close, switching to attacker")
            change_role(state_machine, is_goalkeeper=False, state=AttackerApproachState, bt_message="goalkeeper_has_ball", require_sync_for_role_change=True)
            return


        if data.sensors.own_goal.detected:
            self._goal_lost_ticks = 0
        else:
            self._goal_lost_ticks += 1

        position = vision.get_position_estimate()

        if position is None:
            if LOG_GOALKEEPER_STATE:
                logger.info("Goalkeeper defend: no position estimate, re-centering")
            state_machine.transition(GoalkeeperApproachState)
            return

        ball_visible = (
            data.sensors.ir_ball_detected
            or data.sensors.cam_ball_detected
            or data.sensors.ball_likely_inside_robot
        )
        goal_distance_mm = FIELD_LENGTH_MM - position.y_mm

        if not ball_visible:
            if self._goal_lost_ticks > GOALKEEPER_GOAL_LOST_TOLERANCE_TICKS:
                if LOG_GOALKEEPER_STATE:
                    logger.info(f"Goalkeeper defend: goal lost for {self._goal_lost_ticks} ticks, re-centering")
                state_machine.transition(GoalkeeperApproachState)
                return
            if goal_distance_mm > GOALKEEPER_MAX_DEFEND_GOAL_DISTANCE_MM:
                if LOG_GOALKEEPER_STATE:
                    logger.info(f"Goalkeeper defend: far from goal (distance={goal_distance_mm:.1f} mm), re-centering")
                state_machine.transition(GoalkeeperApproachState)
                return


        ball_angle = data.sensors.ball_angle

        if ball_angle is None:
            ball_angle = 0.0

        arc_angle = _clamp(
            ball_angle * GOALKEEPER_BALL_ANGLE_TO_ARC_RATIO,
            -GOALKEEPER_MAX_ARC_ANGLE_DEG,
            GOALKEEPER_MAX_ARC_ANGLE_DEG,
        )

        heading = (
            data.sensors.heading
            if data.sensors.heading != 999.0
            else 0.0
        )

        radius = GOALKEEPER_DEFEND_ARC_RADIUS_MM

        dx = position.x_mm
        dy = position.y_mm - FIELD_LENGTH_MM

        current_radius = math.hypot(dx, dy)

        current_arc_angle = math.degrees(
            math.atan2(
                position.x_mm,
                FIELD_LENGTH_MM - position.y_mm
            )
        )
        angle_error = utils.normalize_angle_deg(arc_angle - current_arc_angle)

        radius_error = radius - current_radius
        if abs(radius_error) < 20:
            radius_error = 0.0

        if current_radius > 1:
            rx = dx / current_radius
            ry = dy / current_radius
        else:
            rx = 0.0
            ry = -1.0

        if angle_error > 0:
            tx = -ry
            ty = rx
        else:
            tx = ry
            ty = -rx

        RADIAL_GAIN = 0.01

        vx = tx + rx * radius_error * RADIAL_GAIN
        vy = ty + ry * radius_error * RADIAL_GAIN

        if abs(angle_error) < 20.0 and abs(radius_error) < 20:
            speed = 0.0
            local_angle = 0.0
        else:
            global_angle = _field_delta_to_global_angle_deg(
                vx,
                vy
            )

            local_angle = _global_to_local_angle_deg(
                global_angle,
                heading
            )

            speed = _clamp(
                max(
                    abs(angle_error) / 20.0,
                    abs(radius_error) / 100.0
                ),
                0.0,
                GOALKEEPER_DEFEND_MAX_SPEED
            )

        speed *= AUTO_SPEED_MULTIPLIER

        rotate = _get_goal_tracking_rotation(
            state_machine,
            data,
            own_goal=True,
        )

        state_machine.motors.set_motors(
            angle=local_angle,
            speed=speed,
            rotate=rotate,
        )


_motors = SmartMotorsController()
_motors.set_functions_enabled(line_avoiding_enabled=False)
attacker_state_machine = StateMachine(
    name=ATTACKER_STATE_MACHINE_NAME,
    initial_state=IdleState,
    motors=_motors,
)
_update_soccer_data(attacker_state_machine, is_goalkeeper=False)
    
goalkeeper_state_machine = StateMachine(
    name=GOALKEEPER_STATE_MACHINE_NAME,
    initial_state=GoalkeeperApproachState,
    motors=_motors,
)
_update_soccer_data(goalkeeper_state_machine, is_goalkeeper=True)


def get_attacker_state_machine() -> StateMachine:
    return attacker_state_machine

def get_goalkeeper_state_machine() -> StateMachine:
    return goalkeeper_state_machine
