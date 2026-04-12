import time
from dataclasses import dataclass

from robot import utils
from robot.logic.autonomous_mode.state_machine import State, StateMachine, CrossStateData
from robot.multiprocessing import shared_data

from robot.vision import GoalDetectionResult
from robot.hardware.motors import SmartMotorsController
from robot.config import *



@dataclass
class SensorsData(CrossStateData):
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


def _update_cross_state_data(state_machine: StateMachine) -> SensorsData:
    prev_data        = state_machine.cross_state_data if isinstance(state_machine.cross_state_data, SensorsData) else None

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

    data = SensorsData(
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
    state_machine.cross_state_data = data
    return data



# ------------------------------------------------------------------
# State: IDLE
# No ball visible - hold still.
# ------------------------------------------------------------------
class IdleState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)
        if data.ir_ball_detected or data.cam_ball_detected or data.ball_likely_inside_robot:
            state_machine.transition(ApproachState)
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
class ApproachState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)

        if data.ball_possession or data.ball_likely_inside_robot:
            state_machine.transition(PushState)
            return

        if not data.ir_ball_detected and (not data.cam_ball_detected or not data.use_cam_ball):
            state_machine.transition(IdleState)
            return

        if not data.goal.detected:
            state_machine.motors.set_functions_enabled(rotation_correction_enabled=True)
            state_machine.motors.target_heading = 0
        else:
            state_machine.motors.set_functions_enabled(rotation_correction_enabled=False)

        move_angle = 0.0
        move_speed = AUTO_APPROACH_SPEED
        rotate = 0.0

        if data.use_cam_ball:
            if abs(data.cam_ball_angle) > AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0:
                move_angle = data.cam_ball_angle * AUTO_CAM_BALL_APPROACH_ANGLE_RATIO
                move_angle = max(-180, min(180, move_angle))
        else:
            move_angle = utils.normalize_angle_deg(data.ir_ball_angle + data.heading) * data.ir_ball_distance * AUTO_IR_BALL_APPROACH_ANGLE_RATIO
            move_angle = max(-180, min(180, move_angle))

        if data.goal.detected:
            rotate = data.goal.alignment * AUTO_GOAL_TRACK_ROTATE_GAIN * AUTO_SPEED_MULTIPLIER
            rotate = max(-1.0, min(1.0, rotate))

        move_speed *= AUTO_SPEED_MULTIPLIER
        state_machine.motors.set_motors(angle=move_angle, speed=move_speed, rotate=rotate)


# ------------------------------------------------------------------
# State: PUSH
# Ball is in the front pocket. Drive straight forward while keeping
# the goal centred via rotation. If the ball is lost, re-approach.
# ------------------------------------------------------------------
class PushState(State):
    def tick(self, state_machine: StateMachine) -> None:
        data = _update_cross_state_data(state_machine)

        if not data.ball_possession and not data.ball_likely_inside_robot:
            state_machine.transition(ApproachState)
            return

        if data.goal.detected and shared_data.get_always_facing_goal_enabled():
            if data.goal.distance_mm is not None and data.goal.distance_mm < AUTO_GOAL_SCORED_DISTANCE_MM:
                state_machine.transition(IdleState)
                return
            state_machine.motors.set_functions_enabled(rotation_correction_enabled=False)
        else:
            state_machine.motors.set_functions_enabled(rotation_correction_enabled=True)
            state_machine.motors.target_heading = 0

        move_speed = AUTO_PUSH_SPEED
        move_angle = 0.0
        rotate = 0.0

        if data.use_cam_ball:
            move_angle = -data.cam_ball_angle / (AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0) * AUTO_PUSH_STEERING_MAX_ANGLE_DEG
            move_angle = max(-90, min(90, move_angle))

        if data.goal.detected:
            rotate = data.sensors.goal.alignment * AUTO_GOAL_TRACK_ROTATE_GAIN * AUTO_SPEED_MULTIPLIER
            rotate = max(-1.0, min(1.0, rotate))

        move_speed *= AUTO_SPEED_MULTIPLIER
        state_machine.motors.set_motors(angle=move_angle, speed=move_speed, rotate=rotate)


_state_machine = StateMachine(name="Soccer State Machine", initial_state=IdleState, motors=SmartMotorsController())

def get_state_machine() -> StateMachine:
    return _state_machine

