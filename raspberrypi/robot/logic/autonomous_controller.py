import math
import time

from robot import utils, vision
from robot.multiprocessing import shared_data

from robot.hardware.motors import SmartMotorsController
from robot.config import *



# ==================== AUTONOMOUS CONTROLLER ====================


logger = utils.get_logger("Autonomous Controller")


class AutoState:
    IDLE          = "idle"          # no ball visible - hold position
    APPROACH      = "approach"      # drive toward ball, rotate to keep goal centred
    PUSH          = "push"          # ball possessed - drive forward, keep goal centred
    GOAL_SCORED   = "goal_scored"   # goal scored - wait until the ball isn't in the goal anymore

class AutonomousController:
    """
    Autonomous soccer behaviour.

    State flow:
        IDLE:
            - No ball visible. Hold position.
            - If ball detected (IR/camera) or likely inside robot -> APPROACH

        APPROACH:
            - Move toward ball, rotate to keep goal centered.
            - If ball lost (not detected by IR/camera and not likely inside) -> IDLE
            - If ball possessed or likely inside -> PUSH

        PUSH:
            - Ball is possessed, drive forward, keep goal centered.
            - If ball lost -> APPROACH
            - If goal is very close -> GOAL_SCORED

        GOAL_SCORED:
            - Goal scored, wait until ball is not in goal area.
            - If goal no longer detected or far -> IDLE
            - If ball detected outside goal area -> APPROACH

    No kicker - ball is pushed with the front cutout.
    """

    def __init__(self):
        self.state: str = AutoState.IDLE
        self.prev_state: str = ""
        self.state_start_time: float = time.time()
        self._possession_ticks: int = 0
        self._POSSESSION_CONFIRM_TICKS: int = 3
        self._ball_likely_inside_robot: bool = False

    # ------------------------------------------------------------------
    # Public tick - called every logic loop iteration
    # ------------------------------------------------------------------
    def tick(self, motors: SmartMotorsController) -> None:
        goal     = shared_data.get_goal_detection_result()
        hw       = shared_data.get_hardware_data()
        cam_ball = shared_data.get_camera_ball_data()

        heading           = hw.compass.heading if hw else None
        goal_visible      = (goal is not None and goal.goal_detected)
        ir_ball_angle     = utils.normalize_angle_deg(hw.ir.angle) if hw else 999
        ir_ball_distance  = hw.ir.distance if hw else 999
        ir_ball_detected  = (ir_ball_angle != 999 and ir_ball_distance != 0)
        cam_ball_angle    = cam_ball.angle
        cam_ball_distance = cam_ball.distance
        cam_ball_detected = cam_ball.detected
        cam_possession    = shared_data.get_camera_ball_possession()

        cam_data_valid    = (
            cam_ball_detected and cam_ball_distance <= AUTO_CAMERA_BALL_TRACKING_MAX_DISTANCE_MM and
            (abs(ir_ball_angle - cam_ball_angle) < AUTO_CAMERA_BALL_TRACKING_MAX_IR_DIFF_DEG or not ir_ball_detected)
        )
        use_cam_data      = shared_data.get_camera_ball_usage_enabled() and cam_data_valid

        if cam_possession: self._cam_possession_elapsed_ticks = 0
        else:              self._cam_possession_elapsed_ticks += 1
        ir_possession = (
            ir_ball_detected
            and abs(ir_ball_angle) < AUTO_BALL_FRONT_THRESHOLD_DEG
            and ir_ball_distance > AUTO_BALL_CLOSE_THRESHOLD # The IR distance value is flipped (higher means closer)
        )

        have_ball_raw = cam_possession if use_cam_data else ir_possession
        if have_ball_raw:
            self._possession_ticks = min(self._possession_ticks + 1, self._POSSESSION_CONFIRM_TICKS)
        else:
            self._possession_ticks = max(self._possession_ticks - 1, 0)
        have_ball = (self._possession_ticks >= self._POSSESSION_CONFIRM_TICKS)
        
        if self._ball_likely_inside_robot != True:
            self._ball_likely_inside_robot = False
            if self._cam_possession_elapsed_ticks < 3 and not cam_ball_detected:
                self._ball_likely_inside_robot = True
        if not ir_ball_detected or abs(ir_ball_angle) >= AUTO_BALL_INSIDE_ROBOT_IR_ANGLE_RANGE_DEG:
            self._ball_likely_inside_robot = False

        # ---- State machine --------------------------------------------
        if self.state == AutoState.IDLE:
            self._tick_idle(motors, ir_ball_detected, use_cam_data, self._ball_likely_inside_robot)

        elif self.state == AutoState.APPROACH:
            self._tick_approach(motors, heading, ir_ball_detected, ir_ball_angle, ir_ball_distance,
                                use_cam_data, cam_ball_angle, cam_ball_distance,
                                goal, goal_visible, have_ball, self._ball_likely_inside_robot)

        elif self.state == AutoState.PUSH:
            self._tick_push(motors, have_ball, self._ball_likely_inside_robot, goal, goal_visible, cam_data_valid, cam_possession, cam_ball_angle)

        elif self.state == AutoState.GOAL_SCORED:
            self._tick_goal_scored(motors, goal, goal_visible, ir_ball_detected, ir_ball_angle, cam_ball_detected, cam_ball_angle, cam_data_valid)

        self._publish_state(ir_ball_angle, ir_ball_distance, goal, have_ball)

    # ------------------------------------------------------------------
    # State: IDLE
    # No ball visible - hold still.
    # ------------------------------------------------------------------
    def _tick_idle(self, motors, ir_ball_detected, use_cam_data, ball_likely_inside_robot):
        if ir_ball_detected or use_cam_data or ball_likely_inside_robot:
            self._transition(AutoState.APPROACH,
                f"  - ir_ball_detected: {ir_ball_detected}",
                f"  - use_cam_data: {use_cam_data}",
                f"  - ball_likely_inside_robot: {ball_likely_inside_robot}",
            )
            return
        motors.set_motors(angle=0, speed=0, rotate=0)

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
    def _tick_approach(self,
        motors, heading, ir_ball_detected, ir_ball_angle, ir_ball_distance,
        use_cam_data, cam_ball_angle, cam_ball_distance,
        goal, goal_visible, have_ball, ball_likely_inside_robot
    ):
        if not ir_ball_detected and not use_cam_data and not ball_likely_inside_robot:
            self._transition(AutoState.IDLE,
                f"  - ir_ball_detected: {ir_ball_detected}",
                f"  - use_cam_data: {use_cam_data}",
                f"  - ball_likely_inside_robot: {ball_likely_inside_robot}",
            )
            return

        if have_ball or ball_likely_inside_robot:
            self._transition(AutoState.PUSH,
                f"  - have_ball: {have_ball}",
                f"  - use_cam_data: {use_cam_data}",
                f"  - ball_likely_inside_robot: {ball_likely_inside_robot}",
            )
            return

        fwd  = 0.0
        side = 0.0

        if use_cam_data:
            if cam_ball_distance is None or cam_ball_distance == 0 or cam_ball_angle is None or cam_ball_angle == 999:
                self._transition(AutoState.IDLE,
                    f"Invalid camera data:",
                    f"  - cam_ball_distance: {cam_ball_distance}",
                    f"  - cam_ball_angle: {cam_ball_angle}",
                )
                return
            angle_rad = 0.0
            if abs(cam_ball_angle) > AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0:
                angle_rad = math.radians(cam_ball_angle * AUTO_CAM_BALL_APPROACH_ANGLE_RATIO)
            # elif abs(cam_ball_angle) > AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0 * 0.5:
            #     angle_rad = math.radians(-cam_ball_angle * AUTO_CAM_BALL_APPROACH_ANGLE_RATIO)
            angle_rad = max(math.radians(-180), min(math.radians(180), angle_rad))
            speed = AUTO_APPROACH_SPEED

            fwd = math.cos(angle_rad) * speed
            side = math.sin(angle_rad) * speed
        else:
            if ir_ball_distance is None or ir_ball_distance == 0 or ir_ball_angle is None or ir_ball_angle == 999:
                self._transition(AutoState.IDLE,
                    f"Invalid IR data:",
                    f"  - ir_ball_distance: {ir_ball_distance}",
                    f"  - ir_ball_angle: {ir_ball_angle}",
                )
                return
            speed = AUTO_APPROACH_SPEED
            angle_rad = math.radians(utils.normalize_angle_deg(ir_ball_angle + heading) * ir_ball_distance * AUTO_IR_BALL_APPROACH_ANGLE_RATIO)
            angle_rad = max(math.radians(-180), min(math.radians(180), angle_rad))
            fwd = math.cos(angle_rad) * speed
            side = math.sin(angle_rad) * speed
    
        move_angle_deg = math.degrees(math.atan2(side, fwd)) - heading if heading else math.degrees(math.atan2(side, fwd))
        pos_factor = self._position_speed_factor()
        move_speed = min(1.0, math.sqrt(fwd**2 + side**2)) * AUTO_SPEED_MULTIPLIER * pos_factor

        rotate = self._goal_track_rotation(goal, goal_visible)

        motors.set_motors(angle=move_angle_deg, speed=move_speed, rotate=rotate)
        motors.target_heading = 0

    # ------------------------------------------------------------------
    # State: PUSH
    # Ball is in the front pocket. Drive straight forward while keeping
    # the goal centred via rotation. If the ball is lost, re-approach.
    # ------------------------------------------------------------------
    def _tick_push(self, motors, have_ball, ball_likely_inside_robot, goal, goal_visible, cam_data_valid, cam_possession, cam_ball_angle):
        if not have_ball and not ball_likely_inside_robot:
            self._possession_ticks = 0
            self._transition(AutoState.APPROACH,
                f"Ball lost during PUSH:",
                f"  - have_ball: {have_ball}",
                f"  - ball_likely_inside_robot: {ball_likely_inside_robot}",
            )
            return

        if goal_visible and goal.distance_mm is not None and goal.distance_mm < AUTO_GOAL_SCORED_DISTANCE_MM:
            self._transition(AutoState.GOAL_SCORED,
                f"  - goal.distance_mm: {goal.distance_mm}",
            )
            return

        pos_factor = self._position_speed_factor()
        move_speed = AUTO_PUSH_SPEED * AUTO_SPEED_MULTIPLIER * pos_factor
        move_angle = 0.0
        rotate = self._goal_track_rotation(goal, goal_visible)

        if cam_data_valid and cam_possession:
            move_angle = -cam_ball_angle / (AUTO_BALL_POSSESSION_AREA_WIDTH_DEG / 2.0) * AUTO_PUSH_STEERING_MAX_ANGLE_DEG
            move_angle = max(-90, min(90, move_angle))

        motors.set_motors(angle=move_angle, speed=move_speed, rotate=rotate)
        motors.target_heading = 0

    # ------------------------------------------------------------------
    # State: GOAL_SCORED
    # Goal is scored - wait until the ball isn't in the goal anymore,
    # then return to IDLE.
    # ------------------------------------------------------------------
    def _tick_goal_scored(self, motors, goal, goal_visible, ir_ball_detected, ir_ball_angle, cam_ball_detected, cam_ball_angle, cam_data_valid):
        motors.set_motors(angle=0, speed=0, rotate=0)
        if self.state_start_time is not None and time.time() - self.state_start_time <= 0.5:
            return
        if not goal_visible or goal.distance_mm is None or goal.distance_mm >= AUTO_GOAL_SCORED_DISTANCE_MM + 100:
            self._transition(AutoState.IDLE,
                f"Not in a \"goal scored\" position anymore:",
                f"  - goal_visible: {goal_visible}",
                f"  - goal.distance_mm: {goal.distance_mm if goal else None}",
            )
            return
        if shared_data.get_camera_ball_usage_enabled() and cam_data_valid:
            if abs(cam_ball_angle) > 30:
                self._transition(AutoState.APPROACH,
                    f"Ball detected by camera outside of goal area:",
                    f"  - cam_ball_detected: {cam_ball_detected}",
                    f"  - cam_ball_angle: {cam_ball_angle}",
                )
                return
        else:
            if not ir_ball_detected:
                self._transition(AutoState.IDLE,
                    f"Ball not detected:",
                    f"  - ir_ball_detected: {ir_ball_detected}",
                    f"  - camera_ball_usage_enabled: {shared_data.get_camera_ball_usage_enabled()}",
                    f"  - cam_data_valid: {cam_data_valid}",
                )
                return
            if abs(ir_ball_angle) > 30:
                self._transition(AutoState.APPROACH,
                    f"Ball detected by IR outside of goal area:",
                    f"  - ir_ball_detected: {ir_ball_detected}",
                    f"  - ir_ball_angle: {ir_ball_angle}",
                )
                return

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _goal_track_rotation(self, goal, goal_visible: bool) -> float:
        """
        Compute rotation command to keep goal centred in camera frame.

        alignment > 0 => goal to the right => rotate clockwise (positive).
        If goal not visible, rotate slowly to search for it.
        Returns a value in [-1, 1].
        """
        if goal_visible:
            rotate = goal.alignment * AUTO_GOAL_TRACK_ROTATE_GAIN * AUTO_SPEED_MULTIPLIER
            return max(-1.0, min(1.0, rotate))
        return 0

    def _position_speed_factor(self) -> float:
        """Get a speed multiplier based on the position on field to avoid lines/drive slower near them."""
        if not shared_data.get_position_based_speed_enabled():
            return 1.0
        pos = vision.get_position_estimate()
        if pos is None:
            return 1.0
        distances = [
            pos.x_mm - AUTO_POSITION_SLOW_START_DISTANCE_X_MM,
            AUTO_POSITION_SLOW_START_DISTANCE_Y_MIN_MM - pos.y_mm,
            pos.y_mm - AUTO_POSITION_SLOW_START_DISTANCE_Y_MAX_MM
        ]
        highest = max(*distances)
        highest = max(0.0, min(AUTO_POSITION_SLOW_END_DISTANCE_MM, highest))
        return 1.0 - (highest / AUTO_POSITION_SLOW_END_DISTANCE_MM) * AUTO_POSITION_SLOW_MIN_SPEED


    def _transition(self, new_state: str, *debug_messages: str):
        if new_state != self.state:
            debug_info = "\n".join(debug_messages) if debug_messages else ""
            debug_info_str = f"\n{debug_info}" if debug_info else ""
            logger.info(f"Autonomous: {self.state} -> {new_state}{debug_info_str}")
            self.prev_state = self.state
            self.state = new_state
            self.state_start_time = time.time()

    def _publish_state(self, ball_angle, ball_distance, goal, have_ball: bool):
        state_info = {
            'state': self.state,
            'ball_angle': int(ball_angle) if ball_angle != 999 else None,
            'ball_distance': int(ball_distance) if ball_distance != 999 else None,
            'have_ball': have_ball,
            'goal_detected': goal.goal_detected if goal else False,
            'goal_alignment': round(goal.alignment, 3) if goal else None,
            'goal_distance_mm': int(round(goal.distance_mm)) if goal and goal.distance_mm else None,
        }
        with shared_data.autonomous_state_lock:
            shared_data.autonomous_state.clear()
            shared_data.autonomous_state.update(state_info)

    def reset(self):
        self._possession_ticks = 0
        self._cam_possession_elapsed_ticks = 10000
        self._ball_likely_inside_robot = False
        self._transition(AutoState.IDLE)
        with shared_data.autonomous_state_lock:
            shared_data.autonomous_state.clear()

