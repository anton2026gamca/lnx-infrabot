import asyncio
import cv2
import multiprocessing.synchronize
import numpy as np
import uvicorn
import urllib.parse
from engineio.packet import base64

import socketio
from fastapi import FastAPI

from robot import calibration, utils, vision
from robot.hardware import line_sensors
from robot.logic import autonomous_mode
from robot.multiprocessing import shared_data
from robot.robot import RobotManualControl
from robot.vision import DetectedObject, PositionEstimate
from robot.config import *


# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------

logger = utils.get_logger("API Process")

sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins="*",
    max_http_buffer_size=5 * 1024 * 1024,
    logger=False,
    engineio_logger=False,
)

fastapi_app = FastAPI()
app = socketio.ASGIApp(sio, other_asgi_app=fastapi_app)

@fastapi_app.on_event("startup")
async def _on_startup():
    await _start_monitoring_loop()

@fastapi_app.on_event("shutdown")
async def _on_shutdown():
    await _stop_monitoring_loop()

_video_tasks: dict[str, asyncio.Task] = {}
_update_subscriptions: dict[str, dict] = {}  # {sid: {subscribed_updates: set, task: asyncio.Task}}
_state_tracker: dict = {  # Track previous state for change detection
    "robot_mode": None,
    "goal_color": None,
    "line_detected": None,
    "ball_detected": None,
    "goal_detected": None,
    "last_log_id": 0,
}
_state_tracker_lock = asyncio.Lock()

encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), API_VIDEO_JPEG_QUALITY]

_state_monitor_task: asyncio.Task | None = None


# ---------------------------------------------------------------------------
# State change monitoring and subscriptions
# ---------------------------------------------------------------------------

async def _start_monitoring_loop():
    global _state_monitor_task
    _state_monitor_task = asyncio.create_task(_monitor_state_changes())
    logger.info("State monitor task started")

async def _stop_monitoring_loop():
    global _state_monitor_task
    if _state_monitor_task:
        _state_monitor_task.cancel()
        try:
            await _state_monitor_task
        except asyncio.CancelledError:
            pass
    logger.info("State monitor task stopped")

async def _monitor_state_changes() -> None:
    """Continuously monitor state changes and emit updates to subscribed clients."""
    global _state_tracker
    check_interval = 0.1  # Check for changes every 100ms
    
    try:
        while True:
            await asyncio.sleep(check_interval)
            
            try:
                # Get current state
                current_mode = shared_data.get_robot_mode()
                current_goal_color = shared_data.get_goal_color()
                current_line_detected = line_sensors.get_line_detected()
                current_hw_data = shared_data.get_hardware_data()
                current_ir_data = current_hw_data.ir if current_hw_data else None
                current_cam_data = shared_data.get_camera_ball_data()
                
                current_ir_detected = current_ir_data.angle != 999 if current_ir_data else False
                current_cam_detected = current_cam_data.detected if current_cam_data else False
                current_ball_detected = current_cam_detected or current_ir_detected

                logs, last_id = utils.get_logs(_state_tracker["last_log_id"])

                async with _state_tracker_lock:
                    if current_mode != _state_tracker["robot_mode"]:
                        _state_tracker["robot_mode"] = current_mode
                        mode_name = ["idle", "manual", "autonomous"][current_mode] if current_mode in [0, 1, 2] else "unknown"
                        await _broadcast_update("mode_changed", {"mode": mode_name})
                    
                    if current_goal_color != _state_tracker["goal_color"]:
                        _state_tracker["goal_color"] = current_goal_color
                        await _broadcast_update("goal_color_changed", {"goal_color": current_goal_color})
                    
                    if (current_line_detected != _state_tracker["line_detected"] or
                        current_ir_detected != _state_tracker["ir_detected"] or
                        current_ball_detected != _state_tracker["ball_detected"]
                    ):  
                        _state_tracker["line_detected"] = current_line_detected
                        _state_tracker["ir_detected"] = current_ir_detected
                        _state_tracker["ball_detected"] = current_ball_detected
                        await _broadcast_update("important_sensor_data_change", create_sensor_data())

                    if last_id > _state_tracker["last_log_id"]:
                        _state_tracker["last_log_id"] = last_id
                        await _broadcast_update("new_logs", {"logs": logs})
                        
            except Exception as exc:
                logger.error(f"Error in state monitoring: {exc}", exc_info=True)
                
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        logger.error(f"State monitoring loop error: {exc}", exc_info=True)


async def _broadcast_update(event_name: str, data: dict) -> None:
    """Broadcast an update event to all subscribed clients."""
    sids_to_remove = []
    for sid, sub_info in _update_subscriptions.items():
        if event_name in sub_info["subscribed_updates"]:
            try:
                await sio.emit(event_name, data, to=sid)
            except Exception as exc:
                logger.debug(f"Error emitting {event_name} to {sid}: {exc}")
                sids_to_remove.append(sid)
    
    # Clean up disconnected clients
    for sid in sids_to_remove:
        _update_subscriptions.pop(sid, None)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ok(**kwargs) -> dict:
    return {"status": "ok", **kwargs}


def _err(msg: str) -> dict:
    return {"status": "error", "error": msg}


def _build_detected_objects(detections: list) -> list[DetectedObject]:
    result = []
    for det in detections:
        if isinstance(det, dict):
            color = tuple(det.get("color", (255, 255, 255))) if det.get("color") else (255, 255, 255)
            result.append(DetectedObject(
                object_type=det.get("object_type", "unknown"),
                x=det.get("x", 0),
                y=det.get("y", 0),
                width=det.get("width", 0),
                height=det.get("height", 0),
                confidence=det.get("confidence", 0.0),
                color=color,
            ))
        else:
            result.append(det)
    return result


# ---------------------------------------------------------------------------
# Video streaming
# ---------------------------------------------------------------------------

async def _video_loop(sid: str, fps: float, show_detections: bool) -> None:
    """Push binary JPEG frames to a single client until cancelled."""
    sleep_time = 1.0 / fps if fps > 0 else 0
    try:
        while True:
            if sleep_time:
                await asyncio.sleep(sleep_time)

            frame_data = shared_data.get_camera_frame()
            if frame_data is None or frame_data.frame is None:
                continue

            frame = frame_data.frame

            if show_detections:
                try:
                    detections = shared_data.get_detected_objects()
                    if isinstance(detections, list):
                        objects = _build_detected_objects(detections)
                        if objects:
                            frame = vision.draw_detections_on_frame(frame, objects, draw_labels=True)
                except Exception as exc:
                    logger.debug(f"Error drawing detections: {exc}")

            ret, buffer = cv2.imencode(".jpg", frame, encode_params)
            if not ret:
                continue

            await sio.emit("video_frame", buffer.tobytes(), to=sid)
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        logger.error(f"Video loop error for {sid}: {exc}", exc_info=True)


# ---------------------------------------------------------------------------
# Connection lifecycle
# ---------------------------------------------------------------------------

@sio.event
async def connect(sid: str, environ: dict, auth: dict | None = None):
    expected_token = base64.b64encode(AUTH_TOKEN.encode()).decode() if AUTH_TOKEN else None

    query = environ.get("QUERY_STRING", "")
    params = dict(qc.split("=", 1) for qc in query.split("&") if "=" in qc)
    token = params.get("token")
    if token is not None:
        token = urllib.parse.unquote(token)

    if expected_token is None or token != expected_token:
        logger.warning(f"Unauthorized connection attempt from {sid} with token: {token}, expected: {expected_token}")
        return False
    logger.info(f"Client connected: {sid}")
    return True


@sio.event
async def disconnect(sid: str):
    logger.info(f"Client disconnected: {sid}")
    task = _video_tasks.pop(sid, None)
    if task:
        task.cancel()
    _update_subscriptions.pop(sid, None)


# ---------------------------------------------------------------------------
# Update subscriptions
# ---------------------------------------------------------------------------

@sio.event
async def subscribe_updates(sid: str, data: dict | None = None):
    """
    Subscribe to state change notifications.
    """
    data = data or {}
    updates = data.get("updates", {})

    if not isinstance(updates, dict):
        return _err("updates must be a dictionary of [event: bool]")

    valid_events = {
        "mode_changed",
        "goal_color_changed",
        "important_sensor_data_change",
        "new_logs",
    }

    current = _update_subscriptions.get(sid, {}).get("subscribed_updates", set())

    subscribed = set()
    for event in valid_events:
        if updates.get(event, None) is not False:
            if event in current or updates.get(event, False):
                subscribed.add(event)

    if not subscribed:
        return _err(f"No valid updates specified. Valid events: {', '.join(sorted(valid_events))}")

    _update_subscriptions[sid] = {"subscribed_updates": subscribed}
    logger.info(f"Client {sid} subscribed to updates: {subscribed}")
    return _ok(message=f"Subscribed to {len(subscribed)} update(s)")


@sio.event
async def unsubscribe_updates(sid: str, data: dict | None = None):
    """Unsubscribe from state change notifications."""
    if sid in _update_subscriptions:
        _update_subscriptions.pop(sid)
        logger.info(f"Client {sid} unsubscribed from updates")
    return _ok()


# ---------------------------------------------------------------------------
# Video subscription
# ---------------------------------------------------------------------------

@sio.event
async def subscribe_video(sid: str, data: dict | None = None):
    """
    Client -> subscribe_video  { fps?: number, show_detections?: bool }
    Server -> ack              { status: "ok" | "error" }
    Server -> video_frame      <bytes>   (repeated until unsubscribed)
    """
    data = data or {}
    if sid in _video_tasks:
        return _ok(message="already subscribed")

    try:
        fps = float(data.get("fps", API_VIDEO_TARGET_FPS))
        if fps <= 0:
            fps = API_VIDEO_TARGET_FPS
    except (TypeError, ValueError):
        fps = API_VIDEO_TARGET_FPS

    show_detections = bool(data.get("show_detections", True))

    task = asyncio.create_task(_video_loop(sid, fps, show_detections))
    _video_tasks[sid] = task
    return _ok()


@sio.event
async def unsubscribe_video(sid: str, data: dict | None = None):
    """Stop sending video frames to this client."""
    task = _video_tasks.pop(sid, None)
    if task:
        task.cancel()
    return _ok()


# ---------------------------------------------------------------------------
# Sensor / state queries
# ---------------------------------------------------------------------------

def create_sensor_data() -> dict:
    hw = shared_data.get_hardware_data()
    if hw is None:
        return _err("No data available yet")

    running_state = shared_data.get_running_state()
    cam = shared_data.get_camera_ball_data()

    return _ok(
        compass={
            "heading": hw.compass.heading,
            "pitch": hw.compass.pitch,
            "roll": hw.compass.roll,
        },
        ir={
            "angle": hw.ir.angle,
            "distance": hw.ir.distance,
            "sensors": hw.ir.sensors,
            "status": hw.ir.status,
        },
        camera_ball={
            "angle": cam.angle if cam else None,
            "distance": cam.distance if cam else None,
            "detected": cam.detected if cam else None,
        },
        line={
            "raw": hw.line,
            "detected": line_sensors.get_line_detected(),
            "thresholds": shared_data.get_line_detection_thresholds(),
        },
        motors=shared_data.get_motor_speeds(),
        kicker=shared_data.get_kicker_state(),
        running_state={
            "running": running_state.running if running_state else False,
            "bt_module_enabled": running_state.bt_module_enabled if running_state else False,
            "bt_module_state": running_state.bt_module_state if running_state else False,
            "switch_state": running_state.switch_state if running_state else False,
        } if running_state else None,
        timestamp=hw.timestamp,
    )

@sio.event
async def get_sensor_data(sid: str | None = None, data: dict | None = None):
    try:
        return create_sensor_data()
    except Exception as exc:
        logger.error(f"get_sensor_data: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_logs(sid: str, data: dict | None = None):
    try:
        since_id = int((data or {}).get("since", 0))
        items, last_id = utils.get_logs(since_id)
        return _ok(logs=items, last_id=last_id)
    except Exception as exc:
        logger.error(f"get_logs: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_mode(sid: str, data: dict | None = None):
    try:
        mode = shared_data.get_robot_mode()
        if mode not in [0, 1, 2]:
            raise ValueError(f"Invalid robot mode: {mode}")
        return _ok(mode=["idle", "manual", "autonomous"][mode])
    except Exception as exc:
        logger.error(f"get_mode: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_motor_settings(sid: str, data: dict | None = None):
    try:
        return _ok(
            rotation_correction_enabled=shared_data.get_rotation_correction_enabled(),
            line_avoiding_enabled=shared_data.get_line_avoiding_enabled(),
            position_based_speed_enabled=shared_data.get_position_based_speed_enabled(),
        )
    except Exception as exc:
        logger.error(f"get_motor_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_settings(sid: str, data: dict | None = None):
    try:
        goal_color = shared_data.get_goal_color()
        y_ranges = shared_data.get_goal_calibration("yellow")
        b_ranges = shared_data.get_goal_calibration("blue")
        
        y_ranges_list = [{"lower": list(lower), "upper": list(upper)} for lower, upper in y_ranges]
        b_ranges_list = [{"lower": list(lower), "upper": list(upper)} for lower, upper in b_ranges]
        
        return _ok(
            goal_color=goal_color,
            calibration={
                "yellow": {"ranges": y_ranges_list},
                "blue":   {"ranges": b_ranges_list},
            },
        )
    except Exception as exc:
        logger.error(f"get_goal_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_detection(sid: str, data: dict | None = None):
    try:
        result = shared_data.get_goal_detection_result()
        if result is None:
            return _ok(goal_detected=False, alignment=0.0, goal_center_x=None, goal_area=0.0)
        return _ok(
            goal_detected=result.detected,
            alignment=result.alignment,
            goal_center_x=result.center_x,
            goal_area=result.area,
            distance_mm=result.distance_mm,
            goal_height_pixels=result.height_pixels,
        )
    except Exception as exc:
        logger.error(f"get_goal_detection: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_position_estimate(sid: str, data: dict | None = None):
    try:
        position: PositionEstimate | None = vision.get_position_estimate()
        if position is None:
            return _ok(x_mm=None, y_mm=None, confidence=0.0)
        return _ok(x_mm=position.x_mm, y_mm=position.y_mm, confidence=position.confidence)
    except Exception as exc:
        logger.error(f"get_position_estimate: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_detections(sid: str, data: dict | None = None):
    try:
        return _ok(detections=shared_data.get_detected_objects_raw())
    except Exception as exc:
        logger.error(f"get_detections: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_ball_calibration(sid: str, data: dict | None = None):
    try:
        ranges = shared_data.get_ball_calibration()
        ranges_list = [{"lower": list(lower), "upper": list(upper)} for lower, upper in ranges]
        return _ok(ranges=ranges_list)
    except Exception as exc:
        logger.error(f"get_ball_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_focal_length(sid: str, data: dict | None = None):
    try:
        return _ok(focal_length_pixels=shared_data.get_goal_focal_length())
    except Exception as exc:
        logger.error(f"get_goal_focal_length: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_all_state_machines(sid: str, data: dict | None = None):
    try:
        machines = autonomous_mode.get_available_state_machines()
        return _ok(state_machines=list(machines.keys()))
    except Exception as exc:
        logger.error(f"get_all_state_machines: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_autonomous_state(sid: str, data: dict | None = None):
    try:
        state_machine = autonomous_mode.get_current_state_machine_name()
        return _ok(
            state_machine=state_machine,
            always_face_goal_enabled=shared_data.get_always_facing_goal_enabled(),
            camera_ball_usage_enabled=shared_data.get_camera_ball_usage_enabled(),
        )
    except Exception as exc:
        logger.error(f"get_autonomous_state: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_line_calibration_status(sid: str, data: dict | None = None):
    try:
        return calibration.get_line_calibration_status()
    except Exception as exc:
        logger.error(f"get_line_calibration_status: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_distance_calibration_status(sid: str, data: dict | None = None):
    try:
        return calibration.get_goal_distance_calibration_status()
    except Exception as exc:
        logger.error(f"get_goal_distance_calibration_status: {exc}", exc_info=True)
        return _err("Internal server error")


# ---------------------------------------------------------------------------
# Mutations
# ---------------------------------------------------------------------------

@sio.event
async def set_mode(sid: str, data: dict | None = None):
    """data: { mode: "idle" | "manual" | "autonomous" }"""
    try:
        mode = (data or {}).get("mode")
        if mode not in ["idle", "manual", "autonomous"]:
            return _err("mode must be 'idle', 'manual', or 'autonomous'")
        shared_data.set_robot_mode(["idle", "manual", "autonomous"].index(mode))
        return _ok()
    except Exception as exc:
        logger.error(f"set_mode: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_manual_control(sid: str, data: dict | None = None):
    """data: { move: { angle: float, speed: float }, rotate: float }"""
    try:
        d = data or {}
        move = d.get("move")
        rotate = d.get("rotate")
        if (
            not isinstance(move, dict)
            or not isinstance(move.get("angle"), (int, float))
            or not isinstance(move.get("speed"), (int, float))
            or not isinstance(rotate, (int, float))
        ):
            return _err(f"Invalid request data: {d}")

        shared_data.set_manual_control(RobotManualControl(
            move_angle=move["angle"],
            move_speed=move["speed"],
            rotate=rotate,
        ))
        return _ok()
    except Exception as exc:
        logger.error(f"set_manual_control: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def reset_compass(sid: str, data: dict | None = None):
    try:
        shared_data.request_compass_reset()
        return _ok()
    except Exception as exc:
        logger.error(f"reset_compass: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_motor_settings(sid: str, data: dict | None = None):
    """data: { rotation_correction_enabled?: bool, line_avoiding_enabled?: bool, ... }"""
    try:
        d = data or {}
        bool_settings = {
            "rotation_correction_enabled":  shared_data.set_rotation_correction_enabled,
            "line_avoiding_enabled":        shared_data.set_line_avoiding_enabled,
            "position_based_speed_enabled": shared_data.set_position_based_speed_enabled,
        }
        for key, setter in bool_settings.items():
            if key in d:
                if not isinstance(d[key], bool):
                    return _err(f"{key} must be a boolean")
                setter(d[key])
                logger.info(f"{key} set to {d[key]}")
        return _ok()
    except Exception as exc:
        logger.error(f"set_motor_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_goal_settings(sid: str, data: dict | None = None):
    """data: { goal_color?: str, calibration?: { yellow?: {...}, blue?: {...} } }
    
    Each color's calibration can have either:
    - "ranges": [{"lower": [h,s,v], "upper": [h,s,v]}, ...] for multiple ranges
    - "lower": [h,s,v], "upper": [h,s,v] for backward compatibility (sets single range)
    """
    try:
        d = data or {}
        if "goal_color" in d:
            if d["goal_color"] not in ["yellow", "blue"]:
                return _err("goal_color must be 'yellow' or 'blue'")
            calibration.set_enemy_goal_color(d["goal_color"])

        cal = d.get("calibration", {})
        for color in ("yellow", "blue"):
            if color in cal:
                entry = cal[color]
                
                # Support new format with multiple ranges
                if "ranges" in entry and isinstance(entry["ranges"], list):
                    ranges = []
                    for r in entry["ranges"]:
                        lower = r.get("lower")
                        upper = r.get("upper")
                        if lower and len(lower) == 3 and upper and len(upper) == 3:
                            ranges.append((tuple(lower), tuple(upper)))
                    if ranges:
                        calibration.set_goal_color_ranges(color, ranges)
                # Support old format with single range
                elif "lower" in entry and "upper" in entry:
                    lower = entry.get("lower")
                    upper = entry.get("upper")
                    if lower and len(lower) == 3 and upper and len(upper) == 3:
                        calibration.set_goal_color_range(color, tuple(lower), tuple(upper))
        return _ok()
    except Exception as exc:
        logger.error(f"set_goal_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_ball_calibration(sid: str, data: dict | None = None):
    """data: { ranges?: [{"lower": [h,s,v], "upper": [h,s,v]}, ...], lower?: [h,s,v], upper?: [h,s,v] }
    
    Supports both:
    - New format: ranges array with multiple ranges
    - Old format: single lower/upper pair for backward compatibility
    """
    try:
        d = data or {}
        
        # Support new format with multiple ranges
        if "ranges" in d and isinstance(d["ranges"], list):
            ranges = []
            for r in d["ranges"]:
                lower = r.get("lower")
                upper = r.get("upper")
                if lower and len(lower) == 3 and upper and len(upper) == 3:
                    ranges.append((tuple(lower), tuple(upper)))
            if ranges:
                calibration.set_ball_color_ranges(ranges)
        # Support old format with single range
        elif "lower" in d and "upper" in d:
            lower = d.get("lower")
            upper = d.get("upper")
            if not lower or not upper or len(lower) != 3 or len(upper) != 3:
                return _err("'lower' and 'upper' must be lists of 3 values")
            calibration.set_ball_color_range(
                tuple(int(v) for v in lower),
                tuple(int(v) for v in upper),
            )
        else:
            return _err("Must provide either 'ranges' array or 'lower'/'upper' pair")
        return _ok()
    except Exception as exc:
        logger.error(f"set_ball_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_goal_focal_length(sid: str, data: dict | None = None):
    """data: { focal_length_pixels: float }"""
    try:
        d = data or {}
        fl = d.get("focal_length_pixels")
        if not isinstance(fl, (int, float)) or fl <= 0:
            return _err("focal_length_pixels must be a positive number")
        calibration.set_goal_focal_length(float(fl))
        return _ok(focal_length_pixels=fl)
    except Exception as exc:
        logger.error(f"set_goal_focal_length: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_autonomous_state(sid: str, data: dict | None = None):
    """data: { name: str }"""
    try:
        state_machine = (data or {}).get("state_machine")
        if isinstance(state_machine, str):
            sm = autonomous_mode.find_state_machine_by_name(state_machine)
            if sm is None:
                return _err(f"State machine '{state_machine}' not found")
            autonomous_mode.set_current_state_machine(sm)
            logger.info(f"Autonomous state machine set to '{sm.name}'")

        always_face_goal_enabled = (data or {}).get("always_face_goal_enabled")
        if isinstance(always_face_goal_enabled, bool):
            shared_data.set_always_facing_goal_enabled(always_face_goal_enabled)
            logger.info(f"always_facing_goal_enabled set to {always_face_goal_enabled}")

        camera_ball_usage_enabled = (data or {}).get("camera_ball_usage_enabled")
        if isinstance(camera_ball_usage_enabled, bool):
            shared_data.set_camera_ball_usage_enabled(camera_ball_usage_enabled)
            logger.info(f"camera_ball_usage_enabled set to {camera_ball_usage_enabled}")

        return _ok()
    except Exception as exc:
        logger.error(f"set_autonomous_state: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_line_thresholds(sid: str, data: dict | None = None):
    """data: { thresholds: [[min,max], ...] }  (LINE_SENSOR_COUNT pairs)"""
    try:
        d = data or {}
        thresholds = d.get("thresholds")
        if not isinstance(thresholds, list) or len(thresholds) != LINE_SENSOR_COUNT:
            return _err(f"thresholds must be a list of {LINE_SENSOR_COUNT} [min, max] pairs")
        ranges = []
        for t in thresholds:
            if isinstance(t, list) and len(t) == 2:
                ranges.append([int(t[0]), int(t[1])])
            else:
                return _err(f"thresholds must be a list of {LINE_SENSOR_COUNT} [min, max] pairs")
        calibration.set_line_detection_thresholds(ranges)
        return _ok(thresholds=ranges)
    except (ValueError, TypeError) as exc:
        return _err(f"Invalid threshold values: {exc}")
    except Exception as exc:
        logger.error(f"set_line_thresholds: {exc}", exc_info=True)
        return _err("Internal server error")


# ---------------------------------------------------------------------------
# Calibration procedures
# ---------------------------------------------------------------------------

@sio.event
async def camera_ball_distance_calibration(sid: str, data: dict | None = None):
    """data: { known_distance_mm: float }"""
    try:
        d = data or {}
        known = d.get("known_distance_mm")
        if known is None:
            return _err("Missing known_distance_mm")
        known = float(known)
        if known <= 0:
            return _err("known_distance_mm must be a positive number")
        constant = calibration.calibrate_ball_distance(known)
        return _ok(calibration_constant=constant)
    except Exception as exc:
        logger.error(f"camera_ball_distance_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def add_goal_color_range(sid: str, data: dict | None = None):
    """data: { goal_color: str, lower: [h,s,v], upper: [h,s,v] }"""
    try:
        d = data or {}
        goal_color = d.get("goal_color")
        lower = d.get("lower")
        upper = d.get("upper")
        
        if goal_color not in ["yellow", "blue"]:
            return _err("goal_color must be 'yellow' or 'blue'")
        if not lower or not upper or len(lower) != 3 or len(upper) != 3:
            return _err("'lower' and 'upper' must be lists of 3 values")
        
        calibration.add_goal_color_range(goal_color, tuple(lower), tuple(upper))
        ranges = calibration.get_goal_color_ranges(goal_color)
        return _ok(ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"add_goal_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def remove_goal_color_range(sid: str, data: dict | None = None):
    """data: { goal_color: str, index: int }"""
    try:
        d = data or {}
        goal_color = d.get("goal_color")
        index = d.get("index")
        
        if goal_color not in ["yellow", "blue"]:
            return _err("goal_color must be 'yellow' or 'blue'")
        if not isinstance(index, int) or index < 0:
            return _err("index must be a non-negative integer")
        
        if not calibration.remove_goal_color_range(goal_color, index):
            return _err(f"Invalid range index: {index}")
        
        ranges = calibration.get_goal_color_ranges(goal_color)
        return _ok(ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"remove_goal_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def add_ball_color_range(sid: str, data: dict | None = None):
    """data: { lower: [h,s,v], upper: [h,s,v] }"""
    try:
        d = data or {}
        lower = d.get("lower")
        upper = d.get("upper")
        
        if not lower or not upper or len(lower) != 3 or len(upper) != 3:
            return _err("'lower' and 'upper' must be lists of 3 values")
        
        calibration.add_ball_color_range(tuple(lower), tuple(upper))
        ranges = calibration.get_ball_color_ranges()
        return _ok(ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"add_ball_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def remove_ball_color_range(sid: str, data: dict | None = None):
    """data: { index: int }"""
    try:
        d = data or {}
        index = d.get("index")
        
        if not isinstance(index, int) or index < 0:
            return _err("index must be a non-negative integer")
        
        if not calibration.remove_ball_color_range(index):
            return _err(f"Invalid range index: {index}")
        
        ranges = calibration.get_ball_color_ranges()
        return _ok(ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"remove_ball_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def start_line_calibration(sid: str, data: dict | None = None):
    """data: { phase: 1 | 2 }"""
    try:
        phase = int((data or {}).get("phase", 1))
        if phase not in [1, 2]:
            return _err("phase must be 1 or 2")
        calibration.start_line_calibration(phase)
        return _ok(phase=phase, message=f"Phase {phase} started")
    except Exception as exc:
        logger.error(f"start_line_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def stop_line_calibration(sid: str, data: dict | None = None):
    try:
        status = calibration.get_line_calibration_status()
        if not status["phase"] > 0:
            return _err("Calibration is not active")
        thresholds, min_values, max_values, phase = calibration.stop_line_calibration()
        can_phase2 = phase == 1 and any(min_values[i] != float("inf") for i in range(len(min_values)))
        return _ok(
            phase=phase,
            thresholds=thresholds,
            min_values=min_values,
            max_values=max_values,
            can_start_phase2=can_phase2,
        )
    except Exception as exc:
        logger.error(f"stop_line_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def cancel_line_calibration(sid: str, data: dict | None = None):
    try:
        status = calibration.get_line_calibration_status()
        if not status["phase"] > 0:
            return _err("Calibration is not active")
        phase = status.get("phase", 0)
        calibration.stop_line_calibration(cancel=True)
        return _ok(phase=phase, message="Calibration cancelled.")
    except Exception as exc:
        logger.error(f"cancel_line_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def start_goal_distance_calibration(sid: str, data: dict | None = None):
    """data: { initial_distance?: float, line_distance?: float }"""
    try:
        d = data or {}
        init_dist = d.get("initial_distance", 200.0)
        line_dist = d.get("line_distance",    200.0)
        if not isinstance(init_dist, (int, float)) or init_dist <= 0:
            return _err("initial_distance must be a positive number")
        if not isinstance(line_dist, (int, float)) or line_dist <= 0:
            return _err("line_distance must be a positive number")
        calibration.start_goal_distance_calibration(float(init_dist), float(line_dist))
        return _ok(message="Drive the robot toward the enemy goal until it detects the line, then stop calibration")
    except Exception as exc:
        logger.error(f"start_goal_distance_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def stop_goal_distance_calibration(sid: str, data: dict | None = None):
    try:
        return calibration.stop_goal_distance_calibration()
    except Exception as exc:
        logger.error(f"stop_goal_distance_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def cancel_goal_distance_calibration(sid: str, data: dict | None = None):
    try:
        calibration.cancel_goal_distance_calibration()
        return _ok(message="Calibration cancelled")
    except Exception as exc:
        logger.error(f"cancel_goal_distance_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def compute_hsv_from_regions(sid: str, data: dict | None = None):
    """
    data: { regions: [{ x, y, width, height }, ...] }
    Returns the union HSV range across all regions.
    """
    try:
        d = data or {}
        regions = d.get("regions", [])
        if not regions:
            return _err("Missing 'regions'")

        h_min, s_min, v_min = 179, 255, 255
        h_max, s_max, v_max = 0,   0,   0

        for region_spec in regions:
            x      = int(region_spec.get("x",      0))
            y      = int(region_spec.get("y",      0))
            width  = int(region_spec.get("width",  0))
            height = int(region_spec.get("height", 0))

            if width <= 0 or height <= 0:
                return _err("Invalid region dimensions")

            frame_data = shared_data.get_camera_frame()
            if frame_data is None or frame_data.frame is None:
                return _err("No camera frame available")
            frame = frame_data.frame

            fh, fw = frame.shape[:2]
            x  = max(0, min(x,        fw - 1))
            y  = max(0, min(y,        fh - 1))
            x2 = max(0, min(x + width,  fw))
            y2 = max(0, min(y + height, fh))

            region_crop = frame[y:y2, x:x2]
            if region_crop.size == 0:
                return _err("Empty region")

            hsv = cv2.cvtColor(region_crop, cv2.COLOR_BGR2HSV)
            h_vals = hsv[:, :, 0].flatten()
            s_vals = hsv[:, :, 1].flatten()
            v_vals = hsv[:, :, 2].flatten()

            nh_min = max(0,   int(np.percentile(h_vals, 5))  - 5)
            nh_max = min(179, int(np.percentile(h_vals, 95)) + 5)
            ns_min = max(0,   int(np.percentile(s_vals, 5))  - 20)
            ns_max = min(255, int(np.percentile(s_vals, 95)) + 20)
            nv_min = max(0,   int(np.percentile(v_vals, 5))  - 20)
            nv_max = min(255, int(np.percentile(v_vals, 95)) + 20)

            h_min = min(h_min, nh_min); h_max = max(h_max, nh_max)
            s_min = min(s_min, ns_min); s_max = max(s_max, ns_max)
            v_min = min(v_min, nv_min); v_max = max(v_max, nv_max)

        return _ok(lower=[h_min, s_min, v_min], upper=[h_max, s_max, v_max])
    except Exception as exc:
        logger.error(f"compute_hsv_from_regions: {exc}", exc_info=True)
        return _err("Internal server error")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def start(
    host: str = API_HOST,
    port: int = API_PORT,
    stop_event: multiprocessing.synchronize.Event | None = None,
) -> None:
    config = uvicorn.Config(
        app=app,
        host=host,
        port=port,
        log_level="warning",
        access_log=False,
    )
    server = uvicorn.Server(config)
    logger.info(f"API server started on {host}:{port}")

    if stop_event:
        import threading

        def _watch():
            stop_event.wait()
            server.should_exit = True

        threading.Thread(target=_watch, daemon=True).start()

    server.run()


if __name__ == "__main__":
    start()
