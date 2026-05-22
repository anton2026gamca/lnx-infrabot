import asyncio
import cv2
import multiprocessing.synchronize
import numpy as np
import threading
import uvicorn
import urllib.parse
from engineio.packet import base64

import socketio
from fastapi import FastAPI

from robot import calibration, utils, vision
from robot.profiling import async_sleep
import robot.bluetooth.utils as bluetooth_utils
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
    global _state_tracker
    check_interval = 0.1  # Check for changes every 100ms
    
    try:
        while True:
            await async_sleep(check_interval)
            
            try:
                # Get current state
                current_mode = shared_data.get_robot_mode()
                current_goal_color = shared_data.get_goal_color()
                current_line_detected = line_sensors.get_line_detected()
                current_hw_data = shared_data.get_hardware_compass_ir()
                current_ir_angle = current_hw_data[1]
                current_cam_data = shared_data.get_camera_ball_data()
                
                current_ir_detected = current_ir_angle != 999
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
                camera=det.get("camera"),
            ))
        else:
            result.append(det)
    return result


def _filter_raw_detections_by_camera(detections: list[dict], camera: str) -> list[dict]:
    if camera == "both":
        return detections
    return [det for det in detections if det.get("camera") == camera or det.get("camera") is None]


# ---------------------------------------------------------------------------
# Video streaming
# ---------------------------------------------------------------------------

async def _video_loop(sid: str, fps: float, show_detections: bool, camera: str) -> None:
    sleep_time = 1.0 / fps if fps > 0 else 0
    try:
        while True:
            if sleep_time:
                await async_sleep(sleep_time)

            emit_front = camera in ("front", "both")
            emit_back = camera in ("back", "both")
            detections_raw = shared_data.get_detected_objects_raw() if show_detections else []

            if emit_front:
                front_frame_data = shared_data.get_camera_frame("front")
                if front_frame_data is not None and front_frame_data.frame is not None:
                    front_frame = front_frame_data.frame
                    if show_detections:
                        try:
                            front_detections = _filter_raw_detections_by_camera(detections_raw, "front")
                            objects = _build_detected_objects(front_detections)
                            if objects:
                                front_frame = vision.draw_detections_on_frame(front_frame, objects, draw_labels=True)
                        except Exception as exc:
                            logger.debug(f"Error drawing detections: {exc}")

                    ret_front, front_buffer = cv2.imencode(".jpg", front_frame, encode_params)
                    if ret_front:
                        await sio.emit("video_frame_front", front_buffer.tobytes(), to=sid)

            if emit_back:
                back_frame_data = shared_data.get_camera_frame("back")
                if back_frame_data is not None and back_frame_data.frame is not None:
                    back_frame = back_frame_data.frame
                    if show_detections:
                        try:
                            back_detections = _filter_raw_detections_by_camera(detections_raw, "back")
                            objects = _build_detected_objects(back_detections)
                            if objects:
                                back_frame = vision.draw_detections_on_frame(back_frame, objects, draw_labels=True)
                        except Exception as exc:
                            logger.debug(f"Error drawing detections: {exc}")
                    ret_back, back_buffer = cv2.imencode(".jpg", back_frame, encode_params)
                    if ret_back:
                        await sio.emit("video_frame_back", back_buffer.tobytes(), to=sid)
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
    return _ok(message=f"Subscribed to {len(subscribed)} update(s)")


@sio.event
async def unsubscribe_updates(sid: str, data: dict | None = None):
    if sid in _update_subscriptions:
        _update_subscriptions.pop(sid)
    return _ok()


# ---------------------------------------------------------------------------
# Video subscription
# ---------------------------------------------------------------------------

@sio.event
async def subscribe_video(sid: str, data: dict | None = None):
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
    camera = str(data.get("camera", "both")).lower()
    if camera not in ("front", "back", "both"):
        return _err("camera must be one of: front, back, both")

    task = asyncio.create_task(_video_loop(sid, fps, show_detections, camera))
    _video_tasks[sid] = task
    return _ok()


@sio.event
async def unsubscribe_video(sid: str, data: dict | None = None):
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
            "bt_module_state": running_state.bt_module_value if running_state else False,
            "switch_state": running_state.main_switch_value if running_state else False,
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
        
        return _ok(
            goal_color=goal_color,
        )
    except Exception as exc:
        logger.error(f"get_goal_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_color_calibration(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "front")).lower()
        if camera not in ("front", "back"):
            return _err("camera must be one of: front, back")
        y_ranges = shared_data.get_goal_calibration("yellow", camera)
        b_ranges = shared_data.get_goal_calibration("blue", camera)
        
        y_ranges_list = [{"lower": list(lower), "upper": list(upper)} for lower, upper in y_ranges]
        b_ranges_list = [{"lower": list(lower), "upper": list(upper)} for lower, upper in b_ranges]
        
        return _ok(
            camera=camera,
            yellow_ranges=y_ranges_list,
            blue_ranges=b_ranges_list
        )

    except Exception as exc:
        logger.error(f"get_goal_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_detection(sid: str, data: dict | None = None):
    try:
        result = shared_data.get_goal_detection_result()
        enemy_color = shared_data.get_goal_color().lower()
        own_color = "blue" if enemy_color == "yellow" else "yellow"
        yellow_result = shared_data.get_goal_detection_result_for_color("yellow")
        blue_result = shared_data.get_goal_detection_result_for_color("blue")

        def _result_to_dict(goal_result):
            if goal_result is None:
                return {
                    "goal_detected": False,
                    "alignment": 0.0,
                    "goal_center_x": None,
                    "goal_area": 0.0,
                    "distance_mm": None,
                    "goal_height_pixels": 0.0,
                    "camera_yaw_deg": 0.0,
                }
            return {
                "goal_detected": goal_result.detected,
                "alignment": goal_result.alignment,
                "goal_center_x": goal_result.center_x,
                "goal_area": goal_result.area,
                "distance_mm": goal_result.distance_mm,
                "goal_height_pixels": goal_result.height_pixels,
                "camera_yaw_deg": goal_result.camera_yaw_deg,
            }

        if result is None:
            return _ok(
                goal_detected=False,
                alignment=0.0,
                goal_center_x=None,
                goal_area=0.0,
                goals_by_color={
                    "yellow": _result_to_dict(yellow_result),
                    "blue": _result_to_dict(blue_result),
                },
                enemy_goal_color=enemy_color,
                own_goal_color=own_color,
            )
        return _ok(
            goal_detected=result.detected,
            alignment=result.alignment,
            goal_center_x=result.center_x,
            goal_area=result.area,
            distance_mm=result.distance_mm,
            goal_height_pixels=result.height_pixels,
            camera_yaw_deg=result.camera_yaw_deg,
            goals_by_color={
                "yellow": _result_to_dict(yellow_result),
                "blue": _result_to_dict(blue_result),
            },
            enemy_goal_color=enemy_color,
            own_goal_color=own_color,
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
        d = data or {}
        camera = str(d.get("camera", "both")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")

        detections = shared_data.get_detected_objects_raw()
        for d in detections:
            if "color" not in d or d["color"] is None or d["color"] == (-1, -1, -1):
                conf = vision.get_visualizer().get_object_type_config(d.get("object_type", ""))
                if conf is not None:
                    d["color"] = conf.color

        return _ok(
            camera=camera,
            detections={
                "front": _filter_raw_detections_by_camera(detections, "front") if camera in ["both", "front"] else None,
                "back": _filter_raw_detections_by_camera(detections, "back") if camera in ["both", "back"] else None,
            },
        )
    except Exception as exc:
        logger.error(f"get_detections: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_ball_calibration(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "front")).lower()
        if camera not in ("front", "back"):
            return _err("camera must be one of: front, back")
        ranges = shared_data.get_ball_calibration(camera)
        ranges_list = [{"lower": list(lower), "upper": list(upper)} for lower, upper in ranges]
        return _ok(camera=camera, ranges=ranges_list)
    except Exception as exc:
        logger.error(f"get_ball_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_goal_focal_length(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "front")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back")
        return _ok(camera=camera, focal_length_pixels=shared_data.get_goal_focal_length(camera))
    except Exception as exc:
        logger.error(f"get_goal_focal_length: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_camera_settings(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "both")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        if camera == "both":
            settings = {
                "front": shared_data.get_camera_settings("front"),
                "back": shared_data.get_camera_settings("back"),
            }
        else:
            settings = shared_data.get_camera_settings(camera)
        return _ok(camera=camera, settings=settings)
    except Exception as exc:
        logger.error(f"get_camera_settings: {exc}", exc_info=True)
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
    try:
        d = data or {}
        if "goal_color" in d:
            if d["goal_color"] not in ["yellow", "blue"]:
                return _err("goal_color must be 'yellow' or 'blue'")
            calibration.set_enemy_goal_color(d["goal_color"])
        return _ok()
    except Exception as exc:
        logger.error(f"set_goal_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_goal_color_calibration(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "both")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")

        for color in ("yellow", "blue"):
            if f"{color}_ranges" in d:
                entry = d[f"{color}_ranges"]
                
                if isinstance(entry, list):
                    ranges = []
                    for r in entry:
                        lower = r.get("lower")
                        upper = r.get("upper")
                        if lower and len(lower) == 3 and upper and len(upper) == 3:
                            ranges.append((tuple(lower), tuple(upper)))
                    if ranges:
                        calibration.set_goal_color_ranges(color, ranges, camera=camera)
                else:
                    return _err(f"'{color}_ranges' must be an array")
        return _ok(camera=camera)
    except Exception as exc:
        logger.error(f"set_goal_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_ball_calibration(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "both")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        
        if "ranges" in d and isinstance(d["ranges"], list):
            ranges = []
            for r in d["ranges"]:
                lower = r.get("lower")
                upper = r.get("upper")
                if lower and len(lower) == 3 and upper and len(upper) == 3:
                    ranges.append((tuple(lower), tuple(upper)))
            if ranges:
                calibration.set_ball_color_ranges(ranges, camera=camera)
        else:
            return _err("Must provide 'ranges' array")
        return _ok(camera=camera)
    except Exception as exc:
        logger.error(f"set_ball_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_goal_focal_length(sid: str, data: dict | None = None):
    try:
        d = data or {}
        fl = d.get("focal_length_pixels")
        camera = str(d.get("camera", "both")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        if isinstance(fl, str):
            try:
                fl = float(fl)
            except ValueError:
                return _err("focal_length_pixels must be a positive number")
        if not isinstance(fl, (int, float)) or fl <= 0:
            return _err("focal_length_pixels must be a positive number")
        calibration.set_goal_focal_length(float(fl), camera=camera)
        return _ok(focal_length_pixels=fl, camera=camera)
    except Exception as exc:
        logger.error(f"set_goal_focal_length: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_camera_settings(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera = str(d.get("camera", "both")).lower()
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")

        color_gains = d.get("color_gains")
        if color_gains is not None:
            if not isinstance(color_gains, list) or len(color_gains) != 2:
                return _err("color_gains must be a list of exactly 2 numbers")
            if not all(isinstance(v, (int, float)) and float(v) > 0 for v in color_gains):
                return _err("color_gains values must be positive numbers")

        exposure_time = d.get("exposure_time")
        if exposure_time is not None:
            if isinstance(exposure_time, str):
                try:
                    exposure_time = float(exposure_time)
                except ValueError:
                    return _err("exposure_time must be a positive number")
            if not isinstance(exposure_time, (int, float)) or float(exposure_time) <= 0:
                return _err("exposure_time must be a positive number")

        analogue_gain = d.get("analogue_gain")
        if analogue_gain is not None:
            if isinstance(analogue_gain, str):
                try:
                    analogue_gain = float(analogue_gain)
                except ValueError:
                    return _err("analogue_gain must be a positive number")
            if not isinstance(analogue_gain, (int, float)) or float(analogue_gain) <= 0:
                return _err("analogue_gain must be a positive number")

        if color_gains is None and exposure_time is None and analogue_gain is None:
            return _err("Provide at least one setting: color_gains, exposure_time, or analogue_gain")

        request_id = shared_data.request_camera_settings_update(
            camera=camera,
            color_gains=[float(color_gains[0]), float(color_gains[1])] if color_gains is not None else None,
            exposure_time=float(exposure_time) if exposure_time is not None else None,
            analogue_gain=float(analogue_gain) if analogue_gain is not None else None,
        )
        if request_id is None:
            return _err("A camera settings update is already in progress")

        deadline = asyncio.get_running_loop().time() + 5.0
        while asyncio.get_running_loop().time() < deadline:
            result = shared_data.get_camera_settings_update_result(request_id)
            if result and result.get("done", False):
                if result.get("success", False):
                    return _ok(camera=camera, settings=result.get("settings", {}))
                return _err(result.get("error") or "Camera settings update failed")
            await async_sleep(0.05)

        return _err("Camera settings update timed out")
    except Exception as exc:
        logger.error(f"set_camera_settings: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_autonomous_state(sid: str, data: dict | None = None):
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
    try:
        d = data or {}
        known = d.get("known_distance_mm")
        camera = str(d.get("camera", "front")).lower()
        if camera not in ("front", "back"):
            return _err("camera must be one of: front, back")
        if known is None:
            return _err("Missing known_distance_mm")
        known = float(known)
        if known <= 0:
            return _err("known_distance_mm must be a positive number")
        constant = calibration.calibrate_ball_distance(known, camera=camera)
        return _ok(camera=camera, calibration_constant=constant)
    except Exception as exc:
        logger.error(f"camera_ball_distance_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def camera_auto_calibration(sid: str, data: dict | None = None):
    try:
        d = data or {}
        camera_name = str(d.get("camera", "front")).lower()
        if camera_name not in ("front", "back"):
            return _err("camera must be one of: front, back")

        settle_time_s = d.get("settle_time_s", 2.0)
        if isinstance(settle_time_s, str):
            try:
                settle_time_s = float(settle_time_s)
            except ValueError:
                return _err("settle_time_s must be a positive number")
        if not isinstance(settle_time_s, (int, float)) or settle_time_s <= 0:
            return _err("settle_time_s must be a positive number")

        request_id = shared_data.request_camera_auto_calibration(
            camera=camera_name,
            settle_time_s=float(settle_time_s),
        )
        if request_id is None:
            return _err("A camera auto calibration is already in progress")

        timeout_s = float(settle_time_s) + 5.0
        deadline = asyncio.get_running_loop().time() + timeout_s
        while asyncio.get_running_loop().time() < deadline:
            result = shared_data.get_camera_auto_calibration_result(request_id)
            if result and result.get("done", False):
                if result.get("success", False):
                    payload = result.get("result", {})
                    return _ok(camera=payload.get("camera", camera_name), result=payload.get("result", {}))
                return _err(result.get("error") or "Camera auto calibration failed")
            await async_sleep(0.05)

        return _err("Camera auto calibration timed out")
    except Exception as exc:
        logger.error(f"camera_auto_calibration: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def add_goal_color_range(sid: str, data: dict | None = None):
    try:
        d = data or {}
        goal_color = d.get("goal_color")
        lower = d.get("lower")
        upper = d.get("upper")
        camera = str(d.get("camera", "both")).lower()
        
        if goal_color not in ["yellow", "blue"]:
            return _err("goal_color must be 'yellow' or 'blue'")
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        if not lower or not upper or len(lower) != 3 or len(upper) != 3:
            return _err("'lower' and 'upper' must be lists of 3 values")
        
        calibration.add_goal_color_range(goal_color, tuple(lower), tuple(upper), camera=camera)
        ranges = calibration.get_goal_color_ranges(goal_color, "front" if camera == "both" else camera)
        return _ok(camera=camera, ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"add_goal_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def remove_goal_color_range(sid: str, data: dict | None = None):
    try:
        d = data or {}
        goal_color = d.get("goal_color")
        index = d.get("index")
        camera = str(d.get("camera", "both")).lower()
        
        if goal_color not in ["yellow", "blue"]:
            return _err("goal_color must be 'yellow' or 'blue'")
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        if not isinstance(index, int) or index < 0:
            return _err("index must be a non-negative integer")
        
        if not calibration.remove_goal_color_range(goal_color, index, camera=camera):
            return _err(f"Invalid range index: {index}")
        
        ranges = calibration.get_goal_color_ranges(goal_color, "front" if camera == "both" else camera)
        return _ok(camera=camera, ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"remove_goal_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def add_ball_color_range(sid: str, data: dict | None = None):
    try:
        d = data or {}
        lower = d.get("lower")
        upper = d.get("upper")
        camera = str(d.get("camera", "both")).lower()
        
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        if not lower or not upper or len(lower) != 3 or len(upper) != 3:
            return _err("'lower' and 'upper' must be lists of 3 values")
        
        calibration.add_ball_color_range(tuple(lower), tuple(upper), camera=camera)
        ranges = calibration.get_ball_color_ranges("front" if camera == "both" else camera)
        return _ok(camera=camera, ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"add_ball_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def remove_ball_color_range(sid: str, data: dict | None = None):
    try:
        d = data or {}
        index = d.get("index")
        camera = str(d.get("camera", "both")).lower()
        
        if camera not in ("front", "back", "both"):
            return _err("camera must be one of: front, back, both")
        if not isinstance(index, int) or index < 0:
            return _err("index must be a non-negative integer")
        
        if not calibration.remove_ball_color_range(index, camera=camera):
            return _err(f"Invalid range index: {index}")
        
        ranges = calibration.get_ball_color_ranges("front" if camera == "both" else camera)
        return _ok(camera=camera, ranges=[{"lower": list(l), "upper": list(u)} for l, u in ranges])
    except Exception as exc:
        logger.error(f"remove_ball_color_range: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def start_line_calibration(sid: str, data: dict | None = None):
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
    try:
        d = data or {}
        init_dist = d.get("initial_distance", 200.0)
        line_dist = d.get("line_distance",    200.0)
        camera = str(d.get("camera", "front")).lower()
        if not isinstance(init_dist, (int, float)) or init_dist <= 0:
            return _err("initial_distance must be a positive number")
        if not isinstance(line_dist, (int, float)) or line_dist <= 0:
            return _err("line_distance must be a positive number")
        if camera not in ("front", "back"):
            return _err("camera must be one of: front, back")
        calibration.start_goal_distance_calibration(float(init_dist), float(line_dist), camera=camera)
        return _ok(camera=camera, message="Drive the robot toward the enemy goal until it detects the line, then stop calibration")
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
    try:
        d = data or {}
        camera = str(d.get("camera", "front")).lower()
        if camera not in ("front", "back"):
            return _err("camera must be one of: front, back")
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

            frame_data = shared_data.get_camera_frame(camera)
            if frame_data is None or frame_data.frame is None:
                return _err(f"No {camera} camera frame available")
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
# Bluetooth
# ---------------------------------------------------------------------------

@sio.event
async def get_bluetooth_state(sid: str, data: dict | None = None):
    try:
        return _ok(
            process_alive=bluetooth_utils.is_bluetooth_process_alive(),
            local_device=bluetooth_utils.get_local_device_info(),
            connected_devices=bluetooth_utils.get_connected_devices(),
            paired_devices=bluetooth_utils.get_paired_devices(),
            other_robot=bluetooth_utils.get_other_robot_info(),
        )
    except Exception as exc:
        logger.error(f"get_bluetooth_state: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_other_robot(sid: str, data: dict | None = None):
    try:
        d = data or {}
        if d.get("clear") is True:
            bluetooth_utils.clear_other_robot_info()
            return _ok(other_robot={})

        mac_address = d.get("mac_address")
        if not isinstance(mac_address, str) or not mac_address.strip():
            return _err("mac_address is required")

        info = {
            "mac_address": mac_address.strip(),
            "name": d.get("name"),
            "hostname": d.get("hostname"),
            "ip_address": d.get("ip_address"),
            "note": d.get("note"),
        }
        info = {k: v for k, v in info.items() if v is not None}

        bluetooth_utils.set_other_robot_info(info)
        return _ok(other_robot=bluetooth_utils.get_other_robot_info())
    except Exception as exc:
        logger.error(f"set_other_robot: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def bluetooth_connect_other_robot(sid: str, data: dict | None = None):
    try:
        d = data or {}
        mac_address = d.get("mac_address") or bluetooth_utils.get_other_robot_info().get("mac_address")
        if not isinstance(mac_address, str) or not mac_address.strip():
            return _err("mac_address is required (or set other_robot first)")

        result = bluetooth_utils.connect(mac_address.strip())
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to connect")

        return _ok(result=result, connected_devices=bluetooth_utils.get_connected_devices())
    except Exception as exc:
        logger.error(f"bluetooth_connect_other_robot: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def bluetooth_disconnect_other_robot(sid: str, data: dict | None = None):
    try:
        d = data or {}
        mac_address = d.get("mac_address") or bluetooth_utils.get_other_robot_info().get("mac_address")
        if not isinstance(mac_address, str) or not mac_address.strip():
            return _err("mac_address is required (or set other_robot first)")

        result = bluetooth_utils.disconnect(mac_address.strip())
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to disconnect")

        return _ok(result=result, connected_devices=bluetooth_utils.get_connected_devices())
    except Exception as exc:
        logger.error(f"bluetooth_disconnect_other_robot: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def bluetooth_send_message(sid: str, data: dict | None = None):
    try:
        d = data or {}
        mac_address = d.get("mac_address") or bluetooth_utils.get_other_robot_info().get("mac_address")
        message_type = d.get("message_type")
        content = d.get("content")

        if not isinstance(mac_address, str) or not mac_address.strip():
            return _err("mac_address is required (or set other_robot first)")
        if not isinstance(message_type, str) or not message_type.strip():
            return _err("message_type is required")
        if content is None:
            return _err("content is required")

        result = bluetooth_utils.send_message(
            mac_address=mac_address.strip(),
            message_type=message_type.strip(),
            content=str(content),
        )
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to send message")

        return _ok(result=result)
    except Exception as exc:
        logger.error(f"bluetooth_send_message: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def get_bluetooth_messages(sid: str, data: dict | None = None):
    try:
        d = data or {}
        clear = bool(d.get("clear", False))
        limit_raw = d.get("limit")
        limit = limit_raw if isinstance(limit_raw, int) and limit_raw > 0 else None

        return _ok(
            received=bluetooth_utils.get_received_messages(clear=clear, limit=limit),
            sent=bluetooth_utils.get_sent_messages(clear=clear, limit=limit),
        )
    except Exception as exc:
        logger.error(f"get_bluetooth_messages: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def bluetooth_list_pairable_devices(sid: str, data: dict | None = None):
    try:
        d = data or {}
        timeout_raw = d.get("timeout_seconds", 6)
        if not isinstance(timeout_raw, int) or timeout_raw <= 0:
            return _err("timeout_seconds must be a positive integer")

        result = bluetooth_utils.list_pairable_devices(timeout_seconds=timeout_raw)
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to list pairable devices")

        devices = result.get("data", {}).get("devices", [])
        return _ok(result=result, devices=devices)
    except Exception as exc:
        logger.error(f"bluetooth_list_pairable_devices: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def bluetooth_pair_device(sid: str, data: dict | None = None):
    try:
        d = data or {}
        mac_address = d.get("mac_address")
        
        if not isinstance(mac_address, str) or not mac_address.strip():
            return _err("mac_address is required")
        
        result = bluetooth_utils.pair_device(
            mac_address=mac_address.strip(),
        )
        
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to pair device")
        
        return _ok(result=result, paired_devices=bluetooth_utils.get_paired_devices())
    except Exception as exc:
        logger.error(f"bluetooth_pair_device: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def bluetooth_unpair_device(sid: str, data: dict | None = None):
    try:
        d = data or {}
        mac_address = d.get("mac_address")
        
        if not isinstance(mac_address, str) or not mac_address.strip():
            return _err("mac_address is required")
        
        result = bluetooth_utils.unpair_device(mac_address.strip())
        
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to unpair device")
        
        return _ok(result=result, paired_devices=bluetooth_utils.get_paired_devices())
    except Exception as exc:
        logger.error(f"bluetooth_unpair_device: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def set_bluetooth_pairing_mode(sid: str, data: dict | None = None):
    try:
        d = data or {}
        enabled_raw = d.get("enabled")
        
        if not isinstance(enabled_raw, bool):
            return _err("enabled must be a boolean (true/false)")
        
        result = bluetooth_utils.set_pairing_mode(enabled=enabled_raw)
        
        if not result.get("success", False):
            return _err(result.get("error") or "Failed to set pairing mode")
        
        return _ok(result=result, pairing_mode_enabled=enabled_raw)
    except Exception as exc:
        logger.error(f"set_bluetooth_pairing_mode: {exc}", exc_info=True)
        return _err("Internal server error")


# ---------------------------------------------------------------------------
# Profiling Control
# ---------------------------------------------------------------------------

@sio.event
async def profiling_start(sid: str, data: dict | None = None):
    try:
        from robot import profiling
        
        collector = profiling.get_collector()
        if collector:
            collector.start_collection()
            return _ok(message="Profiling collection started")
        else:
            return _err("Profiling system not available")
    except Exception as exc:
        logger.error(f"profiling_start: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def profiling_stop(sid: str, data: dict | None = None):
    try:
        from robot import profiling
        
        collector = profiling.get_collector()
        if collector:
            collector.stop_collection()
            return _ok(message="Profiling collection stopped")
        else:
            return _err("Profiling system not available")
    except Exception as exc:
        logger.error(f"profiling_stop: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def profiling_status(sid: str, data: dict | None = None):
    try:
        from robot import profiling
        
        collector = profiling.get_collector()
        if collector:
            status = collector.get_status()
            return _ok(**status)
        else:
            return _err("Profiling system not available")
    except Exception as exc:
        logger.error(f"profiling_status: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def profiling_report(sid: str, data: dict | None = None):
    try:
        from robot import profiling

        d = data or {}
        collector = profiling.get_collector()
        if collector:
            report = collector.get_report(force_refresh=bool(d.get("force_refresh", False)))
            if d.get("consume", True):
                collector.clear_collection()
            return _ok(report=report)
        else:
            return _err("Profiling system not available")
    except Exception as exc:
        logger.error(f"profiling_report: {exc}", exc_info=True)
        return _err("Internal server error")


@sio.event
async def profiling_clear(sid: str, data: dict | None = None):
    try:
        from robot import profiling
        
        collector = profiling.get_collector()
        if collector:
            collector.clear_collection()
            return _ok(message="Profiling data cleared")
        else:
            return _err("Profiling system not available")
    except Exception as exc:
        logger.error(f"profiling_clear: {exc}", exc_info=True)
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

    async def serve():
        if stop_event is not None:
            def monitor():
                if stop_event is None:
                    return
                stop_event.wait()
                server.should_exit = True
            
            monitor_thread = threading.Thread(target=monitor, daemon=True)
            monitor_thread.start()
        
        await server.serve()
    
    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        server.should_exit = True


if __name__ == "__main__":
    start()
