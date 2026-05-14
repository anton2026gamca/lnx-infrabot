"""
Controller to Robot Forwarder

Requirements:
    pip install pygame python-socketio[client] aiohttp dotenv
"""

import asyncio
import logging
import math
import os
from dotenv import load_dotenv

import pygame
import socketio

load_dotenv()


# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------

CONTROLLER_ROBOT_MAP: dict[int, str] = {
    0: "http://192.168.0.161:5000",
    1: "http://192.168.0.144:5000",
}

AUTH_TOKEN: str = os.getenv("AUTH_TOKEN", "") 

AUTO_SET_MANUAL_MODE: bool = True

STICK_DEADZONE: float = 0.10

SEND_INTERVAL: float = 0.05

FORWARD_BACK_AXIS: int = 1  # Left stick vertical
STRAFE_AXIS: int = 0        # Left stick horizontal
ROTATE_AXIS: int = 2        # Right stick horizontal

TOGGLE_BUTTON: int | None = None

BOOST_AXIS: int = 4         # Right trigger

ROBOT_SPEED_MULTIPLIER: float = 0.7
ROBOT_BOOST_MULTIPLIER: float = 1.2

# ---------------------------------------------------------------------------
# END OF CONFIG
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("forwarder")


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def axes_to_move_rotate(
    left_x: float,
    left_y: float,
    right_x: float,
) -> tuple[float, float, float]:
    vx =  left_x
    vy = -left_y

    speed = min(math.hypot(vx, vy), 1.0)
    if speed > 0.0:
        angle_rad = math.atan2(vx, vy)
        angle_deg = math.degrees(angle_rad)
    else:
        angle_deg = 0.0

    rotate = right_x

    return angle_deg, speed, rotate


class RobotConnection:
    def __init__(self, url: str, token: str, controller_index: int):
        self.url = url
        self.token = token
        self.controller_index = controller_index
        self.name = f"robot@{url} (ctrl={controller_index})"

        auth_b64 = __import__("base64").b64encode(token.encode()).decode() if token else ""
        query = f"token={auth_b64}" if auth_b64 else ""
        self._connect_url = f"{url}?{query}" if query else url

        self.sio = socketio.AsyncClient(reconnection=True, reconnection_attempts=0, logger=False)
        self._connected = False
        self._sending = True

        @self.sio.event
        async def connect():
            self._connected = True
            log.info(f"[{self.name}] Connected")
            if AUTO_SET_MANUAL_MODE:
                await self.sio.call("set_mode", {"mode": "manual"}, timeout=5)
                await self.sio.call("set_motor_settings", {"line_avoiding_enabled": False})
                log.info(f"[{self.name}] Mode set to manual and disabled line avoiding")

        @self.sio.event
        async def disconnect():
            self._connected = False
            log.warning(f"[{self.name}] Disconnected")
        async def connect_error(data):
            log.error(f"[{self.name}] Connection error: {data}")

    async def start(self):
        log.info(f"[{self.name}] Connecting to {self.url} ...")
        try:
            await self.sio.connect(self._connect_url, transports=["websocket"])
        except Exception as exc:
            log.error(f"[{self.name}] Initial connect failed: {exc}. Will keep retrying.")

    async def stop(self):
        if self._connected:
            await self.send_command(0.0, 0.0, 0.0)
        await self.sio.disconnect()

    async def send_command(self, angle_deg: float, speed: float, rotate: float):
        if not self._connected:
            return
        try:
            await self.sio.emit("set_manual_control", {
                "move": {"angle": angle_deg, "speed": speed},
                "rotate": rotate,
            })
        except Exception as exc:
            log.debug(f"[{self.name}] Emit error: {exc}")

    def toggle_sending(self):
        self._sending = not self._sending
        state = "ENABLED" if self._sending else "PAUSED"
        log.info(f"[{self.name}] Sending {state}")

    @property
    def is_sending(self) -> bool:
        return self._sending


async def forwarder_loop(
    robots: dict[int, RobotConnection],
    stop_event: asyncio.Event,
):
    toggle_was_pressed: dict[int, bool] = {idx: False for idx in robots}

    log.info("Forwarder running. Press Ctrl+C to stop.")

    while not stop_event.is_set():
        pygame.event.pump()

        for ctrl_idx, robot in robots.items():
            joystick = None
            for i in range(pygame.joystick.get_count()):
                js = pygame.joystick.Joystick(i)
                if i == ctrl_idx:
                    joystick = js
                    break

            if joystick is None:
                continue

            if TOGGLE_BUTTON is not None:
                try:
                    pressed = bool(joystick.get_button(TOGGLE_BUTTON))
                except Exception:
                    pressed = False
                if pressed and not toggle_was_pressed[ctrl_idx]:
                    robot.toggle_sending()
                toggle_was_pressed[ctrl_idx] = pressed

            if not robot.is_sending:
                continue

            def get_axis(idx: int) -> float:
                if joystick is None:
                    return 0.0
                try:
                    return joystick.get_axis(idx)
                except Exception as e:
                    log.error(f"{e}")
                    return 0.0

            left_x  = apply_deadzone(get_axis(STRAFE_AXIS),  STICK_DEADZONE)
            left_y  = apply_deadzone(get_axis(FORWARD_BACK_AXIS),  STICK_DEADZONE)
            right_x = apply_deadzone(get_axis(ROTATE_AXIS), STICK_DEADZONE)
            boost   = get_axis(BOOST_AXIS) > 0.5

            angle_deg, speed, rotate = axes_to_move_rotate(left_x, left_y, right_x)
            speed *= ROBOT_BOOST_MULTIPLIER if boost else ROBOT_SPEED_MULTIPLIER
            await robot.send_command(angle_deg, speed, rotate)

        await asyncio.sleep(SEND_INTERVAL)


async def main():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    log.info(f"Detected {joystick_count} joystick(s)")
    joysticks: dict[int, pygame.joystick.JoystickType] = {}
    for i in range(joystick_count):
        js = pygame.joystick.Joystick(i)
        js.init()
        joysticks[i] = js
        log.info(f"  Controller {i}: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")

    for ctrl_idx in CONTROLLER_ROBOT_MAP:
        if ctrl_idx not in joysticks:
            log.warning(
                f"Controller {ctrl_idx} is mapped but NOT detected. "
                "Commands for that robot will be skipped until it is plugged in."
            )

    robots: dict[int, RobotConnection] = {}
    for ctrl_idx, url in CONTROLLER_ROBOT_MAP.items():
        robots[ctrl_idx] = RobotConnection(url, AUTH_TOKEN, ctrl_idx)

    await asyncio.gather(*(r.start() for r in robots.values()))

    stop_event = asyncio.Event()

    try:
        await forwarder_loop(robots, stop_event)
    except (KeyboardInterrupt, asyncio.CancelledError):
        log.info("Shutting down …")
    finally:
        stop_event.set()
        await asyncio.gather(*(r.stop() for r in robots.values()))
        pygame.quit()
        log.info("Done.")


if __name__ == "__main__":
    asyncio.run(main())
