from __future__ import annotations

import logging
import math
import multiprocessing.synchronize
from operator import contains
import os
import socket
import time
from dataclasses import dataclass
from typing import Callable, Sequence

from gpiozero import Button
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

import robot.bluetooth.utils as bluetooth_utils
from robot.config import CAMERA_FOV_DEG, LINE_SENSOR_COUNT, LINE_SENSOR_MAX_VALUE, LINE_SENSOR_MIN_VALUE
from robot.display.rendering import draw_mono_text
from robot.logic import autonomous_mode
from robot.multiprocessing import shared_data
from robot.profiling import sleep
from robot.robot import RobotMode
from robot.vision import GoalDetectionResult

DISPLAY_WIDTH = 128
DISPLAY_HEIGHT = 32

MAX_LINES = 3
LINE_HEIGHT = DISPLAY_HEIGHT // MAX_LINES
LINE_MARGIN = max(0, (DISPLAY_HEIGHT - (LINE_HEIGHT * MAX_LINES)) // 2)
TEXT_SPACING = 1

MONO_CHAR_WIDTH = 5

SCROLL_ARROW_WIDTH = 5
SCROLL_ARROW_HEIGHT = 3
SCROLL_ARROW_MARGIN_X = 1
SCROLL_ARROW_MARGIN_Y = 1

BUTTON_UP_PIN = 22
BUTTON_DOWN_PIN = 23
BUTTON_SELECT_PIN = 24
BUTTON_BACK_PIN = 25

REPEAT_INITIAL_DELAY_S = 0.45
REPEAT_INTERVAL_S = 0.12

SCREENSAVER_IDLE_S = 10.0
LOCK_ICON_WIDTH = 7
LOCK_ICON_HEIGHT = 7
LOCK_ICON_MARGIN_X = 1
LOCK_ICON_MARGIN_Y = 1

IP_CACHE_TTL_S = 5.0
IP_FALLBACK = "---"

MODE_DISPLAY_TITLE = "LNX InfraBot"

ROLE_LABELS = {
    "Attacker State Machine": "Attacker",
    "Goalkeeper State Machine": "Goalkeeper",
}

LINE_PREFIX = "Line: "
LINE_ROW_INDEX = 2
LINE_VALUES_SPACING = 2


@dataclass
class MenuItem:
    label: str | Callable[[], str]
    action: Callable[[], Screen | None]


class Screen:
    def render(self, now: float, max_chars: int) -> tuple[list[str], int | None, RenderHints]:
        raise NotImplementedError

    def handle_event(self, event: str) -> Screen | None:
        raise NotImplementedError


class MenuScreen(Screen):
    def __init__(self, items: Sequence[MenuItem], selected_index: int = 0) -> None:
        self._items = list(items)
        self._selected_index = 0 if not items else min(max(selected_index, 0), len(items) - 1)
        self._start_index = 0

    def handle_event(self, event: str) -> Screen | None:
        if not self._items:
            return None

        if event == "up":
            self._selected_index = (self._selected_index - 1) % len(self._items)
        elif event == "down":
            self._selected_index = (self._selected_index + 1) % len(self._items)
        elif event == "select":
            return self._items[self._selected_index].action()

        self._update_start_index()
        return None

    def render(self, now: float, max_chars: int) -> tuple[list[str], int | None, RenderHints]:
        labels = [_label_text(item.label) for item in self._items]
        self._update_start_index()
        return _render_menu_lines(labels, self._selected_index, max_chars, self._start_index)

    def _update_start_index(self) -> None:
        if len(self._items) <= MAX_LINES:
            self._start_index = 0
            return

        if self._selected_index < self._start_index:
            self._start_index = self._selected_index
        elif self._selected_index >= self._start_index + MAX_LINES:
            self._start_index = self._selected_index - (MAX_LINES - 1)

        self._start_index = max(0, min(self._start_index, len(self._items) - MAX_LINES))


class StatusScreen(Screen):
    def __init__(self) -> None:
        self._page_index = 0

    def handle_event(self, event: str) -> Screen | None:
        pages_count = self._pages_count()
        if pages_count == 0:
            return None

        if event == "up":
            self._page_index = (self._page_index - 1) % pages_count
        elif event == "down":
            self._page_index = (self._page_index + 1) % pages_count

        return None

    def render(self, now: float, max_chars: int) -> tuple[list[str], int | None, RenderHints]:
        line_text, line_display = _line_detection_display()
        pages = _build_status_pages(now, line_text)
        if not pages:
            return _pad_lines(["No status available"]), None, RenderHints()

        page_index = self._page_index % len(pages)
        render_hints = RenderHints(
            show_up=page_index > 0,
            show_down=page_index < len(pages) - 1,
        )
        if page_index == 0:
            render_hints = RenderHints(
                show_up=render_hints.show_up,
                show_down=render_hints.show_down,
                line_display=line_display,
            )
        lines = [_truncate_text(line, max_chars) for line in pages[page_index]]
        return _apply_scroll_gutter(lines, max_chars, render_hints), None, render_hints

    def _pages_count(self) -> int:
        return len(_build_status_pages(time.monotonic()))


class MessageScreen(Screen):
    def __init__(self, lines: Sequence[str], duration_s: float = 0.5) -> None:
        self._lines = list(lines)
        self._expires_at = time.monotonic() + duration_s

    def handle_event(self, event: str) -> Screen | None:
        return None

    def render(self, now: float, max_chars: int) -> tuple[list[str], int | None, RenderHints]:
        lines = [_truncate_text(line, max_chars) for line in self._lines]
        return _pad_lines(lines), None, RenderHints()

    def is_expired(self, now: float) -> bool:
        return now >= self._expires_at


class ModeDisplayScreen(Screen):
    def __init__(self, show_lock: bool = False, show_down: bool = False) -> None:
        self._show_lock = show_lock
        self._show_down = show_down

    def handle_event(self, event: str) -> Screen | None:
        return None

    def render(self, now: float, max_chars: int) -> tuple[list[str], int | None, RenderHints]:
        lines = _mode_display_lines(max_chars)
        return _pad_lines(lines), None, RenderHints(centered=True, show_lock=self._show_lock, show_down=self._show_down)


class RepeatButton:
    button: Button
    last_pressed: bool = False
    next_repeat_at: float = 0.0

    def __init__(self, button: Button) -> None:
        self.button = button

    def poll(self, now: float, allow_repeat: bool) -> bool:
        pressed = self.button.is_pressed # pyright: ignore[reportAttributeAccessIssue]
        fired = False

        if pressed and not self.last_pressed:
            fired = True
            if allow_repeat:
                self.next_repeat_at = now + REPEAT_INITIAL_DELAY_S
        elif pressed and allow_repeat and self.next_repeat_at and now >= self.next_repeat_at:
            fired = True
            self.next_repeat_at = now + REPEAT_INTERVAL_S

        if not pressed:
            self.next_repeat_at = 0.0

        self.last_pressed = pressed
        return fired


def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger) -> None:
    logger.info("Starting display process")

    serial = i2c(port=1, address=0x3C)
    device = ssd1306(serial, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)

    font_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "font.ttf"))
    logger.info(f"Loading font from {font_path}")
    font = ImageFont.truetype(font_path, 8)

    char_width = font.getbbox("A")[2]
    max_chars = int(max(1, DISPLAY_WIDTH // (char_width + TEXT_SPACING)))

    buttons = _init_buttons()

    main_menu = _build_main_menu()
    screen_stack: list[Screen] = [main_menu, ModeDisplayScreen(show_down=True)]
    screensaver = ModeDisplayScreen(show_down=True)

    last_frame: tuple[object, ...] | None = None
    last_input_at = time.monotonic()

    while not stop_event.is_set():
        now = time.monotonic()

        running_state = shared_data.get_running_state()
        motors_on = bool(running_state and running_state.running)

        if motors_on:
            lines = _mode_display_lines(max_chars)
            frame = ("mode", tuple(lines))
            if frame != last_frame:
                _render_centered_frame(device, font, lines, RenderHints(show_lock=True))
                last_frame = frame
            sleep(0.05)
            continue

        event = _read_event(buttons, now)
        if event:
            current = screen_stack[-1]
            last_input_at = now
            if isinstance(current, ModeDisplayScreen):
                screen_stack.pop()
            elif event == "back":
                if len(screen_stack) > 1:
                    screen_stack.pop()
            else:
                next_screen = current.handle_event(event)
                if next_screen is not None:
                    screen_stack.append(next_screen)

        current = screen_stack[-1]
        if isinstance(current, MessageScreen) and current.is_expired(now):
            screen_stack.pop()
            screen_stack.pop()
            if len(screen_stack) == 0:
                screen_stack.append(main_menu)
            current = screen_stack[-1]

        if (
            now - last_input_at >= SCREENSAVER_IDLE_S
            and isinstance(current, (MenuScreen))
        ):
            screen_stack.append(screensaver)
            current = screen_stack[-1]

        lines, highlight, render_hints = current.render(now, max_chars)
        if render_hints.centered:
            frame = ("centered", tuple(lines), render_hints.show_lock)
            if frame != last_frame:
                _render_centered_frame(device, font, lines, render_hints)
                last_frame = frame
        else:
            line_display_key = None
            if render_hints.line_display is not None:
                line_display_key = (
                    render_hints.line_display.row_index,
                    render_hints.line_display.digits,
                    render_hints.line_display.detected,
                )
            frame = (
                "menu",
                tuple(lines),
                highlight,
                render_hints.show_up,
                render_hints.show_down,
                line_display_key,
            )
            if frame != last_frame:
                _render_frame(device, font, lines, highlight, render_hints)
                last_frame = frame

        sleep(0.05)


def _init_buttons() -> dict[str, RepeatButton]:
    return {
        "up": RepeatButton(Button(BUTTON_UP_PIN, pull_up=True, bounce_time=0.05)),
        "down": RepeatButton(Button(BUTTON_DOWN_PIN, pull_up=True, bounce_time=0.05)),
        "select": RepeatButton(Button(BUTTON_SELECT_PIN, pull_up=True, bounce_time=0.05)),
        "back": RepeatButton(Button(BUTTON_BACK_PIN, pull_up=True, bounce_time=0.05)),
    }


def _read_event(buttons: dict[str, RepeatButton], now: float) -> str | None:
    if buttons["up"].poll(now, allow_repeat=True):
        return "up"
    if buttons["down"].poll(now, allow_repeat=True):
        return "down"
    if buttons["select"].poll(now, allow_repeat=False):
        return "select"
    if buttons["back"].poll(now, allow_repeat=False):
        return "back"
    return None


def _build_main_menu() -> MenuScreen:
    return MenuScreen(
        [
            MenuItem("Status", lambda: StatusScreen()),
            MenuItem("Reset Compass", _reset_compass),
            MenuItem(_goal_color_menu_label, _build_goal_color_menu),
            MenuItem(_bluetooth_menu_label, _build_bluetooth_menu),
            MenuItem(_mode_menu_label, _build_mode_menu),
            MenuItem(_state_machine_menu_label, _build_state_machine_menu),
        ],
    )


def _build_mode_menu() -> MenuScreen:
    options = [
        RobotMode.IDLE,
        RobotMode.MANUAL,
        RobotMode.AUTONOMOUS,
    ]
    current_mode = shared_data.get_robot_mode()
    selected_index = next(
        (idx for idx, option in enumerate(options) if option == current_mode),
        0,
    )

    items = [
        MenuItem(_mode_label(mode), lambda mode=mode: _set_mode(mode, _mode_label(mode)))
        for mode in options
    ]
    return MenuScreen(items, selected_index=selected_index)


def _build_state_machine_menu() -> MenuScreen:
    machines = list(autonomous_mode.get_available_state_machines().keys())
    machines.sort()

    current = shared_data.get_current_state_machine_name()
    selected_index = machines.index(current) if current in machines else 0

    whitelist = ["Attacker State Machine", "Goalkeeper State Machine"]
    machines = [m for m in machines if m in whitelist]

    items = [
        MenuItem(name, lambda machine=name: _set_state_machine(machine))
        for name in machines
    ]
    return MenuScreen(items, selected_index=selected_index)


def _build_goal_color_menu() -> MenuScreen:
    options = ["yellow", "blue"]
    current_color = shared_data.get_goal_color().lower()
    selected_index = options.index(current_color) if current_color in options else 0

    items = [
        MenuItem(_goal_color_label(color), lambda color=color: _set_goal_color(color))
        for color in options
    ]
    return MenuScreen(items, selected_index=selected_index)


def _build_bluetooth_menu() -> MenuScreen:
    options = [True, False]
    labels = ["Enabled", "Disabled"]
    current = shared_data.get_bluetooth_enabled()
    selected_index = 0 if current else 1

    items = [
        MenuItem(labels[idx], lambda enabled=option: _set_bluetooth_enabled(enabled))
        for idx, option in enumerate(options)
    ]
    return MenuScreen(items, selected_index=selected_index)


def _reset_compass() -> Screen:
    shared_data.request_compass_reset()
    return MessageScreen(["Compass reset"])


def _set_mode(mode: int, label: str) -> Screen:
    shared_data.set_robot_mode(mode)
    return MessageScreen(["Mode set:", label])


def _set_state_machine(machine: str) -> Screen:
    autonomous_mode.set_current_state_machine(machine)
    return MessageScreen(["State machine:", machine])


def _set_goal_color(color: str) -> Screen:
    shared_data.set_goal_color(color)
    return MessageScreen(["Enemy goal:", _goal_color_label(color)])


def _set_bluetooth_enabled(enabled: bool) -> Screen:
    bluetooth_utils.set_bluetooth_enabled(enabled)
    return MessageScreen(["Bluetooth:", "Enabled" if enabled else "Disabled"])


def _mode_menu_label() -> str:
    return f"Mode: {_mode_label(shared_data.get_robot_mode())}"


def _state_machine_menu_label() -> str:
    name = shared_data.get_current_state_machine_name() or "--"
    return f"SM: {name}"


def _goal_color_menu_label() -> str:
    return f"Enemy goal: {_goal_color_label(shared_data.get_goal_color())}"


def _bluetooth_menu_label() -> str:
    return f"BT Comm: {_format_bool(shared_data.get_bluetooth_enabled())}"


def _build_status_pages(now: float, line_display: str | None = None) -> list[list[str]]:
    ip_address = _get_ip_address(now)
    heading = _heading_text()

    running_state = shared_data.get_running_state()
    main_switch = _format_bool(running_state.main_switch_value if running_state else None)
    bt_enabled = _format_bool(running_state.bt_module_enabled if running_state else None)
    bt_state = _format_bool(running_state.bt_module_value if running_state else None)

    _, ir_angle, ir_distance = shared_data.get_hardware_compass_ir()
    ir_detected = ir_angle != 999.0 and ir_distance != 0.0

    camera_ball = shared_data.get_camera_ball_data()

    line_display = line_display or _line_detection_display()[0]

    enemy_color = shared_data.get_goal_color().lower()
    own_color = "blue" if enemy_color == "yellow" else "yellow"
    enemy_goal = shared_data.get_goal_detection_result_for_color(enemy_color)
    own_goal = shared_data.get_goal_detection_result_for_color(own_color)

    bluetooth_info = shared_data.get_bluetooth_other_robot_info()
    bluetooth_mac_address = bluetooth_info.get("mac_address")
    bluetooth_status = _bluetooth_other_robot_status(bluetooth_mac_address)
    bluetooth_enabled = _format_bool(shared_data.get_bluetooth_enabled())

    return [
        [
            f"IP: {ip_address}",
            f"Head: {heading}",
            line_display,
        ],
        [
            "Ball",
            _format_ball_line("IR", ir_angle, ir_distance, ir_detected),
            _format_ball_line("Cam", camera_ball.angle, camera_ball.distance, camera_ball.detected),
        ],
        [
            "Goal",
            _format_goal_line("E", enemy_color, enemy_goal),
            _format_goal_line("O", own_color, own_goal),
        ],
        _position_estimate_lines(),
        [
            f"BT Comm: {bluetooth_enabled}",
            f"State: {bluetooth_status}",
            _bluetooth_other_robot_detail_line(bluetooth_info),
        ],
        [
            f"Switch: {main_switch}",
            f"BT Mdl: {bt_enabled}",
            f"BT Val: {bt_state}",
        ],
    ]


def _heading_text() -> str:
    heading, _, _ = shared_data.get_hardware_compass_ir()
    if heading is None or heading == 999.0:
        return "--"
    return f"{int(heading) % 360}deg"


def _format_bool(value: bool | None) -> str:
    if value is None:
        return "--"
    return "ON" if value else "OFF"


def _bluetooth_other_robot_status(mac_address: str | None) -> str:
    if not mac_address:
        return "Not Set"
    connected_devices = shared_data.get_bluetooth_devices_info()
    for device in connected_devices:
        device_mac = device.get("mac_address")
        if isinstance(device_mac, str) and device_mac.lower() == mac_address.lower():
            return "Connected"
    return "Disconnected"


def _bluetooth_other_robot_detail_line(info: dict) -> str:
    ip_address = info.get("ip_address")
    if isinstance(ip_address, str) and ip_address.strip():
        return f"IP: {ip_address}"
    name = info.get("name") or info.get("hostname")
    if isinstance(name, str) and name.strip():
        return f"Name: {name}"
    return "IP: --"


def _format_ball_line(label: str, angle: float, distance: float, detected: bool) -> str:
    if not detected:
        return f"{label}: ND"
    angle_text = _format_angle(angle)
    distance_text = _format_distance(distance)
    return f"{label}: {angle_text} {distance_text}"


def _format_angle(angle: float) -> str:
    if angle == 999.0:
        return "--"
    return f"{int(round(angle))}d"


def _format_distance(distance: float | None) -> str:
    if distance is None or distance <= 0:
        return "--"
    if distance >= 1000.0:
        return f"{distance / 1000.0:.1f}m"
    return f"{int(round(distance))}mm"


def _format_position_mm(value: float | None) -> str:
    if value is None:
        return "--"
    return f"{int(round(value))}mm"


def _format_confidence(confidence: float | None) -> str:
    if confidence is None:
        return "--"
    return f"{int(round(max(0.0, min(confidence, 1.0)) * 100))}%"


def _position_estimate_lines() -> list[str]:
    estimate = shared_data.get_last_position_estimate()
    if not estimate:
        return ["Position", "X: -- Y: --", "Conf: --"]

    x_mm = estimate.get("x_mm")
    y_mm = estimate.get("y_mm")
    confidence = estimate.get("confidence")

    x_text = _format_position_mm(x_mm if isinstance(x_mm, (int, float)) else None)
    y_text = _format_position_mm(y_mm if isinstance(y_mm, (int, float)) else None)
    conf_text = _format_confidence(confidence if isinstance(confidence, (int, float)) else None)

    return ["Position", f"X: {x_text} Y: {y_text}", f"Conf: {conf_text}"]


def _line_detection_display() -> tuple[str, LineDisplay]:
    hardware_data = shared_data.get_hardware_data()
    line_values = hardware_data.line if hardware_data else []

    thresholds = shared_data.get_line_detection_thresholds()
    with shared_data.line_detected_lock:
        detected_values = list(shared_data.line_detected[:])

    digits: list[str] = []
    detected: list[bool] = []
    for idx in range(LINE_SENSOR_COUNT):
        value = line_values[idx] if idx < len(line_values) else None
        threshold_min, threshold_max = _line_threshold_for_index(thresholds, idx)
        percent = _line_strength_percent(value, threshold_min, threshold_max)
        digits.append(_line_detection_digit(percent))
        detected.append(detected_values[idx] if idx < len(detected_values) else False)

    digits_text = "".join(digits)
    line_text = f"{LINE_PREFIX}{digits_text}"
    return line_text, LineDisplay(
        row_index=LINE_ROW_INDEX,
        digits=tuple(digits),
        detected=tuple(detected),
    )


def _line_threshold_for_index(thresholds: list[list[int]], idx: int) -> tuple[int, int]:
    if idx < len(thresholds):
        threshold_min, threshold_max = thresholds[idx]
    else:
        threshold_min, threshold_max = LINE_SENSOR_MIN_VALUE, LINE_SENSOR_MAX_VALUE

    if threshold_min > threshold_max:
        threshold_min, threshold_max = threshold_max, threshold_min
    return threshold_min, threshold_max


def _line_strength_percent(value: int | float | None, threshold_min: int, threshold_max: int) -> float:
    if value is None:
        return 0.0
    value = max(LINE_SENSOR_MIN_VALUE, min(int(value), LINE_SENSOR_MAX_VALUE))

    if value < threshold_min:
        span = max(1, threshold_min - LINE_SENSOR_MIN_VALUE)
        return min(100.0, ((threshold_min - value) / span) * 100.0)
    if value > threshold_max:
        span = max(1, LINE_SENSOR_MAX_VALUE - threshold_max)
        return min(100.0, ((value - threshold_max) / span) * 100.0)
    return 0.0


def _line_detection_digit(percent: float) -> str:
    if percent <= 0:
        return "0"
    digit = int(percent) // 10
    if digit >= 9:
        return "9"
    return str(digit)


def _format_goal_line(label: str, color: str, result: GoalDetectionResult | None) -> str:
    color_label = _goal_color_short_label(color)
    if not result or not result.detected or result.distance_mm is None:
        return f"{label}: {color_label}: --"
    return f"{label}: {color_label}: {_format_distance(result.distance_mm)} {result.camera_yaw_deg + result.alignment * CAMERA_FOV_DEG / 2:.0f}d"


def _goal_color_label(color: str) -> str:
    return "Yellow" if color.lower() == "yellow" else "Blue"


def _goal_color_short_label(color: str) -> str:
    return "Yelw" if color.lower() == "yellow" else "Blue"


def _mode_label(mode: int) -> str:
    if mode == RobotMode.IDLE:
        return "Idle"
    if mode == RobotMode.MANUAL:
        return "Manual"
    if mode == RobotMode.AUTONOMOUS:
        return "Autonomous"
    return "Unknown"


_last_ip_lookup: float = 0.0
_last_ip_value: str = IP_FALLBACK


def _get_ip_address(now: float) -> str:
    global _last_ip_lookup, _last_ip_value
    if now - _last_ip_lookup < IP_CACHE_TTL_S:
        return _last_ip_value

    _last_ip_lookup = now
    _last_ip_value = _resolve_ip_address()
    return _last_ip_value


def _resolve_ip_address() -> str:
    socket_instance: socket.socket | None = None
    try:
        socket_instance = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socket_instance.connect(("8.8.8.8", 80))
        return socket_instance.getsockname()[0]
    except OSError:
        return IP_FALLBACK
    finally:
        if socket_instance is not None:
            socket_instance.close()


def _render_menu_lines(
    labels: Sequence[str],
    selected_index: int,
    max_chars: int,
    start_index: int = 0,
) -> tuple[list[str], int | None, RenderHints]:
    if not labels:
        return _pad_lines(["No menu items"]), None, RenderHints()

    max_label_chars = max(1, max_chars - 2)
    if len(labels) <= MAX_LINES:
        start_index = 0
    else:
        start_index = max(0, min(start_index, len(labels) - MAX_LINES))

    visible_labels = labels[start_index : start_index + MAX_LINES]
    highlight_index = selected_index - start_index
    lines = []

    for idx, label in enumerate(visible_labels):
        prefix = "> " if idx == highlight_index else "  "
        lines.append(prefix + _truncate_text(label, max_label_chars))

    render_hints = RenderHints(
        show_up=start_index > 0,
        show_down=start_index + MAX_LINES < len(labels),
    )
    return _apply_scroll_gutter(lines, max_chars, render_hints), highlight_index, render_hints


@dataclass(frozen=True)
class LineDisplay:
    row_index: int
    digits: tuple[str, ...]
    detected: tuple[bool, ...]


@dataclass(frozen=True)
class RenderHints:
    show_up: bool = False
    show_down: bool = False
    line_display: LineDisplay | None = None
    centered: bool = False
    show_lock: bool = False


def _render_frame(
    device: ssd1306,
    font: ImageFont.FreeTypeFont,
    lines: Sequence[str],
    highlight_index: int | None,
    render_hints: RenderHints,
) -> None:
    image = Image.new("1", (DISPLAY_WIDTH, DISPLAY_HEIGHT))
    draw = ImageDraw.Draw(image)

    padded_lines = _pad_lines(list(lines))

    for idx, line in enumerate(padded_lines[:MAX_LINES]):
        y = LINE_HEIGHT * idx + LINE_MARGIN
        if render_hints.line_display and idx == render_hints.line_display.row_index:
            _draw_line_detection_row(draw, font, y, render_hints.line_display)
            continue
        if highlight_index is not None and idx == highlight_index:
            top = max(0, y)
            bottom = min(DISPLAY_HEIGHT - 1, y + LINE_HEIGHT - 1)
            draw.rectangle((0, top, DISPLAY_WIDTH, bottom - (1 if LINE_MARGIN % 2 == 1 else 0)), fill=255)
            draw_mono_text(draw, (0, y), line, font, fill=0, spacing=TEXT_SPACING, char_width=MONO_CHAR_WIDTH)
        else:
            draw_mono_text(draw, (0, y), line, font, fill=255, spacing=TEXT_SPACING, char_width=MONO_CHAR_WIDTH)

    _draw_scroll_indicators(draw, render_hints, highlight_index)
    device.display(image)


def _truncate_text(text: str, max_chars: int) -> str:
    if len(text) <= max_chars:
        return text
    if max_chars <= 3:
        return text[:max_chars]
    return text[: max_chars - 1] + "…"


def _pad_lines(lines: Sequence[str]) -> list[str]:
    padded = list(lines)
    while len(padded) < MAX_LINES:
        padded.append("")
    return padded[:MAX_LINES]


def _apply_scroll_gutter(lines: Sequence[str], max_chars: int, scroll_hints: RenderHints) -> list[str]:
    padded = _pad_lines(lines)
    if max_chars <= 1:
        return padded
    if scroll_hints.show_up:
        padded[0] = _truncate_text(padded[0], max_chars - 1)
    if scroll_hints.show_down:
        padded[-1] = _truncate_text(padded[-1], max_chars - 1)
    return padded


def _render_centered_frame(
    device: ssd1306,
    font: ImageFont.FreeTypeFont,
    lines: Sequence[str],
    render_hints: RenderHints | None = None,
) -> None:
    image = Image.new("1", (DISPLAY_WIDTH, DISPLAY_HEIGHT))
    draw = ImageDraw.Draw(image)

    padded_lines = _pad_lines(list(lines))

    for idx, line in enumerate(padded_lines[:MAX_LINES]):
        if not line:
            continue
        y = LINE_HEIGHT * idx + LINE_MARGIN
        text_width = _mono_text_width(line)
        x = max(0, (DISPLAY_WIDTH - text_width) // 2)
        draw_mono_text(draw, (x, y), line, font, fill=255, spacing=TEXT_SPACING, char_width=MONO_CHAR_WIDTH)

    if render_hints is not None:
        if render_hints.show_lock:
            _draw_lock_indicator(draw)
        _draw_scroll_indicators(draw, render_hints)

    device.display(image)


def _mono_text_width(text: str) -> int:
    if not text:
        return 0
    return (len(text) * (MONO_CHAR_WIDTH + TEXT_SPACING)) - TEXT_SPACING


def _draw_lock_indicator(draw: ImageDraw.ImageDraw) -> None:
    x = DISPLAY_WIDTH - LOCK_ICON_WIDTH - LOCK_ICON_MARGIN_X
    y = LOCK_ICON_MARGIN_Y
    x = max(0, x)

    shackle_bottom = y + 2
    draw.rectangle(
        (x + 1, y, x + LOCK_ICON_WIDTH - 2, shackle_bottom),
        outline=255,
        fill=0,
    )

    body_top = shackle_bottom + 1
    draw.rectangle(
        (x, body_top, x + LOCK_ICON_WIDTH - 1, y + LOCK_ICON_HEIGHT - 1),
        fill=255,
    )


def _draw_line_detection_row(
    draw: ImageDraw.ImageDraw,
    font: ImageFont.FreeTypeFont,
    y: int,
    line_display: LineDisplay,
) -> None:
    x = 0
    draw_mono_text(draw, (x, y), LINE_PREFIX, font, fill=255, spacing=TEXT_SPACING, char_width=MONO_CHAR_WIDTH)
    x += _mono_text_width(LINE_PREFIX)

    for digit, detected in zip(line_display.digits, line_display.detected):
        if detected:
            draw.rectangle(
                (x - math.floor(LINE_VALUES_SPACING / 2), y, x + MONO_CHAR_WIDTH + math.ceil(LINE_VALUES_SPACING / 2) - 1, y + LINE_HEIGHT - 1),
                fill=255,
            )
            draw_mono_text(draw, (x, y), digit, font, fill=0, spacing=LINE_VALUES_SPACING, char_width=MONO_CHAR_WIDTH)
        else:
            draw_mono_text(draw, (x, y), digit, font, fill=255, spacing=LINE_VALUES_SPACING, char_width=MONO_CHAR_WIDTH)
        x += MONO_CHAR_WIDTH + LINE_VALUES_SPACING


def _draw_scroll_indicators(
    draw: ImageDraw.ImageDraw,
    scroll_hints: RenderHints,
    highlight_index: int | None = None,
) -> None:
    if not scroll_hints.show_up and not scroll_hints.show_down:
        return

    x = DISPLAY_WIDTH - SCROLL_ARROW_WIDTH - SCROLL_ARROW_MARGIN_X

    if scroll_hints.show_up:
        y = LINE_MARGIN + SCROLL_ARROW_MARGIN_Y
        fill = 0 if highlight_index == 0 else 255
        draw.polygon(
            (
                (x, y + SCROLL_ARROW_HEIGHT - 1),
                (x + SCROLL_ARROW_WIDTH // 2, y),
                (x + SCROLL_ARROW_WIDTH - 1, y + SCROLL_ARROW_HEIGHT - 1),
            ),
            fill=fill,
        )

    if scroll_hints.show_down:
        y = DISPLAY_HEIGHT - LINE_MARGIN - SCROLL_ARROW_MARGIN_Y - SCROLL_ARROW_HEIGHT
        fill = 0 if highlight_index == MAX_LINES - 1 else 255
        draw.polygon(
            (
                (x, y),
                (x + SCROLL_ARROW_WIDTH // 2, y + SCROLL_ARROW_HEIGHT - 1),
                (x + SCROLL_ARROW_WIDTH - 1, y),
            ),
            fill=fill,
        )


def _label_text(label: str | Callable[[], str]) -> str:
    return label() if callable(label) else label


def _role_label(name: str | None) -> str:
    if not name:
        return "--"
    if name in ROLE_LABELS:
        return ROLE_LABELS[name]
    lowered = name.lower()
    if "goalkeeper" in lowered:
        return "Goalkeeper"
    if "attacker" in lowered:
        return "Attacker"
    return "--"


def _mode_display_lines(max_chars: int) -> list[str]:
    mode = shared_data.get_robot_mode()
    title = _truncate_text(MODE_DISPLAY_TITLE, max_chars)
    if mode == RobotMode.AUTONOMOUS:
        subtitle = _role_label(shared_data.get_current_state_machine_name())
    else:
        subtitle = _mode_label(mode)
    subtitle = _truncate_text(subtitle, max_chars)
    return ["", title, subtitle]
