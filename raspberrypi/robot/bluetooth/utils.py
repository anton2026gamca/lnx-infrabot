"""High-level Bluetooth helpers for non-Bluetooth processes.

This module is intentionally process-safe and communicates with the dedicated
Bluetooth process via shared_data queues/state.
"""

import time

from robot import calibration
from robot.multiprocessing import shared_data


_DEFAULT_TIMEOUT_S = 3.0
_DEFAULT_POLL_INTERVAL_S = 0.02


def _execute_command(
    command_type: str,
    payload: dict | None = None,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
    poll_interval_s: float = _DEFAULT_POLL_INTERVAL_S,
    pop_result: bool = True,
) -> dict:
    """Send a command to the Bluetooth process and wait for result."""
    command_id = shared_data.enqueue_bluetooth_command(command_type, payload or {})

    start = time.time()
    while time.time() - start <= timeout_s:
        result = shared_data.get_bluetooth_command_result(command_id, pop=pop_result)
        if result is not None:
            return result
        time.sleep(poll_interval_s)

    return {
        "command_id": command_id,
        "success": False,
        "data": {},
        "error": f"timeout waiting for bluetooth command '{command_type}'",
        "timestamp": time.time(),
    }


# ---------------------------------------------------------------------------
# State readers
# ---------------------------------------------------------------------------

def is_bluetooth_process_alive() -> bool:
    return shared_data.get_bluetooth_process_alive()


def get_local_device_info() -> dict:
    return shared_data.get_bluetooth_device_info()


def get_connected_devices() -> list[dict]:
    return shared_data.get_bluetooth_devices_info()


def get_paired_devices() -> list[dict]:
    return shared_data.get_bluetooth_paired_devices_info()


def get_received_messages(clear: bool = False, limit: int | None = None) -> list[dict]:
    return shared_data.get_bluetooth_received_messages(clear=clear, limit=limit)


def get_sent_messages(clear: bool = False, limit: int | None = None) -> list[dict]:
    return shared_data.get_bluetooth_sent_messages(clear=clear, limit=limit)


def clear_message_history() -> None:
    shared_data.clear_bluetooth_received_messages()
    shared_data.clear_bluetooth_sent_messages()


def get_other_robot_info() -> dict:
    return shared_data.get_bluetooth_other_robot_info()


def set_other_robot_info(info: dict) -> None:
    shared_data.set_bluetooth_other_robot_info(info or {})
    calibration.save_calibration_data()


def clear_other_robot_info() -> None:
    shared_data.clear_bluetooth_other_robot_info()
    calibration.save_calibration_data()


# ---------------------------------------------------------------------------
# Command helpers
# ---------------------------------------------------------------------------

def refresh_state(timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command("refresh_state", timeout_s=timeout_s)


def connect(mac_address: str, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command("connect", payload={"mac_address": mac_address}, timeout_s=timeout_s)


def disconnect(mac_address: str, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command("disconnect", payload={"mac_address": mac_address}, timeout_s=timeout_s)


def add_paired_device(
    name: str,
    mac_address: str,
    hostname: str | None = None,
    ip_address: str | None = None,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> dict:
    return _execute_command(
        "add_paired_device",
        payload={
            "name": name,
            "mac_address": mac_address,
            "hostname": hostname,
            "ip_address": ip_address,
        },
        timeout_s=timeout_s,
    )


def remove_paired_device(mac_address: str, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command("remove_paired_device", payload={"mac_address": mac_address}, timeout_s=timeout_s)


def send_message(
    mac_address: str,
    content: str,
    message_type: str,
    sender_id: str | None = None,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> dict:
    return _execute_command(
        "send_message",
        payload={
            "mac_address": mac_address,
            "content": content,
            "message_type": message_type,
            "sender_id": sender_id,
        },
        timeout_s=timeout_s,
    )


def list_pairable_devices(timeout_seconds: int = 6, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command(
        "list_pairable_devices",
        payload={"timeout_seconds": timeout_seconds},
        timeout_s=timeout_s + max(timeout_seconds, 0),
    )


def set_discoverable(duration_seconds: int | None = None, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command(
        "set_discoverable",
        payload={"duration_seconds": duration_seconds},
        timeout_s=timeout_s,
    )


def set_not_discoverable(timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    return _execute_command(
        "set_not_discoverable",
        payload={},
        timeout_s=timeout_s,
    )
