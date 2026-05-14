import logging
import multiprocessing.synchronize
import time

from robot.bluetooth.bluetooth_manager import BluetoothManager, BluetoothMessage
from robot.multiprocessing import shared_data


_COMMAND_POLL_INTERVAL_S = 0.05


def _serialize_message(message: BluetoothMessage, sender_mac: str) -> dict:
    data = message.to_dict()
    data["sender_mac"] = sender_mac
    return data


def _refresh_shared_state(manager: BluetoothManager) -> None:
    shared_data.set_bluetooth_device_info(manager.get_device_info())
    shared_data.set_bluetooth_paired_devices_info([d.to_dict() for d in manager.list_paired_devices()])
    shared_data.set_bluetooth_devices_info([d.to_dict() for d in manager.list_paired_devices() if d.connected])


def _drain_incoming_messages(manager: BluetoothManager) -> None:
    messages_by_mac = manager.get_messages()
    if not messages_by_mac:
        return

    for sender_mac, message in messages_by_mac:
        shared_data.add_bluetooth_received_message(_serialize_message(message, sender_mac))


def _execute_command(manager: BluetoothManager, command: dict) -> None:
    command_id = int(command.get("id", 0))
    command_type = command.get("type", "")
    payload = command.get("payload", {}) or {}

    try:
        if command_type == "connect":
            mac_address = payload.get("mac_address")
            if not mac_address:
                raise ValueError("mac_address is required")
            success = manager.connect(mac_address)
            shared_data.set_bluetooth_command_result(command_id, success, data={"mac_address": mac_address})

        elif command_type == "disconnect":
            mac_address = payload.get("mac_address")
            if not mac_address:
                raise ValueError("mac_address is required")
            manager.disconnect(mac_address)
            shared_data.set_bluetooth_command_result(command_id, True, data={"mac_address": mac_address})

        elif command_type == "send_message":
            mac_address = payload.get("mac_address")
            message_type = payload.get("message_type")
            content = payload.get("content", "")

            if not mac_address:
                raise ValueError("mac_address is required")
            if not isinstance(message_type, str) or not message_type.strip():
                raise ValueError("message_type is required")

            outgoing = BluetoothMessage(
                message_type=message_type,
                content=str(content),
                sender_id=payload.get("sender_id", ""),
            )
            success = manager.send(mac_address, outgoing)

            if success:
                sent_data = outgoing.to_dict()
                sent_data["target_mac"] = mac_address
                shared_data.add_bluetooth_sent_message(sent_data)

            shared_data.set_bluetooth_command_result(
                command_id,
                success,
                data={"mac_address": mac_address, "message_id": outgoing.message_id},
                error=None if success else "send failed",
            )

        elif command_type == "pair_device":
            mac_address = payload.get("mac_address")
            if not mac_address:
                raise ValueError("mac_address is required")

            success = manager.pair(
                mac_address=mac_address,
            )
            shared_data.set_bluetooth_command_result(command_id, success)

        elif command_type == "unpair_device":
            mac_address = payload.get("mac_address")
            if not mac_address:
                raise ValueError("mac_address is required")

            success = manager.remove_pairing(mac_address)
            shared_data.set_bluetooth_command_result(command_id, success, data={"mac_address": mac_address})

        elif command_type == "refresh_state":
            shared_data.set_bluetooth_command_result(command_id, True, data={"refreshed": True})

        elif command_type == "list_pairable_devices":
            timeout_seconds_raw = payload.get("timeout_seconds", 6)
            timeout_seconds = timeout_seconds_raw if isinstance(timeout_seconds_raw, int) else 6
            if timeout_seconds <= 0:
                raise ValueError("timeout_seconds must be positive")

            devices = manager.scan(timeout_seconds)
            devices_dicts = [d.to_dict() for d in devices]
            shared_data.set_bluetooth_command_result(
                command_id,
                True,
                data={"devices": devices_dicts, "timeout_seconds": timeout_seconds},
            )

        elif command_type == "set_pairing_mode":
            enabled = payload.get("enabled", False)
            if not isinstance(enabled, bool):
                raise ValueError("enabled must be a boolean")
            success = manager.set_pairing_mode(enabled=enabled)
            shared_data.set_bluetooth_command_result(
                command_id,
                success,
                data={"pairing_mode_enabled": enabled} if success else {},
                error=None if success else f"Failed to set pairing mode to {enabled}"
            )

        else:
            raise ValueError(f"Unknown bluetooth command: {command_type}")

    except Exception as exc:
        shared_data.set_bluetooth_command_result(command_id, False, error=str(exc))


def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    manager = BluetoothManager()

    try:
        manager.start_server()

        shared_data.set_bluetooth_process_alive(True)
        _refresh_shared_state(manager)

        while not stop_event.is_set():
            commands = shared_data.pop_bluetooth_commands()
            for command in commands:
                _execute_command(manager, command)

            _drain_incoming_messages(manager)
            _refresh_shared_state(manager)
            time.sleep(_COMMAND_POLL_INTERVAL_S)

    except Exception as e:
        logger.error(f"Bluetooth process crashed: {e}", exc_info=True)
    finally:
        shared_data.set_bluetooth_process_alive(False)
        shared_data.set_bluetooth_devices_info([])
        manager.stop_server()

