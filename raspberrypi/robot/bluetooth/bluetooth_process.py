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
    shared_data.set_bluetooth_paired_devices_info([d.to_dict() for d in manager.get_paired_devices()])
    shared_data.set_bluetooth_devices_info([d.to_dict() for d in manager.get_connected_devices()])


def _drain_incoming_messages(manager: BluetoothManager) -> None:
    messages_by_mac = manager.get_messages()
    if not messages_by_mac:
        return

    for sender_mac, messages in messages_by_mac.items():
        for message in messages:
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
            success = manager.connect_to_device(mac_address)
            shared_data.set_bluetooth_command_result(command_id, success, data={"mac_address": mac_address})

        elif command_type == "disconnect":
            mac_address = payload.get("mac_address")
            if not mac_address:
                raise ValueError("mac_address is required")
            success = manager.disconnect_from_device(mac_address)
            shared_data.set_bluetooth_command_result(command_id, success, data={"mac_address": mac_address})

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
                sender_id=payload.get("sender_id"),
            )
            success = manager.send_message(mac_address, outgoing)

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

        elif command_type == "add_paired_device":
            name = payload.get("name")
            mac_address = payload.get("mac_address")
            if not name or not mac_address:
                raise ValueError("name and mac_address are required")

            device = manager.add_paired_device(
                name=name,
                mac_address=mac_address,
                hostname=payload.get("hostname"),
                ip_address=payload.get("ip_address"),
            )
            shared_data.set_bluetooth_command_result(command_id, True, data=device.to_dict())

        elif command_type == "remove_paired_device":
            mac_address = payload.get("mac_address")
            if not mac_address:
                raise ValueError("mac_address is required")

            success = manager.remove_paired_device(mac_address)
            shared_data.set_bluetooth_command_result(command_id, success, data={"mac_address": mac_address})

        elif command_type == "refresh_state":
            shared_data.set_bluetooth_command_result(command_id, True, data={"refreshed": True})

        elif command_type == "list_pairable_devices":
            timeout_seconds_raw = payload.get("timeout_seconds", 6)
            timeout_seconds = timeout_seconds_raw if isinstance(timeout_seconds_raw, int) else 6
            if timeout_seconds <= 0:
                raise ValueError("timeout_seconds must be positive")

            devices = manager.list_pairable_devices(timeout_seconds=timeout_seconds)
            shared_data.set_bluetooth_command_result(
                command_id,
                True,
                data={"devices": devices, "timeout_seconds": timeout_seconds},
            )

        elif command_type == "set_discoverable":
            duration = payload.get("duration_seconds")
            success = manager.set_discoverable(duration)
            shared_data.set_bluetooth_command_result(command_id, success, data={"discoverable": True} if success else {}, error=None if success else "Failed to set discoverable")

        elif command_type == "set_not_discoverable":
            success = manager.set_not_discoverable()
            shared_data.set_bluetooth_command_result(command_id, success, data={"discoverable": False} if success else {}, error=None if success else "Failed to set non-discoverable")

        else:
            raise ValueError(f"Unknown bluetooth command: {command_type}")

    except Exception as exc:
        shared_data.set_bluetooth_command_result(command_id, False, error=str(exc))


def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    manager = BluetoothManager()

    try:
        listening_ok = manager.start_listening()
        if not listening_ok:
            logger.warning("Bluetooth listening failed to start; command processing will continue")

        shared_data.set_bluetooth_process_alive(True)
        _refresh_shared_state(manager)

        while not stop_event.is_set():
            commands = shared_data.pop_bluetooth_commands()
            for command in commands:
                _execute_command(manager, command)

            _drain_incoming_messages(manager)
            _refresh_shared_state(manager)
            time.sleep(_COMMAND_POLL_INTERVAL_S)

    except Exception as exc:
        logger.error(f"Bluetooth process crashed: {exc}", exc_info=True)
    finally:
        shared_data.set_bluetooth_process_alive(False)
        shared_data.set_bluetooth_devices_info([])
        manager.stop_listening()

