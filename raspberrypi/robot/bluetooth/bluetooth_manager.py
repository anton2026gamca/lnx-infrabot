"""
Bluetooth Manager - Core Bluetooth communication logic

Handles Bluetooth socket connections, message sending/receiving,
device pairing management, and connection state tracking.

Uses Python's built-in socket module (AF_BLUETOOTH / BTPROTO_RFCOMM)
instead of the unmaintained PyBluez library — compatible with Python 3.13+.

Requirements: None (standard library only)
Platform: Linux / Raspberry Pi (RFCOMM requires BlueZ kernel support)
"""

import socket
import threading
import time
import json
import logging
import re
import subprocess
from collections.abc import Callable
from dataclasses import dataclass, asdict
import uuid

# AF_BLUETOOTH and BTPROTO_RFCOMM exist at runtime on Linux/BlueZ but are absent from the typeshed stubs
_AF_BLUETOOTH: int = getattr(socket, "AF_BLUETOOTH")
_BTPROTO_RFCOMM: int = getattr(socket, "BTPROTO_RFCOMM")


logger = logging.getLogger("Bluetooth Manager")


@dataclass
class BluetoothMessage:
    message_type: str
    content: str
    sender_id: str | None = None
    timestamp: float | None = None
    message_id: str | None = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
        if self.message_id is None:
            self.message_id = str(uuid.uuid4())
        if self.sender_id is None:
            self.sender_id = socket.gethostname()

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls, json_str: str) -> "BluetoothMessage | None":
        try:
            data = json.loads(json_str)
            return cls(
                message_type=data.get("message_type"),
                content=data.get("content"),
                sender_id=data.get("sender_id"),
                timestamp=data.get("timestamp"),
                message_id=data.get("message_id"),
            )
        except (json.JSONDecodeError, TypeError) as e:
            logger.error(f"Failed to parse message from JSON: {e}")
            return None


@dataclass
class PairedDevice:
    name: str                               # Device/robot name
    mac_address: str                        # Bluetooth MAC address
    hostname: str | None = None             # Robot hostname
    ip_address: str | None = None           # Robot IP address
    last_connected: float | None = None     # Timestamp of last connection
    is_connected: bool = False              # Current connection status
    device_id: str | None = None            # Unique device identifier

    def to_dict(self) -> dict:
        return asdict(self)


def _make_rfcomm_socket() -> socket.socket:
    """
    Create a raw RFCOMM Bluetooth socket using Python's built-in socket module.

    AF_BLUETOOTH and BTPROTO_RFCOMM are available on Linux/Raspberry Pi
    via the BlueZ kernel stack. No third-party library required.
    """
    return socket.socket(_AF_BLUETOOTH, socket.SOCK_STREAM, _BTPROTO_RFCOMM)


class BluetoothManager:
    """
    Manages Bluetooth Classic (RFCOMM) communication between robots.

    Uses Python's built-in socket module with AF_BLUETOOTH / BTPROTO_RFCOMM

    Features:
    - Listens for incoming RFCOMM connections
    - Connects to other robots
    - Sends/receives newline-delimited JSON messages
    - Manages paired devices
    - Tracks connection status
    """

    # Bluetooth RFCOMM channel (1–30; channel 1 is the standard SPP channel)
    RFCOMM_CHANNEL = 1

    # Bluetooth UUID for Serial Port Profile (informational — not used by raw sockets)
    SERVICE_UUID = "00001101-0000-1000-8000-00805F9B34FB"

    def __init__(self):
        self.device_id = socket.gethostname()
        self.hostname = socket.gethostname()
        self.ip_address = self._get_ip_address()

        # Paired devices: MAC address -> PairedDevice
        self.paired_devices: dict[str, PairedDevice] = {}

        # Active connections: MAC address -> (socket, thread)
        self.active_connections: dict[str, tuple[socket.socket, threading.Thread]] = {}

        # Message callbacks: message_type -> [callable, ...]
        self.message_handlers: dict[str, list[Callable[..., None]]] = {}

        # Server socket for accepting incoming connections
        self.server_socket: socket.socket | None = None
        self.server_thread: threading.Thread | None = None
        self.running = False

        # Re-entrant lock for thread safety
        self.lock = threading.RLock()

        # Received messages queue: MAC address -> [BluetoothMessage, ...]
        self.message_queue: dict[str, list[BluetoothMessage]] = {}

        logger.info(
            f"BluetoothManager initialized — device: {self.device_id}, "
            f"hostname: {self.hostname}"
        )

    @staticmethod
    def _get_ip_address() -> str:
        """Determine the local IP address (does not send any traffic)"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    # ------------------------------------------------------------------ #
    #  Message handler registration                                        #
    # ------------------------------------------------------------------ #

    def register_message_handler(self, message_type: str, callback: Callable[..., None]) -> None:
        """
        Register a callback for a specific message type.

        Args:
            message_type: Application-defined message type string
            callback: Called as callback(message: BluetoothMessage, sender_mac: str)
        """
        with self.lock:
            self.message_handlers.setdefault(message_type, []).append(callback)
            logger.debug(f"Registered handler for message type: {message_type}")

    def unregister_message_handler(self, message_type: str, callback: Callable[..., None]) -> None:
        """Unregister a previously registered message handler"""
        with self.lock:
            handlers = self.message_handlers.get(message_type, [])
            try:
                handlers.remove(callback)
            except ValueError:
                pass

    # ------------------------------------------------------------------ #
    #  Server — listen for incoming connections                            #
    # ------------------------------------------------------------------ #

    def start_listening(self) -> bool:
        """
        Bind an RFCOMM socket and start accepting incoming connections.

        Returns:
            True if the server started successfully, False otherwise.
        """
        if self.running:
            logger.warning("Already listening for connections")
            return True

        try:
            # Bind to BDADDR_ANY (all local Bluetooth adapters)
            self.server_socket = _make_rfcomm_socket()
            self.server_socket.bind(("00:00:00:00:00:00", self.RFCOMM_CHANNEL))
            self.server_socket.listen(1)

            self.running = True
            self.server_thread = threading.Thread(
                target=self._listen_loop, daemon=True
            )
            self.server_thread.start()

            logger.info(f"Listening on RFCOMM channel {self.RFCOMM_CHANNEL}")
            return True

        except Exception as e:
            logger.error(f"Failed to start listening: {e}", exc_info=True)
            self.running = False
            if self.server_socket:
                self.server_socket.close()
                self.server_socket = None
            return False

    def stop_listening(self) -> None:
        """Stop listening for incoming connections and close the server socket"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None
        logger.info("Stopped listening for connections")

    def _listen_loop(self) -> None:
        """Accept loop — runs in the server thread"""
        while self.running:
            try:
                if self.server_socket:
                    client_socket, client_addr = self.server_socket.accept()
                    # client_addr is (mac_address, channel) for RFCOMM
                    client_mac = client_addr[0]
                    logger.info(f"Incoming connection from {client_mac}")
                    self._register_connection(client_socket, client_mac)
            except OSError as e:
                if self.running:
                    logger.debug(f"Accept error: {e}")
                time.sleep(0.1)

    # ------------------------------------------------------------------ #
    #  Client — connect to a remote device                                 #
    # ------------------------------------------------------------------ #

    def connect_to_device(self, mac_address: str) -> bool:
        """
        Open an outgoing RFCOMM connection to another robot.

        Args:
            mac_address: Bluetooth MAC address of the target (e.g. "AA:BB:CC:DD:EE:FF")

        Returns:
            True if connected successfully, False otherwise.
        """
        try:
            with self.lock:
                self._close_connection(mac_address)

                sock = _make_rfcomm_socket()
                sock.connect((mac_address, self.RFCOMM_CHANNEL))

                hostname = self._derive_hostname(mac_address)
                self._register_connection(sock, mac_address)

                if mac_address in self.paired_devices:
                    dev = self.paired_devices[mac_address]
                    dev.is_connected = True
                    dev.last_connected = time.time()
                    dev.hostname = hostname

                logger.info(f"Connected to {mac_address} ({hostname})")
                return True

        except Exception as e:
            logger.error(f"Failed to connect to {mac_address}: {e}")
            return False

    def disconnect_from_device(self, mac_address: str) -> bool:
        """
        Close the connection to a device.

        Args:
            mac_address: Bluetooth MAC address

        Returns:
            True if disconnected successfully, False otherwise.
        """
        try:
            with self.lock:
                disconnected = self._close_connection(mac_address)
                if disconnected and mac_address in self.paired_devices:
                    self.paired_devices[mac_address].is_connected = False
                return disconnected
        except Exception as e:
            logger.error(f"Error disconnecting from {mac_address}: {e}")
            return False

    # ------------------------------------------------------------------ #
    #  Messaging                                                           #
    # ------------------------------------------------------------------ #

    def send_message(self, mac_address: str, message: BluetoothMessage) -> bool:
        """
        Send a BluetoothMessage to a connected device.

        Messages are serialised as JSON and delimited with a newline character.

        Args:
            mac_address: Recipient's Bluetooth MAC address
            message: BluetoothMessage instance to send

        Returns:
            True if sent successfully, False otherwise.
        """
        try:
            with self.lock:
                if mac_address not in self.active_connections:
                    logger.warning(f"Not connected to {mac_address}")
                    return False

                sock, _ = self.active_connections[mac_address]
                payload = (message.to_json() + "\n").encode("utf-8")
                sock.sendall(payload)
                logger.debug(f"Sent message {message.message_id} to {mac_address}")
                return True

        except Exception as e:
            logger.error(f"Failed to send message to {mac_address}: {e}")
            self.disconnect_from_device(mac_address)
            return False

    def get_messages(
        self, mac_address: str | None = None
    ) -> dict[str, list[BluetoothMessage]]:
        """
        Drain and return received messages.

        Args:
            mac_address: If given, return messages only from that device.
                         Otherwise return all queued messages.

        Returns:
            Dict mapping MAC address -> list of BluetoothMessage objects.
            The returned messages are removed from the internal queue.
        """
        with self.lock:
            if mac_address:
                messages = self.message_queue.pop(mac_address, [])
                return {mac_address: messages}
            result = dict(self.message_queue)
            self.message_queue.clear()
            return result

    # ------------------------------------------------------------------ #
    #  Paired device management                                            #
    # ------------------------------------------------------------------ #

    def add_paired_device(
        self,
        name: str,
        mac_address: str,
        hostname: str | None = None,
        ip_address: str | None = None,
    ) -> PairedDevice:
        """
        Register a known/paired device.

        Args:
            name: Human-readable label for the robot
            mac_address: Bluetooth MAC address
            hostname: Optional hostname override
            ip_address: Optional IP address

        Returns:
            The created PairedDevice object.
        """
        with self.lock:
            device = PairedDevice(
                name=name,
                mac_address=mac_address,
                hostname=hostname or self._derive_hostname(mac_address),
                ip_address=ip_address,
                device_id=name,
            )
            self.paired_devices[mac_address] = device
            logger.info(f"Added paired device: {name} ({mac_address})")
            return device

    def remove_paired_device(self, mac_address: str) -> bool:
        """
        Remove a paired device (disconnects first if connected).

        Returns:
            True if the device was found and removed, False otherwise.
        """
        with self.lock:
            self._close_connection(mac_address)
            if mac_address in self.paired_devices:
                del self.paired_devices[mac_address]
                logger.info(f"Removed paired device: {mac_address}")
                return True
        return False

    def get_paired_devices(self) -> list[PairedDevice]:
        """Return all paired devices (system paired devices)."""
        with self.lock:
            devices: dict[str, PairedDevice] = dict(self.paired_devices)

        system_paired = self._get_system_paired_devices()
        system_connected = self._get_system_connected_devices()

        for mac, name in system_paired.items():
            if mac in devices:
                device = devices[mac]
                if not device.name or device.name == "Unknown":
                    device.name = name
            else:
                devices[mac] = PairedDevice(
                    name=name,
                    mac_address=mac,
                    hostname=self._derive_hostname(mac),
                    device_id=name,
                )

        connected_macs = set(system_connected.keys())
        for mac in connected_macs:
            if mac in devices:
                devices[mac].is_connected = True
                if devices[mac].name in ("", "Unknown"):
                    devices[mac].name = system_connected.get(mac, devices[mac].name)
                continue

            connected_name = system_connected.get(mac, "Unknown")
            devices[mac] = PairedDevice(
                name=connected_name,
                mac_address=mac,
                hostname=self._derive_hostname(mac),
                is_connected=True,
                device_id=connected_name,
            )

        return sorted(devices.values(), key=lambda d: (d.name.lower(), d.mac_address))

    def get_paired_device(self, mac_address: str) -> PairedDevice | None:
        """Return the PairedDevice for a given MAC address, or None"""
        with self.lock:
            device = self.paired_devices.get(mac_address)

        if device is not None:
            return device

        system_paired = self._get_system_paired_devices()
        system_connected = self._get_system_connected_devices()

        if mac_address in system_paired:
            return PairedDevice(
                name=system_paired[mac_address],
                mac_address=mac_address,
                hostname=self._derive_hostname(mac_address),
                is_connected=mac_address in system_connected,
                device_id=system_paired[mac_address],
            )

        if mac_address in system_connected:
            connected_name = system_connected.get(mac_address, "Unknown")
            return PairedDevice(
                name=connected_name,
                mac_address=mac_address,
                hostname=self._derive_hostname(mac_address),
                is_connected=True,
                device_id=connected_name,
            )

        return None

    def get_connected_devices(self) -> list[PairedDevice]:
        """Return all currently connected devices (API-managed + system connected)."""
        system_connected = self._get_system_connected_devices()
        connected_macs = set(system_connected.keys())

        with self.lock:
            connected_macs.update(self.active_connections.keys())
            connected_macs.update(
                mac for mac, device in self.paired_devices.items() if device.is_connected
            )

        all_devices = self.get_paired_devices()
        connected = [device for device in all_devices if device.mac_address in connected_macs]

        for device in connected:
            device.is_connected = True
            if device.name in ("", "Unknown"):
                device.name = system_connected.get(device.mac_address, device.name)

        for mac in connected_macs:
            if any(device.mac_address == mac for device in connected):
                continue

            connected_name = system_connected.get(mac, "Unknown")
            connected.append(
                PairedDevice(
                    name=connected_name,
                    mac_address=mac,
                    hostname=self._derive_hostname(mac),
                    is_connected=True,
                    device_id=connected_name,
                )
            )

        return sorted(connected, key=lambda d: (d.name.lower(), d.mac_address))


    def get_device_info(self) -> dict:
        """Return identifying information about this device"""
        return {
            "device_id": self.device_id,
            "hostname": self.hostname,
            "ip_address": self.ip_address,
        }

    def list_pairable_devices(self, timeout_seconds: int = 6) -> list[dict]:
        """
        Scan for nearby discoverable Bluetooth devices.

        Args:
            timeout_seconds: Scan duration in seconds.

        Returns:
            List of discovered device objects with `name`, `mac_address`, and `is_paired`.
        """
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")

        try:
            scan_cmd = ["bluetoothctl", "--timeout", str(timeout_seconds), "scan", "on"]
            scan_result = subprocess.run(scan_cmd, capture_output=True, text=True, timeout=timeout_seconds + 3)

            # `bluetoothctl scan on` can still discover devices even if it exits non-zero on some systems.
            output = (scan_result.stdout or "") + "\n" + (scan_result.stderr or "")

            device_pattern = re.compile(r"Device\s+([0-9A-Fa-f:]{17})\s+(.+)$")
            discovered: dict[str, str] = {}

            for raw_line in output.splitlines():
                line = raw_line.strip()
                match = device_pattern.search(line)
                if not match:
                    continue

                mac_address = match.group(1).upper()
                name = match.group(2).strip()
                if not name:
                    name = "Unknown"
                discovered[mac_address] = name

            with self.lock:
                paired_macs = set(self.paired_devices.keys())

            devices = [
                {
                    "name": name,
                    "mac_address": mac,
                    "is_paired": mac in paired_macs,
                }
                for mac, name in discovered.items()
            ]

            devices.sort(key=lambda d: (d["name"].lower(), d["mac_address"]))
            return devices
        except Exception as e:
            logger.error(f"Failed to list pairable devices: {e}")
            return []

    def _get_system_paired_devices(self) -> dict[str, str]:
        """Query system paired devices via bluetoothctl."""
        try:
            result = subprocess.run(
                ["bluetoothctl", "devices", "Paired"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            output = (result.stdout or "") + "\n" + (result.stderr or "")

            device_pattern = re.compile(r"Device\s+([0-9A-Fa-f:]{17})\s+(.+)$")
            paired: dict[str, str] = {}

            for raw_line in output.splitlines():
                line = raw_line.strip()
                match = device_pattern.search(line)
                if not match:
                    continue

                mac = match.group(1).upper()
                name = match.group(2).strip() or "Unknown"
                paired[mac] = name

            return paired
        except Exception as e:
            logger.debug(f"Failed to query system paired devices: {e}")
            return {}

    def _get_system_connected_devices(self) -> dict[str, str]:
        """Query system connected devices via bluetoothctl (MAC -> device name)."""
        try:
            result = subprocess.run(
                ["bluetoothctl", "devices", "Connected"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            output = (result.stdout or "") + "\n" + (result.stderr or "")

            device_pattern = re.compile(r"Device\s+([0-9A-Fa-f:]{17})\s+(.+)$")
            connected: dict[str, str] = {}

            for raw_line in output.splitlines():
                line = raw_line.strip()
                match = device_pattern.search(line)
                if not match:
                    continue

                mac = match.group(1).upper()
                name = match.group(2).strip() or "Unknown"
                connected[mac] = name

            return connected
        except Exception as e:
            logger.debug(f"Failed to query system connected devices: {e}")
            return {}

    def _get_system_connected_macs(self) -> set[str]:
        """Backward-compatible helper returning connected MAC addresses."""
        return set(self._get_system_connected_devices().keys())

    def set_discoverable(self, duration_seconds: int | None = None) -> bool:
        """
        Make this device discoverable via Bluetooth.

        Uses hciconfig to set the device in discoverable mode.
        This requires root/sudo access on most Linux systems.

        Args:
            duration_seconds: Duration in seconds (None = indefinite, typically ~120s)

        Returns:
            True if the command succeeded, False otherwise.
        """
        try:
            if duration_seconds is not None:
                cmd = ["sudo", "hciconfig", "hci0", "piscan", str(duration_seconds)]
            else:
                cmd = ["sudo", "hciconfig", "hci0", "piscan"]
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            success = result.returncode == 0
            if success:
                logger.info(f"Device set to discoverable mode (duration: {duration_seconds}s)")
            else:
                logger.error(f"Failed to set discoverable: {result.stderr.decode()}")
            return success
        except Exception as e:
            logger.error(f"Failed to set discoverable: {e}")
            return False

    def set_not_discoverable(self) -> bool:
        """
        Make this device non-discoverable via Bluetooth.

        Uses hciconfig to disable discoverable mode.
        This requires root/sudo access on most Linux systems.

        Returns:
            True if the command succeeded, False otherwise.
        """
        try:
            cmd = ["hciconfig", "hci0", "noscan"]
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            success = result.returncode == 0
            if success:
                logger.info("Device set to non-discoverable mode")
            else:
                logger.error(f"Failed to set non-discoverable: {result.stderr.decode()}")
            return success
        except Exception as e:
            logger.error(f"Failed to set non-discoverable: {e}")
            return False

    # ------------------------------------------------------------------
    #  Internal helpers
    # ------------------------------------------------------------------

    def _register_connection(self, sock: socket.socket, mac_address: str) -> None:
        """Store a new socket and start its receive thread"""
        with self.lock:
            self._close_connection(mac_address)
            receive_thread = threading.Thread(
                target=self._receive_loop,
                args=(sock, mac_address),
                daemon=True,
            )
            self.active_connections[mac_address] = (sock, receive_thread)
            receive_thread.start()

    def _close_connection(self, mac_address: str) -> bool:
        """Close and remove an active connection (must be called with self.lock held)"""
        if mac_address in self.active_connections:
            sock, _ = self.active_connections.pop(mac_address)
            try:
                sock.close()
            except Exception:
                pass
            return True
        return False

    def _receive_loop(self, sock: socket.socket, mac_address: str) -> None:
        """Read newline-delimited JSON messages from a socket until it closes"""
        buffer = ""
        try:
            while self.running and mac_address in self.active_connections:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="ignore")

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        message = BluetoothMessage.from_json(line)
                        if message:
                            self._dispatch_message(message, mac_address)

        except OSError as e:
            logger.debug(f"Receive error from {mac_address}: {e}")
        finally:
            self.disconnect_from_device(mac_address)

    def _dispatch_message(self, message: BluetoothMessage, mac_address: str) -> None:
        """Store a received message and invoke all matching handlers"""
        logger.debug(f"Received message {message.message_id} from {mac_address}")

        with self.lock:
            self.message_queue.setdefault(mac_address, []).append(message)
            handlers = list(self.message_handlers.get(message.message_type, []))

        for callback in handlers:
            try:
                callback(message, mac_address)
            except Exception as e:
                logger.error(f"Error in message handler: {e}")

    @staticmethod
    def _derive_hostname(mac_address: str) -> str:
        """Derive a default hostname from the last four hex digits of the MAC address"""
        return f"robot_{mac_address.replace(':', '')[-4:]}"
