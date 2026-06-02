from __future__ import annotations

import errno
import json
import logging
import queue
import re
import socket
import select
import subprocess
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass, field


logger = logging.getLogger("BluetoothManager")

_AF_BLUETOOTH: int = getattr(socket, "AF_BLUETOOTH")
_BTPROTO_RFCOMM: int = getattr(socket, "BTPROTO_RFCOMM")

RFCOMM_CHANNEL = 1
_CONNECT_TIMEOUT_S = 6.0

DEVICE_REGEX = re.compile(r"Device\s+([0-9A-F:]{17})\s+(.+)$")


# ============================================================
# Messages
# ============================================================

@dataclass(slots=True)
class BluetoothMessage:
    message_type: str
    content: str
    sender_id: str = field(default_factory=socket.gethostname)
    timestamp: float = field(default_factory=time.time)
    message_id: str = field(default_factory=lambda: str(uuid.uuid4()))

    def to_dict(self) -> dict:
        return {
            "message_type": self.message_type,
            "content": self.content,
            "sender_id": self.sender_id,
            "timestamp": self.timestamp,
            "message_id": self.message_id,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls, raw: str) -> BluetoothMessage | None:
        try:
            return cls.from_dict(json.loads(raw))
        except Exception as e:
            logger.debug(f"Invalid message: {e}")
            return None

    @classmethod
    def from_dict(cls, raw: dict) -> BluetoothMessage | None:
        try:
            message_type = raw.get("message_type")
            content = raw.get("content")
            if not isinstance(message_type, str) or not message_type.strip():
                raise ValueError("message_type must be a non-empty string")
            if content is None:
                raise ValueError("content is required")

            sender_id = raw.get("sender_id")
            if not isinstance(sender_id, str):
                sender_id = ""

            timestamp_raw = raw.get("timestamp", time.time())
            timestamp = float(timestamp_raw)

            message_id = raw.get("message_id")
            if not isinstance(message_id, str) or not message_id:
                message_id = str(uuid.uuid4())

            return cls(
                message_type=message_type,
                content=str(content),
                sender_id=sender_id,
                timestamp=timestamp,
                message_id=message_id,
            )
        except Exception as e:
            logger.debug(f"Invalid message: {e}")
            return None


# ============================================================
# Device model
# ============================================================

@dataclass(slots=True)
class PairedDevice:
    mac_address: str
    name: str = "Unknown"
    connected: bool = False
    last_seen: float | None = None

    def to_dict(self) -> dict:
        return {
            "mac_address": self.mac_address,
            "name": self.name,
            "connected": self.connected,
            "last_seen": self.last_seen,
        }

    @classmethod
    def from_dict(cls, raw: dict) -> PairedDevice | None:
        try:
            mac_address = raw.get("mac_address")
            if not isinstance(mac_address, str) or not mac_address.strip():
                raise ValueError("mac_address is required")
            return cls(
                mac_address=mac_address.strip().upper(),
                name=str(raw.get("name", "Unknown")),
                connected=bool(raw.get("connected", False)),
                last_seen=float(raw["last_seen"]) if raw.get("last_seen") is not None else None,
            )
        except Exception as e:
            logger.debug(f"Invalid paired device: {e}", exc_info=True)
            return None


@dataclass(slots=True)
class BluetoothDeviceInfo:
    mac_address: str
    hostname: str
    ip_address: str

    def to_dict(self) -> dict:
        return {
            "mac_address": self.mac_address,
            "hostname": self.hostname,
            "ip_address": self.ip_address,
        }

    @classmethod
    def from_dict(cls, raw: dict | None) -> BluetoothDeviceInfo | None:
        try:
            if not isinstance(raw, dict):
                raise TypeError("raw must be a dict")
            mac_address = raw.get("mac_address")
            hostname = raw.get("hostname")
            ip_address = raw.get("ip_address")
            if not isinstance(mac_address, str) or not isinstance(hostname, str) or not isinstance(ip_address, str):
                raise ValueError("mac_address, hostname and ip_address must be strings")
            return cls(mac_address=mac_address, hostname=hostname, ip_address=ip_address)
        except Exception as e:
            logger.debug(f"Invalid device info: {e}", exc_info=True)
            return None


@dataclass(slots=True)
class OtherRobotInfo:
    mac_address: str
    name: str | None = None
    hostname: str | None = None
    ip_address: str | None = None
    note: str | None = None

    def to_dict(self) -> dict:
        data = {"mac_address": self.mac_address}
        if self.name is not None:
            data["name"] = self.name
        if self.hostname is not None:
            data["hostname"] = self.hostname
        if self.ip_address is not None:
            data["ip_address"] = self.ip_address
        if self.note is not None:
            data["note"] = self.note
        return data

    @classmethod
    def from_dict(cls, raw: dict | None) -> OtherRobotInfo | None:
        try:
            if not isinstance(raw, dict):
                raise TypeError("raw must be a dict")
            mac_address = raw.get("mac_address")
            if not isinstance(mac_address, str) or not mac_address.strip():
                raise ValueError("mac_address is required")
            return cls(
                mac_address=mac_address.strip().upper(),
                name=raw.get("name") if isinstance(raw.get("name"), str) else None,
                hostname=raw.get("hostname") if isinstance(raw.get("hostname"), str) else None,
                ip_address=raw.get("ip_address") if isinstance(raw.get("ip_address"), str) else None,
                note=raw.get("note") if isinstance(raw.get("note"), str) else None,
            )
        except Exception:
            return None


@dataclass(slots=True)
class BluetoothReceivedMessage:
    message: BluetoothMessage
    sender_mac: str

    def to_dict(self) -> dict:
        data = self.message.to_dict()
        data["sender_mac"] = self.sender_mac
        return data

    @classmethod
    def from_dict(cls, raw: dict | None) -> BluetoothReceivedMessage | None:
        if not isinstance(raw, dict):
            return None
        message = BluetoothMessage.from_dict(raw)
        sender_mac = raw.get("sender_mac")
        if message is None or not isinstance(sender_mac, str) or not sender_mac.strip():
            return None
        return cls(message=message, sender_mac=sender_mac.strip().upper())


@dataclass(slots=True)
class BluetoothSentMessage:
    message: BluetoothMessage
    target_mac: str

    def to_dict(self) -> dict:
        data = self.message.to_dict()
        data["target_mac"] = self.target_mac
        return data

    @classmethod
    def from_dict(cls, raw: dict | None) -> BluetoothSentMessage | None:
        if not isinstance(raw, dict):
            return None
        message = BluetoothMessage.from_dict(raw)
        target_mac = raw.get("target_mac")
        if message is None or not isinstance(target_mac, str) or not target_mac.strip():
            return None
        return cls(message=message, target_mac=target_mac.strip().upper())


@dataclass(slots=True)
class BluetoothCommandResult:
    command_id: int
    success: bool
    data: dict = field(default_factory=dict)
    error: str | None = None
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "command_id": self.command_id,
            "success": self.success,
            "data": self.data,
            "error": self.error,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, raw: dict) -> BluetoothCommandResult | None:
        try:
            command_id = int(raw["command_id"])
            success = bool(raw.get("success", False))
            data = raw.get("data")
            if not isinstance(data, dict):
                data = {}
            error = raw.get("error")
            if error is not None and not isinstance(error, str):
                error = str(error)
            timestamp = float(raw.get("timestamp", time.time()))
            return cls(
                command_id=command_id,
                success=success,
                data=data,
                error=error,
                timestamp=timestamp,
            )
        except Exception:
            return None


# ============================================================
# RFCOMM connection wrapper
# ============================================================

class RFCOMMConnection:
    def __init__(
        self,
        sock: socket.socket,
        mac_address: str,
        on_message: Callable[[str, BluetoothMessage], None],
        on_disconnect: Callable[[str], None],
    ):
        self.sock = sock
        self.mac_address = mac_address
        self.on_message = on_message
        self.on_disconnect = on_disconnect

        self.running = True

        self.sock.settimeout(1.0)

        self.thread = threading.Thread(
            target=self._recv_loop,
            daemon=True,
        )
        self.thread.start()

    def send(self, message: BluetoothMessage) -> bool:
        try:
            payload = (message.to_json() + "\n").encode()
            self.sock.sendall(payload)
            return True
        except Exception as e:
            logger.debug(f"Send failed ({self.mac_address}): {e}")
            self.close()
            return False

    def close(self) -> None:
        if not self.running:
            return

        self.running = False

        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass

        try:
            self.sock.close()
        except Exception:
            pass

        self.on_disconnect(self.mac_address)

    def _recv_loop(self) -> None:
        buffer = ""

        try:
            while self.running:
                try:
                    data = self.sock.recv(4096)

                    if not data:
                        break

                    buffer += data.decode("utf-8", errors="ignore")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)

                        line = line.strip()

                        if not line:
                            continue

                        message = BluetoothMessage.from_json(line)

                        if message:
                            self.on_message(self.mac_address, message)

                except socket.timeout:
                    continue

        except Exception as e:
            logger.debug(f"Receive loop error ({self.mac_address}): {e}")

        finally:
            self.close()


# ============================================================
# Bluetooth manager
# ============================================================

class BluetoothManager:
    def __init__(self):
        self.hostname = socket.gethostname()

        self.server_socket: socket.socket | None = None
        self.server_thread: threading.Thread | None = None

        self.running = False

        self.lock = threading.RLock()

        self.connections: dict[str, RFCOMMConnection] = {}

        self.message_handlers: dict[str, list[Callable]] = {}

        self.message_queue: queue.Queue[tuple[str, BluetoothMessage]] = queue.Queue()

    @staticmethod
    def _normalize_mac(mac_address: str) -> str:
        return mac_address.strip().upper()

    # --------------------------------------------------------
    # bluetoothctl helpers
    # --------------------------------------------------------

    @staticmethod
    def _run_btctl(*args: str, timeout: int = 10) -> str:
        result = subprocess.run(
            ["bluetoothctl", *args],
            capture_output=True,
            text=True,
            timeout=timeout,
        )

        return (result.stdout or "") + "\n" + (result.stderr or "")

    @staticmethod
    def _parse_devices(output: str) -> dict[str, str]:
        devices = {}

        for line in output.splitlines():
            match = DEVICE_REGEX.search(line.strip())

            if not match:
                continue

            mac = match.group(1).upper()
            name = match.group(2).strip() or "Unknown"

            devices[mac] = name

        return devices

    @staticmethod
    def _connect_with_timeout(
        sock: socket.socket,
        address: tuple[str, int],
        timeout_s: float,
    ) -> None:
        sock.setblocking(False)

        err = sock.connect_ex(address)
        if err in (0, errno.EISCONN):
            sock.setblocking(True)
            return

        if err not in (errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EALREADY):
            raise OSError(err, f"Bluetooth connect failed: {err}")

        _, writable, _ = select.select([], [sock], [], timeout_s)
        if not writable:
            raise TimeoutError("Bluetooth connect timed out")

        err = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err != 0:
            raise OSError(err, f"Bluetooth connect failed: {err}")

        sock.setblocking(True)

    # --------------------------------------------------------
    # Pairing
    # --------------------------------------------------------

    def pair(self, mac_address: str) -> bool:
        try:
            self._run_btctl("pair", "--agent", "NoInputNoOutput", mac_address)
            self._run_btctl("trust", mac_address)
            self._run_btctl("connect", mac_address)
            return True
        except Exception as e:
            logger.error(f"Pair failed: {e}")
            return False

    def remove_pairing(self, mac_address: str) -> bool:
        try:
            self._run_btctl("remove", mac_address)
            return True
        except Exception as e:
            logger.error(f"Remove pairing failed: {e}")
            return False

    def list_paired_devices(self) -> list[PairedDevice]:
        output = self._run_btctl("devices", "Paired")

        connected = set(
            self._parse_devices(
                self._run_btctl("devices", "Connected")
            ).keys()
        )

        return [
            PairedDevice(
                mac_address=mac,
                name=name,
                connected=mac in connected,
            )
            for mac, name in self._parse_devices(output).items()
        ]

    def set_pairing_mode(self, enabled: bool = True) -> bool:
        try:
            if enabled:
                self._run_btctl("power", "on")
                self._run_btctl("discoverable", "on")
                self._run_btctl("pairable", "on")
            else:
                self._run_btctl("discoverable", "off")
                self._run_btctl("pairable", "off")
            return True
        except Exception as e:
            logger.error(f"Set pairing mode failed: {e}")
            return False



    def scan(self, duration: int = 5) -> list[PairedDevice]:
        output = self._run_btctl(
            "--timeout",
            str(duration),
            "scan",
            "on",
            timeout=duration + 3,
        )

        return [
            PairedDevice(mac_address=mac, name=name)
            for mac, name in self._parse_devices(output).items()
        ]

    # --------------------------------------------------------
    # Server
    # --------------------------------------------------------

    def get_device_info(self) -> BluetoothDeviceInfo:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
            s.close()
        except Exception:
            ip_address = "127.0.0.1"

        output = self._run_btctl("list")
        parts = output.split()
        mac_address = parts[1] if len(parts) > 1 else "00:00:00:00:00:00"
        hostname = parts[2] if len(parts) > 2 else self.hostname

        return BluetoothDeviceInfo(
            mac_address=mac_address,
            hostname=hostname,
            ip_address=ip_address,
        )

    def start_server(self) -> bool:
        if self.running:
            return True

        try:
            self.running = True

            self.server_socket = socket.socket(
                _AF_BLUETOOTH,
                socket.SOCK_STREAM,
                _BTPROTO_RFCOMM,
            )

            self.server_socket.settimeout(1.0)

            self.server_socket.bind(
                ("00:00:00:00:00:00", RFCOMM_CHANNEL)
            )

            self.server_socket.listen(5)

            self.server_thread = threading.Thread(
                target=self._accept_loop,
                daemon=True,
            )

            self.server_thread.start()

            self._run_btctl("agent", "NoInputNoOutput")
            self._run_btctl("default-agent")

            return True

        except Exception as e:
            logger.error(f"Failed to start server: {e}")
            self.running = False
            return False

    def stop_server(self) -> None:
        self.running = False

        for conn in list(self.connections.values()):
            conn.close()

        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass

    def _accept_loop(self) -> None:
        while self.running and self.server_socket is not None:
            try:
                sock, addr = self.server_socket.accept()

                mac = self._normalize_mac(addr[0])

                logger.info(f"Incoming connection from {mac}")

                self._add_connection(sock, mac)

            except socket.timeout:
                continue

            except Exception as e:
                if self.running:
                    logger.debug(f"Accept error: {e}")

    # --------------------------------------------------------
    # Connections
    # --------------------------------------------------------

    def connect(self, mac_address: str) -> bool:
        try:
            if not isinstance(mac_address, str) or not mac_address.strip():
                logger.error("Connect failed: mac_address is required")
                return False
            mac_address = self._normalize_mac(mac_address)
            if self.is_connected(mac_address):
                return True

            sock = socket.socket(
                _AF_BLUETOOTH,
                socket.SOCK_STREAM,
                _BTPROTO_RFCOMM,
            )

            self._connect_with_timeout(sock, (mac_address, RFCOMM_CHANNEL), _CONNECT_TIMEOUT_S)

            self._add_connection(sock, mac_address)

            logger.info(f"Connected to {mac_address}")

            return True

        except Exception as e:
            logger.error(f"Connect failed: {e}")
            return False

    def is_connected(self, mac_address: str) -> bool:
        if not isinstance(mac_address, str) or not mac_address.strip():
            return False
        mac_address = self._normalize_mac(mac_address)
        with self.lock:
            return mac_address in self.connections

    def disconnect(self, mac_address: str) -> None:
        if not isinstance(mac_address, str) or not mac_address.strip():
            return
        mac_address = self._normalize_mac(mac_address)
        with self.lock:
            conn = self.connections.get(mac_address)

        if conn:
            conn.close()

    def _add_connection(
        self,
        sock: socket.socket,
        mac_address: str,
    ) -> None:
        mac_address = self._normalize_mac(mac_address)
        self.disconnect(mac_address)

        with self.lock:
            self.connections[mac_address] = RFCOMMConnection(
                sock=sock,
                mac_address=mac_address,
                on_message=self._handle_message,
                on_disconnect=self._handle_disconnect,
            )

    def _handle_disconnect(self, mac_address: str) -> None:
        if not isinstance(mac_address, str) or not mac_address.strip():
            return
        mac_address = self._normalize_mac(mac_address)
        with self.lock:
            self.connections.pop(mac_address, None)

    # --------------------------------------------------------
    # Messaging
    # --------------------------------------------------------

    def send(
        self,
        mac_address: str,
        message: BluetoothMessage,
    ) -> bool:
        if not isinstance(mac_address, str) or not mac_address.strip():
            return False
        mac_address = self._normalize_mac(mac_address)
        with self.lock:
            conn = self.connections.get(mac_address)

        if not conn:
            return False

        return conn.send(message)

    def register_handler(
        self,
        message_type: str,
        callback: Callable[[BluetoothMessage, str], None],
    ) -> None:
        self.message_handlers.setdefault(
            message_type,
            []
        ).append(callback)

    def _handle_message(
        self,
        mac_address: str,
        message: BluetoothMessage,
    ) -> None:
        self.message_queue.put((mac_address, message))

        handlers = self.message_handlers.get(
            message.message_type,
            [],
        )

        for callback in handlers:
            try:
                callback(message, mac_address)
            except Exception as e:
                logger.error(f"Handler error: {e}")

    def get_messages(self) -> list[tuple[str, BluetoothMessage]]:
        messages = []

        while True:
            try:
                messages.append(
                    self.message_queue.get_nowait()
                )
            except queue.Empty:
                break

        return messages
