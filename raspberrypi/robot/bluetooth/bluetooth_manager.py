from __future__ import annotations

import json
import logging
from multiprocessing.managers import DictProxy
import queue
import re
import socket
import subprocess
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass, field

from numpy import split

logger = logging.getLogger("BluetoothManager")

_AF_BLUETOOTH: int = getattr(socket, "AF_BLUETOOTH")
_BTPROTO_RFCOMM: int = getattr(socket, "BTPROTO_RFCOMM")

RFCOMM_CHANNEL = 1

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
    def from_json(cls, raw: str) -> "BluetoothMessage | None":
        try:
            return cls(**json.loads(raw))
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

    def get_device_info(self) -> dict:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
            s.close()
        except Exception:
            ip_address = "127.0.0.1"

        output = self._run_btctl("list")

        mac_address = output.split()[1]
        hostname = output.split()[2]

        return {
            "mac_address": mac_address,
            "hostname": hostname,
            "ip_address": ip_address,
        }

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

                mac = addr[0]

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
            sock = socket.socket(
                _AF_BLUETOOTH,
                socket.SOCK_STREAM,
                _BTPROTO_RFCOMM,
            )

            sock.settimeout(10)

            sock.connect((mac_address, RFCOMM_CHANNEL))

            self._add_connection(sock, mac_address)

            logger.info(f"Connected to {mac_address}")

            return True

        except Exception as e:
            logger.error(f"Connect failed: {e}")
            return False

    def disconnect(self, mac_address: str) -> None:
        conn = self.connections.get(mac_address)

        if conn:
            conn.close()

    def _add_connection(
        self,
        sock: socket.socket,
        mac_address: str,
    ) -> None:
        self.disconnect(mac_address)

        self.connections[mac_address] = RFCOMMConnection(
            sock=sock,
            mac_address=mac_address,
            on_message=self._handle_message,
            on_disconnect=self._handle_disconnect,
        )

    def _handle_disconnect(self, mac_address: str) -> None:
        self.connections.pop(mac_address, None)

    # --------------------------------------------------------
    # Messaging
    # --------------------------------------------------------

    def send(
        self,
        mac_address: str,
        message: BluetoothMessage,
    ) -> bool:
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
