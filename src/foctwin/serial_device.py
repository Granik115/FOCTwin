from __future__ import annotations

import threading
import time
from collections.abc import Callable

from foctwin.protocol import CommanderProtocol

try:
    import serial
except ImportError:  # pragma: no cover - permits core tests without hardware dependencies
    serial = None


LineCallback = Callable[[float, str], None]
StateCallback = Callable[[bool, str], None]


class SerialLineFramer:
    """Accumulate USB CDC fragments and emit only complete newline-terminated records."""

    def __init__(self, max_pending_bytes: int = 65536) -> None:
        self.max_pending_bytes = max_pending_bytes
        self._pending = bytearray()
        self.overflow_count = 0

    def feed(self, chunk: bytes) -> list[str]:
        if not chunk:
            return []
        self._pending.extend(chunk)
        lines: list[str] = []
        while True:
            newline = self._pending.find(b"\n")
            if newline < 0:
                break
            raw = bytes(self._pending[:newline])
            del self._pending[: newline + 1]
            line = raw.rstrip(b"\r").decode("utf-8", errors="replace")
            if line:
                lines.append(line)
        if len(self._pending) > self.max_pending_bytes:
            self._pending.clear()
            self.overflow_count += 1
        return lines


class SerialDevice:
    """Threaded, UI-independent transport for the existing USB CDC firmware."""

    def __init__(self, protocol: CommanderProtocol | None = None) -> None:
        self.protocol = protocol or CommanderProtocol()
        self._port = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self.on_line: LineCallback = lambda _timestamp, _line: None
        self.on_state: StateCallback = lambda _connected, _message: None

    @staticmethod
    def available_ports() -> list[tuple[str, str]]:
        if serial is None:
            return []
        from serial.tools import list_ports

        return [(port.device, port.description) for port in list_ports.comports()]

    @property
    def connected(self) -> bool:
        return bool(self._port and self._port.is_open)

    def connect(self, port: str, baudrate: int = 115200, timeout: float = 0.2) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.disconnect()
        self._port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, name="foctwin-serial", daemon=True)
        self._reader.start()
        self.on_state(True, f"Подключено: {port} @ {baudrate}")

    def disconnect(self) -> None:
        self._stop.set()
        port, self._port = self._port, None
        if port is not None:
            try:
                if port.is_open:
                    port.close()
            finally:
                self.on_state(False, "Отключено")
        reader, self._reader = self._reader, None
        if reader and reader is not threading.current_thread():
            reader.join(timeout=0.5)

    def send(self, command: str) -> None:
        if not self.connected:
            raise RuntimeError("Serial device is not connected")
        payload = f"{command.rstrip()}\n".encode("utf-8")
        with self._write_lock:
            self._port.write(payload)
            self._port.flush()

    def set_dtr(self, enabled: bool) -> None:
        """Change only the USB CDC DTR state without reopening or resetting the board."""

        if not self.connected:
            raise RuntimeError("Serial device is not connected")
        with self._write_lock:
            self._port.setDTR(bool(enabled))

    def discard_pending_input(self) -> None:
        """Drop an incomplete/stale host-side USB fragment before stream recovery."""

        if not self.connected:
            raise RuntimeError("Serial device is not connected")
        self._port.reset_input_buffer()

    def emergency_stop(self) -> list[str]:
        sent: list[str] = []
        for command in self.protocol.emergency_sequence():
            try:
                self.send(command)
                sent.append(command)
                time.sleep(0.015)
            except Exception:
                break
        return sent

    def _read_loop(self) -> None:
        framer = SerialLineFramer()
        try:
            while not self._stop.is_set() and self.connected:
                waiting = int(getattr(self._port, "in_waiting", 0) or 0)
                raw = self._port.read(max(1, min(waiting, 4096)))
                received_at = time.monotonic()
                for line in framer.feed(raw):
                    self.on_line(received_at, line)
        except Exception as exc:
            self.on_state(False, f"Ошибка Serial: {exc}")
        finally:
            if not self._stop.is_set():
                self.disconnect()
