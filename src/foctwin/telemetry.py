from __future__ import annotations

import csv
import math
import queue
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

from foctwin.domain import TelemetrySample


@dataclass(slots=True)
class TelemetryStatistics:
    sample_count: int = 0
    _last_timestamp_s: float | None = None
    _interval_count: int = 0
    _interval_mean_s: float = 0.0
    _interval_m2_s2: float = 0.0
    minimum_interval_s: float | None = None
    maximum_interval_s: float | None = None

    def add(self, timestamp_s: float) -> None:
        self.sample_count += 1
        if self._last_timestamp_s is not None:
            interval = timestamp_s - self._last_timestamp_s
            if interval > 0:
                self._interval_count += 1
                delta = interval - self._interval_mean_s
                self._interval_mean_s += delta / self._interval_count
                self._interval_m2_s2 += delta * (interval - self._interval_mean_s)
                self.minimum_interval_s = (
                    interval
                    if self.minimum_interval_s is None
                    else min(self.minimum_interval_s, interval)
                )
                self.maximum_interval_s = (
                    interval
                    if self.maximum_interval_s is None
                    else max(self.maximum_interval_s, interval)
                )
        self._last_timestamp_s = timestamp_s

    @property
    def frequency_hz(self) -> float:
        if self._interval_mean_s <= 0:
            return 0.0
        return 1.0 / self._interval_mean_s

    @property
    def jitter_s(self) -> float:
        if self._interval_count < 2:
            return 0.0
        return math.sqrt(self._interval_m2_s2 / (self._interval_count - 1))

    def reset(self) -> None:
        self.sample_count = 0
        self._last_timestamp_s = None
        self._interval_count = 0
        self._interval_mean_s = 0.0
        self._interval_m2_s2 = 0.0
        self.minimum_interval_s = None
        self.maximum_interval_s = None


class TelemetryRecorder:
    FIELDS = (
        "sequence",
        "received_at_utc",
        "timestamp_s",
        "target",
        "voltage_q_v",
        "voltage_d_v",
        "current_q_a",
        "current_d_a",
        "velocity_rad_s",
        "angle_rad",
        "raw",
    )

    def __init__(self) -> None:
        self.path: Path | None = None
        self._handle: TextIO | None = None
        self._writer: csv.DictWriter | None = None
        self._queue: queue.Queue[TelemetrySample | None] | None = None
        self._thread: threading.Thread | None = None
        self._active = False
        self.last_error: str | None = None

    @property
    def active(self) -> bool:
        return self._active

    def start(self, path: str | Path) -> Path:
        self.stop()
        self.path = Path(path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self._handle.flush()
        self._queue = queue.Queue()
        self._active = True
        self.last_error = None
        self._thread = threading.Thread(
            target=self._write_loop,
            args=(self._queue, self._writer, self._handle),
            name="foctwin-telemetry-writer",
            daemon=True,
        )
        self._thread.start()
        return self.path

    def append(self, sample: TelemetrySample) -> None:
        if not self._active or self._queue is None:
            return
        self._queue.put(sample)

    def _write_loop(
        self,
        pending: queue.Queue[TelemetrySample | None],
        writer: csv.DictWriter,
        handle: TextIO,
    ) -> None:
        try:
            while True:
                sample = pending.get()
                if sample is None:
                    break
                writer.writerow({field: getattr(sample, field) for field in self.FIELDS})
                # Keep completed rows durable without making the Qt event loop wait for disk I/O.
                handle.flush()
        except Exception as exc:  # noqa: BLE001  # pragma: no cover - filesystem failures
            self.last_error = str(exc)
        finally:
            try:
                handle.flush()
                handle.close()
            except OSError as exc:  # pragma: no cover - depends on filesystem failures
                self.last_error = self.last_error or str(exc)
            if self._handle is handle:
                self._handle = None
                self._writer = None
                self._active = False

    def stop(self) -> Path | None:
        path = self.path
        thread, pending = self._thread, self._queue
        if thread is not None and thread.is_alive() and pending is not None:
            pending.put(None)
            thread.join(timeout=5.0)
            if thread.is_alive():  # pragma: no cover - local CSV writes should complete promptly
                self.last_error = "Таймаут завершения записи телеметрии"
        self._thread = None
        self._queue = None
        self._active = False
        return path


def monitor_stale_timeout(downsample: int) -> float:
    """Return a tolerant stream timeout for the firmware's nominal 1 kHz motor loop."""

    if downsample < 1:
        raise ValueError("Monitor downsample must be positive")
    expected_interval_s = downsample / 1000.0
    return max(2.0, expected_interval_s * 3.0 + 0.5)
