from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any

from foctwin.domain import MotionMode, TorqueMode

MOTION_CODES = {
    MotionMode.TORQUE: 0,
    MotionMode.VELOCITY: 1,
    MotionMode.ANGLE: 2,
    MotionMode.VELOCITY_OPEN_LOOP: 3,
    MotionMode.ANGLE_OPEN_LOOP: 4,
}

TORQUE_CODES = {
    TorqueMode.VOLTAGE: 0,
    TorqueMode.DC_CURRENT: 1,
    TorqueMode.FOC_CURRENT: 2,
}

PID_CODES = {
    "velocity": "V",
    "angle": "A",
    "current_q": "Q",
    "current_d": "D",
}

MONITOR_FIELDS = (
    "target",
    "voltage_q_v",
    "voltage_d_v",
    "current_q_a",
    "current_d_a",
    "velocity_rad_s",
    "angle_rad",
)

_MONITOR_NUMBER = re.compile(r"[-+]?\d+\.\d{4}")


@dataclass(frozen=True, slots=True)
class CommanderResponse:
    key: str
    value: Any
    raw: str


@dataclass(frozen=True, slots=True)
class CommanderProtocol:
    """Encode the SimpleFOC Commander grammar used by the azimuth firmware."""

    device_id: str = "A"

    def __post_init__(self) -> None:
        if len(self.device_id) != 1 or not self.device_id.isascii():
            raise ValueError("Commander device ID must be one ASCII character")

    @staticmethod
    def _number(value: float) -> str:
        return format(float(value), ".12g")

    def raw(self, suffix: str = "") -> str:
        return f"{self.device_id}{suffix}"

    def enable(self, enabled: bool | None = True) -> str:
        suffix = "E" if enabled is None else f"E{int(enabled)}"
        return self.raw(suffix)

    def disable(self) -> str:
        return self.raw("E0")

    def target(self, value: float) -> str:
        return self.raw(self._number(value))

    def motion_mode(self, mode: MotionMode | None = None) -> str:
        encoded = "" if mode is None else str(MOTION_CODES[mode])
        return self.raw(f"C{encoded}")

    def torque_mode(self, mode: TorqueMode | None = None) -> str:
        encoded = "" if mode is None else str(TORQUE_CODES[mode])
        return self.raw(f"T{encoded}")

    def velocity_limit(self, value: float | None = None) -> str:
        encoded = "" if value is None else self._number(value)
        return self.raw(f"LV{encoded}")

    def voltage_limit(self, value: float | None = None) -> str:
        encoded = "" if value is None else self._number(value)
        return self.raw(f"LU{encoded}")

    def current_limit(self, value: float | None = None) -> str:
        encoded = "" if value is None else self._number(value)
        return self.raw(f"LC{encoded}")

    def phase_resistance(self, value: float | None = None) -> str:
        """Read or write the resistance used by SimpleFOC Voltage torque mode."""

        encoded = "" if value is None else self._number(value)
        return self.raw(f"R{encoded}")

    def pid(self, loop: str, field: str, value: float | None = None) -> str:
        suffixes = {"p": "P", "i": "I", "d": "D", "ramp": "R", "limit": "L", "lpf": "F"}
        try:
            prefix = PID_CODES[loop]
            suffix = suffixes[field]
        except KeyError as exc:
            raise ValueError(f"Unknown PID selector: {loop}.{field}") from exc
        encoded = "" if value is None else self._number(value)
        return self.raw(f"{prefix}{suffix}{encoded}")

    def monitor_downsample(self, value: int | None = None) -> str:
        encoded = "" if value is None else str(int(value))
        return self.raw(f"MD{encoded}")

    def monitor_clear(self) -> str:
        return self.raw("MC")

    def monitor_variables(self, mask: str) -> str:
        if len(mask) != 7 or any(bit not in "01" for bit in mask):
            raise ValueError("Monitor mask must contain exactly seven 0/1 digits")
        return self.raw(f"MS{mask}")

    def query_monitor(self, index: int) -> str:
        if index not in range(7):
            raise ValueError("Monitor index must be between 0 and 6")
        return self.raw(f"MG{index}")

    def emergency_sequence(self) -> list[str]:
        # Target zero first is useful in velocity/torque mode. Repeating disable is deliberate:
        # Serial cannot provide a hard real-time guarantee, but duplicate writes improve the
        # probability that one complete line reaches the existing firmware.
        return [self.target(0.0), self.disable(), self.disable(), self.disable()]


def parse_monitor_line(line: str, mask: str = "1111111") -> dict[str, float] | None:
    """Parse an intact SimpleFOC monitor row and normalize streamed currents from mA to A.

    The bundled firmware prints every monitor value with four decimal places. Requiring that
    exact shape is intentional: USB CDC corruption observed on hardware can remove one symbol
    while leaving a syntactically valid but dangerously wrong number (``1.0000`` -> ``10000``).
    """

    if len(mask) != 7 or any(bit not in "01" for bit in mask):
        raise ValueError("Monitor mask must contain exactly seven 0/1 digits")
    names = [name for name, enabled in zip(MONITOR_FIELDS, mask, strict=True) if enabled == "1"]
    fields = line.strip().split("\t")
    if not names or len(fields) != len(names):
        return None
    if any(_MONITOR_NUMBER.fullmatch(value) is None for value in fields):
        return None
    try:
        values = [float(value) for value in fields]
    except ValueError:
        return None
    if any(not math.isfinite(value) for value in values):
        return None
    parsed = dict(zip(names, values, strict=True))
    for current_name in ("current_q_a", "current_d_a"):
        if current_name in parsed:
            parsed[current_name] /= 1000.0
    return parsed


def is_monitor_candidate(line: str, mask: str = "1111111") -> bool:
    """Return true when a line has the expected monitor field count but invalid contents."""

    if len(mask) != 7 or any(bit not in "01" for bit in mask):
        raise ValueError("Monitor mask must contain exactly seven 0/1 digits")
    expected_fields = mask.count("1")
    return expected_fields > 0 and len(line.strip().split("\t")) == expected_fields


def parse_commander_response(line: str) -> CommanderResponse | None:
    """Parse the user-friendly responses emitted by the bundled SimpleFOC firmware."""

    stripped = line.strip()
    number = r"([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"
    patterns = (
        (rf"^Limits\|\s*curr:\s*{number}$", "limit.current_a", float),
        (rf"^Limits\|\s*volt:\s*{number}$", "limit.voltage_v", float),
        (rf"^Limits\|\s*vel:\s*{number}$", "limit.velocity_rad_s", float),
        (rf"^Status:\s*{number}$", "enabled", lambda value: bool(int(float(value)))),
        (r"^Motion:\s*(.+)$", "motion_mode", str),
        (r"^Torque:\s*(.+)$", "torque_mode", str),
    )
    for pattern, key, converter in patterns:
        match = re.match(pattern, stripped, flags=re.IGNORECASE)
        if match:
            return CommanderResponse(key, converter(match.group(1).strip()), stripped)

    pid_match = re.match(
        rf"^PID\s+(vel|angle|curr\s+q|curr\s+d)\|\s*"
        rf"(P|I|D|ramp|limit|Tf):\s*{number}$",
        stripped,
        flags=re.IGNORECASE,
    )
    if pid_match:
        loop_names = {
            "vel": "velocity",
            "angle": "angle",
            "curr q": "current_q",
            "curr d": "current_d",
        }
        field_names = {
            "p": "p",
            "i": "i",
            "d": "d",
            "ramp": "ramp",
            "limit": "limit",
            "tf": "lpf",
        }
        loop = loop_names[" ".join(pid_match.group(1).lower().split())]
        field = field_names[pid_match.group(2).lower()]
        return CommanderResponse(f"pid.{loop}.{field}", float(pid_match.group(3)), stripped)
    return None
