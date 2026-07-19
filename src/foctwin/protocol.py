from __future__ import annotations

from dataclasses import dataclass

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

    def enable(self) -> str:
        return self.raw("E1")

    def disable(self) -> str:
        return self.raw("E0")

    def target(self, value: float) -> str:
        return self.raw(self._number(value))

    def motion_mode(self, mode: MotionMode) -> str:
        return self.raw(f"C{MOTION_CODES[mode]}")

    def torque_mode(self, mode: TorqueMode) -> str:
        return self.raw(f"T{TORQUE_CODES[mode]}")

    def velocity_limit(self, value: float) -> str:
        return self.raw(f"LV{self._number(value)}")

    def voltage_limit(self, value: float) -> str:
        return self.raw(f"LU{self._number(value)}")

    def current_limit(self, value: float) -> str:
        return self.raw(f"LC{self._number(value)}")

    def pid(self, loop: str, field: str, value: float | None = None) -> str:
        suffixes = {"p": "P", "i": "I", "d": "D", "ramp": "R", "limit": "L", "lpf": "F"}
        try:
            prefix = PID_CODES[loop]
            suffix = suffixes[field]
        except KeyError as exc:
            raise ValueError(f"Unknown PID selector: {loop}.{field}") from exc
        encoded = "" if value is None else self._number(value)
        return self.raw(f"{prefix}{suffix}{encoded}")

    def monitor_downsample(self, value: int) -> str:
        return self.raw(f"MD{int(value)}")

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


def parse_monitor_line(line: str) -> dict[str, float] | None:
    """Parse SimpleFOC's seven-column monitor stream; return None for command replies."""

    fields = line.strip().split("\t")
    if len(fields) != 7:
        return None
    try:
        values = [float(value) for value in fields]
    except ValueError:
        return None
    names = ("target", "voltage_q_v", "voltage_d_v", "current_q_a", "current_d_a", "velocity_rad_s", "angle_rad")
    return dict(zip(names, values, strict=True))

