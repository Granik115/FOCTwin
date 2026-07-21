from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


SAFETY_SOFT_LIMIT_SAMPLES = 3
SAFETY_SOFT_LIMIT_TOLERANCE = 0.05
SAFETY_HARD_LIMIT_MULTIPLIER = 2.0


class MotionMode(str, Enum):
    TORQUE = "torque"
    VELOCITY = "velocity"
    ANGLE = "angle"
    VELOCITY_OPEN_LOOP = "velocity_open_loop"
    ANGLE_OPEN_LOOP = "angle_open_loop"


class TorqueMode(str, Enum):
    VOLTAGE = "voltage"
    DC_CURRENT = "dc_current"
    FOC_CURRENT = "foc_current"


@dataclass(slots=True)
class SafetyLimits:
    current_a: float = 1.0
    voltage_v: float = 12.0
    velocity_rad_s: float = 0.7
    angle_min_rad: float = -6.283185307179586
    angle_max_rad: float = 6.283185307179586
    trial_timeout_s: float = 10.0
    telemetry_timeout_s: float = 1.0

    def validate(self) -> None:
        if self.current_a <= 0 or self.voltage_v <= 0 or self.velocity_rad_s <= 0:
            raise ValueError("Ток, напряжение и скорость должны быть больше нуля")
        if self.angle_min_rad >= self.angle_max_rad:
            raise ValueError("Минимальная координата должна быть меньше максимальной")
        if self.trial_timeout_s <= 0 or self.telemetry_timeout_s <= 0:
            raise ValueError("Таймауты должны быть больше нуля")


@dataclass(slots=True)
class PIDParameters:
    p: float = 0.0
    i: float = 0.0
    d: float = 0.0
    output_ramp: float = 0.0
    output_limit: float = 0.0
    lpf_tf: float = 0.0
    anti_windup_kc: float = 0.8


@dataclass(slots=True)
class MotorProfile:
    name: str = "JCM115x25S / azimuth / baseline"
    model: str = "JCM115x25S"
    command_id: str = "A"
    pole_pairs: int = 15
    phase_resistance_ohm: float = 0.675
    ld_h: float = 0.0013
    lq_h: float = 0.0013
    back_emf_v_per_krpm: float = 92.6
    flux_linkage_wb: float = 0.05897
    torque_constant_nm_per_a: float = 1.3264
    rotor_inertia_kg_m2: float = 4.1e-4
    inertia_kg_m2: float = 0.07
    viscous_friction_nm_s_rad: float = 1e-5
    coulomb_friction_nm: float = 0.0
    breakaway_friction_nm: float = 0.0
    breakaway_velocity_rad_s: float = 0.01
    safety: SafetyLimits = field(default_factory=SafetyLimits)
    angle: PIDParameters = field(
        default_factory=lambda: PIDParameters(p=35.0, output_limit=0.7, anti_windup_kc=0.8)
    )
    velocity: PIDParameters = field(
        default_factory=lambda: PIDParameters(
            p=20.4, i=470.0, output_ramp=1000.0, output_limit=12.0, lpf_tf=0.01
        )
    )
    current_q: PIDParameters = field(
        default_factory=lambda: PIDParameters(
            p=8.4222, i=814.0, output_ramp=3000.0, output_limit=12.0, lpf_tf=0.005
        )
    )
    current_d: PIDParameters = field(
        default_factory=lambda: PIDParameters(
            p=8.4222, i=814.0, output_ramp=3000.0, output_limit=12.0, lpf_tf=0.005
        )
    )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class TelemetrySample:
    timestamp_s: float
    sequence: int = 0
    received_at_utc: str = ""
    target: float | None = None
    voltage_q_v: float | None = None
    voltage_d_v: float | None = None
    current_q_a: float | None = None
    current_d_a: float | None = None
    velocity_rad_s: float | None = None
    angle_rad: float | None = None
    raw: str = ""


@dataclass(slots=True)
class SafetyViolation:
    signal: str
    value: float
    limit: float
    message: str


class SafetyGuard:
    def __init__(self, limits: SafetyLimits):
        self.limits = limits
        self._soft_limit_counts: dict[str, int] = {}

    def check(self, sample: TelemetrySample) -> list[SafetyViolation]:
        violations: list[SafetyViolation] = []
        for signal, value, limit in (
            ("current_q_a", sample.current_q_a, self.limits.current_a),
            ("current_d_a", sample.current_d_a, self.limits.current_a),
            ("voltage_q_v", sample.voltage_q_v, self.limits.voltage_v),
            ("voltage_d_v", sample.voltage_d_v, self.limits.voltage_v),
            ("velocity_rad_s", sample.velocity_rad_s, self.limits.velocity_rad_s),
        ):
            if value is None:
                continue
            absolute = abs(value)
            if absolute > limit * SAFETY_HARD_LIMIT_MULTIPLIER:
                self._soft_limit_counts[signal] = 0
                violations.append(
                    SafetyViolation(
                        signal,
                        value,
                        limit * SAFETY_HARD_LIMIT_MULTIPLIER,
                        f"{signal}: резкий выброс {value:g} > "
                        f"±{limit * SAFETY_HARD_LIMIT_MULTIPLIER:g}",
                    )
                )
            elif absolute > limit * (1.0 + SAFETY_SOFT_LIMIT_TOLERANCE):
                count = self._soft_limit_counts.get(signal, 0) + 1
                self._soft_limit_counts[signal] = count
                if count >= SAFETY_SOFT_LIMIT_SAMPLES:
                    violations.append(
                        SafetyViolation(
                            signal,
                            value,
                            limit,
                            f"{signal}: {value:g} устойчиво выше ±{limit:g} "
                            f"({count} отсчёта подряд)",
                        )
                    )
            else:
                self._soft_limit_counts[signal] = 0
        if sample.angle_rad is not None:
            if sample.angle_rad < self.limits.angle_min_rad:
                violations.append(
                    SafetyViolation(
                        "angle_rad",
                        sample.angle_rad,
                        self.limits.angle_min_rad,
                        "Координата ниже разрешённого минимума",
                    )
                )
            if sample.angle_rad > self.limits.angle_max_rad:
                violations.append(
                    SafetyViolation(
                        "angle_rad",
                        sample.angle_rad,
                        self.limits.angle_max_rad,
                        "Координата выше разрешённого максимума",
                    )
                )
        return violations
