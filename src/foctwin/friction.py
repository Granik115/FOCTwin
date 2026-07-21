from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Iterable

import numpy as np

from foctwin.domain import TelemetrySample


FRICTION_MONITOR_MASK = "1111111"
FRICTION_MIN_TARGET_VELOCITY_RAD_S = 0.005
FRICTION_MAX_TARGET_VELOCITY_RAD_S = 1.0
FRICTION_MIN_CURRENT_LIMIT_A = 0.001
FRICTION_MAX_CURRENT_LIMIT_A = 10.0
FRICTION_MIN_VOLTAGE_LIMIT_V = 0.1
FRICTION_MAX_VOLTAGE_LIMIT_V = 100.0
FRICTION_MAX_VELOCITY_LIMIT_RAD_S = 100.0


class FrictionPhase(str, Enum):
    IDLE = "idle"
    ZERO = "zero"
    SETTLING = "settling"
    MEASURING = "measuring"
    PAUSE = "pause"
    RECOVERING = "recovering"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True, slots=True)
class FrictionAction:
    kind: str
    value: float | None = None


@dataclass(slots=True)
class FrictionTestConfig:
    low_velocity_rad_s: float = 0.02
    high_velocity_rad_s: float = 0.05
    current_limit_a: float = 0.05
    voltage_limit_v: float = 12.0
    velocity_limit_rad_s: float = 0.3
    angle_min_rad: float = -3.0
    angle_max_rad: float = 3.0
    settle_s: float = 2.0
    measure_s: float = 4.0
    pause_s: float = 1.0
    monitor_downsample: int = 20
    max_recovery_attempts: int = 3

    @property
    def targets(self) -> tuple[float, ...]:
        return (
            self.low_velocity_rad_s,
            -self.low_velocity_rad_s,
            self.high_velocity_rad_s,
            -self.high_velocity_rad_s,
        )

    @property
    def estimated_duration_s(self) -> float:
        return self.pause_s + len(self.targets) * (
            self.settle_s + self.measure_s + self.pause_s
        )

    @property
    def position_margin_rad(self) -> float:
        one_way_targets = self.low_velocity_rad_s + self.high_velocity_rad_s
        return one_way_targets * (self.settle_s + self.measure_s) + 0.1

    def validate(self) -> None:
        if not (
            FRICTION_MIN_TARGET_VELOCITY_RAD_S
            <= self.low_velocity_rad_s
            < self.high_velocity_rad_s
            <= FRICTION_MAX_TARGET_VELOCITY_RAD_S
        ):
            raise ValueError(
                "Скорости опыта должны удовлетворять "
                f"{FRICTION_MIN_TARGET_VELOCITY_RAD_S:g} ≤ малая < большая ≤ "
                f"{FRICTION_MAX_TARGET_VELOCITY_RAD_S:g} рад/с"
            )
        if not (
            FRICTION_MIN_CURRENT_LIMIT_A
            <= self.current_limit_a
            <= FRICTION_MAX_CURRENT_LIMIT_A
        ):
            raise ValueError(
                "Предел тока опыта должен быть от "
                f"{FRICTION_MIN_CURRENT_LIMIT_A:g} до {FRICTION_MAX_CURRENT_LIMIT_A:g} А"
            )
        if not (
            FRICTION_MIN_VOLTAGE_LIMIT_V
            <= self.voltage_limit_v
            <= FRICTION_MAX_VOLTAGE_LIMIT_V
        ):
            raise ValueError(
                "Предел напряжения опыта должен быть от "
                f"{FRICTION_MIN_VOLTAGE_LIMIT_V:g} до {FRICTION_MAX_VOLTAGE_LIMIT_V:g} В"
            )
        if not (
            self.high_velocity_rad_s
            <= self.velocity_limit_rad_s
            <= FRICTION_MAX_VELOCITY_LIMIT_RAD_S
        ):
            raise ValueError(
                "Предел скорости должен покрывать большую цель и быть не больше "
                f"{FRICTION_MAX_VELOCITY_LIMIT_RAD_S:g} рад/с"
            )
        if not self.angle_min_rad < self.angle_max_rad:
            raise ValueError("Минимальная координата опыта должна быть меньше максимальной")
        if self.angle_max_rad - self.angle_min_rad <= 2 * self.position_margin_rad:
            raise ValueError("Диапазон координат слишком узок для заданной длительности опыта")
        if self.settle_s < 1.0 or self.measure_s < 2.0 or self.pause_s < 0.5:
            raise ValueError("Нужно не менее 1 с стабилизации, 2 с измерения и 0,5 с паузы")
        if not 5 <= self.monitor_downsample <= 100:
            raise ValueError("Downsample опыта должен быть от 5 до 100")
        if not 0 <= self.max_recovery_attempts <= 10:
            raise ValueError("Число автоматических восстановлений должно быть от 0 до 10")

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["targets_rad_s"] = list(self.targets)
        payload["monitor_mask"] = FRICTION_MONITOR_MASK
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> FrictionTestConfig:
        field_names = cls.__dataclass_fields__.keys()
        config = cls(**{name: payload[name] for name in field_names if name in payload})
        config.validate()
        return config


@dataclass(slots=True)
class FrictionPointResult:
    target_velocity_rad_s: float
    mean_velocity_rad_s: float
    velocity_std_rad_s: float
    mean_current_q_a: float
    current_std_a: float
    friction_torque_nm: float
    breakaway_torque_nm: float
    sample_count: int
    valid: bool
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> FrictionPointResult:
        return cls(**payload)


@dataclass(slots=True)
class FrictionEstimate:
    valid: bool
    coulomb_friction_nm: float | None
    coulomb_positive_nm: float | None
    coulomb_negative_nm: float | None
    viscous_friction_nm_s_rad: float | None
    breakaway_friction_nm: float | None
    breakaway_positive_nm: float | None
    breakaway_negative_nm: float | None
    asymmetry_percent: float | None
    r_squared: float | None
    note: str
    points: list[FrictionPointResult] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _trimmed(values: Iterable[float], fraction: float = 0.1) -> np.ndarray:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size < 3:
        return array
    ordered = np.sort(array)
    trim = min(int(ordered.size * fraction), max(0, (ordered.size - 3) // 2))
    return ordered[trim : ordered.size - trim] if trim else ordered


def summarize_friction_point(
    target_velocity_rad_s: float,
    steady_samples: Iterable[TelemetrySample],
    transient_samples: Iterable[TelemetrySample],
    torque_constant_nm_per_a: float,
) -> FrictionPointResult:
    usable = [
        sample
        for sample in steady_samples
        if sample.velocity_rad_s is not None
        and sample.current_q_a is not None
        and math.isfinite(sample.velocity_rad_s)
        and math.isfinite(sample.current_q_a)
    ]
    velocities = _trimmed(sample.velocity_rad_s for sample in usable if sample.velocity_rad_s is not None)
    currents = _trimmed(sample.current_q_a for sample in usable if sample.current_q_a is not None)
    if velocities.size < 10 or currents.size < 10:
        return FrictionPointResult(
            target_velocity_rad_s,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            int(min(velocities.size, currents.size)),
            False,
            "Недостаточно устойчивых отсчётов",
        )

    mean_velocity = float(np.mean(velocities))
    mean_current = float(np.mean(currents))
    velocity_std = float(np.std(velocities))
    current_std = float(np.std(currents))
    direction_ok = mean_velocity * target_velocity_rad_s > 0
    tracking_tolerance = max(0.005, abs(target_velocity_rad_s) * 0.3)
    tracking_error = abs(mean_velocity - target_velocity_rad_s)
    stable = velocity_std <= max(0.003, abs(target_velocity_rad_s) * 0.25)

    transient_currents = _trimmed(
        abs(sample.current_q_a)
        for sample in transient_samples
        if sample.current_q_a is not None and math.isfinite(sample.current_q_a)
    )
    if transient_currents.size:
        breakaway = float(np.percentile(transient_currents, 95)) * torque_constant_nm_per_a
    else:
        breakaway = abs(mean_current) * torque_constant_nm_per_a

    valid = direction_ok and tracking_error <= tracking_tolerance and stable
    problems: list[str] = []
    if not direction_ok:
        problems.append("направление не совпало с целью")
    if tracking_error > tracking_tolerance:
        problems.append("скорость не достигнута")
    if not stable:
        problems.append("скорость неустойчива")
    return FrictionPointResult(
        target_velocity_rad_s=target_velocity_rad_s,
        mean_velocity_rad_s=mean_velocity,
        velocity_std_rad_s=velocity_std,
        mean_current_q_a=mean_current,
        current_std_a=current_std,
        friction_torque_nm=abs(mean_current) * torque_constant_nm_per_a,
        breakaway_torque_nm=breakaway,
        sample_count=int(min(velocities.size, currents.size)),
        valid=valid,
        note="ОК" if valid else "; ".join(problems),
    )


def estimate_friction(points: Iterable[FrictionPointResult]) -> FrictionEstimate:
    point_list = list(points)
    usable = [point for point in point_list if point.valid]
    positive = [point for point in usable if point.target_velocity_rad_s > 0]
    negative = [point for point in usable if point.target_velocity_rad_s < 0]
    if len(positive) < 2 or len(negative) < 2:
        return FrictionEstimate(
            valid=False,
            coulomb_friction_nm=None,
            coulomb_positive_nm=None,
            coulomb_negative_nm=None,
            viscous_friction_nm_s_rad=None,
            breakaway_friction_nm=None,
            breakaway_positive_nm=None,
            breakaway_negative_nm=None,
            asymmetry_percent=None,
            r_squared=None,
            note="Нужны две устойчивые скорости в каждом направлении; вероятно, момента недостаточно",
            points=point_list,
        )

    design: list[list[float]] = []
    torques: list[float] = []
    for point in usable:
        design.append(
            [
                1.0 if point.target_velocity_rad_s > 0 else 0.0,
                1.0 if point.target_velocity_rad_s < 0 else 0.0,
                abs(point.mean_velocity_rad_s),
            ]
        )
        torques.append(point.friction_torque_nm)
    matrix = np.asarray(design, dtype=float)
    vector = np.asarray(torques, dtype=float)
    coefficients, *_ = np.linalg.lstsq(matrix, vector, rcond=None)
    coulomb_positive, coulomb_negative, viscous = (float(value) for value in coefficients)
    prediction = matrix @ coefficients
    residual = float(np.sum((vector - prediction) ** 2))
    total = float(np.sum((vector - np.mean(vector)) ** 2))
    r_squared = 1.0 - residual / total if total > 1e-18 else 1.0
    coulomb = (coulomb_positive + coulomb_negative) / 2.0
    breakaway_positive = float(np.mean([point.breakaway_torque_nm for point in positive]))
    breakaway_negative = float(np.mean([point.breakaway_torque_nm for point in negative]))
    breakaway = (breakaway_positive + breakaway_negative) / 2.0
    asymmetry = (
        100.0 * (coulomb_positive - coulomb_negative) / coulomb
        if abs(coulomb) > 1e-12
        else 0.0
    )
    physical = coulomb_positive >= 0 and coulomb_negative >= 0 and viscous >= 0
    quality_ok = r_squared >= 0.5
    if not physical:
        note = "Аппроксимация дала отрицательный коэффициент; результат нельзя принимать в профиль"
    elif not quality_ok:
        note = "Слабое соответствие модели (R² < 0,5); результат нельзя принимать в профиль"
    else:
        note = "Грубая начальная оценка; подтвердите отдельным прогоном"
    return FrictionEstimate(
        valid=physical and quality_ok,
        coulomb_friction_nm=coulomb,
        coulomb_positive_nm=coulomb_positive,
        coulomb_negative_nm=coulomb_negative,
        viscous_friction_nm_s_rad=viscous,
        breakaway_friction_nm=breakaway,
        breakaway_positive_nm=breakaway_positive,
        breakaway_negative_nm=breakaway_negative,
        asymmetry_percent=asymmetry,
        r_squared=r_squared,
        note=note,
        points=point_list,
    )


class FrictionExperiment:
    """Telemetry-driven state for one four-point low-speed friction experiment."""

    def __init__(
        self,
        config: FrictionTestConfig,
        torque_constant_nm_per_a: float,
        completed_points: Iterable[FrictionPointResult] = (),
    ) -> None:
        config.validate()
        self.config = config
        self.torque_constant_nm_per_a = torque_constant_nm_per_a
        restored_points = list(completed_points)
        self.phase = FrictionPhase.IDLE
        self.phase_started_s = 0.0
        self.point_index = len(restored_points) - 1
        self.points = restored_points
        self.transient_samples: list[TelemetrySample] = []
        self.steady_samples: list[TelemetrySample] = []
        self.recovery_attempts = 0
        self.interruption_count = 0
        self.abort_reason = ""

    @property
    def active(self) -> bool:
        return self.phase not in {FrictionPhase.COMPLETE, FrictionPhase.ABORTED}

    @property
    def current_target(self) -> float | None:
        if 0 <= self.point_index < len(self.config.targets):
            return self.config.targets[self.point_index]
        return None

    def start(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.IDLE:
            raise RuntimeError("Friction experiment has already been started")
        self.phase = FrictionPhase.ZERO
        self.phase_started_s = now_s
        return [FrictionAction("target", 0.0)]

    def add_sample(self, sample: TelemetrySample) -> str | None:
        violation = self._violation(sample)
        if violation:
            return violation
        if self.phase == FrictionPhase.SETTLING:
            self.transient_samples.append(sample)
        elif self.phase == FrictionPhase.MEASURING:
            self.steady_samples.append(sample)
        return None

    def tick(self, now_s: float) -> list[FrictionAction]:
        elapsed = now_s - self.phase_started_s
        if self.phase == FrictionPhase.ZERO and elapsed >= self.config.pause_s:
            return self._start_next_point(now_s)
        if self.phase == FrictionPhase.SETTLING and elapsed >= self.config.settle_s:
            self.phase = FrictionPhase.MEASURING
            self.phase_started_s = now_s
            return []
        if self.phase == FrictionPhase.MEASURING and elapsed >= self.config.measure_s:
            target = self.current_target
            if target is None:
                return self.abort("Внутренняя ошибка: потеряна текущая скорость")
            self.points.append(
                summarize_friction_point(
                    target,
                    self.steady_samples,
                    self.transient_samples,
                    self.torque_constant_nm_per_a,
                )
            )
            self.phase = FrictionPhase.PAUSE
            self.phase_started_s = now_s
            return [FrictionAction("target", 0.0), FrictionAction("checkpoint")]
        if self.phase == FrictionPhase.PAUSE and elapsed >= self.config.pause_s:
            if self.point_index + 1 >= len(self.config.targets):
                self.phase = FrictionPhase.COMPLETE
                return [FrictionAction("finish")]
            return self._start_next_point(now_s)
        return []

    def enter_recovery(self) -> list[FrictionAction]:
        if not self.active or self.phase == FrictionPhase.RECOVERING:
            return []
        self.recovery_attempts += 1
        self.interruption_count += 1
        if self.recovery_attempts > self.config.max_recovery_attempts:
            return self.abort("Превышено число автоматических восстановлений телеметрии")
        self.phase = FrictionPhase.RECOVERING
        self.transient_samples.clear()
        self.steady_samples.clear()
        return [FrictionAction("safe_stop")]

    def resume_after_recovery(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.RECOVERING:
            return []
        self.phase_started_s = now_s
        if self.current_target is None:
            self.phase = FrictionPhase.ZERO
            return [FrictionAction("target", 0.0)]
        self.phase = FrictionPhase.SETTLING
        return [FrictionAction("target", self.current_target)]

    def abort(self, reason: str) -> list[FrictionAction]:
        self.abort_reason = reason
        self.phase = FrictionPhase.ABORTED
        return [FrictionAction("safe_stop")]

    def estimate(self) -> FrictionEstimate:
        return estimate_friction(self.points)

    def checkpoint_payload(self, experiment_id: int | None) -> dict[str, Any]:
        return {
            "schema": 1,
            "experiment_id": experiment_id,
            "phase": self.phase.value,
            "config": self.config.to_dict(),
            "completed_points": [point.to_dict() for point in self.points],
            "interruption_count": self.interruption_count,
            "abort_reason": self.abort_reason,
        }

    def _start_next_point(self, now_s: float) -> list[FrictionAction]:
        self.point_index += 1
        self.transient_samples.clear()
        self.steady_samples.clear()
        self.phase = FrictionPhase.SETTLING
        self.phase_started_s = now_s
        return [FrictionAction("target", self.config.targets[self.point_index])]

    def _violation(self, sample: TelemetrySample) -> str | None:
        checks = (
            ("Iq", sample.current_q_a, self.config.current_limit_a),
            ("Id", sample.current_d_a, self.config.current_limit_a),
            ("Uq", sample.voltage_q_v, self.config.voltage_limit_v),
            ("Ud", sample.voltage_d_v, self.config.voltage_limit_v),
            ("скорость", sample.velocity_rad_s, self.config.velocity_limit_rad_s),
        )
        for name, value, limit in checks:
            if value is not None and abs(value) > limit:
                return f"{name} вышел за предел ±{limit:g}: {value:g}"
        if sample.angle_rad is not None:
            if sample.angle_rad < self.config.angle_min_rad:
                return "Координата вышла ниже предела опыта"
            if sample.angle_rad > self.config.angle_max_rad:
                return "Координата вышла выше предела опыта"
        return None
