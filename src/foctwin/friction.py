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
FRICTION_MIN_CURRENT_TRIP_A = 0.01
FRICTION_MAX_CURRENT_TRIP_A = 10.0
FRICTION_MIN_VOLTAGE_LIMIT_V = 0.1
FRICTION_MAX_VOLTAGE_LIMIT_V = 100.0
FRICTION_MAX_VELOCITY_LIMIT_RAD_S = 100.0
FRICTION_TORQUE_MODE = "voltage"
FRICTION_DIRECT_VOLTAGE_SENTINEL = -12345.0
FRICTION_VELOCITY_WINDOW_S = 0.5
FRICTION_SOFT_LIMIT_SAMPLES = 3
FRICTION_SOFT_LIMIT_TOLERANCE = 0.05
FRICTION_HARD_LIMIT_MULTIPLIER = 2.0


class FrictionPhase(str, Enum):
    IDLE = "idle"
    ACTUATOR_BASELINE = "actuator_baseline"
    ACTUATOR_PULSE = "actuator_pulse"
    ACTUATOR_PAUSE = "actuator_pause"
    CONFIGURING_VELOCITY = "configuring_velocity"
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
class ActuatorPulseResult:
    direction: int
    command_voltage_v: float
    mean_voltage_q_v: float
    mean_measured_current_q_a: float
    mean_abs_measured_current_a: float
    peak_measured_current_q_a: float
    angle_delta_rad: float
    movement_detected: bool
    current_detected: bool
    sample_count: int
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> ActuatorPulseResult:
        field_names = cls.__dataclass_fields__.keys()
        return cls(**{name: payload[name] for name in field_names if name in payload})


@dataclass(slots=True)
class FrictionTestConfig:
    low_velocity_rad_s: float = 0.02
    high_velocity_rad_s: float = 0.05
    current_trip_limit_a: float = 1.0
    voltage_limit_v: float = 12.0
    velocity_limit_rad_s: float = 0.3
    angle_min_rad: float = -3.0
    angle_max_rad: float = 3.0
    pulse_start_voltage_v: float = 0.1
    pulse_step_voltage_v: float = 0.1
    pulse_max_voltage_v: float = 0.5
    pulse_duration_s: float = 0.5
    actuator_pause_s: float = 0.7
    baseline_s: float = 1.0
    movement_threshold_rad: float = 0.002
    measured_current_floor_a: float = 0.01
    breakaway_margin: float = 1.2
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
        actuator_s = self.baseline_s + 2 * len(self.pulse_levels) * (
            self.pulse_duration_s + self.actuator_pause_s
        )
        return actuator_s + self.pause_s + len(self.targets) * (
            self.settle_s + self.measure_s + self.pause_s
        )

    @property
    def position_margin_rad(self) -> float:
        one_way_targets = self.low_velocity_rad_s + self.high_velocity_rad_s
        actuator_margin = 2.0 * self.movement_threshold_rad
        return one_way_targets * (self.settle_s + self.measure_s) + actuator_margin + 0.1

    @property
    def pulse_levels(self) -> tuple[float, ...]:
        values: list[float] = []
        value = self.pulse_start_voltage_v
        while value < self.pulse_max_voltage_v - 1e-12:
            values.append(value)
            value += self.pulse_step_voltage_v
        if not values or values[-1] < self.pulse_max_voltage_v - 1e-12:
            values.append(self.pulse_max_voltage_v)
        return tuple(values)

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
        if not (FRICTION_MIN_CURRENT_TRIP_A <= self.current_trip_limit_a <= FRICTION_MAX_CURRENT_TRIP_A):
            raise ValueError(
                "Аварийный предел измеренного тока должен быть от "
                f"{FRICTION_MIN_CURRENT_TRIP_A:g} до {FRICTION_MAX_CURRENT_TRIP_A:g} А"
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
        if not 0.001 <= self.pulse_start_voltage_v <= self.pulse_max_voltage_v:
            raise ValueError("Начальная команда Uq должна быть положительной и не выше максимальной")
        if not 0.001 <= self.pulse_step_voltage_v <= self.pulse_max_voltage_v:
            raise ValueError("Шаг Uq должен быть положительным и не выше максимальной команды")
        if len(self.pulse_levels) > 200:
            raise ValueError("План Uq содержит больше 200 уровней; увеличьте шаг")
        if self.pulse_max_voltage_v > self.voltage_limit_v:
            raise ValueError("Максимальный импульс Uq не должен превышать предел напряжения опыта")
        if not 0.1 <= self.pulse_duration_s <= 5.0:
            raise ValueError("Длительность импульса должна быть от 0,1 до 5 с")
        if not 0.2 <= self.actuator_pause_s <= 10.0 or not 0.5 <= self.baseline_s <= 10.0:
            raise ValueError("Пауза привода должна быть 0,2–10 с, исходный ноль — 0,5–10 с")
        if not 0.0001 <= self.movement_threshold_rad <= 0.5:
            raise ValueError("Порог движения должен быть от 0,0001 до 0,5 рад")
        if not 0.001 <= self.measured_current_floor_a <= self.current_trip_limit_a:
            raise ValueError("Порог подтверждения Iq должен быть положительным и ниже аварийного тока")
        if not 1.0 < self.breakaway_margin <= 2.0:
            raise ValueError("Запас над страгиванием должен быть больше 1 и не больше 2")
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
        payload["algorithm_schema"] = 3
        payload["targets_rad_s"] = list(self.targets)
        payload["monitor_mask"] = FRICTION_MONITOR_MASK
        payload["torque_mode"] = FRICTION_TORQUE_MODE
        payload["velocity_estimator"] = {
            "source": "angle_slope",
            "window_s": FRICTION_VELOCITY_WINDOW_S,
        }
        payload["actuator_preflight"] = {
            "command": "direct_uq_voltage",
            "directions": [1, -1],
            "requires_measured_iq": True,
        }
        payload["soft_limit_samples"] = FRICTION_SOFT_LIMIT_SAMPLES
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
    current_source: str = "measured_iq"
    mean_measured_current_q_a: float | None = None
    voltage_saturation_fraction: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> FrictionPointResult:
        field_names = cls.__dataclass_fields__.keys()
        return cls(**{name: payload[name] for name in field_names if name in payload})


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


def _angle_slope_metrics(
    samples: Iterable[TelemetrySample],
    window_s: float = FRICTION_VELOCITY_WINDOW_S,
) -> tuple[float, float] | None:
    pairs = sorted(
        (
            (float(sample.timestamp_s), float(sample.angle_rad))
            for sample in samples
            if sample.angle_rad is not None
            and math.isfinite(sample.timestamp_s)
            and math.isfinite(sample.angle_rad)
        ),
        key=lambda pair: pair[0],
    )
    if len(pairs) < 10:
        return None
    timestamps = np.asarray([pair[0] for pair in pairs], dtype=float)
    angles = np.asarray([pair[1] for pair in pairs], dtype=float)
    unique = np.r_[True, np.diff(timestamps) > 0]
    timestamps = timestamps[unique]
    angles = angles[unique]
    if timestamps.size < 10 or timestamps[-1] - timestamps[0] < window_s:
        return None

    centered_time = timestamps - timestamps[0]
    mean_velocity = float(np.polyfit(centered_time, angles, 1)[0])
    local_slopes: list[float] = []
    for start in range(timestamps.size):
        stop = int(np.searchsorted(timestamps, timestamps[start] + window_s, side="right"))
        if stop - start < 8:
            continue
        local_time = timestamps[start:stop] - timestamps[start]
        if local_time[-1] < window_s * 0.75:
            continue
        local_slopes.append(float(np.polyfit(local_time, angles[start:stop], 1)[0]))
    trimmed_slopes = _trimmed(local_slopes)
    if trimmed_slopes.size < 3:
        return mean_velocity, 0.0
    return mean_velocity, float(np.std(trimmed_slopes))


def summarize_friction_point(
    target_velocity_rad_s: float,
    steady_samples: Iterable[TelemetrySample],
    transient_samples: Iterable[TelemetrySample],
    torque_constant_nm_per_a: float,
    *,
    voltage_limit_v: float | None = None,
) -> FrictionPointResult:
    steady_samples = list(steady_samples)
    transient_samples = list(transient_samples)
    usable = [
        sample
        for sample in steady_samples
        if sample.velocity_rad_s is not None
        and sample.current_q_a is not None
        and math.isfinite(sample.velocity_rad_s)
        and math.isfinite(sample.current_q_a)
    ]
    velocity_metrics = _angle_slope_metrics(usable)
    velocities = _trimmed(
        sample.velocity_rad_s for sample in usable if sample.velocity_rad_s is not None
    )
    if velocity_metrics is None:
        mean_velocity = float(np.mean(velocities)) if velocities.size else 0.0
        velocity_std = float(np.std(velocities)) if velocities.size else 0.0
    else:
        mean_velocity, velocity_std = velocity_metrics

    measured_currents = _trimmed(
        sample.current_q_a for sample in usable if sample.current_q_a is not None
    )
    current_source = "measured_iq"
    currents = measured_currents
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
            current_source,
            float(np.mean(measured_currents)) if measured_currents.size else None,
        )

    mean_current = float(np.mean(currents))
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
    saturation_fraction: float | None = None
    if voltage_limit_v is not None and voltage_limit_v > 0:
        voltages = np.asarray(
            [
                abs(sample.voltage_q_v)
                for sample in usable
                if sample.voltage_q_v is not None and math.isfinite(sample.voltage_q_v)
            ],
            dtype=float,
        )
        if voltages.size:
            saturation_fraction = float(np.mean(voltages >= voltage_limit_v * 0.98))
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
        current_source=current_source,
        mean_measured_current_q_a=(
            float(np.mean(measured_currents)) if measured_currents.size else None
        ),
        voltage_saturation_fraction=saturation_fraction,
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
    """Two-stage actuator preflight followed by measured-current friction points."""

    def __init__(
        self,
        config: FrictionTestConfig,
        torque_constant_nm_per_a: float,
        completed_points: Iterable[FrictionPointResult] = (),
        *,
        phase_resistance_ohm: float,
        actuator_attempts: Iterable[ActuatorPulseResult] = (),
    ) -> None:
        config.validate()
        if phase_resistance_ohm <= 0:
            raise ValueError("Для опыта нужно положительное сопротивление фазы")
        self.config = config
        self.torque_constant_nm_per_a = torque_constant_nm_per_a
        self.phase_resistance_ohm = phase_resistance_ohm
        self.points = list(completed_points)
        self.actuator_attempts = list(actuator_attempts)
        self.phase = FrictionPhase.IDLE
        self.phase_started_s = 0.0
        self.point_index = len(self.points) - 1
        self.transient_samples: list[TelemetrySample] = []
        self.steady_samples: list[TelemetrySample] = []
        self.phase_samples: list[TelemetrySample] = []
        self.angle_window: list[TelemetrySample] = []
        self.pulse_start_angle: float | None = None
        self.current_detection_threshold_a = config.measured_current_floor_a
        self.recovery_attempts = 0
        self.interruption_count = 0
        self.abort_reason = ""
        self.soft_limit_counts: dict[str, int] = {}
        self._repeat_velocity_point = False
        self._recovery_mode = "actuator"

    @property
    def active(self) -> bool:
        return self.phase not in {FrictionPhase.COMPLETE, FrictionPhase.ABORTED}

    @property
    def current_target(self) -> float | None:
        if 0 <= self.point_index < len(self.config.targets):
            return self.config.targets[self.point_index]
        return None

    @property
    def current_pulse_voltage_v(self) -> float | None:
        return self._pulse_target()

    @property
    def breakaway_results(self) -> dict[int, ActuatorPulseResult]:
        found: dict[int, ActuatorPulseResult] = {}
        for attempt in self.actuator_attempts:
            if attempt.movement_detected:
                found.setdefault(attempt.direction, attempt)
        return found

    @property
    def actuator_complete(self) -> bool:
        found = self.breakaway_results
        return all(direction in found and found[direction].current_detected for direction in (1, -1))

    @property
    def configuration_mode(self) -> str:
        if self.phase == FrictionPhase.RECOVERING:
            return self._recovery_mode
        if self.actuator_complete or self.phase in {
            FrictionPhase.CONFIGURING_VELOCITY,
            FrictionPhase.ZERO,
            FrictionPhase.SETTLING,
            FrictionPhase.MEASURING,
            FrictionPhase.PAUSE,
            FrictionPhase.COMPLETE,
        }:
            return "velocity"
        return "actuator"

    @property
    def working_current_limit_a(self) -> float | None:
        if not self.actuator_complete:
            return None
        voltage = max(abs(result.command_voltage_v) for result in self.breakaway_results.values())
        return voltage / self.phase_resistance_ohm * self.config.breakaway_margin

    def start(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.IDLE:
            raise RuntimeError("Friction experiment has already been started")
        self.phase_started_s = now_s
        self.phase_samples.clear()
        if self.actuator_complete:
            self.phase = FrictionPhase.ZERO
        else:
            self.phase = FrictionPhase.ACTUATOR_BASELINE
        return [FrictionAction("target", 0.0)]

    def add_sample(
        self,
        sample: TelemetrySample,
        now_s: float | None = None,
    ) -> tuple[str | None, list[FrictionAction]]:
        event_time_s = sample.timestamp_s if now_s is None else now_s
        if self.phase in {FrictionPhase.IDLE, FrictionPhase.RECOVERING}:
            return None, []
        violation = self._violation(sample)
        if violation:
            return violation, []
        self._update_angle_window(sample)

        if self.phase in {FrictionPhase.ACTUATOR_BASELINE, FrictionPhase.ACTUATOR_PAUSE}:
            self.phase_samples.append(sample)
        elif self.phase == FrictionPhase.ACTUATOR_PULSE:
            self.phase_samples.append(sample)
            movement = self._pulse_movement(sample)
            if movement < 0:
                self._record_pulse(False, "движение пошло против команды")
                return "При импульсе Uq вал начал двигаться против заданного направления", []
            if movement > 0:
                return None, self._finish_pulse(event_time_s, moved=True)
        elif self.phase == FrictionPhase.SETTLING:
            self.transient_samples.append(sample)
        elif self.phase == FrictionPhase.MEASURING:
            self.steady_samples.append(sample)
        angle_violation = self._angle_velocity_violation()
        if angle_violation:
            return angle_violation, []
        return None, []

    def tick(self, now_s: float) -> list[FrictionAction]:
        elapsed = now_s - self.phase_started_s
        if self.phase == FrictionPhase.ACTUATOR_BASELINE:
            if elapsed < self.config.baseline_s:
                return []
            if not self._phase_is_stationary():
                if elapsed >= self.config.baseline_s * 5.0:
                    return self.abort("Вал не остаётся неподвижным при нулевой команде")
                return []
            self._set_current_detection_threshold()
            return self._start_next_pulse(now_s)
        if self.phase == FrictionPhase.ACTUATOR_PULSE and elapsed >= self.config.pulse_duration_s:
            return self._finish_pulse(now_s, moved=False)
        if self.phase == FrictionPhase.ACTUATOR_PAUSE:
            if elapsed < self.config.actuator_pause_s or not self._phase_is_stationary():
                if elapsed >= self.config.actuator_pause_s * 5.0:
                    return self.abort("Вал не остановился после нулевой команды")
                return []
            found = self.breakaway_results
            if all(direction in found for direction in (1, -1)):
                missing_current = [
                    "+" if direction > 0 else "−"
                    for direction in (1, -1)
                    if not found[direction].current_detected
                ]
                if missing_current:
                    return self.abort(
                        "Страгивание найдено, но измеренный Iq не подтвердился для направления "
                        + ", ".join(missing_current)
                    )
                working_limit = self.working_current_limit_a
                if working_limit is None or working_limit > self.config.current_trip_limit_a:
                    return self.abort(
                        "Найденное страгивание требует тока выше аварийного предела опыта"
                    )
                self.phase = FrictionPhase.CONFIGURING_VELOCITY
                self.phase_started_s = now_s
                return [FrictionAction("configure_velocity"), FrictionAction("checkpoint")]
            return self._start_next_pulse(now_s)
        if self.phase == FrictionPhase.ZERO and elapsed >= self.config.pause_s:
            if self._repeat_velocity_point and self.current_target is not None:
                self._repeat_velocity_point = False
                return self._start_velocity_point(now_s, increment=False)
            return self._start_velocity_point(now_s, increment=True)
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
                    voltage_limit_v=self.config.voltage_limit_v,
                )
            )
            self.phase = FrictionPhase.PAUSE
            self.phase_started_s = now_s
            return [FrictionAction("target", 0.0), FrictionAction("checkpoint")]
        if self.phase == FrictionPhase.PAUSE and elapsed >= self.config.pause_s:
            if self.point_index + 1 >= len(self.config.targets):
                self.phase = FrictionPhase.COMPLETE
                return [FrictionAction("finish")]
            return self._start_velocity_point(now_s, increment=True)
        return []

    def velocity_configuration_applied(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.CONFIGURING_VELOCITY:
            return []
        self.phase = FrictionPhase.ZERO
        self.phase_started_s = now_s
        self.phase_samples.clear()
        return [FrictionAction("target", 0.0)]

    def enter_recovery(self) -> list[FrictionAction]:
        if not self.active or self.phase == FrictionPhase.RECOVERING:
            return []
        self.recovery_attempts += 1
        self.interruption_count += 1
        if self.recovery_attempts > self.config.max_recovery_attempts:
            return self.abort("Превышено число автоматических восстановлений телеметрии")
        self._recovery_mode = self.configuration_mode
        self._repeat_velocity_point = self._recovery_mode == "velocity" and self.current_target is not None
        self.phase = FrictionPhase.RECOVERING
        self.transient_samples.clear()
        self.steady_samples.clear()
        self.phase_samples.clear()
        return [FrictionAction("safe_stop")]

    def resume_after_recovery(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.RECOVERING:
            return []
        self.phase_started_s = now_s
        self.phase_samples.clear()
        if self._recovery_mode == "velocity":
            self.phase = FrictionPhase.ZERO
        else:
            self.phase = FrictionPhase.ACTUATOR_BASELINE
        return [FrictionAction("target", 0.0)]

    def abort(self, reason: str) -> list[FrictionAction]:
        self.abort_reason = reason
        self.phase = FrictionPhase.ABORTED
        return [FrictionAction("safe_stop")]

    def estimate(self) -> FrictionEstimate:
        estimate = estimate_friction(self.points)
        found = self.breakaway_results
        if all(direction in found for direction in (1, -1)):
            positive = found[1].mean_abs_measured_current_a * self.torque_constant_nm_per_a
            negative = found[-1].mean_abs_measured_current_a * self.torque_constant_nm_per_a
            estimate.breakaway_positive_nm = positive
            estimate.breakaway_negative_nm = negative
            estimate.breakaway_friction_nm = (positive + negative) / 2.0
        return estimate

    def checkpoint_payload(self, experiment_id: int | None) -> dict[str, Any]:
        return {
            "schema": 3,
            "experiment_id": experiment_id,
            "phase": self.phase.value,
            "config": self.config.to_dict(),
            "motor_parameters": {
                "torque_constant_nm_per_a": self.torque_constant_nm_per_a,
                "phase_resistance_ohm": self.phase_resistance_ohm,
            },
            "actuator_attempts": [attempt.to_dict() for attempt in self.actuator_attempts],
            "completed_points": [point.to_dict() for point in self.points],
            "interruption_count": self.interruption_count,
            "abort_reason": self.abort_reason,
        }

    def _start_next_pulse(self, now_s: float) -> list[FrictionAction]:
        found = self.breakaway_results
        if all(direction in found for direction in (1, -1)):
            missing_current = [
                "+" if direction > 0 else "−"
                for direction in (1, -1)
                if not found[direction].current_detected
            ]
            if missing_current:
                return self.abort(
                    "Страгивание найдено, но измеренный Iq не подтвердился для направления "
                    + ", ".join(missing_current)
                )
        attempted = {
            (attempt.direction, round(abs(attempt.command_voltage_v), 12))
            for attempt in self.actuator_attempts
        }
        for voltage in self.config.pulse_levels:
            for direction in (1, -1):
                key = (direction, round(voltage, 12))
                if direction in found or key in attempted:
                    continue
                self.phase = FrictionPhase.ACTUATOR_PULSE
                self.phase_started_s = now_s
                self.phase_samples.clear()
                self.pulse_start_angle = self._latest_angle()
                if self.pulse_start_angle is None:
                    return self.abort("Нет координаты перед импульсом Uq")
                return [FrictionAction("target", direction * voltage)]
        missing = ["+" if direction > 0 else "−" for direction in (1, -1) if direction not in found]
        return self.abort(
            "Страгивание не найдено до максимального Uq в направлении " + ", ".join(missing)
        )

    def _finish_pulse(self, now_s: float, *, moved: bool) -> list[FrictionAction]:
        self._record_pulse(moved, "страгивание" if moved else "движения нет")
        self.phase = FrictionPhase.ACTUATOR_PAUSE
        self.phase_started_s = now_s
        self.phase_samples.clear()
        return [FrictionAction("target", 0.0), FrictionAction("checkpoint")]

    def _record_pulse(self, moved: bool, note: str) -> None:
        target = self._pulse_target()
        if target is None:
            return
        voltages = np.asarray(
            [sample.voltage_q_v for sample in self.phase_samples if sample.voltage_q_v is not None],
            dtype=float,
        )
        currents = np.asarray(
            [sample.current_q_a for sample in self.phase_samples if sample.current_q_a is not None],
            dtype=float,
        )
        angle_delta = 0.0
        latest = self._latest_angle(self.phase_samples)
        if latest is not None and self.pulse_start_angle is not None:
            angle_delta = latest - self.pulse_start_angle
        above_threshold = (
            int(np.count_nonzero(np.abs(currents) >= self.current_detection_threshold_a))
            if currents.size
            else 0
        )
        current_detected = above_threshold >= 3
        self.actuator_attempts.append(
            ActuatorPulseResult(
                direction=1 if target > 0 else -1,
                command_voltage_v=target,
                mean_voltage_q_v=float(np.mean(voltages)) if voltages.size else 0.0,
                mean_measured_current_q_a=float(np.mean(currents)) if currents.size else 0.0,
                mean_abs_measured_current_a=(
                    float(np.mean(np.abs(currents))) if currents.size else 0.0
                ),
                peak_measured_current_q_a=float(np.max(np.abs(currents))) if currents.size else 0.0,
                angle_delta_rad=angle_delta,
                movement_detected=moved,
                current_detected=current_detected,
                sample_count=int(min(voltages.size, currents.size)),
                note=(
                    note
                    if current_detected
                    else f"{note}; измеренный Iq не подтверждён"
                ),
            )
        )

    def _pulse_target(self) -> float | None:
        if self.phase != FrictionPhase.ACTUATOR_PULSE:
            return None
        attempted = {
            (attempt.direction, round(abs(attempt.command_voltage_v), 12))
            for attempt in self.actuator_attempts
        }
        found = self.breakaway_results
        for voltage in self.config.pulse_levels:
            for direction in (1, -1):
                if direction in found:
                    continue
                if (direction, round(voltage, 12)) not in attempted:
                    return direction * voltage
        return None

    def _pulse_movement(self, sample: TelemetrySample) -> int:
        target = self._pulse_target()
        if (
            target is None
            or sample.angle_rad is None
            or self.pulse_start_angle is None
        ):
            return 0
        signed_delta = math.copysign(1.0, target) * (sample.angle_rad - self.pulse_start_angle)
        if signed_delta >= self.config.movement_threshold_rad:
            return 1
        if signed_delta <= -self.config.movement_threshold_rad:
            return -1
        return 0

    def _start_velocity_point(self, now_s: float, *, increment: bool) -> list[FrictionAction]:
        if increment:
            self.point_index += 1
        if not 0 <= self.point_index < len(self.config.targets):
            return self.abort("Внутренняя ошибка: неверный номер точки скорости")
        self.transient_samples.clear()
        self.steady_samples.clear()
        self.phase = FrictionPhase.SETTLING
        self.phase_started_s = now_s
        return [FrictionAction("target", self.config.targets[self.point_index])]

    def _phase_is_stationary(self) -> bool:
        if not self.phase_samples:
            return False
        latest_timestamp = self.phase_samples[-1].timestamp_s
        window_s = min(self.config.baseline_s, self.config.actuator_pause_s)
        angles = [
            sample.angle_rad
            for sample in self.phase_samples
            if sample.timestamp_s >= latest_timestamp - window_s
            and sample.angle_rad is not None
            and math.isfinite(sample.angle_rad)
        ]
        return len(angles) >= 4 and max(angles) - min(angles) < self.config.movement_threshold_rad

    def _set_current_detection_threshold(self) -> None:
        latest_timestamp = self.phase_samples[-1].timestamp_s if self.phase_samples else 0.0
        currents = np.asarray(
            [
                abs(sample.current_q_a)
                for sample in self.phase_samples
                if sample.timestamp_s >= latest_timestamp - self.config.baseline_s
                and sample.current_q_a is not None
                and math.isfinite(sample.current_q_a)
            ],
            dtype=float,
        )
        if not currents.size:
            self.current_detection_threshold_a = self.config.measured_current_floor_a
            return
        median = float(np.median(currents))
        mad = float(np.median(np.abs(currents - median)))
        self.current_detection_threshold_a = max(
            self.config.measured_current_floor_a,
            median + 6.0 * 1.4826 * mad,
        )

    def _latest_angle(self, samples: Iterable[TelemetrySample] | None = None) -> float | None:
        source = list(samples) if samples is not None else self.angle_window
        for sample in reversed(source):
            if sample.angle_rad is not None and math.isfinite(sample.angle_rad):
                return float(sample.angle_rad)
        return None

    def _update_angle_window(self, sample: TelemetrySample) -> None:
        if sample.angle_rad is None or not math.isfinite(sample.angle_rad):
            return
        self.angle_window.append(sample)
        cutoff = sample.timestamp_s - max(1.0, FRICTION_VELOCITY_WINDOW_S * 2.0)
        self.angle_window = [item for item in self.angle_window if item.timestamp_s >= cutoff]

    def _angle_velocity_violation(self) -> str | None:
        metrics = _angle_slope_metrics(self.angle_window)
        if metrics is None:
            return None
        slope = metrics[0]
        limit = self.config.velocity_limit_rad_s
        return self._debounced_limit("скорость по углу", slope, limit)

    def _violation(self, sample: TelemetrySample) -> str | None:
        uq_limit = (
            self.config.pulse_max_voltage_v
            if self.configuration_mode == "actuator"
            else self.config.voltage_limit_v
        )
        for name, value, limit in (
            ("Iq", sample.current_q_a, self.config.current_trip_limit_a),
            ("Id", sample.current_d_a, self.config.current_trip_limit_a),
            ("Uq", sample.voltage_q_v, uq_limit),
            ("Ud", sample.voltage_d_v, self.config.voltage_limit_v),
        ):
            violation = self._debounced_limit(name, value, limit)
            if violation:
                return violation
        if sample.angle_rad is not None:
            if sample.angle_rad < self.config.angle_min_rad:
                return "Координата вышла ниже предела опыта"
            if sample.angle_rad > self.config.angle_max_rad:
                return "Координата вышла выше предела опыта"
        return None

    def _debounced_limit(self, name: str, value: float | None, limit: float) -> str | None:
        if value is None or not math.isfinite(value):
            return None
        absolute = abs(value)
        if absolute > limit * FRICTION_HARD_LIMIT_MULTIPLIER:
            self.soft_limit_counts.clear()
            return (
                f"резкий выброс {name}: {value:g}; жёсткий порог "
                f"±{limit * FRICTION_HARD_LIMIT_MULTIPLIER:g}"
            )
        if absolute > limit * (1.0 + FRICTION_SOFT_LIMIT_TOLERANCE):
            count = self.soft_limit_counts.get(name, 0) + 1
            self.soft_limit_counts[name] = count
            if count >= FRICTION_SOFT_LIMIT_SAMPLES:
                self.soft_limit_counts.clear()
                return (
                    f"{name} устойчиво выше рабочего предела ±{limit:g}: "
                    f"{value:g} ({count} отсчёта подряд)"
                )
        else:
            self.soft_limit_counts[name] = 0
        return None
