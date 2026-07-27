from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from typing import Any

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
FRICTION_DEFAULT_RECOVERY_ATTEMPTS = 50
FRICTION_MAX_RECOVERY_ATTEMPTS = 100
FRICTION_ANGLE_GLITCH_RATE_RAD_S = 0.5
FRICTION_ANGLE_GLITCH_MARGIN_RAD = 0.0005
FRICTION_ANGLE_RESOLUTION_RAD = 0.0001
FRICTION_MOVEMENT_CONFIRMATION_SAMPLES = 2
FRICTION_MIN_POSITION_BIN_WIDTH_RAD = 0.01
FRICTION_MAX_POSITION_BIN_WIDTH_RAD = 10.0
FRICTION_MIN_POSITION_BIN_SAMPLES = 8
FRICTION_CURRENT_CONFIRMATION_FRACTION = 0.6
FRICTION_CURRENT_DIRECTION_FRACTION = 0.8
FRICTION_MIN_AUTOMATIC_POSITION_STEP_RAD = 0.01
FRICTION_MAX_AUTOMATIC_POSITION_STEP_RAD = 100.0
FRICTION_MAX_AUTOMATIC_POSITIONS = 20
FRICTION_MAX_MAP_PASSES = 3
FRICTION_MIN_POSITION_TOLERANCE_RAD = 0.0001
FRICTION_MAX_POSITION_TOLERANCE_RAD = 0.1
FRICTION_MIN_POSITIONING_VOLTAGE_STEP_V = 0.01
FRICTION_MAX_POSITIONING_VOLTAGE_STEP_V = 20.0
FRICTION_MIN_POSITION_STALL_WINDOW_S = 1.0
FRICTION_MAX_POSITION_STALL_WINDOW_S = 15.0
FRICTION_POSITION_SETTLE_S = 1.0
FRICTION_POSITION_TIMEOUT_S = 60.0
FRICTION_POSITION_SATURATION_FRACTION = 0.9


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
    CONFIGURING_POSITION = "configuring_position"
    POSITIONING = "positioning"
    POSITION_SETTLING = "position_settling"
    CONFIGURING_ACTUATOR = "configuring_actuator"
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
    start_angle_rad: float | None = None
    end_angle_rad: float | None = None
    position_index: int = 0
    measurement_position_rad: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> ActuatorPulseResult:
        field_names = cls.__dataclass_fields__.keys()
        return cls(**{name: payload[name] for name in field_names if name in payload})


@dataclass(slots=True)
class BaselineDiagnostic:
    position_index: int
    measurement_position_rad: float
    sample_count: int
    duration_s: float
    mean_angle_rad: float | None
    angle_std_rad: float | None
    angle_drift_rad_s: float | None
    mean_current_q_a: float | None
    current_std_a: float | None
    mean_voltage_q_v: float | None
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> BaselineDiagnostic:
        field_names = cls.__dataclass_fields__.keys()
        return cls(**{name: payload[name] for name in field_names if name in payload})


@dataclass(slots=True)
class PositioningResult:
    position_index: int
    start_position_rad: float | None
    target_position_rad: float
    final_position_rad: float | None
    final_error_rad: float | None
    duration_s: float
    reached: bool
    initial_voltage_limit_v: float
    final_voltage_limit_v: float
    voltage_boost_count: int
    maximum_measured_voltage_v: float | None
    hold_voltage_q_v: float | None
    hold_current_q_a: float | None
    overshoot_rad: float | None
    approach_direction: int
    saturated_at_end: bool
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> PositioningResult:
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
    movement_threshold_rad: float = 0.001
    measured_current_floor_a: float = 0.01
    breakaway_margin: float = 1.2
    settle_s: float = 2.0
    measure_s: float = 4.0
    pause_s: float = 1.0
    monitor_downsample: int = 20
    max_recovery_attempts: int = FRICTION_DEFAULT_RECOVERY_ATTEMPTS
    position_bin_width_rad: float = 0.1
    automatic_position_count: int = 1
    automatic_position_step_rad: float = 1.0
    map_passes: int = 2
    position_tolerance_rad: float = 0.005
    positioning_voltage_step_v: float = 0.25
    positioning_voltage_max_v: float = 3.0
    position_stall_window_s: float = 3.0
    position_min_progress_rad: float = 0.002

    @property
    def speed_levels(self) -> tuple[float, ...]:
        middle = math.sqrt(self.low_velocity_rad_s * self.high_velocity_rad_s)
        return (self.low_velocity_rad_s, middle, self.high_velocity_rad_s)

    @property
    def targets(self) -> tuple[float, ...]:
        low, middle, high = self.speed_levels
        return (
            low,
            -low,
            middle,
            -middle,
            high,
            -high,
        )

    @property
    def estimated_duration_s(self) -> float:
        actuator_s = self.baseline_s + 2 * len(self.pulse_levels) * (
            self.pulse_duration_s + self.actuator_pause_s
        )
        measurement_s = actuator_s + self.pause_s + len(self.targets) * (
            self.settle_s + self.measure_s + self.pause_s
        )
        positioning_s = max(
            FRICTION_POSITION_SETTLE_S,
            abs(self.automatic_position_step_rad) / self.velocity_limit_rad_s,
        )
        return self.measurement_position_count * measurement_s + max(
            0,
            self.measurement_position_count - 1,
        ) * positioning_s

    @property
    def position_margin_rad(self) -> float:
        one_way_targets = sum(self.speed_levels)
        actuator_margin = 2.0 * self.movement_threshold_rad
        return one_way_targets * (self.settle_s + self.measure_s) + actuator_margin + 0.1

    @property
    def measurement_position_count(self) -> int:
        if self.automatic_position_count == 1:
            return self.map_passes
        return self.automatic_position_count + (self.map_passes - 1) * (
            self.automatic_position_count - 1
        )

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

    def automatic_position_targets(self, start_angle_rad: float) -> tuple[float, ...]:
        unique_targets = tuple(
            start_angle_rad + index * self.automatic_position_step_rad
            for index in range(self.automatic_position_count)
        )
        targets = list(unique_targets)
        direction = -1
        for _pass_index in range(1, self.map_passes):
            if self.automatic_position_count == 1:
                targets.append(unique_targets[0])
                continue
            if direction < 0:
                targets.extend(reversed(unique_targets[:-1]))
            else:
                targets.extend(unique_targets[1:])
            direction *= -1
        safe_min = self.angle_min_rad + self.position_margin_rad
        safe_max = self.angle_max_rad - self.position_margin_rad
        outside = [target for target in targets if not safe_min <= target <= safe_max]
        if outside:
            raise ValueError(
                "Автоматические точки выходят из безопасного диапазона стартовых координат "
                f"[{safe_min:.6g}; {safe_max:.6g}] рад: "
                + ", ".join(f"{target:.6g}" for target in outside)
            )
        return tuple(targets)

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
        if not 0 <= self.max_recovery_attempts <= FRICTION_MAX_RECOVERY_ATTEMPTS:
            raise ValueError(
                "Число автоматических восстановлений должно быть от 0 до "
                f"{FRICTION_MAX_RECOVERY_ATTEMPTS}"
            )
        if not (
            FRICTION_MIN_POSITION_BIN_WIDTH_RAD
            <= self.position_bin_width_rad
            <= FRICTION_MAX_POSITION_BIN_WIDTH_RAD
        ):
            raise ValueError(
                "Ширина интервала карты должна быть от "
                f"{FRICTION_MIN_POSITION_BIN_WIDTH_RAD:g} до "
                f"{FRICTION_MAX_POSITION_BIN_WIDTH_RAD:g} рад"
            )
        if not 1 <= self.automatic_position_count <= FRICTION_MAX_AUTOMATIC_POSITIONS:
            raise ValueError(
                "Число автоматических положений должно быть от 1 до "
                f"{FRICTION_MAX_AUTOMATIC_POSITIONS}"
            )
        if self.automatic_position_count > 1 and not (
            FRICTION_MIN_AUTOMATIC_POSITION_STEP_RAD
            <= abs(self.automatic_position_step_rad)
            <= FRICTION_MAX_AUTOMATIC_POSITION_STEP_RAD
        ):
            raise ValueError(
                "Модуль шага автоматического смещения должен быть от "
                f"{FRICTION_MIN_AUTOMATIC_POSITION_STEP_RAD:g} до "
                f"{FRICTION_MAX_AUTOMATIC_POSITION_STEP_RAD:g} рад"
            )
        if not 1 <= self.map_passes <= FRICTION_MAX_MAP_PASSES:
            raise ValueError(f"Число проходов карты должно быть от 1 до {FRICTION_MAX_MAP_PASSES}")
        if not (
            FRICTION_MIN_POSITION_TOLERANCE_RAD
            <= self.position_tolerance_rad
            <= FRICTION_MAX_POSITION_TOLERANCE_RAD
        ):
            raise ValueError(
                "Допуск автопозиции должен быть от "
                f"{FRICTION_MIN_POSITION_TOLERANCE_RAD:g} до "
                f"{FRICTION_MAX_POSITION_TOLERANCE_RAD:g} рад"
            )
        if not (
            FRICTION_MIN_POSITIONING_VOLTAGE_STEP_V
            <= self.positioning_voltage_step_v
            <= FRICTION_MAX_POSITIONING_VOLTAGE_STEP_V
        ):
            raise ValueError(
                "Шаг повышения Uq автосмещения должен быть от "
                f"{FRICTION_MIN_POSITIONING_VOLTAGE_STEP_V:g} до "
                f"{FRICTION_MAX_POSITIONING_VOLTAGE_STEP_V:g} В"
            )
        if not self.pulse_max_voltage_v <= self.positioning_voltage_max_v <= self.voltage_limit_v:
            raise ValueError(
                "Максимальный Uq автосмещения должен быть не ниже импульсного Uq "
                "и не выше предела напряжения опыта"
            )
        if not (
            FRICTION_MIN_POSITION_STALL_WINDOW_S
            <= self.position_stall_window_s
            <= FRICTION_MAX_POSITION_STALL_WINDOW_S
        ):
            raise ValueError(
                "Окно обнаружения остановки должно быть от "
                f"{FRICTION_MIN_POSITION_STALL_WINDOW_S:g} до "
                f"{FRICTION_MAX_POSITION_STALL_WINDOW_S:g} с"
            )
        if not 0.0001 <= self.position_min_progress_rad <= self.position_tolerance_rad:
            raise ValueError(
                "Минимальный прогресс автосмещения должен быть от 0,0001 рад "
                "до допуска автопозиции"
            )

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["algorithm_schema"] = 7
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
        values = {name: payload[name] for name in field_names if name in payload}
        schema = int(payload.get("algorithm_schema", 0) or 0)
        if schema < 4:
            if values.get("max_recovery_attempts") == 3:
                values["max_recovery_attempts"] = FRICTION_DEFAULT_RECOVERY_ATTEMPTS
            if values.get("movement_threshold_rad") == 0.002:
                values["movement_threshold_rad"] = 0.001
        if schema < 7 and "positioning_voltage_max_v" not in values:
            defaults = cls()
            voltage_limit = float(values.get("voltage_limit_v", defaults.voltage_limit_v))
            pulse_max = float(values.get("pulse_max_voltage_v", defaults.pulse_max_voltage_v))
            values["positioning_voltage_max_v"] = max(
                pulse_max,
                min(defaults.positioning_voltage_max_v, voltage_limit),
            )
        config = cls(**values)
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
    measured_current_detected: bool = True
    start_angle_rad: float | None = None
    end_angle_rad: float | None = None
    mean_angle_rad: float | None = None
    mean_voltage_q_v: float | None = None
    position_index: int = 0
    measurement_position_rad: float | None = None
    motion_valid: bool = False
    tracking_error_fraction: float | None = None
    transient_peak_velocity_rad_s: float | None = None
    transient_acceleration_rad_s2: float | None = None
    rise_time_s: float | None = None
    overshoot_fraction: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> FrictionPointResult:
        field_names = cls.__dataclass_fields__.keys()
        return cls(**{name: payload[name] for name in field_names if name in payload})


@dataclass(slots=True)
class PositionFrictionObservation:
    position_center_rad: float
    position_min_rad: float
    position_max_rad: float
    target_velocity_rad_s: float
    mean_velocity_rad_s: float
    mean_voltage_q_v: float
    mean_measured_current_q_a: float
    measured_torque_nm: float | None
    voltage_equivalent_current_a: float
    voltage_equivalent_torque_nm: float
    sample_count: int
    motion_valid: bool
    measured_current_detected: bool
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> PositionFrictionObservation:
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


def _measured_current_is_confirmed(
    currents: np.ndarray,
    floor_a: float,
    target_direction: float,
) -> bool:
    if currents.size < 3:
        return False
    above_floor = np.abs(currents) >= floor_a
    required = max(3, math.ceil(currents.size * FRICTION_CURRENT_CONFIRMATION_FRACTION))
    count = int(np.count_nonzero(above_floor))
    if count < required:
        return False
    aligned = currents[above_floor] * math.copysign(1.0, target_direction) >= floor_a
    return bool(np.count_nonzero(aligned) >= math.ceil(count * FRICTION_CURRENT_DIRECTION_FRACTION))


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


def _transient_response_metrics(
    samples: Iterable[TelemetrySample],
    target_velocity_rad_s: float,
) -> tuple[float | None, float | None, float | None, float | None]:
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
        return None, None, None, None
    timestamps = np.asarray([pair[0] for pair in pairs], dtype=float)
    angles = np.asarray([pair[1] for pair in pairs], dtype=float)
    unique = np.r_[True, np.diff(timestamps) > 0]
    timestamps = timestamps[unique]
    angles = angles[unique]
    if timestamps.size < 10 or timestamps[-1] - timestamps[0] < 0.2:
        return None, None, None, None

    slopes: list[tuple[float, float]] = []
    window_s = 0.2
    for start in range(timestamps.size):
        stop = int(np.searchsorted(timestamps, timestamps[start] + window_s, side="right"))
        if stop - start < 6:
            continue
        local_time = timestamps[start:stop] - timestamps[start]
        if local_time[-1] < window_s * 0.7:
            continue
        slopes.append(
            (
                float(timestamps[start] - timestamps[0] + local_time[-1] / 2.0),
                float(np.polyfit(local_time, angles[start:stop], 1)[0]),
            )
        )
    if len(slopes) < 3:
        return None, None, None, None

    direction = math.copysign(1.0, target_velocity_rad_s)
    signed_velocities = np.asarray([direction * value for _, value in slopes], dtype=float)
    peak_signed = float(np.max(signed_velocities))
    peak_velocity = direction * peak_signed
    target_abs = abs(target_velocity_rad_s)
    rise_time = next(
        (
            elapsed
            for (elapsed, _velocity), signed_velocity in zip(
                slopes,
                signed_velocities,
                strict=True,
            )
            if signed_velocity >= target_abs * 0.9
        ),
        None,
    )
    overshoot_fraction = max(0.0, peak_signed / target_abs - 1.0) if target_abs > 0 else None

    early = [
        (elapsed, direction * velocity)
        for elapsed, velocity in slopes
        if elapsed <= max(0.5, slopes[-1][0] * 0.5)
    ]
    acceleration = None
    if len(early) >= 3 and early[-1][0] - early[0][0] > 0.1:
        early_time = np.asarray([item[0] for item in early], dtype=float)
        early_velocity = np.asarray([item[1] for item in early], dtype=float)
        acceleration = direction * float(np.polyfit(early_time, early_velocity, 1)[0])
    return peak_velocity, acceleration, rise_time, overshoot_fraction


def summarize_friction_point(
    target_velocity_rad_s: float,
    steady_samples: Iterable[TelemetrySample],
    transient_samples: Iterable[TelemetrySample],
    torque_constant_nm_per_a: float,
    *,
    voltage_limit_v: float | None = None,
    measured_current_floor_a: float | None = None,
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
    angles = _trimmed(
        sample.angle_rad for sample in usable if sample.angle_rad is not None
    )
    voltages_q = _trimmed(
        sample.voltage_q_v for sample in usable if sample.voltage_q_v is not None
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
            measured_current_detected=False,
        )

    mean_current = float(np.mean(currents))
    current_std = float(np.std(currents))
    measured_current_detected = True
    if measured_current_floor_a is not None:
        measured_current_detected = _measured_current_is_confirmed(
            measured_currents,
            measured_current_floor_a,
            target_velocity_rad_s,
        )
    direction_ok = mean_velocity * target_velocity_rad_s > 0
    tracking_tolerance = max(0.005, abs(target_velocity_rad_s) * 0.3)
    tracking_error = abs(mean_velocity - target_velocity_rad_s)
    stable = velocity_std <= max(0.003, abs(target_velocity_rad_s) * 0.25)
    motion_valid = direction_ok and tracking_error <= tracking_tolerance and stable
    tracking_error_fraction = tracking_error / abs(target_velocity_rad_s)
    (
        transient_peak_velocity,
        transient_acceleration,
        rise_time,
        overshoot_fraction,
    ) = _transient_response_metrics(transient_samples, target_velocity_rad_s)

    transient_currents = _trimmed(
        abs(sample.current_q_a)
        for sample in transient_samples
        if sample.current_q_a is not None and math.isfinite(sample.current_q_a)
    )
    if transient_currents.size:
        breakaway = float(np.percentile(transient_currents, 95)) * torque_constant_nm_per_a
    else:
        breakaway = abs(mean_current) * torque_constant_nm_per_a

    valid = motion_valid and measured_current_detected
    problems: list[str] = []
    if not direction_ok:
        problems.append("направление не совпало с целью")
    if tracking_error > tracking_tolerance:
        problems.append("скорость не достигнута")
    if not stable:
        problems.append("скорость неустойчива")
    if not measured_current_detected:
        if direction_ok and tracking_error <= tracking_tolerance and stable:
            problems.append(
                "движение пригодно для карты Uq; измеренный Iq нельзя считать моментом"
            )
        else:
            problems.append("измеренный Iq не подтверждён устойчиво или имеет неверный знак")
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
        measured_current_detected=measured_current_detected,
        start_angle_rad=(
            next(
                (
                    float(sample.angle_rad)
                    for sample in usable
                    if sample.angle_rad is not None and math.isfinite(sample.angle_rad)
                ),
                None,
            )
        ),
        end_angle_rad=(
            next(
                (
                    float(sample.angle_rad)
                    for sample in reversed(usable)
                    if sample.angle_rad is not None and math.isfinite(sample.angle_rad)
                ),
                None,
            )
        ),
        mean_angle_rad=float(np.mean(angles)) if angles.size else None,
        mean_voltage_q_v=float(np.mean(voltages_q)) if voltages_q.size else None,
        motion_valid=motion_valid,
        tracking_error_fraction=tracking_error_fraction,
        transient_peak_velocity_rad_s=transient_peak_velocity,
        transient_acceleration_rad_s2=transient_acceleration,
        rise_time_s=rise_time,
        overshoot_fraction=overshoot_fraction,
    )


def summarize_position_observations(
    target_velocity_rad_s: float,
    samples: Iterable[TelemetrySample],
    torque_constant_nm_per_a: float,
    phase_resistance_ohm: float,
    back_emf_v_per_krpm: float,
    *,
    bin_width_rad: float,
    measured_current_floor_a: float,
) -> list[PositionFrictionObservation]:
    """Preserve local friction diagnostics instead of collapsing a whole sweep to one row.

    Measured-Iq torque remains the only physically accepted torque.  The voltage-derived value
    is retained as an explicitly diagnostic estimate because the present hardware current signal
    has repeatedly vanished in one direction or at particular shaft positions.
    """

    if phase_resistance_ohm <= 0:
        raise ValueError("Для позиционной карты нужно положительное сопротивление фазы")
    if not (
        FRICTION_MIN_POSITION_BIN_WIDTH_RAD
        <= bin_width_rad
        <= FRICTION_MAX_POSITION_BIN_WIDTH_RAD
    ):
        raise ValueError("Недопустимая ширина интервала позиционной карты")
    usable = [
        sample
        for sample in samples
        if sample.angle_rad is not None
        and sample.voltage_q_v is not None
        and sample.current_q_a is not None
        and math.isfinite(sample.timestamp_s)
        and math.isfinite(sample.angle_rad)
        and math.isfinite(sample.voltage_q_v)
        and math.isfinite(sample.current_q_a)
    ]
    grouped: dict[int, list[TelemetrySample]] = {}
    for sample in usable:
        assert sample.angle_rad is not None
        index = math.floor(sample.angle_rad / bin_width_rad)
        grouped.setdefault(index, []).append(sample)

    back_emf_v_per_rad_s = back_emf_v_per_krpm / (1000.0 * 2.0 * math.pi / 60.0)
    observations: list[PositionFrictionObservation] = []
    for index in sorted(grouped):
        group = sorted(grouped[index], key=lambda sample: sample.timestamp_s)
        unique_group: list[TelemetrySample] = []
        previous_timestamp: float | None = None
        for sample in group:
            if previous_timestamp is None or sample.timestamp_s > previous_timestamp:
                unique_group.append(sample)
                previous_timestamp = sample.timestamp_s
        if len(unique_group) < FRICTION_MIN_POSITION_BIN_SAMPLES:
            continue
        timestamps = np.asarray([sample.timestamp_s for sample in unique_group], dtype=float)
        angles = np.asarray([float(sample.angle_rad) for sample in unique_group], dtype=float)
        if timestamps[-1] - timestamps[0] < 0.05:
            continue
        mean_velocity = float(np.polyfit(timestamps - timestamps[0], angles, 1)[0])
        voltages = _trimmed(float(sample.voltage_q_v) for sample in unique_group)
        currents = _trimmed(float(sample.current_q_a) for sample in unique_group)
        mean_voltage = float(np.mean(voltages))
        mean_current = float(np.mean(currents))
        current_detected = _measured_current_is_confirmed(
            currents,
            measured_current_floor_a,
            target_velocity_rad_s,
        )
        tracking_tolerance = max(0.01, abs(target_velocity_rad_s) * 0.35)
        motion_valid = bool(
            mean_velocity * target_velocity_rad_s > 0
            and abs(mean_velocity - target_velocity_rad_s) <= tracking_tolerance
        )
        equivalent_current = (
            mean_voltage - back_emf_v_per_rad_s * mean_velocity
        ) / phase_resistance_ohm
        problems: list[str] = []
        if not motion_valid:
            problems.append("локальная скорость не достигнута")
        if not current_detected:
            problems.append("Iq не подтверждён; оценка по Uq только диагностическая")
        observations.append(
            PositionFrictionObservation(
                position_center_rad=(index + 0.5) * bin_width_rad,
                position_min_rad=float(np.min(angles)),
                position_max_rad=float(np.max(angles)),
                target_velocity_rad_s=target_velocity_rad_s,
                mean_velocity_rad_s=mean_velocity,
                mean_voltage_q_v=mean_voltage,
                mean_measured_current_q_a=mean_current,
                measured_torque_nm=(
                    abs(mean_current) * torque_constant_nm_per_a
                    if current_detected
                    else None
                ),
                voltage_equivalent_current_a=equivalent_current,
                voltage_equivalent_torque_nm=abs(equivalent_current)
                * torque_constant_nm_per_a,
                sample_count=len(unique_group),
                motion_valid=motion_valid,
                measured_current_detected=current_detected,
                note="ОК" if not problems else "; ".join(problems),
            )
        )
    return observations


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
        back_emf_v_per_krpm: float = 0.0,
        actuator_attempts: Iterable[ActuatorPulseResult] = (),
        position_observations: Iterable[PositionFrictionObservation] = (),
        baseline_diagnostics: Iterable[BaselineDiagnostic] = (),
        positioning_results: Iterable[PositioningResult] = (),
        position_targets_rad: Iterable[float] = (0.0,),
        position_index: int = 0,
        point_index: int | None = None,
    ) -> None:
        config.validate()
        if phase_resistance_ohm <= 0:
            raise ValueError("Для опыта нужно положительное сопротивление фазы")
        self.config = config
        self.torque_constant_nm_per_a = torque_constant_nm_per_a
        self.phase_resistance_ohm = phase_resistance_ohm
        self.back_emf_v_per_krpm = back_emf_v_per_krpm
        self.points = list(completed_points)
        self.actuator_attempts = list(actuator_attempts)
        self.position_observations = list(position_observations)
        self.baseline_diagnostics = list(baseline_diagnostics)
        self.positioning_results = list(positioning_results)
        self.position_targets_rad = tuple(float(value) for value in position_targets_rad)
        if not self.position_targets_rad:
            raise ValueError("Для опыта нужна хотя бы одна координата измерения")
        if not 0 <= position_index < len(self.position_targets_rad):
            raise ValueError("Неверный номер автоматической координаты")
        self.position_index = position_index
        self.phase = FrictionPhase.IDLE
        self.phase_started_s = 0.0
        completed_here = sum(
            point.position_index == self.position_index for point in self.points
        )
        self.point_index = completed_here - 1 if point_index is None else point_index
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
        self.rejected_angle_samples = 0
        self._trusted_angle_sample: TelemetrySample | None = None
        self._pending_angle_sample: TelemetrySample | None = None
        self._trusted_fast_angle_step: float | None = None
        self._movement_candidate = 0
        self._movement_candidate_samples = 0
        self._continuous_angle_reference_rad: float | None = None
        self._board_angle_offset_rad = 0.0
        self._positioning_started_s = 0.0
        self._positioning_start_angle_rad: float | None = None
        self._positioning_initial_voltage_limit_v = 0.0
        self._positioning_active_voltage_limit_v = 0.0
        self._positioning_voltage_boost_count = 0
        self._position_progress_reference_angle_rad: float | None = None
        self._position_progress_reference_s = 0.0

    @property
    def active(self) -> bool:
        return self.phase not in {FrictionPhase.COMPLETE, FrictionPhase.ABORTED}

    @property
    def current_target(self) -> float | None:
        if 0 <= self.point_index < len(self.config.targets):
            return self.config.targets[self.point_index]
        return None

    @property
    def current_position_target_rad(self) -> float:
        return self.position_targets_rad[self.position_index]

    @property
    def automatic_positioning_enabled(self) -> bool:
        return len(self.position_targets_rad) > 1

    @property
    def current_pulse_voltage_v(self) -> float | None:
        return self._pulse_target()

    @property
    def breakaway_results(self) -> dict[int, ActuatorPulseResult]:
        found: dict[int, ActuatorPulseResult] = {}
        for attempt in self.actuator_attempts:
            if attempt.position_index != self.position_index:
                continue
            if attempt.movement_detected:
                found.setdefault(attempt.direction, attempt)
        return found

    def breakaway_results_for_position(self, position_index: int) -> dict[int, ActuatorPulseResult]:
        found: dict[int, ActuatorPulseResult] = {}
        for attempt in self.actuator_attempts:
            if attempt.position_index != position_index:
                continue
            if attempt.movement_detected:
                found.setdefault(attempt.direction, attempt)
        return found

    @property
    def actuator_complete(self) -> bool:
        found = self.breakaway_results
        return all(direction in found for direction in (1, -1))

    @property
    def measured_current_complete(self) -> bool:
        found = self.breakaway_results
        return all(direction in found and found[direction].current_detected for direction in (1, -1))

    @property
    def all_actuator_positions_complete(self) -> bool:
        return all(
            all(
                direction in self.breakaway_results_for_position(position_index)
                for direction in (1, -1)
            )
            for position_index in range(len(self.position_targets_rad))
        )

    @property
    def all_measured_current_positions_complete(self) -> bool:
        return all(
            all(
                direction in found and found[direction].current_detected
                for direction in (1, -1)
            )
            for found in (
                self.breakaway_results_for_position(position_index)
                for position_index in range(len(self.position_targets_rad))
            )
        )

    @property
    def configuration_mode(self) -> str:
        if self.phase == FrictionPhase.RECOVERING:
            return self._recovery_mode
        if self.phase in {
            FrictionPhase.CONFIGURING_POSITION,
            FrictionPhase.POSITIONING,
            FrictionPhase.POSITION_SETTLING,
        }:
            return "position"
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

    @property
    def positioning_current_limit_a(self) -> float | None:
        moving_attempts = [
            attempt
            for attempt in self.actuator_attempts
            if attempt.movement_detected
        ]
        if not moving_attempts:
            return None
        voltage = max(abs(attempt.command_voltage_v) for attempt in moving_attempts)
        start_voltage = min(
            voltage * self.config.breakaway_margin,
            self.config.positioning_voltage_max_v,
        )
        active_voltage = self._positioning_active_voltage_limit_v or start_voltage
        return active_voltage / self.phase_resistance_ohm

    @property
    def positioning_voltage_limit_v(self) -> float | None:
        current_limit = self.positioning_current_limit_a
        if current_limit is None:
            return None
        return current_limit * self.phase_resistance_ohm

    def seed_angle(
        self,
        angle_rad: float | None,
        *,
        continuous_reference_rad: float | None = None,
    ) -> None:
        if angle_rad is not None and math.isfinite(angle_rad):
            raw_angle = float(angle_rad)
            if continuous_reference_rad is None:
                continuous_angle = raw_angle
            else:
                continuous_angle = raw_angle + math.tau * round(
                    (continuous_reference_rad - raw_angle) / math.tau
                )
            self._continuous_angle_reference_rad = continuous_angle
            self._board_angle_offset_rad = continuous_angle - raw_angle

    def board_target_for_continuous(self, continuous_target_rad: float) -> float:
        return continuous_target_rad - self._board_angle_offset_rad

    def start(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.IDLE:
            raise RuntimeError("Friction experiment has already been started")
        self.phase_started_s = now_s
        self.phase_samples.clear()
        self._reset_angle_filter()
        completed_here = sum(
            point.position_index == self.position_index for point in self.points
        )
        if completed_here >= len(self.config.targets):
            if self.position_index + 1 >= len(self.position_targets_rad):
                self.phase = FrictionPhase.COMPLETE
                return [FrictionAction("finish")]
            self.position_index += 1
            self.point_index = -1
            self._reset_positioning_state()
            self.phase = FrictionPhase.CONFIGURING_POSITION
            return [FrictionAction("configure_position")]
        if (
            self._continuous_angle_reference_rad is not None
            and abs(
                self._continuous_angle_reference_rad - self.current_position_target_rad
            )
            > self.config.position_tolerance_rad
        ):
            self._reset_positioning_state()
            self.phase = FrictionPhase.CONFIGURING_POSITION
            return [FrictionAction("configure_position")]
        if self.actuator_complete:
            self.phase = FrictionPhase.ZERO
        else:
            self.phase = FrictionPhase.ACTUATOR_BASELINE
        return [FrictionAction("target", 0.0)]

    def add_sample(
        self,
        sample: TelemetrySample,
        now_s: float | None = None,
        *,
        angle_prepared: bool = False,
    ) -> tuple[str | None, list[FrictionAction]]:
        event_time_s = sample.timestamp_s if now_s is None else now_s
        if self.phase == FrictionPhase.IDLE:
            return None, []
        if not angle_prepared:
            sample = self.prepare_sample(sample)
        if self.phase == FrictionPhase.RECOVERING:
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
        elif self.phase in {FrictionPhase.POSITIONING, FrictionPhase.POSITION_SETTLING}:
            self.phase_samples.append(sample)
            if self.phase == FrictionPhase.POSITIONING:
                self._update_position_progress(sample, event_time_s)
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
            self._record_baseline()
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
                working_limit = self.working_current_limit_a
                if working_limit is None:
                    return self.abort("Не удалось рассчитать командный предел скоростного этапа")
                self.phase = FrictionPhase.CONFIGURING_VELOCITY
                self.phase_started_s = now_s
                return [FrictionAction("configure_velocity"), FrictionAction("checkpoint")]
            return self._start_next_pulse(now_s)
        if self.phase == FrictionPhase.POSITIONING:
            if elapsed >= FRICTION_POSITION_TIMEOUT_S:
                return self._abort_positioning(
                    "превышено предельное время "
                    f"{FRICTION_POSITION_TIMEOUT_S:g} с"
                )
            latest = self._latest_angle(self.phase_samples)
            if (
                latest is not None
                and abs(latest - self.current_position_target_rad)
                <= self.config.position_tolerance_rad
            ):
                self.phase = FrictionPhase.POSITION_SETTLING
                self.phase_started_s = now_s
                return []
            if (
                latest is not None
                and now_s - self._position_progress_reference_s
                >= self.config.position_stall_window_s
            ):
                active_voltage = self.positioning_voltage_limit_v
                if active_voltage is None:
                    return self._abort_positioning("не удалось определить предел автосмещения")
                saturated = self._position_is_saturated(active_voltage)
                if saturated and active_voltage < self.config.positioning_voltage_max_v - 1e-12:
                    next_voltage = min(
                        active_voltage + self.config.positioning_voltage_step_v,
                        self.config.positioning_voltage_max_v,
                    )
                    self._positioning_active_voltage_limit_v = next_voltage
                    self._positioning_voltage_boost_count += 1
                    self._position_progress_reference_angle_rad = latest
                    self._position_progress_reference_s = now_s
                    return [
                        FrictionAction(
                            "position_limit",
                            next_voltage / self.phase_resistance_ohm,
                        ),
                        FrictionAction("checkpoint"),
                    ]
                if saturated:
                    return self._abort_positioning(
                        "вал остановился при достигнутом максимальном Uq автосмещения "
                        f"{active_voltage:.6g} В"
                    )
                return self._abort_positioning(
                    "вал остановился без насыщения Uq; причина вероятнее в позиционном/"
                    "скоростном регуляторе или режиме управления, а не в пределе напряжения"
                )
            return []
        if self.phase == FrictionPhase.POSITION_SETTLING:
            latest = self._latest_angle(self.phase_samples)
            if latest is None:
                return []
            if abs(latest - self.current_position_target_rad) > self.config.position_tolerance_rad:
                self.phase = FrictionPhase.POSITIONING
                self.phase_started_s = self._positioning_started_s
                self._position_progress_reference_angle_rad = latest
                self._position_progress_reference_s = now_s
                return []
            if now_s - self._positioning_started_s >= FRICTION_POSITION_TIMEOUT_S:
                return self._abort_positioning("координата не установилась за предельное время")
            if elapsed >= FRICTION_POSITION_SETTLE_S and self._position_is_settled():
                self._record_positioning_result(now_s, reached=True)
                self.phase = FrictionPhase.CONFIGURING_ACTUATOR
                self.phase_started_s = now_s
                return [FrictionAction("configure_actuator"), FrictionAction("checkpoint")]
            return []
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
            point = summarize_friction_point(
                target,
                self.steady_samples,
                self.transient_samples,
                self.torque_constant_nm_per_a,
                voltage_limit_v=self.config.voltage_limit_v,
                measured_current_floor_a=self.current_detection_threshold_a,
            )
            point.position_index = self.position_index
            point.measurement_position_rad = self.current_position_target_rad
            self.points.append(point)
            self.position_observations.extend(
                summarize_position_observations(
                    target,
                    self.steady_samples,
                    self.torque_constant_nm_per_a,
                    self.phase_resistance_ohm,
                    self.back_emf_v_per_krpm,
                    bin_width_rad=self.config.position_bin_width_rad,
                    measured_current_floor_a=self.current_detection_threshold_a,
                )
            )
            self.phase = FrictionPhase.PAUSE
            self.phase_started_s = now_s
            return [FrictionAction("target", 0.0), FrictionAction("checkpoint")]
        if self.phase == FrictionPhase.PAUSE and elapsed >= self.config.pause_s:
            if self.point_index + 1 >= len(self.config.targets):
                if self.position_index + 1 >= len(self.position_targets_rad):
                    self.phase = FrictionPhase.COMPLETE
                    return [FrictionAction("finish")]
                self.position_index += 1
                self.point_index = -1
                self._reset_positioning_state()
                self.phase = FrictionPhase.CONFIGURING_POSITION
                self.phase_started_s = now_s
                self.phase_samples.clear()
                return [FrictionAction("configure_position"), FrictionAction("checkpoint")]
            return self._start_velocity_point(now_s, increment=True)
        return []

    def velocity_configuration_applied(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.CONFIGURING_VELOCITY:
            return []
        self.phase = FrictionPhase.ZERO
        self.phase_started_s = now_s
        self.phase_samples.clear()
        return [FrictionAction("target", 0.0)]

    def position_configuration_applied(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.CONFIGURING_POSITION:
            return []
        self.phase = FrictionPhase.POSITIONING
        self.phase_started_s = now_s
        self._positioning_started_s = now_s
        start_angle = self._continuous_angle_reference_rad
        initial_voltage = self.positioning_voltage_limit_v
        if initial_voltage is None:
            return self.abort("Не удалось определить стартовый предел автосмещения")
        self._positioning_start_angle_rad = start_angle
        self._positioning_initial_voltage_limit_v = initial_voltage
        self._positioning_active_voltage_limit_v = initial_voltage
        self._positioning_voltage_boost_count = 0
        self._position_progress_reference_angle_rad = start_angle
        self._position_progress_reference_s = now_s
        self.phase_samples.clear()
        self.angle_window.clear()
        return [FrictionAction("position_target", self.current_position_target_rad)]

    def actuator_configuration_applied(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.CONFIGURING_ACTUATOR:
            return []
        self.phase = FrictionPhase.ACTUATOR_BASELINE
        self.phase_started_s = now_s
        self.phase_samples.clear()
        self.angle_window.clear()
        self._reset_angle_filter()
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
        self.angle_window.clear()
        self._reset_angle_filter()
        return [FrictionAction("safe_stop")]

    def resume_after_recovery(self, now_s: float) -> list[FrictionAction]:
        if self.phase != FrictionPhase.RECOVERING:
            return []
        self.phase_started_s = now_s
        self.phase_samples.clear()
        self.angle_window.clear()
        self._reset_angle_filter()
        if self._recovery_mode == "velocity":
            self.phase = FrictionPhase.ZERO
        elif self._recovery_mode == "position":
            self.phase = FrictionPhase.POSITIONING
            self._positioning_started_s = now_s
            self._positioning_start_angle_rad = self._continuous_angle_reference_rad
            self._position_progress_reference_angle_rad = self._continuous_angle_reference_rad
            self._position_progress_reference_s = now_s
        else:
            self.phase = FrictionPhase.ACTUATOR_BASELINE
        target = (
            self.current_position_target_rad
            if self._recovery_mode == "position"
            else 0.0
        )
        action_kind = "position_target" if self._recovery_mode == "position" else "target"
        return [FrictionAction(action_kind, target)]

    def abort(self, reason: str) -> list[FrictionAction]:
        self.abort_reason = reason
        self.phase = FrictionPhase.ABORTED
        return [FrictionAction("safe_stop")]

    def estimate(self) -> FrictionEstimate:
        estimate = estimate_friction(self.points)
        directional_torques: dict[int, list[float]] = {1: [], -1: []}
        missing: list[str] = []
        for position_index, position_target in enumerate(self.position_targets_rad):
            found = self.breakaway_results_for_position(position_index)
            for direction in (1, -1):
                attempt = found.get(direction)
                if attempt is None or not attempt.current_detected:
                    missing.append(
                        f"{position_target:.6g} рад/"
                        + ("+" if direction > 0 else "−")
                    )
                    continue
                directional_torques[direction].append(
                    attempt.mean_abs_measured_current_a * self.torque_constant_nm_per_a
                )
        positive = directional_torques[1]
        negative = directional_torques[-1]
        if positive:
            estimate.breakaway_positive_nm = float(np.mean(positive))
        if negative:
            estimate.breakaway_negative_nm = float(np.mean(negative))
        if positive and negative:
            estimate.breakaway_friction_nm = (
                estimate.breakaway_positive_nm + estimate.breakaway_negative_nm
            ) / 2.0
        if missing:
            estimate.valid = False
            estimate.note = (
                "Мотор и карта по Uq пригодны для продолжения, но физический момент по Iq "
                "нельзя принимать для точек "
                + ", ".join(missing)
                + ": измеренный Iq отсутствует или имеет неверный знак"
            )
        return estimate

    def diagnostic_report(self) -> dict[str, Any]:
        findings: list[dict[str, Any]] = []

        def finding(
            severity: str,
            code: str,
            conclusion: str,
            evidence: dict[str, Any],
            next_action: str,
        ) -> None:
            findings.append(
                {
                    "severity": severity,
                    "code": code,
                    "conclusion": conclusion,
                    "evidence": evidence,
                    "next_action": next_action,
                }
            )

        moving_attempts = [
            attempt for attempt in self.actuator_attempts if attempt.movement_detected
        ]
        breakaway_by_direction = {
            direction: [
                abs(attempt.command_voltage_v)
                for attempt in moving_attempts
                if attempt.direction == direction
            ]
            for direction in (1, -1)
        }
        breakaway_envelope: dict[str, dict[str, float | None]] = {}
        maximum_position_ratio = 1.0
        for direction, values in breakaway_by_direction.items():
            minimum = min(values) if values else None
            maximum = max(values) if values else None
            ratio = maximum / minimum if minimum and maximum is not None else None
            if ratio is not None:
                maximum_position_ratio = max(maximum_position_ratio, ratio)
            breakaway_envelope["positive" if direction > 0 else "negative"] = {
                "min_voltage_v": minimum,
                "max_voltage_v": maximum,
                "max_to_min_ratio": ratio,
            }
        if maximum_position_ratio >= 1.5:
            finding(
                "high",
                "position_dependent_breakaway",
                "Трение страгивания сильно зависит от координаты; один постоянный коэффициент "
                "не описывает привод.",
                {"maximum_breakaway_ratio": maximum_position_ratio},
                "Использовать в модели координатную карту или минимум/максимум и робастную настройку.",
            )
        elif maximum_position_ratio >= 1.2:
            finding(
                "medium",
                "position_dependent_breakaway",
                "Трение страгивания заметно меняется по координате.",
                {"maximum_breakaway_ratio": maximum_position_ratio},
                "Сохранить координатную зависимость как минимум для проверки крайних случаев.",
            )

        positive = breakaway_by_direction[1]
        negative = breakaway_by_direction[-1]
        direction_asymmetry_percent = None
        if positive and negative:
            positive_mean = float(np.mean(positive))
            negative_mean = float(np.mean(negative))
            denominator = (positive_mean + negative_mean) / 2.0
            if denominator > 0:
                direction_asymmetry_percent = (
                    abs(positive_mean - negative_mean) / denominator * 100.0
                )
            if direction_asymmetry_percent is not None and direction_asymmetry_percent >= 25.0:
                finding(
                    "high",
                    "directional_asymmetry",
                    "Для положительного и отрицательного движения требуется существенно разное "
                    "воздействие.",
                    {
                        "positive_mean_voltage_v": positive_mean,
                        "negative_mean_voltage_v": negative_mean,
                        "asymmetry_percent": direction_asymmetry_percent,
                    },
                    "Задавать раздельные положительный и отрицательный Coulomb/страгивание.",
                )

        repeat_groups: dict[tuple[float, int], list[float]] = {}
        for attempt in moving_attempts:
            coordinate = (
                attempt.measurement_position_rad
                if attempt.measurement_position_rad is not None
                else attempt.start_angle_rad
            )
            if coordinate is None:
                continue
            repeat_groups.setdefault((round(float(coordinate), 6), attempt.direction), []).append(
                abs(attempt.command_voltage_v)
            )
        repeatability_ratios = [
            max(values) / min(values)
            for values in repeat_groups.values()
            if len(values) >= 2 and min(values) > 0
        ]
        worst_repeatability_ratio = max(repeatability_ratios, default=None)
        if worst_repeatability_ratio is not None and worst_repeatability_ratio >= 1.3:
            finding(
                "high",
                "poor_breakaway_repeatability",
                "Даже в одной координате порог страгивания повторяется плохо; присутствует "
                "стохастическое трение, нагрев, люфт или меняющийся преднатяг.",
                {"worst_repeat_max_to_min_ratio": worst_repeatability_ratio},
                "Оптимизировать по диапазону, а не по одной кривой, и проверить механический люфт.",
            )

        failed_positioning = [result for result in self.positioning_results if not result.reached]
        boosted_positioning = [
            result for result in self.positioning_results if result.voltage_boost_count > 0
        ]
        if failed_positioning:
            saturated_failures = [
                result for result in failed_positioning if result.saturated_at_end
            ]
            if saturated_failures:
                worst = max(
                    saturated_failures,
                    key=lambda item: abs(item.final_error_rad or 0.0),
                )
                finding(
                    "critical",
                    "positioning_torque_limit",
                    "Автоматическая координата не достигнута из-за насыщения доступного Uq.",
                    {
                        "target_rad": worst.target_position_rad,
                        "final_rad": worst.final_position_rad,
                        "residual_error_rad": worst.final_error_rad,
                        "final_voltage_limit_v": worst.final_voltage_limit_v,
                    },
                    "Повысить отдельный безопасный максимум Uq автосмещения либо уменьшить шаг; "
                    "PID сам по себе эту механическую остановку не устранит.",
                )
            if any(not result.saturated_at_end for result in failed_positioning):
                worst = next(
                    result for result in failed_positioning if not result.saturated_at_end
                )
                finding(
                    "critical",
                    "positioning_controller_path",
                    "Позиционирование остановилось без насыщения Uq; ограничение момента не является "
                    "главной причиной.",
                    {
                        "target_rad": worst.target_position_rad,
                        "final_rad": worst.final_position_rad,
                        "residual_error_rad": worst.final_error_rad,
                        "final_voltage_limit_v": worst.final_voltage_limit_v,
                    },
                    "Проверить angle P, velocity PI, их лимиты, знак и реально применённый режим.",
                )
        elif boosted_positioning:
            finding(
                "medium",
                "adaptive_positioning_required",
                "Часть координат достигнута только после повышения Uq: локальное трение больше "
                "первоначального порога страгивания.",
                {
                    "boosted_moves": len(boosted_positioning),
                    "maximum_boost_count": max(
                        result.voltage_boost_count for result in boosted_positioning
                    ),
                    "maximum_final_voltage_v": max(
                        result.final_voltage_limit_v for result in boosted_positioning
                    ),
                },
                "Сохранить максимум как рабочую границу позиционирования и учитывать карту трения.",
            )

        motion_points = [point for point in self.points if point.motion_valid]
        motion_fraction = len(motion_points) / len(self.points) if self.points else 0.0
        current_points = [point for point in self.points if point.measured_current_detected]
        current_fraction = len(current_points) / len(self.points) if self.points else 0.0
        if self.points and motion_fraction < 0.8:
            failed = [point for point in self.points if not point.motion_valid]
            saturation_failures = 0
            for point in failed:
                local = self.breakaway_results_for_position(point.position_index)
                local_voltage_limit = (
                    max(abs(item.command_voltage_v) for item in local.values())
                    * self.config.breakaway_margin
                    if local
                    else 0.0
                )
                if (
                    point.mean_voltage_q_v is not None
                    and local_voltage_limit > 0
                    and abs(point.mean_voltage_q_v)
                    >= local_voltage_limit * FRICTION_POSITION_SATURATION_FRACTION
                ):
                    saturation_failures += 1
            finding(
                "high",
                "velocity_tracking_failed",
                "Скоростной контур не выполнил значительную часть заданных скоростей.",
                {
                    "motion_valid_fraction": motion_fraction,
                    "failed_points": len(failed),
                    "failed_while_saturated": saturation_failures,
                },
                (
                    "Увеличить рабочий предел скоростного этапа и повторить."
                    if saturation_failures >= max(1, len(failed) // 2)
                    else "Проверить коэффициенты velocity PI и знак обратной связи."
                ),
            )
        if self.points and current_fraction < 0.8:
            finding(
                "critical",
                "measured_iq_unreliable",
                "Измеренный Iq отсутствует, слишком редко появляется или имеет неверный знак; "
                "абсолютный момент и инерцию по нему определять нельзя.",
                {
                    "confirmed_velocity_points": len(current_points),
                    "total_velocity_points": len(self.points),
                    "confirmed_fraction": current_fraction,
                },
                "Исправить измерение/знак Iq; до этого использовать Uq только как относительный "
                "диагностический показатель.",
            )

        baseline_angle_noise = [
            diagnostic.angle_std_rad
            for diagnostic in self.baseline_diagnostics
            if diagnostic.angle_std_rad is not None
        ]
        baseline_drift = [
            abs(diagnostic.angle_drift_rad_s)
            for diagnostic in self.baseline_diagnostics
            if diagnostic.angle_drift_rad_s is not None
        ]
        baseline_current_offset = [
            abs(diagnostic.mean_current_q_a)
            for diagnostic in self.baseline_diagnostics
            if diagnostic.mean_current_q_a is not None
        ]
        if any(diagnostic.note != "ОК" for diagnostic in self.baseline_diagnostics):
            finding(
                "medium",
                "zero_baseline_unstable",
                "На спокойном нуле обнаружен шум, дрейф координаты или смещение Iq.",
                {
                    "max_angle_std_rad": max(baseline_angle_noise, default=None),
                    "max_abs_angle_drift_rad_s": max(baseline_drift, default=None),
                    "max_abs_current_offset_a": max(baseline_current_offset, default=None),
                },
                "Проверить энкодер, жёсткость механики и калибровку датчиков до точной идентификации.",
            )
        if self.interruption_count:
            finding(
                "medium",
                "telemetry_interruptions",
                "Во время автоматического опыта прерывалась телеметрия.",
                {
                    "interruption_count": self.interruption_count,
                    "rejected_angle_samples": self.rejected_angle_samples,
                },
                "Сначала подтвердить повторяемость на прогоне без обрывов либо учитывать их в "
                "неопределённости.",
            )

        estimate = self.estimate()
        friction_model = ["directional_coulomb", "viscous"]
        if maximum_position_ratio >= 1.2:
            friction_model.append("position_lookup_or_envelope")
        if worst_repeatability_ratio is not None and worst_repeatability_ratio >= 1.3:
            friction_model.append("uncertainty_range")
        if estimate.r_squared is not None and estimate.r_squared < 0.8:
            friction_model.append("nonlinear_velocity_term")

        critical = [item for item in findings if item["severity"] == "critical"]
        if critical:
            verdict = "Есть блокирующие причины; принимать единственную модель мотора пока нельзя."
        elif findings:
            verdict = "Опыт пригоден для построения диапазонов, но модель должна учитывать найденные особенности."
        else:
            verdict = "Явных аппаратных проблем в собранных тестах не обнаружено."
        inertia_ready = bool(
            current_fraction >= 0.8
            and sum(
                point.transient_acceleration_rad_s2 is not None for point in self.points
            )
            >= 4
        )
        return {
            "schema": 1,
            "verdict": verdict,
            "tests": {
                "zero_baseline": {
                    "completed": len(self.baseline_diagnostics),
                    "observations": [
                        diagnostic.to_dict() for diagnostic in self.baseline_diagnostics
                    ],
                },
                "breakaway_map": {
                    "completed_directional_points": len(moving_attempts),
                    "envelope": breakaway_envelope,
                    "direction_asymmetry_percent": direction_asymmetry_percent,
                    "worst_repeatability_ratio": worst_repeatability_ratio,
                },
                "positioning": {
                    "completed_moves": len(self.positioning_results),
                    "failed_moves": len(failed_positioning),
                    "adaptive_moves": len(boosted_positioning),
                    "results": [result.to_dict() for result in self.positioning_results],
                },
                "velocity_response": {
                    "completed_points": len(self.points),
                    "motion_valid_fraction": motion_fraction,
                    "measured_current_confirmed_fraction": current_fraction,
                    "points": [point.to_dict() for point in self.points],
                },
                "telemetry": {
                    "interruption_count": self.interruption_count,
                    "rejected_angle_samples": self.rejected_angle_samples,
                },
            },
            "findings": findings,
            "model_recommendation": {
                "friction_terms": friction_model,
                "use_measured_iq_as_torque": current_fraction >= 0.8,
                "absolute_inertia_identification_ready": inertia_ready,
                "robust_range_required": (
                    maximum_position_ratio >= 1.2
                    or (
                        worst_repeatability_ratio is not None
                        and worst_repeatability_ratio >= 1.3
                    )
                ),
            },
        }

    def checkpoint_payload(self, experiment_id: int | None) -> dict[str, Any]:
        return {
            "schema": 7,
            "experiment_id": experiment_id,
            "phase": self.phase.value,
            "config": self.config.to_dict(),
            "position_targets_rad": list(self.position_targets_rad),
            "position_index": self.position_index,
            "point_index": self.point_index,
            "motor_parameters": {
                "torque_constant_nm_per_a": self.torque_constant_nm_per_a,
                "phase_resistance_ohm": self.phase_resistance_ohm,
                "back_emf_v_per_krpm": self.back_emf_v_per_krpm,
            },
            "actuator_attempts": [attempt.to_dict() for attempt in self.actuator_attempts],
            "completed_points": [point.to_dict() for point in self.points],
            "position_observations": [
                observation.to_dict() for observation in self.position_observations
            ],
            "baseline_diagnostics": [
                diagnostic.to_dict() for diagnostic in self.baseline_diagnostics
            ],
            "positioning_results": [
                result.to_dict() for result in self.positioning_results
            ],
            "interruption_count": self.interruption_count,
            "rejected_angle_samples": self.rejected_angle_samples,
            "abort_reason": self.abort_reason,
        }

    def _reset_positioning_state(self) -> None:
        self._positioning_started_s = 0.0
        self._positioning_start_angle_rad = None
        self._positioning_initial_voltage_limit_v = 0.0
        self._positioning_active_voltage_limit_v = 0.0
        self._positioning_voltage_boost_count = 0
        self._position_progress_reference_angle_rad = None
        self._position_progress_reference_s = 0.0

    def _record_baseline(self) -> None:
        if any(
            diagnostic.position_index == self.position_index
            for diagnostic in self.baseline_diagnostics
        ):
            return
        usable = [
            sample
            for sample in self.phase_samples
            if math.isfinite(sample.timestamp_s)
        ]
        angles = np.asarray(
            [
                float(sample.angle_rad)
                for sample in usable
                if sample.angle_rad is not None and math.isfinite(sample.angle_rad)
            ],
            dtype=float,
        )
        currents = np.asarray(
            [
                float(sample.current_q_a)
                for sample in usable
                if sample.current_q_a is not None and math.isfinite(sample.current_q_a)
            ],
            dtype=float,
        )
        voltages = np.asarray(
            [
                float(sample.voltage_q_v)
                for sample in usable
                if sample.voltage_q_v is not None and math.isfinite(sample.voltage_q_v)
            ],
            dtype=float,
        )
        duration_s = (
            max(0.0, usable[-1].timestamp_s - usable[0].timestamp_s)
            if len(usable) >= 2
            else 0.0
        )
        angle_drift = None
        if angles.size >= 3 and duration_s > 0:
            angle_times = np.asarray(
                [
                    sample.timestamp_s
                    for sample in usable
                    if sample.angle_rad is not None and math.isfinite(sample.angle_rad)
                ],
                dtype=float,
            )
            angle_drift = float(np.polyfit(angle_times - angle_times[0], angles, 1)[0])
        problems: list[str] = []
        angle_std = float(np.std(angles)) if angles.size else None
        current_mean = float(np.mean(currents)) if currents.size else None
        current_std = float(np.std(currents)) if currents.size else None
        if angle_std is not None and angle_std > max(0.0005, self.config.position_tolerance_rad * 0.2):
            problems.append("повышенный шум координаты")
        if angle_drift is not None and abs(angle_drift) > 0.001:
            problems.append("дрейф координаты при нулевой команде")
        if current_mean is not None and abs(current_mean) > self.config.measured_current_floor_a:
            problems.append("смещение Iq выше порога подтверждения")
        if current_std is not None and current_std > self.config.measured_current_floor_a:
            problems.append("шум Iq выше порога подтверждения")
        self.baseline_diagnostics.append(
            BaselineDiagnostic(
                position_index=self.position_index,
                measurement_position_rad=self.current_position_target_rad,
                sample_count=len(usable),
                duration_s=duration_s,
                mean_angle_rad=float(np.mean(angles)) if angles.size else None,
                angle_std_rad=angle_std,
                angle_drift_rad_s=angle_drift,
                mean_current_q_a=current_mean,
                current_std_a=current_std,
                mean_voltage_q_v=float(np.mean(voltages)) if voltages.size else None,
                note="ОК" if not problems else "; ".join(problems),
            )
        )

    def _update_position_progress(self, sample: TelemetrySample, now_s: float) -> None:
        angle = sample.angle_rad
        reference = self._position_progress_reference_angle_rad
        start = self._positioning_start_angle_rad
        if angle is None or reference is None or start is None:
            return
        direction = math.copysign(1.0, self.current_position_target_rad - start)
        if direction * (angle - reference) >= self.config.position_min_progress_rad:
            self._position_progress_reference_angle_rad = angle
            self._position_progress_reference_s = now_s

    def _position_is_saturated(self, voltage_limit_v: float) -> bool:
        if voltage_limit_v <= 0 or not self.phase_samples:
            return False
        latest_timestamp = self.phase_samples[-1].timestamp_s
        voltages = np.asarray(
            [
                abs(float(sample.voltage_q_v))
                for sample in self.phase_samples
                if sample.voltage_q_v is not None
                and math.isfinite(sample.voltage_q_v)
                and sample.timestamp_s >= latest_timestamp - min(
                    1.0,
                    self.config.position_stall_window_s,
                )
            ],
            dtype=float,
        )
        return bool(
            voltages.size >= 3
            and float(np.median(voltages))
            >= voltage_limit_v * FRICTION_POSITION_SATURATION_FRACTION
        )

    def _record_positioning_result(
        self,
        now_s: float,
        *,
        reached: bool,
        note: str = "",
    ) -> None:
        if any(result.position_index == self.position_index for result in self.positioning_results):
            return
        start = self._positioning_start_angle_rad
        target = self.current_position_target_rad
        latest = self._latest_angle(self.phase_samples)
        direction = (
            0
            if start is None or math.isclose(target, start, rel_tol=0.0, abs_tol=1e-12)
            else (1 if target > start else -1)
        )
        angles = [
            float(sample.angle_rad)
            for sample in self.phase_samples
            if sample.angle_rad is not None and math.isfinite(sample.angle_rad)
        ]
        voltages = [
            float(sample.voltage_q_v)
            for sample in self.phase_samples
            if sample.voltage_q_v is not None and math.isfinite(sample.voltage_q_v)
        ]
        latest_timestamp = (
            self.phase_samples[-1].timestamp_s if self.phase_samples else None
        )
        hold_samples = (
            [
                sample
                for sample in self.phase_samples
                if latest_timestamp is not None and sample.timestamp_s >= latest_timestamp - 1.0
            ]
            if latest_timestamp is not None
            else []
        )
        hold_voltages = [
            float(sample.voltage_q_v)
            for sample in hold_samples
            if sample.voltage_q_v is not None and math.isfinite(sample.voltage_q_v)
        ]
        hold_currents = [
            float(sample.current_q_a)
            for sample in hold_samples
            if sample.current_q_a is not None and math.isfinite(sample.current_q_a)
        ]
        overshoot = None
        if direction and angles:
            overshoot = max(0.0, max(direction * (angle - target) for angle in angles))
        active_voltage = self.positioning_voltage_limit_v or 0.0
        saturated_at_end = self._position_is_saturated(active_voltage)
        if not note:
            note = (
                "координата достигнута"
                if self._positioning_voltage_boost_count == 0
                else "координата достигнута после адаптивного повышения Uq"
            )
        self.positioning_results.append(
            PositioningResult(
                position_index=self.position_index,
                start_position_rad=start,
                target_position_rad=target,
                final_position_rad=latest,
                final_error_rad=None if latest is None else target - latest,
                duration_s=max(0.0, now_s - self._positioning_started_s),
                reached=reached,
                initial_voltage_limit_v=self._positioning_initial_voltage_limit_v,
                final_voltage_limit_v=active_voltage,
                voltage_boost_count=self._positioning_voltage_boost_count,
                maximum_measured_voltage_v=max((abs(value) for value in voltages), default=None),
                hold_voltage_q_v=(
                    float(np.median(hold_voltages)) if hold_voltages else None
                ),
                hold_current_q_a=(
                    float(np.median(hold_currents)) if hold_currents else None
                ),
                overshoot_rad=overshoot,
                approach_direction=direction,
                saturated_at_end=saturated_at_end,
                note=note,
            )
        )

    def _abort_positioning(self, reason: str) -> list[FrictionAction]:
        now_s = self.phase_started_s + max(
            0.0,
            (
                self.phase_samples[-1].timestamp_s - self.phase_samples[0].timestamp_s
                if len(self.phase_samples) >= 2
                else 0.0
            ),
        )
        self._record_positioning_result(now_s, reached=False, note=reason)
        latest = self._latest_angle(self.phase_samples)
        residual = (
            "неизвестна"
            if latest is None
            else f"{abs(self.current_position_target_rad - latest):.6g} рад"
        )
        return self.abort(
            "Автоматическое смещение к "
            f"{self.current_position_target_rad:.6g} рад остановлено: {reason}; "
            f"остаточная ошибка {residual}"
        )

    def _start_next_pulse(self, now_s: float) -> list[FrictionAction]:
        found = self.breakaway_results
        attempted = {
            (attempt.direction, round(abs(attempt.command_voltage_v), 12))
            for attempt in self.actuator_attempts
            if attempt.position_index == self.position_index
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
                self._movement_candidate = 0
                self._movement_candidate_samples = 0
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
        required_samples = 3 if currents.size >= 3 else 2
        current_detected = currents.size >= 2 and above_threshold >= required_samples
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
                start_angle_rad=self.pulse_start_angle,
                end_angle_rad=latest,
                position_index=self.position_index,
                measurement_position_rad=self.current_position_target_rad,
            )
        )

    def _pulse_target(self) -> float | None:
        if self.phase != FrictionPhase.ACTUATOR_PULSE:
            return None
        attempted = {
            (attempt.direction, round(abs(attempt.command_voltage_v), 12))
            for attempt in self.actuator_attempts
            if attempt.position_index == self.position_index
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
        detected = 0
        movement_threshold = max(
            self.config.movement_threshold_rad * 0.5,
            self.config.movement_threshold_rad - FRICTION_ANGLE_RESOLUTION_RAD,
        )
        if signed_delta + 1e-12 >= movement_threshold:
            detected = 1
        elif signed_delta - 1e-12 <= -movement_threshold:
            detected = -1
        if detected == 0:
            self._movement_candidate = 0
            self._movement_candidate_samples = 0
            return 0
        if detected != self._movement_candidate:
            self._movement_candidate = detected
            self._movement_candidate_samples = 1
        else:
            self._movement_candidate_samples += 1
        if self._movement_candidate_samples >= FRICTION_MOVEMENT_CONFIRMATION_SAMPLES:
            return detected
        return 0

    def _reset_angle_filter(self) -> None:
        self._trusted_angle_sample = None
        self._pending_angle_sample = None
        self._trusted_fast_angle_step = None

    def prepare_sample(self, sample: TelemetrySample) -> TelemetrySample:
        """Unwrap the shaft angle across board resets, then reject isolated angle glitches."""

        raw_angle = sample.angle_rad
        prepared = self._filter_angle_sample(sample)
        if (
            raw_angle is not None
            and prepared.angle_rad is not None
            and self._trusted_angle_sample is prepared
        ):
            self._board_angle_offset_rad = prepared.angle_rad - raw_angle
        return prepared

    def _unwrap_angle_sample(self, sample: TelemetrySample) -> TelemetrySample:
        angle = sample.angle_rad
        reference = self._continuous_angle_reference_rad
        if angle is None or not math.isfinite(angle) or reference is None:
            return sample
        unwrapped = angle + math.tau * round((reference - angle) / math.tau)
        if math.isclose(unwrapped, angle, rel_tol=0.0, abs_tol=1e-12):
            return sample
        return replace(sample, angle_rad=unwrapped)

    def _trust_angle_sample(self, sample: TelemetrySample) -> TelemetrySample:
        self._trusted_angle_sample = sample
        if sample.angle_rad is not None:
            self._continuous_angle_reference_rad = float(sample.angle_rad)
        return sample

    def _angle_jump_limit(self, previous: TelemetrySample, current: TelemetrySample) -> float:
        elapsed = min(0.02, max(0.0, current.timestamp_s - previous.timestamp_s))
        return max(
            self.config.movement_threshold_rad * 3.0,
            FRICTION_ANGLE_GLITCH_RATE_RAD_S * elapsed + FRICTION_ANGLE_GLITCH_MARGIN_RAD,
        )

    def _filter_angle_sample(self, sample: TelemetrySample) -> TelemetrySample:
        if sample.angle_rad is None or not math.isfinite(sample.angle_rad):
            return sample
        sample = self._unwrap_angle_sample(sample)
        trusted = self._trusted_angle_sample
        if trusted is None or trusted.angle_rad is None:
            self._pending_angle_sample = None
            return self._trust_angle_sample(sample)
        if abs(sample.angle_rad - trusted.angle_rad) <= self._angle_jump_limit(trusted, sample):
            if self._pending_angle_sample is not None:
                self.rejected_angle_samples += 1
            self._pending_angle_sample = None
            self._trusted_fast_angle_step = None
            return self._trust_angle_sample(sample)
        current_step_from_trusted = sample.angle_rad - trusted.angle_rad
        fast_step = self._trusted_fast_angle_step
        if (
            fast_step is not None
            and fast_step * current_step_from_trusted > 0
            and 0.25 <= abs(current_step_from_trusted / fast_step) <= 4.0
        ):
            self._pending_angle_sample = None
            self._trusted_fast_angle_step = current_step_from_trusted
            return self._trust_angle_sample(sample)
        pending = self._pending_angle_sample
        if (
            pending is not None
            and pending.angle_rad is not None
        ):
            pending_step = pending.angle_rad - trusted.angle_rad
            current_step = sample.angle_rad - pending.angle_rad
            steps_form_motion = (
                pending_step * current_step > 0
                and 0.25 <= abs(current_step / pending_step) <= 4.0
            )
            if (
                abs(current_step) <= self._angle_jump_limit(pending, sample)
                or steps_form_motion
            ):
                self._pending_angle_sample = None
                self._trusted_fast_angle_step = current_step if steps_form_motion else None
                return self._trust_angle_sample(sample)
        if pending is not None:
            self.rejected_angle_samples += 1
        self._trusted_fast_angle_step = None
        self._pending_angle_sample = sample
        return replace(sample, angle_rad=None)

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

    def _position_is_settled(self) -> bool:
        if not self.phase_samples:
            return False
        latest_timestamp = self.phase_samples[-1].timestamp_s
        angles = [
            sample.angle_rad
            for sample in self.phase_samples
            if sample.timestamp_s >= latest_timestamp - FRICTION_POSITION_SETTLE_S
            and sample.angle_rad is not None
            and math.isfinite(sample.angle_rad)
        ]
        return (
            len(angles) >= 4
            and max(angles) - min(angles)
            <= max(
                self.config.movement_threshold_rad,
                self.config.position_tolerance_rad / 2.0,
            )
            and abs(angles[-1] - self.current_position_target_rad)
            <= self.config.position_tolerance_rad
        )

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
