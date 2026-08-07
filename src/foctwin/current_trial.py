from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from itertools import pairwise
from typing import Any

from foctwin.domain import TelemetrySample

CURRENT_TRIAL_MONITOR_MASK = "1111111"
CURRENT_TRIAL_CHECKPOINT_SCHEMA = 1
CURRENT_TRIAL_RESULT_SCHEMA = 1


class CurrentTrialPhase(str, Enum):
    IDLE = "idle"
    CONFIGURING_POSITION = "configuring_position"
    POSITIONING = "positioning"
    POSITION_SETTLING = "position_settling"
    CONFIGURING_CURRENT = "configuring_current"
    CURRENT_BASELINE = "current_baseline"
    CURRENT_STEP = "current_step"
    CURRENT_POST = "current_post"
    CONFIGURING_RETURN = "configuring_return"
    RETURNING = "returning"
    RETURN_SETTLING = "return_settling"
    RECOVERING = "recovering"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True, slots=True)
class CurrentTrialAction:
    kind: str
    value: float | None = None


def _default_angle_pid() -> dict[str, float]:
    return {"p": 35.0, "i": 0.0, "d": 0.0, "ramp": 0.0, "lpf": 0.0}


def _default_velocity_pid() -> dict[str, float]:
    return {"p": 20.4, "i": 470.0, "d": 0.0, "ramp": 1000.0, "lpf": 0.01}


@dataclass(slots=True)
class CurrentTrialConfig:
    """Conservative defaults for the first guarded real-motor current trial."""

    step_current_a: float = 0.1
    current_kp: float = 8.4222
    current_ki: float = 814.0
    current_kd: float = 0.0
    current_lpf_tf_s: float = 0.005
    current_output_ramp_v_s: float = 3000.0
    current_target_limit_a: float = 0.5
    current_trip_limit_a: float = 1.0
    current_voltage_limit_v: float = 12.0
    baseline_s: float = 1.0
    step_s: float = 2.0
    post_s: float = 1.0
    monitor_downsample: int = 10
    minimum_phase_samples: int = 10
    transport_voltage_equivalent_v: float = 3.0
    transport_voltage_limit_v: float = 12.0
    transport_velocity_limit_rad_s: float = 0.2
    position_tolerance_rad: float = 0.01
    settled_velocity_rad_s: float = 0.02
    position_settle_s: float = 1.0
    position_timeout_s: float = 15.0
    working_angle_min_rad: float = -3.0
    working_angle_max_rad: float = 3.0
    stop_angle_min_rad: float = -3.5
    stop_angle_max_rad: float = 3.5
    absolute_angle_min_rad: float = -4.0
    absolute_angle_max_rad: float = 4.0
    absolute_current_limit_a: float = 5.0
    absolute_voltage_limit_v: float = 24.0
    velocity_trip_limit_rad_s: float = 0.5
    limit_confirmation_samples: int = 2
    max_recovery_attempts: int = 50
    transport_angle_pid: dict[str, float] = field(default_factory=_default_angle_pid)
    transport_velocity_pid: dict[str, float] = field(default_factory=_default_velocity_pid)

    def validate(self) -> None:
        if not 0 < abs(self.step_current_a) <= self.current_target_limit_a:
            raise ValueError(
                "Модуль токовой ступени должен быть больше нуля и не выше лимита цели"
            )
        if not (
            self.current_target_limit_a
            <= self.current_trip_limit_a
            <= self.absolute_current_limit_a
            <= 5.0
        ):
            raise ValueError(
                "Нужен порядок: лимит цели ≤ аварийный ток опыта ≤ абсолютный ток ≤ 5 А"
            )
        if not (
            0 < self.current_voltage_limit_v <= self.absolute_voltage_limit_v <= 24.0
        ):
            raise ValueError(
                "Рабочее напряжение должно быть не выше абсолютного предела 24 В"
            )
        if not (
            0
            < self.transport_voltage_equivalent_v
            <= self.transport_voltage_limit_v
            <= self.absolute_voltage_limit_v
        ):
            raise ValueError("Некорректные пределы напряжения транспортного режима")
        if self.transport_velocity_limit_rad_s <= 0:
            raise ValueError("Предел скорости транспортного режима должен быть больше нуля")
        if not 0 < self.velocity_trip_limit_rad_s:
            raise ValueError("Аварийный предел скорости должен быть больше нуля")
        if self.transport_velocity_limit_rad_s > self.velocity_trip_limit_rad_s:
            raise ValueError("Рабочая скорость не может быть выше аварийного предела")
        if min(self.baseline_s, self.step_s, self.post_s) <= 0:
            raise ValueError("Все выдержки токового опыта должны быть больше нуля")
        if not 5 <= self.monitor_downsample <= 100:
            raise ValueError("Downsample телеметрии должен быть от 5 до 100")
        if self.minimum_phase_samples < 2:
            raise ValueError("Для каждой фазы нужны хотя бы два отсчёта")
        if min(
            self.position_tolerance_rad,
            self.settled_velocity_rad_s,
            self.position_settle_s,
            self.position_timeout_s,
        ) <= 0:
            raise ValueError("Допуски и таймаут позиционирования должны быть больше нуля")
        if not (
            self.absolute_angle_min_rad
            < self.stop_angle_min_rad
            < self.working_angle_min_rad
            < self.working_angle_max_rad
            < self.stop_angle_max_rad
            < self.absolute_angle_max_rad
        ):
            raise ValueError(
                "Координатные коридоры должны быть вложены: рабочий, остановочный, ±4 рад"
            )
        if self.limit_confirmation_samples < 1:
            raise ValueError("Число подтверждений превышения должно быть положительным")
        if not 0 <= self.max_recovery_attempts <= 100:
            raise ValueError("Число восстановлений должно быть от 0 до 100")
        for name, value in (
            ("P", self.current_kp),
            ("I", self.current_ki),
            ("D", self.current_kd),
            ("LPF Tf", self.current_lpf_tf_s),
            ("output ramp", self.current_output_ramp_v_s),
        ):
            if value < 0 or not math.isfinite(value):
                raise ValueError(f"Токовый {name} должен быть конечным неотрицательным числом")
        for loop_name, values in (
            ("позиционный", self.transport_angle_pid),
            ("скоростной", self.transport_velocity_pid),
        ):
            if set(values) != {"p", "i", "d", "ramp", "lpf"}:
                raise ValueError(f"Не полностью сохранён {loop_name} транспортный PID")
            if any(value < 0 or not math.isfinite(value) for value in values.values()):
                raise ValueError(f"Некорректный {loop_name} транспортный PID")

    @property
    def estimated_duration_s(self) -> float:
        command_overhead_s = 3.0
        return (
            command_overhead_s
            + 2.0 * (self.position_settle_s + min(2.0, self.position_timeout_s))
            + self.baseline_s
            + self.step_s
            + self.post_s
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> CurrentTrialConfig:
        field_names = cls.__dataclass_fields__
        values = {name: payload[name] for name in field_names if name in payload}
        config = cls(**values)
        config.transport_angle_pid = {
            key: float(value) for key, value in config.transport_angle_pid.items()
        }
        config.transport_velocity_pid = {
            key: float(value) for key, value in config.transport_velocity_pid.items()
        }
        config.validate()
        return config


def _values(samples: list[TelemetrySample], name: str) -> list[float]:
    return [
        float(value)
        for sample in samples
        if (value := getattr(sample, name)) is not None and math.isfinite(value)
    ]


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _std(values: list[float]) -> float | None:
    return statistics.pstdev(values) if len(values) >= 2 else (0.0 if values else None)


def _rms(values: list[float]) -> float | None:
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else None


def _peak_abs(values: list[float]) -> float | None:
    return max((abs(value) for value in values), default=None)


def _sample_rate_hz(samples: list[TelemetrySample]) -> float | None:
    timestamps = [sample.timestamp_s for sample in samples]
    intervals = [
        later - earlier
        for earlier, later in pairwise(timestamps)
        if later > earlier
    ]
    mean_interval = _mean(intervals)
    return None if mean_interval is None or mean_interval <= 0 else 1.0 / mean_interval


def _stage_statistics(samples: list[TelemetrySample]) -> dict[str, Any]:
    currents_q = _values(samples, "current_q_a")
    currents_d = _values(samples, "current_d_a")
    voltages_q = _values(samples, "voltage_q_v")
    voltages_d = _values(samples, "voltage_d_v")
    velocities = _values(samples, "velocity_rad_s")
    angles = _values(samples, "angle_rad")
    return {
        "sample_count": len(samples),
        "sample_rate_hz": _sample_rate_hz(samples),
        "duration_s": (
            samples[-1].timestamp_s - samples[0].timestamp_s
            if len(samples) >= 2
            else 0.0
        ),
        "current_q": {
            "mean_a": _mean(currents_q),
            "std_a": _std(currents_q),
            "rms_a": _rms(currents_q),
            "peak_abs_a": _peak_abs(currents_q),
        },
        "current_d": {
            "mean_a": _mean(currents_d),
            "std_a": _std(currents_d),
            "rms_a": _rms(currents_d),
            "peak_abs_a": _peak_abs(currents_d),
        },
        "voltage_q": {
            "mean_v": _mean(voltages_q),
            "peak_abs_v": _peak_abs(voltages_q),
        },
        "voltage_d": {
            "mean_v": _mean(voltages_d),
            "peak_abs_v": _peak_abs(voltages_d),
        },
        "velocity": {
            "mean_rad_s": _mean(velocities),
            "peak_abs_rad_s": _peak_abs(velocities),
        },
        "angle": {
            "start_rad": angles[0] if angles else None,
            "finish_rad": angles[-1] if angles else None,
            "span_rad": max(angles) - min(angles) if angles else None,
        },
    }


class CurrentTrialExperiment:
    """One guarded current step with transport, durable restart and evidence export."""

    CURRENT_PHASES = frozenset(
        {
            CurrentTrialPhase.CURRENT_BASELINE,
            CurrentTrialPhase.CURRENT_STEP,
            CurrentTrialPhase.CURRENT_POST,
        }
    )
    POSITION_PHASES = frozenset(
        {
            CurrentTrialPhase.POSITIONING,
            CurrentTrialPhase.POSITION_SETTLING,
            CurrentTrialPhase.RETURNING,
            CurrentTrialPhase.RETURN_SETTLING,
        }
    )

    def __init__(
        self,
        config: CurrentTrialConfig,
        start_angle_rad: float,
        *,
        recovery_attempts: int = 0,
        interruption_count: int = 0,
        total_sample_count: int = 0,
        rejected_angle_samples: int = 0,
        invalid_attempts: list[dict[str, Any]] | None = None,
        events: list[dict[str, Any]] | None = None,
    ) -> None:
        config.validate()
        if not config.working_angle_min_rad <= start_angle_rad <= config.working_angle_max_rad:
            raise ValueError(
                "Начальная координата должна находиться внутри рабочего коридора ±3 рад"
            )
        self.config = config
        self.start_angle_rad = float(start_angle_rad)
        self.phase = CurrentTrialPhase.IDLE
        self.phase_started_s = 0.0
        self.abort_reason = ""
        self.recovery_attempts = int(recovery_attempts)
        self.interruption_count = int(interruption_count)
        self.invalid_attempts = list(invalid_attempts or [])
        self.events = list(events or [])
        self.baseline_samples: list[TelemetrySample] = []
        self.step_samples: list[TelemetrySample] = []
        self.post_samples: list[TelemetrySample] = []
        self.total_sample_count = int(total_sample_count)
        self.rejected_angle_samples = int(rejected_angle_samples)
        self._position_window: deque[tuple[float, float]] = deque()
        self._limit_counts: dict[str, int] = {}
        self._last_accepted_angle_rad: float | None = None
        self._pending_angle_rad: float | None = None
        self._board_angle_offset_rad = 0.0
        self._continuous_reference_rad = self.start_angle_rad
        self._recovery_source_phase = CurrentTrialPhase.IDLE

    @property
    def active(self) -> bool:
        return self.phase not in {
            CurrentTrialPhase.COMPLETE,
            CurrentTrialPhase.ABORTED,
        }

    @property
    def configuration_mode(self) -> str:
        if self.phase in self.CURRENT_PHASES or self.phase == CurrentTrialPhase.CONFIGURING_CURRENT:
            return "current"
        return "position"

    @property
    def current_target(self) -> float:
        return self.config.step_current_a if self.phase == CurrentTrialPhase.CURRENT_STEP else 0.0

    def _event(self, kind: str, message: str) -> None:
        self.events.append(
            {
                "kind": kind,
                "phase": self.phase.value,
                "message": message,
                "sample_count": self.total_sample_count,
            }
        )

    def seed_angle(
        self,
        raw_angle_rad: float,
        *,
        continuous_reference_rad: float | None = None,
    ) -> None:
        reference = (
            float(raw_angle_rad)
            if continuous_reference_rad is None
            else float(continuous_reference_rad)
        )
        self._board_angle_offset_rad = reference - float(raw_angle_rad)
        self._last_accepted_angle_rad = reference
        self._continuous_reference_rad = reference
        self._pending_angle_rad = None
        self._position_window.clear()

    def reseed_after_recovery(self, raw_angle_rad: float) -> None:
        reference = self._continuous_reference_rad
        turn_offset = round((reference - float(raw_angle_rad)) / math.tau) * math.tau
        continuous_angle = float(raw_angle_rad) + turn_offset
        self.seed_angle(
            raw_angle_rad,
            continuous_reference_rad=continuous_angle,
        )
        self._event(
            "board_reference_restored",
            f"Новая координата платы {raw_angle_rad:.6g}; непрерывная "
            f"{continuous_angle:.6g}; до обрыва {reference:.6g}",
        )

    def board_target_for_continuous(self, target_rad: float) -> float:
        return float(target_rad) - self._board_angle_offset_rad

    def continuous_angle_for_raw(self, raw_angle_rad: float) -> float:
        return float(raw_angle_rad) + self._board_angle_offset_rad

    def prepare_sample(self, sample: TelemetrySample) -> TelemetrySample:
        raw_angle = sample.raw_angle_rad
        if raw_angle is None:
            raw_angle = sample.angle_rad
        if raw_angle is None:
            return sample
        continuous_angle = float(raw_angle) + self._board_angle_offset_rad
        previous = self._last_accepted_angle_rad
        if previous is not None and abs(continuous_angle - previous) > 0.5:
            pending = self._pending_angle_rad
            if pending is None or abs(continuous_angle - pending) > 0.05:
                self._pending_angle_rad = continuous_angle
                self.rejected_angle_samples += 1
                return replace(
                    sample,
                    raw_angle_rad=float(raw_angle),
                    angle_rad=previous,
                    angle_rejected=True,
                )
        self._pending_angle_rad = None
        self._last_accepted_angle_rad = continuous_angle
        self._continuous_reference_rad = continuous_angle
        return replace(
            sample,
            raw_angle_rad=float(raw_angle),
            angle_rad=continuous_angle,
            angle_rejected=False,
        )

    def start(self, now_s: float) -> list[CurrentTrialAction]:
        if self.phase != CurrentTrialPhase.IDLE:
            raise RuntimeError("Токовый опыт уже запущен")
        self.phase = CurrentTrialPhase.CONFIGURING_POSITION
        self.phase_started_s = now_s
        self._event(
            "start",
            f"Начальная координата зафиксирована: {self.start_angle_rad:.6g} рад",
        )
        return [
            CurrentTrialAction("configure_position"),
            CurrentTrialAction("checkpoint"),
        ]

    def position_configuration_applied(
        self,
        now_s: float,
        *,
        returning: bool = False,
    ) -> list[CurrentTrialAction]:
        expected = (
            CurrentTrialPhase.CONFIGURING_RETURN
            if returning
            else CurrentTrialPhase.CONFIGURING_POSITION
        )
        if self.phase != expected:
            return []
        self.phase = (
            CurrentTrialPhase.RETURNING if returning else CurrentTrialPhase.POSITIONING
        )
        self.phase_started_s = now_s
        self._position_window.clear()
        self._limit_counts.clear()
        self._event(
            "position_enabled",
            "Включён безопасный транспортный Angle + Voltage",
        )
        return [
            CurrentTrialAction(
                "position_target",
                self.start_angle_rad,
            )
        ]

    def current_configuration_applied(self, now_s: float) -> list[CurrentTrialAction]:
        if self.phase != CurrentTrialPhase.CONFIGURING_CURRENT:
            return []
        self.phase = CurrentTrialPhase.CURRENT_BASELINE
        self.phase_started_s = now_s
        self._limit_counts.clear()
        self._position_window.clear()
        self._event("current_enabled", "FOC Current включён с нулевой целью")
        return [CurrentTrialAction("target", 0.0)]

    def add_sample(
        self,
        sample: TelemetrySample,
        now_s: float | None = None,
        *,
        angle_prepared: bool = False,
    ) -> tuple[str | None, list[CurrentTrialAction]]:
        if self.phase in {
            CurrentTrialPhase.IDLE,
            CurrentTrialPhase.RECOVERING,
            CurrentTrialPhase.COMPLETE,
            CurrentTrialPhase.ABORTED,
        }:
            return None, []
        if not angle_prepared:
            sample = self.prepare_sample(sample)
        self.total_sample_count += 1
        event_time = sample.timestamp_s if now_s is None else now_s
        if sample.angle_rad is not None and not sample.angle_rejected:
            self._position_window.append((event_time, sample.angle_rad))
            while (
                self._position_window
                and event_time - self._position_window[0][0] > 1.0
            ):
                self._position_window.popleft()
        violation = self._violation(sample)
        if violation:
            return violation, []
        if self.phase == CurrentTrialPhase.CURRENT_BASELINE:
            self.baseline_samples.append(sample)
        elif self.phase == CurrentTrialPhase.CURRENT_STEP:
            self.step_samples.append(sample)
        elif self.phase == CurrentTrialPhase.CURRENT_POST:
            self.post_samples.append(sample)
        return None, []

    def tick(self, now_s: float) -> list[CurrentTrialAction]:
        elapsed = now_s - self.phase_started_s
        if self.phase in {CurrentTrialPhase.POSITIONING, CurrentTrialPhase.RETURNING}:
            if elapsed >= self.config.position_timeout_s:
                return self.abort("Транспортный позиционный режим не достиг цели до таймаута")
            if self._position_in_tolerance():
                self.phase = (
                    CurrentTrialPhase.RETURN_SETTLING
                    if self.phase == CurrentTrialPhase.RETURNING
                    else CurrentTrialPhase.POSITION_SETTLING
                )
                self.phase_started_s = now_s
                return []
        elif self.phase in {
            CurrentTrialPhase.POSITION_SETTLING,
            CurrentTrialPhase.RETURN_SETTLING,
        }:
            returning = self.phase == CurrentTrialPhase.RETURN_SETTLING
            if not self._position_in_tolerance():
                self.phase = (
                    CurrentTrialPhase.RETURNING
                    if returning
                    else CurrentTrialPhase.POSITIONING
                )
                self.phase_started_s = now_s
                return []
            if elapsed >= self.config.position_settle_s and self._position_is_settled():
                if returning:
                    self.phase = CurrentTrialPhase.COMPLETE
                    self.phase_started_s = now_s
                    self._event("complete", "Токовый опыт и возврат завершены")
                    return [
                        CurrentTrialAction("safe_stop"),
                        CurrentTrialAction("finish"),
                        CurrentTrialAction("checkpoint"),
                    ]
                self.phase = CurrentTrialPhase.CONFIGURING_CURRENT
                self.phase_started_s = now_s
                return [
                    CurrentTrialAction("configure_current"),
                    CurrentTrialAction("checkpoint"),
                ]
        elif self.phase == CurrentTrialPhase.CURRENT_BASELINE:
            return self._advance_measured_phase(
                now_s,
                self.config.baseline_s,
                self.baseline_samples,
                next_phase=CurrentTrialPhase.CURRENT_STEP,
                next_target=self.config.step_current_a,
                event_message=f"Подана токовая ступень {self.config.step_current_a:.6g} А",
            )
        elif self.phase == CurrentTrialPhase.CURRENT_STEP:
            return self._advance_measured_phase(
                now_s,
                self.config.step_s,
                self.step_samples,
                next_phase=CurrentTrialPhase.CURRENT_POST,
                next_target=0.0,
                event_message="Токовая ступень снята",
            )
        elif self.phase == CurrentTrialPhase.CURRENT_POST:
            if self._phase_ready(elapsed, self.config.post_s, self.post_samples):
                self.phase = CurrentTrialPhase.CONFIGURING_RETURN
                self.phase_started_s = now_s
                self._event("return", "Начат возврат к исходной координате")
                return [
                    CurrentTrialAction("configure_return"),
                    CurrentTrialAction("checkpoint"),
                ]
            if self._phase_data_timeout(elapsed, self.config.post_s, self.post_samples):
                return self.abort("Недостаточно телеметрии после снятия токовой ступени")
        return []

    def _advance_measured_phase(
        self,
        now_s: float,
        required_s: float,
        samples: list[TelemetrySample],
        *,
        next_phase: CurrentTrialPhase,
        next_target: float,
        event_message: str,
    ) -> list[CurrentTrialAction]:
        elapsed = now_s - self.phase_started_s
        if self._phase_ready(elapsed, required_s, samples):
            self.phase = next_phase
            self.phase_started_s = now_s
            self._event("target", event_message)
            return [
                CurrentTrialAction("target", next_target),
                CurrentTrialAction("checkpoint"),
            ]
        if self._phase_data_timeout(elapsed, required_s, samples):
            return self.abort("Недостаточно телеметрии на активном токовом участке")
        return []

    def _phase_ready(
        self,
        elapsed_s: float,
        required_s: float,
        samples: list[TelemetrySample],
    ) -> bool:
        return (
            elapsed_s >= required_s
            and len(samples) >= self.config.minimum_phase_samples
        )

    def _phase_data_timeout(
        self,
        elapsed_s: float,
        required_s: float,
        samples: list[TelemetrySample],
    ) -> bool:
        return (
            elapsed_s >= max(required_s * 3.0, required_s + 2.0)
            and len(samples) < self.config.minimum_phase_samples
        )

    def enter_recovery(self, reason: str) -> list[CurrentTrialAction]:
        if not self.active or self.phase == CurrentTrialPhase.RECOVERING:
            return []
        self.recovery_attempts += 1
        self.interruption_count += 1
        source_phase = self.phase
        self._recovery_source_phase = source_phase
        self.invalid_attempts.append(
            {
                "reason": reason,
                "phase": source_phase.value,
                "baseline_samples": len(self.baseline_samples),
                "step_samples": len(self.step_samples),
                "post_samples": len(self.post_samples),
            }
        )
        self._event("interruption", f"{reason}; незавершённая попытка будет повторена")
        self.baseline_samples.clear()
        self.step_samples.clear()
        self.post_samples.clear()
        self._position_window.clear()
        self._limit_counts.clear()
        if self.recovery_attempts > self.config.max_recovery_attempts:
            return self.abort("Превышено число восстановлений токового опыта")
        self.phase = CurrentTrialPhase.RECOVERING
        return [
            CurrentTrialAction("safe_stop"),
            CurrentTrialAction("checkpoint"),
        ]

    def resume_after_recovery(self, now_s: float) -> list[CurrentTrialAction]:
        if self.phase != CurrentTrialPhase.RECOVERING:
            return []
        self.phase = CurrentTrialPhase.CONFIGURING_POSITION
        self.phase_started_s = now_s
        self._event(
            "recovery",
            "Связь восстановлена; весь незавершённый токовый опыт начинается заново",
        )
        return [
            CurrentTrialAction("configure_position"),
            CurrentTrialAction("checkpoint"),
        ]

    def abort(self, reason: str) -> list[CurrentTrialAction]:
        self.abort_reason = reason
        self.phase = CurrentTrialPhase.ABORTED
        self._event("abort", reason)
        return [
            CurrentTrialAction("safe_stop"),
            CurrentTrialAction("checkpoint"),
        ]

    def _position_in_tolerance(self) -> bool:
        if self._last_accepted_angle_rad is None:
            return False
        return (
            abs(self._last_accepted_angle_rad - self.start_angle_rad)
            <= self.config.position_tolerance_rad
        )

    def _estimated_angle_velocity(self) -> float | None:
        if len(self._position_window) < 2:
            return None
        start_t, start_angle = self._position_window[0]
        finish_t, finish_angle = self._position_window[-1]
        duration = finish_t - start_t
        if duration <= 0:
            return None
        return (finish_angle - start_angle) / duration

    def _position_is_settled(self) -> bool:
        velocity = self._estimated_angle_velocity()
        return velocity is not None and abs(velocity) <= self.config.settled_velocity_rad_s

    def _confirmed_limit(self, name: str, exceeded: bool) -> bool:
        if not exceeded:
            self._limit_counts[name] = 0
            return False
        count = self._limit_counts.get(name, 0) + 1
        self._limit_counts[name] = count
        return count >= self.config.limit_confirmation_samples

    def _violation(self, sample: TelemetrySample) -> str | None:
        angle = sample.angle_rad
        if angle is not None and not sample.angle_rejected:
            if not self.config.absolute_angle_min_rad <= angle <= self.config.absolute_angle_max_rad:
                return (
                    f"АБСОЛЮТНАЯ ГРАНИЦА: координата {angle:.6g} рад вне "
                    f"[{self.config.absolute_angle_min_rad:g}; "
                    f"{self.config.absolute_angle_max_rad:g}]"
                )
            if not self.config.stop_angle_min_rad <= angle <= self.config.stop_angle_max_rad:
                return (
                    f"Предупредительная остановка: координата {angle:.6g} рад достигла "
                    "коридора ±3,5 рад"
                )
        iq = sample.current_q_a
        id_ = sample.current_d_a
        if iq is not None and id_ is not None:
            current = math.hypot(iq, id_)
            if current > self.config.absolute_current_limit_a * 2.0:
                return f"Резкий выброс полного тока {current:.6g} А"
            current_limit = (
                self.config.current_trip_limit_a
                if self.phase in self.CURRENT_PHASES
                or self.phase == CurrentTrialPhase.CONFIGURING_CURRENT
                else self.config.absolute_current_limit_a
            )
            if self._confirmed_limit("current", current > current_limit):
                return (
                    f"Полный ток {current:.6g} А устойчиво выше "
                    f"{current_limit:g} А"
                )
        uq = sample.voltage_q_v
        ud = sample.voltage_d_v
        if uq is not None and ud is not None:
            voltage = math.hypot(uq, ud)
            if voltage > self.config.absolute_voltage_limit_v * 2.0:
                return f"Резкий выброс полного напряжения {voltage:.6g} В"
            if self._confirmed_limit(
                "voltage",
                voltage > self.config.absolute_voltage_limit_v,
            ):
                return (
                    f"Полное напряжение {voltage:.6g} В устойчиво выше "
                    f"{self.config.absolute_voltage_limit_v:g} В"
                )
        angle_velocity = self._estimated_angle_velocity()
        if angle_velocity is not None:
            if abs(angle_velocity) > self.config.velocity_trip_limit_rad_s * 2.0:
                return f"Резкий разгон по координате: {angle_velocity:.6g} рад/с"
            if self._confirmed_limit(
                "velocity",
                abs(angle_velocity) > self.config.velocity_trip_limit_rad_s,
            ):
                return (
                    f"Скорость по координате {angle_velocity:.6g} рад/с устойчиво выше "
                    f"{self.config.velocity_trip_limit_rad_s:g} рад/с"
                )
        return None

    def checkpoint_payload(self, experiment_id: int | None) -> dict[str, Any]:
        return {
            "schema": CURRENT_TRIAL_CHECKPOINT_SCHEMA,
            "experiment_id": experiment_id,
            "phase": self.phase.value,
            "config": self.config.to_dict(),
            "start_angle_rad": self.start_angle_rad,
            "recovery_attempts": self.recovery_attempts,
            "interruption_count": self.interruption_count,
            "total_sample_count": self.total_sample_count,
            "rejected_angle_samples": self.rejected_angle_samples,
            "invalid_attempts": list(self.invalid_attempts),
            "events": list(self.events),
            "abort_reason": self.abort_reason or None,
            "rule": "always_repeat_whole_unfinished_trial",
        }

    @classmethod
    def from_checkpoint(cls, payload: dict[str, Any]) -> CurrentTrialExperiment:
        if int(payload.get("schema", 0)) != CURRENT_TRIAL_CHECKPOINT_SCHEMA:
            raise ValueError("Checkpoint токового опыта имеет несовместимую схему")
        phase = CurrentTrialPhase(str(payload.get("phase", "")))
        if phase == CurrentTrialPhase.COMPLETE:
            raise ValueError("Токовый опыт из checkpoint уже завершён")
        config = CurrentTrialConfig.from_dict(payload["config"])
        experiment = cls(
            config,
            float(payload["start_angle_rad"]),
            recovery_attempts=int(payload.get("recovery_attempts", 0)),
            interruption_count=int(payload.get("interruption_count", 0)),
            total_sample_count=int(payload.get("total_sample_count", 0)),
            rejected_angle_samples=int(payload.get("rejected_angle_samples", 0)),
            invalid_attempts=list(payload.get("invalid_attempts", [])),
            events=list(payload.get("events", [])),
        )
        experiment._event(
            "application_resume",
            f"Восстановлен checkpoint фазы {phase.value}; опыт будет повторён целиком",
        )
        return experiment

    def result(self, status: str, *, error: str = "") -> dict[str, Any]:
        baseline = _stage_statistics(self.baseline_samples)
        step = _stage_statistics(self.step_samples)
        post = _stage_statistics(self.post_samples)
        step_iq = _values(self.step_samples, "current_q_a")
        baseline_iq = _values(self.baseline_samples, "current_q_a")
        steady_start = len(step_iq) // 2
        steady_iq = step_iq[steady_start:]
        baseline_mean = _mean(baseline_iq)
        steady_mean = _mean(steady_iq)
        signed_response = (
            None
            if steady_mean is None or baseline_mean is None
            else (steady_mean - baseline_mean) * math.copysign(1.0, self.config.step_current_a)
        )
        response_threshold = max(0.02, abs(self.config.step_current_a) * 0.2)
        current_response_observed = (
            signed_response is not None and signed_response >= response_threshold
        )
        steady_error = (
            None
            if steady_mean is None
            else self.config.step_current_a - steady_mean
        )
        peak_iq = _peak_abs(step_iq)
        overshoot_percent = None
        if peak_iq is not None and abs(self.config.step_current_a) > 1e-12:
            overshoot_percent = max(
                0.0,
                (peak_iq - abs(self.config.step_current_a))
                / abs(self.config.step_current_a)
                * 100.0,
            )
        enough_data = all(
            stage["sample_count"] >= self.config.minimum_phase_samples
            for stage in (baseline, step, post)
        )
        valid = status == "completed" and enough_data and current_response_observed
        notes: list[str] = []
        if not enough_data:
            notes.append("В одной или нескольких фазах недостаточно телеметрии.")
        if enough_data and not current_response_observed:
            notes.append(
                "Измеренный Iq не показал надёжный отклик на ступень; данные сохранены, "
                "но коэффициенты по ним пока нельзя оценивать."
            )
        if valid:
            notes.append(
                "Одиночный исполнительный опыт завершён и пригоден для проверки "
                "автоматизации; это ещё не оптимизация коэффициентов."
            )
        if error:
            notes.append(error)
        return {
            "schema": CURRENT_TRIAL_RESULT_SCHEMA,
            "kind": "guarded_single_current_step",
            "status": status,
            "valid": valid,
            "config": self.config.to_dict(),
            "start_angle_rad": self.start_angle_rad,
            "stages": {
                "baseline": baseline,
                "step": step,
                "post": post,
            },
            "metrics": {
                "steady_current_q_a": steady_mean,
                "steady_error_a": steady_error,
                "peak_abs_current_q_a": peak_iq,
                "overshoot_percent": overshoot_percent,
                "rms_current_d_a": _rms(_values(self.step_samples, "current_d_a")),
                "peak_abs_voltage_q_v": _peak_abs(
                    _values(self.step_samples, "voltage_q_v")
                ),
                "angle_span_during_step_rad": step["angle"]["span_rad"],
                "current_response_observed": current_response_observed,
                "response_threshold_a": response_threshold,
            },
            "telemetry": {
                "accepted_sample_count": self.total_sample_count,
                "rejected_angle_samples": self.rejected_angle_samples,
                "interruption_count": self.interruption_count,
                "recovery_attempts": self.recovery_attempts,
            },
            "invalid_attempts": list(self.invalid_attempts),
            "events": list(self.events),
            "error": error or None,
            "note": " ".join(notes) or "Опыт завершён без дополнительного вывода.",
        }
