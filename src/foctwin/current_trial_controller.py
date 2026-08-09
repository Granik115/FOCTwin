"""Shared, durable admission controller for button and instruction current trials.

This module deliberately owns no Serial, Commander or Qt object.  It validates the same
``CurrentTrialConfig`` for both callers, records immutable request parameters and decides whether
a request is safe to simulate, is waiting for a motor, is waiting for local permission or may be
handed to the existing attended UI executor.

Remote hardware execution is hard-disabled in this beta.  A remote request can therefore move
from ``waiting_for_motor`` to ``waiting_for_permission`` when the environment changes, but it can
never become ``ready`` or ``running`` through this controller.
"""

from __future__ import annotations

import json
import sqlite3
import threading
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path

from foctwin.current_trial import CurrentTrialConfig

CURRENT_TRIAL_REQUEST_SCHEMA = 1


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat()


class CurrentTrialControllerError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


class CurrentTrialRequestSource(str, Enum):
    LOCAL_BUTTON = "local_button"
    INSTRUCTION = "instruction"


class CurrentTrialRequestMode(str, Enum):
    HARDWARE = "hardware"
    SIMULATION = "simulation"


class CurrentTrialRequestState(str, Enum):
    BLOCKED = "blocked"
    WAITING_FOR_MOTOR = "waiting_for_motor"
    WAITING_FOR_PERMISSION = "waiting_for_permission"
    READY = "ready"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


TERMINAL_REQUEST_STATES = frozenset(
    {
        CurrentTrialRequestState.COMPLETED,
        CurrentTrialRequestState.FAILED,
        CurrentTrialRequestState.CANCELLED,
    }
)


@dataclass(frozen=True, slots=True)
class CurrentTrialEnvironment:
    project_open: bool = False
    motor_connected: bool = False
    pwm_disabled: bool = True
    telemetry_fresh: bool = False
    telemetry_complete: bool = False
    friction_idle: bool = True
    command_channel_idle: bool = True
    phase_resistance_valid: bool = False
    local_permission: bool = False

    def blockers(self) -> tuple[str, ...]:
        blockers: list[str] = []
        if not self.project_open:
            blockers.append("project_not_open")
        if not self.motor_connected:
            blockers.append("motor_not_connected")
        if not self.pwm_disabled:
            blockers.append("pwm_not_disabled")
        if not self.telemetry_fresh:
            blockers.append("telemetry_not_fresh")
        if not self.telemetry_complete:
            blockers.append("telemetry_incomplete")
        if not self.friction_idle:
            blockers.append("friction_trial_running")
        if not self.command_channel_idle:
            blockers.append("command_channel_busy")
        if not self.phase_resistance_valid:
            blockers.append("phase_resistance_invalid")
        return tuple(blockers)

    def to_dict(self) -> dict[str, object]:
        return {
            "project_open": self.project_open,
            "motor_connected": self.motor_connected,
            "pwm_disabled": self.pwm_disabled,
            "telemetry_fresh": self.telemetry_fresh,
            "telemetry_complete": self.telemetry_complete,
            "friction_idle": self.friction_idle,
            "command_channel_idle": self.command_channel_idle,
            "phase_resistance_valid": self.phase_resistance_valid,
            "local_permission": self.local_permission,
            "blockers": list(self.blockers()),
        }


@dataclass(frozen=True, slots=True)
class CurrentTrialDecision:
    request_id: str
    command_id: str
    source: CurrentTrialRequestSource
    mode: CurrentTrialRequestMode
    state: CurrentTrialRequestState
    reason_code: str
    message: str
    config: dict[str, object]
    environment: dict[str, object]
    result: dict[str, object]
    created_at: str
    updated_at: str

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL_REQUEST_STATES

    def to_dict(self) -> dict[str, object]:
        return {
            "schema": CURRENT_TRIAL_REQUEST_SCHEMA,
            "request_id": self.request_id,
            "command_id": self.command_id,
            "source": self.source.value,
            "mode": self.mode.value,
            "state": self.state.value,
            "terminal": self.terminal,
            "reason": self.reason_code,
            "message": self.message,
            "config": self.config,
            "environment": self.environment,
            "result": self.result,
            "remote_hardware_execution_enabled": False,
        }


class CurrentTrialController:
    """Persistent request admission shared by local UI and file instructions."""

    def __init__(
        self,
        root: Path,
        *,
        now_factory: Callable[[], datetime] = utc_now,
    ) -> None:
        self.root = Path(root)
        self.db_path = self.root / "current_trial_requests.sqlite3"
        self.now_factory = now_factory
        self._lock = threading.RLock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.db_path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA synchronous = FULL")
        return connection

    def _initialize(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        now = _iso(self.now_factory())
        with self._lock, self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS current_trial_requests (
                    request_id TEXT PRIMARY KEY,
                    command_id TEXT NOT NULL DEFAULT '',
                    source TEXT NOT NULL,
                    mode TEXT NOT NULL,
                    state TEXT NOT NULL,
                    reason_code TEXT NOT NULL DEFAULT '',
                    message TEXT NOT NULL DEFAULT '',
                    config_json TEXT NOT NULL,
                    environment_json TEXT NOT NULL DEFAULT '{}',
                    result_json TEXT NOT NULL DEFAULT '{}',
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE UNIQUE INDEX IF NOT EXISTS idx_current_trial_command
                    ON current_trial_requests(command_id) WHERE command_id != '';
                CREATE INDEX IF NOT EXISTS idx_current_trial_state
                    ON current_trial_requests(state, updated_at);
                """
            )
            connection.execute(
                """
                UPDATE current_trial_requests
                SET state = ?, reason_code = ?, message = ?, updated_at = ?
                WHERE state IN (?, ?)
                """,
                (
                    CurrentTrialRequestState.WAITING_FOR_PERMISSION.value,
                    "restart_requires_permission",
                    "После перезапуска требуется новое локальное разрешение",
                    now,
                    CurrentTrialRequestState.READY.value,
                    CurrentTrialRequestState.RUNNING.value,
                ),
            )

    @staticmethod
    def _load_json(value: object) -> dict[str, object]:
        decoded = json.loads(str(value))
        return dict(decoded) if isinstance(decoded, dict) else {}

    @classmethod
    def _row_to_decision(cls, row: sqlite3.Row) -> CurrentTrialDecision:
        return CurrentTrialDecision(
            request_id=str(row["request_id"]),
            command_id=str(row["command_id"]),
            source=CurrentTrialRequestSource(str(row["source"])),
            mode=CurrentTrialRequestMode(str(row["mode"])),
            state=CurrentTrialRequestState(str(row["state"])),
            reason_code=str(row["reason_code"]),
            message=str(row["message"]),
            config=cls._load_json(row["config_json"]),
            environment=cls._load_json(row["environment_json"]),
            result=cls._load_json(row["result_json"]),
            created_at=str(row["created_at"]),
            updated_at=str(row["updated_at"]),
        )

    @staticmethod
    def _json(payload: dict[str, object]) -> str:
        return json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )

    def _get_by(self, field: str, value: str) -> CurrentTrialDecision | None:
        if field not in {"request_id", "command_id"}:
            raise ValueError("Unsupported current-trial lookup")
        with self._lock, self._connect() as connection:
            row = connection.execute(
                f"SELECT * FROM current_trial_requests WHERE {field} = ?",
                (value,),
            ).fetchone()
        return None if row is None else self._row_to_decision(row)

    def get(self, request_id: str) -> CurrentTrialDecision | None:
        return self._get_by("request_id", request_id)

    def get_by_command(self, command_id: str) -> CurrentTrialDecision | None:
        return self._get_by("command_id", command_id)

    @staticmethod
    def parse_instruction(
        arguments: dict[str, object],
    ) -> tuple[CurrentTrialRequestMode, CurrentTrialConfig]:
        unknown = set(arguments) - {"mode", "config"}
        if unknown:
            names = ", ".join(sorted(unknown))
            raise CurrentTrialControllerError(
                "invalid_arguments",
                f"Неизвестные arguments для run_current_trial: {names}",
            )
        raw_mode = arguments.get("mode", CurrentTrialRequestMode.HARDWARE.value)
        try:
            mode = CurrentTrialRequestMode(str(raw_mode).strip().lower())
        except ValueError as exc:
            raise CurrentTrialControllerError(
                "invalid_mode",
                "mode должен быть hardware или simulation",
            ) from exc
        raw_config = arguments.get("config", {})
        if not isinstance(raw_config, dict):
            raise CurrentTrialControllerError(
                "invalid_config",
                "config должен быть JSON-объектом",
            )
        unknown_config = set(raw_config) - set(CurrentTrialConfig.__dataclass_fields__)
        if unknown_config:
            names = ", ".join(sorted(unknown_config))
            raise CurrentTrialControllerError(
                "invalid_config",
                f"Неизвестные поля config: {names}",
            )
        try:
            config = CurrentTrialConfig.from_dict(dict(raw_config))
        except (KeyError, TypeError, ValueError) as exc:
            raise CurrentTrialControllerError("invalid_config", str(exc)) from exc
        return mode, config

    @staticmethod
    def _decision_for(
        source: CurrentTrialRequestSource,
        mode: CurrentTrialRequestMode,
        environment: CurrentTrialEnvironment,
    ) -> tuple[CurrentTrialRequestState, str, str]:
        if mode == CurrentTrialRequestMode.SIMULATION:
            return (
                CurrentTrialRequestState.READY,
                "simulation_ready",
                "Безопасная имитация готова; мотор и PWM не используются",
            )
        blockers = environment.blockers()
        if source == CurrentTrialRequestSource.INSTRUCTION:
            if "motor_not_connected" in blockers:
                return (
                    CurrentTrialRequestState.WAITING_FOR_MOTOR,
                    "motor_not_connected",
                    "Аппаратный запрос сохранён и ждёт подключения мотора",
                )
            return (
                CurrentTrialRequestState.WAITING_FOR_PERMISSION,
                "remote_hardware_locked",
                "Удалённый аппаратный запуск заблокирован до будущего локального вооружения",
            )
        if blockers:
            first = blockers[0]
            messages = {
                "project_not_open": "Сначала создайте или откройте проект FOCTwin",
                "motor_not_connected": "Сначала подключите мотор",
                "pwm_not_disabled": "Перед запуском вручную отключите PWM",
                "telemetry_not_fresh": "Нет свежей распознанной телеметрии",
                "telemetry_incomplete": "Нужны угол, Iq, Id, Uq и Ud",
                "friction_trial_running": "Сначала завершите выполняющийся тест трения",
                "command_channel_busy": "Дождитесь завершения текущей отправки команд",
                "phase_resistance_invalid": "Нужно положительное сопротивление фазы",
            }
            return CurrentTrialRequestState.BLOCKED, first, messages[first]
        if not environment.local_permission:
            return (
                CurrentTrialRequestState.WAITING_FOR_PERMISSION,
                "local_confirmation_required",
                "Требуется подтверждение запуска в окне FOCTwin",
            )
        return (
            CurrentTrialRequestState.READY,
            "locally_authorized",
            "Локально подтверждённый опыт готов к запуску",
        )

    def preview_local(
        self,
        config: CurrentTrialConfig,
        environment: CurrentTrialEnvironment,
    ) -> CurrentTrialDecision:
        config.validate()
        state, code, message = self._decision_for(
            CurrentTrialRequestSource.LOCAL_BUTTON,
            CurrentTrialRequestMode.HARDWARE,
            environment,
        )
        now = _iso(self.now_factory())
        return CurrentTrialDecision(
            request_id="",
            command_id="",
            source=CurrentTrialRequestSource.LOCAL_BUTTON,
            mode=CurrentTrialRequestMode.HARDWARE,
            state=state,
            reason_code=code,
            message=message,
            config=config.to_dict(),
            environment=environment.to_dict(),
            result={},
            created_at=now,
            updated_at=now,
        )

    def _insert(
        self,
        *,
        request_id: str,
        command_id: str,
        source: CurrentTrialRequestSource,
        mode: CurrentTrialRequestMode,
        state: CurrentTrialRequestState,
        reason_code: str,
        message: str,
        config: CurrentTrialConfig,
        environment: CurrentTrialEnvironment,
        result: dict[str, object] | None = None,
    ) -> CurrentTrialDecision:
        now = _iso(self.now_factory())
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO current_trial_requests(
                    request_id, command_id, source, mode, state, reason_code, message,
                    config_json, environment_json, result_json, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    request_id,
                    command_id,
                    source.value,
                    mode.value,
                    state.value,
                    reason_code,
                    message,
                    self._json(config.to_dict()),
                    self._json(environment.to_dict()),
                    self._json(result or {}),
                    now,
                    now,
                ),
            )
        decision = self.get(request_id)
        if decision is None:  # pragma: no cover - SQLite insert invariant
            raise CurrentTrialControllerError("store_failed", "Запрос не сохранён")
        return decision

    @staticmethod
    def _simulation_result(config: CurrentTrialConfig) -> dict[str, object]:
        return {
            "schema": CURRENT_TRIAL_REQUEST_SCHEMA,
            "simulated": True,
            "executed": False,
            "hardware_access": False,
            "estimated_duration_s": config.estimated_duration_s,
            "phases": [
                "configure_position",
                "position_and_settle",
                "configure_current_at_zero",
                "baseline",
                "current_step",
                "post_zero",
                "return_and_safe_stop",
            ],
            "targets": {
                "baseline_a": 0.0,
                "step_a": config.step_current_a,
                "post_a": 0.0,
            },
            "durations_s": {
                "baseline": config.baseline_s,
                "step": config.step_s,
                "post": config.post_s,
            },
            "limits": {
                "target_current_a": config.current_target_limit_a,
                "trip_current_a": config.current_trip_limit_a,
                "working_voltage_v": config.current_voltage_limit_v,
                "absolute_current_a": config.absolute_current_limit_a,
                "absolute_voltage_v": config.absolute_voltage_limit_v,
                "absolute_angle_rad": [
                    config.absolute_angle_min_rad,
                    config.absolute_angle_max_rad,
                ],
            },
            "note": "Проверен только маршрут и план опыта; Serial и PWM не открывались.",
        }

    def submit_instruction(
        self,
        command_id: str,
        arguments: dict[str, object],
        environment: CurrentTrialEnvironment,
    ) -> CurrentTrialDecision:
        existing = self.get_by_command(command_id)
        if existing is not None:
            return existing
        mode, config = self.parse_instruction(arguments)
        state, code, message = self._decision_for(
            CurrentTrialRequestSource.INSTRUCTION,
            mode,
            environment,
        )
        result: dict[str, object] = {}
        if mode == CurrentTrialRequestMode.SIMULATION:
            state = CurrentTrialRequestState.COMPLETED
            code = "simulation_completed"
            message = "Безопасная имитация токового опыта завершена"
            result = self._simulation_result(config)
        return self._insert(
            request_id=str(uuid.uuid4()),
            command_id=command_id,
            source=CurrentTrialRequestSource.INSTRUCTION,
            mode=mode,
            state=state,
            reason_code=code,
            message=message,
            config=config,
            environment=environment,
            result=result,
        )

    def submit_local(
        self,
        config: CurrentTrialConfig,
        environment: CurrentTrialEnvironment,
        *,
        request_id: str | None = None,
    ) -> CurrentTrialDecision:
        config.validate()
        state, code, message = self._decision_for(
            CurrentTrialRequestSource.LOCAL_BUTTON,
            CurrentTrialRequestMode.HARDWARE,
            environment,
        )
        if state != CurrentTrialRequestState.READY:
            raise CurrentTrialControllerError(code, message)
        if request_id is not None:
            existing = self.get(request_id)
            if existing is not None:
                if existing.source != CurrentTrialRequestSource.LOCAL_BUTTON:
                    raise CurrentTrialControllerError(
                        "request_source_mismatch",
                        "Сохранённый запрос создан не локальной кнопкой",
                    )
                if existing.config != config.to_dict():
                    raise CurrentTrialControllerError(
                        "request_config_mismatch",
                        "Параметры checkpoint не совпадают с сохранённым запросом",
                    )
                if existing.state not in TERMINAL_REQUEST_STATES:
                    return self._update(
                        request_id,
                        state=state,
                        reason_code=code,
                        message=message,
                        environment=environment.to_dict(),
                        result={},
                    )
                request_id = None
        return self._insert(
            request_id=request_id or str(uuid.uuid4()),
            command_id="",
            source=CurrentTrialRequestSource.LOCAL_BUTTON,
            mode=CurrentTrialRequestMode.HARDWARE,
            state=state,
            reason_code=code,
            message=message,
            config=config,
            environment=environment,
        )

    def refresh_instruction(
        self,
        command_id: str,
        environment: CurrentTrialEnvironment,
    ) -> CurrentTrialDecision:
        decision = self.get_by_command(command_id)
        if decision is None:
            raise CurrentTrialControllerError("request_not_found", "Запрос опыта не найден")
        if decision.terminal or decision.mode == CurrentTrialRequestMode.SIMULATION:
            return decision
        state, code, message = self._decision_for(
            CurrentTrialRequestSource.INSTRUCTION,
            CurrentTrialRequestMode.HARDWARE,
            environment,
        )
        if (
            state == decision.state
            and code == decision.reason_code
            and environment.to_dict() == decision.environment
        ):
            return decision
        return self._update(
            decision.request_id,
            state=state,
            reason_code=code,
            message=message,
            environment=environment.to_dict(),
        )

    def _update(
        self,
        request_id: str,
        *,
        state: CurrentTrialRequestState,
        reason_code: str,
        message: str,
        environment: dict[str, object] | None = None,
        result: dict[str, object] | None = None,
    ) -> CurrentTrialDecision:
        assignments = ["state = ?", "reason_code = ?", "message = ?", "updated_at = ?"]
        values: list[object] = [state.value, reason_code, message, _iso(self.now_factory())]
        if environment is not None:
            assignments.append("environment_json = ?")
            values.append(self._json(environment))
        if result is not None:
            assignments.append("result_json = ?")
            values.append(self._json(result))
        values.append(request_id)
        with self._lock, self._connect() as connection:
            cursor = connection.execute(
                f"UPDATE current_trial_requests SET {', '.join(assignments)} "
                "WHERE request_id = ?",
                values,
            )
            if cursor.rowcount != 1:
                raise CurrentTrialControllerError("request_not_found", "Запрос опыта не найден")
        decision = self.get(request_id)
        if decision is None:  # pragma: no cover - SQLite update invariant
            raise CurrentTrialControllerError("store_failed", "Запрос опыта потерян")
        return decision

    def mark_running(self, request_id: str) -> CurrentTrialDecision:
        decision = self.get(request_id)
        if decision is None:
            raise CurrentTrialControllerError("request_not_found", "Запрос опыта не найден")
        if decision.state != CurrentTrialRequestState.READY:
            raise CurrentTrialControllerError(
                "request_not_ready",
                f"Опыт нельзя запустить из состояния {decision.state.value}",
            )
        return self._update(
            request_id,
            state=CurrentTrialRequestState.RUNNING,
            reason_code="local_trial_running",
            message="Локально подтверждённый токовый опыт выполняется",
        )

    def finish_local(
        self,
        request_id: str,
        *,
        status: str,
        result: dict[str, object],
        error: str = "",
    ) -> CurrentTrialDecision:
        if status == "completed":
            state = CurrentTrialRequestState.COMPLETED
            code = "trial_completed"
            message = "Токовый опыт завершён"
        elif status == "interrupted":
            state = CurrentTrialRequestState.CANCELLED
            code = "trial_interrupted"
            message = error or "Токовый опыт остановлен"
        else:
            state = CurrentTrialRequestState.FAILED
            code = "trial_failed"
            message = error or "Токовый опыт завершился ошибкой"
        return self._update(
            request_id,
            state=state,
            reason_code=code,
            message=message,
            result=result,
        )

    def cancel_instruction(self, command_id: str) -> CurrentTrialDecision:
        decision = self.get_by_command(command_id)
        if decision is None:
            raise CurrentTrialControllerError("request_not_found", "Запрос опыта не найден")
        if decision.terminal:
            return decision
        if decision.state == CurrentTrialRequestState.RUNNING:
            raise CurrentTrialControllerError(
                "remote_running_cancel_unavailable",
                "Эта бета не запускает удалённые аппаратные опыты",
            )
        return self._update(
            decision.request_id,
            state=CurrentTrialRequestState.CANCELLED,
            reason_code="cancelled_by_instruction",
            message="Ожидающий запрос отменён отдельной инструкцией",
        )

    def recent(self, limit: int = 50) -> list[CurrentTrialDecision]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                """
                SELECT * FROM current_trial_requests
                ORDER BY created_at DESC, request_id DESC LIMIT ?
                """,
                (limit,),
            ).fetchall()
        return [self._row_to_decision(row) for row in rows]
