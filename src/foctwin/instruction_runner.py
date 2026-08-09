"""Durable local instruction channel with a deliberately hardware-isolated capability set.

FolderBridge (or another one-way file transport) may deliver immutable command JSON files to
``inbox/commands`` and copy FOCTwin-owned events/status from ``outbox``.  This module owns only
file validation, SQLite deduplication and command journaling.  It intentionally does not import
Serial, Commander, Qt, or any motor experiment module.  Current-trial admission is exposed only
through a narrow callback supplied by the shared controller in the UI layer.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import sqlite3
import tempfile
import threading
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

INSTRUCTION_SCHEMA = 1
INSTRUCTION_PROTOCOL = "foctwin-instruction-runner"
MAX_COMMAND_FILE_BYTES = 256 * 1024
MAX_COMMAND_FILES_PER_SCAN = 100
MAX_JSON_DEPTH = 12
MAX_COMMAND_LIFETIME = timedelta(days=7)
FUTURE_CLOCK_TOLERANCE = timedelta(minutes=5)
COMMAND_NAME_PATTERN = re.compile(
    r"^cmd_([0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12})\.json$",
    re.IGNORECASE,
)


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat()


def _parse_timestamp(value: object, field: str) -> datetime:
    if not isinstance(value, str) or not value.strip():
        raise InstructionValidationError("invalid_timestamp", f"Поле {field} не задано")
    try:
        parsed = datetime.fromisoformat(value.strip().replace("Z", "+00:00"))
    except ValueError as exc:
        raise InstructionValidationError(
            "invalid_timestamp", f"Поле {field} не является ISO-датой"
        ) from exc
    if parsed.tzinfo is None:
        raise InstructionValidationError(
            "invalid_timestamp", f"Поле {field} должно содержать часовой пояс"
        )
    return parsed.astimezone(timezone.utc)


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"unsupported JSON constant {value}")


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _check_json_shape(value: object, *, depth: int = 0) -> None:
    if depth > MAX_JSON_DEPTH:
        raise InstructionValidationError(
            "arguments_too_deep", f"JSON глубже допустимых {MAX_JSON_DEPTH} уровней"
        )
    if isinstance(value, dict):
        if any(not isinstance(key, str) for key in value):
            raise InstructionValidationError(
                "invalid_arguments", "Все ключи JSON должны быть строками"
            )
        for nested in value.values():
            _check_json_shape(nested, depth=depth + 1)
    elif isinstance(value, list):
        for nested in value:
            _check_json_shape(nested, depth=depth + 1)
    elif isinstance(value, float) and not math.isfinite(value):
        raise InstructionValidationError(
            "invalid_arguments", "Числа JSON должны быть конечными"
        )
    elif value is not None and not isinstance(value, (str, int, float, bool)):
        raise InstructionValidationError(
            "invalid_arguments", f"Неподдерживаемый тип JSON: {type(value).__name__}"
        )


def _atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


class InstructionRunnerError(RuntimeError):
    """Base error suitable for the instruction-runner UI."""


class InstructionValidationError(InstructionRunnerError):
    def __init__(self, code: str, message: str, *, command_id: str | None = None) -> None:
        self.code = code
        self.command_id = command_id
        super().__init__(message)


class _TransientCommandRead(InstructionRunnerError):
    """The transport is still replacing a command file; retry it during the next scan."""


class _RejectedCommandFile(InstructionRunnerError):
    def __init__(self, code: str, message: str, fingerprint: str) -> None:
        self.code = code
        self.fingerprint = fingerprint
        super().__init__(message)


@dataclass(frozen=True)
class Capability:
    name: str
    description: str
    hardware_access: bool = False
    motor_required: bool = False
    execution_policy: str = "immediate"

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "description": self.description,
            "hardware_access": self.hardware_access,
            "motor_required": self.motor_required,
            "execution_policy": self.execution_policy,
            "remote_hardware_execution_enabled": False,
        }


CAPABILITIES = (
    Capability("ping", "Проверить доставку команды и получить pong"),
    Capability("get_status", "Получить безопасный статус локального обработчика"),
    Capability("list_capabilities", "Получить жёсткий список доступных команд"),
    Capability("self_test", "Проверить папки, SQLite и запись status.json"),
    Capability("dry_run", "Проверить допустимость вложенной команды без её выполнения"),
    Capability(
        "run_current_trial",
        "Имитировать токовый опыт или сохранить аппаратный запрос в безопасном ожидании",
        hardware_access=True,
        motor_required=True,
        execution_policy="simulate_or_wait_for_local_permission",
    ),
    Capability(
        "cancel_current_trial",
        "Отменить ожидающий аппаратный запрос по command_id",
        execution_policy="cancel_waiting_request",
    ),
)
CAPABILITY_BY_NAME = {item.name: item for item in CAPABILITIES}
HARDWARE_COMMAND_HINTS = {
    "abort",
    "apply_modes",
    "commander",
    "connect_motor",
    "enable_pwm",
    "run_current_trial",
    "run_friction_test",
    "serial",
    "set_target",
}

WAITING_COMMAND_STATES = frozenset({"waiting_for_motor", "waiting_for_permission"})
TERMINAL_COMMAND_STATES = frozenset(
    {"completed", "failed", "rejected", "ignored", "cancelled"}
)


@dataclass(frozen=True)
class InstructionCommand:
    command_id: str
    target: str
    command_type: str
    created_at: str
    expires_at: str
    arguments: dict[str, object]
    source_file: str
    state: str = ""

    @classmethod
    def from_payload(
        cls,
        payload: object,
        *,
        source_file: str,
        now: datetime,
    ) -> InstructionCommand:
        if not isinstance(payload, dict):
            raise InstructionValidationError("invalid_envelope", "Корень JSON должен быть объектом")
        if payload.get("schema") != INSTRUCTION_SCHEMA:
            raise InstructionValidationError(
                "unsupported_schema", f"Поддерживается только schema={INSTRUCTION_SCHEMA}"
            )
        protocol = payload.get("protocol")
        if protocol not in {None, INSTRUCTION_PROTOCOL}:
            raise InstructionValidationError(
                "unsupported_protocol", f"Неизвестный protocol: {protocol}"
            )

        raw_command_id = payload.get("command_id")
        if not isinstance(raw_command_id, str):
            raise InstructionValidationError("invalid_command_id", "command_id должен быть UUID")
        try:
            command_id = str(uuid.UUID(raw_command_id.strip()))
        except (ValueError, AttributeError) as exc:
            raise InstructionValidationError(
                "invalid_command_id", "command_id должен быть UUID"
            ) from exc
        expected_name = f"cmd_{command_id}.json"
        if source_file.casefold() != expected_name.casefold():
            raise InstructionValidationError(
                "command_filename_mismatch",
                f"Имя файла должно быть {expected_name}",
                command_id=command_id,
            )

        target = payload.get("target")
        if not isinstance(target, str) or not target.strip():
            raise InstructionValidationError(
                "invalid_target", "target должен быть Instance ID или *", command_id=command_id
            )
        target = target.strip()
        if len(target) > 128:
            raise InstructionValidationError(
                "invalid_target", "target слишком длинный", command_id=command_id
            )

        raw_type = payload.get("type")
        if not isinstance(raw_type, str):
            raise InstructionValidationError(
                "invalid_type", "type должен быть строкой", command_id=command_id
            )
        command_type = raw_type.strip().lower()
        if not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", command_type):
            raise InstructionValidationError(
                "invalid_type", "type имеет недопустимый формат", command_id=command_id
            )

        arguments = payload.get("arguments", {})
        if not isinstance(arguments, dict):
            raise InstructionValidationError(
                "invalid_arguments", "arguments должен быть JSON-объектом", command_id=command_id
            )
        _check_json_shape(arguments)

        created = _parse_timestamp(payload.get("created_at"), "created_at")
        expires = _parse_timestamp(payload.get("expires_at"), "expires_at")
        if created > now + FUTURE_CLOCK_TOLERANCE:
            raise InstructionValidationError(
                "created_in_future",
                "created_at слишком далеко в будущем",
                command_id=command_id,
            )
        if expires <= created:
            raise InstructionValidationError(
                "invalid_lifetime",
                "expires_at должен быть позже created_at",
                command_id=command_id,
            )
        if expires - created > MAX_COMMAND_LIFETIME:
            raise InstructionValidationError(
                "invalid_lifetime",
                f"Срок команды не может превышать {MAX_COMMAND_LIFETIME.days} дней",
                command_id=command_id,
            )
        if expires <= now:
            raise InstructionValidationError(
                "expired", "Срок действия команды истёк", command_id=command_id
            )

        return cls(
            command_id=command_id,
            target=target,
            command_type=command_type,
            created_at=_iso(created),
            expires_at=_iso(expires),
            arguments=dict(arguments),
            source_file=source_file,
        )

    def to_payload(self) -> dict[str, object]:
        return {
            "schema": INSTRUCTION_SCHEMA,
            "protocol": INSTRUCTION_PROTOCOL,
            "command_id": self.command_id,
            "target": self.target,
            "type": self.command_type,
            "created_at": self.created_at,
            "expires_at": self.expires_at,
            "arguments": self.arguments,
        }


@dataclass(frozen=True)
class CommandRecord:
    command_id: str
    command_type: str
    state: str
    received_at: str
    finished_at: str
    error_code: str
    error_message: str
    result: dict[str, object]


@dataclass(frozen=True)
class InstructionSnapshot:
    instance_id: str
    exchange_root: Path
    state: str
    last_scan_at: str
    last_error: str
    command_counts: dict[str, int]
    recent_commands: list[CommandRecord]


@dataclass(frozen=True)
class ScanResult:
    accepted: int = 0
    completed: int = 0
    rejected: int = 0
    ignored: int = 0
    duplicates: int = 0
    deferred: int = 0
    waiting: int = 0


@dataclass(frozen=True)
class CommandExecution:
    state: str
    result: dict[str, object]

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL_COMMAND_STATES


class InstructionStore:
    """Small SQLite journal kept outside the FolderBridge-synchronized tree."""

    def __init__(self, root: Path) -> None:
        self.root = Path(root)
        self.db_path = self.root / "instructions.sqlite3"
        self._lock = threading.RLock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.db_path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA synchronous = FULL")
        return connection

    def _initialize(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        with self._lock, self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS seen_files (
                    source_file TEXT NOT NULL,
                    sha256 TEXT NOT NULL,
                    command_id TEXT,
                    outcome TEXT NOT NULL,
                    seen_at TEXT NOT NULL,
                    PRIMARY KEY (source_file, sha256)
                );
                CREATE TABLE IF NOT EXISTS commands (
                    command_id TEXT PRIMARY KEY,
                    source_file TEXT NOT NULL,
                    payload_sha256 TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    command_type TEXT NOT NULL,
                    state TEXT NOT NULL,
                    received_at TEXT NOT NULL,
                    started_at TEXT NOT NULL DEFAULT '',
                    finished_at TEXT NOT NULL DEFAULT '',
                    result_json TEXT NOT NULL DEFAULT '{}',
                    error_code TEXT NOT NULL DEFAULT '',
                    error_message TEXT NOT NULL DEFAULT ''
                );
                CREATE TABLE IF NOT EXISTS events (
                    event_id TEXT PRIMARY KEY,
                    command_id TEXT,
                    sequence INTEGER NOT NULL,
                    event_type TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    file_name TEXT NOT NULL UNIQUE,
                    exported INTEGER NOT NULL DEFAULT 0
                );
                CREATE INDEX IF NOT EXISTS idx_commands_state ON commands(state, received_at);
                CREATE INDEX IF NOT EXISTS idx_events_exported ON events(exported, created_at);
                """
            )
            instance_id = connection.execute(
                "SELECT value FROM metadata WHERE key = 'instance_id'"
            ).fetchone()
            if instance_id is None:
                connection.execute(
                    "INSERT INTO metadata(key, value) VALUES ('instance_id', ?)",
                    (str(uuid.uuid4()),),
                )

    def get_metadata(self, key: str, default: str = "") -> str:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT value FROM metadata WHERE key = ?", (key,)
            ).fetchone()
            return str(row["value"]) if row is not None else default

    def set_metadata(self, key: str, value: str) -> None:
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO metadata(key, value) VALUES (?, ?)
                ON CONFLICT(key) DO UPDATE SET value = excluded.value
                """,
                (key, value),
            )

    @property
    def instance_id(self) -> str:
        return self.get_metadata("instance_id")

    def has_seen_file(self, source_file: str, digest: str) -> bool:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT 1 FROM seen_files WHERE source_file = ? AND sha256 = ?",
                (source_file, digest),
            ).fetchone()
            return row is not None

    def record_seen_file(
        self,
        source_file: str,
        digest: str,
        *,
        command_id: str | None,
        outcome: str,
        seen_at: str,
    ) -> None:
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT OR IGNORE INTO seen_files(
                    source_file, sha256, command_id, outcome, seen_at
                ) VALUES (?, ?, ?, ?, ?)
                """,
                (source_file, digest, command_id, outcome, seen_at),
            )

    def get_command_digest(self, command_id: str) -> str | None:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT payload_sha256 FROM commands WHERE command_id = ?", (command_id,)
            ).fetchone()
            return str(row["payload_sha256"]) if row is not None else None

    def insert_command(
        self,
        command: InstructionCommand,
        *,
        digest: str,
        state: str,
        received_at: str,
    ) -> bool:
        payload_json = json.dumps(
            command.to_payload(), ensure_ascii=False, separators=(",", ":"), allow_nan=False
        )
        with self._lock, self._connect() as connection:
            cursor = connection.execute(
                """
                INSERT OR IGNORE INTO commands(
                    command_id, source_file, payload_sha256, payload_json,
                    command_type, state, received_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    command.command_id,
                    command.source_file,
                    digest,
                    payload_json,
                    command.command_type,
                    state,
                    received_at,
                ),
            )
            return cursor.rowcount == 1

    def update_command(
        self,
        command_id: str,
        *,
        state: str,
        started_at: str | None = None,
        finished_at: str | None = None,
        result: dict[str, object] | None = None,
        error_code: str = "",
        error_message: str = "",
    ) -> None:
        assignments = ["state = ?", "error_code = ?", "error_message = ?"]
        values: list[object] = [state, error_code, error_message]
        if started_at is not None:
            assignments.append("started_at = ?")
            values.append(started_at)
        if finished_at is not None:
            assignments.append("finished_at = ?")
            values.append(finished_at)
        if result is not None:
            assignments.append("result_json = ?")
            values.append(
                json.dumps(result, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
            )
        values.append(command_id)
        with self._lock, self._connect() as connection:
            connection.execute(
                f"UPDATE commands SET {', '.join(assignments)} WHERE command_id = ?", values
            )

    def pending_commands(self) -> list[InstructionCommand]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                """
                SELECT payload_json, source_file, state FROM commands
                WHERE state IN ('received', 'running')
                ORDER BY received_at, command_id
                """
            ).fetchall()
        commands: list[InstructionCommand] = []
        for row in rows:
            payload = json.loads(str(row["payload_json"]))
            commands.append(
                InstructionCommand(
                    command_id=str(payload["command_id"]),
                    target=str(payload["target"]),
                    command_type=str(payload["type"]),
                    created_at=str(payload["created_at"]),
                    expires_at=str(payload["expires_at"]),
                    arguments=dict(payload.get("arguments", {})),
                    source_file=str(row["source_file"]),
                    state=str(row["state"]),
                )
            )
        return commands

    def waiting_commands(self) -> list[InstructionCommand]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                """
                SELECT payload_json, source_file, state FROM commands
                WHERE state IN ('waiting_for_motor', 'waiting_for_permission')
                ORDER BY received_at, command_id
                """
            ).fetchall()
        commands: list[InstructionCommand] = []
        for row in rows:
            payload = json.loads(str(row["payload_json"]))
            commands.append(
                InstructionCommand(
                    command_id=str(payload["command_id"]),
                    target=str(payload["target"]),
                    command_type=str(payload["type"]),
                    created_at=str(payload["created_at"]),
                    expires_at=str(payload["expires_at"]),
                    arguments=dict(payload.get("arguments", {})),
                    source_file=str(row["source_file"]),
                    state=str(row["state"]),
                )
            )
        return commands

    def get_command(self, command_id: str) -> CommandRecord | None:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                """
                SELECT command_id, command_type, state, received_at, finished_at,
                       error_code, error_message, result_json
                FROM commands WHERE command_id = ?
                """,
                (command_id,),
            ).fetchone()
        if row is None:
            return None
        return CommandRecord(
            command_id=str(row["command_id"]),
            command_type=str(row["command_type"]),
            state=str(row["state"]),
            received_at=str(row["received_at"]),
            finished_at=str(row["finished_at"]),
            error_code=str(row["error_code"]),
            error_message=str(row["error_message"]),
            result=json.loads(str(row["result_json"])),
        )

    def add_event(
        self,
        *,
        instance_id: str,
        command_id: str | None,
        event_type: str,
        created_at: str,
        source_file: str,
        details: dict[str, object],
    ) -> dict[str, object]:
        event_id = str(uuid.uuid4())
        file_name = f"event_{event_id}.json"
        with self._lock, self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT COALESCE(MAX(sequence), 0) + 1 AS next FROM events WHERE command_id IS ?",
                (command_id,),
            ).fetchone()
            sequence = int(row["next"])
            payload: dict[str, object] = {
                "schema": INSTRUCTION_SCHEMA,
                "protocol": INSTRUCTION_PROTOCOL,
                "event_id": event_id,
                "command_id": command_id,
                "sequence": sequence,
                "type": event_type,
                "created_at": created_at,
                "instance_id": instance_id,
                "source_file": source_file,
                "details": details,
            }
            connection.execute(
                """
                INSERT INTO events(
                    event_id, command_id, sequence, event_type, created_at,
                    payload_json, file_name, exported
                ) VALUES (?, ?, ?, ?, ?, ?, ?, 0)
                """,
                (
                    event_id,
                    command_id,
                    sequence,
                    event_type,
                    created_at,
                    json.dumps(
                        payload,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        allow_nan=False,
                    ),
                    file_name,
                ),
            )
        return payload

    def unexported_events(self, limit: int = 500) -> list[tuple[str, dict[str, object]]]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                """
                SELECT file_name, payload_json FROM events
                WHERE exported = 0 ORDER BY created_at, event_id LIMIT ?
                """,
                (limit,),
            ).fetchall()
        return [
            (str(row["file_name"]), json.loads(str(row["payload_json"]))) for row in rows
        ]

    def mark_event_exported(self, file_name: str) -> None:
        with self._lock, self._connect() as connection:
            connection.execute(
                "UPDATE events SET exported = 1 WHERE file_name = ?", (file_name,)
            )

    def has_events(self) -> bool:
        with self._lock, self._connect() as connection:
            return connection.execute("SELECT 1 FROM events LIMIT 1").fetchone() is not None

    def reset_event_exports(self) -> None:
        with self._lock, self._connect() as connection:
            connection.execute("UPDATE events SET exported = 0")

    def command_counts(self) -> dict[str, int]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                "SELECT state, COUNT(*) AS count FROM commands GROUP BY state"
            ).fetchall()
        return {str(row["state"]): int(row["count"]) for row in rows}

    def recent_commands(self, limit: int = 50) -> list[CommandRecord]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                """
                SELECT command_id, command_type, state, received_at, finished_at,
                       error_code, error_message, result_json
                FROM commands ORDER BY received_at DESC, command_id DESC LIMIT ?
                """,
                (limit,),
            ).fetchall()
        return [
            CommandRecord(
                command_id=str(row["command_id"]),
                command_type=str(row["command_type"]),
                state=str(row["state"]),
                received_at=str(row["received_at"]),
                finished_at=str(row["finished_at"]),
                error_code=str(row["error_code"]),
                error_message=str(row["error_message"]),
                result=json.loads(str(row["result_json"])),
            )
            for row in rows
        ]


class InstructionRunner:
    """Scan, validate, deduplicate and execute the schema-1 capability set."""

    def __init__(
        self,
        store: InstructionStore,
        exchange_root: Path,
        *,
        app_version: str,
        now_factory: Callable[[], datetime] = utc_now,
        current_trial_gateway: Callable[
            [str, str, dict[str, object]], dict[str, object]
        ]
        | None = None,
        context_factory: Callable[[], dict[str, object]] | None = None,
    ) -> None:
        self.store = store
        self.app_version = app_version
        self.now_factory = now_factory
        self.current_trial_gateway = current_trial_gateway
        self.context_factory = context_factory
        self._lock = threading.RLock()
        self._state = "listening"
        self.exchange_root = Path(exchange_root)
        self.configure_exchange_root(exchange_root)

    @property
    def instance_id(self) -> str:
        return self.store.instance_id

    @property
    def commands_dir(self) -> Path:
        return self.exchange_root / "inbox" / "commands"

    @property
    def events_dir(self) -> Path:
        return self.exchange_root / "outbox" / "events"

    @property
    def artifacts_dir(self) -> Path:
        return self.exchange_root / "outbox" / "artifacts"

    @property
    def status_path(self) -> Path:
        return self.exchange_root / "outbox" / "status.json"

    def configure_exchange_root(self, exchange_root: Path) -> None:
        resolved = Path(exchange_root).expanduser().resolve(strict=False)
        if resolved.exists() and not resolved.is_dir():
            raise InstructionRunnerError(f"Путь обмена является файлом: {resolved}")
        previous_root = self.store.get_metadata("exchange_root")
        self.exchange_root = resolved
        for directory in (self.commands_dir, self.events_dir, self.artifacts_dir):
            directory.mkdir(parents=True, exist_ok=True)
        self.store.set_metadata("exchange_root", str(resolved))
        path_changed = bool(previous_root) and previous_root.casefold() != str(resolved).casefold()
        output_missing = self.store.has_events() and not any(
            self.events_dir.glob("event_*.json")
        )
        if path_changed or output_missing:
            self.store.reset_event_exports()
        self._flush_events()
        self._write_status()

    def snapshot(self) -> InstructionSnapshot:
        return InstructionSnapshot(
            instance_id=self.instance_id,
            exchange_root=self.exchange_root,
            state=self._state,
            last_scan_at=self.store.get_metadata("last_scan_at"),
            last_error=self.store.get_metadata("last_error"),
            command_counts=self.store.command_counts(),
            recent_commands=self.store.recent_commands(),
        )

    def _status_payload(self) -> dict[str, object]:
        counts = self.store.command_counts()
        last_error = self.store.get_metadata("last_error")
        context = self._context_payload()
        return {
            "schema": INSTRUCTION_SCHEMA,
            "protocol": INSTRUCTION_PROTOCOL,
            "instance_id": self.instance_id,
            "app_version": self.app_version,
            "updated_at": _iso(self.now_factory()),
            "state": self._state,
            "last_scan_at": self.store.get_metadata("last_scan_at"),
            "last_error_present": bool(last_error),
            "command_counts": counts,
            "pending_commands": sum(
                counts.get(state, 0)
                for state in ("received", "running", *sorted(WAITING_COMMAND_STATES))
            ),
            "motor_connected": bool(context.get("motor_connected", False)),
            "hardware_commands_enabled": False,
            "remote_hardware_execution_enabled": False,
            "hardware_requests_accepted": True,
            "current_trial_controller_connected": self.current_trial_gateway is not None,
            "accepted_command_types": [item.name for item in CAPABILITIES],
            "paths": {
                "commands": "inbox/commands",
                "events": "outbox/events",
                "artifacts": "outbox/artifacts",
            },
        }

    def _context_payload(self) -> dict[str, object]:
        if self.context_factory is None:
            return {
                "motor_connected": False,
                "project_open": False,
                "pwm_disabled": True,
            }
        try:
            payload = self.context_factory()
        except Exception as exc:  # noqa: BLE001 - status must remain writable
            return {"context_error": str(exc), "motor_connected": False}
        return dict(payload) if isinstance(payload, dict) else {"motor_connected": False}

    def _write_status(self) -> None:
        _atomic_write_json(self.status_path, self._status_payload())

    def _emit(
        self,
        event_type: str,
        *,
        command_id: str | None,
        source_file: str,
        details: dict[str, object],
    ) -> None:
        self.store.add_event(
            instance_id=self.instance_id,
            command_id=command_id,
            event_type=event_type,
            created_at=_iso(self.now_factory()),
            source_file=source_file,
            details=details,
        )
        self._flush_events()

    def _flush_events(self) -> None:
        for file_name, payload in self.store.unexported_events():
            _atomic_write_json(self.events_dir / file_name, payload)
            self.store.mark_event_exported(file_name)

    @staticmethod
    def _candidate_fingerprint(path: Path, label: str) -> str:
        stat = path.lstat()
        material = f"{label}:{path.name}:{stat.st_size}:{stat.st_mtime_ns}"
        return hashlib.sha256(material.encode("utf-8")).hexdigest()

    def _read_candidate(self, path: Path) -> tuple[bytes, str]:
        if path.is_symlink():
            raise _RejectedCommandFile(
                "symlink_not_allowed",
                "Символические ссылки во входящей папке запрещены",
                self._candidate_fingerprint(path, "symlink"),
            )
        before = path.stat()
        if before.st_size > MAX_COMMAND_FILE_BYTES:
            raise _RejectedCommandFile(
                "file_too_large",
                f"Команда больше допустимых {MAX_COMMAND_FILE_BYTES} байт",
                self._candidate_fingerprint(path, "oversize"),
            )
        try:
            raw = path.read_bytes()
        except OSError as exc:
            raise _TransientCommandRead(str(exc)) from exc
        after = path.stat()
        if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
            raise _TransientCommandRead("Файл менялся во время чтения")
        return raw, hashlib.sha256(raw).hexdigest()

    @staticmethod
    def _command_id_from_name(name: str) -> str | None:
        match = COMMAND_NAME_PATTERN.fullmatch(name)
        if match is None:
            return None
        try:
            return str(uuid.UUID(match.group(1)))
        except ValueError:
            return None

    @staticmethod
    def _validate_arguments(command: InstructionCommand) -> None:
        if command.command_type in {"ping", "get_status", "list_capabilities", "self_test"}:
            if command.arguments:
                raise InstructionValidationError(
                    "invalid_arguments",
                    f"Команда {command.command_type} не принимает arguments",
                    command_id=command.command_id,
                )
            return
        if command.command_type == "dry_run":
            candidate = command.arguments.get("command")
            if not isinstance(candidate, dict):
                raise InstructionValidationError(
                    "invalid_arguments",
                    "dry_run требует arguments.command в виде объекта",
                    command_id=command.command_id,
                )
            candidate_type = candidate.get("type")
            candidate_arguments = candidate.get("arguments", {})
            if not isinstance(candidate_type, str) or not re.fullmatch(
                r"[a-z][a-z0-9_]{0,63}", candidate_type.strip().lower()
            ):
                raise InstructionValidationError(
                    "invalid_arguments",
                    "arguments.command.type имеет недопустимый формат",
                    command_id=command.command_id,
                )
            if not isinstance(candidate_arguments, dict):
                raise InstructionValidationError(
                    "invalid_arguments",
                    "arguments.command.arguments должен быть объектом",
                    command_id=command.command_id,
                )
            _check_json_shape(candidate_arguments)
            return
        if command.command_type == "run_current_trial":
            unknown = set(command.arguments) - {"mode", "config"}
            if unknown:
                raise InstructionValidationError(
                    "invalid_arguments",
                    "run_current_trial принимает только mode и config",
                    command_id=command.command_id,
                )
            mode = command.arguments.get("mode", "hardware")
            if mode not in {"hardware", "simulation"}:
                raise InstructionValidationError(
                    "invalid_arguments",
                    "mode должен быть hardware или simulation",
                    command_id=command.command_id,
                )
            config = command.arguments.get("config", {})
            if not isinstance(config, dict):
                raise InstructionValidationError(
                    "invalid_arguments",
                    "config должен быть JSON-объектом",
                    command_id=command.command_id,
                )
            _check_json_shape(config)
            return
        if command.command_type == "cancel_current_trial":
            if set(command.arguments) != {"command_id"}:
                raise InstructionValidationError(
                    "invalid_arguments",
                    "cancel_current_trial требует только arguments.command_id",
                    command_id=command.command_id,
                )
            target = command.arguments.get("command_id")
            try:
                uuid.UUID(str(target))
            except (ValueError, AttributeError) as exc:
                raise InstructionValidationError(
                    "invalid_arguments",
                    "arguments.command_id должен быть UUID",
                    command_id=command.command_id,
                ) from exc
            return
        raise InstructionValidationError(
            "invalid_arguments",
            f"Нет проверки arguments для {command.command_type}",
            command_id=command.command_id,
        )

    def _record_rejected_file(
        self,
        *,
        source_file: str,
        digest: str,
        command_id: str | None,
        code: str,
        message: str,
    ) -> None:
        if self.store.has_seen_file(source_file, digest):
            return
        now = _iso(self.now_factory())
        self.store.record_seen_file(
            source_file,
            digest,
            command_id=command_id,
            outcome="rejected",
            seen_at=now,
        )
        self._emit(
            "rejected",
            command_id=command_id,
            source_file=source_file,
            details={"code": code, "message": message, "sha256": digest},
        )

    def _intake_file(self, path: Path) -> str:
        source_file = f"commands/{path.name}"
        try:
            raw, digest = self._read_candidate(path)
        except _TransientCommandRead:
            return "deferred"
        except _RejectedCommandFile as exc:
            if self.store.has_seen_file(source_file, exc.fingerprint):
                return "duplicate"
            self._record_rejected_file(
                source_file=source_file,
                digest=exc.fingerprint,
                command_id=self._command_id_from_name(path.name),
                code=exc.code,
                message=str(exc),
            )
            return "rejected"

        if self.store.has_seen_file(source_file, digest):
            return "duplicate"
        command_id_hint = self._command_id_from_name(path.name)
        try:
            text = raw.decode("utf-8")
            payload = json.loads(
                text,
                parse_constant=_reject_json_constant,
                object_pairs_hook=_unique_json_object,
            )
            command = InstructionCommand.from_payload(
                payload,
                source_file=path.name,
                now=self.now_factory(),
            )
        except UnicodeDecodeError:
            self._record_rejected_file(
                source_file=source_file,
                digest=digest,
                command_id=command_id_hint,
                code="invalid_utf8",
                message="Файл команды должен быть UTF-8",
            )
            return "rejected"
        except json.JSONDecodeError as exc:
            self._record_rejected_file(
                source_file=source_file,
                digest=digest,
                command_id=command_id_hint,
                code="invalid_json",
                message=f"Некорректный JSON: строка {exc.lineno}, столбец {exc.colno}",
            )
            return "rejected"
        except (ValueError, InstructionValidationError) as exc:
            if isinstance(exc, InstructionValidationError):
                code = exc.code
                message = str(exc)
                command_id = exc.command_id or command_id_hint
            else:
                code = "invalid_json"
                message = str(exc)
                command_id = command_id_hint
            self._record_rejected_file(
                source_file=source_file,
                digest=digest,
                command_id=command_id,
                code=code,
                message=message,
            )
            return "rejected"

        existing_digest = self.store.get_command_digest(command.command_id)
        if existing_digest is not None:
            self.store.record_seen_file(
                source_file,
                digest,
                command_id=command.command_id,
                outcome="duplicate" if existing_digest == digest else "conflict",
                seen_at=_iso(self.now_factory()),
            )
            if existing_digest == digest:
                return "duplicate"
            self._emit(
                "rejected",
                command_id=command.command_id,
                source_file=source_file,
                details={
                    "code": "command_id_conflict",
                    "message": "Этот command_id уже использован с другим содержимым",
                    "sha256": digest,
                    "original_sha256": existing_digest,
                },
            )
            return "rejected"

        now_text = _iso(self.now_factory())
        if command.target not in {"*", self.instance_id}:
            self.store.insert_command(
                command, digest=digest, state="ignored", received_at=now_text
            )
            self.store.record_seen_file(
                source_file,
                digest,
                command_id=command.command_id,
                outcome="ignored",
                seen_at=now_text,
            )
            self.store.update_command(
                command.command_id,
                state="ignored",
                finished_at=now_text,
                error_code="target_mismatch",
                error_message="Команда адресована другому экземпляру FOCTwin",
            )
            self._emit(
                "ignored",
                command_id=command.command_id,
                source_file=source_file,
                details={"code": "target_mismatch", "target": command.target},
            )
            return "ignored"

        if command.command_type not in CAPABILITY_BY_NAME:
            self.store.insert_command(
                command, digest=digest, state="rejected", received_at=now_text
            )
            self.store.record_seen_file(
                source_file,
                digest,
                command_id=command.command_id,
                outcome="rejected",
                seen_at=now_text,
            )
            hardware_hint = command.command_type in HARDWARE_COMMAND_HINTS
            code = "hardware_commands_disabled" if hardware_hint else "type_not_allowed"
            message = (
                "Тип аппаратной команды отсутствует в безопасном списке FOCTwin"
                if hardware_hint
                else "Тип команды отсутствует в жёстком списке возможностей"
            )
            self.store.update_command(
                command.command_id,
                state="rejected",
                finished_at=now_text,
                error_code=code,
                error_message=message,
            )
            self._emit(
                "rejected",
                command_id=command.command_id,
                source_file=source_file,
                details={"code": code, "message": message, "type": command.command_type},
            )
            return "rejected"

        try:
            self._validate_arguments(command)
        except InstructionValidationError as exc:
            self.store.insert_command(
                command, digest=digest, state="rejected", received_at=now_text
            )
            self.store.record_seen_file(
                source_file,
                digest,
                command_id=command.command_id,
                outcome="rejected",
                seen_at=now_text,
            )
            self.store.update_command(
                command.command_id,
                state="rejected",
                finished_at=now_text,
                error_code=exc.code,
                error_message=str(exc),
            )
            self._emit(
                "rejected",
                command_id=command.command_id,
                source_file=source_file,
                details={"code": exc.code, "message": str(exc)},
            )
            return "rejected"

        inserted = self.store.insert_command(
            command, digest=digest, state="received", received_at=now_text
        )
        self.store.record_seen_file(
            source_file,
            digest,
            command_id=command.command_id,
            outcome="accepted" if inserted else "duplicate",
            seen_at=now_text,
        )
        if not inserted:
            return "duplicate"
        self._emit(
            "accepted",
            command_id=command.command_id,
            source_file=source_file,
            details={
                "type": command.command_type,
                "hardware_access": CAPABILITY_BY_NAME[command.command_type].hardware_access,
                "remote_hardware_execution_enabled": False,
            },
        )
        return "accepted"

    def _call_current_trial_gateway(
        self,
        action: str,
        command_id: str,
        arguments: dict[str, object],
    ) -> dict[str, object]:
        if self.current_trial_gateway is None:
            if action in {"submit", "refresh"} and arguments.get("mode", "hardware") == "hardware":
                return {
                    "schema": 1,
                    "request_id": command_id,
                    "command_id": command_id,
                    "source": "instruction",
                    "mode": "hardware",
                    "state": "waiting_for_motor",
                    "terminal": False,
                    "reason": "controller_not_connected",
                    "message": "Общий контроллер недоступен; аппаратный запрос сохранён без запуска",
                    "environment": self._context_payload(),
                    "result": {},
                    "remote_hardware_execution_enabled": False,
                }
            raise InstructionRunnerError(
                "Общий контроллер токового опыта не подключён к окну инструкций"
            )
        try:
            response = self.current_trial_gateway(action, command_id, dict(arguments))
        except InstructionRunnerError:
            raise
        except Exception as exc:
            raise InstructionRunnerError(str(exc)) from exc
        if not isinstance(response, dict):
            raise InstructionRunnerError("Контроллер вернул некорректный ответ")
        state = response.get("state")
        allowed_states = WAITING_COMMAND_STATES | {"completed", "failed", "cancelled"}
        if state not in allowed_states:
            raise InstructionRunnerError(
                f"Контроллер вернул недопустимое состояние: {state}"
            )
        if bool(response.get("remote_hardware_execution_enabled", False)):
            raise InstructionRunnerError(
                "Контроллер попытался включить запрещённое удалённое аппаратное выполнение"
            )
        return dict(response)

    def _write_current_trial_artifact(
        self,
        command_id: str,
        payload: dict[str, object],
    ) -> str:
        relative = Path(command_id) / "current_trial_simulation.json"
        _atomic_write_json(self.artifacts_dir / relative, payload)
        return (Path("artifacts") / relative).as_posix()

    def _cancel_current_trial_command(
        self,
        command: InstructionCommand,
    ) -> CommandExecution:
        target_id = str(uuid.UUID(str(command.arguments["command_id"])))
        target = self.store.get_command(target_id)
        if target is None:
            raise InstructionRunnerError("Отменяемый command_id не найден")
        if target.command_type != "run_current_trial":
            raise InstructionRunnerError("command_id относится не к токовому опыту")
        if target.state in TERMINAL_COMMAND_STATES:
            return CommandExecution(
                "completed",
                {
                    "cancelled": target.state == "cancelled",
                    "target_command_id": target_id,
                    "target_state": target.state,
                    "message": "Запрос уже находился в конечном состоянии",
                },
            )

        controller_result: dict[str, object] = {}
        if target.state in WAITING_COMMAND_STATES and self.current_trial_gateway is not None:
            controller_result = self._call_current_trial_gateway("cancel", target_id, {})
        finished = _iso(self.now_factory())
        target_result = {
            "cancelled": True,
            "target_command_id": target_id,
            "previous_state": target.state,
            "controller": controller_result,
        }
        self.store.update_command(
            target_id,
            state="cancelled",
            finished_at=finished,
            result=target_result,
            error_code="cancelled_by_instruction",
            error_message="Ожидающий токовый опыт отменён отдельной инструкцией",
        )
        self._emit(
            "cancelled",
            command_id=target_id,
            source_file=f"commands/cmd_{target_id}.json",
            details=target_result,
        )
        return CommandExecution(
            "completed",
            {
                "cancelled": True,
                "target_command_id": target_id,
                "previous_state": target.state,
            },
        )

    def _execute_command(self, command: InstructionCommand) -> CommandExecution:
        if command.command_type == "ping":
            return CommandExecution(
                "completed",
                {
                    "pong": True,
                    "app_version": self.app_version,
                    "instance_id": self.instance_id,
                    "handled_at": _iso(self.now_factory()),
                },
            )
        if command.command_type == "get_status":
            return CommandExecution("completed", self._status_payload())
        if command.command_type == "list_capabilities":
            return CommandExecution(
                "completed",
                {
                    "capabilities": [item.to_dict() for item in CAPABILITIES],
                    "hardware_commands_enabled": False,
                    "remote_hardware_execution_enabled": False,
                },
            )
        if command.command_type == "self_test":
            return CommandExecution(
                "completed",
                {
                    "ok": True,
                    "checks": {
                        "sqlite": self.store.db_path.is_file(),
                        "commands_directory": self.commands_dir.is_dir(),
                        "events_directory": self.events_dir.is_dir(),
                        "artifacts_directory": self.artifacts_dir.is_dir(),
                        "status_file": self.status_path.is_file(),
                    },
                    "runner_hardware_access": False,
                    "shared_controller_connected": self.current_trial_gateway is not None,
                    "remote_hardware_execution_enabled": False,
                },
            )
        if command.command_type == "dry_run":
            candidate = dict(command.arguments["command"])
            candidate_type = str(candidate["type"]).strip().lower()
            if candidate_type in CAPABILITY_BY_NAME:
                capability = CAPABILITY_BY_NAME[candidate_type]
                return CommandExecution(
                    "completed",
                    {
                        "allowed": True,
                        "type": candidate_type,
                        "would_access_hardware": capability.hardware_access,
                        "execution_policy": capability.execution_policy,
                        "remote_hardware_execution_enabled": False,
                        "executed": False,
                    },
                )
            hardware_hint = candidate_type in HARDWARE_COMMAND_HINTS
            return CommandExecution(
                "completed",
                {
                    "allowed": False,
                    "type": candidate_type,
                    "reason": "type_not_allowed",
                    "would_access_hardware": hardware_hint,
                    "executed": False,
                },
            )
        if command.command_type == "run_current_trial":
            response = self._call_current_trial_gateway(
                "submit", command.command_id, command.arguments
            )
            state = str(response["state"])
            if state == "completed" and isinstance(response.get("result"), dict):
                artifact = self._write_current_trial_artifact(command.command_id, response)
                response["artifact"] = artifact
            return CommandExecution(state, response)
        if command.command_type == "cancel_current_trial":
            return self._cancel_current_trial_command(command)
        raise InstructionRunnerError(f"Нет обработчика для {command.command_type}")

    def _execute_pending(self) -> int:
        completed = 0
        for command in self.store.pending_commands():
            current = self.store.get_command(command.command_id)
            if current is None or current.state not in {"received", "running"}:
                continue
            now = self.now_factory()
            expires = _parse_timestamp(command.expires_at, "expires_at")
            if expires <= now:
                finished = _iso(now)
                self.store.update_command(
                    command.command_id,
                    state="rejected",
                    finished_at=finished,
                    error_code="expired_before_execution",
                    error_message="Команда истекла до выполнения",
                )
                self._emit(
                    "rejected",
                    command_id=command.command_id,
                    source_file=f"commands/{command.source_file}",
                    details={
                        "code": "expired_before_execution",
                        "message": "Команда истекла до выполнения",
                    },
                )
                continue
            if current.state != "running":
                started = _iso(now)
                self.store.update_command(
                    command.command_id,
                    state="running",
                    started_at=started,
                    error_code="",
                    error_message="",
                )
                event_type = (
                    "evaluating"
                    if command.command_type == "run_current_trial"
                    else "running"
                )
                self._emit(
                    event_type,
                    command_id=command.command_id,
                    source_file=f"commands/{command.source_file}",
                    details={
                        "type": command.command_type,
                        "hardware_running": False,
                    },
                )
            try:
                execution = self._execute_command(command)
            except (
                InstructionRunnerError,
                KeyError,
                OSError,
                TypeError,
                ValueError,
                sqlite3.Error,
            ) as exc:
                finished = _iso(self.now_factory())
                self.store.update_command(
                    command.command_id,
                    state="failed",
                    finished_at=finished,
                    error_code="execution_failed",
                    error_message=str(exc),
                )
                self._emit(
                    "failed",
                    command_id=command.command_id,
                    source_file=f"commands/{command.source_file}",
                    details={"code": "execution_failed", "message": str(exc)},
                )
                continue
            if execution.state in WAITING_COMMAND_STATES:
                self.store.update_command(
                    command.command_id,
                    state=execution.state,
                    result=execution.result,
                    error_code=str(execution.result.get("reason", "")),
                    error_message=str(execution.result.get("message", "")),
                )
                self._emit(
                    execution.state,
                    command_id=command.command_id,
                    source_file=f"commands/{command.source_file}",
                    details={"type": command.command_type, "result": execution.result},
                )
                continue
            finished = _iso(self.now_factory())
            state = execution.state if execution.terminal else "failed"
            self.store.update_command(
                command.command_id,
                state=state,
                finished_at=finished,
                result=execution.result,
                error_code=("" if state == "completed" else str(execution.result.get("reason", ""))),
                error_message=(
                    "" if state == "completed" else str(execution.result.get("message", ""))
                ),
            )
            self._emit(
                state,
                command_id=command.command_id,
                source_file=f"commands/{command.source_file}",
                details={"type": command.command_type, "result": execution.result},
            )
            if state == "completed":
                completed += 1
        return completed

    def _refresh_waiting(self) -> None:
        if self.current_trial_gateway is None:
            return
        for command in self.store.waiting_commands():
            now = self.now_factory()
            if _parse_timestamp(command.expires_at, "expires_at") <= now:
                finished = _iso(now)
                self.store.update_command(
                    command.command_id,
                    state="rejected",
                    finished_at=finished,
                    error_code="expired_while_waiting",
                    error_message="Команда истекла во время безопасного ожидания",
                )
                self._emit(
                    "rejected",
                    command_id=command.command_id,
                    source_file=f"commands/{command.source_file}",
                    details={
                        "code": "expired_while_waiting",
                        "message": "Команда истекла во время безопасного ожидания",
                    },
                )
                continue
            response = self._call_current_trial_gateway(
                "refresh", command.command_id, command.arguments
            )
            state = str(response["state"])
            if state == command.state:
                self.store.update_command(
                    command.command_id,
                    state=state,
                    result=response,
                    error_code=str(response.get("reason", "")),
                    error_message=str(response.get("message", "")),
                )
                continue
            finished_at = _iso(self.now_factory()) if state in TERMINAL_COMMAND_STATES else None
            self.store.update_command(
                command.command_id,
                state=state,
                finished_at=finished_at,
                result=response,
                error_code=str(response.get("reason", "")),
                error_message=str(response.get("message", "")),
            )
            self._emit(
                state,
                command_id=command.command_id,
                source_file=f"commands/{command.source_file}",
                details={"type": command.command_type, "result": response},
            )

    def scan_once(self) -> ScanResult:
        with self._lock:
            self._state = "scanning"
            self.store.set_metadata("last_error", "")
            counters = {
                "accepted": 0,
                "completed": 0,
                "rejected": 0,
                "ignored": 0,
                "duplicates": 0,
                "deferred": 0,
                "waiting": 0,
            }
            try:
                candidates = sorted(
                    (
                        path
                        for path in self.commands_dir.glob("*.json")
                        if path.is_file()
                        and not path.name.casefold().startswith(".folderbridge.part-")
                    ),
                    key=lambda path: path.name.casefold(),
                )
                cursor = self.store.get_metadata("scan_cursor").casefold()
                split = next(
                    (
                        index
                        for index, path in enumerate(candidates)
                        if path.name.casefold() > cursor
                    ),
                    len(candidates),
                )
                ordered = candidates[split:] + candidates[:split]
                batch = ordered[:MAX_COMMAND_FILES_PER_SCAN]
                for path in batch:
                    outcome = self._intake_file(path)
                    if outcome == "duplicate":
                        counters["duplicates"] += 1
                    elif outcome in counters:
                        counters[outcome] += 1
                if batch:
                    self.store.set_metadata("scan_cursor", batch[-1].name)
                counters["deferred"] += max(0, len(candidates) - len(batch))
                counters["completed"] = self._execute_pending()
                self._refresh_waiting()
                current_counts = self.store.command_counts()
                counters["waiting"] = sum(
                    current_counts.get(state, 0) for state in WAITING_COMMAND_STATES
                )
                self.store.set_metadata("last_scan_at", _iso(self.now_factory()))
            except (
                InstructionRunnerError,
                KeyError,
                OSError,
                TypeError,
                ValueError,
                sqlite3.Error,
            ) as exc:
                self.store.set_metadata("last_error", str(exc))
                if isinstance(exc, InstructionRunnerError):
                    raise
                raise InstructionRunnerError(str(exc)) from exc
            finally:
                self._state = "listening"
                self._flush_events()
                self._write_status()
            return ScanResult(**counters)

    def create_sample_command(
        self,
        command_type: str,
        *,
        target_command_id: str | None = None,
    ) -> Path:
        requested_type = command_type.strip().lower()
        command_type = requested_type
        now = self.now_factory()
        if command_type == "dry_run":
            arguments: dict[str, object] = {
                "command": {"type": "run_current_trial", "arguments": {}}
            }
        elif command_type == "run_current_trial":
            arguments = {"mode": "simulation", "config": {}}
        elif command_type == "queue_current_trial":
            command_type = "run_current_trial"
            arguments = {"mode": "hardware", "config": {}}
        elif command_type == "cancel_current_trial":
            if target_command_id is None:
                raise InstructionRunnerError("Для отмены выберите ожидающий токовый опыт")
            try:
                target_command_id = str(uuid.UUID(target_command_id))
            except ValueError as exc:
                raise InstructionRunnerError("Command ID для отмены должен быть UUID") from exc
            arguments = {"command_id": target_command_id}
        elif command_type in CAPABILITY_BY_NAME:
            arguments = {}
        else:
            raise InstructionRunnerError(f"Нет тестового шаблона для {requested_type}")
        command_id = str(uuid.uuid4())
        payload: dict[str, object] = {
            "schema": INSTRUCTION_SCHEMA,
            "protocol": INSTRUCTION_PROTOCOL,
            "command_id": command_id,
            "target": self.instance_id,
            "type": command_type,
            "created_at": _iso(now),
            "expires_at": _iso(now + timedelta(hours=1)),
            "arguments": arguments,
        }
        path = self.commands_dir / f"cmd_{command_id}.json"
        _atomic_write_json(path, payload)
        return path
