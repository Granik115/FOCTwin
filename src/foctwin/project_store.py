from __future__ import annotations

import json
import os
import sqlite3
import tempfile
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator

from foctwin.domain import MotorProfile


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ProjectStore:
    """Durable project folder; each completed trial is a resumable transaction boundary."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser().resolve()
        self.db_path = self.root / "project.sqlite3"
        self.profile_dir = self.root / "profiles"
        self.telemetry_dir = self.root / "telemetry"
        self.checkpoint_dir = self.root / "checkpoints"
        self.export_dir = self.root / "exports"
        self.log_dir = self.root / "logs"

    def initialize(self, profile: MotorProfile | None = None) -> None:
        for directory in (
            self.root,
            self.profile_dir,
            self.telemetry_dir,
            self.checkpoint_dir,
            self.export_dir,
            self.log_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)
        with self.connection() as connection:
            connection.executescript(
                """
                PRAGMA journal_mode=WAL;
                PRAGMA foreign_keys=ON;
                CREATE TABLE IF NOT EXISTS experiments (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    kind TEXT NOT NULL,
                    status TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    started_at TEXT,
                    finished_at TEXT,
                    config_json TEXT NOT NULL,
                    result_json TEXT,
                    error TEXT
                );
                CREATE TABLE IF NOT EXISTS events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    level TEXT NOT NULL,
                    source TEXT NOT NULL,
                    message TEXT NOT NULL,
                    payload_json TEXT
                );
                CREATE TABLE IF NOT EXISTS accepted_parameters (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    profile_name TEXT NOT NULL,
                    stage TEXT NOT NULL,
                    parameters_json TEXT NOT NULL,
                    score REAL,
                    note TEXT
                );
                """
            )
        if profile is not None:
            self.save_profile(profile)
        self.event("INFO", "project", "Проект инициализирован", {"root": str(self.root)})

    @contextmanager
    def connection(self) -> Iterator[sqlite3.Connection]:
        connection = sqlite3.connect(self.db_path)
        try:
            yield connection
            connection.commit()
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()

    def event(self, level: str, source: str, message: str, payload: dict[str, Any] | None = None) -> None:
        with self.connection() as connection:
            connection.execute(
                "INSERT INTO events(timestamp, level, source, message, payload_json) VALUES (?, ?, ?, ?, ?)",
                (utc_now(), level, source, message, json.dumps(payload, ensure_ascii=False) if payload else None),
            )

    def create_experiment(self, kind: str, config: dict[str, Any]) -> int:
        with self.connection() as connection:
            cursor = connection.execute(
                "INSERT INTO experiments(kind, status, created_at, config_json) VALUES (?, 'queued', ?, ?)",
                (kind, utc_now(), json.dumps(config, ensure_ascii=False)),
            )
            return int(cursor.lastrowid)

    def update_experiment(self, experiment_id: int, status: str, **fields: Any) -> None:
        allowed = {"started_at", "finished_at", "result_json", "error"}
        unknown = set(fields) - allowed
        if unknown:
            raise ValueError(f"Unknown experiment fields: {sorted(unknown)}")
        assignments = ["status = ?"] + [f"{name} = ?" for name in fields]
        values = [status] + [fields[name] for name in fields] + [experiment_id]
        with self.connection() as connection:
            connection.execute(
                f"UPDATE experiments SET {', '.join(assignments)} WHERE id = ?", values
            )

    def experiment_results(
        self,
        kind: str,
        *,
        status: str = "completed",
    ) -> list[tuple[int, dict[str, Any]]]:
        """Return durable result payloads for building cumulative identification maps."""

        with self.connection() as connection:
            rows = connection.execute(
                """
                SELECT id, result_json
                FROM experiments
                WHERE kind = ? AND status = ? AND result_json IS NOT NULL
                ORDER BY id
                """,
                (kind, status),
            ).fetchall()
        results: list[tuple[int, dict[str, Any]]] = []
        for experiment_id, result_json in rows:
            try:
                payload = json.loads(result_json)
            except (TypeError, json.JSONDecodeError):
                continue
            if isinstance(payload, dict):
                results.append((int(experiment_id), payload))
        return results

    def save_profile(self, profile: MotorProfile) -> Path:
        safe_name = "".join(char if char.isalnum() or char in "-_" else "_" for char in profile.name)
        path = self.profile_dir / f"{safe_name}.json"
        self.atomic_json(path, profile.to_dict())
        return path

    def save_checkpoint(self, stage: str, payload: dict[str, Any]) -> Path:
        path = self.checkpoint_dir / f"{stage}.json"
        envelope = {"saved_at": utc_now(), "stage": stage, "payload": payload}
        self.atomic_json(path, envelope)
        return path

    def load_checkpoint(self, stage: str) -> dict[str, Any] | None:
        path = self.checkpoint_dir / f"{stage}.json"
        if not path.exists():
            return None
        with path.open(encoding="utf-8") as handle:
            envelope = json.load(handle)
        if not isinstance(envelope, dict) or envelope.get("stage") != stage:
            raise ValueError(f"Некорректный checkpoint этапа {stage}")
        payload = envelope.get("payload")
        if not isinstance(payload, dict):
            raise ValueError(f"Checkpoint этапа {stage} не содержит данных")
        return payload

    def save_export(self, label: str, payload: dict[str, Any]) -> Path:
        safe_label = "".join(char if char.isalnum() or char in "-_" else "_" for char in label)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        path = self.export_dir / f"{timestamp}_{safe_label or 'result'}.json"
        self.atomic_json(path, payload)
        return path

    def accept_parameters(
        self,
        profile_name: str,
        stage: str,
        parameters: dict[str, Any],
        *,
        score: float | None = None,
        note: str | None = None,
    ) -> int:
        with self.connection() as connection:
            cursor = connection.execute(
                """
                INSERT INTO accepted_parameters(
                    timestamp, profile_name, stage, parameters_json, score, note
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    utc_now(),
                    profile_name,
                    stage,
                    json.dumps(parameters, ensure_ascii=False),
                    score,
                    note,
                ),
            )
            return int(cursor.lastrowid)

    def new_telemetry_path(self, label: str = "manual") -> Path:
        safe_label = "".join(char if char.isalnum() or char in "-_" else "_" for char in label)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        return self.telemetry_dir / f"{timestamp}_{safe_label or 'manual'}.csv"

    @staticmethod
    def atomic_json(path: Path, payload: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
