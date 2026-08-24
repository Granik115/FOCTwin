from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO

MAX_PROGRAM_BYTES = 256 * 1024
MAX_PROGRAM_STEPS = 10_000
MAX_COMMAND_CHARACTERS = 256
MAX_WAIT_SECONDS = 24 * 60 * 60
MAX_TOTAL_WAIT_SECONDS = 7 * 24 * 60 * 60
PROGRAM_SUFFIX = ".focscript"

_WAIT_PATTERN = re.compile(
    r"^WAIT\s+([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)$",
    flags=re.IGNORECASE,
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True, slots=True)
class ProgramStep:
    line_number: int
    source: str
    command: str | None = None
    wait_seconds: float | None = None

    @property
    def kind(self) -> str:
        return "wait" if self.wait_seconds is not None else "command"


@dataclass(frozen=True, slots=True)
class MotorProgram:
    source: str
    steps: tuple[ProgramStep, ...]

    @property
    def command_count(self) -> int:
        return sum(step.command is not None for step in self.steps)

    @property
    def wait_count(self) -> int:
        return sum(step.wait_seconds is not None for step in self.steps)

    @property
    def total_wait_seconds(self) -> float:
        return sum(step.wait_seconds or 0.0 for step in self.steps)

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.source.encode("utf-8")).hexdigest()


class ProgramError(ValueError):
    pass


class MotorProgramCompiler:
    """Parse an auditable program made only of raw firmware commands and WAIT."""

    def __init__(self, device_id: str = "A") -> None:
        if len(device_id) != 1 or not device_id.isascii() or device_id.isspace():
            raise ValueError("ID мотора должен быть одним ASCII-символом")
        self.device_id = device_id

    def compile(self, source: str) -> MotorProgram:
        if len(source.encode("utf-8")) > MAX_PROGRAM_BYTES:
            raise ProgramError("Файл программы больше допустимых 256 КиБ")

        steps: list[ProgramStep] = []
        total_wait_seconds = 0.0
        for line_number, raw_line in enumerate(source.splitlines(), start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            wait_match = _WAIT_PATTERN.fullmatch(line)
            if wait_match is not None:
                seconds = float(wait_match.group(1))
                if not math.isfinite(seconds) or seconds < 0:
                    raise ProgramError(
                        f"Строка {line_number}: WAIT должен быть конечным неотрицательным числом"
                    )
                if seconds > MAX_WAIT_SECONDS:
                    raise ProgramError(
                        f"Строка {line_number}: одна задержка не может превышать 24 часа"
                    )
                total_wait_seconds += seconds
                if total_wait_seconds > MAX_TOTAL_WAIT_SECONDS:
                    raise ProgramError("Суммарная задержка программы превышает семь суток")
                steps.append(
                    ProgramStep(
                        line_number=line_number,
                        source=line,
                        wait_seconds=seconds,
                    )
                )
            else:
                if line.upper().startswith("WAIT"):
                    raise ProgramError(
                        f"Строка {line_number}: ожидается WAIT <секунды>, например WAIT 0.5"
                    )
                self._validate_command(line_number, line)
                steps.append(
                    ProgramStep(
                        line_number=line_number,
                        source=line,
                        command=line,
                    )
                )

            if len(steps) > MAX_PROGRAM_STEPS:
                raise ProgramError("В программе больше допустимых 10 000 выполняемых строк")

        if not steps:
            raise ProgramError("В программе нет выполняемых строк")
        if not any(step.command is not None for step in steps):
            raise ProgramError("В программе нет ни одной команды прошивки")
        return MotorProgram(source=source, steps=tuple(steps))

    def _validate_command(self, line_number: int, command: str) -> None:
        if not command.startswith(self.device_id):
            raise ProgramError(
                f"Строка {line_number}: команда должна начинаться с ID мотора {self.device_id}"
            )
        if len(command) > MAX_COMMAND_CHARACTERS:
            raise ProgramError(
                f"Строка {line_number}: команда длиннее {MAX_COMMAND_CHARACTERS} символов"
            )
        try:
            command.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ProgramError(
                f"Строка {line_number}: команда прошивки должна содержать только ASCII"
            ) from exc
        if any(character.isspace() for character in command):
            raise ProgramError(
                f"Строка {line_number}: внутри команды прошивки не должно быть пробелов"
            )
        if any(ord(character) < 0x21 or ord(character) > 0x7E for character in command):
            raise ProgramError(
                f"Строка {line_number}: команда содержит недопустимый управляющий символ"
            )


def render_program(program: MotorProgram) -> str:
    lines: list[str] = []
    for step in program.steps:
        if step.command is not None:
            lines.append(f"{step.line_number:>4}: TX   {step.command}")
        else:
            lines.append(f"{step.line_number:>4}: WAIT {step.wait_seconds:g} с")
    lines.extend(
        (
            "",
            f"Команд: {program.command_count}",
            f"Задержек: {program.wait_count}",
            f"Минимальная длительность задержек: {program.total_wait_seconds:g} с",
        )
    )
    return "\n".join(lines)


def _safe_name(value: str) -> str:
    rendered = "".join(
        character if character.isalnum() or character in "-_" else "-"
        for character in value
    )
    rendered = re.sub(r"-+", "-", rendered).strip("-_")
    return rendered[:64] or "program"


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def save_program_source(path: str | Path, source: str) -> Path:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(source)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


class ProgramRunLogger:
    """Durable, plain-file journal for one attended motor-program run."""

    def __init__(
        self,
        output_root: str | Path,
        program: MotorProgram,
        *,
        program_name: str,
        source_path: str = "",
        metadata: dict[str, Any] | None = None,
    ) -> None:
        self.output_root = Path(output_root).expanduser().resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
        self.run_id = f"{stamp}_{_safe_name(program_name)}_{uuid.uuid4().hex[:8]}"
        self.run_dir = self.output_root / self.run_id
        self.run_dir.mkdir(parents=False, exist_ok=False)
        self.program_path = self.run_dir / f"program{PROGRAM_SUFFIX}"
        self.events_path = self.run_dir / "execution.jsonl"
        self.telemetry_path = self.run_dir / "telemetry.csv"
        self.summary_path = self.run_dir / "summary.json"
        self._started_monotonic = time.monotonic()
        self._sequence = 0
        self._closed = False

        with self.program_path.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write(program.source)
            handle.flush()
            os.fsync(handle.fileno())
        self.summary: dict[str, Any] = {
            "schema": 1,
            "run_id": self.run_id,
            "status": "running",
            "started_at": utc_now(),
            "finished_at": None,
            "duration_s": None,
            "error": "",
            "program": {
                "name": program_name,
                "source_path": source_path,
                "sha256": program.sha256,
                "command_count": program.command_count,
                "wait_count": program.wait_count,
                "total_wait_seconds": program.total_wait_seconds,
            },
            "files": {
                "program": self.program_path.name,
                "events": self.events_path.name,
                "telemetry": self.telemetry_path.name,
            },
            "metadata": metadata or {},
        }
        _atomic_write_json(self.summary_path, self.summary)
        self._events: TextIO = self.events_path.open(
            "a", encoding="utf-8", newline="\n", buffering=1
        )
        try:
            self.event("run_started", program=self.summary["program"], metadata=metadata or {})
        except Exception:
            self._events.close()
            self._closed = True
            raise

    @property
    def closed(self) -> bool:
        return self._closed

    def event(
        self,
        kind: str,
        *,
        line_number: int | None = None,
        **details: Any,
    ) -> None:
        if self._closed:
            return
        self._sequence += 1
        payload: dict[str, Any] = {
            "schema": 1,
            "sequence": self._sequence,
            "time_utc": utc_now(),
            "elapsed_s": round(time.monotonic() - self._started_monotonic, 6),
            "kind": kind,
        }
        if line_number is not None:
            payload["line_number"] = line_number
        payload.update(details)
        self._events.write(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        )
        self._events.flush()
        os.fsync(self._events.fileno())

    def finalize(self, status: str, *, error: str = "") -> Path:
        if self._closed:
            return self.summary_path
        duration_s = round(time.monotonic() - self._started_monotonic, 6)
        try:
            self.event("run_finished", status=status, error=error, duration_s=duration_s)
            self.summary.update(
                {
                    "status": status,
                    "finished_at": utc_now(),
                    "duration_s": duration_s,
                    "error": error,
                }
            )
            _atomic_write_json(self.summary_path, self.summary)
        finally:
            self._events.close()
            self._closed = True
        return self.summary_path
