import ast
import json
import tempfile
import unittest
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import patch

from foctwin.instruction_runner import (
    INSTRUCTION_PROTOCOL,
    InstructionRunner,
    InstructionStore,
)


class Clock:
    def __init__(self) -> None:
        self.value = datetime(2026, 8, 9, 18, 0, tzinfo=timezone.utc)

    def __call__(self) -> datetime:
        current = self.value
        self.value += timedelta(milliseconds=1)
        return current


class InstructionRunnerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.state_root = root / "state"
        self.exchange_root = root / "exchange"
        self.clock = Clock()
        self.store = InstructionStore(self.state_root)
        self.runner = InstructionRunner(
            self.store,
            self.exchange_root,
            app_version="test-version",
            now_factory=self.clock,
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _payload(
        self,
        command_type: str,
        *,
        command_id: str | None = None,
        target: str | None = None,
        arguments: dict[str, object] | None = None,
        created_at: datetime | None = None,
        expires_at: datetime | None = None,
    ) -> dict[str, object]:
        created = created_at or self.clock.value
        expires = expires_at or created + timedelta(hours=1)
        return {
            "schema": 1,
            "protocol": INSTRUCTION_PROTOCOL,
            "command_id": command_id or str(uuid.uuid4()),
            "target": target or self.runner.instance_id,
            "type": command_type,
            "created_at": created.isoformat(),
            "expires_at": expires.isoformat(),
            "arguments": arguments or {},
        }

    def _write(self, payload: dict[str, object], *, name: str | None = None) -> Path:
        filename = name or f"cmd_{payload['command_id']}.json"
        path = self.runner.commands_dir / filename
        path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        return path

    def _events(self, command_id: str | None = None) -> list[dict[str, object]]:
        events = [
            json.loads(path.read_text(encoding="utf-8"))
            for path in self.runner.events_dir.glob("event_*.json")
        ]
        if command_id is not None:
            events = [event for event in events if event["command_id"] == command_id]
        return sorted(events, key=lambda event: int(event["sequence"]))

    def test_layout_status_and_capabilities_are_hardware_free(self):
        self.assertTrue(self.runner.commands_dir.is_dir())
        self.assertTrue(self.runner.events_dir.is_dir())
        self.assertTrue(self.runner.artifacts_dir.is_dir())
        status = json.loads(self.runner.status_path.read_text(encoding="utf-8"))
        self.assertEqual(status["protocol"], INSTRUCTION_PROTOCOL)
        self.assertEqual(status["instance_id"], self.runner.instance_id)
        self.assertFalse(status["motor_connected"])
        self.assertFalse(status["hardware_commands_enabled"])
        self.assertEqual(
            status["accepted_command_types"],
            ["ping", "get_status", "list_capabilities", "self_test", "dry_run"],
        )
        self.assertNotIn(str(self.exchange_root), json.dumps(status))

    def test_backend_has_no_qt_serial_commander_or_experiment_import(self):
        source_path = Path(__file__).parents[1] / "src" / "foctwin" / "instruction_runner.py"
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        modules: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules.add(node.module)
        forbidden = {
            "PySide6",
            "foctwin.current_trial",
            "foctwin.friction",
            "foctwin.protocol",
            "foctwin.serial_device",
        }
        self.assertTrue(forbidden.isdisjoint(modules))

    def test_ping_is_completed_once_and_deduplicated_after_restart(self):
        payload = self._payload("ping")
        command_id = str(payload["command_id"])
        self._write(payload)

        first = self.runner.scan_once()
        self.assertEqual(first.accepted, 1)
        self.assertEqual(first.completed, 1)
        self.assertEqual(
            [event["type"] for event in self._events(command_id)],
            ["accepted", "running", "completed"],
        )
        completed = self._events(command_id)[-1]
        self.assertTrue(completed["details"]["result"]["pong"])

        second_runner = InstructionRunner(
            InstructionStore(self.state_root),
            self.exchange_root,
            app_version="test-version",
            now_factory=self.clock,
        )
        second = second_runner.scan_once()
        self.assertEqual(second.completed, 0)
        self.assertEqual(second.duplicates, 1)
        self.assertEqual(len(self._events(command_id)), 3)

    def test_changed_payload_with_same_command_id_is_a_persistent_conflict(self):
        command_id = str(uuid.uuid4())
        path = self._write(self._payload("ping", command_id=command_id))
        self.runner.scan_once()

        path.write_text(
            json.dumps(self._payload("get_status", command_id=command_id)),
            encoding="utf-8",
        )
        conflict = self.runner.scan_once()
        self.assertEqual(conflict.rejected, 1)
        events = self._events(command_id)
        self.assertEqual(events[-1]["type"], "rejected")
        self.assertEqual(events[-1]["details"]["code"], "command_id_conflict")

        repeated = self.runner.scan_once()
        self.assertEqual(repeated.rejected, 0)
        self.assertEqual(len(self._events(command_id)), len(events))

    def test_expired_and_hardware_commands_are_rejected_without_running(self):
        expired_id = str(uuid.uuid4())
        expired = self._payload(
            "ping",
            command_id=expired_id,
            created_at=self.clock.value - timedelta(hours=2),
            expires_at=self.clock.value - timedelta(hours=1),
        )
        hardware = self._payload("run_current_trial")
        hardware_id = str(hardware["command_id"])
        self._write(expired)
        self._write(hardware)

        result = self.runner.scan_once()
        self.assertEqual(result.rejected, 2)
        expired_event = self._events(expired_id)[-1]
        hardware_event = self._events(hardware_id)[-1]
        self.assertEqual(expired_event["details"]["code"], "expired")
        self.assertEqual(hardware_event["details"]["code"], "hardware_commands_disabled")
        self.assertNotIn("running", [event["type"] for event in self._events(expired_id)])
        self.assertNotIn("running", [event["type"] for event in self._events(hardware_id)])

    def test_dry_run_reports_hardware_command_but_does_not_execute_it(self):
        payload = self._payload(
            "dry_run",
            arguments={"command": {"type": "run_current_trial", "arguments": {}}},
        )
        command_id = str(payload["command_id"])
        self._write(payload)

        result = self.runner.scan_once()
        self.assertEqual(result.completed, 1)
        dry_result = self._events(command_id)[-1]["details"]["result"]
        self.assertFalse(dry_result["allowed"])
        self.assertFalse(dry_result["executed"])
        self.assertTrue(dry_result["would_access_hardware"])
        self.assertEqual(dry_result["reason"], "hardware_commands_disabled")

    def test_command_for_another_instance_is_ignored(self):
        payload = self._payload("ping", target=str(uuid.uuid4()))
        command_id = str(payload["command_id"])
        self._write(payload)

        result = self.runner.scan_once()
        self.assertEqual(result.ignored, 1)
        events = self._events(command_id)
        self.assertEqual([event["type"] for event in events], ["ignored"])
        self.assertEqual(self.store.recent_commands()[0].state, "ignored")

    def test_received_command_resumes_after_process_interruption(self):
        payload = self._payload("self_test")
        command_id = str(payload["command_id"])
        self._write(payload)
        original_execute = self.runner._execute_pending

        def interrupt():
            raise RuntimeError("simulated process interruption")

        self.runner._execute_pending = interrupt
        with self.assertRaisesRegex(RuntimeError, "simulated process interruption"):
            self.runner.scan_once()
        self.runner._execute_pending = original_execute
        self.assertEqual(self.store.recent_commands()[0].state, "received")

        resumed = InstructionRunner(
            InstructionStore(self.state_root),
            self.exchange_root,
            app_version="test-version",
            now_factory=self.clock,
        )
        result = resumed.scan_once()
        self.assertEqual(result.completed, 1)
        self.assertEqual(resumed.store.recent_commands()[0].state, "completed")
        self.assertEqual(self._events(command_id)[-1]["type"], "completed")

    def test_invalid_json_is_reported_only_once_until_file_changes(self):
        command_id = str(uuid.uuid4())
        path = self.runner.commands_dir / f"cmd_{command_id}.json"
        path.write_text("{not-json", encoding="utf-8")
        first = self.runner.scan_once()
        self.assertEqual(first.rejected, 1)
        event_count = len(self._events(command_id))

        second = self.runner.scan_once()
        self.assertEqual(second.rejected, 0)
        self.assertEqual(len(self._events(command_id)), event_count)

    def test_duplicate_json_keys_are_rejected_as_ambiguous(self):
        command_id = str(uuid.uuid4())
        path = self.runner.commands_dir / f"cmd_{command_id}.json"
        payload = self._payload("ping", command_id=command_id)
        encoded = json.dumps(payload)
        ambiguous = encoded[:-1] + ',"type":"get_status"}'
        path.write_text(ambiguous, encoding="utf-8")

        result = self.runner.scan_once()
        self.assertEqual(result.rejected, 1)
        event = self._events(command_id)[-1]
        self.assertEqual(event["details"]["code"], "invalid_json")
        self.assertIn("duplicate JSON key", event["details"]["message"])

    def test_folderbridge_partial_download_is_ignored(self):
        command_id = str(uuid.uuid4())
        temporary = self.runner.commands_dir / (
            f".folderbridge.part-transfer-cmd_{command_id}.json"
        )
        temporary.write_text("{not-finished", encoding="utf-8")

        result = self.runner.scan_once()
        self.assertEqual(result.rejected, 0)
        self.assertEqual(self._events(), [])

    def test_scan_cursor_prevents_old_files_from_starving_new_commands(self):
        payloads = [
            self._payload("ping", command_id=f"00000000-0000-4000-8000-{index:012d}")
            for index in range(1, 4)
        ]
        for payload in payloads:
            self._write(payload)

        with patch("foctwin.instruction_runner.MAX_COMMAND_FILES_PER_SCAN", 2):
            first = self.runner.scan_once()
            second = self.runner.scan_once()
        self.assertEqual(first.completed, 2)
        self.assertEqual(first.deferred, 1)
        self.assertEqual(second.completed, 1)
        self.assertEqual(len(self.store.recent_commands()), 3)

    def test_switching_exchange_root_republishes_event_history(self):
        payload = self._payload("ping")
        self._write(payload)
        self.runner.scan_once()
        self.assertEqual(len(list(self.runner.events_dir.glob("event_*.json"))), 3)

        new_root = Path(self.temporary.name) / "new-exchange"
        self.runner.configure_exchange_root(new_root)
        self.assertEqual(self.runner.exchange_root, new_root.resolve())
        self.assertEqual(len(list(self.runner.events_dir.glob("event_*.json"))), 3)


if __name__ == "__main__":
    unittest.main()
