import json
import tempfile
import unittest
from pathlib import Path

from foctwin.program import (
    MotorProgramCompiler,
    ProgramError,
    ProgramRunLogger,
    render_program,
    save_program_source,
)


class MotorProgramTests(unittest.TestCase):
    def test_program_contains_only_raw_commands_and_waits(self):
        program = MotorProgramCompiler("A").compile(
            """
            # exact Commander program
            AE0
            AC2
            A0.2
            WAIT 0.05
            A0
            AE0
            """
        )

        self.assertEqual(program.command_count, 5)
        self.assertEqual(program.wait_count, 1)
        self.assertAlmostEqual(program.total_wait_seconds, 0.05)
        self.assertEqual(program.steps[0].command, "AE0")
        self.assertEqual(program.steps[3].wait_seconds, 0.05)
        self.assertIn("TX   AE0", render_program(program))
        self.assertIn("WAIT 0.05 с", render_program(program))

    def test_old_high_level_dsl_is_not_accepted(self):
        with self.assertRaisesRegex(ProgramError, "ID мотора A"):
            MotorProgramCompiler("A").compile("MODE ANGLE")

    def test_command_must_match_active_device_and_remain_one_token(self):
        compiler = MotorProgramCompiler("A")
        with self.assertRaisesRegex(ProgramError, "ID мотора A"):
            compiler.compile("BE0")
        with self.assertRaisesRegex(ProgramError, "пробелов"):
            compiler.compile("AE 0")

    def test_wait_requires_seconds_and_rejects_negative_values(self):
        compiler = MotorProgramCompiler("A")
        with self.assertRaisesRegex(ProgramError, "WAIT <секунды>"):
            compiler.compile("AE0\nWAIT")
        with self.assertRaisesRegex(ProgramError, "неотрицательным"):
            compiler.compile("AE0\nWAIT -1")

    def test_source_save_and_run_journal_are_plain_durable_files(self):
        program = MotorProgramCompiler("A").compile("AE0\nWAIT 0\nAE1")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_path = save_program_source(root / "inbox" / "test.focscript", program.source)
            logger = ProgramRunLogger(
                root / "outbox" / "runs",
                program,
                program_name="test",
                source_path=str(source_path),
                metadata={"port": "COM7"},
            )
            logger.telemetry_path.write_text("sequence\n", encoding="utf-8")
            logger.event("command", line_number=1, command="AE0", serial_write_ok=True)
            summary_path = logger.finalize("completed")

            self.assertEqual(logger.program_path.read_text(encoding="utf-8"), program.source)
            events = [
                json.loads(line)
                for line in logger.events_path.read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(
                [event["kind"] for event in events],
                ["run_started", "command", "run_finished"],
            )
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["status"], "completed")
            self.assertEqual(summary["metadata"]["port"], "COM7")
            self.assertEqual(summary["program"]["sha256"], program.sha256)
            self.assertTrue(logger.telemetry_path.exists())


if __name__ == "__main__":
    unittest.main()
