import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

from foctwin.current_trial import CurrentTrialConfig
from foctwin.current_trial_controller import (
    CurrentTrialController,
    CurrentTrialControllerError,
    CurrentTrialEnvironment,
    CurrentTrialRequestState,
)


class Clock:
    def __init__(self) -> None:
        self.value = datetime(2026, 8, 9, 18, 0, tzinfo=timezone.utc)

    def __call__(self) -> datetime:
        current = self.value
        self.value += timedelta(milliseconds=1)
        return current


def ready_environment(*, permission: bool = False) -> CurrentTrialEnvironment:
    return CurrentTrialEnvironment(
        project_open=True,
        motor_connected=True,
        pwm_disabled=True,
        telemetry_fresh=True,
        telemetry_complete=True,
        friction_idle=True,
        command_channel_idle=True,
        phase_resistance_valid=True,
        local_permission=permission,
    )


class CurrentTrialControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.clock = Clock()
        self.controller = CurrentTrialController(self.root, now_factory=self.clock)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_remote_hardware_request_waits_for_motor_then_permission(self):
        command_id = "00000000-0000-4000-8000-000000000001"
        waiting_motor = self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            CurrentTrialEnvironment(),
        )

        self.assertEqual(
            waiting_motor.state,
            CurrentTrialRequestState.WAITING_FOR_MOTOR,
        )
        self.assertFalse(waiting_motor.terminal)
        self.assertFalse(waiting_motor.to_dict()["remote_hardware_execution_enabled"])

        waiting_permission = self.controller.refresh_instruction(
            command_id,
            ready_environment(permission=True),
        )
        self.assertEqual(
            waiting_permission.state,
            CurrentTrialRequestState.WAITING_FOR_PERMISSION,
        )
        self.assertEqual(waiting_permission.reason_code, "remote_hardware_locked")
        self.assertNotEqual(waiting_permission.state, CurrentTrialRequestState.READY)

    def test_waiting_remote_request_survives_controller_restart(self):
        command_id = "00000000-0000-4000-8000-000000000002"
        first = self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {"step_current_a": 0.12}},
            CurrentTrialEnvironment(),
        )

        restarted = CurrentTrialController(self.root, now_factory=self.clock)
        restored = restarted.get_by_command(command_id)

        self.assertIsNotNone(restored)
        self.assertEqual(restored.request_id, first.request_id)
        self.assertEqual(restored.state, CurrentTrialRequestState.WAITING_FOR_MOTOR)
        self.assertEqual(restored.config["step_current_a"], 0.12)

    def test_simulation_completes_without_hardware_and_preserves_plan(self):
        decision = self.controller.submit_instruction(
            "00000000-0000-4000-8000-000000000003",
            {
                "mode": "simulation",
                "config": {
                    "step_current_a": 0.15,
                    "baseline_s": 1.5,
                    "step_s": 2.5,
                    "post_s": 1.25,
                },
            },
            CurrentTrialEnvironment(),
        )

        self.assertEqual(decision.state, CurrentTrialRequestState.COMPLETED)
        self.assertTrue(decision.terminal)
        self.assertTrue(decision.result["simulated"])
        self.assertFalse(decision.result["executed"])
        self.assertFalse(decision.result["hardware_access"])
        self.assertEqual(decision.result["targets"]["step_a"], 0.15)
        self.assertEqual(decision.result["durations_s"]["step"], 2.5)

    def test_invalid_instruction_config_is_rejected_before_request_is_stored(self):
        with self.assertRaisesRegex(CurrentTrialControllerError, "лимита цели"):
            self.controller.submit_instruction(
                "00000000-0000-4000-8000-000000000004",
                {
                    "mode": "simulation",
                    "config": {
                        "step_current_a": 2.0,
                        "current_target_limit_a": 0.5,
                    },
                },
                CurrentTrialEnvironment(),
            )

        self.assertEqual(self.controller.recent(), [])

    def test_local_button_uses_preview_permission_running_and_completion_states(self):
        config = CurrentTrialConfig()
        blocked = self.controller.preview_local(config, CurrentTrialEnvironment())
        self.assertEqual(blocked.state, CurrentTrialRequestState.BLOCKED)
        self.assertEqual(blocked.reason_code, "project_not_open")

        confirmation = self.controller.preview_local(config, ready_environment())
        self.assertEqual(
            confirmation.state,
            CurrentTrialRequestState.WAITING_FOR_PERMISSION,
        )

        ready = self.controller.submit_local(
            config,
            ready_environment(permission=True),
        )
        running = self.controller.mark_running(ready.request_id)
        completed = self.controller.finish_local(
            ready.request_id,
            status="completed",
            result={"valid": True},
        )

        self.assertEqual(running.state, CurrentTrialRequestState.RUNNING)
        self.assertEqual(completed.state, CurrentTrialRequestState.COMPLETED)
        self.assertTrue(completed.result["valid"])

    def test_restart_never_resumes_a_locally_running_request_automatically(self):
        ready = self.controller.submit_local(
            CurrentTrialConfig(),
            ready_environment(permission=True),
        )
        self.controller.mark_running(ready.request_id)

        restarted = CurrentTrialController(self.root, now_factory=self.clock)
        restored = restarted.get(ready.request_id)

        self.assertEqual(
            restored.state,
            CurrentTrialRequestState.WAITING_FOR_PERMISSION,
        )
        self.assertEqual(restored.reason_code, "restart_requires_permission")

    def test_waiting_instruction_can_be_cancelled_idempotently(self):
        command_id = "00000000-0000-4000-8000-000000000005"
        self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            CurrentTrialEnvironment(),
        )

        cancelled = self.controller.cancel_instruction(command_id)
        repeated = self.controller.cancel_instruction(command_id)

        self.assertEqual(cancelled.state, CurrentTrialRequestState.CANCELLED)
        self.assertEqual(repeated.state, CurrentTrialRequestState.CANCELLED)
        self.assertEqual(cancelled.request_id, repeated.request_id)


if __name__ == "__main__":
    unittest.main()
