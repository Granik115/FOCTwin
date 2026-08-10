import json
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
        self.assertEqual(waiting_permission.reason_code, "local_arm_required")
        self.assertNotEqual(waiting_permission.state, CurrentTrialRequestState.READY)

    def test_waiting_remote_request_survives_controller_restart(self):
        command_id = "00000000-0000-4000-8000-000000000002"
        first = self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {"step_current_a": 0.02}},
            CurrentTrialEnvironment(),
        )

        restarted = CurrentTrialController(self.root, now_factory=self.clock)
        restored = restarted.get_by_command(command_id)

        self.assertIsNotNone(restored)
        self.assertEqual(restored.request_id, first.request_id)
        self.assertEqual(restored.state, CurrentTrialRequestState.WAITING_FOR_MOTOR)
        self.assertEqual(restored.config["step_current_a"], 0.02)

    def test_simulation_completes_without_hardware_and_preserves_plan(self):
        decision = self.controller.submit_instruction(
            "00000000-0000-4000-8000-000000000003",
            {
                "mode": "simulation",
                "config": {
                    "step_current_a": 0.02,
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
        self.assertEqual(decision.result["targets"]["step_a"], 0.02)
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

    def test_remote_request_requires_explicit_arm_and_separate_one_shot_start(self):
        command_id = "00000000-0000-4000-8000-000000000006"
        waiting = self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            ready_environment(),
        )

        armed = self.controller.arm_instruction(
            command_id,
            ready_environment(permission=True),
        )
        started = self.controller.start_instruction(
            command_id,
            ready_environment(),
        )

        self.assertEqual(waiting.state, CurrentTrialRequestState.WAITING_FOR_PERMISSION)
        self.assertEqual(armed.state, CurrentTrialRequestState.READY)
        self.assertTrue(armed.to_dict()["locally_armed"])
        self.assertTrue(armed.to_dict()["remote_hardware_execution_enabled"])
        self.assertFalse(armed.result["arm_consumed"])
        self.assertEqual(started.state, CurrentTrialRequestState.RUNNING)
        self.assertTrue(started.result["arm_consumed"])
        self.assertFalse(started.to_dict()["remote_hardware_execution_enabled"])

        with self.assertRaisesRegex(CurrentTrialControllerError, "повторно разрешить"):
            self.controller.arm_instruction(
                command_id,
                ready_environment(permission=True),
            )

    def test_remote_arm_remains_ready_without_a_timer(self):
        command_id = "00000000-0000-4000-8000-000000000007"
        self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            ready_environment(),
        )
        self.controller.arm_instruction(
            command_id,
            ready_environment(permission=True),
        )
        self.clock.value += timedelta(days=7)

        still_ready = self.controller.refresh_instruction(
            command_id,
            ready_environment(),
        )

        self.assertEqual(still_ready.state, CurrentTrialRequestState.READY)
        self.assertEqual(still_ready.reason_code, "locally_armed")
        self.assertTrue(still_ready.to_dict()["remote_hardware_execution_enabled"])
        self.assertIn("armed_at", still_ready.result)
        self.assertNotIn("armed_until", still_ready.result)

    def test_restart_revokes_an_unconsumed_remote_arm(self):
        command_id = "00000000-0000-4000-8000-000000000010"
        self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            ready_environment(),
        )
        self.controller.arm_instruction(
            command_id,
            ready_environment(permission=True),
        )

        restarted = CurrentTrialController(self.root, now_factory=self.clock)
        restored = restarted.get_by_command(command_id)

        self.assertEqual(restored.state, CurrentTrialRequestState.WAITING_FOR_PERMISSION)
        self.assertEqual(restored.reason_code, "restart_requires_permission")
        self.assertEqual(restored.result, {})

    def test_running_local_trial_revokes_a_remote_arm(self):
        command_id = "00000000-0000-4000-8000-000000000009"
        self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            ready_environment(),
        )
        self.controller.arm_instruction(
            command_id,
            ready_environment(permission=True),
        )
        busy = CurrentTrialEnvironment(
            project_open=True,
            motor_connected=True,
            pwm_disabled=True,
            telemetry_fresh=True,
            telemetry_complete=True,
            friction_idle=True,
            current_trial_idle=False,
            command_channel_idle=True,
            phase_resistance_valid=True,
        )

        revoked = self.controller.refresh_instruction(command_id, busy)

        self.assertEqual(revoked.state, CurrentTrialRequestState.WAITING_FOR_PERMISSION)
        self.assertEqual(revoked.reason_code, "current_trial_running")

    def test_old_persisted_plan_is_failed_before_it_can_be_armed(self):
        command_id = "00000000-0000-4000-8000-000000000008"
        decision = self.controller.submit_instruction(
            command_id,
            {"mode": "hardware", "config": {}},
            ready_environment(),
        )
        old_config = dict(decision.config)
        old_config.update(
            {
                "step_current_a": 0.1,
                "current_kp": 8.4222,
                "current_ki": 814.0,
            }
        )
        with self.controller._connect() as connection:
            connection.execute(
                "UPDATE current_trial_requests SET config_json = ? WHERE request_id = ?",
                (json.dumps(old_config), decision.request_id),
            )

        failed = self.controller.refresh_instruction(command_id, ready_environment())

        self.assertEqual(failed.state, CurrentTrialRequestState.FAILED)
        self.assertEqual(failed.reason_code, "stored_config_no_longer_valid")


if __name__ == "__main__":
    unittest.main()
