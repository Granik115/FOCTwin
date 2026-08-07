import unittest

from foctwin.current_trial import (
    CurrentTrialConfig,
    CurrentTrialExperiment,
    CurrentTrialPhase,
)
from foctwin.domain import TelemetrySample


def sample(
    timestamp_s: float,
    *,
    angle_rad: float = 0.0,
    current_q_a: float = 0.0,
    current_d_a: float = 0.0,
    voltage_q_v: float = 0.0,
    voltage_d_v: float = 0.0,
    velocity_rad_s: float = 0.0,
) -> TelemetrySample:
    return TelemetrySample(
        timestamp_s=timestamp_s,
        raw_angle_rad=angle_rad,
        angle_rad=angle_rad,
        current_q_a=current_q_a,
        current_d_a=current_d_a,
        voltage_q_v=voltage_q_v,
        voltage_d_v=voltage_d_v,
        velocity_rad_s=velocity_rad_s,
    )


class CurrentTrialConfigTests(unittest.TestCase):
    def test_default_is_conservative_and_within_requested_absolute_limits(self):
        config = CurrentTrialConfig()

        config.validate()

        self.assertEqual(config.step_current_a, 0.1)
        self.assertEqual(config.current_kp, 8.4222)
        self.assertEqual(config.current_ki, 814.0)
        self.assertEqual(config.current_target_limit_a, 0.5)
        self.assertEqual(config.current_trip_limit_a, 1.0)
        self.assertEqual(config.current_voltage_limit_v, 12.0)
        self.assertEqual(config.absolute_current_limit_a, 5.0)
        self.assertEqual(config.absolute_voltage_limit_v, 24.0)
        self.assertEqual(config.absolute_angle_min_rad, -4.0)
        self.assertEqual(config.absolute_angle_max_rad, 4.0)
        self.assertEqual(config.monitor_downsample, 10)
        self.assertEqual(config.max_recovery_attempts, 50)

    def test_config_round_trip_preserves_transport_pid(self):
        config = CurrentTrialConfig(
            transport_angle_pid={"p": 31.0, "i": 1.0, "d": 0.0, "ramp": 2.0, "lpf": 0.01}
        )

        restored = CurrentTrialConfig.from_dict(config.to_dict())

        self.assertEqual(restored.transport_angle_pid, config.transport_angle_pid)
        self.assertEqual(restored.transport_velocity_pid, config.transport_velocity_pid)


class CurrentTrialExperimentTests(unittest.TestCase):
    def setUp(self):
        self.config = CurrentTrialConfig()
        self.experiment = CurrentTrialExperiment(self.config, 0.0)
        self.experiment.seed_angle(0.0)

    def add(self, current: TelemetrySample, now_s: float | None = None):
        prepared = self.experiment.prepare_sample(current)
        return self.experiment.add_sample(
            prepared,
            current.timestamp_s if now_s is None else now_s,
            angle_prepared=True,
        )

    def advance_position(self, start_s: float, *, returning: bool = False) -> float:
        self.experiment.position_configuration_applied(start_s, returning=returning)
        self.add(sample(start_s, angle_rad=0.0))
        self.experiment.tick(start_s)
        finish = start_s + 1.1
        for index in range(12):
            timestamp = start_s + 0.1 * (index + 1)
            self.add(sample(timestamp, angle_rad=0.0))
        self.experiment.tick(finish)
        return finish

    def test_uninterrupted_trial_runs_full_safe_lifecycle_and_builds_metrics(self):
        actions = self.experiment.start(0.0)
        self.assertEqual(actions[0].kind, "configure_position")

        now = self.advance_position(0.0)
        self.assertEqual(self.experiment.phase, CurrentTrialPhase.CONFIGURING_CURRENT)

        self.experiment.current_configuration_applied(now)
        for index in range(12):
            timestamp = now + 0.1 * (index + 1)
            self.add(sample(timestamp, current_q_a=0.0))
        now += 1.2
        actions = self.experiment.tick(now)
        self.assertEqual(self.experiment.phase, CurrentTrialPhase.CURRENT_STEP)
        self.assertEqual(actions[0].value, 0.1)

        for index in range(22):
            timestamp = now + 0.1 * (index + 1)
            self.add(
                sample(
                    timestamp,
                    current_q_a=0.1,
                    current_d_a=0.002,
                    voltage_q_v=0.2,
                )
            )
        now += 2.2
        actions = self.experiment.tick(now)
        self.assertEqual(self.experiment.phase, CurrentTrialPhase.CURRENT_POST)
        self.assertEqual(actions[0].value, 0.0)

        for index in range(12):
            timestamp = now + 0.1 * (index + 1)
            self.add(sample(timestamp, current_q_a=0.0))
        now += 1.2
        actions = self.experiment.tick(now)
        self.assertEqual(self.experiment.phase, CurrentTrialPhase.CONFIGURING_RETURN)
        self.assertEqual(actions[0].kind, "configure_return")

        now = self.advance_position(now, returning=True)
        self.assertEqual(self.experiment.phase, CurrentTrialPhase.COMPLETE)

        result = self.experiment.result("completed")
        self.assertTrue(result["valid"])
        self.assertTrue(result["metrics"]["current_response_observed"])
        self.assertAlmostEqual(result["metrics"]["steady_current_q_a"], 0.1)
        self.assertEqual(result["telemetry"]["interruption_count"], 0)

    def test_two_confirmed_overcurrent_samples_abort_current_phase(self):
        self.experiment.phase = CurrentTrialPhase.CONFIGURING_CURRENT
        self.experiment.current_configuration_applied(0.0)

        violation, _ = self.add(sample(0.1, current_q_a=1.2))
        self.assertIsNone(violation)
        violation, _ = self.add(sample(0.2, current_q_a=1.2))

        self.assertIn("Полный ток", violation)

    def test_connection_loss_discards_partial_measurement_and_repeats_whole_trial(self):
        self.experiment.phase = CurrentTrialPhase.CURRENT_STEP
        self.experiment.phase_started_s = 0.0
        self.add(sample(0.1, current_q_a=0.08))

        actions = self.experiment.enter_recovery("Serial отключён")

        self.assertEqual(self.experiment.phase, CurrentTrialPhase.RECOVERING)
        self.assertEqual(actions[0].kind, "safe_stop")
        self.assertEqual(len(self.experiment.step_samples), 0)
        self.assertEqual(self.experiment.interruption_count, 1)
        self.assertEqual(self.experiment.invalid_attempts[0]["phase"], "current_step")

        resumed = self.experiment.resume_after_recovery(5.0)
        self.assertEqual(self.experiment.phase, CurrentTrialPhase.CONFIGURING_POSITION)
        self.assertEqual(resumed[0].kind, "configure_position")

    def test_checkpoint_restores_only_evidence_and_restarts_from_safe_positioning(self):
        self.experiment.phase = CurrentTrialPhase.CURRENT_STEP
        self.add(sample(0.1, current_q_a=0.08))
        self.experiment.enter_recovery("Питание отключено")
        payload = self.experiment.checkpoint_payload(17)

        restored = CurrentTrialExperiment.from_checkpoint(payload)
        actions = restored.start(10.0)

        self.assertEqual(restored.start_angle_rad, 0.0)
        self.assertEqual(restored.interruption_count, 1)
        self.assertEqual(restored.recovery_attempts, 1)
        self.assertEqual(restored.total_sample_count, 1)
        self.assertEqual(restored.phase, CurrentTrialPhase.CONFIGURING_POSITION)
        self.assertEqual(actions[0].kind, "configure_position")

    def test_first_large_angle_jump_is_rejected_but_persistent_jump_is_not_hidden(self):
        first = self.experiment.prepare_sample(sample(0.1, angle_rad=1.0))
        second = self.experiment.prepare_sample(sample(0.2, angle_rad=1.0))

        self.assertTrue(first.angle_rejected)
        self.assertEqual(first.angle_rad, 0.0)
        self.assertFalse(second.angle_rejected)
        self.assertEqual(second.angle_rad, 1.0)

    def test_recovery_unwraps_full_turn_without_hiding_real_small_motion(self):
        self.experiment.seed_angle(5.9, continuous_reference_rad=5.9)

        self.experiment.reseed_after_recovery(-0.2)

        self.assertAlmostEqual(
            self.experiment.continuous_angle_for_raw(-0.2),
            2.0 * 3.141592653589793 - 0.2,
        )
        self.assertNotAlmostEqual(
            self.experiment.continuous_angle_for_raw(-0.2),
            5.9,
        )


if __name__ == "__main__":
    unittest.main()
