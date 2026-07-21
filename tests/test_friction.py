import unittest

from foctwin.domain import TelemetrySample
from foctwin.friction import (
    FRICTION_MONITOR_MASK,
    FrictionExperiment,
    FrictionPhase,
    FrictionPointResult,
    FrictionTestConfig,
    estimate_friction,
    summarize_friction_point,
)


class FrictionAnalysisTests(unittest.TestCase):
    def test_four_directional_points_recover_coulomb_and_viscous_terms(self):
        points = []
        for target in (0.02, -0.02, 0.05, -0.05):
            coulomb = 0.12 if target > 0 else 0.14
            torque = coulomb + 0.8 * abs(target)
            points.append(
                FrictionPointResult(
                    target_velocity_rad_s=target,
                    mean_velocity_rad_s=target,
                    velocity_std_rad_s=0.0001,
                    mean_current_q_a=torque,
                    current_std_a=0.001,
                    friction_torque_nm=torque,
                    breakaway_torque_nm=0.2 if target > 0 else 0.22,
                    sample_count=100,
                    valid=True,
                    note="ОК",
                )
            )

        result = estimate_friction(points)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.coulomb_positive_nm, 0.12)
        self.assertAlmostEqual(result.coulomb_negative_nm, 0.14)
        self.assertAlmostEqual(result.coulomb_friction_nm, 0.13)
        self.assertAlmostEqual(result.viscous_friction_nm_s_rad, 0.8)
        self.assertAlmostEqual(result.breakaway_friction_nm, 0.21)
        self.assertAlmostEqual(result.r_squared, 1.0)

    def test_point_is_rejected_when_small_target_does_not_move_motor(self):
        samples = [
            TelemetrySample(timestamp_s=index * 0.02, velocity_rad_s=0.0, current_q_a=0.05)
            for index in range(50)
        ]

        result = summarize_friction_point(0.02, samples, samples, 1.3264)

        self.assertFalse(result.valid)
        self.assertIn("направление", result.note)

    def test_default_experiment_starts_narrow_but_accepts_user_increases(self):
        config = FrictionTestConfig()
        config.validate()
        self.assertEqual(config.targets, (0.02, -0.02, 0.05, -0.05))
        self.assertEqual(config.to_dict()["monitor_mask"], FRICTION_MONITOR_MASK)

        config.current_limit_a = 0.15
        config.voltage_limit_v = 24.0
        config.velocity_limit_rad_s = 0.5
        config.angle_min_rad = -4.0
        config.angle_max_rad = 4.0
        config.high_velocity_rad_s = 0.1
        config.validate()

        config.current_limit_a = 10.001
        with self.assertRaises(ValueError):
            config.validate()


class FrictionExperimentTests(unittest.TestCase):
    def config(self) -> FrictionTestConfig:
        return FrictionTestConfig(settle_s=1.0, measure_s=2.0, pause_s=0.5)

    def test_state_machine_runs_a_point_and_checkpoints_result(self):
        experiment = FrictionExperiment(self.config(), 1.0)
        self.assertEqual(experiment.start(0.0)[0].value, 0.0)
        actions = experiment.tick(0.5)
        self.assertEqual((experiment.phase, actions[0].value), (FrictionPhase.SETTLING, 0.02))

        experiment.tick(1.5)
        self.assertEqual(experiment.phase, FrictionPhase.MEASURING)
        for index in range(20):
            experiment.add_sample(
                TelemetrySample(
                    timestamp_s=1.5 + index * 0.05,
                    velocity_rad_s=0.02,
                    current_q_a=0.04,
                    angle_rad=0.1,
                )
            )
        actions = experiment.tick(3.5)

        self.assertEqual(experiment.phase, FrictionPhase.PAUSE)
        self.assertEqual([action.kind for action in actions], ["target", "checkpoint"])
        self.assertEqual(len(experiment.points), 1)
        self.assertTrue(experiment.points[0].valid)

    def test_telemetry_recovery_repeats_current_point(self):
        experiment = FrictionExperiment(self.config(), 1.0)
        experiment.start(0.0)
        experiment.tick(0.5)

        actions = experiment.enter_recovery()
        self.assertEqual(experiment.phase, FrictionPhase.RECOVERING)
        self.assertEqual(actions[0].kind, "safe_stop")

        actions = experiment.resume_after_recovery(10.0)
        self.assertEqual(experiment.phase, FrictionPhase.SETTLING)
        self.assertEqual(actions[0].value, 0.02)

    def test_sample_outside_test_bounds_requests_abort(self):
        experiment = FrictionExperiment(self.config(), 1.0)
        experiment.start(0.0)
        experiment.tick(0.5)

        violation = experiment.add_sample(
            TelemetrySample(timestamp_s=1.0, current_q_a=0.06, angle_rad=0.0)
        )

        self.assertIn("Iq", violation)
