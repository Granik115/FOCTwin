import unittest

from foctwin.domain import TelemetrySample
from foctwin.friction import (
    FRICTION_MONITOR_MASK,
    ActuatorPulseResult,
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

    def test_angle_slope_accepts_motion_despite_noisy_firmware_velocity(self):
        samples = [
            TelemetrySample(
                timestamp_s=index * 0.02,
                angle_rad=0.02 * index * 0.02,
                velocity_rad_s=0.02 + (0.02 if index % 2 else -0.02),
                current_q_a=0.01,
            )
            for index in range(200)
        ]

        result = summarize_friction_point(0.02, samples, samples, 1.3264)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.mean_velocity_rad_s, 0.02, places=5)
        self.assertLess(result.velocity_std_rad_s, 0.001)

    def test_default_experiment_starts_narrow_but_accepts_user_increases(self):
        config = FrictionTestConfig()
        config.validate()
        self.assertEqual(config.targets, (0.02, -0.02, 0.05, -0.05))
        self.assertEqual(config.to_dict()["monitor_mask"], FRICTION_MONITOR_MASK)
        self.assertEqual(config.to_dict()["torque_mode"], "voltage")
        self.assertEqual(config.to_dict()["velocity_estimator"]["source"], "angle_slope")

        config.current_trip_limit_a = 1.5
        config.voltage_limit_v = 24.0
        config.velocity_limit_rad_s = 0.5
        config.angle_min_rad = -4.0
        config.angle_max_rad = 4.0
        config.high_velocity_rad_s = 0.1
        config.validate()

        config.current_trip_limit_a = 10.001
        with self.assertRaises(ValueError):
            config.validate()


class FrictionExperimentTests(unittest.TestCase):
    def config(self) -> FrictionTestConfig:
        return FrictionTestConfig(
            pulse_start_voltage_v=0.1,
            pulse_step_voltage_v=0.1,
            pulse_max_voltage_v=0.2,
            pulse_duration_s=0.3,
            actuator_pause_s=0.2,
            baseline_s=0.5,
            movement_threshold_rad=0.002,
            current_trip_limit_a=1.0,
            settle_s=1.0,
            measure_s=2.0,
            pause_s=0.5,
        )

    @staticmethod
    def confirmed_attempts() -> list[ActuatorPulseResult]:
        return [
            ActuatorPulseResult(
                direction=direction,
                command_voltage_v=direction * 0.2,
                mean_voltage_q_v=direction * 0.2,
                mean_measured_current_q_a=direction * 0.25,
                mean_abs_measured_current_a=0.25,
                peak_measured_current_q_a=0.3,
                angle_delta_rad=direction * 0.003,
                movement_detected=True,
                current_detected=True,
                sample_count=10,
                note="страгивание",
            )
            for direction in (1, -1)
        ]

    def experiment_after_preflight(self) -> FrictionExperiment:
        return FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
        )

    @staticmethod
    def add_stationary_samples(
        experiment: FrictionExperiment,
        start_s: float,
        angle_rad: float,
        current_q_a: float = 0.0,
    ) -> None:
        for index in range(10):
            timestamp = start_s + index * 0.05
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=timestamp,
                    voltage_q_v=0.0,
                    voltage_d_v=0.0,
                    current_q_a=current_q_a,
                    current_d_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=angle_rad,
                )
            )
            if violation:
                raise AssertionError(violation)

    def test_actuator_preflight_zeros_immediately_and_requires_both_directions(self):
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
        )
        self.assertEqual(experiment.start(0.0)[0].value, 0.0)
        self.add_stationary_samples(experiment, 0.0, 0.0)

        actions = experiment.tick(0.5)
        self.assertEqual((experiment.phase, actions[0].value), (FrictionPhase.ACTUATOR_PULSE, 0.1))

        movement_actions = []
        for index, angle in enumerate((0.0, 0.0005, 0.001, 0.0025)):
            violation, movement_actions = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=0.55 + index * 0.05,
                    voltage_q_v=0.1,
                    voltage_d_v=0.0,
                    current_q_a=0.2,
                    current_d_a=0.0,
                    velocity_rad_s=43.0,
                    angle_rad=angle,
                )
            )
            self.assertIsNone(violation)
        self.assertEqual([action.kind for action in movement_actions], ["target", "checkpoint"])
        self.assertEqual(movement_actions[0].value, 0.0)
        self.assertTrue(experiment.actuator_attempts[0].current_detected)

        self.add_stationary_samples(experiment, 0.75, 0.0025)
        actions = experiment.tick(1.4)
        self.assertEqual(actions[0].value, -0.1)

        for index, angle in enumerate((0.0025, 0.002, 0.0015, 0.0)):
            violation, movement_actions = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=1.45 + index * 0.05,
                    voltage_q_v=-0.1,
                    voltage_d_v=0.0,
                    current_q_a=-0.2,
                    current_d_a=0.0,
                    velocity_rad_s=-42.0,
                    angle_rad=angle,
                )
            )
            self.assertIsNone(violation)
        self.assertEqual(movement_actions[0].value, 0.0)
        self.add_stationary_samples(experiment, 1.65, 0.0)

        actions = experiment.tick(2.3)

        self.assertTrue(experiment.actuator_complete)
        self.assertEqual(experiment.phase, FrictionPhase.CONFIGURING_VELOCITY)
        self.assertEqual([action.kind for action in actions], ["configure_velocity", "checkpoint"])
        self.assertAlmostEqual(experiment.working_current_limit_a, 0.1 / 0.675 * 1.2)

    def test_actuator_preflight_refuses_velocity_stage_without_measured_iq(self):
        attempts = self.confirmed_attempts()
        for attempt in attempts:
            attempt.current_detected = False
            attempt.mean_measured_current_q_a = 0.001
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=attempts,
        )

        self.assertFalse(experiment.actuator_complete)
        self.assertEqual(experiment.configuration_mode, "actuator")

    def test_state_machine_runs_a_point_and_checkpoints_result(self):
        experiment = self.experiment_after_preflight()
        self.assertEqual(experiment.start(0.0)[0].value, 0.0)
        actions = experiment.tick(0.5)
        self.assertEqual((experiment.phase, actions[0].value), (FrictionPhase.SETTLING, 0.02))

        experiment.tick(1.5)
        self.assertEqual(experiment.phase, FrictionPhase.MEASURING)
        for index in range(20):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=1.5 + index * 0.05,
                    velocity_rad_s=0.02,
                    current_q_a=0.04,
                    voltage_q_v=0.1,
                    voltage_d_v=0.0,
                    current_d_a=0.0,
                    angle_rad=0.1 + index * 0.001,
                )
            )
            self.assertIsNone(violation)
        actions = experiment.tick(3.5)

        self.assertEqual(experiment.phase, FrictionPhase.PAUSE)
        self.assertEqual([action.kind for action in actions], ["target", "checkpoint"])
        self.assertEqual(len(experiment.points), 1)
        self.assertTrue(experiment.points[0].valid)

    def test_telemetry_recovery_repeats_current_point(self):
        experiment = self.experiment_after_preflight()
        experiment.start(0.0)
        experiment.tick(0.5)

        actions = experiment.enter_recovery()
        self.assertEqual(experiment.phase, FrictionPhase.RECOVERING)
        self.assertEqual(actions[0].kind, "safe_stop")

        actions = experiment.resume_after_recovery(10.0)
        self.assertEqual(experiment.phase, FrictionPhase.ZERO)
        self.assertEqual(actions[0].value, 0.0)
        actions = experiment.tick(10.5)
        self.assertEqual(experiment.phase, FrictionPhase.SETTLING)
        self.assertEqual(actions[0].value, 0.02)

    def test_soft_limit_requires_three_consecutive_samples(self):
        config = self.config()
        config.current_trip_limit_a = 0.05
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
        )
        experiment.start(0.0)
        experiment.tick(0.5)

        first = experiment.add_sample(
            TelemetrySample(timestamp_s=1.0, current_q_a=0.06, angle_rad=0.0)
        )[0]
        second = experiment.add_sample(
            TelemetrySample(timestamp_s=1.02, current_q_a=0.06, angle_rad=0.0)
        )[0]
        violation = experiment.add_sample(
            TelemetrySample(timestamp_s=1.04, current_q_a=0.06, angle_rad=0.0)
        )[0]

        self.assertIsNone(first)
        self.assertIsNone(second)
        self.assertIn("Iq", violation)
        self.assertIn("3 отсчёта", violation)

    def test_firmware_velocity_spike_is_ignored_but_angle_slope_is_limited(self):
        experiment = self.experiment_after_preflight()
        experiment.start(0.0)
        experiment.tick(0.5)

        violation, _ = experiment.add_sample(
            TelemetrySample(timestamp_s=0.6, velocity_rad_s=43.0, angle_rad=0.0)
        )
        self.assertIsNone(violation)

        violation = None
        for index in range(12):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=0.7 + index * 0.05,
                    velocity_rad_s=0.0,
                    current_q_a=0.1,
                    current_d_a=0.0,
                    voltage_q_v=0.1,
                    voltage_d_v=0.0,
                    angle_rad=index * 0.05,
                )
            )
            if violation:
                break

        self.assertIn("скорость по углу", violation)
