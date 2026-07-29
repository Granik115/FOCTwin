import math
import unittest
from dataclasses import replace

from foctwin.domain import TelemetrySample
from foctwin.friction import (
    FRICTION_MONITOR_MASK,
    ActuatorPulseResult,
    BaselineDiagnostic,
    FrictionExperiment,
    FrictionPhase,
    FrictionPointResult,
    FrictionTestConfig,
    PositioningResult,
    estimate_friction,
    summarize_friction_point,
    summarize_position_observations,
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
        self.assertTrue(result.motion_valid)
        self.assertIsNotNone(result.rise_time_s)
        self.assertIsNotNone(result.transient_peak_velocity_rad_s)

    def test_point_is_not_accepted_when_measured_current_stays_in_noise(self):
        samples = [
            TelemetrySample(
                timestamp_s=index * 0.02,
                angle_rad=0.02 * index * 0.02,
                velocity_rad_s=0.02,
                current_q_a=0.001,
            )
            for index in range(200)
        ]

        result = summarize_friction_point(
            0.02,
            samples,
            samples,
            1.3264,
            measured_current_floor_a=0.01,
        )

        self.assertFalse(result.valid)
        self.assertFalse(result.measured_current_detected)
        self.assertIn("Iq", result.note)

    def test_sparse_or_wrong_sign_current_does_not_validate_a_speed_point(self):
        samples = [
            TelemetrySample(
                timestamp_s=index * 0.02,
                angle_rad=0.02 * index * 0.02,
                velocity_rad_s=0.02,
                current_q_a=(0.1 if index % 4 == 0 else (-0.1 if index % 7 == 0 else 0.001)),
            )
            for index in range(200)
        ]

        result = summarize_friction_point(
            0.02,
            samples,
            samples,
            1.3264,
            measured_current_floor_a=0.01,
        )

        self.assertFalse(result.valid)
        self.assertFalse(result.measured_current_detected)
        self.assertIn("пригодно для карты Uq", result.note)

    def test_default_experiment_starts_narrow_but_accepts_user_increases(self):
        config = FrictionTestConfig()
        config.validate()
        self.assertEqual(
            config.targets,
            (
                0.02,
                -0.02,
                (0.02 * 0.05) ** 0.5,
                -(0.02 * 0.05) ** 0.5,
                0.05,
                -0.05,
            ),
        )
        self.assertEqual(config.to_dict()["monitor_mask"], FRICTION_MONITOR_MASK)
        self.assertEqual(config.to_dict()["torque_mode"], "voltage")
        self.assertEqual(config.to_dict()["velocity_estimator"]["source"], "angle_slope")
        self.assertEqual(config.to_dict()["algorithm_schema"], 8)
        self.assertEqual(config.movement_threshold_rad, 0.001)
        self.assertEqual(config.max_recovery_attempts, 50)
        self.assertEqual(config.position_bin_width_rad, 0.1)
        self.assertEqual(config.automatic_position_count, 1)
        self.assertEqual(config.map_passes, 2)
        self.assertEqual(config.automatic_position_targets(0.25), (0.25, 0.25))

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

    def test_old_defaults_migrate_to_new_motion_and_recovery_limits(self):
        config = FrictionTestConfig.from_dict(
            {
                "algorithm_schema": 3,
                "movement_threshold_rad": 0.002,
                "max_recovery_attempts": 3,
            }
        )

        self.assertEqual(config.movement_threshold_rad, 0.001)
        self.assertEqual(config.max_recovery_attempts, 50)

    def test_automatic_positions_use_signed_step_and_preserve_sweep_margin(self):
        config = FrictionTestConfig(
            angle_min_rad=-5.0,
            angle_max_rad=5.0,
            automatic_position_count=3,
            automatic_position_step_rad=-1.0,
            map_passes=2,
        )

        self.assertEqual(
            config.automatic_position_targets(1.0),
            (1.0, 0.0, -1.0, 0.0, 1.0),
        )

        config.automatic_position_step_rad = 3.0
        with self.assertRaisesRegex(ValueError, "безопасного диапазона"):
            config.automatic_position_targets(1.0)

    def test_position_map_keeps_measured_and_voltage_torque_separate(self):
        samples = [
            TelemetrySample(
                timestamp_s=index * 0.02,
                angle_rad=0.02 * index * 0.02,
                velocity_rad_s=43.0,
                voltage_q_v=1.2,
                current_q_a=0.001,
            )
            for index in range(200)
        ]

        observations = summarize_position_observations(
            0.02,
            samples,
            1.3264,
            0.675,
            92.6,
            bin_width_rad=0.02,
            measured_current_floor_a=0.01,
        )

        self.assertGreaterEqual(len(observations), 3)
        self.assertTrue(all(observation.motion_valid for observation in observations))
        self.assertTrue(
            all(observation.measured_torque_nm is None for observation in observations)
        )
        self.assertTrue(
            all(observation.voltage_equivalent_torque_nm > 0 for observation in observations)
        )
        self.assertTrue(all("диагностическая" in observation.note for observation in observations))


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
            map_passes=1,
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
        for index, angle in enumerate((0.0, 0.0005, 0.001, 0.0025, 0.003)):
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

        for index, angle in enumerate((0.0025, 0.002, 0.0015, 0.0, -0.0005)):
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
        self.add_stationary_samples(experiment, 1.75, -0.0005)

        actions = experiment.tick(2.4)

        self.assertTrue(experiment.actuator_complete)
        self.assertEqual(experiment.phase, FrictionPhase.CONFIGURING_VELOCITY)
        self.assertEqual([action.kind for action in actions], ["configure_velocity", "checkpoint"])
        self.assertAlmostEqual(experiment.working_current_limit_a, 0.1 / 0.675 * 1.2)

    def test_actuator_preflight_continues_but_flags_missing_measured_iq(self):
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

        self.assertTrue(experiment.actuator_complete)
        self.assertFalse(experiment.measured_current_complete)
        self.assertEqual(experiment.configuration_mode, "velocity")
        self.assertFalse(experiment.estimate().valid)
        self.assertIn("Мотор и карта по Uq пригодны", experiment.estimate().note)
        self.assertIn("0 рад/+", experiment.estimate().note)

    def test_command_limit_is_independent_from_measured_current_trip(self):
        config = self.config()
        config.current_trip_limit_a = 0.5
        attempts = self.confirmed_attempts()
        for attempt in attempts:
            attempt.command_voltage_v = 2.0 * attempt.direction
            attempt.current_detected = attempt.direction < 0
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=attempts,
        )

        self.assertTrue(experiment.actuator_complete)
        self.assertGreater(experiment.working_current_limit_a, config.current_trip_limit_a)

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
        self.assertGreaterEqual(len(experiment.position_observations), 1)
        checkpoint = experiment.checkpoint_payload(7)
        self.assertEqual(checkpoint["schema"], 8)
        self.assertEqual(checkpoint["experiment_id"], 7)
        self.assertGreaterEqual(len(checkpoint["position_observations"]), 1)

    def test_board_reset_angle_shift_by_full_turn_is_unwrapped(self):
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
            position_targets_rad=(5.358,),
        )
        experiment.seed_angle(5.358)

        before = experiment.prepare_sample(
            TelemetrySample(timestamp_s=1.0, angle_rad=5.3578)
        )
        after_reset = experiment.prepare_sample(
            TelemetrySample(timestamp_s=10.0, angle_rad=-0.9269)
        )

        self.assertAlmostEqual(before.angle_rad, 5.3578, places=4)
        self.assertAlmostEqual(after_reset.angle_rad, 5.356285, places=4)
        self.assertAlmostEqual(
            experiment.board_target_for_continuous(6.36),
            0.0768147,
            places=4,
        )

    def test_completed_position_transitions_to_bounded_automatic_move(self):
        config = self.config()
        config.automatic_position_count = 2
        config.automatic_position_step_rad = 1.0
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
            position_targets_rad=(0.0, 1.0),
            point_index=len(config.targets) - 1,
        )
        experiment.phase = FrictionPhase.PAUSE
        experiment.phase_started_s = 0.0

        actions = experiment.tick(config.pause_s)

        self.assertEqual(experiment.position_index, 1)
        self.assertEqual(experiment.phase, FrictionPhase.CONFIGURING_POSITION)
        self.assertEqual(
            [action.kind for action in actions],
            ["configure_position", "checkpoint"],
        )
        self.assertGreater(experiment.positioning_current_limit_a, 0.0)

        actions = experiment.position_configuration_applied(2.0)
        self.assertEqual(experiment.phase, FrictionPhase.POSITIONING)
        self.assertEqual(actions[0].kind, "position_target")
        self.assertEqual(actions[0].value, 1.0)

        experiment.seed_angle(0.0)
        for index in range(201):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=2.0 + index * 0.02,
                    voltage_q_v=0.2,
                    voltage_d_v=0.0,
                    current_q_a=0.02,
                    current_d_a=0.0,
                    velocity_rad_s=0.25,
                    angle_rad=index * 0.005,
                )
            )
            self.assertIsNone(violation)
        self.assertEqual(experiment.tick(6.0), [])
        self.assertEqual(experiment.phase, FrictionPhase.POSITION_SETTLING)
        for index in range(60):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=6.02 + index * 0.02,
                    voltage_q_v=0.0,
                    voltage_d_v=0.0,
                    current_q_a=0.0,
                    current_d_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=1.0,
                )
            )
            self.assertIsNone(violation)
        actions = experiment.tick(7.2)
        self.assertEqual(experiment.phase, FrictionPhase.CONFIGURING_ACTUATOR)
        self.assertEqual(
            [action.kind for action in actions],
            ["configure_actuator", "checkpoint"],
        )
        self.assertFalse(experiment.actuator_complete)

    def test_stalled_saturated_positioning_raises_only_its_own_limit(self):
        config = self.config()
        config.positioning_voltage_step_v = 0.25
        config.positioning_voltage_max_v = 0.7
        config.position_stall_window_s = 3.0
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
            position_targets_rad=(0.0, 1.0),
            position_index=1,
        )
        experiment.phase = FrictionPhase.CONFIGURING_POSITION
        experiment.seed_angle(0.0)

        actions = experiment.position_configuration_applied(0.0)
        self.assertEqual(actions[0].kind, "position_target")
        self.assertAlmostEqual(experiment.positioning_voltage_limit_v, 0.24)
        for index in range(70):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=index * 0.05,
                    voltage_q_v=0.24,
                    current_q_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=0.0,
                )
            )
            self.assertIsNone(violation)

        actions = experiment.tick(3.2)

        self.assertEqual([action.kind for action in actions], ["position_limit", "checkpoint"])
        self.assertAlmostEqual(experiment.positioning_voltage_limit_v, 0.49)
        self.assertAlmostEqual(actions[0].value, 0.49 / 0.675)
        self.assertEqual(experiment.phase, FrictionPhase.POSITIONING)

    def test_positioning_stall_without_saturation_reports_controller_path(self):
        config = self.config()
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
            position_targets_rad=(0.0, 1.0),
            position_index=1,
        )
        experiment.phase = FrictionPhase.CONFIGURING_POSITION
        experiment.seed_angle(0.0)
        experiment.position_configuration_applied(0.0)
        for index in range(70):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=index * 0.05,
                    voltage_q_v=0.05,
                    current_q_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=0.0,
                )
            )
            self.assertIsNone(violation)

        actions = experiment.tick(3.2)
        report = experiment.diagnostic_report()

        self.assertEqual(actions[0].kind, "safe_stop")
        self.assertEqual(experiment.phase, FrictionPhase.ABORTED)
        self.assertIn("без насыщения", experiment.abort_reason)
        self.assertFalse(experiment.positioning_results[0].reached)
        self.assertTrue(
            any(
                item["code"] == "positioning_controller_path"
                for item in report["findings"]
            )
        )

    def test_positioning_at_maximum_uq_reports_local_torque_limit(self):
        config = self.config()
        config.positioning_voltage_step_v = 0.04
        config.positioning_voltage_max_v = 0.24
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
            position_targets_rad=(0.0, 1.0),
            position_index=1,
        )
        experiment.phase = FrictionPhase.CONFIGURING_POSITION
        experiment.seed_angle(0.0)
        experiment.position_configuration_applied(0.0)
        for index in range(70):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=index * 0.05,
                    voltage_q_v=0.24,
                    current_q_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=0.0,
                )
            )
            self.assertIsNone(violation)

        actions = experiment.tick(3.2)
        report = experiment.diagnostic_report()

        self.assertEqual(actions[0].kind, "safe_stop")
        self.assertIn("максимальном Uq", experiment.abort_reason)
        self.assertTrue(experiment.positioning_results[0].saturated_at_end)
        self.assertTrue(
            any(
                item["code"] == "positioning_torque_limit"
                for item in report["findings"]
            )
        )

    def test_baseline_is_saved_as_a_separate_sensor_diagnostic(self):
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
        )
        experiment.start(0.0)
        self.add_stationary_samples(experiment, 0.0, 0.0, current_q_a=0.002)

        experiment.tick(0.5)

        self.assertEqual(len(experiment.baseline_diagnostics), 1)
        diagnostic = experiment.baseline_diagnostics[0]
        self.assertEqual(diagnostic.position_index, 0)
        self.assertAlmostEqual(diagnostic.mean_current_q_a, 0.002)
        self.assertEqual(diagnostic.note, "ОК")

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

    def test_pulse_without_measurements_enters_recovery_and_is_retried(self):
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
        )
        experiment.start(0.0)
        self.add_stationary_samples(experiment, 0.0, 0.0)
        actions = experiment.tick(0.5)
        self.assertEqual(actions[0].value, 0.1)

        actions = experiment.tick(1.0)

        self.assertEqual(experiment.phase, FrictionPhase.RECOVERING)
        self.assertEqual([action.kind for action in actions], ["safe_stop"])
        self.assertEqual(experiment.actuator_attempts, [])
        self.assertEqual(experiment.interruption_count, 1)

        actions = experiment.resume_after_recovery(10.0)
        self.assertEqual(experiment.phase, FrictionPhase.ACTUATOR_BASELINE)
        self.assertEqual(actions[0].value, 0.0)
        self.add_stationary_samples(experiment, 10.0, 0.0)
        actions = experiment.tick(10.5)
        self.assertEqual(actions[0].value, 0.1)

    def test_checkpoint_counters_are_preserved_across_process_restart(self):
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
            interruption_count=3,
            rejected_angle_samples=4,
        )

        payload = experiment.checkpoint_payload(55)

        self.assertEqual(payload["interruption_count"], 3)
        self.assertEqual(payload["rejected_angle_samples"], 4)

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

    def test_single_angle_dropout_does_not_fake_reverse_movement(self):
        experiment = FrictionExperiment(
            self.config(),
            1.0,
            phase_resistance_ohm=0.675,
        )
        experiment.start(0.0)
        self.add_stationary_samples(experiment, 0.0, 0.015)
        experiment.tick(0.5)

        violation, actions = experiment.add_sample(
            TelemetrySample(
                timestamp_s=0.55,
                voltage_q_v=0.1,
                current_q_a=0.02,
                angle_rad=0.0,
            )
        )
        self.assertIsNone(violation)
        self.assertEqual(actions, [])
        violation, actions = experiment.add_sample(
            TelemetrySample(
                timestamp_s=0.60,
                voltage_q_v=0.1,
                current_q_a=0.02,
                angle_rad=0.015,
            )
        )

        self.assertIsNone(violation)
        self.assertEqual(actions, [])
        self.assertEqual(experiment.rejected_angle_samples, 1)
        self.assertEqual(experiment.actuator_attempts, [])

    def test_small_sustained_encoder_motion_is_detected(self):
        config = self.config()
        config.movement_threshold_rad = 0.001
        experiment = FrictionExperiment(config, 1.0, phase_resistance_ohm=0.675)
        experiment.start(0.0)
        self.add_stationary_samples(experiment, 0.0, 0.0002)
        experiment.tick(0.5)

        actions = []
        for index in range(3):
            violation, actions = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=0.55 + index * 0.05,
                    voltage_q_v=0.1,
                    current_q_a=0.001,
                    angle_rad=0.0011,
                )
            )
            self.assertIsNone(violation)
            if actions:
                break

        self.assertEqual([action.kind for action in actions], ["target", "checkpoint"])
        self.assertTrue(experiment.actuator_attempts[0].movement_detected)
        self.assertFalse(experiment.actuator_attempts[0].current_detected)


class EvidenceProtocolTests(unittest.TestCase):
    @staticmethod
    def config(**overrides) -> FrictionTestConfig:
        values = {
            "evidence_mode": True,
            "pwm_off_observation_s": 5.0,
            "baseline_s": 0.5,
            "pulse_start_voltage_v": 0.1,
            "pulse_step_voltage_v": 0.1,
            "pulse_max_voltage_v": 0.2,
            "pulse_duration_s": 0.3,
            "actuator_pause_s": 0.2,
            "breakaway_verify_s": 0.2,
            "breakaway_repeats": 1,
            "movement_threshold_rad": 0.001,
            "residual_movement_threshold_rad": 0.005,
            "low_velocity_rad_s": 0.02,
            "high_velocity_rad_s": 0.05,
            "velocity_travel_rad": 0.02,
            "fixed_velocity_voltage_limit_v": 0.5,
            "positioning_voltage_max_v": 0.5,
            "map_passes": 1,
            "position_validation_enabled": False,
        }
        values.update(overrides)
        return FrictionTestConfig(**values)

    @staticmethod
    def baselines(position: float = 1.0) -> list[BaselineDiagnostic]:
        return [
            BaselineDiagnostic(
                position_index=0,
                measurement_position_rad=position,
                sample_count=100,
                duration_s=5.0 if not pwm_enabled else 0.5,
                mean_angle_rad=position,
                angle_std_rad=0.0,
                angle_drift_rad_s=0.0,
                mean_current_q_a=0.0,
                current_std_a=0.0,
                mean_voltage_q_v=0.0,
                note="ОК",
                pwm_enabled=pwm_enabled,
                raw_angle_sample_count=100,
            )
            for pwm_enabled in (False, True)
        ]

    @staticmethod
    def confirmed_attempts(
        *,
        positions: tuple[float, ...] = (1.0,),
        repeats: int = 1,
    ) -> list[ActuatorPulseResult]:
        attempts = []
        for position_index, position in enumerate(positions):
            for repeat_index in range(repeats):
                for direction in (1, -1):
                    attempts.append(
                        ActuatorPulseResult(
                            direction=direction,
                            command_voltage_v=direction * 0.2,
                            mean_voltage_q_v=direction * 0.2,
                            mean_measured_current_q_a=direction * 0.2,
                            mean_abs_measured_current_a=0.2,
                            peak_measured_current_q_a=0.25,
                            angle_delta_rad=direction * 0.006,
                            movement_detected=True,
                            current_detected=True,
                            sample_count=10,
                            note="подтверждено",
                            position_index=position_index,
                            measurement_position_rad=position,
                            repeat_index=repeat_index,
                            residual_angle_delta_rad=direction * 0.006,
                            confirmed_breakaway=True,
                        )
                    )
        return attempts

    @staticmethod
    def add_stationary(
        experiment: FrictionExperiment,
        start_s: float,
        duration_s: float,
        angle: float,
    ) -> None:
        count = max(10, int(duration_s / 0.05) + 1)
        for index in range(count):
            experiment.add_sample(
                TelemetrySample(
                    timestamp_s=start_s + index * 0.05,
                    voltage_q_v=0.0,
                    current_q_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=angle,
                    raw_angle_rad=angle,
                )
            )

    def test_evidence_config_exposes_electrical_period_and_fixed_trials(self):
        config = self.config(
            breakaway_repeats=3,
            position_validation_enabled=True,
        )
        config.validate()

        self.assertAlmostEqual(config.electrical_period_rad, math.tau / 15)
        self.assertAlmostEqual(config.recommended_electrical_step_rad, math.tau / 120)
        self.assertEqual(len(config.position_validation_cycle_offsets), 12)
        self.assertEqual(len(config.position_validation_offsets), 36)
        self.assertEqual(config.to_dict()["algorithm_schema"], 8)
        self.assertEqual(
            config.to_dict()["actuator_preflight"]["breakaway_confirmation"],
            "residual_displacement_after_zero",
        )

    def test_evidence_preflight_expands_only_to_bounded_recommended_ceiling(self):
        config = self.config(
            pulse_max_voltage_v=0.5,
            positioning_voltage_max_v=3.0,
            fixed_velocity_voltage_limit_v=3.0,
            voltage_limit_v=12.0,
        )

        expanded = config.with_recommended_evidence_pulse_ceiling()

        self.assertEqual(expanded.pulse_max_voltage_v, 3.0)
        self.assertEqual(expanded.positioning_voltage_max_v, 3.0)
        self.assertEqual(expanded.fixed_velocity_voltage_limit_v, 3.0)
        self.assertEqual(expanded.voltage_limit_v, 12.0)
        expanded.validate()

        deliberate_higher = replace(
            expanded,
            pulse_max_voltage_v=10.0,
            positioning_voltage_max_v=10.0,
            fixed_velocity_voltage_limit_v=10.0,
        )
        self.assertEqual(
            deliberate_higher.with_recommended_evidence_pulse_ceiling(),
            deliberate_higher,
        )

    def test_pwm_off_observer_ignores_inactive_voltage_telemetry(self):
        config = self.config(
            pulse_max_voltage_v=0.5,
            fixed_velocity_voltage_limit_v=0.5,
            positioning_voltage_max_v=0.5,
        )
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            position_targets_rad=(1.0,),
        )
        experiment.seed_angle(1.0)
        experiment.start(0.0)

        for index in range(5):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=index * 0.05,
                    voltage_q_v=-0.627,
                    voltage_d_v=0.75,
                    current_q_a=0.0,
                    current_d_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=1.0,
                    raw_angle_rad=1.0,
                )
            )
            self.assertIsNone(violation)

        experiment.tick(config.pwm_off_observation_s + 0.1)
        experiment.actuator_configuration_applied(config.pwm_off_observation_s + 0.2)
        violation = None
        for index in range(3):
            violation, _ = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=config.pwm_off_observation_s + 0.3 + index * 0.05,
                    voltage_q_v=-0.627,
                    voltage_d_v=0.0,
                    current_q_a=0.0,
                    current_d_a=0.0,
                    velocity_rad_s=0.0,
                    angle_rad=1.0,
                    raw_angle_rad=1.0,
                )
            )
            if violation:
                break

        self.assertIn("Uq", violation)

    def test_checkpoint_after_pwm_off_resumes_in_actuator_mode(self):
        config = self.config()
        pwm_off_baseline = self.baselines()[0]
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            baseline_diagnostics=(pwm_off_baseline,),
            position_targets_rad=(1.0,),
        )
        experiment.seed_angle(1.0)

        self.assertEqual(experiment.configuration_mode, "actuator")
        actions = experiment.start(0.0)
        self.assertEqual(experiment.phase, FrictionPhase.ACTUATOR_BASELINE)
        self.assertEqual(actions[0].value, 0.0)

    def test_repeated_fixed_steps_measure_controller_objective_noise(self):
        config = self.config()
        results = []
        for repeat_index, duration in enumerate((1.0, 1.1, 2.0)):
            results.append(
                PositioningResult(
                    position_index=100 + repeat_index,
                    start_position_rad=1.0,
                    target_position_rad=1.1,
                    final_position_rad=1.1,
                    final_error_rad=0.0,
                    duration_s=duration,
                    reached=True,
                    initial_voltage_limit_v=0.5,
                    final_voltage_limit_v=0.5,
                    voltage_boost_count=0,
                    maximum_measured_voltage_v=0.3,
                    hold_voltage_q_v=0.1,
                    hold_current_q_a=0.1,
                    overshoot_rad=0.0,
                    approach_direction=1,
                    saturated_at_end=False,
                    note="ОК",
                    purpose="controller_validation",
                    validation_repeat_index=repeat_index,
                    validation_offset_rad=0.1,
                )
            )
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            positioning_results=results,
            position_targets_rad=(1.0,),
        )

        report = experiment.diagnostic_report()
        codes = {item["code"] for item in report["findings"]}

        self.assertIn("repeat_controller_objective_noise", codes)
        self.assertGreater(
            report["tests"]["positioning"]["controller_objective_cv"],
            0.2,
        )

    def test_pwm_off_and_on_dropout_comparison_identifies_non_pwm_fault(self):
        config = self.config()
        baselines = self.baselines()
        baselines[0].isolated_zero_dropouts = 3
        baselines[0].zero_dropout_rate_hz = 0.6
        baselines[1].duration_s = 5.0
        baselines[1].isolated_zero_dropouts = 1
        baselines[1].zero_dropout_rate_hz = 0.2
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            baseline_diagnostics=baselines,
            position_targets_rad=(1.0,),
        )

        codes = {item["code"] for item in experiment.diagnostic_report()["findings"]}

        self.assertIn("encoder_dropout_independent_of_pwm", codes)

    def test_transient_shift_is_not_breakaway_when_it_returns_after_zero(self):
        config = self.config()
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            position_targets_rad=(1.0,),
        )
        experiment.seed_angle(1.0)
        experiment.start(0.0)
        self.add_stationary(experiment, 0.0, 5.0, 1.0)
        actions = experiment.tick(5.1)
        self.assertEqual(actions[0].kind, "configure_actuator")
        experiment.actuator_configuration_applied(5.2)
        self.add_stationary(experiment, 5.2, 0.5, 1.0)
        experiment.tick(5.8)

        actions = []
        for index, angle in enumerate((1.0, 1.0012, 1.0025)):
            _, actions = experiment.add_sample(
                TelemetrySample(
                    timestamp_s=5.85 + index * 0.05,
                    voltage_q_v=0.1,
                    current_q_a=0.1,
                    angle_rad=angle,
                    raw_angle_rad=angle,
                )
            )
        self.assertEqual(actions[0].value, 0.0)
        self.add_stationary(experiment, 6.0, 0.2, 1.0002)
        experiment.tick(6.3)

        self.assertFalse(experiment.actuator_attempts[0].confirmed_breakaway)
        self.assertIn("упругий", experiment.actuator_attempts[0].note)

    def test_fixed_velocity_alc_does_not_change_with_local_breakaway(self):
        config = self.config(breakaway_repeats=3, fixed_velocity_voltage_limit_v=0.5)
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(repeats=3),
            baseline_diagnostics=self.baselines(),
            position_targets_rad=(1.0,),
        )

        self.assertAlmostEqual(experiment.working_current_limit_a, 0.5 / 0.675)

    def test_velocity_point_ends_by_distance_before_legacy_measure_time(self):
        config = self.config()
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
            baseline_diagnostics=self.baselines(),
            position_targets_rad=(1.0,),
        )
        experiment.seed_angle(1.0)
        experiment.start(0.0)
        experiment.tick(config.pause_s)
        for index in range(30):
            angle = 1.0 + index * 0.001
            experiment.add_sample(
                TelemetrySample(
                    timestamp_s=0.55 + index * 0.05,
                    voltage_q_v=0.2,
                    current_q_a=0.1,
                    velocity_rad_s=0.02,
                    angle_rad=angle,
                    raw_angle_rad=angle,
                )
            )
        actions = experiment.tick(2.0)

        self.assertEqual(experiment.phase, FrictionPhase.PAUSE)
        self.assertEqual(actions[0].value, 0.0)
        self.assertEqual(len(experiment.points), 1)

    def test_electrical_periodicity_is_confirmed_from_two_periods(self):
        config = self.config(breakaway_repeats=1)
        step = config.recommended_electrical_step_rad
        positions = tuple(index * step for index in range(17))
        attempts: list[ActuatorPulseResult] = []
        for position_index, position in enumerate(positions):
            threshold = 1.0 + 0.3 * math.sin(config.pole_pairs * position)
            for direction in (1, -1):
                attempts.append(
                    ActuatorPulseResult(
                        direction=direction,
                        command_voltage_v=direction * threshold,
                        mean_voltage_q_v=direction * threshold,
                        mean_measured_current_q_a=direction * 0.2,
                        mean_abs_measured_current_a=0.2,
                        peak_measured_current_q_a=0.2,
                        angle_delta_rad=direction * 0.006,
                        movement_detected=True,
                        current_detected=True,
                        sample_count=10,
                        note="подтверждено",
                        position_index=position_index,
                        measurement_position_rad=position,
                        confirmed_breakaway=True,
                    )
                )
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=attempts,
            position_targets_rad=positions,
        )

        evidence = experiment.diagnostic_report()["tests"]["electrical_periodicity"]

        self.assertTrue(evidence["sufficient"])
        self.assertTrue(evidence["confirmed"])
        self.assertGreater(evidence["sinusoidal_r_squared"], 0.9)

    def test_same_coordinate_after_opposite_approaches_reports_hysteresis(self):
        config = self.config()
        positions = (0.0, 1.0, 0.0, -1.0, 0.0)
        attempts: list[ActuatorPulseResult] = []
        for position_index, position in enumerate(positions):
            approach = 0
            if position_index:
                approach = 1 if position > positions[position_index - 1] else -1
            threshold = 1.5 if position_index == 4 else 1.0
            for direction in (1, -1):
                attempts.append(
                    ActuatorPulseResult(
                        direction=direction,
                        command_voltage_v=direction * threshold,
                        mean_voltage_q_v=direction * threshold,
                        mean_measured_current_q_a=direction * 0.2,
                        mean_abs_measured_current_a=0.2,
                        peak_measured_current_q_a=0.2,
                        angle_delta_rad=direction * 0.006,
                        movement_detected=True,
                        current_detected=True,
                        sample_count=10,
                        note="подтверждено",
                        position_index=position_index,
                        measurement_position_rad=position,
                        confirmed_breakaway=True,
                        approach_direction=approach,
                    )
                )
        experiment = FrictionExperiment(
            config,
            1.0,
            phase_resistance_ohm=0.675,
            actuator_attempts=attempts,
            position_targets_rad=positions,
        )

        codes = {item["code"] for item in experiment.diagnostic_report()["findings"]}

        self.assertIn("approach_direction_hysteresis", codes)

    def test_completed_map_enters_fixed_position_validation_sequence(self):
        config = self.config(position_validation_enabled=True)
        completed = [
            FrictionPointResult(
                target_velocity_rad_s=target,
                mean_velocity_rad_s=target,
                velocity_std_rad_s=0.0,
                mean_current_q_a=0.1,
                current_std_a=0.0,
                friction_torque_nm=0.1,
                breakaway_torque_nm=0.2,
                sample_count=20,
                valid=True,
                note="ОК",
                position_index=0,
            )
            for target in config.targets
        ]
        experiment = FrictionExperiment(
            config,
            1.0,
            completed,
            phase_resistance_ohm=0.675,
            actuator_attempts=self.confirmed_attempts(),
            baseline_diagnostics=self.baselines(),
            position_targets_rad=(1.0,),
        )
        experiment.seed_angle(1.0)

        actions = experiment.start(0.0)

        self.assertEqual(actions[0].kind, "configure_position")
        self.assertAlmostEqual(experiment.current_position_target_rad, 1.1)
        self.assertAlmostEqual(experiment.positioning_voltage_limit_v, 0.5)
