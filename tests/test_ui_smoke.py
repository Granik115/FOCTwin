import os
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtCore import QSettings, Qt
    from PySide6.QtWidgets import QApplication

    from foctwin.protocol import CommanderResponse
    from foctwin.ui import MainWindow
except ImportError:
    QApplication = None
    MainWindow = None


@unittest.skipIf(QApplication is None, "PySide6 runtime is unavailable")
class UiSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def test_main_window_builds_with_manual_control_widgets(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            self.assertEqual(window.navigation.count(), len(window.NAVIGATION))
            self.assertEqual(window.pid_tabs.count(), 4)
            self.assertTrue(all(pid_table.rowCount() == 5 for pid_table in window.pid_tables.values()))
            self.assertNotIn("limit", window.PID_ROW_BY_FIELD)
            self.assertEqual(set(window.device_limit_spins), {"current_a", "voltage_v", "velocity_rad_s"})
            self.assertEqual(set(window.telemetry_values), set(window.monitor_checks))
            self.assertEqual(window.plot_window_spin.value(), 30)
            self.assertTrue(window.plot_follow_checkbox.isChecked())
            self.assertFalse(window.raw_telemetry_checkbox.isChecked())
            self.assertIs(window.manual_scroll.widget(), window.manual_splitter)
            self.assertGreaterEqual(window.manual_splitter.minimumWidth(), 1040)
            self.assertEqual(window.friction_points_table.rowCount(), 12)
            self.assertEqual(window.friction_actuator_table.rowCount(), 0)
            self.assertEqual(window.friction_position_table.rowCount(), 0)
            self.assertEqual(window.friction_positioning_table.rowCount(), 0)
            self.assertGreater(window.friction_current_trip.maximum(), 1.0)
            self.assertEqual(window.friction_pulse_start.value(), 0.1)
            self.assertEqual(window.friction_pulse_step.value(), 0.1)
            self.assertEqual(window.friction_pulse_max.value(), 0.5)
            self.assertEqual(window.friction_movement_threshold.value(), 0.001)
            self.assertEqual(window.friction_recoveries.value(), 50)
            self.assertEqual(window.friction_position_bin.value(), 0.1)
            self.assertEqual(window.friction_automatic_positions.value(), 1)
            self.assertEqual(window.friction_automatic_position_step.value(), 1.0)
            self.assertEqual(window.friction_map_passes.value(), 2)
            self.assertEqual(window.friction_position_tolerance.value(), 0.005)
            self.assertEqual(window.friction_position_voltage_step.value(), 0.25)
            self.assertEqual(window.friction_position_voltage_max.value(), 3.0)
            self.assertTrue(window.friction_evidence_mode.isChecked())
            self.assertEqual(window.friction_pwm_off_observation.value(), 60.0)
            self.assertEqual(window.friction_breakaway_repeats.value(), 3)
            self.assertEqual(window.friction_residual_movement.value(), 0.005)
            self.assertEqual(window.friction_velocity_travel.value(), 0.2)
            self.assertEqual(window.friction_fixed_velocity_voltage.value(), 3.0)
            self.assertFalse(window.friction_adaptive_positioning.isChecked())
            self.assertTrue(window.friction_position_validation.isChecked())
            self.assertGreater(window.friction_high_speed.maximum(), 0.05)
            self.assertGreater(window.friction_voltage_limit.maximum(), 12.0)
            self.assertGreater(window.friction_velocity_limit.maximum(), 0.3)
            self.assertGreater(window.friction_angle_max.maximum(), 3.0)
            self.assertFalse(window.friction_stop_button.isEnabled())
            self.assertEqual(
                window.manual_scroll.horizontalScrollBarPolicy(),
                Qt.ScrollBarPolicy.ScrollBarAsNeeded,
            )
            window.close()

    def test_manual_configuration_is_restored_between_program_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = f"{temporary}/settings.ini"
            first = MainWindow(QSettings(path, QSettings.Format.IniFormat))
            first.port_combo.setCurrentText("COM17")
            first.motion_combo.setCurrentIndex(first.motion_combo.findData("velocity"))
            first.target_spin.setValue(1.25)
            first.device_limit_spins["current_a"].setValue(8.5)
            first.current_limit.setValue(7.5)
            first.monitor_downsample_spin.setValue(12)
            first.pid_tables["velocity"].item(first.PID_ROW_BY_FIELD["p"], 1).setText("21.5")
            first._save_user_settings()
            first.close()

            second = MainWindow(QSettings(path, QSettings.Format.IniFormat))
            self.assertEqual(second.port_combo.currentText(), "COM17")
            self.assertEqual(second.motion_combo.currentData(), "velocity")
            self.assertEqual(second.target_spin.value(), 1.25)
            self.assertEqual(second.device_limit_spins["current_a"].value(), 8.5)
            self.assertEqual(second.current_limit.value(), 7.5)
            self.assertEqual(second.monitor_downsample_spin.value(), 12)
            self.assertEqual(
                second.pid_tables["velocity"].item(second.PID_ROW_BY_FIELD["p"], 1).text(),
                "21.5",
            )
            second.close()

    def test_friction_configuration_is_persisted(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = f"{temporary}/settings.ini"
            first = MainWindow(QSettings(path, QSettings.Format.IniFormat))
            first.friction_low_speed.setValue(0.015)
            first.friction_high_speed.setValue(0.1)
            first.friction_current_trip.setValue(2.0)
            first.friction_voltage_limit.setValue(24.0)
            first.friction_velocity_limit.setValue(0.5)
            first.friction_pulse_start.setValue(0.2)
            first.friction_pulse_step.setValue(0.15)
            first.friction_pulse_max.setValue(0.8)
            first.friction_movement_threshold.setValue(0.003)
            first.friction_angle_min.setValue(-4.0)
            first.friction_angle_max.setValue(4.0)
            first.friction_measure.setValue(8.0)
            first.friction_recoveries.setValue(5)
            first.friction_position_bin.setValue(0.2)
            first.friction_automatic_positions.setValue(4)
            first.friction_automatic_position_step.setValue(-0.75)
            first.friction_map_passes.setValue(3)
            first.friction_position_tolerance.setValue(0.003)
            first.friction_position_voltage_step.setValue(0.4)
            first.friction_position_voltage_max.setValue(4.0)
            first.friction_position_stall_window.setValue(4.0)
            first.friction_position_min_progress.setValue(0.001)
            first.friction_evidence_mode.setChecked(True)
            first.friction_pwm_off_observation.setValue(90.0)
            first.friction_breakaway_repeats.setValue(4)
            first.friction_residual_movement.setValue(0.007)
            first.friction_breakaway_verify.setValue(1.4)
            first.friction_velocity_travel.setValue(0.25)
            first.friction_fixed_velocity_voltage.setValue(3.5)
            first.friction_adaptive_positioning.setChecked(False)
            first.friction_pole_pairs.setValue(15)
            first.friction_electrical_divisions.setValue(10)
            first.friction_position_validation.setChecked(True)
            first.friction_position_validation_small.setValue(0.12)
            first.friction_position_validation_medium.setValue(0.35)
            first.friction_position_validation_large.setValue(0.7)
            first._save_user_settings()
            first.close()

            second = MainWindow(QSettings(path, QSettings.Format.IniFormat))
            self.assertEqual(second.friction_low_speed.value(), 0.015)
            self.assertEqual(second.friction_high_speed.value(), 0.1)
            self.assertEqual(second.friction_current_trip.value(), 2.0)
            self.assertEqual(second.friction_voltage_limit.value(), 24.0)
            self.assertEqual(second.friction_velocity_limit.value(), 0.5)
            self.assertEqual(second.friction_pulse_start.value(), 0.2)
            self.assertEqual(second.friction_pulse_step.value(), 0.15)
            self.assertEqual(second.friction_pulse_max.value(), 0.8)
            self.assertEqual(second.friction_movement_threshold.value(), 0.003)
            self.assertEqual(second.friction_angle_min.value(), -4.0)
            self.assertEqual(second.friction_angle_max.value(), 4.0)
            self.assertEqual(second.friction_measure.value(), 8.0)
            self.assertEqual(second.friction_recoveries.value(), 5)
            self.assertEqual(second.friction_position_bin.value(), 0.2)
            self.assertEqual(second.friction_automatic_positions.value(), 4)
            self.assertEqual(second.friction_automatic_position_step.value(), -0.75)
            self.assertEqual(second.friction_map_passes.value(), 3)
            self.assertEqual(second.friction_position_tolerance.value(), 0.003)
            self.assertEqual(second.friction_position_voltage_step.value(), 0.4)
            self.assertEqual(second.friction_position_voltage_max.value(), 4.0)
            self.assertEqual(second.friction_position_stall_window.value(), 4.0)
            self.assertEqual(second.friction_position_min_progress.value(), 0.001)
            self.assertTrue(second.friction_evidence_mode.isChecked())
            self.assertEqual(second.friction_pwm_off_observation.value(), 90.0)
            self.assertEqual(second.friction_breakaway_repeats.value(), 4)
            self.assertEqual(second.friction_residual_movement.value(), 0.007)
            self.assertEqual(second.friction_breakaway_verify.value(), 1.4)
            self.assertEqual(second.friction_velocity_travel.value(), 0.25)
            self.assertEqual(second.friction_fixed_velocity_voltage.value(), 3.5)
            self.assertFalse(second.friction_adaptive_positioning.isChecked())
            self.assertEqual(second.friction_pole_pairs.value(), 15)
            self.assertEqual(second.friction_electrical_divisions.value(), 10)
            self.assertTrue(second.friction_position_validation.isChecked())
            self.assertEqual(second.friction_position_validation_small.value(), 0.12)
            self.assertEqual(second.friction_position_validation_medium.value(), 0.35)
            self.assertEqual(second.friction_position_validation_large.value(), 0.7)
            second.close()

    def test_friction_configuration_forces_bounded_modes_and_monitoring(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            commands = window._friction_configuration_commands(
                window._friction_config_from_widgets()
            )

            self.assertEqual(
                commands[:5],
                ["AE0", "AR-12345", "ALC1", "ALU12", "ALV0.3"],
            )
            self.assertEqual(
                commands[-7:],
                ["AT0", "AC0", "A0", "AMC", "AMD20", "AMS1111111", "AE1"],
            )
            window.close()

    def test_friction_configuration_uses_raised_user_limits(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            window.friction_current_trip.setValue(2.0)
            window.friction_voltage_limit.setValue(24.0)
            window.friction_velocity_limit.setValue(0.5)
            window.friction_pulse_max.setValue(0.8)

            config = window._friction_config_from_widgets()
            config.validate()
            commands = window._friction_configuration_commands(config)

            self.assertEqual(
                commands[:5],
                ["AE0", "AR-12345", "ALC2", "ALU24", "ALV0.5"],
            )
            velocity_commands = window._friction_configuration_commands(
                config,
                mode="velocity",
                working_current_limit_a=0.42,
            )
            self.assertEqual(
                velocity_commands[:5],
                ["AE0", "AR0.675", "ALC0.42", "ALU24", "ALV0.5"],
            )
            self.assertEqual(velocity_commands[-7:][1], "AC1")
            window.close()

    def test_friction_limits_are_applied_automatically_and_explained(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            window.friction_current_trip.setValue(1.0)
            window.friction_voltage_limit.setValue(24.0)
            window.friction_velocity_limit.setValue(0.5)
            window.friction_angle_min.setValue(-4.0)
            window.friction_angle_max.setValue(4.0)
            window.friction_pulse_max.setValue(2.0)
            config = window._friction_config_from_widgets()

            text = window._friction_confirmation_text(config, (0.0, 1.0, 2.0))
            window._apply_friction_limits(config)

            self.assertIn("автоматически", text)
            self.assertIn("2.96 А", text)
            self.assertIn("не будет искусственно повышен", text)
            self.assertIn("Скоростные участки действительно перемещают вал", text)
            self.assertIn("интервалы по 0.1 рад", text)
            self.assertIn("Измерений по координатам: 3 (0, 1, 2 рад)", text)
            self.assertIn("Позиционный ALC фиксирован", text)
            self.assertIn("остаточным перемещением", text)
            self.assertIn("PWM отключённым", text)
            self.assertIn("шесть скоростных точек", text)
            self.assertIn("angle + Voltage torque", text)
            self.assertEqual(window.guard.limits.current_a, 1.0)
            self.assertEqual(window.guard.limits.voltage_v, 24.0)
            self.assertEqual(window.guard.limits.velocity_rad_s, 0.5)
            self.assertEqual(window.guard.limits.angle_min_rad, -4.0)
            self.assertEqual(window.guard.limits.angle_max_rad, 4.0)
            self.assertEqual(window.device_limit_spins["current_a"].value(), 1.0)
            self.assertEqual(window.device_limit_spins["voltage_v"].value(), 24.0)
            self.assertEqual(window.device_limit_spins["velocity_rad_s"].value(), 0.5)
            window.close()

    def test_evidence_observer_keeps_pwm_disabled_and_preset_uses_two_periods(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            config = window._friction_config_from_widgets()

            commands = window._friction_configuration_commands(config, mode="observer")
            window._apply_friction_evidence_preset()
            preset = window._friction_config_from_widgets()

            self.assertEqual(commands[0], "AE0")
            self.assertNotIn("AE1", commands)
            self.assertTrue(preset.evidence_mode)
            self.assertEqual(preset.automatic_position_count, 17)
            self.assertEqual(preset.map_passes, 2)
            self.assertAlmostEqual(
                preset.automatic_position_step_rad,
                preset.recommended_electrical_step_rad,
                places=6,
            )
            self.assertFalse(preset.adaptive_positioning_enabled)
            window.close()

    def test_automatic_positioning_uses_angle_mode_and_loaded_pid(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)

            commands = window._friction_positioning_commands(
                window._friction_config_from_widgets(),
                1.25,
                2.5,
            )

            self.assertEqual(
                commands[:5],
                ["AE0", "AR0.675", "ALC2.5", "ALU12", "ALV0.3"],
            )
            self.assertIn("AAP35", commands)
            self.assertIn("AVP20.4", commands)
            self.assertEqual(
                commands[-7:],
                ["AT0", "AC2", "A1.25", "AMC", "AMD20", "AMS1111111", "AE1"],
            )
            window.close()

    def test_slow_recovery_beeps_once_only_when_enabled(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            window._friction_recovery_started_at = 100.0
            window._friction_recovery_sound_enabled = True
            window._friction_recovery_alerted = False

            with patch("foctwin.ui.QApplication.beep") as beep:
                self.assertFalse(window._alert_slow_friction_recovery(105.0))
                self.assertTrue(window._alert_slow_friction_recovery(105.1))
                self.assertFalse(window._alert_slow_friction_recovery(110.0))
                beep.assert_called_once_with()

            window._friction_recovery_sound_enabled = False
            window._friction_recovery_alerted = False
            with patch("foctwin.ui.QApplication.beep") as beep:
                self.assertFalse(window._alert_slow_friction_recovery(110.0))
                beep.assert_not_called()
            window.close()

    def test_full_configuration_contains_limits_pid_modes_target_and_monitoring(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            pid_values = {loop: window._pid_values(loop) for loop in window.pid_tables}
            commands = window._full_configuration_commands(pid_values, "1111111")

            self.assertEqual(commands[:4], ["AR0.675", "ALC10", "ALU20", "ALV0.7"])
            self.assertIn("AAP35", commands)
            self.assertIn("AVP20.4", commands)
            self.assertIn("AQP8.4222", commands)
            self.assertIn("ADP8.4222", commands)
            self.assertEqual(commands[-6:], ["AT0", "AC2", "A0", "AMC", "AMD20", "AMS1111111"])
            window.close()

    def test_automatic_readback_does_not_replace_desired_device_limit(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            window.device_limit_spins["current_a"].setValue(8.0)

            response = CommanderResponse("limit.current_a", 1.0, "Limits| curr: 1.000")
            window._apply_commander_response(response)
            self.assertEqual(window.device_limit_spins["current_a"].value(), 8.0)
            self.assertEqual(window.device_limit_confirmed["current_a"].text(), "1")

            window._device_limit_copy_pending.add("current_a")
            window._apply_commander_response(response)
            self.assertEqual(window.device_limit_spins["current_a"].value(), 1.0)
            window.close()
