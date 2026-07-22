import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtCore import QSettings, Qt
    from PySide6.QtWidgets import QApplication

    from foctwin.ui import MainWindow
    from foctwin.protocol import CommanderResponse
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
            self.assertEqual(window.friction_points_table.rowCount(), 4)
            self.assertEqual(window.friction_actuator_table.rowCount(), 0)
            self.assertGreater(window.friction_current_trip.maximum(), 1.0)
            self.assertEqual(window.friction_pulse_start.value(), 0.1)
            self.assertEqual(window.friction_pulse_step.value(), 0.1)
            self.assertEqual(window.friction_pulse_max.value(), 0.5)
            self.assertEqual(window.friction_movement_threshold.value(), 0.001)
            self.assertEqual(window.friction_recoveries.value(), 50)
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

            text = window._friction_confirmation_text(config)
            window._apply_friction_limits(config)

            self.assertIn("автоматически", text)
            self.assertIn("2.96 А", text)
            self.assertIn("не будет искусственно повышен", text)
            self.assertEqual(window.guard.limits.current_a, 1.0)
            self.assertEqual(window.guard.limits.voltage_v, 24.0)
            self.assertEqual(window.guard.limits.velocity_rad_s, 0.5)
            self.assertEqual(window.guard.limits.angle_min_rad, -4.0)
            self.assertEqual(window.guard.limits.angle_max_rad, 4.0)
            self.assertEqual(window.device_limit_spins["current_a"].value(), 1.0)
            self.assertEqual(window.device_limit_spins["voltage_v"].value(), 24.0)
            self.assertEqual(window.device_limit_spins["velocity_rad_s"].value(), 0.5)
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
