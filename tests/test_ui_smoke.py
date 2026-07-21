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

    def test_full_configuration_contains_limits_pid_modes_target_and_monitoring(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = QSettings(f"{temporary}/settings.ini", QSettings.Format.IniFormat)
            window = MainWindow(settings)
            pid_values = {loop: window._pid_values(loop) for loop in window.pid_tables}
            commands = window._full_configuration_commands(pid_values, "1111111")

            self.assertEqual(commands[:3], ["ALC10", "ALU20", "ALV0.7"])
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
