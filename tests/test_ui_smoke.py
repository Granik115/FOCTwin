import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication

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
        window = MainWindow()
        try:
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
        finally:
            window._reconnect_timer.stop()
            window._telemetry_ui_timer.stop()
            window._telemetry_watchdog_timer.stop()
            window.close()
