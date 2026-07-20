import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
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
            self.assertEqual(set(window.device_limit_spins), {"current_a", "voltage_v", "velocity_rad_s"})
            self.assertEqual(set(window.telemetry_values), set(window.monitor_checks))
        finally:
            window._reconnect_timer.stop()
            window.close()
