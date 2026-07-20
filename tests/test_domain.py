import unittest

from foctwin.domain import SafetyGuard, SafetyLimits, TelemetrySample


class DomainTests(unittest.TestCase):
    def test_safety_guard_reports_current_velocity_and_position(self):
        guard = SafetyGuard(SafetyLimits())
        sample = TelemetrySample(
            timestamp_s=0,
            current_q_a=1.2,
            velocity_rad_s=-0.8,
            angle_rad=7.0,
        )

        signals = {violation.signal for violation in guard.check(sample)}

        self.assertEqual(signals, {"current_q_a", "velocity_rad_s", "angle_rad"})
