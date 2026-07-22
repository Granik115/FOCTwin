import unittest

from foctwin.domain import SafetyGuard, SafetyLimits, TelemetrySample


class DomainTests(unittest.TestCase):
    def test_safety_guard_reports_position_immediately(self):
        guard = SafetyGuard(SafetyLimits())
        sample = TelemetrySample(
            timestamp_s=0,
            current_q_a=1.2,
            velocity_rad_s=-0.8,
            angle_rad=7.0,
        )

        signals = {violation.signal for violation in guard.check(sample)}

        self.assertEqual(signals, {"angle_rad"})

    def test_safety_guard_requires_three_confirmed_soft_limit_samples(self):
        guard = SafetyGuard(SafetyLimits())
        sample = TelemetrySample(timestamp_s=0, current_q_a=1.2, velocity_rad_s=-0.8)

        self.assertEqual(guard.check(sample), [])
        self.assertEqual(guard.check(sample), [])
        signals = {violation.signal for violation in guard.check(sample)}

        self.assertEqual(signals, {"current_q_a", "velocity_rad_s"})

    def test_safety_guard_reports_extreme_sample_immediately(self):
        guard = SafetyGuard(SafetyLimits())
        sample = TelemetrySample(timestamp_s=0, current_q_a=2.1)

        violations = guard.check(sample)

        self.assertEqual([violation.signal for violation in violations], ["current_q_a"])
        self.assertIn("резкий выброс", violations[0].message)

    def test_safety_guard_can_ignore_untrusted_velocity_and_reset_history(self):
        guard = SafetyGuard(SafetyLimits())
        sample = TelemetrySample(timestamp_s=0, current_q_a=1.2, velocity_rad_s=43.0)

        self.assertEqual(
            guard.check(sample, ignored_signals=frozenset({"velocity_rad_s"})),
            [],
        )
        guard.check(sample, ignored_signals=frozenset({"velocity_rad_s"}))
        guard.reset()

        self.assertEqual(
            guard.check(sample, ignored_signals=frozenset({"velocity_rad_s"})),
            [],
        )

    def test_safety_guard_can_defer_angle_to_experiment_filter(self):
        guard = SafetyGuard(SafetyLimits(angle_min_rad=-1.0, angle_max_rad=1.0))
        sample = TelemetrySample(timestamp_s=0, angle_rad=1.5708)

        self.assertEqual(
            guard.check(sample, ignored_signals=frozenset({"angle_rad"})),
            [],
        )
