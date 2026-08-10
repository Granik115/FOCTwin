import unittest

from foctwin.domain import MotionMode, SafetyGuard, SafetyLimits, TelemetrySample, TorqueMode
from foctwin.protocol import (
    CommanderProtocol,
    is_monitor_candidate,
    parse_commander_response,
    parse_monitor_line,
)


class ProtocolTests(unittest.TestCase):
    def test_commander_commands_match_simplefoc_grammar(self):
        protocol = CommanderProtocol("A")

        self.assertEqual(protocol.enable(), "AE1")
        self.assertEqual(protocol.disable(), "AE0")
        self.assertEqual(protocol.target(0.25), "A0.25")
        self.assertEqual(protocol.motion_mode(MotionMode.ANGLE), "AC2")
        self.assertEqual(protocol.torque_mode(TorqueMode.FOC_CURRENT), "AT2")
        self.assertEqual(protocol.current_limit(1), "ALC1")
        self.assertEqual(protocol.voltage_limit(12), "ALU12")
        self.assertEqual(protocol.velocity_limit(0.7), "ALV0.7")
        self.assertEqual(protocol.current_limit(), "ALC")
        self.assertEqual(protocol.phase_resistance(0.675), "AR0.675")
        self.assertEqual(protocol.phase_resistance(-12345), "AR-12345")
        self.assertEqual(protocol.phase_resistance(), "AR")
        self.assertEqual(protocol.motion_mode(), "AC")
        self.assertEqual(protocol.enable(None), "AE")
        self.assertEqual(protocol.pid("current_q", "p", 3), "AQP3")
        self.assertEqual(protocol.pid("velocity", "lpf", 0.01), "AVF0.01")
        self.assertEqual(protocol.monitor_clear(), "AMC")

    def test_monitor_parser_preserves_streamed_amperes(self):
        parsed = parse_monitor_line(
            "0.0588\t-0.5818\t0.0017\t-5.8088\t-29.9526\t0.0958\t0.0587"
        )
        self.assertEqual(parsed, {
            "target": 0.0588,
            "voltage_q_v": -0.5818,
            "voltage_d_v": 0.0017,
            "current_q_a": -5.8088,
            "current_d_a": -29.9526,
            "velocity_rad_s": 0.0958,
            "angle_rad": 0.0587,
        })
        self.assertIsNone(parse_monitor_line("Status: enabled"))

    def test_trial_61_current_packet_reaches_emergency_guard_without_rescaling(self):
        parsed = parse_monitor_line(
            "0.0588\t-0.5818\t0.0017\t-5.8088\t-29.9526\t0.0958\t0.0587"
        )
        sample = TelemetrySample(timestamp_s=0.0, **parsed)

        violations = SafetyGuard(SafetyLimits(current_a=0.1)).check(sample)

        self.assertEqual(
            {violation.signal for violation in violations},
            {"current_q_a", "current_d_a"},
        )

    def test_monitor_parser_supports_selected_variables(self):
        self.assertEqual(
            parse_monitor_line("1.0000\t0.2500\t0.5000", "1001010"),
            {"target": 1.0, "current_q_a": 0.25, "velocity_rad_s": 0.5},
        )

    def test_monitor_parser_rejects_character_loss_that_still_looks_numeric(self):
        lost_decimal = "10000\t0.1000\t0.0000\t1.0000\t2.0000\t0.0000\t1.0000"
        lost_leading_digit = ".0000\t0.1000\t0.0000\t1.0000\t2.0000\t0.0000\t1.0000"

        self.assertIsNone(parse_monitor_line(lost_decimal))
        self.assertIsNone(parse_monitor_line(lost_leading_digit))
        self.assertTrue(is_monitor_candidate(lost_decimal))
        self.assertTrue(is_monitor_candidate(lost_leading_digit))

    def test_commander_response_parser_reads_limits_status_and_pid(self):
        current = parse_commander_response("Limits| curr: 1.000")
        status = parse_commander_response("Status: 1")
        pid = parse_commander_response("PID angle| limit: 0.700")

        self.assertEqual((current.key, current.value), ("limit.current_a", 1.0))
        self.assertEqual((status.key, status.value), ("enabled", True))
        self.assertEqual((pid.key, pid.value), ("pid.angle.limit", 0.7))

    def test_invalid_monitor_mask_is_rejected(self):
        with self.assertRaises(ValueError):
            CommanderProtocol().monitor_variables("111")
