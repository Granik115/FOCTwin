import unittest

from foctwin.domain import MotionMode, TorqueMode
from foctwin.protocol import CommanderProtocol, parse_commander_response, parse_monitor_line


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
        self.assertEqual(protocol.motion_mode(), "AC")
        self.assertEqual(protocol.enable(None), "AE")
        self.assertEqual(protocol.pid("current_q", "p", 3), "AQP3")
        self.assertEqual(protocol.pid("velocity", "lpf", 0.01), "AVF0.01")

    def test_monitor_parser_normalizes_streamed_milliamps(self):
        parsed = parse_monitor_line("1\t2\t3\t400\t-50\t6\t7")
        self.assertEqual(parsed, {
            "target": 1.0,
            "voltage_q_v": 2.0,
            "voltage_d_v": 3.0,
            "current_q_a": 0.4,
            "current_d_a": -0.05,
            "velocity_rad_s": 6.0,
            "angle_rad": 7.0,
        })
        self.assertIsNone(parse_monitor_line("Status: enabled"))

    def test_monitor_parser_supports_selected_variables(self):
        self.assertEqual(
            parse_monitor_line("1\t250\t0.5", "1001010"),
            {"target": 1.0, "current_q_a": 0.25, "velocity_rad_s": 0.5},
        )

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
