import unittest

from foctwin.domain import MotionMode, TorqueMode
from foctwin.protocol import CommanderProtocol, parse_monitor_line


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
        self.assertEqual(protocol.pid("current_q", "p", 3), "AQP3")
        self.assertEqual(protocol.pid("velocity", "lpf", 0.01), "AVF0.01")

    def test_monitor_parser_requires_all_seven_columns(self):
        parsed = parse_monitor_line("1\t2\t3\t4\t5\t6\t7")
        self.assertEqual(parsed, {
            "target": 1.0,
            "voltage_q_v": 2.0,
            "voltage_d_v": 3.0,
            "current_q_a": 4.0,
            "current_d_a": 5.0,
            "velocity_rad_s": 6.0,
            "angle_rad": 7.0,
        })
        self.assertIsNone(parse_monitor_line("Status: enabled"))

    def test_invalid_monitor_mask_is_rejected(self):
        with self.assertRaises(ValueError):
            CommanderProtocol().monitor_variables("111")
