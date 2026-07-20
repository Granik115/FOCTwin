import unittest

from foctwin.domain import SafetyLimits
from foctwin.protocol import CommanderProtocol
from foctwin.scenario import ScenarioCompiler, ScenarioError


class ScenarioTests(unittest.TestCase):
    def compiler(self) -> ScenarioCompiler:
        return ScenarioCompiler(CommanderProtocol("A"), SafetyLimits())

    def test_scenario_compiles_human_commands_to_board_commands(self):
        steps = self.compiler().compile(
            """
            LIMIT CURRENT 1
            LIMIT VOLTAGE 12
            MODE ANGLE
            TORQUE VOLTAGE
            EN
            TARGET 0.2
            WAIT 1
            STOP
            """
        )

        flattened = [command for step in steps for command in step.commander_commands]
        self.assertEqual(flattened[:6], ["ALC1", "ALU12", "AC2", "AT0", "AE1", "A0.2"])
        self.assertEqual(flattened[-4:], ["A0", "AE0", "AE0", "AE0"])

    def test_target_outside_software_travel_is_rejected(self):
        with self.assertRaisesRegex(ScenarioError, "TARGET"):
            self.compiler().compile("TARGET 7")

    def test_limit_cannot_exceed_active_safety_envelope(self):
        with self.assertRaisesRegex(ScenarioError, "LIMIT"):
            self.compiler().compile("LIMIT CURRENT 1.1")

    def test_raw_command_must_target_active_motor_id(self):
        with self.assertRaisesRegex(ScenarioError, "ID мотора"):
            self.compiler().compile("RAW BE0")
