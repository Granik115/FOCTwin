from __future__ import annotations

import shlex
from dataclasses import dataclass, field

from foctwin.domain import MotionMode, SafetyLimits, TorqueMode
from foctwin.protocol import CommanderProtocol


@dataclass(slots=True)
class ScenarioStep:
    line_number: int
    operation: str
    arguments: tuple[str, ...]
    commander_commands: list[str] = field(default_factory=list)


class ScenarioError(ValueError):
    pass


class ScenarioCompiler:
    """Compile a small, auditable FOCTwin DSL to existing Commander commands."""

    def __init__(self, protocol: CommanderProtocol, limits: SafetyLimits) -> None:
        self.protocol = protocol
        self.limits = limits

    def compile(self, source: str) -> list[ScenarioStep]:
        steps: list[ScenarioStep] = []
        for line_number, raw_line in enumerate(source.splitlines(), start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                tokens = shlex.split(line, comments=True)
            except ValueError as exc:
                raise ScenarioError(f"Строка {line_number}: {exc}") from exc
            operation, *args = [token.upper() if index == 0 else token for index, token in enumerate(tokens)]
            commands = self._compile_step(line_number, operation, args)
            steps.append(ScenarioStep(line_number, operation, tuple(args), commands))
        return steps

    def _compile_step(self, line_number: int, operation: str, args: list[str]) -> list[str]:
        def require(count: int) -> None:
            if len(args) != count:
                raise ScenarioError(f"Строка {line_number}: {operation} ожидает {count} аргумент(а)")

        if operation == "EN":
            require(0)
            return [self.protocol.enable()]
        if operation in {"DIS", "STOP"}:
            require(0)
            return self.protocol.emergency_sequence()
        if operation == "TARGET":
            require(1)
            target = float(args[0])
            if not self.limits.angle_min_rad <= target <= self.limits.angle_max_rad:
                raise ScenarioError(f"Строка {line_number}: TARGET вне разрешённого диапазона")
            return [self.protocol.target(target)]
        if operation == "MODE":
            require(1)
            aliases = {
                "TORQUE": MotionMode.TORQUE,
                "VELOCITY": MotionMode.VELOCITY,
                "ANGLE": MotionMode.ANGLE,
                "VELOCITY_OPEN": MotionMode.VELOCITY_OPEN_LOOP,
                "ANGLE_OPEN": MotionMode.ANGLE_OPEN_LOOP,
            }
            try:
                return [self.protocol.motion_mode(aliases[args[0].upper()])]
            except KeyError as exc:
                raise ScenarioError(f"Строка {line_number}: неизвестный MODE") from exc
        if operation == "TORQUE":
            require(1)
            aliases = {
                "VOLTAGE": TorqueMode.VOLTAGE,
                "DC_CURRENT": TorqueMode.DC_CURRENT,
                "FOC_CURRENT": TorqueMode.FOC_CURRENT,
            }
            try:
                return [self.protocol.torque_mode(aliases[args[0].upper()])]
            except KeyError as exc:
                raise ScenarioError(f"Строка {line_number}: неизвестный TORQUE") from exc
        if operation == "LIMIT":
            require(2)
            selector, value = args[0].upper(), float(args[1])
            maximums = {
                "CURRENT": self.limits.current_a,
                "VOLTAGE": self.limits.voltage_v,
                "VELOCITY": self.limits.velocity_rad_s,
            }
            if selector not in maximums or value <= 0 or value > maximums[selector]:
                raise ScenarioError(f"Строка {line_number}: небезопасный LIMIT {selector}")
            encoders = {
                "CURRENT": self.protocol.current_limit,
                "VOLTAGE": self.protocol.voltage_limit,
                "VELOCITY": self.protocol.velocity_limit,
            }
            return [encoders[selector](value)]
        if operation == "RAW":
            require(1)
            if not args[0].startswith(self.protocol.device_id):
                raise ScenarioError(f"Строка {line_number}: RAW должен начинаться с ID мотора")
            return [args[0]]
        if operation == "WAIT":
            require(1)
            seconds = float(args[0])
            if seconds < 0 or seconds > self.limits.trial_timeout_s:
                raise ScenarioError(f"Строка {line_number}: WAIT превышает таймаут опыта")
            return []
        if operation in {"RECORD", "NOTE"}:
            if not args:
                raise ScenarioError(f"Строка {line_number}: {operation} требует аргумент")
            return []
        raise ScenarioError(f"Строка {line_number}: неизвестная команда {operation}")

