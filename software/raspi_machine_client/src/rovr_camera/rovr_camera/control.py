"""Pure control logic for the camera PTZ controller."""

from __future__ import annotations

from typing import Any

from rovr_common.clamp import clamp_int, clamp_sequence
from rovr_common.command_models import CameraTargetCommand
from rovr_common.smoothing import ema_step


class CameraCore:
    """Smooths and clamps PTZ target positions."""

    def __init__(
        self,
        limits: list[tuple[int, int]],
        bus: Any,
        ema_alpha: float,
        max_relative_target: int,
        target_snap_delta: int = 32,
    ) -> None:
        self.limits = list(limits)
        self.bus = bus
        self.ema_alpha = float(ema_alpha)
        self.max_relative_target = int(max_relative_target)
        self.target_snap_delta = int(max(1, target_snap_delta))
        self.target_positions = self.bus.read_positions()
        self.command_positions = list(self.target_positions)

    def arm_servo(self, servo_id: int) -> bool:
        if hasattr(self.bus, "arm_servo"):
            return bool(self.bus.arm_servo(int(servo_id)))
        return False

    def arm_all(self) -> list[int]:
        if hasattr(self.bus, "arm_all"):
            return [int(value) for value in self.bus.arm_all()]
        return []

    def disarm_all(self) -> None:
        if hasattr(self.bus, "disarm_all"):
            self.bus.disarm_all()
        self.target_positions = self.bus.read_positions()
        self.command_positions = list(self.target_positions)

    def apply_command(self, command: CameraTargetCommand) -> None:
        self.target_positions = self.clamp_positions(list(command.servos))

    def clamp_positions(self, positions: list[int]) -> list[int]:
        return clamp_sequence(positions, self.limits)

    def tick(self) -> list[int]:
        positions_before = self.bus.read_positions()
        next_commands: list[int] = []

        for index, target in enumerate(self.target_positions):
            current_command = int(self.command_positions[index])
            smoothed = int(round(ema_step(float(current_command), float(target), self.ema_alpha)))
            if self.max_relative_target > 0:
                smoothed = clamp_int(
                    smoothed,
                    current_command - int(self.max_relative_target),
                    current_command + int(self.max_relative_target),
                )
            lower_limit, upper_limit = self.limits[index]
            next_command = clamp_int(smoothed, int(lower_limit), int(upper_limit))
            if (
                abs(int(target) - int(positions_before[index])) <= self.target_snap_delta
                or abs(int(target) - int(next_command)) <= self.target_snap_delta
            ):
                next_command = int(target)
            next_commands.append(next_command)

        self.command_positions = list(next_commands)
        self.bus.write_positions(next_commands)
        return self.bus.read_positions()
