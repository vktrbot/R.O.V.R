"""Mock serial servo backend for arm and camera controllers."""

from __future__ import annotations

from dataclasses import dataclass

from .clamp import clamp
from .servo_calibration import ServoCalibration
from .smoothing import ema_step


@dataclass
class ServoBusState:
    positions: list[int]
    loads: list[float]
    torque_enabled: list[bool]


class FakeServoBus:
    """Simple in-memory servo bus that supports smoothing and jam simulation."""

    def __init__(self, servo_ids: list[int], initial_positions: list[int] | None = None) -> None:
        self.servo_ids = list(servo_ids)
        seed_positions = initial_positions or [2048] * len(self.servo_ids)
        self.positions = [int(value) for value in seed_positions]
        self.goal_positions = list(self.positions)
        self.loads = [0.0 for _ in self.servo_ids]
        self.torque_enabled = [False for _ in self.servo_ids]
        self.overload_flags = [False for _ in self.servo_ids]
        self.voltage_fault_flags = [False for _ in self.servo_ids]
        self.available = [True for _ in self.servo_ids]
        self.connected = False
        self.stuck_ids: set[int] = set()
        self.forced_loads: dict[int, float] = {}
        self.calibrations = [
            ServoCalibration(servo_id=servo_id, minimum=0, maximum=4095, home=int(position))
            for servo_id, position in zip(self.servo_ids, self.positions)
        ]
        self.history: list[ServoBusState] = []

    def load_calibration(self, calibrations: list[ServoCalibration]) -> None:
        self.calibrations = list(calibrations)
        if not self.connected:
            self.goal_positions = self.get_home_positions()

    def connect(self) -> None:
        self.connected = True
        self.disarm_all()
        self.goal_positions = list(self.positions)

    def read_positions(self) -> list[int]:
        return list(self.positions)

    def read_loads(self) -> list[float]:
        return list(self.loads)

    def read_temperatures(self) -> list[float]:
        return [0.0 for _ in self.servo_ids]

    def read_torque_enabled(self) -> list[bool]:
        return list(self.torque_enabled)

    def get_cached_torque_enabled(self) -> list[bool]:
        return self.read_torque_enabled()

    def get_overload_flags(self) -> list[bool]:
        return list(self.overload_flags)

    def get_voltage_fault_flags(self) -> list[bool]:
        return list(self.voltage_fault_flags)

    def get_command_limits(self) -> list[tuple[int, int]]:
        return [(int(item.minimum), int(item.maximum)) for item in self.calibrations]

    def get_home_positions(self) -> list[int]:
        return [int(item.home) for item in self.calibrations]

    def available_servo_ids(self) -> list[int]:
        return [servo_id for servo_id, available in zip(self.servo_ids, self.available) if available]

    def armed_states(self) -> list[bool]:
        return self.read_torque_enabled()

    def set_forced_load(self, servo_id: int, load: float) -> None:
        self.forced_loads[int(servo_id)] = float(load)

    def clear_forced_load(self, servo_id: int) -> None:
        self.forced_loads.pop(int(servo_id), None)

    def set_stuck(self, servo_id: int, stuck: bool = True) -> None:
        servo_id = int(servo_id)
        if stuck:
            self.stuck_ids.add(servo_id)
        else:
            self.stuck_ids.discard(servo_id)

    def set_overload(self, servo_id: int, overloaded: bool = True) -> None:
        index = self._servo_index(servo_id)
        self.overload_flags[index] = bool(overloaded)

    def disable_torque(self, servo_id: int) -> None:
        index = self._servo_index(servo_id)
        self.torque_enabled[index] = False
        self._record()

    def enable_torque(self, servo_id: int) -> None:
        index = self._servo_index(servo_id)
        self.torque_enabled[index] = True
        self.goal_positions[index] = self.positions[index]
        self._record()

    def arm_servo(self, servo_id: int) -> bool:
        index = self._servo_index(servo_id)
        if not self.available[index]:
            return False
        self.enable_torque(servo_id)
        return True

    def arm_servos(self, servo_ids: list[int]) -> list[int]:
        armed: list[int] = []
        for servo_id in servo_ids:
            if self.arm_servo(int(servo_id)):
                armed.append(int(servo_id))
        return armed

    def arm_all(self) -> list[int]:
        return self.arm_servos(self.available_servo_ids())

    def disarm_all(self) -> None:
        for index in range(len(self.servo_ids)):
            self.torque_enabled[index] = False
        self._record()

    def write_positions(self, positions: list[int]) -> None:
        for index, value in enumerate(positions):
            if self.torque_enabled[index]:
                servo_id = int(self.servo_ids[index])
                command = int(value)
                self.goal_positions[index] = command
                if servo_id not in self.stuck_ids:
                    self.positions[index] = command
        self._update_loads(positions)
        self._record()

    def step_towards(self, targets: list[int], alpha: float, max_step: int | None = None) -> list[int]:
        next_positions: list[int] = []
        for index, target in enumerate(targets):
            servo_id = self.servo_ids[index]
            current = self.positions[index]
            if not self.torque_enabled[index]:
                next_positions.append(current)
                continue
            if servo_id in self.stuck_ids:
                next_positions.append(current)
                continue
            smoothed = int(round(ema_step(float(current), float(target), alpha)))
            if max_step is not None and max_step > 0:
                lower = current - int(max_step)
                upper = current + int(max_step)
                smoothed = int(clamp(smoothed, lower, upper))
            next_positions.append(smoothed)
        self.positions = next_positions
        self.goal_positions = list(next_positions)
        self._update_loads(targets)
        self._record()
        return self.read_positions()

    def _update_loads(self, targets: list[int]) -> None:
        next_loads: list[float] = []
        for index, target in enumerate(targets):
            servo_id = self.servo_ids[index]
            forced = self.forced_loads.get(servo_id)
            if forced is not None:
                next_loads.append(float(forced))
                continue
            delta = abs(int(target) - int(self.positions[index]))
            next_loads.append(min(1.0, delta / 1024.0))
        self.loads = next_loads

    def _servo_index(self, servo_id: int) -> int:
        return self.servo_ids.index(int(servo_id))

    def _record(self) -> None:
        self.history.append(
            ServoBusState(
                positions=self.read_positions(),
                loads=self.read_loads(),
                torque_enabled=self.read_torque_enabled(),
            )
        )
