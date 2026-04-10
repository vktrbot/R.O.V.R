"""Real serial servo backend backed by the st3215 library."""

from __future__ import annotations

import logging
import time

from st3215 import ST3215
from st3215.values import (
    COMM_SUCCESS,
    ERRBIT_ANGLE,
    ERRBIT_OVERELE,
    ERRBIT_OVERHEAT,
    ERRBIT_OVERLOAD,
    ERRBIT_VOLTAGE,
    STS_GOAL_POSITION_L,
    STS_MODE,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_LOAD_L,
    STS_PRESENT_VOLTAGE,
    STS_STATUS,
    STS_TORQUE_ENABLE,
)

from .clamp import clamp
from .servo_calibration import ServoCalibration
from .smoothing import ema_step

# STS3215 register addresses not exposed in the third-party values table.
STS_MAX_TORQUE_LIMIT_L = 16
STS_P_COEFFICIENT = 21
STS_D_COEFFICIENT = 22
STS_I_COEFFICIENT = 23
STS_CW_DEAD = 26
STS_CCW_DEAD = 27
STS_PROTECTION_CURRENT_L = 28
STS_PRESENT_TEMPERATURE = 63
STS_PROTECTION_TIME = 35
STS_OVERLOAD_TORQUE = 36


class St3215ServoBus:
    """Servo bus adapter with explicit connect/load-calibration/arm phases."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        servo_ids: list[int],
        logger: logging.Logger | None = None,
        strict: bool = True,
        position_speed: int = 2400,
        position_acceleration: int = 50,
        position_speed_per_servo: list[int] | None = None,
        position_acceleration_per_servo: list[int] | None = None,
        pid_p_per_servo: list[int] | None = None,
        pid_i_per_servo: list[int] | None = None,
        pid_d_per_servo: list[int] | None = None,
        max_torque_limit_per_servo: list[int] | None = None,
        protection_current_per_servo: list[int] | None = None,
        overload_torque_per_servo: list[int] | None = None,
        protection_time_per_servo: list[int] | None = None,
        use_sync_write: bool = True,
        diag_poll_period_sec: float = 0.4,
        compliance_margin: int = 0,
    ) -> None:
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.servo_ids = [int(value) for value in servo_ids]
        self.logger = logger or logging.getLogger(__name__)
        self.strict = bool(strict)
        self.position_speed = int(clamp(int(position_speed), 0, 3400))
        self.position_acceleration = int(clamp(int(position_acceleration), 0, 254))
        self.position_speed_per_servo = self._normalize_profile(
            values=position_speed_per_servo,
            maximum=3400,
            fallback=self.position_speed,
        )
        self.position_acceleration_per_servo = self._normalize_profile(
            values=position_acceleration_per_servo,
            maximum=254,
            fallback=self.position_acceleration,
        )
        self.pid_p_per_servo = self._normalize_profile(
            values=pid_p_per_servo,
            maximum=254,
            fallback=24,
        )
        self.pid_i_per_servo = self._normalize_profile(
            values=pid_i_per_servo,
            maximum=254,
            fallback=0,
        )
        self.pid_d_per_servo = self._normalize_profile(
            values=pid_d_per_servo,
            maximum=254,
            fallback=32,
        )
        self.max_torque_limit_per_servo = self._normalize_profile(
            values=max_torque_limit_per_servo,
            maximum=1000,
            fallback=900,
        )
        self.protection_current_per_servo = self._normalize_profile(
            values=protection_current_per_servo,
            maximum=1000,
            fallback=750,
        )
        self.overload_torque_per_servo = self._normalize_profile(
            values=overload_torque_per_servo,
            maximum=254,
            fallback=90,
        )
        self.protection_time_per_servo = self._normalize_profile(
            values=protection_time_per_servo,
            maximum=254,
            fallback=15,
        )
        self.use_sync_write = bool(use_sync_write)
        self.diag_poll_period_sec = max(0.05, float(diag_poll_period_sec))
        self.compliance_margin = int(clamp(int(compliance_margin), 0, 127))

        self.dev = ST3215(self.port)
        self.connected = False
        self.available = [True for _ in self.servo_ids]
        self.torque_enabled = [False for _ in self.servo_ids]
        self.armed = [False for _ in self.servo_ids]
        self.last_positions_raw = [2048 for _ in self.servo_ids]
        self.last_loads = [0.0 for _ in self.servo_ids]
        self.last_voltages = [0.0 for _ in self.servo_ids]
        self.last_currents = [0.0 for _ in self.servo_ids]
        self.last_temperatures = [0.0 for _ in self.servo_ids]
        self.goal_positions_raw = [2048 for _ in self.servo_ids]
        self.overload_flags = [False for _ in self.servo_ids]
        self.voltage_fault_flags = [False for _ in self.servo_ids]
        self._comm_last_success = time.monotonic()
        self._comm_recent_failures = 0
        self._last_diag_poll = 0.0
        self._warn_times: dict[str, float] = {}  # throttle repeated serial-failure warnings
        self.calibrations = [
            ServoCalibration(servo_id=servo_id, minimum=0, maximum=4095, home=2048)
            for servo_id in self.servo_ids
        ]

    def load_calibration(self, calibrations: list[ServoCalibration]) -> None:
        """Attach calibration data before connect/arming."""
        self.calibrations = list(calibrations)
        if not self.connected:
            self.goal_positions_raw = [item.command_to_raw(item.home) for item in self.calibrations]

    def connect(self) -> None:
        """Open the port, discover live IDs, disable torque, and sync goals to current state."""
        self._configure_port()

        missing = [servo_id for servo_id in self.servo_ids if not self._ping(servo_id)]
        missing_set = set(missing)
        self.available = [servo_id not in missing_set for servo_id in self.servo_ids]
        self.armed = [False for _ in self.servo_ids]
        self.torque_enabled = [False for _ in self.servo_ids]

        if self.strict and missing:
            raise RuntimeError(f"Servos not responding on {self.port}: {missing}")
        if missing:
            self.logger.warning(f"Some servos did not respond on {self.port}: {missing}")

        for servo_id in self.available_servo_ids():
            self._log_servo_state(servo_id)

        raw_positions: list[int] = []
        for index, servo_id in enumerate(self.servo_ids):
            if not self.available[index]:
                raw_positions.append(int(self.goal_positions_raw[index]))
                continue
            raw_position = self._read_position(servo_id)
            if raw_position is None:
                raw_position = self.goal_positions_raw[index]
            raw_positions.append(int(raw_position))
        self.last_positions_raw = list(raw_positions)

        self.last_loads = self.read_loads()
        self.disarm_all()
        self.sync_goals_to_current()
        self.connected = True

    def get_command_limits(self) -> list[tuple[int, int]]:
        return [(int(item.minimum), int(item.maximum)) for item in self.calibrations]

    def get_home_positions(self) -> list[int]:
        return [int(item.home) for item in self.calibrations]

    def available_servo_ids(self) -> list[int]:
        return [servo_id for servo_id, available in zip(self.servo_ids, self.available) if available]

    def armed_states(self) -> list[bool]:
        return list(self.armed)

    def read_positions(self) -> list[int]:
        """Read current positions in command-space with retry and fallback."""
        raw_positions: list[int] = []
        for index, servo_id in enumerate(self.servo_ids):
            if not self.available[index]:
                raw_positions.append(int(self.last_positions_raw[index]))
                continue
            raw_position = self._read_position(servo_id)
            if raw_position is None:
                raw_position = self.last_positions_raw[index]
            raw_positions.append(int(raw_position))
        self.last_positions_raw = list(raw_positions)
        return [
            calibration.raw_to_command(raw_value)
            for calibration, raw_value in zip(self.calibrations, raw_positions)
        ]

    def read_loads(self) -> list[float]:
        """Read normalized loads (0..1) from cached periodic diagnostics."""
        self._poll_diagnostics(force=False)
        return list(self.last_loads)

    def get_overload_flags(self) -> list[bool]:
        self._poll_diagnostics(force=False)
        return list(self.overload_flags)

    def get_voltage_fault_flags(self) -> list[bool]:
        self._poll_diagnostics(force=False)
        return list(self.voltage_fault_flags)

    def read_temperatures(self) -> list[float]:
        """Return cached servo temperatures in Celsius from last diagnostic poll."""
        self._poll_diagnostics(force=False)
        return list(self.last_temperatures)

    def read_torque_enabled(self) -> list[bool]:
        """Read the torque enable register for each servo."""
        states: list[bool] = []
        for index, servo_id in enumerate(self.servo_ids):
            if not self.available[index]:
                states.append(False)
                continue
            try:
                value, comm, err = self.dev.read1ByteTxRx(int(servo_id), STS_TORQUE_ENABLE)
            except Exception:
                states.append(self.torque_enabled[index])
                self._mark_comm_failure()
                continue
            if comm == COMM_SUCCESS and err == 0:
                state = bool(value)
                self._mark_comm_success()
            else:
                state = self.torque_enabled[index]
                self._mark_comm_failure()
                self.logger.warning(
                    f"Torque read rejected for servo {servo_id} on {self.port}: "
                    f"comm={comm} err={err} flags={self._decode_error_flags(err)}"
                )
            states.append(state)
        self.torque_enabled = list(states)
        self.armed = [armed and torque for armed, torque in zip(self.armed, self.torque_enabled)]
        return states

    def get_cached_torque_enabled(self) -> list[bool]:
        return list(self.torque_enabled)

    def comm_link_ok(self, max_silence_sec: float = 0.8, max_recent_failures: int = 8) -> bool:
        age = time.monotonic() - self._comm_last_success
        return age <= float(max_silence_sec) and self._comm_recent_failures <= int(max_recent_failures)

    def sync_goals_to_current(self) -> None:
        """Use current raw positions as the active goals so future arming does not jump."""
        for index, available in enumerate(self.available):
            if not available:
                continue
            self.goal_positions_raw[index] = int(self.last_positions_raw[index])

    def arm_servo(self, servo_id: int) -> bool:
        """Switch one servo into position mode and enable torque safely."""
        index = self._servo_index(servo_id)
        if not self.available[index]:
            return False
        if self.armed[index]:
            return True

        current_raw = self._read_position(int(servo_id), retries=2)
        if current_raw is not None:
            self.last_positions_raw[index] = int(current_raw)
            self.goal_positions_raw[index] = int(current_raw)

        current_mode = self._read_mode(int(servo_id))
        if current_mode != 0:
            self._set_mode_position(int(servo_id))
            time.sleep(0.02)

        self._apply_register_profile(int(servo_id), index)
        profile_acceleration = int(self.position_acceleration_per_servo[index])
        profile_speed = int(self.position_speed_per_servo[index])
        self._set_acceleration(int(servo_id), profile_acceleration)
        self._set_speed(int(servo_id), profile_speed)
        self._write_raw_position(int(servo_id), int(self.goal_positions_raw[index]))
        time.sleep(0.02)
        self._set_torque(int(servo_id), True)
        self.armed[index] = bool(self.torque_enabled[index])
        return self.armed[index]

    def arm_servos(self, servo_ids: list[int]) -> list[int]:
        """Arm the requested servos sequentially."""
        armed: list[int] = []
        for servo_id in servo_ids:
            if self.arm_servo(int(servo_id)):
                armed.append(int(servo_id))
                time.sleep(0.02)
        return armed

    def arm_all(self) -> list[int]:
        return self.arm_servos(self.available_servo_ids())

    def disable_torque(self, servo_id: int) -> None:
        """Disable torque on one servo."""
        self._set_torque(int(servo_id), False)
        index = self._servo_index(servo_id)
        self.armed[index] = False

    def enable_torque(self, servo_id: int) -> None:
        """Compatibility helper; explicit arm is preferred."""
        self.arm_servo(int(servo_id))

    def disarm_all(self) -> None:
        """Disable torque on all live servos."""
        for servo_id in self.available_servo_ids():
            self._set_torque(int(servo_id), False)
        self.armed = [False for _ in self.servo_ids]

    def write_positions(self, positions: list[int]) -> None:
        """Write target positions in command-space to armed servos."""
        sync_payload: list[int] = []
        sync_servo_ids: list[int] = []
        for index, servo_id in enumerate(self.servo_ids):
            if not self.available[index] or not self.armed[index]:
                continue
            command_value = self.calibrations[index].clamp_command(int(positions[index]))
            raw_value = self.calibrations[index].command_to_raw(command_value)
            self.goal_positions_raw[index] = int(raw_value)
            if self.use_sync_write:
                sync_payload.extend(
                    [int(servo_id), int(raw_value) & 0xFF, (int(raw_value) >> 8) & 0xFF]
                )
                sync_servo_ids.append(int(servo_id))
            else:
                self._write_raw_position(int(servo_id), int(raw_value))

        if not self.use_sync_write or not sync_servo_ids:
            return
        if len(sync_servo_ids) == 1:
            only_id = int(sync_servo_ids[0])
            self._write_raw_position(only_id, int(self.goal_positions_raw[self._servo_index(only_id)]))
            return
        if self._sync_write_raw_positions(sync_payload):
            return
        for servo_id in sync_servo_ids:
            index = self._servo_index(int(servo_id))
            self._write_raw_position(int(servo_id), int(self.goal_positions_raw[index]))

    def step_towards(self, targets: list[int], alpha: float, max_step: int | None = None) -> list[int]:
        """Read current command positions, smooth towards targets, write, then read back."""
        current_positions = self.read_positions()
        next_positions: list[int] = []
        for index, target in enumerate(targets):
            current = int(current_positions[index])
            if not self.armed[index]:
                next_positions.append(current)
                continue
            smoothed = int(round(ema_step(float(current), float(target), alpha)))
            if max_step is not None and max_step > 0:
                smoothed = int(clamp(smoothed, current - int(max_step), current + int(max_step)))
            next_positions.append(self.calibrations[index].clamp_command(smoothed))
        self.write_positions(next_positions)
        return self.read_positions()

    def _configure_port(self) -> None:
        try:
            self.dev.portHandler.closePort()
        except Exception:
            pass
        self.dev.portHandler.baudrate = self.baudrate
        self.dev.portHandler.setupPort()

    def _ping(self, servo_id: int) -> bool:
        try:
            model, comm, err = self.dev.ping(int(servo_id))
        except Exception:
            return False
        return comm == COMM_SUCCESS and err == 0 and model is not None

    def _log_servo_state(self, servo_id: int) -> None:
        current_mode = self._read_mode(int(servo_id))
        voltage = self._read_voltage(int(servo_id))
        temperature = self._read_temperature(int(servo_id))
        torque_state: str
        try:
            value, comm, err = self.dev.read1ByteTxRx(int(servo_id), STS_TORQUE_ENABLE)
            if comm == COMM_SUCCESS and err == 0:
                torque_state = str(int(value))
            else:
                torque_state = f"comm={comm} err={err} flags={self._decode_error_flags(err)}"
            self.torque_enabled[self._servo_index(servo_id)] = torque_state == "1"
        except Exception as exc:
            torque_state = f"ERR:{exc}"
        self.logger.info(
            f"Servo {servo_id} on {self.port}: mode={current_mode} torque={torque_state} "
            f"voltage={voltage} temp={temperature} (startup is connect-only)"
        )

    def _read_mode(self, servo_id: int) -> int | None:
        try:
            return self.dev.ReadMode(int(servo_id))
        except Exception as exc:
            self.logger.warning(f"Failed to read mode for servo {servo_id} on {self.port}: {exc}")
            return None

    def _read_voltage(self, servo_id: int) -> float | None:
        try:
            return self.dev.ReadVoltage(int(servo_id))
        except Exception:
            return None

    def _read_temperature(self, servo_id: int) -> int | None:
        if not hasattr(self.dev, "ReadTemperature"):
            return None
        try:
            return self.dev.ReadTemperature(int(servo_id))
        except Exception:
            return None

    def _read_position(self, servo_id: int, retries: int = 4) -> int | None:
        for _ in range(max(1, retries)):
            try:
                position = self.dev.ReadPosition(int(servo_id))
            except Exception:
                position = None
            if position is not None:
                self._mark_comm_success()
                return int(position)
            time.sleep(0.01)
        self._mark_comm_failure()
        self._throttle_warn(
            f"pos_read_{servo_id}",
            f"Position read failed for servo {servo_id} on {self.port}",
        )
        return None

    def _read_load(self, servo_id: int, retries: int = 3) -> float | None:
        for _ in range(max(1, retries)):
            try:
                load_raw, comm, err = self.dev.read2ByteTxRx(int(servo_id), STS_PRESENT_LOAD_L)
            except Exception:
                load_raw = None
                comm = -1
                err = 0
            if load_raw is not None and comm == COMM_SUCCESS and err == 0:
                magnitude = int(load_raw) & 0x03FF
                self._mark_comm_success()
                return float(clamp(float(magnitude) / 1000.0, 0.0, 1.0))
            time.sleep(0.01)
        self._mark_comm_failure()
        self._throttle_warn(
            f"load_read_{servo_id}",
            f"Load read failed for servo {servo_id} on {self.port}",
        )
        return None

    def _write_raw_position(self, servo_id: int, position: int) -> bool:
        try:
            comm, err = self.dev.write2ByteTxRx(int(servo_id), STS_GOAL_POSITION_L, int(position))
        except Exception as exc:
            self._mark_comm_failure()
            self._throttle_warn(
                f"pos_write_exc_{servo_id}",
                f"Position write failed for servo {servo_id} on {self.port}: {exc}",
            )
            return False
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            self._throttle_warn(
                f"pos_write_rej_{servo_id}",
                f"Position write rejected for servo {servo_id} on {self.port}: "
                f"comm={comm} err={err} flags={self._decode_error_flags(err)}",
            )
            return False
        self._mark_comm_success()
        return True

    def _set_mode_position(self, servo_id: int) -> None:
        try:
            comm, err = self.dev.write1ByteTxRx(int(servo_id), STS_MODE, 0)
        except Exception as exc:
            self.logger.warning(f"Mode switch failed for servo {servo_id} on {self.port}: {exc}")
            self._mark_comm_failure()
            return
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            self.logger.warning(
                f"Mode switch rejected for servo {servo_id} on {self.port}: "
                f"comm={comm} err={err} flags={self._decode_error_flags(err)}"
            )
            return
        self._mark_comm_success()
        self.logger.info(f"Set servo {servo_id} on {self.port} to position mode")

    def _set_acceleration(self, servo_id: int, acceleration: int) -> None:
        try:
            result = self.dev.SetAcceleration(int(servo_id), int(acceleration))
        except Exception as exc:
            self.logger.warning(
                f"Acceleration update failed for servo {servo_id} on {self.port}: {exc}"
            )
            self._mark_comm_failure()
            return
        if result is None:
            self._mark_comm_failure()
            self.logger.warning(
                f"Acceleration update rejected for servo {servo_id} on {self.port}: acc={acceleration}"
            )
            return
        self._mark_comm_success()

    def _set_speed(self, servo_id: int, speed: int) -> None:
        try:
            result = self.dev.SetSpeed(int(servo_id), int(speed))
        except Exception as exc:
            self.logger.warning(f"Speed update failed for servo {servo_id} on {self.port}: {exc}")
            self._mark_comm_failure()
            return
        if result is None:
            self._mark_comm_failure()
            self.logger.warning(
                f"Speed update rejected for servo {servo_id} on {self.port}: speed={speed}"
            )
            return
        self._mark_comm_success()

    def _set_torque(self, servo_id: int, enabled: bool) -> None:
        try:
            comm, err = self.dev.write1ByteTxRx(int(servo_id), STS_TORQUE_ENABLE, 1 if enabled else 0)
        except Exception as exc:
            self.logger.warning(f"Torque update failed for servo {servo_id} on {self.port}: {exc}")
            self._mark_comm_failure()
            return
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            self.logger.warning(
                f"Torque update rejected for servo {servo_id} on {self.port}: "
                f"comm={comm} err={err} flags={self._decode_error_flags(err)}"
            )
            return
        index = self._servo_index(int(servo_id))
        self.torque_enabled[index] = bool(enabled)
        self._mark_comm_success()

    def _write_register_1byte(self, servo_id: int, address: int, value: int, label: str) -> bool:
        try:
            comm, err = self.dev.write1ByteTxRx(int(servo_id), int(address), int(value) & 0xFF)
        except Exception as exc:
            self.logger.warning(f"{label} write failed for servo {servo_id} on {self.port}: {exc}")
            self._mark_comm_failure()
            return False
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            self.logger.warning(
                f"{label} write rejected for servo {servo_id} on {self.port}: "
                f"comm={comm} err={err} flags={self._decode_error_flags(err)}"
            )
            return False
        self._mark_comm_success()
        return True

    def _write_register_2byte(self, servo_id: int, address: int, value: int, label: str) -> bool:
        try:
            comm, err = self.dev.write2ByteTxRx(int(servo_id), int(address), int(value) & 0xFFFF)
        except Exception as exc:
            self.logger.warning(f"{label} write failed for servo {servo_id} on {self.port}: {exc}")
            self._mark_comm_failure()
            return False
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            self.logger.warning(
                f"{label} write rejected for servo {servo_id} on {self.port}: "
                f"comm={comm} err={err} flags={self._decode_error_flags(err)}"
            )
            return False
        self._mark_comm_success()
        return True

    def _apply_register_profile(self, servo_id: int, index: int) -> None:
        # Profile writes are more reliable with torque disabled.
        self._set_torque(int(servo_id), False)
        if self.compliance_margin > 0:
            self._write_register_1byte(
                int(servo_id),
                STS_CW_DEAD,
                int(self.compliance_margin),
                "CW compliance margin",
            )
            self._write_register_1byte(
                int(servo_id),
                STS_CCW_DEAD,
                int(self.compliance_margin),
                "CCW compliance margin",
            )
        self._write_register_1byte(
            int(servo_id),
            STS_P_COEFFICIENT,
            int(self.pid_p_per_servo[index]),
            "P coefficient",
        )
        self._write_register_1byte(
            int(servo_id),
            STS_I_COEFFICIENT,
            int(self.pid_i_per_servo[index]),
            "I coefficient",
        )
        self._write_register_1byte(
            int(servo_id),
            STS_D_COEFFICIENT,
            int(self.pid_d_per_servo[index]),
            "D coefficient",
        )
        self._write_register_2byte(
            int(servo_id),
            STS_MAX_TORQUE_LIMIT_L,
            int(self.max_torque_limit_per_servo[index]),
            "Max torque limit",
        )
        self._write_register_2byte(
            int(servo_id),
            STS_PROTECTION_CURRENT_L,
            int(self.protection_current_per_servo[index]),
            "Protection current",
        )
        self._write_register_1byte(
            int(servo_id),
            STS_OVERLOAD_TORQUE,
            int(self.overload_torque_per_servo[index]),
            "Overload torque",
        )
        self._write_register_1byte(
            int(servo_id),
            STS_PROTECTION_TIME,
            int(self.protection_time_per_servo[index]),
            "Protection time",
        )

    def _sync_write_raw_positions(self, payload: list[int]) -> bool:
        try:
            comm = self.dev.syncWriteTxOnly(
                STS_GOAL_POSITION_L,
                2,
                payload,
                len(payload),
            )
        except Exception as exc:
            self._mark_comm_failure()
            self._throttle_warn("sync_write_exc", f"Sync position write failed on {self.port}: {exc}")
            return False
        if comm != COMM_SUCCESS:
            self._mark_comm_failure()
            self._throttle_warn("sync_write_rej", f"Sync position write rejected on {self.port}: comm={comm}")
            return False
        self._mark_comm_success()
        return True

    def _poll_diagnostics(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force:
            elapsed = now - self._last_diag_poll
            if elapsed >= 0.0 and elapsed < self.diag_poll_period_sec:
                return

        next_loads = list(self.last_loads)
        next_voltages = list(self.last_voltages)
        next_currents = list(self.last_currents)
        next_temperatures = list(self.last_temperatures)
        next_overload = list(self.overload_flags)
        next_voltage_fault = list(self.voltage_fault_flags)

        for index, servo_id in enumerate(self.servo_ids):
            if not self.available[index]:
                next_loads[index] = 0.0
                next_voltages[index] = 0.0
                next_currents[index] = 0.0
                next_temperatures[index] = 0.0
                next_overload[index] = False
                next_voltage_fault[index] = False
                continue

            status, _, err = self._read_status_reg(int(servo_id))
            if status is not None:
                next_overload[index] = bool(int(status) & ERRBIT_OVERLOAD)
                next_voltage_fault[index] = bool(int(status) & ERRBIT_VOLTAGE)
            elif err is not None:
                next_overload[index] = bool(int(err) & ERRBIT_OVERLOAD)
                next_voltage_fault[index] = bool(int(err) & ERRBIT_VOLTAGE)

            voltage = self._read_voltage_reg(int(servo_id))
            if voltage is not None:
                next_voltages[index] = float(voltage)

            current = self._read_current_reg(int(servo_id))
            if current is not None:
                next_currents[index] = float(current)

            load = self._read_load(int(servo_id), retries=1)
            if load is not None:
                next_loads[index] = float(load)

            temperature = self._read_temperature_reg(int(servo_id))
            if temperature is not None:
                next_temperatures[index] = float(temperature)

        self.last_loads = next_loads
        self.last_voltages = next_voltages
        self.last_currents = next_currents
        self.last_temperatures = next_temperatures
        self.overload_flags = next_overload
        self.voltage_fault_flags = next_voltage_fault
        self._last_diag_poll = now

    def _read_status_reg(self, servo_id: int) -> tuple[int | None, int | None, int | None]:
        try:
            value, comm, err = self.dev.read1ByteTxRx(int(servo_id), STS_STATUS)
        except Exception:
            self._mark_comm_failure()
            return None, None, None
        if comm == COMM_SUCCESS and err == 0:
            self._mark_comm_success()
            return int(value), int(comm), int(err)
        self._mark_comm_failure()
        return None, int(comm), int(err)

    def _read_voltage_reg(self, servo_id: int) -> float | None:
        try:
            value, comm, err = self.dev.read1ByteTxRx(int(servo_id), STS_PRESENT_VOLTAGE)
        except Exception:
            self._mark_comm_failure()
            return None
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            return None
        self._mark_comm_success()
        return float(value) * 0.1

    def _read_temperature_reg(self, servo_id: int) -> float | None:
        try:
            value, comm, err = self.dev.read1ByteTxRx(int(servo_id), STS_PRESENT_TEMPERATURE)
        except Exception:
            self._mark_comm_failure()
            return None
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            return None
        self._mark_comm_success()
        return float(int(value))

    def _read_current_reg(self, servo_id: int) -> float | None:
        try:
            value, comm, err = self.dev.read2ByteTxRx(int(servo_id), STS_PRESENT_CURRENT_L)
        except Exception:
            self._mark_comm_failure()
            return None
        if comm != COMM_SUCCESS or err != 0:
            self._mark_comm_failure()
            return None
        self._mark_comm_success()
        return float(int(value) & 0x03FF) * 6.5

    _WARN_THROTTLE_SEC = 5.0  # repeated serial-failure warnings suppressed within this window

    def _throttle_warn(self, key: str, message: str) -> None:
        """Emit a warning at most once per _WARN_THROTTLE_SEC for the given key."""
        now = time.monotonic()
        if now - self._warn_times.get(key, 0.0) >= self._WARN_THROTTLE_SEC:
            self._warn_times[key] = now
            self.logger.warning(message)

    def _mark_comm_success(self) -> None:
        self._comm_last_success = time.monotonic()
        self._comm_recent_failures = max(0, self._comm_recent_failures - 1)
        self._warn_times.clear()  # reset throttle so next failure is logged immediately

    def _mark_comm_failure(self) -> None:
        self._comm_recent_failures = min(1000, self._comm_recent_failures + 1)

    def _normalize_profile(
        self,
        values: list[int] | None,
        maximum: int,
        fallback: int,
    ) -> list[int]:
        if values is None:
            return [int(fallback) for _ in self.servo_ids]
        normalized = [int(clamp(int(value), 0, int(maximum))) for value in list(values)]
        if len(normalized) < len(self.servo_ids):
            normalized.extend([int(fallback)] * (len(self.servo_ids) - len(normalized)))
        if len(normalized) > len(self.servo_ids):
            normalized = normalized[: len(self.servo_ids)]
        return normalized

    def _servo_index(self, servo_id: int) -> int:
        return self.servo_ids.index(int(servo_id))

    def _decode_error_flags(self, err: int) -> str:
        flags: list[str] = []
        if err & ERRBIT_VOLTAGE:
            flags.append("voltage")
        if err & ERRBIT_ANGLE:
            flags.append("angle")
        if err & ERRBIT_OVERHEAT:
            flags.append("overheat")
        if err & ERRBIT_OVERELE:
            flags.append("overele")
        if err & ERRBIT_OVERLOAD:
            flags.append("overload")
        return ",".join(flags) if flags else "none"
