"""Pure control logic for the arm controller.

State machine:
  IDLE        - torque off, waiting for deadman=True
  ACTIVE      - deadman=True, tracking target positions with EMA smoothing
  HOMING      - deadman released or watchdog expired; EMA toward home, then release torque
  SAFE_MODE   - fault (jam or overheat); moves to home, releases torque, stays locked
                until explicitly reset via reset_safe_mode()

Seam-point note:
  Raw encoder range is 0-4095 (one full revolution). The seam at 0/4095 is handled by
  servo_calibration.py using center_raw offsets so the working range never crosses the
  raw boundary. All control logic operates in virtual command-space (centered at 2048)
  which is continuous within the calibrated working range.
"""

from __future__ import annotations

import enum
import os
import time
from dataclasses import dataclass
from typing import Any

from rovr_common.clamp import clamp_int, clamp_sequence
from rovr_common.command_models import ArmTargetCommand, ArmTelemetryData
from rovr_common.servo_calibration import load_servo_calibrations
from rovr_common.watchdog import TimeoutWatchdog


class ArmState(enum.Enum):
    IDLE = "idle"
    ACTIVE = "active"
    HOMING = "homing"
    SAFE_MODE = "safe_mode"


@dataclass(frozen=True)
class ArmTickResult:
    telemetry: ArmTelemetryData
    state: str


def resolve_serial_port(primary_path: str, fallback_path: str) -> str:
    """Prefer the primary serial path and fall back only if needed."""
    if primary_path and os.path.exists(primary_path):
        return primary_path
    if fallback_path and os.path.exists(fallback_path):
        return fallback_path
    return primary_path or fallback_path


def load_servo_limits(path_str: str, servo_ids: list[int]) -> tuple[list[tuple[int, int]], list[int]]:
    """Load min/max ranges and home positions from a JSON config."""
    calibrations = load_servo_calibrations(path_str, servo_ids)
    ranges: list[tuple[int, int]] = []
    homes: list[int] = []
    for item in calibrations:
        ranges.append((int(item.minimum), int(item.maximum)))
        homes.append(int(item.home))
    return ranges, homes


class ArmCore:
    """Arm control state machine with safety guards and smooth trajectory execution.

    Safety features:
    - Temperature guard: any servo > temp_limit_celsius -> SAFE_MODE (whole arm)
    - Jam detection: high load + commanded motion + no movement for jam_hold_time_sec
      -> SAFE_MODE for the whole arm
    - Overload guard: hardware overload flag suppresses motion briefly
    - Load derate: reduces max_step_per_tick as load rises to protect motors
    - Network watchdog: no commands for heartbeat_timeout_sec -> HOMING
    - Deadband: no serial writes when servo is within position_deadband of target,
      preventing MOS-FET switching and resistive heating
    - Compliance margin: written to CW/CCW deadband HW registers at arm time (bus layer)
    - Home-and-release: on HOMING/SAFE_MODE arm approaches home then releases torque
    """

    def __init__(
        self,
        servo_ids: list[int],
        limits: list[tuple[int, int]],
        home_positions: list[int],
        bus: Any,
        heartbeat_timeout_sec: float = 1.0,
        control_rate_hz: float = 50.0,
        ema_alpha: float = 0.05,
        max_step_per_tick: int = 10,
        position_deadband: int = 2,
        home_deadband: int = 8,
        home_release_torque: bool = True,
        temp_limit_celsius: float = 60.0,
        jam_load_threshold: float = 0.97,
        jam_hold_time_sec: float = 3.0,
        jam_target_delta_threshold: int = 96,
        jam_no_motion_epsilon: int = 2,
        overload_guard_enabled: bool = True,
        overload_guard_hold_time_sec: float = 0.35,
        overload_target_delta_threshold: int = 64,
        load_derate_start: float = 0.65,
        load_derate_end: float = 0.90,
        load_min_step: int = 6,
    ) -> None:
        self.servo_ids = list(servo_ids)
        self.n = len(servo_ids)
        self.limits = list(limits)
        self.home_positions = list(home_positions)
        self.bus = bus

        self.heartbeat = TimeoutWatchdog(float(heartbeat_timeout_sec))
        self.ema_alpha = float(ema_alpha)
        self.max_step_per_tick = int(max(1, max_step_per_tick))
        self.position_deadband = int(max(0, position_deadband))
        self.home_deadband = int(max(1, home_deadband))
        self.home_release_torque = bool(home_release_torque)
        self.temp_limit_celsius = float(temp_limit_celsius)

        self.jam_load_threshold = float(jam_load_threshold)
        self.jam_hold_time_sec = float(jam_hold_time_sec)
        self.jam_target_delta_threshold = int(max(1, jam_target_delta_threshold))
        self.jam_no_motion_epsilon = int(max(0, jam_no_motion_epsilon))

        self.overload_guard_enabled = bool(overload_guard_enabled)
        self.overload_guard_hold_time_sec = float(max(0.0, overload_guard_hold_time_sec))
        self.overload_target_delta_threshold = int(max(1, overload_target_delta_threshold))

        self.load_derate_start = float(max(0.0, load_derate_start))
        self.load_derate_end = float(max(self.load_derate_start + 0.01, load_derate_end))
        self.load_min_step = int(max(1, load_min_step))

        self._state = ArmState.IDLE
        self._safe_mode_reason = ""

        # Command/feedback positions in virtual command-space
        self._current_commands = list(self.home_positions)
        self._target_positions = list(self.home_positions)
        self._last_feedback_positions = list(self.home_positions)

        # Per-servo deadband reached flags
        self._servo_reached = [False] * self.n

        # Cached telemetry fields
        self._cached_loads = [0.0] * self.n
        self._cached_temperatures = [0.0] * self.n
        self._cached_torque_states = [False] * self.n

        # Jam tracking
        self._jam_since: list[float | None] = [None] * self.n
        self._jam_flags = [False] * self.n

        # Overload guard
        self._overload_guard_until = [0.0] * self.n
        self._last_overload_flags = [False] * self.n
        self._cached_overload_flags = [False] * self.n

        # Diagnostic poll cadence: every ~4 control ticks
        self._last_diag_poll = 0.0
        self._diag_poll_period = max(0.05, 4.0 / max(1.0, float(control_rate_hz)))

    # -------------------------------------------------------------------------
    # Public interface
    # -------------------------------------------------------------------------

    @property
    def state(self) -> ArmState:
        return self._state

    def apply_command(self, command: ArmTargetCommand, now: float | None = None) -> None:
        """Ingest a command arriving from the ROS topic."""
        t = time.monotonic() if now is None else float(now)
        self.heartbeat.mark(t)

        if command.deadman_switch:
            # SAFE_MODE is sticky - operator must call reset_safe_mode() before moving again
            if self._state == ArmState.SAFE_MODE:
                return
            if self._state == ArmState.IDLE:
                self._enter_active()
            elif self._state == ArmState.HOMING:
                # Operator pressed deadman again during homing - cancel homing, resume ACTIVE
                self._state = ArmState.ACTIVE
                self._reset_servo_reached_flags()
            if self._state == ArmState.ACTIVE:
                new_targets = self._clamp_positions(list(command.servos))
                for i, new_target in enumerate(new_targets):
                    if abs(new_target - self._target_positions[i]) > self.position_deadband:
                        self._servo_reached[i] = False
                self._target_positions = new_targets
        else:
            # deadman=False: return to home
            if self._state == ArmState.ACTIVE:
                self._enter_homing()

    def tick(self, now: float | None = None) -> ArmTickResult:
        """Run one control cycle. Must be called at control_rate_hz."""
        t = time.monotonic() if now is None else float(now)

        # Network watchdog: packet silence -> go home
        if self.heartbeat.expired(t) and self._state == ArmState.ACTIVE:
            self._enter_homing()

        # Refresh diagnostics (loads, temperatures, overload flags, torque states)
        self._refresh_diagnostics(t)
        self._update_overload_guard(t)

        # Temperature safety check (ignore zero readings from unavailable sensors)
        if self._state in (ArmState.ACTIVE, ArmState.HOMING):
            for i, temp in enumerate(self._cached_temperatures):
                if float(temp) > self.temp_limit_celsius and float(temp) > 0.0:
                    self._enter_safe_mode(
                        f"servo {self.servo_ids[i]} overheated: {temp:.1f}C > {self.temp_limit_celsius}C"
                    )
                    break

        # Execute state behaviour
        if self._state == ArmState.ACTIVE:
            self._tick_active(t)
        elif self._state in (ArmState.HOMING, ArmState.SAFE_MODE):
            self._tick_homing(t)
        # ArmState.IDLE: torque is off, nothing to do

        # Read back feedback positions from servos
        feedback = self._read_feedback_positions()

        # Jam detection only when actively tracking targets
        if self._state == ArmState.ACTIVE:
            self._update_jam_flags(feedback, t)
            if any(self._jam_flags):
                jammed_ids = [self.servo_ids[i] for i, f in enumerate(self._jam_flags) if f]
                self._enter_safe_mode(f"jam detected on servo(s): {jammed_ids}")

        # Assemble telemetry
        comm_ok = self._check_comm()
        status = self._build_status(t, comm_ok)

        telemetry = ArmTelemetryData(
            timestamp=t,
            positions=tuple(int(v) for v in feedback),
            loads=tuple(float(v) for v in self._cached_loads),
            temperatures=tuple(float(v) for v in self._cached_temperatures),
            jam_detected=tuple(self._jam_flags),
            torque_enabled=tuple(bool(v) for v in self._cached_torque_states),
            status=status,
        )
        self._last_feedback_positions = list(feedback)
        return ArmTickResult(telemetry=telemetry, state=self._state.value)

    def arm_all(self) -> list[int]:
        """Explicitly arm all servos (also called internally on ACTIVE entry)."""
        if hasattr(self.bus, "arm_all"):
            return [int(v) for v in self.bus.arm_all()]
        return []

    def arm_servo(self, servo_id: int) -> bool:
        if hasattr(self.bus, "arm_servo"):
            return bool(self.bus.arm_servo(int(servo_id)))
        return False

    def disarm_all(self) -> None:
        """Force disarm all servos and go to IDLE."""
        if hasattr(self.bus, "disarm_all"):
            self.bus.disarm_all()
        self._state = ArmState.IDLE
        self._jam_flags = [False] * self.n
        self._jam_since = [None] * self.n

    def reset_safe_mode(self) -> None:
        """Clear safe-mode lock and jam flags. Arm returns to IDLE.
        Operator must send deadman=True to resume motion."""
        if hasattr(self.bus, "disarm_all"):
            self.bus.disarm_all()
        self._state = ArmState.IDLE
        self._jam_flags = [False] * self.n
        self._jam_since = [None] * self.n
        self._safe_mode_reason = ""

    # -------------------------------------------------------------------------
    # State transitions
    # -------------------------------------------------------------------------

    def _enter_active(self) -> None:
        self.arm_all()
        # Sync current commands to live feedback to avoid startup position jump
        current = self._read_feedback_positions()
        self._current_commands = list(current)
        self._target_positions = list(current)
        self._reset_servo_reached_flags()
        self._jam_flags = [False] * self.n
        self._jam_since = [None] * self.n
        self._state = ArmState.ACTIVE

    def _enter_homing(self) -> None:
        self._target_positions = list(self.home_positions)
        self._reset_servo_reached_flags()
        self._state = ArmState.HOMING

    def _enter_safe_mode(self, reason: str) -> None:
        self._safe_mode_reason = reason
        self._target_positions = list(self.home_positions)
        self._reset_servo_reached_flags()
        self._state = ArmState.SAFE_MODE
        # Cut torque immediately on any safety fault (jam or overheat)
        if hasattr(self.bus, "disarm_all"):
            self.bus.disarm_all()
        self._cached_torque_states = [False] * self.n

    # -------------------------------------------------------------------------
    # Per-state tick logic
    # -------------------------------------------------------------------------

    def _tick_active(self, t: float) -> None:
        """EMA step toward target. Skip serial write for servos within deadband."""
        next_commands: list[int] = []
        needs_write = False

        for i, target in enumerate(self._target_positions):
            current_cmd = int(self._current_commands[i])

            # Already within deadband: hardware PID holds position, no serial writes
            if self._servo_reached[i]:
                next_commands.append(current_cmd)
                continue

            # Overload guard: suppress large moves temporarily
            if (
                self.overload_guard_enabled
                and self._overload_guard_active(i, t)
                and abs(target - current_cmd) >= self.overload_target_delta_threshold
            ):
                next_commands.append(current_cmd)
                continue

            # EMA step with velocity clamp (prevents startup jerk on large target jumps)
            raw_step = (float(target) - float(current_cmd)) * self.ema_alpha
            max_s = float(self._compute_step_limit(i))
            step = max(-max_s, min(max_s, raw_step))

            next_cmd = int(round(float(current_cmd) + step))
            lower, upper = self.limits[i]
            next_cmd = clamp_int(next_cmd, lower, upper)

            # Snap and mark reached when within deadband
            if abs(target - next_cmd) <= self.position_deadband:
                next_cmd = target
                self._servo_reached[i] = True

            next_commands.append(next_cmd)
            needs_write = True

        self._current_commands = next_commands
        if needs_write:
            self._write_positions(next_commands)

    def _tick_homing(self, t: float) -> None:
        """EMA step toward home positions. Release torque once all joints arrive."""
        next_commands: list[int] = []
        needs_write = False

        for i, home in enumerate(self.home_positions):
            current_cmd = int(self._current_commands[i])

            if self._servo_reached[i]:
                next_commands.append(current_cmd)
                continue

            raw_step = (float(home) - float(current_cmd)) * self.ema_alpha
            max_s = float(self.max_step_per_tick)
            step = max(-max_s, min(max_s, raw_step))

            next_cmd = int(round(float(current_cmd) + step))
            lower, upper = self.limits[i]
            next_cmd = clamp_int(next_cmd, lower, upper)

            if abs(home - next_cmd) <= self.home_deadband:
                next_cmd = home
                self._servo_reached[i] = True

            next_commands.append(next_cmd)
            needs_write = True

        self._current_commands = next_commands
        if needs_write:
            self._write_positions(next_commands)

        # Check if arm has physically arrived at home (use feedback, not commands)
        all_home = all(
            abs(int(self._last_feedback_positions[i]) - int(self.home_positions[i])) <= self.home_deadband
            for i in range(self.n)
        )
        if all_home:
            if self.home_release_torque and hasattr(self.bus, "disarm_all"):
                self.bus.disarm_all()
            # SAFE_MODE stays locked; only HOMING transitions to IDLE
            if self._state == ArmState.HOMING:
                self._state = ArmState.IDLE

    # -------------------------------------------------------------------------
    # Jam detection
    # -------------------------------------------------------------------------

    def _update_jam_flags(self, feedback: list[int], t: float) -> None:
        for i in range(self.n):
            if self._jam_flags[i]:
                continue

            # Skip transient overload events (not a sustained jam)
            if self.overload_guard_enabled and self._overload_guard_active(i, t):
                self._jam_since[i] = None
                continue

            target = self._target_positions[i]
            current = feedback[i]
            last = self._last_feedback_positions[i]

            no_motion = abs(current - last) <= self.jam_no_motion_epsilon
            high_load = float(self._cached_loads[i]) > self.jam_load_threshold
            big_delta = abs(target - current) >= self.jam_target_delta_threshold

            # Not a jam if servo is legitimately pressed against a physical limit
            lower, upper = self.limits[i]
            margin = self.jam_target_delta_threshold
            at_limit = (
                target <= lower + margin and current <= lower + margin
            ) or (
                target >= upper - margin and current >= upper - margin
            )
            if at_limit:
                self._jam_since[i] = None
                continue

            if big_delta and high_load and no_motion and self._cached_torque_states[i]:
                if self._jam_since[i] is None:
                    self._jam_since[i] = t
                elif (t - self._jam_since[i]) >= self.jam_hold_time_sec:
                    self._jam_flags[i] = True
            else:
                self._jam_since[i] = None

    # -------------------------------------------------------------------------
    # Overload guard
    # -------------------------------------------------------------------------

    def _update_overload_guard(self, t: float) -> None:
        if not self.overload_guard_enabled:
            return
        for i in range(self.n):
            overloaded = (
                bool(self._cached_overload_flags[i])
                if i < len(self._cached_overload_flags)
                else False
            )
            if overloaded and not self._last_overload_flags[i]:
                self._overload_guard_until[i] = t + self.overload_guard_hold_time_sec
            self._last_overload_flags[i] = overloaded

    def _overload_guard_active(self, index: int, t: float) -> bool:
        if not self.overload_guard_enabled:
            return False
        return t < self._overload_guard_until[index]

    # -------------------------------------------------------------------------
    # Diagnostics / feedback
    # -------------------------------------------------------------------------

    def _refresh_diagnostics(self, t: float) -> None:
        if (t - self._last_diag_poll) < self._diag_poll_period:
            return
        self._last_diag_poll = t

        if hasattr(self.bus, "read_loads"):
            try:
                vals = list(self.bus.read_loads())
                self._cached_loads = (vals + [0.0] * self.n)[: self.n]
            except Exception:
                pass

        if hasattr(self.bus, "read_temperatures"):
            try:
                vals = list(self.bus.read_temperatures())
                self._cached_temperatures = (vals + [0.0] * self.n)[: self.n]
            except Exception:
                pass

        if hasattr(self.bus, "get_overload_flags"):
            try:
                vals = [bool(v) for v in self.bus.get_overload_flags()]
                self._cached_overload_flags = (vals + [False] * self.n)[: self.n]
            except Exception:
                pass

        # Prefer cached torque states to avoid extra serial reads every tick
        if hasattr(self.bus, "get_cached_torque_enabled"):
            try:
                vals = [bool(v) for v in self.bus.get_cached_torque_enabled()]
                self._cached_torque_states = (vals + [False] * self.n)[: self.n]
            except Exception:
                pass

    def _read_feedback_positions(self) -> list[int]:
        if hasattr(self.bus, "read_positions"):
            try:
                positions = [int(v) for v in self.bus.read_positions()]
                if len(positions) >= self.n:
                    return positions[: self.n]
            except Exception:
                pass
        return list(self._last_feedback_positions)

    def _check_comm(self) -> bool:
        if hasattr(self.bus, "comm_link_ok"):
            try:
                return bool(self.bus.comm_link_ok())
            except Exception:
                return False
        return True

    def _build_status(self, t: float, comm_ok: bool) -> str:
        if self._state == ArmState.SAFE_MODE:
            snippet = self._safe_mode_reason[:60] if self._safe_mode_reason else ""
            return f"safe_mode: {snippet}" if snippet else "safe_mode"
        if not comm_ok:
            return "comm_timeout"
        if self._state == ArmState.IDLE:
            return "idle"
        if self._state == ArmState.HOMING:
            return "homing"
        # ArmState.ACTIVE
        if any(self._overload_guard_active(i, t) for i in range(self.n)):
            return "overload_guard"
        return "ok"

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _compute_step_limit(self, index: int) -> int:
        base = self.max_step_per_tick
        load = float(self._cached_loads[index]) if index < len(self._cached_loads) else 0.0
        if load <= self.load_derate_start:
            return base
        if load >= self.load_derate_end:
            return self.load_min_step
        span = self.load_derate_end - self.load_derate_start
        ratio = (load - self.load_derate_start) / span
        return max(self.load_min_step, int(round(base - ratio * (base - self.load_min_step))))

    def _clamp_positions(self, positions: list[int]) -> list[int]:
        return clamp_sequence(positions, self.limits)

    def _reset_servo_reached_flags(self) -> None:
        self._servo_reached = [False] * self.n

    def _write_positions(self, positions: list[int]) -> None:
        if hasattr(self.bus, "write_positions"):
            try:
                self.bus.write_positions(list(positions))
            except Exception:
                pass
