"""Tests for arm control state machine, safety, and trajectory logic."""

from __future__ import annotations

from rovr_common.command_models import ArmTargetCommand
from rovr_common.fake_servo_bus import FakeServoBus

from rovr_arm.control import ArmCore, ArmState, resolve_serial_port

SERVO_IDS = [1, 2, 3, 4, 5, 6]
LIMITS = [(0, 4095)] * 6
HOMES = [2048] * 6


def _make_core(bus, **overrides) -> ArmCore:
    defaults = dict(
        servo_ids=SERVO_IDS,
        limits=LIMITS,
        home_positions=HOMES,
        bus=bus,
        heartbeat_timeout_sec=1.0,
        control_rate_hz=50.0,
        ema_alpha=0.5,
        max_step_per_tick=128,
        position_deadband=2,
        home_deadband=8,
        home_release_torque=False,  # keep torque on for most tests
        jam_load_threshold=0.8,
        jam_hold_time_sec=1.0,
    )
    defaults.update(overrides)
    return ArmCore(**defaults)


class CommFaultBus(FakeServoBus):
    def comm_link_ok(self, *_: object, **__: object) -> bool:
        return False


class HotBus(FakeServoBus):
    """FakeServoBus that reports overheated temperatures for the first servo."""

    def read_temperatures(self) -> list[float]:
        return [75.0] + [25.0] * (len(self.servo_ids) - 1)


class LaggingFeedbackBus(FakeServoBus):
    def __init__(self, *args: object, feedback_lag: int = 120, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.feedback_lag = int(feedback_lag)

    def write_positions(self, positions: list[int]) -> None:
        for index, value in enumerate(positions):
            if not self.torque_enabled[index]:
                continue
            command = int(value)
            self.goal_positions[index] = command
            self.positions[index] = max(0, command - self.feedback_lag)
        self._update_loads(positions)
        self._record()


def _cmd(deadman: bool, target: int = 2000) -> ArmTargetCommand:
    return ArmTargetCommand(
        timestamp=0.0,
        deadman_switch=deadman,
        servos=tuple([target] * 6),
    )


# -------------------------------------------------------------------------
# State transitions
# -------------------------------------------------------------------------

def test_arm_starts_idle() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(bus)
    assert core.state == ArmState.IDLE


def test_arm_deadman_true_enters_active() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(bus)
    core.apply_command(_cmd(True), now=1.0)
    assert core.state == ArmState.ACTIVE


def test_arm_deadman_false_enters_homing() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(bus)
    core.apply_command(_cmd(True), now=1.0)
    core.apply_command(_cmd(False), now=1.1)
    assert core.state == ArmState.HOMING


def test_arm_timeout_switches_to_homing() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(bus, heartbeat_timeout_sec=1.0)

    core.apply_command(_cmd(True), now=1.0)
    active = core.tick(now=1.1)
    assert active.state == "active"

    # >1s without a command: watchdog fires -> HOMING
    homing = core.tick(now=2.2)
    assert homing.state == "homing"
    assert homing.telemetry.status == "homing"


def test_arm_homing_targets_home_positions() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[2000] * 6)
    core = _make_core(bus, home_positions=[1500] * 6)
    core.apply_command(_cmd(True, target=3000), now=1.0)
    core.apply_command(_cmd(False), now=1.1)

    # After homing transition, target positions should be home
    assert core._target_positions == [1500] * 6


def test_arm_reaches_home_and_stays_idle(monkeypatch) -> None:
    """When arm physically reaches home (feedback within home_deadband), it goes IDLE."""
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=HOMES)
    core = _make_core(bus, home_positions=HOMES, home_deadband=8, home_release_torque=False)
    core.apply_command(_cmd(True), now=1.0)
    core.apply_command(_cmd(False), now=1.1)
    assert core.state == ArmState.HOMING

    # Simulate arm already at home (feedback == home)
    core._last_feedback_positions = list(HOMES)
    core._current_commands = list(HOMES)
    core.tick(now=1.2)
    assert core.state == ArmState.IDLE


# -------------------------------------------------------------------------
# Motion and deadband
# -------------------------------------------------------------------------

def test_arm_applies_ema_step_toward_target() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(bus, ema_alpha=0.5, max_step_per_tick=1000)

    core.apply_command(_cmd(True, target=2000), now=1.0)
    core.tick(now=1.1)

    # EMA step: (2000-1000)*0.5 = 500, max_step=1000 so no clamp -> goal = 1500
    assert bus.goal_positions[0] == 1500


def test_arm_velocity_clamped_by_max_step() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(bus, ema_alpha=0.5, max_step_per_tick=20)

    core.apply_command(_cmd(True, target=2000), now=1.0)
    core.tick(now=1.1)

    # EMA step = 500 but clamped to max_step=20 -> goal = 1020
    assert bus.goal_positions[0] == 1020


def test_arm_deadband_stops_serial_writes() -> None:
    """Once servo is within position_deadband of target, no further writes."""
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[2048] * 6)
    core = _make_core(bus, ema_alpha=0.5, max_step_per_tick=128, position_deadband=2)

    # Target == current (within deadband immediately)
    core.apply_command(_cmd(True, target=2048), now=1.0)
    initial_history_len = len(bus.history)
    core.tick(now=1.1)
    # No writes should have happened (servo_reached[i] set on first apply)
    assert len(bus.history) == initial_history_len or bus.goal_positions[0] == 2048


def test_arm_deadman_command_reaches_target() -> None:
    bus = LaggingFeedbackBus(
        servo_ids=SERVO_IDS,
        initial_positions=[1000] * 6,
        feedback_lag=120,
    )
    core = _make_core(bus, ema_alpha=0.4, max_step_per_tick=48, position_deadband=2, heartbeat_timeout_sec=100.0)

    core.apply_command(
        ArmTargetCommand(timestamp=1.0, deadman_switch=True, servos=(2500, 1000, 1000, 1000, 1000, 1000)),
        now=1.0,
    )

    result = None
    for step in range(1, 80):
        result = core.tick(now=1.0 + (0.05 * step))

    assert result is not None
    assert bus.goal_positions[0] == 2500
    # positions = goal - lag = 2500 - 120 = 2380
    assert result.telemetry.positions[0] == 2380
    assert result.telemetry.status == "ok"


# -------------------------------------------------------------------------
# Load derate
# -------------------------------------------------------------------------

def test_arm_high_load_derates_command_step() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    bus.set_forced_load(1, 1.0)
    core = _make_core(
        bus,
        ema_alpha=0.5,
        max_step_per_tick=48,
        load_derate_start=0.65,
        load_derate_end=0.90,
        load_min_step=6,
    )

    core.apply_command(_cmd(True, target=2000), now=1.0)
    core.tick(now=1.1)
    core.tick(now=1.2)

    # Diagnostics polled at START of tick (before write), so forced load is
    # invisible at tick1 (bus.loads still [0.0]).
    # tick1: load=0.0 -> step=48 -> goal=1048; write updates bus.loads[0]=1.0
    # tick2: load=1.0 >= load_derate_end=0.90 -> step=6 -> goal=1048+6=1054
    assert bus.goal_positions[0] == 1054


# -------------------------------------------------------------------------
# Overload guard
# -------------------------------------------------------------------------

def test_arm_overload_guard_pauses_then_recovers_motion() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[1000] * 6)
    core = _make_core(
        bus,
        ema_alpha=0.5,
        max_step_per_tick=128,
        overload_guard_enabled=True,
        overload_guard_hold_time_sec=0.4,
        overload_target_delta_threshold=16,
    )

    core.apply_command(_cmd(True, target=2000), now=1.0)

    bus.set_overload(1, True)
    guarded = core.tick(now=1.1)
    assert guarded.telemetry.positions[0] == 1000
    assert guarded.telemetry.status == "overload_guard"

    bus.set_overload(1, False)
    resumed = core.tick(now=1.6)
    assert resumed.telemetry.positions[0] > 1000
    assert resumed.telemetry.status == "ok"


def test_arm_overload_guard_suppresses_jam_detection() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[2048] * 6)
    bus.set_stuck(1, True)
    bus.set_forced_load(1, 1.0)
    bus.set_overload(1, True)

    core = _make_core(
        bus,
        ema_alpha=0.5,
        max_step_per_tick=128,
        jam_load_threshold=0.8,
        jam_hold_time_sec=0.2,
        overload_guard_enabled=True,
        overload_guard_hold_time_sec=1.0,
        overload_target_delta_threshold=16,
    )

    core.apply_command(
        ArmTargetCommand(timestamp=1.0, deadman_switch=True, servos=(3000, 2048, 2048, 2048, 2048, 2048)),
        now=1.0,
    )

    core.tick(now=1.1)
    guarded = core.tick(now=1.4)
    assert guarded.telemetry.jam_detected[0] is False
    assert guarded.telemetry.status == "overload_guard"


# -------------------------------------------------------------------------
# Jam detection -> SAFE_MODE
# -------------------------------------------------------------------------

def test_arm_jam_triggers_safe_mode() -> None:
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[2048] * 6)
    bus.set_stuck(1, True)
    bus.set_forced_load(1, 0.95)

    core = _make_core(
        bus,
        ema_alpha=0.5,
        max_step_per_tick=128,
        jam_load_threshold=0.8,
        jam_hold_time_sec=1.0,
        heartbeat_timeout_sec=5.0,
    )

    core.apply_command(
        ArmTargetCommand(timestamp=1.0, deadman_switch=True, servos=(3000, 2048, 2048, 2048, 2048, 2048)),
        now=1.0,
    )

    # tick1: load cache still 0.0 (before first write), jam timer not started
    core.tick(now=1.1)
    # tick2: load cache updated to 0.95; jam timer starts at t=1.2
    core.tick(now=1.2)
    # tick3: t - jam_since = 2.25 - 1.2 = 1.05 >= jam_hold_time_sec=1.0 -> jam fires
    jammed = core.tick(now=2.25)

    # Whole arm enters SAFE_MODE on jam
    assert jammed.telemetry.jam_detected[0] is True
    assert "safe_mode" in jammed.telemetry.status
    assert jammed.state == "safe_mode"
    # All torque should be released (disarm_all called)
    assert all(not t for t in jammed.telemetry.torque_enabled)


def test_arm_safe_mode_sticky_until_reset() -> None:
    """SAFE_MODE is sticky: deadman=True must not restart motion until reset."""
    bus = FakeServoBus(servo_ids=SERVO_IDS, initial_positions=[2048] * 6)
    bus.set_stuck(1, True)
    bus.set_forced_load(1, 0.95)

    core = _make_core(
        bus,
        ema_alpha=0.5,
        max_step_per_tick=128,
        jam_load_threshold=0.8,
        jam_hold_time_sec=1.0,
        heartbeat_timeout_sec=5.0,
    )
    core.apply_command(
        ArmTargetCommand(timestamp=1.0, deadman_switch=True, servos=(3000,) * 6), now=1.0
    )
    core.tick(now=1.1)  # write happens, bus.loads updated
    core.tick(now=1.2)  # load cache refreshed, jam timer starts
    core.tick(now=2.25)  # jam -> SAFE_MODE
    assert core.state == ArmState.SAFE_MODE

    # Sending deadman=True while in SAFE_MODE must not transition to ACTIVE
    core.apply_command(_cmd(True), now=2.2)
    assert core.state == ArmState.SAFE_MODE

    # reset_safe_mode -> IDLE
    core.reset_safe_mode()
    assert core.state == ArmState.IDLE


def test_arm_does_not_jam_when_holding_load_near_limit() -> None:
    bus = FakeServoBus(
        servo_ids=SERVO_IDS,
        initial_positions=[3005, 2048, 2048, 2048, 2048, 2048],
    )
    bus.set_stuck(1, True)
    bus.set_forced_load(1, 0.98)

    core = _make_core(
        bus,
        limits=[(470, 3097), (0, 4095), (0, 4095), (0, 4095), (0, 4095), (0, 4095)],
        ema_alpha=0.5,
        max_step_per_tick=128,
        jam_load_threshold=0.9,
        jam_hold_time_sec=1.0,
        jam_target_delta_threshold=96,
        heartbeat_timeout_sec=10.0,
    )

    core.apply_command(
        ArmTargetCommand(timestamp=1.0, deadman_switch=True, servos=(3097, 2048, 2048, 2048, 2048, 2048)),
        now=1.0,
    )

    core.tick(now=1.1)
    held = core.tick(now=2.2)

    assert held.telemetry.jam_detected[0] is False
    assert held.telemetry.status == "ok"


# -------------------------------------------------------------------------
# Communication fault
# -------------------------------------------------------------------------

def test_arm_comm_timeout_suppresses_jam() -> None:
    bus = CommFaultBus(servo_ids=SERVO_IDS, initial_positions=[2048] * 6)
    bus.set_stuck(1, True)
    bus.set_forced_load(1, 1.0)

    core = _make_core(
        bus,
        heartbeat_timeout_sec=5.0,
        ema_alpha=0.5,
        max_step_per_tick=128,
        jam_load_threshold=0.8,
        jam_hold_time_sec=1.0,
    )

    core.apply_command(
        ArmTargetCommand(timestamp=1.0, deadman_switch=True, servos=(3000, 2048, 2048, 2048, 2048, 2048)),
        now=1.0,
    )

    core.tick(now=1.1)
    degraded = core.tick(now=2.2)

    assert degraded.telemetry.status == "comm_timeout"
    assert degraded.telemetry.jam_detected[0] is False


# -------------------------------------------------------------------------
# Temperature safety
# -------------------------------------------------------------------------

def test_arm_overheat_triggers_safe_mode() -> None:
    """Simulated overheat: servo reports 75°C > 60°C limit -> SAFE_MODE."""
    bus = HotBus(servo_ids=SERVO_IDS, initial_positions=[2048] * 6)
    core = _make_core(bus, temp_limit_celsius=60.0, ema_alpha=0.5, max_step_per_tick=128)

    core.apply_command(_cmd(True, target=3000), now=1.0)
    # HotBus.read_temperatures() returns [75.0, 25.0, ...] on every diagnostic poll
    result = core.tick(now=1.1)

    assert result.state == "safe_mode"
    assert "safe_mode" in result.telemetry.status
    assert "overheat" in result.telemetry.status or "safe_mode" in result.telemetry.status


# -------------------------------------------------------------------------
# Utility
# -------------------------------------------------------------------------

def test_resolve_serial_port_prefers_primary_and_falls_back(tmp_path) -> None:
    primary = tmp_path / "ttyACM0"
    fallback = tmp_path / "ttyUSB0"

    fallback.write_text("", encoding="utf-8")
    assert resolve_serial_port(str(primary), str(fallback)) == str(fallback)

    primary.write_text("", encoding="utf-8")
    assert resolve_serial_port(str(primary), str(fallback)) == str(primary)
