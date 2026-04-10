"""Tests for chassis control safety behavior."""

from __future__ import annotations

from rovr_common.fake_gpio import FakeChassisHardware

from rovr_chassis.control import ChassisCore


def test_chassis_timeout_returns_to_safe_state() -> None:
    hardware = FakeChassisHardware()
    core = ChassisCore(
        hardware=hardware,
        steer_amount=0.6,
        accel_rate=1.4,
        decel_rate=2.2,
        steer_rate=3.0,
        heartbeat_timeout_ms=500,
    )

    core.update_power(0.9)
    core.update_cmd_vel(linear=1.0, angular=1.0, now=1.0)
    active = core.tick(now=1.1, dt=0.1)
    timed_out = core.tick(now=1.8, dt=0.7)

    assert active.duty_cycle > 0.0
    assert active.steering > 0.0
    assert timed_out.timed_out is True
    assert timed_out.duty_cycle == 0.0
    assert timed_out.steering == 0.0


def test_chassis_steering_returns_to_center_when_turn_signal_is_zero() -> None:
    hardware = FakeChassisHardware()
    core = ChassisCore(
        hardware=hardware,
        steer_amount=0.6,
        accel_rate=1.4,
        decel_rate=2.2,
        steer_rate=3.0,
        heartbeat_timeout_ms=500,
    )
    core.update_power(0.6)

    core.update_cmd_vel(linear=1.0, angular=1.0, now=1.0)
    turning = core.tick(now=1.1, dt=0.1)

    core.update_cmd_vel(linear=1.0, angular=0.0, now=1.2)
    centered = core.tick(now=1.3, dt=0.1)

    assert turning.steering > 0.0
    assert centered.steering == 0.0


def test_chassis_left_right_steering_scales_are_asymmetric() -> None:
    left_hardware = FakeChassisHardware()
    right_hardware = FakeChassisHardware()
    left_core = ChassisCore(
        hardware=left_hardware,
        steer_amount=0.6,
        accel_rate=1.4,
        decel_rate=2.2,
        steer_rate=3.0,
        heartbeat_timeout_ms=500,
        steer_left_scale=1.10,
        steer_right_scale=0.90,
    )
    right_core = ChassisCore(
        hardware=right_hardware,
        steer_amount=0.6,
        accel_rate=1.4,
        decel_rate=2.2,
        steer_rate=3.0,
        heartbeat_timeout_ms=500,
        steer_left_scale=1.10,
        steer_right_scale=0.90,
    )

    left_core.update_cmd_vel(linear=0.0, angular=-1.0, now=1.0)
    right_core.update_cmd_vel(linear=0.0, angular=1.0, now=1.0)
    left = left_core.tick(now=1.4, dt=0.4)
    right = right_core.tick(now=1.4, dt=0.4)

    assert abs(left.steering) > abs(right.steering)


def test_chassis_hardware_failure_is_counted_and_reported() -> None:
    """A brown-out / pigpio drop should be surfaced via ChassisOutputs, not silently swallowed."""

    class FlakyHardware(FakeChassisHardware):
        def __init__(self) -> None:
            super().__init__()
            self.fail = False

        def set_pwm(self, duty_cycle: float) -> None:
            if self.fail:
                raise RuntimeError("simulated pigpio disconnect")
            super().set_pwm(duty_cycle)

    hardware = FlakyHardware()
    core = ChassisCore(
        hardware=hardware,
        steer_amount=0.6,
        accel_rate=1.4,
        decel_rate=2.2,
        steer_rate=3.0,
        heartbeat_timeout_ms=500,
    )

    core.update_cmd_vel(linear=1.0, angular=0.0, now=1.0)
    ok = core.tick(now=1.1, dt=0.1)
    assert ok.hardware_ok is True
    assert ok.hardware_error is None
    assert core.hardware_success_total == 1
    assert core.hardware_fail_streak == 0

    hardware.fail = True
    bad1 = core.tick(now=1.2, dt=0.1)
    bad2 = core.tick(now=1.3, dt=0.1)
    assert bad1.hardware_ok is False
    assert bad2.hardware_ok is False
    assert "simulated pigpio disconnect" in (bad2.hardware_error or "")
    assert core.hardware_fail_streak == 2
    assert core.hardware_fail_total == 2

    hardware.fail = False
    recovered = core.tick(now=1.4, dt=0.1)
    assert recovered.hardware_ok is True
    assert core.hardware_fail_streak == 0
    assert core.hardware_success_total == 2


def test_chassis_direction_change_coasts_to_zero_first() -> None:
    """Direction pin must not flip while duty cycle is non-zero (electrical safety)."""
    hardware = FakeChassisHardware()
    core = ChassisCore(
        hardware=hardware,
        steer_amount=0.6,
        accel_rate=10.0,
        decel_rate=10.0,
        steer_rate=3.0,
        heartbeat_timeout_ms=500,
    )
    core.update_power(0.9)

    # Drive forward until duty is well above zero.
    core.update_cmd_vel(linear=1.0, angular=0.0, now=1.0)
    forward = core.tick(now=1.1, dt=0.1)
    assert forward.direction is False
    assert forward.duty_cycle > 0.0

    # Immediately command full reverse — duty should ramp toward 0 before direction flips.
    core.update_cmd_vel(linear=-1.0, angular=0.0, now=1.1)
    coasting = core.tick(now=1.15, dt=0.05)
    # Direction pin must NOT have flipped yet while duty is still positive.
    assert coasting.direction is False

    # After enough time to fully coast to zero, direction should flip.
    reverse = core.tick(now=1.5, dt=0.35)
    assert reverse.duty_cycle == 0.0 or reverse.direction is True
