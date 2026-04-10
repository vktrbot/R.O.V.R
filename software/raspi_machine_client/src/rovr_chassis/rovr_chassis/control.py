"""Pure control logic for the chassis controller."""

from __future__ import annotations

from dataclasses import dataclass

from rovr_common.clamp import clamp
from rovr_common.fake_gpio import FakeChassisHardware
from rovr_common.smoothing import slew_step
from rovr_common.watchdog import TimeoutWatchdog

import json, inspect
import time

@dataclass(frozen=True)
class ChassisOutputs:
    direction: bool
    duty_cycle: float
    steering: float
    timed_out: bool
    hardware_ok: bool = True
    hardware_error: str | None = None


class ChassisCore:
    """State machine for the DC drive and steering servo."""

    def __init__(
        self,
        hardware: FakeChassisHardware,
        steer_amount: float,
        accel_rate: float,
        decel_rate: float,
        steer_rate: float,
        heartbeat_timeout_ms: int,
        steer_left_scale: float = 1.10,
        steer_right_scale: float = 0.90,
    ) -> None:
        self.hardware = hardware
        self.steer_amount = float(steer_amount)
        self.steer_left_scale = float(max(0.1, steer_left_scale))
        self.steer_right_scale = float(max(0.1, steer_right_scale))
        self.accel_rate = float(accel_rate)
        self.decel_rate = float(decel_rate)
        self.steer_rate = float(steer_rate)
        self.command_watchdog = TimeoutWatchdog(float(heartbeat_timeout_ms) / 1000.0)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.motor_power = 0.1

        self.current_duty = 0.0
        self.current_steering = 0.0
        self._output_direction: bool = False  # actual direction sent to hardware

        self.timer_bs = 0

        # Hardware write health tracking — used by the node layer to detect
        # silent brown-outs / pigpio disconnects where the ROS timer keeps
        # firing but nothing actually reaches the motor driver.
        self.hardware_fail_streak = 0
        self.hardware_fail_total = 0
        self.hardware_success_total = 0
        self.last_hardware_error: str | None = None

    def update_cmd_vel(self, linear: float, angular: float, now: float) -> None:
        self.target_linear = clamp(float(linear), -1.0, 1.0)
        self.target_angular = clamp(float(angular), -1.0, 1.0)
        self.command_watchdog.mark(now)

    def update_power(self, power: float) -> None:
        self.motor_power = clamp(float(power), 0.1, 0.9)

    def tick(self, now: float, dt: float) -> ChassisOutputs:
        self.timer_bs += dt

        timed_out = self.command_watchdog.expired(now)
        target_linear = 0.0 if timed_out else self.target_linear
        target_angular = 0.0 if timed_out else self.target_angular

        # --- soft start / soft stop with direction-coast safety ---
        # When direction changes, ramp duty to zero before flipping the motor
        # direction pin to avoid applying reverse voltage to a spinning motor.
        target_direction = bool(target_linear < 0.0)
        if target_direction != self._output_direction and self.current_duty > 0.0:
            # Coast to zero; do not flip direction yet.
            target_duty = 0.0
        else:
            if target_direction != self._output_direction:
                # Duty has reached zero — safe to commit the direction flip.
                self._output_direction = target_direction
            target_duty = abs(target_linear) * self.motor_power

        duty_rate = self.accel_rate if target_duty >= self.current_duty else self.decel_rate
        self.current_duty = clamp(slew_step(self.current_duty, target_duty, duty_rate, dt), 0.0, 1.0)

        turn_scale = 1.0
        if target_angular < 0.0:
            turn_scale = self.steer_left_scale
        elif target_angular > 0.0:
            turn_scale = self.steer_right_scale
        target_steering = clamp(target_angular * self.steer_amount * turn_scale, -1.0, 1.0)
        self.current_steering = clamp(slew_step(self.current_steering, target_steering, self.steer_rate, dt), -1.0, 1.0)

        hardware_ok = True
        hardware_error: str | None = None
        try:
            self.hardware.set_direction(self._output_direction)
            self.hardware.set_pwm(self.current_duty)
            self.hardware.set_steering(self.current_steering)
        except BaseException as exc:  # pigpio/lgpio/gpiozero can raise various types
            hardware_ok = False
            hardware_error = f"{type(exc).__name__}: {exc}"
            self.hardware_fail_streak += 1
            self.hardware_fail_total += 1
            self.last_hardware_error = hardware_error
        else:
            self.hardware_fail_streak = 0
            self.hardware_success_total += 1

        return ChassisOutputs(
            direction=self._output_direction,
            duty_cycle=self.current_duty,
            steering=self.current_steering,
            timed_out=timed_out,
            hardware_ok=hardware_ok,
            hardware_error=hardware_error,
        )
