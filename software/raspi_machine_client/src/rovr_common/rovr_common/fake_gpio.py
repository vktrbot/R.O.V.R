"""GPIO backends for chassis controller (mock + real)."""

from __future__ import annotations

from dataclasses import dataclass
import glob
import os
import time

from .clamp import clamp

try:  # pragma: no cover - optional dependency on target hardware
    from gpiozero import DigitalOutputDevice, PWMOutputDevice, Servo
except Exception:  # pragma: no cover
    DigitalOutputDevice = None
    PWMOutputDevice = None
    Servo = None

try:  # pragma: no cover - optional dependency on target hardware
    from gpiozero.pins.pigpio import PiGPIOFactory
except Exception:  # pragma: no cover
    PiGPIOFactory = None

try:  # pragma: no cover - Pi 5 default factory
    from gpiozero.pins.lgpio import LGPIOFactory
except Exception:  # pragma: no cover
    LGPIOFactory = None

try:  # pragma: no cover - optional dependency on target hardware
    import lgpio
except Exception:  # pragma: no cover
    lgpio = None


@dataclass
class FakeChassisSnapshot:
    direction: bool
    duty_cycle: float
    steering: float


class FakeChassisHardware:
    """In-memory stand-in for GPIO driven motor + steering hardware."""

    def __init__(self) -> None:
        self.direction = False
        self.duty_cycle = 0.0
        self.steering = 0.0
        self.history: list[FakeChassisSnapshot] = []

    def set_direction(self, direction: bool) -> None:
        self.direction = bool(direction)
        self._record()

    def set_pwm(self, duty_cycle: float) -> None:
        self.duty_cycle = clamp(float(duty_cycle), 0.0, 1.0)
        self._record()

    def set_steering(self, steering: float) -> None:
        self.steering = clamp(float(steering), -1.0, 1.0)
        self._record()

    def snapshot(self) -> FakeChassisSnapshot:
        return FakeChassisSnapshot(
            direction=self.direction,
            duty_cycle=self.duty_cycle,
            steering=self.steering,
        )

    def _record(self) -> None:
        self.history.append(self.snapshot())

    def reinit(self) -> None:
        pass

    def close(self) -> None:
        pass


class RealChassisHardware:
    """Real GPIO backend for MD10C drive + steering servo."""

    def __init__(
        self,
        pwm_pin: int,
        dir_pin: int,
        steer_pin: int,
        pwm_frequency: int,
        servo_min_pulse: float,
        servo_max_pulse: float,
        use_pigpio: bool,
        steer_center_offset: float = 0.0,
        shutdown_center_hold_sec: float = 0.2,
        servo_frequency_hz: int = 50,
        steer_pulse_deadband_us: int = 2,
        steer_idle_detach: bool = False,
        steer_idle_deadband: float = 0.02,
        steer_idle_pwm_threshold: float = 0.02,
    ) -> None:
        if DigitalOutputDevice is None or PWMOutputDevice is None or Servo is None:
            raise RuntimeError("gpiozero is required for backend=real")

        # Store construction params for reinit().
        self._pwm_pin = int(pwm_pin)
        self._dir_pin = int(dir_pin)
        self._steer_pin = int(steer_pin)
        self._pwm_frequency = int(pwm_frequency)
        self._use_pigpio = bool(use_pigpio)

        self._servo_min_pulse = float(servo_min_pulse)
        self._servo_max_pulse = float(servo_max_pulse)
        self._servo_frequency_hz = int(max(20, min(400, int(servo_frequency_hz))))
        self._steer_pulse_deadband_us = int(max(0, min(50, int(steer_pulse_deadband_us))))
        self._steer_idle_detach = bool(steer_idle_detach)
        self._steer_idle_deadband = float(clamp(float(steer_idle_deadband), 0.0, 0.2))
        self._steer_idle_pwm_threshold = float(clamp(float(steer_idle_pwm_threshold), 0.0, 0.2))
        self.steer_center_offset = clamp(float(steer_center_offset), -0.5, 0.5)
        self.shutdown_center_hold_sec = max(0.0, float(shutdown_center_hold_sec))

        self.direction = False
        self.duty_cycle = 0.0
        self.steering = 0.0
        self.reinit_count = 0

        self._dir = None
        self._pwm = None
        self._steer = None
        self._steer_lgpio_handle: int | None = None
        self._steer_lgpio_pin: int | None = None
        self._steer_uses_lgpio = False
        self._steer_detached = False
        self._idle_detach_armed = False
        self._last_steer_pulse_us: int | None = None
        self._factory = None

        self._open_gpio()

    def _open_gpio(self) -> None:
        """Create all GPIO handles from scratch.

        Always creates a NEW pin factory so that the underlying lgpio chip
        handle is fresh.  This is critical for recovery after power glitches
        on Pi 5 where the old handle becomes stale but API calls still
        succeed without error.
        """
        if self._use_pigpio:
            if PiGPIOFactory is None:
                raise RuntimeError("PiGPIOFactory is unavailable, install gpiozero[pigpio] + pigpio")
            factory = PiGPIOFactory()
        elif LGPIOFactory is not None:
            factory = LGPIOFactory()
        else:
            factory = None
        self._factory = factory

        self._dir = DigitalOutputDevice(
            self._dir_pin,
            initial_value=self.direction,
            pin_factory=factory,
        )
        self._pwm = PWMOutputDevice(
            self._pwm_pin,
            initial_value=self.duty_cycle,
            frequency=self._pwm_frequency,
            pin_factory=factory,
        )
        self._steer = None
        self._steer_lgpio_handle = None
        self._steer_lgpio_pin = None
        self._steer_uses_lgpio = False
        self._steer_detached = False
        self._idle_detach_armed = False

        initial_target = clamp(self.steering + self.steer_center_offset, -1.0, 1.0)
        if not self._use_pigpio:
            self._steer_uses_lgpio = self._setup_lgpio_servo(self._steer_pin, initial_target)

        if not self._steer_uses_lgpio:
            self._steer = Servo(
                self._steer_pin,
                initial_value=initial_target,
                min_pulse_width=self._servo_min_pulse,
                max_pulse_width=self._servo_max_pulse,
                pin_factory=factory,
            )
        self._last_steer_pulse_us = int(self._steering_to_pulse_us(initial_target))

    def _close_gpio(self) -> None:
        """Release all GPIO handles AND the pin factory.

        Closing the factory ensures the lgpio chip handle is fully released
        so that _open_gpio() gets a completely fresh connection to the GPIO
        hardware.
        """
        if self._pwm is not None:
            try:
                self._pwm.close()
            except Exception:
                pass
            self._pwm = None
        if self._dir is not None:
            try:
                self._dir.close()
            except Exception:
                pass
            self._dir = None
        if self._steer_uses_lgpio:
            self._close_lgpio_servo()
        elif self._steer is not None:
            try:
                self._steer.close()
            except Exception:
                pass
            self._steer = None
        if self._factory is not None:
            try:
                self._factory.close()
            except Exception:
                pass
            self._factory = None

    def reinit(self) -> None:
        """Close and reopen all GPIO handles to reset DMA/PWM hardware state.

        Preserves current direction/duty/steering so the next tick resumes
        from where it left off, but forces the Pi's PWM peripheral and lgpio
        servo timer to be reprogrammed from scratch.
        """
        self._close_gpio()
        time.sleep(0.05)
        self._open_gpio()
        self.reinit_count += 1

    def set_direction(self, direction: bool) -> None:
        self.direction = bool(direction)
        self._dir.value = self.direction

    def set_pwm(self, duty_cycle: float) -> None:
        self.duty_cycle = clamp(float(duty_cycle), 0.0, 1.0)
        self._pwm.value = self.duty_cycle
        if self.duty_cycle > self._steer_idle_pwm_threshold:
            self._idle_detach_armed = True
        if (
            self._steer_uses_lgpio
            and self._steer_detached
            and self.duty_cycle > self._steer_idle_pwm_threshold
        ):
            target = clamp(self.steering + self.steer_center_offset, -1.0, 1.0)
            pulse_us = int(self._steering_to_pulse_us(target))
            self._write_lgpio_servo_pulse(pulse_us)
            self._steer_detached = False
            self._last_steer_pulse_us = int(pulse_us)

    def set_steering(self, steering: float) -> None:
        self.steering = clamp(float(steering), -1.0, 1.0)
        target = clamp(self.steering + self.steer_center_offset, -1.0, 1.0)
        if abs(float(target)) > self._steer_idle_deadband:
            self._idle_detach_armed = True

        if self._steer_uses_lgpio and self._should_idle_detach(target):
            if not self._steer_detached:
                self._write_lgpio_servo_pulse(0)
                self._steer_detached = True
                self._last_steer_pulse_us = None
            return

        pulse_us = int(self._steering_to_pulse_us(target))
        # Avoid reprogramming servo output when command is effectively unchanged.
        if (
            self._last_steer_pulse_us is not None
            and abs(int(pulse_us) - int(self._last_steer_pulse_us)) <= self._steer_pulse_deadband_us
        ):
            return

        if self._steer_uses_lgpio:
            if self._steer_detached:
                self._steer_detached = False
            self._write_lgpio_servo_pulse(int(pulse_us))
            self._last_steer_pulse_us = int(pulse_us)
            return
        if self._steer is not None:
            self._steer.value = target
            self._last_steer_pulse_us = int(pulse_us)

    def close(self) -> None:
        try:
            if self._pwm is not None:
                self._pwm.value = 0.0
            center = clamp(self.steer_center_offset, -1.0, 1.0)
            center_pulse = int(self._steering_to_pulse_us(center))
            if self._steer_uses_lgpio:
                if self._steer_lgpio_handle is not None and self._steer_lgpio_pin is not None and lgpio is not None:
                    self._write_lgpio_servo_pulse(int(center_pulse))
                    self._last_steer_pulse_us = int(center_pulse)
            elif self._steer is not None:
                self._steer.value = center
                self._last_steer_pulse_us = int(center_pulse)
            if self.shutdown_center_hold_sec > 0.0:
                time.sleep(self.shutdown_center_hold_sec)
        except BaseException:
            pass
        finally:
            self._close_gpio()

    def _steering_to_pulse_us(self, steering: float) -> int:
        clamped = clamp(float(steering), -1.0, 1.0)
        ratio = (clamped + 1.0) / 2.0
        pulse_seconds = self._servo_min_pulse + ratio * (self._servo_max_pulse - self._servo_min_pulse)
        return int(round(pulse_seconds * 1_000_000.0))

    def _setup_lgpio_servo(self, steer_pin: int, initial_target: float) -> bool:
        if lgpio is None:
            return False

        initial_pulse = int(self._steering_to_pulse_us(clamp(float(initial_target), -1.0, 1.0)))
        for chip_index in self._candidate_gpiochips():
            handle: int | None = None
            try:
                handle = int(lgpio.gpiochip_open(int(chip_index)))
                lgpio.gpio_claim_output(int(handle), int(steer_pin), 0)
                lgpio.tx_servo(int(handle), int(steer_pin), int(initial_pulse), int(self._servo_frequency_hz))
                self._steer_lgpio_handle = int(handle)
                self._steer_lgpio_pin = int(steer_pin)
                return True
            except Exception as e:
                print(e)  # TODO: remove 
                if handle is not None:
                    try:
                        lgpio.gpiochip_close(int(handle))
                    except Exception as e:
                        pass
        return False

    def _candidate_gpiochips(self) -> list[int]:
        indices: list[int] = []
        for path in sorted(glob.glob("/dev/gpiochip*")):
            name = os.path.basename(path)
            suffix = name.removeprefix("gpiochip")
            if suffix.isdigit():
                indices.append(int(suffix))
        # On RPi 5 the GPIO block is commonly gpiochip4, so keep naturally sorted order.
        return sorted(indices)

    def _close_lgpio_servo(self) -> None:
        if lgpio is None:
            return
        handle = self._steer_lgpio_handle
        pin = self._steer_lgpio_pin
        self._steer_lgpio_handle = None
        self._steer_lgpio_pin = None
        if handle is None or pin is None:
            return
        try:
            lgpio.tx_servo(int(handle), int(pin), 0)
        except Exception:
            pass
        try:
            lgpio.gpio_free(int(handle), int(pin))
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(int(handle))
        except Exception:
            pass

    def _write_lgpio_servo_pulse(self, pulse_us: int) -> None:
        if self._steer_lgpio_handle is None or self._steer_lgpio_pin is None or lgpio is None:
            return
        lgpio.tx_servo(
            int(self._steer_lgpio_handle),
            int(self._steer_lgpio_pin),
            int(pulse_us),
            int(self._servo_frequency_hz),
        )

    def _should_idle_detach(self, target: float) -> bool:
        if not self._steer_idle_detach:
            return False
        if not self._idle_detach_armed:
            return False
        if abs(float(target)) > self._steer_idle_deadband:
            return False
        return self.duty_cycle <= self._steer_idle_pwm_threshold
