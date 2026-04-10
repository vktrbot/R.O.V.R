#!/usr/bin/env python3
"""Keyboard tester for combined drive + steering over /cmd_vel and /cmd_power."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from evdev import InputDevice, categorize, ecodes, list_devices
except Exception:  # pragma: no cover - optional dependency
    InputDevice = None
    categorize = None
    ecodes = None
    list_devices = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard drive/steer test publisher")
    parser.add_argument("--power", type=float, default=0.4, help="Initial power in [0.1..0.9]")
    parser.add_argument("--power-step", type=float, default=0.05, help="Power delta for T/G keys")
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate (Hz)")
    parser.add_argument(
        "--input-mode",
        choices=["auto", "tty", "evdev"],
        default="auto",
        help="Keyboard backend: auto tries evdev first, then tty fallback",
    )
    parser.add_argument(
        "--evdev-device",
        type=str,
        default="",
        help="Optional /dev/input/eventX keyboard device for --input-mode evdev",
    )
    parser.add_argument(
        "--sticky",
        action="store_true",
        help="Keep axis command after key press until changed manually (x/c/space)",
    )
    parser.add_argument(
        "--hold-timeout",
        type=float,
        default=0.25,
        help="Axis auto-stop timeout in seconds (ignored when --sticky is used)",
    )
    return parser.parse_args()


class KeyboardDriveTester(Node):
    def __init__(
        self,
        initial_power: float,
        power_step: float,
        publish_rate_hz: float,
        hold_timeout_sec: float,
        use_axis_timeout: bool,
    ) -> None:
        super().__init__("drive_keyboard_tester")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_power_pub = self.create_publisher(Float32, "/cmd_power", 10)

        self.power = clamp(initial_power, 0.1, 0.9)
        self.power_step = max(0.01, float(power_step))
        self.hold_timeout_sec = max(0.0, float(hold_timeout_sec))
        self.use_axis_timeout = bool(use_axis_timeout)

        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._last_linear_input = 0.0
        self._last_angular_input = 0.0
        self._last_state_print = 0.0
        self._running = True

        self.create_timer(1.0 / max(1.0, float(publish_rate_hz)), self._publish_tick)

    @property
    def running(self) -> bool:
        return bool(self._running)

    def set_axes(self, linear: float, angular: float, now: float | None = None) -> None:
        stamp = time.monotonic() if now is None else float(now)
        self._linear_cmd = clamp(linear, -1.0, 1.0)
        self._angular_cmd = clamp(angular, -1.0, 1.0)
        self._last_linear_input = stamp
        self._last_angular_input = stamp

    def handle_key(self, key: str) -> None:
        now = time.monotonic()
        key = key.lower()

        if key == "w":
            self._linear_cmd = 1.0
            self._last_linear_input = now
            return

        if key == "s":
            self._linear_cmd = -1.0
            self._last_linear_input = now
            return

        if key == "a":
            self._angular_cmd = -1.0
            self._last_angular_input = now
            return

        if key == "d":
            self._angular_cmd = 1.0
            self._last_angular_input = now
            return

        if key == "x":
            self._linear_cmd = 0.0
            self._last_linear_input = now
            return

        if key == "c":
            self._angular_cmd = 0.0
            self._last_angular_input = now
            return

        if key == "t":
            self.power = clamp(self.power + self.power_step, 0.1, 0.9)
            self.get_logger().info(f"Power increased to {self.power:.2f}")
            return

        if key == "g":
            self.power = clamp(self.power - self.power_step, 0.1, 0.9)
            self.get_logger().info(f"Power decreased to {self.power:.2f}")
            return

        if key == " ":
            self._clear_motion()
            self.get_logger().info("Motion cleared (space).")
            return

        if key == "q":
            self._running = False

    def request_stop(self) -> None:
        self._running = False

    def stop(self) -> None:
        self._clear_motion()
        self._publish(linear=0.0, angular=0.0)

    def _clear_motion(self) -> None:
        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._last_linear_input = 0.0
        self._last_angular_input = 0.0

    def _compute_axes(self, now: float) -> tuple[float, float]:
        if self.use_axis_timeout and self.hold_timeout_sec > 0.0:
            if self._linear_cmd != 0.0 and self._last_linear_input > 0.0:
                if (now - self._last_linear_input) > self.hold_timeout_sec:
                    self._linear_cmd = 0.0
            if self._angular_cmd != 0.0 and self._last_angular_input > 0.0:
                if (now - self._last_angular_input) > self.hold_timeout_sec:
                    self._angular_cmd = 0.0

        return float(self._linear_cmd), float(self._angular_cmd)

    def _publish(self, linear: float, angular: float) -> None:
        power_msg = Float32()
        power_msg.data = float(self.power)
        self.cmd_power_pub.publish(power_msg)

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)

    def _publish_tick(self) -> None:
        now = time.monotonic()
        linear, angular = self._compute_axes(now)
        self._publish(linear=linear, angular=angular)

        if (now - self._last_state_print) >= 0.35:
            self._last_state_print = now
            sys.stdout.write(
                "\r"
                f"state: linear={linear:+.1f} angular={angular:+.1f} "
                f"power={self.power:.2f}  "
            )
            sys.stdout.flush()


def _read_key_nonblocking(timeout_sec: float) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], max(0.0, timeout_sec))
    if not ready:
        return None
    value = sys.stdin.read(1)
    if not value:
        return None
    return value


class EvdevKeyboard:
    """Key down/up input backend with true simultaneous key handling."""

    def __init__(self, device_path: str = "") -> None:
        if InputDevice is None or ecodes is None or categorize is None:
            raise RuntimeError("python-evdev is not installed")
        self.device = self._open_device(device_path)
        self._set_nonblocking(self.device)
        self.pressed: set[int] = set()

    @staticmethod
    def _set_nonblocking(device: InputDevice) -> None:
        # Compatibility across evdev versions.
        if hasattr(device, "set_nonblocking"):
            device.set_nonblocking(True)
            return
        try:
            os.set_blocking(int(device.fd), False)
            return
        except Exception:
            pass
        import fcntl

        flags = fcntl.fcntl(int(device.fd), fcntl.F_GETFL)
        fcntl.fcntl(int(device.fd), fcntl.F_SETFL, flags | os.O_NONBLOCK)

    def _open_device(self, device_path: str) -> InputDevice:
        if device_path:
            if not os.path.exists(device_path):
                raise RuntimeError(f"evdev device not found: {device_path}")
            return InputDevice(device_path)

        # Auto-pick the first keyboard-like device with WASD keys.
        if list_devices is None:
            raise RuntimeError("evdev.list_devices is unavailable")
        for path in sorted(list_devices()):
            try:
                dev = InputDevice(path)
                caps = dev.capabilities()
                keys = set(caps.get(ecodes.EV_KEY, []))
                need = {
                    ecodes.KEY_W,
                    ecodes.KEY_A,
                    ecodes.KEY_S,
                    ecodes.KEY_D,
                }
                if need.issubset(keys):
                    return dev
            except Exception:
                continue
        raise RuntimeError("No keyboard-like evdev device with WASD keys was found")

    def _update_axes(self, node: KeyboardDriveTester, now: float) -> None:
        linear = 0.0
        angular = 0.0
        if ecodes.KEY_W in self.pressed and ecodes.KEY_S not in self.pressed:
            linear = 1.0
        elif ecodes.KEY_S in self.pressed and ecodes.KEY_W not in self.pressed:
            linear = -1.0

        if ecodes.KEY_A in self.pressed and ecodes.KEY_D not in self.pressed:
            angular = -1.0
        elif ecodes.KEY_D in self.pressed and ecodes.KEY_A not in self.pressed:
            angular = 1.0

        node.set_axes(linear=linear, angular=angular, now=now)

    def poll(self, node: KeyboardDriveTester) -> None:
        now = time.monotonic()
        try:
            # In some evdev versions `read()` returns an iterator that may raise
            # BlockingIOError during iteration, so materialize inside try.
            events = list(self.device.read())
        except BlockingIOError:
            # No new key events; keep existing pressed-key state.
            self._update_axes(node, now=now)
            return
        except OSError as exc:
            if int(getattr(exc, "errno", 0)) == 11:
                self._update_axes(node, now=now)
                return
            raise

        for event in events:
            if event.type != ecodes.EV_KEY:
                continue
            key_event = categorize(event)
            code = int(key_event.scancode)
            value = int(key_event.keystate)  # 0=up, 1=down, 2=hold

            if value == 0:
                self.pressed.discard(code)
            else:
                self.pressed.add(code)

            if value == 1:
                if code == ecodes.KEY_T:
                    node.power = clamp(node.power + node.power_step, 0.1, 0.9)
                    node.get_logger().info(f"Power increased to {node.power:.2f}")
                elif code == ecodes.KEY_G:
                    node.power = clamp(node.power - node.power_step, 0.1, 0.9)
                    node.get_logger().info(f"Power decreased to {node.power:.2f}")
                elif code == ecodes.KEY_SPACE:
                    node.stop()
                elif code == ecodes.KEY_Q:
                    node.request_stop()

        self._update_axes(node, now=time.monotonic())


def main() -> None:
    args = parse_args()
    if args.power < 0.1 or args.power > 0.9:
        raise SystemExit("--power must be within [0.1..0.9]")

    if not sys.stdin.isatty():
        raise SystemExit("This script must run in an interactive terminal (TTY).")

    rclpy.init()
    node = KeyboardDriveTester(
        initial_power=float(args.power),
        power_step=float(args.power_step),
        publish_rate_hz=float(args.rate),
        hold_timeout_sec=(0.0 if bool(args.sticky) else float(args.hold_timeout)),
        use_axis_timeout=(not bool(args.sticky)),
    )

    use_evdev = False
    evdev_backend: EvdevKeyboard | None = None
    old_term: list[int] | None = None
    selected_mode = str(args.input_mode).strip().lower()

    if selected_mode in {"auto", "evdev"}:
        try:
            evdev_backend = EvdevKeyboard(device_path=str(args.evdev_device).strip())
            use_evdev = True
        except Exception as exc:
            if selected_mode == "evdev":
                raise SystemExit(f"Failed to init evdev mode: {exc}")
            print(f"[WARN] evdev unavailable, fallback to tty mode: {exc}")

    if not use_evdev:
        old_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    print("")
    print("Keyboard drive test")
    print("  w / s -> forward / backward")
    print("  a / d -> left / right steering")
    print("  x     -> stop drive axis (linear=0)")
    print("  c     -> center steering axis (angular=0)")
    print("  t / g -> increase / decrease motor power (0.1..0.9)")
    print("  space -> stop movement immediately")
    print("  q     -> quit")
    print("")
    mode_name = "sticky" if bool(args.sticky) else "auto-stop"
    input_backend_name = "evdev" if use_evdev else "tty"
    print(
        f"Start power={node.power:.2f}, power_step={node.power_step:.2f}, "
        f"mode={mode_name}, hold_timeout={node.hold_timeout_sec:.2f}s, "
        f"input_backend={input_backend_name}"
    )

    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.02)

            if use_evdev and evdev_backend is not None:
                evdev_backend.poll(node)
            else:
                # Drain all currently available key events this cycle.
                while True:
                    key = _read_key_nonblocking(timeout_sec=0.0)
                    if key is None:
                        break
                    if key == "\x03":
                        raise KeyboardInterrupt
                    node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        if old_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        print("")
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
