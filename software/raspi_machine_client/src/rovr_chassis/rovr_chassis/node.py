"""ROS 2 chassis controller node."""

from __future__ import annotations

import os
import threading
import time

from rovr_common.fake_gpio import FakeChassisHardware, RealChassisHardware

from .control import ChassisCore

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Float32, String
except ImportError:  # pragma: no cover - allows importing without ROS installed
    rclpy = None
    Node = None
    Twist = None
    Float32 = None
    String = None


class _UnavailableNode:
    def __init__(self, *_: object, **__: object) -> None:
        raise RuntimeError("rclpy is required to use this node")


BaseNode = Node if Node is not None else _UnavailableNode


class ChassisControllerNode(BaseNode):
    """Bridges ROS topics to chassis control logic."""

    def __init__(self) -> None:
        super().__init__("chassis_controller_node")

        self.declare_parameter("backend", "mock")
        self.declare_parameter("strict_startup", False)
        self.declare_parameter("dir_pin", 23)
        self.declare_parameter("pwm_pin", 18)
        self.declare_parameter("steer_pin", 24)
        self.declare_parameter("pwm_frequency", 1000)
        self.declare_parameter("servo_min_pulse", 0.0009)
        self.declare_parameter("servo_max_pulse", 0.0021)
        self.declare_parameter("servo_frequency_hz", 50)
        self.declare_parameter("steer_pulse_deadband_us", 2)
        self.declare_parameter("steer_idle_detach", False)
        self.declare_parameter("steer_idle_deadband", 0.02)
        self.declare_parameter("steer_idle_pwm_threshold", 0.02)
        self.declare_parameter("steer_center_offset", 0.0)
        self.declare_parameter("shutdown_center_hold_sec", 0.25)
        self.declare_parameter("use_pigpio", True)
        self.declare_parameter("steer_amount", 0.6)
        self.declare_parameter("steer_left_scale", 1.10)
        self.declare_parameter("steer_right_scale", 0.90)
        self.declare_parameter("accel_rate", 1.4)
        self.declare_parameter("decel_rate", 2.2)
        self.declare_parameter("steer_rate", 3.0)
        self.declare_parameter("heartbeat_timeout_ms", 500)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("health_report_sec", 2.0)
        self.declare_parameter("warn_cmd_stale_sec", 1.5)
        self.declare_parameter("stall_kill_sec", 0.0)
        # Self-restart triggers for silent hardware failures (e.g. brown-outs,
        # pigpio daemon drops). Zero disables each trigger.
        self.declare_parameter("hw_fail_streak_kill", 25)
        self.declare_parameter("hw_ok_kill_sec", 2.0)
        self.declare_parameter("health_topic", "/chassis/health")
        self.declare_parameter("health_publish_sec", 1.0)
        # Periodic GPIO reinit to recover from silent DMA/PWM stalls where
        # API calls succeed but the hardware peripheral stops generating
        # pulses (common after power supply glitches on Raspberry Pi).
        # Zero disables.
        self.declare_parameter("hw_reinit_sec", 30.0)

        backend = str(self.get_parameter("backend").value).strip().lower()
        strict_startup = bool(self.get_parameter("strict_startup").value)

        if backend == "real":
            try:
                self.hardware = RealChassisHardware(
                    pwm_pin=int(self.get_parameter("pwm_pin").value),
                    dir_pin=int(self.get_parameter("dir_pin").value),
                    steer_pin=int(self.get_parameter("steer_pin").value),
                    pwm_frequency=int(self.get_parameter("pwm_frequency").value),
                    servo_min_pulse=float(self.get_parameter("servo_min_pulse").value),
                    servo_max_pulse=float(self.get_parameter("servo_max_pulse").value),
                    servo_frequency_hz=int(self.get_parameter("servo_frequency_hz").value),
                    steer_pulse_deadband_us=int(
                        self.get_parameter("steer_pulse_deadband_us").value
                    ),
                    steer_idle_detach=bool(self.get_parameter("steer_idle_detach").value),
                    steer_idle_deadband=float(self.get_parameter("steer_idle_deadband").value),
                    steer_idle_pwm_threshold=float(
                        self.get_parameter("steer_idle_pwm_threshold").value
                    ),
                    steer_center_offset=float(self.get_parameter("steer_center_offset").value),
                    shutdown_center_hold_sec=float(
                        self.get_parameter("shutdown_center_hold_sec").value
                    ),
                    use_pigpio=bool(self.get_parameter("use_pigpio").value),
                )
                using_backend = "real"
            except Exception as exc:
                if strict_startup:
                    raise
                self.get_logger().warning(
                    f"Failed to initialize real chassis backend: {exc}. Falling back to mock backend."
                )
                self.hardware = FakeChassisHardware()
                using_backend = "mock"
        else:
            self.hardware = FakeChassisHardware()
            using_backend = "mock"

        self.core = ChassisCore(
            hardware=self.hardware,
            steer_amount=float(self.get_parameter("steer_amount").value),
            accel_rate=float(self.get_parameter("accel_rate").value),
            decel_rate=float(self.get_parameter("decel_rate").value),
            steer_rate=float(self.get_parameter("steer_rate").value),
            heartbeat_timeout_ms=int(self.get_parameter("heartbeat_timeout_ms").value),
            steer_left_scale=float(self.get_parameter("steer_left_scale").value),
            steer_right_scale=float(self.get_parameter("steer_right_scale").value),
        )

        now = time.monotonic()
        self._last_tick = now
        self._last_tick_ok = now
        self._last_hw_ok = now
        self._last_cmd_vel = now
        self._last_cmd_power = now
        self._last_outputs: object | None = None
        self._monitor_stop = threading.Event()
        self._monitor_thread: threading.Thread | None = None
        self._stall_kill_sec = max(0.0, float(self.get_parameter("stall_kill_sec").value))
        self._hw_fail_streak_kill = max(0, int(self.get_parameter("hw_fail_streak_kill").value))
        self._hw_ok_kill_sec = max(0.0, float(self.get_parameter("hw_ok_kill_sec").value))
        self._hw_reinit_sec = max(0.0, float(self.get_parameter("hw_reinit_sec").value))
        self._tick_exception_streak = 0
        self.get_logger().info(
            f"Configured chassis backend={using_backend} "
            f"pins: DIR={int(self.get_parameter('dir_pin').value)} "
            f"PWM={int(self.get_parameter('pwm_pin').value)} "
            f"STEER={int(self.get_parameter('steer_pin').value)} "
            f"PWM_Hz={int(self.get_parameter('pwm_frequency').value)} "
            f"servo_Hz={int(self.get_parameter('servo_frequency_hz').value)} "
            f"idle_detach={bool(self.get_parameter('steer_idle_detach').value)} "
            f"center_offset={float(self.get_parameter('steer_center_offset').value):+.3f} "
            f"left_scale={float(self.get_parameter('steer_left_scale').value):.2f} "
            f"right_scale={float(self.get_parameter('steer_right_scale').value):.2f} "
            f"heartbeat_ms={int(self.get_parameter('heartbeat_timeout_ms').value)} "
            f"health_report_sec={float(self.get_parameter('health_report_sec').value):.1f} "
            f"stall_kill_sec={self._stall_kill_sec:.1f} "
            f"hw_fail_streak_kill={self._hw_fail_streak_kill} "
            f"hw_ok_kill_sec={self._hw_ok_kill_sec:.1f} "
            f"hw_reinit_sec={self._hw_reinit_sec:.1f}"
        )

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Float32, "/cmd_power", self._on_cmd_power, 10)

        health_topic = str(self.get_parameter("health_topic").value).strip()
        self._health_pub = (
            self.create_publisher(String, health_topic, 10) if health_topic else None
        )

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / control_rate_hz, self._on_tick)
        health_report_sec = max(0.0, float(self.get_parameter("health_report_sec").value))
        if health_report_sec > 0.0:
            self.create_timer(health_report_sec, self._on_health_report)
        health_publish_sec = max(0.0, float(self.get_parameter("health_publish_sec").value))
        if health_publish_sec > 0.0 and self._health_pub is not None:
            self.create_timer(health_publish_sec, self._on_health_publish)
        if self._hw_reinit_sec > 0.0:
            self.create_timer(self._hw_reinit_sec, self._on_hw_reinit)
        monitor_enabled = (
            self._stall_kill_sec > 0.0
            or self._hw_fail_streak_kill > 0
            or self._hw_ok_kill_sec > 0.0
        )
        if monitor_enabled:
            self._monitor_thread = threading.Thread(
                target=self._stall_monitor_worker,
                name="chassis-stall-monitor",
                daemon=True,
            )
            self._monitor_thread.start()

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = time.monotonic()
        self.core.update_cmd_vel(msg.linear.x, msg.angular.z, now)
        self._last_cmd_vel = now

    def _on_cmd_power(self, msg: Float32) -> None:
        self.core.update_power(msg.data)
        self._last_cmd_power = time.monotonic()

    def _on_tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        try:
            outputs = self.core.tick(now=now, dt=dt)
        except BaseException as exc:
            # core.tick() should never raise (hardware writes are wrapped),
            # but treat any leak as a hard failure so we don't keep a zombie
            # node alive while telemetry looks fine.
            self._tick_exception_streak += 1
            self.get_logger().error(
                f"chassis tick raised: {type(exc).__name__}: {exc} "
                f"(streak={self._tick_exception_streak})"
            )
            return
        self._tick_exception_streak = 0
        self._last_outputs = outputs
        self._last_tick_ok = now
        if outputs.hardware_ok:
            self._last_hw_ok = now
        else:
            self.get_logger().error(
                "chassis hardware write failed: "
                f"{outputs.hardware_error} "
                f"(streak={self.core.hardware_fail_streak}, "
                f"total_fail={self.core.hardware_fail_total})"
            )
        self.get_logger().debug(
            f"dir={outputs.direction} duty={outputs.duty_cycle:.3f} steer={outputs.steering:.3f} timeout={outputs.timed_out}"
        )

    def _on_health_report(self) -> None:
        now = time.monotonic()
        tick_age = max(0.0, now - self._last_tick)
        tick_ok_age = max(0.0, now - self._last_tick_ok)
        hw_ok_age = max(0.0, now - self._last_hw_ok)
        cmd_vel_age = max(0.0, now - self._last_cmd_vel)
        cmd_power_age = max(0.0, now - self._last_cmd_power)
        warn_cmd_stale_sec = max(0.1, float(self.get_parameter("warn_cmd_stale_sec").value))
        health_report_sec = max(0.1, float(self.get_parameter("health_report_sec").value))

        outputs = self._last_outputs
        if outputs is None:
            return

        unhealthy = (
            tick_age > max(0.5, health_report_sec * 2.0)
            or tick_ok_age > max(0.5, health_report_sec * 2.0)
            or hw_ok_age > max(0.5, health_report_sec * 2.0)
            or self.core.hardware_fail_streak > 0
            or self._tick_exception_streak > 0
        )
        msg = (
            "health "
            f"tick_age={tick_age:.3f}s "
            f"tick_ok_age={tick_ok_age:.3f}s "
            f"hw_ok_age={hw_ok_age:.3f}s "
            f"cmd_vel_age={cmd_vel_age:.3f}s "
            f"cmd_power_age={cmd_power_age:.3f}s "
            f"warn_cmd_stale_sec={warn_cmd_stale_sec:.3f}s "
            f"timed_out={bool(outputs.timed_out)} "
            f"duty={float(outputs.duty_cycle):.3f} "
            f"steer={float(outputs.steering):.3f} "
            f"dir={bool(outputs.direction)} "
            f"hw_fail_streak={self.core.hardware_fail_streak} "
            f"hw_fail_total={self.core.hardware_fail_total} "
            f"hw_success_total={self.core.hardware_success_total} "
            f"tick_exc_streak={self._tick_exception_streak}"
        )
        if unhealthy:
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)

    def _on_hw_reinit(self) -> None:
        if not hasattr(self.hardware, "reinit"):
            return
        try:
            self.hardware.reinit()
            count = getattr(self.hardware, "reinit_count", "?")
            self.get_logger().info(
                f"periodic GPIO reinit completed (count={count})"
            )
        except BaseException as exc:
            self.get_logger().error(
                f"GPIO reinit failed: {type(exc).__name__}: {exc}"
            )

    def _on_health_publish(self) -> None:
        if self._health_pub is None or String is None:
            return
        now = time.monotonic()
        tick_age = max(0.0, now - self._last_tick)
        tick_ok_age = max(0.0, now - self._last_tick_ok)
        hw_ok_age = max(0.0, now - self._last_hw_ok)
        outputs = self._last_outputs
        hardware_ok = bool(getattr(outputs, "hardware_ok", True)) if outputs is not None else False
        payload = (
            f"tick_age={tick_age:.3f} "
            f"tick_ok_age={tick_ok_age:.3f} "
            f"hw_ok_age={hw_ok_age:.3f} "
            f"hw_fail_streak={self.core.hardware_fail_streak} "
            f"hw_fail_total={self.core.hardware_fail_total} "
            f"hw_success_total={self.core.hardware_success_total} "
            f"tick_exc_streak={self._tick_exception_streak} "
            f"hardware_ok={int(hardware_ok)}"
        )
        msg = String()
        msg.data = payload
        try:
            self._health_pub.publish(msg)
        except Exception:
            pass

    def _stall_monitor_worker(self) -> None:
        while not self._monitor_stop.is_set():
            time.sleep(0.2)
            now = time.monotonic()
            tick_age = max(0.0, now - self._last_tick)
            tick_ok_age = max(0.0, now - self._last_tick_ok)
            hw_ok_age = max(0.0, now - self._last_hw_ok)
            hw_fail_streak = self.core.hardware_fail_streak

            reason: str | None = None
            code = 42
            if self._stall_kill_sec > 0.0 and tick_age > self._stall_kill_sec:
                reason = (
                    f"executor stall: tick_age={tick_age:.3f}s "
                    f"> stall_kill_sec={self._stall_kill_sec:.3f}s"
                )
                code = 42
            elif self._stall_kill_sec > 0.0 and tick_ok_age > self._stall_kill_sec:
                reason = (
                    f"tick exceptions: tick_ok_age={tick_ok_age:.3f}s "
                    f"> stall_kill_sec={self._stall_kill_sec:.3f}s "
                    f"tick_exc_streak={self._tick_exception_streak}"
                )
                code = 43
            elif self._hw_ok_kill_sec > 0.0 and hw_ok_age > self._hw_ok_kill_sec:
                reason = (
                    f"hardware writes stuck: hw_ok_age={hw_ok_age:.3f}s "
                    f"> hw_ok_kill_sec={self._hw_ok_kill_sec:.3f}s "
                    f"last_err={self.core.last_hardware_error!r}"
                )
                code = 44
            elif (
                self._hw_fail_streak_kill > 0
                and hw_fail_streak >= self._hw_fail_streak_kill
            ):
                reason = (
                    f"hardware fail streak={hw_fail_streak} "
                    f">= hw_fail_streak_kill={self._hw_fail_streak_kill} "
                    f"last_err={self.core.last_hardware_error!r}"
                )
                code = 45

            if reason is not None:
                print(
                    f"[chassis_controller_node] self-exit ({code}): {reason}, "
                    "exiting for restart",
                    flush=True,
                )
                os._exit(code)

    def destroy_node(self) -> bool:
        self._monitor_stop.set()
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not installed")
    rclpy.init()
    node = ChassisControllerNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node.hardware, "close"):
            try:
                node.hardware.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
