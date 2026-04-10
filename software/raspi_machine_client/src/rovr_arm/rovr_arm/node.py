"""ROS 2 arm controller node."""

from __future__ import annotations

import os
import time

from rovr_common.command_models import ArmTargetCommand
from rovr_common.fake_servo_bus import FakeServoBus
from rovr_common.servo_calibration import load_servo_calibrations
from rovr_common.st3215_bus import St3215ServoBus

from .control import ArmCore, ArmState, resolve_serial_port

try:
    import rclpy
    from rclpy.node import Node
    from rovr_interfaces.msg import ArmCommand, ArmTelemetry
    from std_msgs.msg import Bool, Int32, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = None
    ArmCommand = None
    ArmTelemetry = None
    Bool = None
    Int32 = None
    String = None


class _UnavailableNode:
    def __init__(self, *_: object, **__: object) -> None:
        raise RuntimeError("rclpy and generated interfaces are required to use this node")


BaseNode = Node if Node is not None else _UnavailableNode


def _scan_servo_ids(port: str, baudrate: int, id_min: int = 1, id_max: int = 16) -> list[int]:
    from st3215 import ST3215
    from st3215.values import COMM_SUCCESS

    dev = ST3215(str(port))
    try:
        dev.portHandler.closePort()
    except Exception:
        pass
    dev.portHandler.baudrate = int(baudrate)
    dev.portHandler.setupPort()

    low = int(min(id_min, id_max))
    high = int(max(id_min, id_max))
    live: list[int] = []
    for servo_id in range(low, high + 1):
        try:
            model, comm, err = dev.ping(int(servo_id))
            if comm == COMM_SUCCESS and err == 0 and model is not None:
                live.append(int(servo_id))
        except Exception:
            pass
    return live


class ArmControllerNode(BaseNode):
    """Subscribes to /arm/target_cmd and publishes /arm/telemetry.

    State transitions:
    - IDLE -> ACTIVE: first deadman=True command auto-arms all servos
    - ACTIVE -> HOMING: deadman=False or watchdog timeout
    - HOMING -> IDLE: arm reaches home positions, torque released
    - Any -> SAFE_MODE: jam or overheat detected
    - SAFE_MODE reset: /arm/reset_safe_mode topic (Bool True)
    """

    def __init__(self) -> None:
        super().__init__("arm_controller_node")

        # --- parameter declarations ---
        self.declare_parameter("backend", "mock")
        self.declare_parameter("strict_startup", False)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("fallback_serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("limits_file", "config/arm_limits.json")
        self.declare_parameter("ignore_limits", False)
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6])

        # Control loop
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("telemetry_rate_hz", 10.0)
        self.declare_parameter("ema_alpha", 0.05)
        self.declare_parameter("max_step_per_tick", 10)
        self.declare_parameter("position_deadband", 2)
        self.declare_parameter("home_deadband", 8)
        self.declare_parameter("home_release_torque", True)

        # Network watchdog
        self.declare_parameter("heartbeat_timeout_sec", 1.0)

        # Safety: temperature
        self.declare_parameter("temp_limit_celsius", 60.0)

        # Safety: jam detection
        self.declare_parameter("jam_load_threshold", 0.97)
        self.declare_parameter("jam_hold_time_sec", 3.0)
        self.declare_parameter("jam_target_delta_threshold", 96)
        self.declare_parameter("jam_no_motion_epsilon", 2)

        # Safety: overload guard
        self.declare_parameter("overload_guard_enabled", True)
        self.declare_parameter("overload_guard_hold_time_sec", 0.35)
        self.declare_parameter("overload_target_delta_threshold", 64)

        # Safety: load derate
        self.declare_parameter("load_derate_start", 0.65)
        self.declare_parameter("load_derate_end", 0.90)
        self.declare_parameter("load_min_step", 6)

        # Hardware profile
        self.declare_parameter("compliance_margin", 4)
        self.declare_parameter("position_speed", 2200)
        self.declare_parameter("position_acceleration", 80)
        self.declare_parameter("position_speed_per_servo", [2000, 1800, 1800, 1300, 1700, 1600])
        self.declare_parameter("position_acceleration_per_servo", [70, 60, 60, 40, 55, 50])
        self.declare_parameter("pid_p_per_servo", [34, 40, 42, 48, 44, 30])
        self.declare_parameter("pid_i_per_servo", [4, 6, 6, 8, 6, 2])
        self.declare_parameter("pid_d_per_servo", [24, 26, 28, 34, 30, 20])
        self.declare_parameter("max_torque_limit_per_servo", [1000, 1000, 1000, 1000, 1000, 900])
        self.declare_parameter("protection_current_per_servo", [900, 930, 940, 940, 920, 750])
        self.declare_parameter("overload_torque_per_servo", [130, 145, 150, 160, 145, 110])
        self.declare_parameter("protection_time_per_servo", [20, 25, 25, 30, 25, 20])
        self.declare_parameter("use_sync_write", True)
        self.declare_parameter("diag_poll_period_sec", 0.2)
        self.declare_parameter("verify_bus_ids", True)
        self.declare_parameter("bus_scan_min_id", 1)
        self.declare_parameter("bus_scan_max_id", 16)

        # --- build servo bus ---
        backend = str(self.get_parameter("backend").value)
        strict_startup = bool(self.get_parameter("strict_startup").value)
        selected_port = resolve_serial_port(
            str(self.get_parameter("serial_port").value),
            str(self.get_parameter("fallback_serial_port").value),
        )
        baudrate = int(self.get_parameter("baudrate").value)
        servo_ids = [int(v) for v in self.get_parameter("servo_ids").value]
        calibrations = load_servo_calibrations(
            str(self.get_parameter("limits_file").value), servo_ids
        )
        ignore_limits = bool(self.get_parameter("ignore_limits").value)

        if backend == "real" and any(item.center_raw is None for item in calibrations):
            self.get_logger().warning(
                "Arm calibration file missing center_raw entries. "
                "Run scripts/calibrate_arm.py to regenerate."
            )

        # ---- Seam-safety validation ----
        # STS3215 uses signed position error (goal − current).  Any joint whose
        # working range wraps through the raw 0/4095 boundary will reverse
        # direction when an EMA step crosses that boundary.  Detect and refuse.
        if not ignore_limits:
            seam_faults: list[str] = []
            for item in calibrations:
                if item.crosses_seam:
                    seam_v = item.seam_crossing_virtual()
                    safe_lo = item.safe_minimum()
                    safe_hi = item.safe_maximum()
                    seam_faults.append(
                        f"servo {item.servo_id}: seam at virtual={seam_v}  "
                        f"safe range=[{safe_lo}, {safe_hi}]  "
                        f"(current limits=[{item.minimum}, {item.maximum}])"
                    )
            if seam_faults:
                msg = (
                    "SEAM WARNING — raw encoder seam (0/4095) lies inside the "
                    "working range of one or more joints.  Motion toward the seam "
                    "boundary will cause the servo to reverse direction at full speed.  "
                    "Stay within the safe sub-range or recalibrate.  "
                    "Affected joints:\n  " + "\n  ".join(seam_faults)
                )
                self.get_logger().warning(msg)

        if ignore_limits:
            limits = [(0, 4095) for _ in servo_ids]
            homes = [2048 for _ in servo_ids]
        else:
            limits = [(int(item.minimum), int(item.maximum)) for item in calibrations]
            homes = [int(item.home) for item in calibrations]

        use_real_bus = backend == "real"
        if use_real_bus and not os.path.exists(selected_port):
            if strict_startup:
                raise RuntimeError(f"Arm serial port not found: {selected_port}")
            self.get_logger().warning(
                f"Arm serial port not found: {selected_port}. Falling back to FakeServoBus."
            )
            use_real_bus = False

        bus: St3215ServoBus | FakeServoBus
        if use_real_bus:
            try:
                if bool(self.get_parameter("verify_bus_ids").value):
                    live_ids = _scan_servo_ids(
                        selected_port,
                        baudrate,
                        id_min=int(self.get_parameter("bus_scan_min_id").value),
                        id_max=int(self.get_parameter("bus_scan_max_id").value),
                    )
                    expected = sorted(int(v) for v in servo_ids)
                    if sorted(live_ids) != expected:
                        raise RuntimeError(
                            f"Arm bus ID mismatch on {selected_port}: "
                            f"expected={expected}, detected={sorted(live_ids)}"
                        )

                bus = St3215ServoBus(
                    port=selected_port,
                    baudrate=baudrate,
                    servo_ids=servo_ids,
                    logger=self.get_logger(),
                    strict=strict_startup,
                    compliance_margin=int(self.get_parameter("compliance_margin").value),
                    position_speed=int(self.get_parameter("position_speed").value),
                    position_acceleration=int(self.get_parameter("position_acceleration").value),
                    position_speed_per_servo=[
                        int(v) for v in self.get_parameter("position_speed_per_servo").value
                    ],
                    position_acceleration_per_servo=[
                        int(v) for v in self.get_parameter("position_acceleration_per_servo").value
                    ],
                    pid_p_per_servo=[int(v) for v in self.get_parameter("pid_p_per_servo").value],
                    pid_i_per_servo=[int(v) for v in self.get_parameter("pid_i_per_servo").value],
                    pid_d_per_servo=[int(v) for v in self.get_parameter("pid_d_per_servo").value],
                    max_torque_limit_per_servo=[
                        int(v) for v in self.get_parameter("max_torque_limit_per_servo").value
                    ],
                    protection_current_per_servo=[
                        int(v) for v in self.get_parameter("protection_current_per_servo").value
                    ],
                    overload_torque_per_servo=[
                        int(v) for v in self.get_parameter("overload_torque_per_servo").value
                    ],
                    protection_time_per_servo=[
                        int(v) for v in self.get_parameter("protection_time_per_servo").value
                    ],
                    use_sync_write=bool(self.get_parameter("use_sync_write").value),
                    diag_poll_period_sec=float(self.get_parameter("diag_poll_period_sec").value),
                )
                bus.connect()
                bus.load_calibration(calibrations)
                bus.sync_goals_to_current()
            except Exception as exc:
                if strict_startup:
                    raise
                self.get_logger().warning(
                    f"Arm real bus startup failed on {selected_port}: {exc}. "
                    "Falling back to FakeServoBus."
                )
                use_real_bus = False
                bus = FakeServoBus(servo_ids=servo_ids, initial_positions=homes)
                if hasattr(bus, "connect"):
                    bus.connect()
                if hasattr(bus, "load_calibration"):
                    bus.load_calibration(calibrations)
        else:
            bus = FakeServoBus(servo_ids=servo_ids, initial_positions=homes)
            if hasattr(bus, "connect"):
                bus.connect()
            if hasattr(bus, "load_calibration"):
                bus.load_calibration(calibrations)

        if not ignore_limits and hasattr(bus, "get_command_limits"):
            limits = list(bus.get_command_limits())
        if hasattr(bus, "get_home_positions"):
            homes = list(bus.get_home_positions())

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        telemetry_rate_hz = float(self.get_parameter("telemetry_rate_hz").value)

        self.core = ArmCore(
            servo_ids=servo_ids,
            limits=limits,
            home_positions=homes,
            bus=bus,
            heartbeat_timeout_sec=float(self.get_parameter("heartbeat_timeout_sec").value),
            control_rate_hz=control_rate_hz,
            ema_alpha=float(self.get_parameter("ema_alpha").value),
            max_step_per_tick=int(self.get_parameter("max_step_per_tick").value),
            position_deadband=int(self.get_parameter("position_deadband").value),
            home_deadband=int(self.get_parameter("home_deadband").value),
            home_release_torque=bool(self.get_parameter("home_release_torque").value),
            temp_limit_celsius=float(self.get_parameter("temp_limit_celsius").value),
            jam_load_threshold=float(self.get_parameter("jam_load_threshold").value),
            jam_hold_time_sec=float(self.get_parameter("jam_hold_time_sec").value),
            jam_target_delta_threshold=int(self.get_parameter("jam_target_delta_threshold").value),
            jam_no_motion_epsilon=int(self.get_parameter("jam_no_motion_epsilon").value),
            overload_guard_enabled=bool(self.get_parameter("overload_guard_enabled").value),
            overload_guard_hold_time_sec=float(
                self.get_parameter("overload_guard_hold_time_sec").value
            ),
            overload_target_delta_threshold=int(
                self.get_parameter("overload_target_delta_threshold").value
            ),
            load_derate_start=float(self.get_parameter("load_derate_start").value),
            load_derate_end=float(self.get_parameter("load_derate_end").value),
            load_min_step=int(self.get_parameter("load_min_step").value),
        )

        self.get_logger().info(
            f"Arm controller ready: backend={'real' if use_real_bus else 'mock'} "
            f"port={selected_port} @ {baudrate} baud "
            f"rate={control_rate_hz}Hz alpha={float(self.get_parameter('ema_alpha').value)} "
            f"max_step={int(self.get_parameter('max_step_per_tick').value)} "
            f"compliance_margin={int(self.get_parameter('compliance_margin').value)} "
            f"temp_limit={float(self.get_parameter('temp_limit_celsius').value)}C"
        )

        self.telemetry_pub = self.create_publisher(ArmTelemetry, "/arm/telemetry", 10)
        self.create_subscription(ArmCommand, "/arm/target_cmd", self._on_target_cmd, 10)
        self.create_subscription(Bool, "/arm/session_active", self._on_session_active, 10)
        self.create_subscription(Int32, "/arm/arm_servo", self._on_arm_servo, 10)
        self.create_subscription(Bool, "/arm/reset_safe_mode", self._on_reset_safe_mode, 10)

        self._last_telemetry = 0.0
        self._telemetry_period = 1.0 / telemetry_rate_hz
        self.create_timer(1.0 / control_rate_hz, self._on_tick)

    # -------------------------------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------------------------------

    def _on_target_cmd(self, msg: ArmCommand) -> None:
        command = ArmTargetCommand(
            timestamp=time.monotonic(),
            deadman_switch=bool(msg.deadman_switch),
            servos=tuple(int(v) for v in msg.servos),
        )
        self.core.apply_command(command, now=command.timestamp)

    def _on_session_active(self, msg: Bool) -> None:
        """Explicit arm/disarm via session_active topic (legacy support)."""
        if bool(msg.data):
            armed = self.core.arm_all()
            self.get_logger().info(f"Session active; armed servos: {armed}")
        else:
            self.core.disarm_all()
            self.get_logger().info("Session inactive; disarmed all servos")

    def _on_arm_servo(self, msg: Int32) -> None:
        servo_id = int(msg.data)
        armed = self.core.arm_servo(servo_id)
        if armed:
            self.get_logger().info(f"Armed arm servo {servo_id}")
        else:
            self.get_logger().warning(f"Failed to arm arm servo {servo_id}")

    def _on_reset_safe_mode(self, msg: Bool) -> None:
        if bool(msg.data):
            self.get_logger().info("Safe mode reset requested; returning to IDLE")
            self.core.reset_safe_mode()

    def _on_tick(self) -> None:
        result = self.core.tick(now=time.monotonic())

        # Log state transitions
        status = result.telemetry.status
        if "safe_mode" in status:
            self.get_logger().warning(f"Arm status: {status}", throttle_duration_sec=5.0)

        now = time.monotonic()
        if (now - self._last_telemetry) < self._telemetry_period:
            return
        self._last_telemetry = now

        msg = ArmTelemetry()
        msg.timestamp = float(result.telemetry.timestamp)
        msg.positions = list(result.telemetry.positions)
        msg.loads = [float(v) for v in result.telemetry.loads]
        msg.temperatures = [float(v) for v in result.telemetry.temperatures]
        msg.jam_detected = list(result.telemetry.jam_detected)
        msg.torque_enabled = list(result.telemetry.torque_enabled)
        msg.status = result.telemetry.status
        self.telemetry_pub.publish(msg)


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not installed")
    rclpy.init()
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.disarm_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
