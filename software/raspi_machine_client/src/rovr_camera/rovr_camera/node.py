"""ROS 2 camera PTZ controller node."""

from __future__ import annotations

import os
import time

from rovr_common.fake_servo_bus import FakeServoBus
from rovr_common.servo_calibration import load_servo_calibrations
from rovr_common.st3215_bus import St3215ServoBus

from rovr_arm.control import resolve_serial_port
from .control import CameraCore

try:
    import rclpy
    from rclpy.node import Node
    from rovr_interfaces.msg import CameraCommand
    from std_msgs.msg import Bool, Int32, Int32MultiArray
except ImportError:  # pragma: no cover - allows importing without ROS installed
    rclpy = None
    Node = None
    CameraCommand = None
    Bool = None
    Int32 = None
    Int32MultiArray = None


class _UnavailableNode:
    def __init__(self, *_: object, **__: object) -> None:
        raise RuntimeError("rclpy and generated interfaces are required to use this node")


BaseNode = Node if Node is not None else _UnavailableNode


def _apply_safety_margin_to_limits_homes(
    *,
    limits: list[tuple[int, int]],
    homes: list[int],
    calibrations: list[object] | None,
    margin_ticks: int,
    wrap_guard_ticks: int,
) -> tuple[list[tuple[int, int]], list[int]]:
    margin = max(0, int(margin_ticks))
    wrap_guard = max(0, int(wrap_guard_ticks))
    adjusted_limits: list[tuple[int, int]] = []
    adjusted_homes: list[int] = []
    for index, (low_raw, high_raw) in enumerate(limits):
        low = int(low_raw)
        high = int(high_raw)
        home = int(homes[index])
        if high > low and (high - low) > (2 * margin):
            low += margin
            high -= margin
        if calibrations is not None and index < len(calibrations):
            calibration = calibrations[index]
            center_raw = getattr(calibration, "center_raw", None)
            center_virtual = int(getattr(calibration, "center_virtual", 2048))
            if center_raw is not None:
                center_raw_i = int(center_raw)
                high_wrap_threshold = center_virtual + (4096 - center_raw_i)
                safe_high = high_wrap_threshold - 1 - wrap_guard
                low_wrap_threshold = center_virtual - center_raw_i
                safe_low = low_wrap_threshold + wrap_guard
                if safe_high >= low and high > safe_high:
                    high = safe_high
                if safe_low <= high and low < safe_low:
                    low = safe_low
        home = int(max(low, min(high, home)))
        adjusted_limits.append((low, high))
        adjusted_homes.append(home)
    return adjusted_limits, adjusted_homes


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


class CameraPtzNode(BaseNode):
    """Subscribes to PTZ target commands and steps the camera servos."""

    def __init__(self) -> None:
        super().__init__("camera_ptz_node")

        self.declare_parameter("backend", "mock")
        self.declare_parameter("strict_startup", False)
        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("fallback_serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("limits_file", "config/camera_limits.json")
        self.declare_parameter("ignore_limits", False)
        self.declare_parameter("servo_ids", [1, 2])
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("ema_alpha", 0.5)
        self.declare_parameter("max_relative_target", 96)
        self.declare_parameter("target_snap_delta", 32)
        self.declare_parameter("position_speed", 3000)
        self.declare_parameter("position_acceleration", 150)
        self.declare_parameter("position_speed_per_servo", [3000, 2600])
        self.declare_parameter("position_acceleration_per_servo", [150, 130])
        self.declare_parameter("pid_p_per_servo", [28, 30])
        self.declare_parameter("pid_i_per_servo", [2, 2])
        self.declare_parameter("pid_d_per_servo", [30, 34])
        self.declare_parameter("max_torque_limit_per_servo", [1000, 1000])
        self.declare_parameter("protection_current_per_servo", [820, 820])
        self.declare_parameter("overload_torque_per_servo", [120, 120])
        self.declare_parameter("protection_time_per_servo", [12, 12])
        self.declare_parameter("use_sync_write", True)
        self.declare_parameter("diag_poll_period_sec", 0.4)
        self.declare_parameter("home_on_startup", True)
        self.declare_parameter("startup_home_timeout_sec", 1.5)
        self.declare_parameter("safety_margin_ticks", 80)
        self.declare_parameter("wrap_guard_ticks", 8)
        self.declare_parameter("verify_bus_ids", True)
        self.declare_parameter("bus_scan_min_id", 1)
        self.declare_parameter("bus_scan_max_id", 16)

        backend = str(self.get_parameter("backend").value)
        selected_port = resolve_serial_port(
            str(self.get_parameter("serial_port").value),
            str(self.get_parameter("fallback_serial_port").value),
        )
        baudrate = int(self.get_parameter("baudrate").value)
        servo_ids = [int(value) for value in self.get_parameter("servo_ids").value]
        calibrations = load_servo_calibrations(str(self.get_parameter("limits_file").value), servo_ids)
        ignore_limits = bool(self.get_parameter("ignore_limits").value)
        safety_margin_ticks = max(0, int(self.get_parameter("safety_margin_ticks").value))
        wrap_guard_ticks = max(0, int(self.get_parameter("wrap_guard_ticks").value))
        if backend == "real" and any(item.center_raw is None for item in calibrations):
            self.get_logger().warning(
                "Camera calibration file is in legacy/raw mode (missing center_raw). "
                "Use safe raw-step testing or re-run calibration with the updated tool."
            )
        if ignore_limits:
            limits = [(0, 4095) for _ in servo_ids]
            homes = [2048 for _ in servo_ids]
        else:
            limits = [(int(item.minimum), int(item.maximum)) for item in calibrations]
            homes = [int(item.home) for item in calibrations]
            limits, homes = _apply_safety_margin_to_limits_homes(
                limits=limits,
                homes=homes,
                calibrations=calibrations,
                margin_ticks=safety_margin_ticks,
                wrap_guard_ticks=wrap_guard_ticks,
            )
        strict_startup = bool(self.get_parameter("strict_startup").value)
        use_real_bus = backend == "real"
        if use_real_bus and not os.path.exists(selected_port):
            if strict_startup:
                raise RuntimeError(f"Camera serial port not found: {selected_port}")
            self.get_logger().warning(
                f"Camera serial port not found: {selected_port}. Falling back to FakeServoBus."
            )
            use_real_bus = False

        if use_real_bus:
            try:
                if bool(self.get_parameter("verify_bus_ids").value):
                    live_ids = _scan_servo_ids(
                        selected_port,
                        baudrate,
                        id_min=int(self.get_parameter("bus_scan_min_id").value),
                        id_max=int(self.get_parameter("bus_scan_max_id").value),
                    )
                    expected_ids = sorted(int(value) for value in servo_ids)
                    if sorted(live_ids) != expected_ids:
                        raise RuntimeError(
                            f"Camera bus ID mismatch on {selected_port}: "
                            f"expected={expected_ids}, detected={sorted(live_ids)}"
                        )

                bus = St3215ServoBus(
                    port=selected_port,
                    baudrate=baudrate,
                    servo_ids=servo_ids,
                    logger=self.get_logger(),
                    strict=strict_startup,
                    position_speed=int(self.get_parameter("position_speed").value),
                    position_acceleration=int(self.get_parameter("position_acceleration").value),
                    position_speed_per_servo=[
                        int(value) for value in self.get_parameter("position_speed_per_servo").value
                    ],
                    position_acceleration_per_servo=[
                        int(value) for value in self.get_parameter("position_acceleration_per_servo").value
                    ],
                    pid_p_per_servo=[
                        int(value) for value in self.get_parameter("pid_p_per_servo").value
                    ],
                    pid_i_per_servo=[
                        int(value) for value in self.get_parameter("pid_i_per_servo").value
                    ],
                    pid_d_per_servo=[
                        int(value) for value in self.get_parameter("pid_d_per_servo").value
                    ],
                    max_torque_limit_per_servo=[
                        int(value) for value in self.get_parameter("max_torque_limit_per_servo").value
                    ],
                    protection_current_per_servo=[
                        int(value) for value in self.get_parameter("protection_current_per_servo").value
                    ],
                    overload_torque_per_servo=[
                        int(value) for value in self.get_parameter("overload_torque_per_servo").value
                    ],
                    protection_time_per_servo=[
                        int(value) for value in self.get_parameter("protection_time_per_servo").value
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
                    f"Camera real bus startup failed on {selected_port}: {exc}. "
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
            limits, homes = _apply_safety_margin_to_limits_homes(
                limits=[(int(low), int(high)) for low, high in limits],
                homes=[int(value) for value in homes],
                calibrations=calibrations,
                margin_ticks=safety_margin_ticks,
                wrap_guard_ticks=wrap_guard_ticks,
            )
        if hasattr(bus, "get_home_positions"):
            homes = list(bus.get_home_positions())
            if not ignore_limits:
                homes = [
                    int(max(limits[index][0], min(limits[index][1], int(value))))
                    for index, value in enumerate(homes)
                ]
        self.core = CameraCore(
            limits=limits,
            bus=bus,
            ema_alpha=float(self.get_parameter("ema_alpha").value),
            max_relative_target=int(self.get_parameter("max_relative_target").value),
            target_snap_delta=int(self.get_parameter("target_snap_delta").value),
        )
        self.get_logger().info(
            f"Configured camera backend={'real' if use_real_bus else 'mock'} "
            f"serial path: {selected_port} @ {baudrate} baud "
            f"(ignore_limits={ignore_limits}, "
            f"speed={int(self.get_parameter('position_speed').value)}, "
            f"acc={int(self.get_parameter('position_acceleration').value)}, "
            f"max_step={int(self.get_parameter('max_relative_target').value)}, "
            f"sync_write={bool(self.get_parameter('use_sync_write').value)}, "
            f"safety_margin_ticks={safety_margin_ticks}, "
            f"wrap_guard_ticks={wrap_guard_ticks})"
        )

        self._home_positions = [int(value) for value in homes]
        if bool(self.get_parameter("home_on_startup").value):
            self._move_to_home_on_startup(float(self.get_parameter("startup_home_timeout_sec").value))

        self.current_positions_pub = self.create_publisher(CameraCommand, "/camera/current_positions", 10)
        self.armed_states_pub = self.create_publisher(Int32MultiArray, "/camera/armed_states", 10)
        self.create_subscription(CameraCommand, "/camera/target_cmd", self._on_target_cmd, 10)
        self.create_subscription(Bool, "/camera/session_active", self._on_session_active, 10)
        self.create_subscription(Int32, "/camera/arm_servo", self._on_arm_servo, 10)

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / control_rate_hz, self._on_tick)

    def _on_target_cmd(self, msg: CameraCommand) -> None:
        command = type("Command", (), {})()
        command.servos = tuple(int(value) for value in msg.servos)
        self.core.apply_command(command)

    def _on_session_active(self, msg: Bool) -> None:
        if bool(msg.data):
            armed = self.core.arm_all()
            self.get_logger().info(f"Camera session active; armed servos: {armed}")
            return
        self.core.disarm_all()
        self.get_logger().info("Camera session inactive; disarmed all servos")

    def _on_arm_servo(self, msg: Int32) -> None:
        servo_id = int(msg.data)
        armed = self.core.arm_servo(servo_id)
        if armed:
            self.get_logger().info(f"Armed camera servo {servo_id}")
        else:
            self.get_logger().warning(f"Failed to arm camera servo {servo_id}")

    def _on_tick(self) -> None:
        positions = self.core.tick()
        msg = CameraCommand()
        msg.servos = list(int(value) for value in positions)
        self.current_positions_pub.publish(msg)
        armed_msg = Int32MultiArray()
        if hasattr(self.core.bus, "armed_states"):
            armed_msg.data = [1 if value else 0 for value in self.core.bus.armed_states()]
        else:
            armed_msg.data = []
        self.armed_states_pub.publish(armed_msg)
        self.get_logger().debug(f"camera_positions={positions}")

    def _move_to_home_on_startup(self, timeout_sec: float) -> None:
        if not self._home_positions:
            return

        armed = self.core.arm_all()
        if not armed:
            self.get_logger().warning("Startup homing skipped: no camera servos were armed")
            return

        command = type("Command", (), {})()
        command.servos = tuple(int(value) for value in self._home_positions)
        self.core.apply_command(command)

        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline:
            self.core.tick()
            time.sleep(0.02)

        self.get_logger().info(f"Camera moved to startup home: {self._home_positions}")


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not installed")
    rclpy.init()
    node = CameraPtzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node.core, "disarm_all"):
            node.core.disarm_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
