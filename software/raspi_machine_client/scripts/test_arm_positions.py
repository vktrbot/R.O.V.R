#!/usr/bin/env python3
"""Interactive absolute-position tester for the six arm axes."""

from __future__ import annotations

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rovr_interfaces.msg import ArmCommand, ArmTelemetry
from std_msgs.msg import Bool


def resolve_config_path(path_str: str) -> str:
    if os.path.isabs(path_str):
        return path_str
    config_dir = os.environ.get("ROVR_CONFIG_DIR")
    if config_dir:
        return os.path.join(config_dir, os.path.basename(path_str))
    return os.path.abspath(path_str)


class ArmPositionCli(Node):
    def __init__(self) -> None:
        super().__init__("arm_position_cli")
        self.session_pub = self.create_publisher(Bool, "/arm/session_active", 10)
        self.target_pub = self.create_publisher(ArmCommand, "/arm/target_cmd", 10)
        self.create_subscription(ArmTelemetry, "/arm/telemetry", self._on_telemetry, 10)

        self.current_positions = [None] * 6
        self.jam_detected = [False] * 6
        self.status = "unknown"
        self.target_tolerance = 8
        config_path = resolve_config_path("config/arm_limits.json")
        with open(config_path, "r", encoding="utf-8") as handle:
            self.config = json.load(handle)
        self.limits = []
        self.homes = []
        for servo_id in range(1, 7):
            item = self.config[str(servo_id)]
            self.limits.append((int(item["min"]), int(item["max"])))
            self.homes.append(int(item["home"]))
        self.session_active = False
        self.command_target = list(self.homes)
        self.stream_deadman = False
        self.create_timer(0.1, self._stream_tick)

    def _on_telemetry(self, msg: ArmTelemetry) -> None:
        self.current_positions = [int(value) for value in msg.positions]
        self.jam_detected = [bool(value) for value in msg.jam_detected]
        self.status = str(msg.status)

    def spin_short(self, duration: float = 0.25) -> None:
        time.sleep(max(0.0, duration))

    def set_session(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.session_pub.publish(msg)
        self.session_active = bool(active)
        if not self.session_active:
            self.stream_deadman = False
        self.spin_short(0.3)

    def _publish_arm_command(self, values: list[int], deadman: bool) -> None:
        msg = ArmCommand()
        msg.timestamp = time.time()
        msg.deadman_switch = bool(deadman)
        msg.servos = list(values)
        self.target_pub.publish(msg)

    def _stream_tick(self) -> None:
        if not self.stream_deadman or not self.session_active:
            return
        if any(self.jam_detected):
            print("Jam detected. Deadman stream stopped.")
            self.stream_deadman = False
            self._publish_arm_command(self.command_target, deadman=False)
            return
        if all(
            current is not None
            and abs(int(current) - int(target)) <= self.target_tolerance
            for current, target in zip(self.current_positions, self.command_target)
        ):
            print("Target reached. Stopping deadman stream.")
            self.stream_deadman = False
            self._publish_arm_command(self.command_target, deadman=False)
            return
        self._publish_arm_command(self.command_target, deadman=True)

    def publish_target(self, values: list[int], deadman: bool) -> None:
        if deadman and not self.session_active:
            print("Arming arm session first...")
            self.set_session(True)
        clamped = []
        for value, (lower, upper) in zip(values, self.limits):
            clamped.append(max(lower, min(upper, int(value))))
        self.command_target = list(clamped)
        self.stream_deadman = bool(deadman)
        self._publish_arm_command(self.command_target, deadman=deadman)
        self.spin_short(0.2)
        print(f"Target sent: deadman={deadman} servos={clamped}")
        if deadman:
            print("Continuous deadman stream is active (10 Hz).")

    def print_status(self) -> None:
        print("")
        print("Arm axes:")
        for index in range(6):
            lower, upper = self.limits[index]
            home = self.homes[index]
            current = self.current_positions[index]
            jam = self.jam_detected[index]
            print(
                f"  joint_{index + 1}: min={lower} max={upper} home={home} "
                f"current={current if current is not None else 'unknown'} jam={jam}"
            )
        print(f"  session_active={self.session_active} status={self.status}")
        print("")


def main() -> None:
    rclpy.init()
    node = ArmPositionCli()
    stop_spin = threading.Event()

    def _spin_worker() -> None:
        while rclpy.ok() and not stop_spin.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_worker, daemon=True)
    spin_thread.start()
    try:
        node.spin_short(0.5)
        print("Arm absolute-position tester")
        print("Commands:")
        print("  arm              -> enable arm session")
        print("  disarm           -> disable arm session")
        print("  show             -> print limits and current positions")
        print("  home             -> move all joints to configured home")
        print("  hold             -> publish deadman=false and hold current pose")
        print("  <6 ints>         -> move to absolute target positions")
        print("  quit             -> exit")
        print(f"Auto-stop tolerance: +/-{node.target_tolerance} ticks")
        node.print_status()

        while True:
            try:
                raw = input("arm> ").strip()
            except EOFError:
                break
            if not raw:
                node.spin_short(0.1)
                continue
            if raw in {"quit", "exit"}:
                break
            if raw == "show":
                node.spin_short(0.2)
                node.print_status()
                continue
            if raw == "arm":
                node.set_session(True)
                node.print_status()
                continue
            if raw == "disarm":
                node.set_session(False)
                node.print_status()
                continue
            if raw == "home":
                node.publish_target(list(node.homes), deadman=True)
                node.print_status()
                continue
            if raw == "hold":
                current = [
                    node.homes[index] if value is None else int(value)
                    for index, value in enumerate(node.current_positions)
                ]
                node.publish_target(current, deadman=False)
                node.print_status()
                continue

            parts = raw.split()
            if len(parts) != 6:
                print("Expected six integers, for example: 2000 2100 2200 2300 2400 2500")
                continue
            try:
                values = [int(item) for item in parts]
            except ValueError:
                print("Command values must be integers.")
                continue
            node.publish_target(values, deadman=True)
            node.print_status()
    finally:
        stop_spin.set()
        spin_thread.join(timeout=1.0)
        try:
            node.set_session(False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
