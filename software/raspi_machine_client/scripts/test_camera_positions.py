#!/usr/bin/env python3
"""Interactive absolute-position tester for the camera PTZ axes."""

from __future__ import annotations

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rovr_interfaces.msg import CameraCommand
from std_msgs.msg import Bool


def resolve_config_path(path_str: str) -> str:
    if os.path.isabs(path_str):
        return path_str
    config_dir = os.environ.get("ROVR_CONFIG_DIR")
    if config_dir:
        return os.path.join(config_dir, os.path.basename(path_str))
    return os.path.abspath(path_str)


class CameraPositionCli(Node):
    def __init__(self) -> None:
        super().__init__("camera_position_cli")
        self.session_pub = self.create_publisher(Bool, "/camera/session_active", 10)
        self.target_pub = self.create_publisher(CameraCommand, "/camera/target_cmd", 10)
        self.create_subscription(CameraCommand, "/camera/current_positions", self._on_positions, 10)

        self.current_positions = [None, None]
        config_path = resolve_config_path("config/camera_limits.json")
        with open(config_path, "r", encoding="utf-8") as handle:
            self.config = json.load(handle)
        self.limits = [
            (int(self.config["1"]["min"]), int(self.config["1"]["max"])),
            (int(self.config["2"]["min"]), int(self.config["2"]["max"])),
        ]
        self.homes = [int(self.config["1"]["home"]), int(self.config["2"]["home"])]
        self.session_active = False
        self.command_target = list(self.homes)
        self.stream_target = False
        self.target_tolerance = 8
        self.create_timer(0.05, self._stream_tick)

    def _on_positions(self, msg: CameraCommand) -> None:
        self.current_positions = [int(value) for value in msg.servos]

    def spin_short(self, duration: float = 0.25) -> None:
        time.sleep(max(0.0, duration))

    def set_session(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.session_pub.publish(msg)
        self.session_active = bool(active)
        if not self.session_active:
            self.stream_target = False
        self.spin_short(0.3)

    def _publish_camera_command(self, values: list[int]) -> None:
        msg = CameraCommand()
        msg.servos = list(int(value) for value in values)
        self.target_pub.publish(msg)

    def _stream_tick(self) -> None:
        if not self.stream_target or not self.session_active:
            return
        if all(
            current is not None and abs(int(current) - int(target)) <= self.target_tolerance
            for current, target in zip(self.current_positions, self.command_target)
        ):
            print("Target reached. Stopping command stream.")
            self.stream_target = False
            self._publish_camera_command(self.command_target)
            return
        self._publish_camera_command(self.command_target)

    def print_status(self) -> None:
        print("")
        print("Camera axes:")
        names = ["pan", "tilt"]
        for index, name in enumerate(names):
            lower, upper = self.limits[index]
            home = self.homes[index]
            current = self.current_positions[index]
            print(
                f"  {name}: min={lower} max={upper} home={home} "
                f"current={current if current is not None else 'unknown'}"
            )
        print(f"  session_active={self.session_active} stream_active={self.stream_target}")
        print("")

    def publish_target(self, values: list[int], stream: bool = True) -> None:
        if not self.session_active:
            print("Arming camera session first...")
            self.set_session(True)
        clamped = []
        for value, (lower, upper) in zip(values, self.limits):
            clamped.append(max(lower, min(upper, int(value))))
        self.command_target = list(clamped)
        self.stream_target = bool(stream)
        self._publish_camera_command(self.command_target)
        self.spin_short(0.2)
        print(f"Target sent: {clamped} stream={self.stream_target}")
        if self.stream_target:
            print("Continuous target stream is active (20 Hz).")


def main() -> None:
    rclpy.init()
    node = CameraPositionCli()
    stop_spin = threading.Event()

    def _spin_worker() -> None:
        while rclpy.ok() and not stop_spin.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_worker, daemon=True)
    spin_thread.start()
    try:
        node.spin_short(0.5)
        print("Camera absolute-position tester")
        print("Commands:")
        print("  arm       -> enable camera session")
        print("  disarm    -> disable camera session")
        print("  show      -> print limits and current positions")
        print("  home      -> move to configured home")
        print("  hold      -> hold current camera pose (stop stream)")
        print("  <p1> <p2> -> move to absolute target positions")
        print("  quit      -> exit")
        print(f"Auto-stop tolerance: +/-{node.target_tolerance} ticks")
        node.print_status()

        while True:
            try:
                raw = input("camera> ").strip()
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
                node.publish_target(list(node.homes), stream=True)
                node.print_status()
                continue
            if raw == "hold":
                current = [
                    node.homes[index] if value is None else int(value)
                    for index, value in enumerate(node.current_positions)
                ]
                node.publish_target(current, stream=False)
                node.print_status()
                continue

            parts = raw.split()
            if len(parts) != 2:
                print("Expected exactly two integers, for example: 2048 2100")
                continue
            try:
                values = [int(parts[0]), int(parts[1])]
            except ValueError:
                print("Command values must be integers.")
                continue
            node.publish_target(values, stream=True)
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
