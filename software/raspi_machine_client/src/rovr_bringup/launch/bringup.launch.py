"""Launch the core ROVR ROS controllers (chassis + arm + camera)."""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config_path(filename: str) -> str:
    config_root = os.environ.get("ROVR_CONFIG_DIR")
    if not config_root:
        config_root = str(Path(__file__).resolve().parents[3] / "config")
    return str(Path(config_root) / filename)


def generate_launch_description() -> LaunchDescription:
    hardware = LaunchConfiguration("hardware")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware",
                default_value="real",
                description="Backend mode: 'real' for hardware, 'mock' for simulated.",
            ),
            Node(
                package="rovr_chassis",
                executable="chassis_controller_node",
                name="chassis_controller_node",
                parameters=[_config_path("chassis.yaml"), {"backend": hardware}],
            ),
            Node(
                package="rovr_arm",
                executable="arm_controller_node",
                name="arm_controller_node",
                parameters=[_config_path("arm.yaml"), {"backend": hardware}],
            ),
            Node(
                package="rovr_camera",
                executable="camera_ptz_node",
                name="camera_ptz_node",
                parameters=[_config_path("camera.yaml"), {"backend": hardware}],
            ),
        ]
    )
