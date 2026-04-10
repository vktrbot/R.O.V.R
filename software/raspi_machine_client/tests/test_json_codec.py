"""Tests for bridge payload normalization."""

from __future__ import annotations

import pytest

from rovr_common.command_models import ArmTelemetryData
from rovr_common.json_codec import CommandValidationError, parse_control_frame, serialize_arm_telemetry


def test_parse_control_frame_normalizes_ranges() -> None:
    frame = parse_control_frame(
        {
            "type": "control_frame",
            "ts": 1710000000.123,
            "drive": {"linear": 4, "angular": -2, "power": 3},
            "arm": {"deadman": True, "servos": [0, 2048, 4096, 100, 200, 300]},
            "camera": {"servos": [5000, -10]},
        }
    )

    assert frame.drive is not None
    assert frame.drive.linear == 1.0
    assert frame.drive.angular == -1.0
    assert frame.drive.power == 0.9
    assert frame.arm is not None
    assert frame.arm.servos[2] == 4095
    assert frame.camera is not None
    assert frame.camera.servos == (4095, 0)


def test_parse_control_frame_rejects_wrong_servo_count() -> None:
    with pytest.raises(CommandValidationError):
        parse_control_frame(
            {
                "type": "control_frame",
                "ts": 1.0,
                "arm": {"deadman": True, "servos": [1, 2, 3]},
            }
        )


def test_parse_control_frame_maps_symbolic_gripper_state_to_sixth_servo() -> None:
    frame = parse_control_frame(
        {
            "type": "control_frame",
            "ts": 1.0,
            "arm": {
                "deadman": True,
                "servos": [1000, 1100, 1200, 1300, 1400],
                "gripper": "open",
            },
        },
        gripper_open_tick=868,
        gripper_closed_tick=2314,
    )

    assert frame.arm is not None
    assert frame.arm.servos == (1000, 1100, 1200, 1300, 1400, 868)


def test_parse_control_frame_rejects_symbolic_gripper_without_presets() -> None:
    with pytest.raises(CommandValidationError):
        parse_control_frame(
            {
                "type": "control_frame",
                "ts": 1.0,
                "arm": {
                    "deadman": True,
                    "servos": [1000, 1100, 1200, 1300, 1400],
                    "gripper": "closed",
                },
            }
        )


def test_parse_drive_message_type_is_supported() -> None:
    frame = parse_control_frame(
        {
            "type": "drive",
            "ts": 2.0,
            "drive": {"linear": 1, "angular": 0.2, "power": 0.4},
        }
    )
    assert frame.drive is not None
    assert frame.drive.linear == 1.0
    assert frame.drive.angular == 0.2
    assert frame.drive.power == 0.4


def test_parse_camera_message_accepts_yaw_pitch_degrees() -> None:
    # Use symmetric limits so that 0.0 deg maps to centre tick 2048
    frame = parse_control_frame(
        {
            "type": "camera",
            "ts": 3.0,
            "camera": {"yaw_deg": 0.0, "pitch_deg": 0.0},
        },
        camera_yaw_deg_limits=(-180.0, 180.0),
        camera_pitch_deg_limits=(-180.0, 180.0),
    )
    assert frame.camera is not None
    assert frame.camera.servos == (2048, 2048)


def test_parse_arm_message_accepts_joint_degrees_and_gripper_symbolic() -> None:
    frame = parse_control_frame(
        {
            "type": "arm",
            "ts": 4.0,
            "arm": {
                "deadman": True,
                "joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0],
                "gripper": "closed",
            },
        },
        gripper_open_tick=868,
        gripper_closed_tick=2314,
    )
    assert frame.arm is not None
    assert frame.arm.servos == (2048, 2048, 2048, 2048, 2048, 2314)


def test_serialize_arm_telemetry_matches_expected_shape() -> None:
    payload = serialize_arm_telemetry(
        ArmTelemetryData(
            timestamp=1.5,
            positions=(1, 2, 3, 4, 5, 6),
            loads=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
            temperatures=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            jam_detected=(False, False, True, False, False, False),
            torque_enabled=(True, True, False, True, True, True),
            status="jam_detected",
        )
    )

    assert payload["type"] == "arm_telemetry"
    assert payload["positions"] == [1, 2, 3, 4, 5, 6]
    assert payload["jam_detected"][2] is True
