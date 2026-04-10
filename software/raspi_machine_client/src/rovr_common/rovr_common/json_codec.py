"""JSON parsing and serialization for bridge traffic."""

from __future__ import annotations

import json
from typing import Any, Mapping

from .clamp import clamp, clamp_int
from .command_models import ArmTargetCommand, ArmTelemetryData, CameraTargetCommand, DriveCommand, NormalizedControlFrame


class CommandValidationError(ValueError):
    """Raised when a control frame cannot be normalized."""


DEFAULT_ARM_JOINT_DEG_LIMITS: tuple[tuple[float, float], ...] = (
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
)
DEFAULT_CAMERA_YAW_DEG_LIMITS: tuple[float, float] = (-140.0, 145.0)
DEFAULT_CAMERA_PITCH_DEG_LIMITS: tuple[float, float] = (-70.0, 27.0)


def parse_control_frame(
    payload: str | bytes | Mapping[str, Any],
    *,
    gripper_open_tick: int | None = None,
    gripper_closed_tick: int | None = None,
    arm_joint_deg_limits: tuple[tuple[float, float], ...] = DEFAULT_ARM_JOINT_DEG_LIMITS,
    camera_yaw_deg_limits: tuple[float, float] = DEFAULT_CAMERA_YAW_DEG_LIMITS,
    camera_pitch_deg_limits: tuple[float, float] = DEFAULT_CAMERA_PITCH_DEG_LIMITS,
    camera_pan_calibration: tuple[int, int, int] | None = None,
    camera_tilt_calibration: tuple[int, int, int] | None = None,
) -> NormalizedControlFrame:
    """Normalize a Unity control payload into stable internal models."""
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    if isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, Mapping):
        raise CommandValidationError("Control payload must be a JSON object.")

    message_type = payload.get("type")
    if message_type not in {"control_frame", "drive", "camera", "arm"}:
        raise CommandValidationError(f"Unsupported payload type: {message_type!r}")

    timestamp = _coerce_float(payload.get("ts", 0.0), "ts")

    required_block = None
    if message_type in {"drive", "camera", "arm"}:
        required_block = message_type
        if payload.get(required_block) is None:
            raise CommandValidationError(f"Payload type {message_type!r} requires '{required_block}' block")

    drive = _parse_drive(payload.get("drive"))
    arm = _parse_arm(
        payload.get("arm"),
        timestamp,
        gripper_open_tick=gripper_open_tick,
        gripper_closed_tick=gripper_closed_tick,
        arm_joint_deg_limits=arm_joint_deg_limits,
    )
    camera = _parse_camera(
        payload.get("camera"),
        camera_yaw_deg_limits=camera_yaw_deg_limits,
        camera_pitch_deg_limits=camera_pitch_deg_limits,
        camera_pan_calibration=camera_pan_calibration,
        camera_tilt_calibration=camera_tilt_calibration,
    )

    if drive is None and arm is None and camera is None:
        raise CommandValidationError("Control frame must contain at least one command block.")

    return NormalizedControlFrame(timestamp=timestamp, drive=drive, arm=arm, camera=camera)


def serialize_arm_telemetry(telemetry: ArmTelemetryData) -> dict[str, Any]:
    """Convert arm telemetry to the outbound Unity payload shape."""
    return {
        "type": "arm_telemetry",
        "ts": telemetry.timestamp,
        "positions": list(telemetry.positions),
        "loads": [float(value) for value in telemetry.loads],
        "temperatures": [float(value) for value in telemetry.temperatures],
        "jam_detected": list(telemetry.jam_detected),
        "torque_enabled": list(telemetry.torque_enabled),
        "status": telemetry.status,
    }


def _parse_drive(raw_drive: Any) -> DriveCommand | None:
    if raw_drive is None:
        return None
    if not isinstance(raw_drive, Mapping):
        raise CommandValidationError("drive must be an object")
    linear = clamp(_coerce_float(raw_drive.get("linear", 0.0), "drive.linear"), -1.0, 1.0)
    angular = clamp(_coerce_float(raw_drive.get("angular", 0.0), "drive.angular"), -1.0, 1.0)
    power = clamp(_coerce_float(raw_drive.get("power", 0.1), "drive.power"), 0.1, 0.9)
    return DriveCommand(linear=linear, angular=angular, power=power)


def _parse_arm(
    raw_arm: Any,
    timestamp: float,
    *,
    gripper_open_tick: int | None,
    gripper_closed_tick: int | None,
    arm_joint_deg_limits: tuple[tuple[float, float], ...],
) -> ArmTargetCommand | None:
    if raw_arm is None:
        return None
    if not isinstance(raw_arm, Mapping):
        raise CommandValidationError("arm must be an object")

    raw_servos = raw_arm.get("servos")
    servos: list[int]
    if isinstance(raw_servos, list):
        if len(raw_servos) == 6:
            servos = _require_int_list(raw_servos, expected_length=6, field_name="arm.servos")
        elif len(raw_servos) == 5:
            base_servos = _require_int_list(raw_servos, expected_length=5, field_name="arm.servos")
            gripper_tick = _parse_gripper_value(
                raw_arm.get("gripper"),
                gripper_open_tick=gripper_open_tick,
                gripper_closed_tick=gripper_closed_tick,
            )
            servos = [*base_servos, int(gripper_tick)]
        else:
            raise CommandValidationError("arm.servos must have length 5 or 6")
    else:
        joints_deg_raw = raw_arm.get("joints_deg")
        if not isinstance(joints_deg_raw, list):
            raise CommandValidationError(
                "arm must contain either servos[5|6] or joints_deg[5] + gripper"
            )
        joints_deg = _require_float_list(
            joints_deg_raw,
            expected_length=5,
            field_name="arm.joints_deg",
        )
        if len(arm_joint_deg_limits) != 5:
            raise CommandValidationError("arm_joint_deg_limits must have 5 ranges")
        base_ticks = [
            _map_angle_to_tick(joints_deg[index], arm_joint_deg_limits[index], f"arm.joints_deg[{index}]")
            for index in range(5)
        ]
        gripper_tick = _parse_gripper_value(
            raw_arm.get("gripper"),
            gripper_open_tick=gripper_open_tick,
            gripper_closed_tick=gripper_closed_tick,
        )
        servos = [*base_ticks, int(gripper_tick)]

    return ArmTargetCommand(
        timestamp=timestamp,
        deadman_switch=bool(raw_arm.get("deadman", False)),
        servos=tuple(clamp_int(value, 0, 4095) for value in servos),
    )


def _parse_camera(
    raw_camera: Any,
    *,
    camera_yaw_deg_limits: tuple[float, float],
    camera_pitch_deg_limits: tuple[float, float],
    camera_pan_calibration: tuple[int, int, int] | None = None,
    camera_tilt_calibration: tuple[int, int, int] | None = None,
) -> CameraTargetCommand | None:
    if raw_camera is None:
        return None
    if not isinstance(raw_camera, Mapping):
        raise CommandValidationError("camera must be an object")
    raw_servos = raw_camera.get("servos")
    if isinstance(raw_servos, list):
        servos = _require_int_list(raw_servos, expected_length=2, field_name="camera.servos")
        return CameraTargetCommand(servos=tuple(clamp_int(value, 0, 4095) for value in servos))

    yaw_raw = raw_camera.get("yaw_deg")
    pitch_raw = raw_camera.get("pitch_deg")
    if isinstance(yaw_raw, (int, float)) and isinstance(pitch_raw, (int, float)):
        if camera_pan_calibration is not None:
            yaw_tick = _map_angle_to_tick_calibrated(
                float(yaw_raw),
                camera_yaw_deg_limits,
                camera_pan_calibration,
                "camera.yaw_deg",
            )
        else:
            yaw_tick = _map_angle_to_tick(float(yaw_raw), camera_yaw_deg_limits, "camera.yaw_deg")
        if camera_tilt_calibration is not None:
            pitch_tick = _map_angle_to_tick_calibrated(
                float(pitch_raw),
                camera_pitch_deg_limits,
                camera_tilt_calibration,
                "camera.pitch_deg",
            )
        else:
            pitch_tick = _map_angle_to_tick(float(pitch_raw), camera_pitch_deg_limits, "camera.pitch_deg")
        return CameraTargetCommand(
            servos=(
                int(clamp_int(yaw_tick, 0, 4095)),
                int(clamp_int(pitch_tick, 0, 4095)),
            )
        )

    raise CommandValidationError("camera must contain either servos[2] or yaw_deg/pitch_deg")


def _require_int_list(raw_values: Any, expected_length: int, field_name: str) -> list[int]:
    if not isinstance(raw_values, list):
        raise CommandValidationError(f"{field_name} must be a list")
    if len(raw_values) != expected_length:
        raise CommandValidationError(f"{field_name} must have length {expected_length}")
    try:
        return [int(value) for value in raw_values]
    except (TypeError, ValueError) as exc:
        raise CommandValidationError(f"{field_name} must contain integers") from exc


def _require_float_list(raw_values: Any, expected_length: int, field_name: str) -> list[float]:
    if not isinstance(raw_values, list):
        raise CommandValidationError(f"{field_name} must be a list")
    if len(raw_values) != expected_length:
        raise CommandValidationError(f"{field_name} must have length {expected_length}")
    try:
        return [float(value) for value in raw_values]
    except (TypeError, ValueError) as exc:
        raise CommandValidationError(f"{field_name} must contain numbers") from exc


def _coerce_float(value: Any, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise CommandValidationError(f"{field_name} must be numeric") from exc


def _map_angle_to_tick(angle_value: float, degree_limits: tuple[float, float], field_name: str) -> int:
    low, high = float(degree_limits[0]), float(degree_limits[1])
    if not high > low:
        raise CommandValidationError(f"Invalid limits for {field_name}: {degree_limits}")
    bounded = float(clamp(float(angle_value), low, high))
    ratio = (bounded - low) / (high - low)
    return int(round(ratio * 4095.0))


def _map_angle_to_tick_calibrated(
    angle_value: float,
    degree_limits: tuple[float, float],
    calibration: tuple[int, int, int],
    field_name: str,
) -> int:
    low, high = float(degree_limits[0]), float(degree_limits[1])
    if not high > low:
        raise CommandValidationError(f"Invalid limits for {field_name}: {degree_limits}")

    minimum, home, maximum = int(calibration[0]), int(calibration[1]), int(calibration[2])
    if minimum > maximum:
        raise CommandValidationError(f"Invalid calibration range for {field_name}: {calibration}")
    if home < minimum:
        home = minimum
    if home > maximum:
        home = maximum

    bounded = float(clamp(float(angle_value), low, high))
    if bounded >= 0.0:
        span = max(1e-9, high)
        ratio = bounded / span
        tick = int(round(float(home) + ratio * float(maximum - home)))
    else:
        span = max(1e-9, abs(low))
        ratio = abs(bounded) / span
        tick = int(round(float(home) - ratio * float(home - minimum)))

    return int(clamp_int(tick, minimum, maximum))


def _parse_gripper_value(
    raw_gripper: Any,
    *,
    gripper_open_tick: int | None,
    gripper_closed_tick: int | None,
) -> int:
    if isinstance(raw_gripper, str):
        normalized = raw_gripper.strip().lower()
        if normalized in {"open", "opened"}:
            if gripper_open_tick is None:
                raise CommandValidationError(
                    "arm.gripper=open is not available because gripper_open_tick is not configured"
                )
            return int(gripper_open_tick)
        if normalized in {"close", "closed"}:
            if gripper_closed_tick is None:
                raise CommandValidationError(
                    "arm.gripper=closed is not available because gripper_closed_tick is not configured"
                )
            return int(gripper_closed_tick)
        raise CommandValidationError("arm.gripper must be 'open', 'closed', or an integer tick")

    if isinstance(raw_gripper, (int, float)):
        return int(raw_gripper)

    raise CommandValidationError("arm.gripper is required when arm.servos has length 5")
