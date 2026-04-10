"""Normalized command data models."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DriveCommand:
    linear: float
    angular: float
    power: float


@dataclass(frozen=True)
class ArmTargetCommand:
    timestamp: float
    deadman_switch: bool
    servos: tuple[int, int, int, int, int, int]


@dataclass(frozen=True)
class CameraTargetCommand:
    servos: tuple[int, int]


@dataclass(frozen=True)
class ArmTelemetryData:
    timestamp: float
    positions: tuple[int, int, int, int, int, int]
    loads: tuple[float, float, float, float, float, float]
    temperatures: tuple[float, float, float, float, float, float]
    jam_detected: tuple[bool, bool, bool, bool, bool, bool]
    torque_enabled: tuple[bool, bool, bool, bool, bool, bool]
    status: str


@dataclass(frozen=True)
class NormalizedControlFrame:
    timestamp: float
    drive: DriveCommand | None = None
    arm: ArmTargetCommand | None = None
    camera: CameraTargetCommand | None = None
