"""Servo calibration helpers for raw and center-shifted coordinate spaces."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass

from .clamp import clamp
RAW_RANGE = 4096
HALF_RANGE = RAW_RANGE // 2
DEFAULT_CENTER_VIRTUAL = 2048


def resolve_config_path(path_str: str) -> str:
    """Resolve config files relative to the workspace config directory."""
    if os.path.isabs(path_str):
        return path_str
    config_dir = os.environ.get("ROVR_CONFIG_DIR")
    if config_dir:
        return os.path.join(config_dir, os.path.basename(path_str))
    return os.path.abspath(path_str)


@dataclass(frozen=True)
class ServoCalibration:
    """Calibration for one servo in either raw or center-shifted virtual space."""

    servo_id: int
    minimum: int
    maximum: int
    home: int
    center_raw: int | None = None
    center_virtual: int = DEFAULT_CENTER_VIRTUAL

    @property
    def uses_virtual_coordinates(self) -> bool:
        return self.center_raw is not None

    # -------------------------------------------------------------------------
    # Seam-safety helpers
    # -------------------------------------------------------------------------

    @property
    def crosses_seam(self) -> bool:
        """True when the calibrated working range requires the raw encoder to pass through
        the 0/4095 hardware boundary.

        The STS3215 firmware uses signed integer arithmetic for position error
        (error = goal − current).  If the working range straddles raw 0 or raw 4095,
        a small EMA step that crosses the boundary produces a goal raw value that is
        ~4096 counts away from the current raw value, and the servo drives hard in the
        wrong direction.

        The fix is to choose center_raw such that the ENTIRE working range maps to a
        monotone (non-wrapping) segment of the 0–4095 raw space.
        """
        if self.center_raw is None:
            return False
        min_raw = (self.center_raw + (self.minimum - self.center_virtual)) % RAW_RANGE
        max_raw = (self.center_raw + (self.maximum - self.center_virtual)) % RAW_RANGE
        return int(min_raw) > int(max_raw)

    def seam_crossing_virtual(self) -> int | None:
        """Virtual tick at which the raw encoder touches the 0/4095 boundary.

        Returns None when the calibration is seam-safe.
        If this value falls inside [minimum, maximum], any motion that passes
        through it will cause the servo to receive a discontinuous goal and
        reverse direction.
        """
        if not self.crosses_seam:
            return None
        if int(self.center_raw) < HALF_RANGE:
            # Working range wraps through raw 0 on the negative side.
            return int(self.center_virtual) - int(self.center_raw)
        else:
            # Working range wraps through raw 4095 on the positive side.
            return int(self.center_virtual) + (RAW_RANGE - 1 - int(self.center_raw))

    def safe_minimum(self) -> int:
        """Largest virtual minimum that keeps the raw range away from the seam.

        Equal to self.minimum when the calibration is already seam-safe.
        """
        seam_v = self.seam_crossing_virtual()
        if seam_v is None:
            return int(self.minimum)
        if int(self.center_raw) < HALF_RANGE:
            # Seam is on the low (negative) side – raise the minimum.
            return max(int(self.minimum), seam_v)
        return int(self.minimum)

    def safe_maximum(self) -> int:
        """Smallest virtual maximum that keeps the raw range away from the seam.

        Equal to self.maximum when the calibration is already seam-safe.
        """
        seam_v = self.seam_crossing_virtual()
        if seam_v is None:
            return int(self.maximum)
        if int(self.center_raw) >= HALF_RANGE:
            # Seam is on the high (positive) side – lower the maximum.
            return min(int(self.maximum), seam_v)
        return int(self.maximum)

    # -------------------------------------------------------------------------
    # Command helpers
    # -------------------------------------------------------------------------

    def clamp_command(self, value: int) -> int:
        return int(clamp(int(value), int(self.minimum), int(self.maximum)))

    def raw_to_command(self, raw_value: int) -> int:
        if self.center_raw is None:
            return int(raw_value)
        delta = int(raw_value) - int(self.center_raw)
        if delta > HALF_RANGE:
            delta -= RAW_RANGE
        elif delta < -HALF_RANGE:
            delta += RAW_RANGE
        return int(self.center_virtual + delta)

    def command_to_raw(self, command_value: int) -> int:
        clamped = self.clamp_command(command_value)
        if self.center_raw is None:
            return int(clamped)
        offset = int(clamped) - int(self.center_virtual)
        return int((int(self.center_raw) + offset) % RAW_RANGE)


def load_servo_calibrations(path_str: str, servo_ids: list[int]) -> list[ServoCalibration]:
    """Load calibrations from JSON. Legacy files without center_raw are treated as raw-space."""
    resolved = resolve_config_path(path_str)
    with open(resolved, "r", encoding="utf-8") as handle:
        raw_config = json.load(handle)

    calibrations: list[ServoCalibration] = []
    for servo_id in servo_ids:
        item = raw_config[str(servo_id)]
        minimum = int(item["min"])
        maximum = int(item["max"])
        home = int(item.get("home", (minimum + maximum) // 2))
        center_raw = item.get("center_raw")
        center_virtual = int(item.get("center_virtual", DEFAULT_CENTER_VIRTUAL))
        calibrations.append(
            ServoCalibration(
                servo_id=int(servo_id),
                minimum=minimum,
                maximum=maximum,
                home=home,
                center_raw=None if center_raw is None else int(center_raw),
                center_virtual=center_virtual,
            )
        )
    return calibrations
