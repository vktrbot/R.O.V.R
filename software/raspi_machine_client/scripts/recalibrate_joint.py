#!/usr/bin/env python3
"""Recalibrate a single joint and update arm_limits.json in place.

Usage:
  python3 scripts/recalibrate_joint.py --joint 4
  python3 scripts/recalibrate_joint.py --joint 4 --port /dev/ttyACM1

Only the specified joint's entry is changed; all others remain untouched.
The script refuses to save if the new calibration crosses the encoder seam.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

try:
    from st3215 import ST3215
    from st3215.values import COMM_SUCCESS, STS_TORQUE_ENABLE
except ImportError:
    print("[ERROR] st3215 library not found.  pip install st3215")
    sys.exit(1)

RAW_RANGE = 4096
HALF_RANGE = RAW_RANGE // 2
CENTER_VIRTUAL = 2048


# ---------------------------------------------------------------------------
# Bus helpers
# ---------------------------------------------------------------------------

def _open_bus(port: str, baudrate: int) -> ST3215:
    dev = ST3215(str(port))
    try:
        dev.portHandler.closePort()
    except Exception:
        pass
    dev.portHandler.baudrate = int(baudrate)
    dev.portHandler.setupPort()
    return dev


def _ping(dev: ST3215, servo_id: int) -> bool:
    try:
        model, comm, err = dev.ping(int(servo_id))
        return comm == COMM_SUCCESS and err == 0 and model is not None
    except Exception:
        return False


def _read_position(dev: ST3215, servo_id: int, retries: int = 5) -> int | None:
    for _ in range(retries):
        try:
            pos = dev.ReadPosition(int(servo_id))
            if pos is not None:
                return int(pos)
        except Exception:
            pass
        time.sleep(0.02)
    return None


def _set_torque(dev: ST3215, servo_id: int, enabled: bool) -> None:
    try:
        dev.write1ByteTxRx(int(servo_id), STS_TORQUE_ENABLE, 1 if enabled else 0)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def _raw_to_virtual(raw: int, center_raw: int) -> int:
    delta = int(raw) - int(center_raw)
    if delta > HALF_RANGE:
        delta -= RAW_RANGE
    elif delta < -HALF_RANGE:
        delta += RAW_RANGE
    return CENTER_VIRTUAL + delta


def _check_seam(center_raw: int, min_raw: int, max_raw: int) -> tuple[bool, int | None, int, int]:
    """Return (crosses, seam_virtual, required_min_center, required_max_center)."""
    delta_min = int(min_raw) - int(center_raw)
    delta_max = int(max_raw) - int(center_raw)
    if delta_min > HALF_RANGE:
        delta_min -= RAW_RANGE
    elif delta_min < -HALF_RANGE:
        delta_min += RAW_RANGE
    if delta_max > HALF_RANGE:
        delta_max -= RAW_RANGE
    elif delta_max < -HALF_RANGE:
        delta_max += RAW_RANGE

    # Seam is crossed if the unwrapped raw endpoint goes outside [0, 4095].
    # Checking sign difference of deltas is WRONG — min is always below center
    # and max above, so deltas always have opposite signs in a normal calibration.
    crosses = (center_raw + delta_min) < 0 or (center_raw + delta_max) >= RAW_RANGE

    if crosses:
        seam_v: int | None = (
            CENTER_VIRTUAL - center_raw if center_raw < HALF_RANGE
            else CENTER_VIRTUAL + (RAW_RANGE - 1 - center_raw)
        )
    else:
        seam_v = None

    half_span = max(abs(delta_min), abs(delta_max)) + 50
    return crosses, seam_v, half_span, RAW_RANGE - half_span


# ---------------------------------------------------------------------------
# Prompts
# ---------------------------------------------------------------------------

def _pause(msg: str) -> None:
    input(f"\n  >>> {msg} — press Enter when ready...")


def _read_raw_here(dev: ST3215, servo_id: int, label: str) -> int:
    while True:
        raw = _read_position(dev, servo_id)
        if raw is not None:
            print(f"      {label}: raw={raw}")
            return raw
        print(f"      [WARN] Failed to read servo {servo_id}, retrying...")
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Recalibrate a single arm joint")
    parser.add_argument("--joint", type=int, required=True, help="Servo/joint ID to recalibrate")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--config", default="config/arm_limits.json")
    args = parser.parse_args()

    joint_id = int(args.joint)
    config_path = args.config
    if not os.path.isabs(config_path):
        config_dir = os.environ.get("ROVR_CONFIG_DIR")
        if config_dir:
            config_path = os.path.join(config_dir, os.path.basename(config_path))
        else:
            config_path = os.path.abspath(config_path)

    print()
    print("=" * 60)
    print(f"  Single-joint recalibration — Joint {joint_id}")
    print("=" * 60)
    print(f"  Port   : {args.port}")
    print(f"  Config : {config_path}")
    print()

    # Load existing config
    with open(config_path, "r", encoding="utf-8") as fh:
        config = json.load(fh)

    if str(joint_id) not in config:
        print(f"[ERROR] Joint {joint_id} not found in {config_path}")
        sys.exit(1)

    joint_name = config[str(joint_id)].get("name", f"joint_{joint_id}")
    print(f"  Joint name: {joint_name}")
    print()

    # Connect
    print("Connecting to servo bus...")
    dev = _open_bus(args.port, args.baudrate)
    if not _ping(dev, joint_id):
        print(f"[ERROR] Servo {joint_id} did not respond on {args.port}")
        sys.exit(1)
    print(f"  Servo {joint_id} found.")

    # Release torque
    _set_torque(dev, joint_id, False)
    time.sleep(0.1)
    print("  Torque released. You can move the joint by hand.")

    # --- ZERO position ---
    print()
    print("  STEP 1: Move joint to the ZERO (neutral) position.")
    print("  This becomes center_raw — the reference point for all angles.")
    _pause(f"Move Joint {joint_id} to ZERO")
    center_raw = _read_raw_here(dev, joint_id, "Zero position")

    # Quick seam proximity check for center_raw
    if center_raw < 200 or center_raw > RAW_RANGE - 200:
        print(f"  [WARNING] center_raw={center_raw} is within 200 ticks of the seam.")
        print(f"  The joint may still cross the seam during operation.")
        print(f"  Ideally center_raw should be between 200 and {RAW_RANGE - 200}.")
        resp = input("  Continue anyway? [y/N] ").strip().lower()
        if resp != "y":
            _set_torque(dev, joint_id, False)
            sys.exit(0)

    # --- MIN ---
    print()
    print("  STEP 2: Move joint to its MINIMUM (most negative) limit.")
    _pause(f"Move Joint {joint_id} to MIN limit")
    min_raw = _read_raw_here(dev, joint_id, "Min position")

    # --- MAX ---
    print()
    print("  STEP 3: Move joint to its MAXIMUM (most positive) limit.")
    _pause(f"Move Joint {joint_id} to MAX limit")
    max_raw = _read_raw_here(dev, joint_id, "Max position")

    # --- HOME ---
    print()
    print("  STEP 4: Move joint to its HOME (rest) position.")
    _pause(f"Move Joint {joint_id} to HOME")
    home_raw = _read_raw_here(dev, joint_id, "Home position")

    # Convert to virtual space
    min_v = _raw_to_virtual(min_raw, center_raw)
    max_v = _raw_to_virtual(max_raw, center_raw)
    home_v = _raw_to_virtual(home_raw, center_raw)

    if min_v > max_v:
        min_v, max_v = max_v, min_v
        print("  [INFO] min/max swapped to ensure min < max")

    home_v = max(min_v, min(max_v, home_v))

    # Seam check
    crosses, seam_v, req_center_min, req_center_max = _check_seam(center_raw, min_raw, max_raw)
    if crosses:
        print()
        print("  *** SEAM CROSSING ERROR ***")
        print(f"  The working range (min_raw={min_raw}, max_raw={max_raw})")
        print(f"  crosses the hardware encoder seam at raw 0/4095.")
        print(f"  (seam falls at virtual tick {seam_v})")
        print()
        print(f"  The STS3215 uses signed position error (goal − current).")
        print(f"  Crossing the seam causes the servo to see an error of ~4096 ticks")
        print(f"  in the WRONG direction and drive hard in reverse.")
        print()
        print(f"  FIX: physically rotate the motor so the encoder reads a value")
        print(f"  between {req_center_min} and {req_center_max} at the neutral position,")
        print(f"  then re-run this script.")
        _set_torque(dev, joint_id, False)
        sys.exit(1)

    # Near-seam warning
    margin = 150
    near = [r for r in (min_raw, max_raw) if r < margin or r > RAW_RANGE - margin]
    if near:
        print(f"  [WARNING] Raw limit value(s) {near} are within {margin} ticks of the seam.")
        print(f"  Motion is safe now but there is little safety margin.")
        resp = input("  Continue? [y/N] ").strip().lower()
        if resp != "y":
            _set_torque(dev, joint_id, False)
            sys.exit(0)

    # Summary
    span_deg = (max_v - min_v) / RAW_RANGE * 360.0
    min_deg = (min_v - CENTER_VIRTUAL) / RAW_RANGE * 360.0
    max_deg = (max_v - CENTER_VIRTUAL) / RAW_RANGE * 360.0
    home_deg = (home_v - CENTER_VIRTUAL) / RAW_RANGE * 360.0

    print()
    print("=" * 60)
    print(f"  Calibration result — {joint_name}")
    print("=" * 60)
    print(f"  center_raw = {center_raw}")
    print(f"  min  : raw={min_raw}  virtual={min_v}  ({min_deg:+.1f}°)")
    print(f"  max  : raw={max_raw}  virtual={max_v}  ({max_deg:+.1f}°)")
    print(f"  home : raw={home_raw}  virtual={home_v}  ({home_deg:+.1f}°)")
    print(f"  span : {span_deg:.1f}°")
    print(f"  seam-safe: YES")
    print()

    resp = input(f"  Save to {config_path}? [Y/n] ").strip().lower()
    if resp not in ("", "y"):
        print("  Aborted, nothing saved.")
        _set_torque(dev, joint_id, False)
        return

    # Update only the joint's entry
    config[str(joint_id)]["min"] = min_v
    config[str(joint_id)]["max"] = max_v
    config[str(joint_id)]["home"] = home_v
    config[str(joint_id)]["center_raw"] = center_raw
    config[str(joint_id)]["center_virtual"] = CENTER_VIRTUAL

    with open(config_path, "w", encoding="utf-8") as fh:
        json.dump(config, fh, indent=2)
    print(f"  Saved: {config_path}")
    print()

    _set_torque(dev, joint_id, False)


if __name__ == "__main__":
    main()
