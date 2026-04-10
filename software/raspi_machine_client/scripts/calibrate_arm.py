#!/usr/bin/env python3
"""Interactive arm calibration tool.

Generates arm_limits.json with per-joint physical limits, home positions,
and center_raw offsets that shift the working range away from the encoder seam (0/4095).

Usage:
  python3 scripts/calibrate_arm.py [--port /dev/ttyACM0] [--baudrate 1000000]
                                   [--output config/arm_limits.json]
                                   [--servo-ids 1 2 3 4 5 6]

Calibration flow:
  1. Arm is auto-detected on the specified port (6 servos expected).
  2. Torque is released on all joints so you can move them by hand.
  3. You move the arm to the ZERO (neutral) position and press Enter.
     This position becomes center_raw = physical raw encoder value,
     center_virtual = 2048 (virtual center in command-space).
  4. For each joint 1-5:
       - Move joint to MIN limit, press Enter
       - Move joint to MAX limit, press Enter
       - Move joint to HOME position, press Enter
  5. For gripper (joint 6):
       - Move gripper to OPEN position, press Enter
       - Move gripper to CLOSED position, press Enter
     Home = open position.
  6. Summary shown, confirm to save.

Seam-point safety:
  The calibration records center_raw so that servo_calibration.py can map
  raw encoder readings to a continuous virtual space. If the working range
  crosses the 0/4095 seam you will see a warning and should physically
  reposition the joint's zero or re-run.
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
    print("[ERROR] st3215 library not found. Install with: pip install st3215")
    sys.exit(1)

# -------------------------------------------------------------------------
# Constants
# -------------------------------------------------------------------------

RAW_RANGE = 4096
HALF_RANGE = RAW_RANGE // 2
CENTER_VIRTUAL = 2048

JOINT_NAMES = {
    1: "Joint 1 (Yaw base)",
    2: "Joint 2 (Pitch shoulder)",
    3: "Joint 3 (Pitch elbow)",
    4: "Joint 4 (Pitch wrist)",
    5: "Joint 5 (Roll wrist)",
    6: "Joint 6 (Gripper)",
}


# -------------------------------------------------------------------------
# Low-level helpers
# -------------------------------------------------------------------------

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


def _read_position(dev: ST3215, servo_id: int, retries: int = 4) -> int | None:
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


def _raw_to_virtual(raw: int, center_raw: int) -> int:
    """Convert raw encoder tick to virtual command-space tick (centered at 2048)."""
    delta = int(raw) - int(center_raw)
    if delta > HALF_RANGE:
        delta -= RAW_RANGE
    elif delta < -HALF_RANGE:
        delta += RAW_RANGE
    return CENTER_VIRTUAL + delta


def _seam_crossing_info(center_raw: int, min_raw: int, max_raw: int) -> dict:
    """Return seam-crossing analysis for a joint's raw working range.

    Returns a dict with keys:
      crosses_seam (bool)  – True when the raw range wraps through 0 or 4095.
      seam_virtual (int)   – Virtual tick of the seam boundary (only when crosses_seam).
      safe_min_raw (int)   – Smallest raw value that avoids the seam.
      safe_max_raw (int)   – Largest raw value that avoids the seam.
      required_center_min  – Minimum center_raw for the range to be seam-free.
      required_center_max  – Maximum center_raw for the range to be seam-free.
    """
    delta_min = int(min_raw) - int(center_raw)
    delta_max = int(max_raw) - int(center_raw)
    for delta in (delta_min, delta_max):
        pass  # normalise below
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
    crosses = (int(center_raw) + delta_min) < 0 or (int(center_raw) + delta_max) >= RAW_RANGE

    # Seam virtual: the virtual tick where the seam boundary falls.
    if crosses:
        if int(center_raw) < HALF_RANGE:
            seam_virtual = CENTER_VIRTUAL - int(center_raw)   # raw crosses 0
        else:
            seam_virtual = CENTER_VIRTUAL + (RAW_RANGE - 1 - int(center_raw))  # raw crosses 4095
    else:
        seam_virtual = None

    # For a seam-free calibration the full raw range must fit within one
    # continuous half of the 0-4095 circle, i.e. center_raw must be at
    # least max_motion_ticks away from both 0 and 4095.
    half_span = max(abs(delta_min), abs(delta_max)) + 50   # +50 safety margin
    required_center_min = half_span
    required_center_max = RAW_RANGE - half_span

    return {
        "crosses_seam": crosses,
        "seam_virtual": seam_virtual,
        "required_center_min": required_center_min,
        "required_center_max": required_center_max,
    }


def _check_seam_crossing(center_raw: int, min_raw: int, max_raw: int, servo_id: int) -> bool:
    """Print a warning if the working range crosses the 0/4095 encoder seam.

    Returns True if the seam is crossed (caller should refuse to save).

    The STS3215 uses signed integer arithmetic for position error (goal − current).
    If the commanded raw position jumps across the seam boundary, the servo computes
    a ~4096-tick error in the WRONG direction and drives hard against the limit.
    This is NOT a software bug that can be worked around: the motor must be
    physically positioned so the entire working range stays on one side of the seam.
    """
    info = _seam_crossing_info(center_raw, min_raw, max_raw)
    margin = 100
    near_seam = any(v < margin or v > RAW_RANGE - margin for v in (min_raw, max_raw))

    if info["crosses_seam"] or near_seam:
        print()
        print(f"  *** SEAM CROSSING ERROR — Joint {servo_id} ***")
        print(f"  The working range (min_raw={min_raw}, max_raw={max_raw}) crosses")
        print(f"  the hardware encoder seam at raw 0/4095.")
        print()
        print(f"  The STS3215 servo uses signed position error (goal − current).")
        print(f"  When an EMA step crosses the seam, the servo sees an error of")
        print(f"  ~4096 counts in the WRONG direction and drives hard in reverse.")
        print(f"  This cannot be fixed in software.")
        print()
        print(f"  REQUIRED FIX: physically rotate the motor so its encoder reads")
        print(f"  a value between {info['required_center_min']} and "
              f"{info['required_center_max']} at the neutral (zero) position,")
        print(f"  then re-run this calibration script.")
        print()
        return True
    return False


# -------------------------------------------------------------------------
# Prompt helpers
# -------------------------------------------------------------------------

def _pause(message: str) -> None:
    input(f"\n  >>> {message} — press Enter when ready...")


def _read_raw_here(dev: ST3215, servo_id: int, label: str) -> int:
    while True:
        raw = _read_position(dev, servo_id)
        if raw is not None:
            print(f"      {label}: raw={raw}")
            return raw
        print(f"      [WARN] Failed to read servo {servo_id}, retrying...")
        time.sleep(0.1)


# -------------------------------------------------------------------------
# Main calibration routine
# -------------------------------------------------------------------------

def calibrate(
    port: str,
    baudrate: int,
    output_path: str,
    servo_ids: list[int],
) -> dict:
    print("\n" + "=" * 60)
    print(" ROVR Arm Calibration Tool")
    print("=" * 60)
    print(f"  Port      : {port}")
    print(f"  Baudrate  : {baudrate}")
    print(f"  Servo IDs : {servo_ids}")
    print(f"  Output    : {output_path}")
    print()

    # Open bus
    print("[1/3] Connecting to servo bus...")
    dev = _open_bus(port, baudrate)

    live = [sid for sid in servo_ids if _ping(dev, sid)]
    missing = [sid for sid in servo_ids if sid not in live]
    if missing:
        print(f"  [WARNING] Servos not responding: {missing}")
        if not live:
            print("  [ERROR] No servos found. Check cable and power.")
            sys.exit(1)
        resp = input(f"  Continue with available servos {live}? [y/N] ").strip().lower()
        if resp != "y":
            sys.exit(0)
    else:
        print(f"  All {len(live)} servos found: {live}")

    # Release torque
    print("\n[2/3] Releasing torque on all joints so you can move them by hand...")
    for sid in live:
        _set_torque(dev, sid, False)
    time.sleep(0.1)
    print("  Torque released. You can now manually position joints.")

    # ---- Step 1: capture zero / center_raw ----
    print("\n[3/3] Calibration procedure")
    print("-" * 40)
    print("  STEP 1: Move the WHOLE ARM to the neutral ZERO position.")
    print("  This position will be mapped to virtual tick 2048 (the mathematical center).")
    print("  All joints should be in their natural neutral / straight configuration.")
    _pause("Move arm to ZERO position")

    center_raws: dict[int, int] = {}
    for sid in live:
        center_raws[sid] = _read_raw_here(dev, sid, f"  Joint {sid} zero")

    # ---- Per-joint limits ----
    result: dict[str, dict] = {}

    arm_joints = [sid for sid in live if sid != 6]
    for sid in arm_joints:
        name = JOINT_NAMES.get(sid, f"Joint {sid}")
        center_raw = center_raws[sid]
        print(f"\n  ---- {name} (servo {sid}) ----")
        print(f"       center_raw = {center_raw}")

        _pause(f"Move {name} to MINIMUM (negative) limit")
        min_raw = _read_raw_here(dev, sid, f"  {name} MIN")

        _pause(f"Move {name} to MAXIMUM (positive) limit")
        max_raw = _read_raw_here(dev, sid, f"  {name} MAX")

        _pause(f"Move {name} to HOME position (where arm rests when idle)")
        home_raw = _read_raw_here(dev, sid, f"  {name} HOME")

        # Convert to virtual command-space
        min_virtual = _raw_to_virtual(min_raw, center_raw)
        max_virtual = _raw_to_virtual(max_raw, center_raw)
        home_virtual = _raw_to_virtual(home_raw, center_raw)

        # Ensure min < max
        if min_virtual > max_virtual:
            min_virtual, max_virtual = max_virtual, min_virtual
            print(f"       [INFO] min/max swapped to ensure min < max")

        home_virtual = max(min_virtual, min(max_virtual, home_virtual))

        if _check_seam_crossing(center_raw, min_raw, max_raw, sid):
            print(f"  Calibration for Joint {sid} is UNSAFE and will not be saved.")
            print(f"  Power off the servos, physically rotate the motor for Joint {sid}")
            print(f"  until the encoder reads a value in the required range, then")
            print(f"  re-run this script.")
            sys.exit(1)

        span_deg = (max_virtual - min_virtual) / 4096.0 * 360.0
        print(f"       virtual: min={min_virtual}  max={max_virtual}  home={home_virtual}")
        print(f"       motion span: {span_deg:.1f} degrees")

        result[str(sid)] = {
            "name": JOINT_NAMES.get(sid, f"joint_{sid}"),
            "min": min_virtual,
            "max": max_virtual,
            "home": home_virtual,
            "center_raw": center_raw,
            "center_virtual": CENTER_VIRTUAL,
        }

    # ---- Gripper (joint 6) ----
    if 6 in live:
        center_raw_6 = center_raws[6]
        print(f"\n  ---- {JOINT_NAMES[6]} (servo 6) ----")
        print(f"       center_raw = {center_raw_6}")

        _pause("Move gripper to OPEN position (this is also home)")
        open_raw = _read_raw_here(dev, 6, "  Gripper OPEN")

        _pause("Move gripper to CLOSED position (maximum grip)")
        closed_raw = _read_raw_here(dev, 6, "  Gripper CLOSED")

        open_virtual = _raw_to_virtual(open_raw, center_raw_6)
        closed_virtual = _raw_to_virtual(closed_raw, center_raw_6)

        min_virtual = min(open_virtual, closed_virtual)
        max_virtual = max(open_virtual, closed_virtual)
        home_virtual = open_virtual  # home = open (less torque needed)

        if _check_seam_crossing(center_raw_6, open_raw, closed_raw, 6):
            print("  Gripper calibration is UNSAFE and will not be saved.")
            sys.exit(1)

        print(f"       virtual: open={open_virtual}  closed={closed_virtual}")
        print(f"       min={min_virtual}  max={max_virtual}  home={home_virtual}")

        result["6"] = {
            "name": "joint_6",
            "min": min_virtual,
            "max": max_virtual,
            "home": home_virtual,
            "center_raw": center_raw_6,
            "center_virtual": CENTER_VIRTUAL,
            "gripper_open_raw": open_raw,
            "gripper_closed_raw": closed_raw,
        }

    # ---- Summary ----
    print("\n" + "=" * 60)
    print(" Calibration Summary")
    print("=" * 60)
    for sid in servo_ids:
        if str(sid) not in result:
            continue
        entry = result[str(sid)]
        print(
            f"  Servo {sid:2d} ({entry['name']:20s}): "
            f"min={entry['min']:5d}  max={entry['max']:5d}  "
            f"home={entry['home']:5d}  center_raw={entry['center_raw']:5d}"
        )

    print()
    resp = input(f"Save to {output_path}? [Y/n] ").strip().lower()
    if resp in ("", "y"):
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as fh:
            json.dump(result, fh, indent=2)
        print(f"  Saved: {output_path}")
    else:
        print("  Aborted, nothing saved.")

    # Re-release torque after calibration
    for sid in live:
        _set_torque(dev, sid, False)

    return result


# -------------------------------------------------------------------------
# Entry point
# -------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive ROVR arm calibration tool")
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port for the arm servo bus (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="Serial baudrate (default: 1000000)",
    )
    parser.add_argument(
        "--output",
        default="config/arm_limits.json",
        help="Output calibration file path (default: config/arm_limits.json)",
    )
    parser.add_argument(
        "--servo-ids",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4, 5, 6],
        help="Servo IDs to calibrate (default: 1 2 3 4 5 6)",
    )
    args = parser.parse_args()

    calibrate(
        port=args.port,
        baudrate=args.baudrate,
        output_path=args.output,
        servo_ids=args.servo_ids,
    )


if __name__ == "__main__":
    main()
