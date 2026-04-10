"""Test bootstrap for source-tree imports."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

for package_dir in (
    SRC / "rovr_common",
    SRC / "rovr_chassis",
    SRC / "rovr_arm",
    SRC / "rovr_camera",
    SRC / "rovr_bringup",
):
    sys.path.insert(0, str(package_dir))
