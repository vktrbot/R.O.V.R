"""Range helpers shared across controller modules."""

from __future__ import annotations

from typing import Iterable


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a float to an inclusive range."""
    return max(minimum, min(maximum, value))


def clamp_int(value: int, minimum: int, maximum: int) -> int:
    """Clamp an integer to an inclusive range."""
    return int(max(minimum, min(maximum, int(value))))


def clamp_sequence(values: Iterable[int], ranges: Iterable[tuple[int, int]]) -> list[int]:
    """Clamp each value against its per-index range."""
    return [clamp_int(value, lower, upper) for value, (lower, upper) in zip(values, ranges)]
