"""Smoothing helpers for controller setpoints."""

from __future__ import annotations

from .clamp import clamp


def ema_step(current: float, target: float, alpha: float) -> float:
    """Apply one exponential moving average step."""
    bounded_alpha = clamp(alpha, 0.0, 1.0)
    return current + (target - current) * bounded_alpha


def slew_step(current: float, target: float, rate: float, dt: float) -> float:
    """Limit the delta applied to a target over time."""
    max_delta = max(rate, 0.0) * max(dt, 0.0)
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


def smooth_positions(current: list[int], target: list[int], alpha: float) -> list[int]:
    """Smooth integer position targets with EMA."""
    return [int(round(ema_step(float(cur), float(dst), alpha))) for cur, dst in zip(current, target)]
