"""Simple timeout tracking utilities."""

from __future__ import annotations

import time


class TimeoutWatchdog:
    """Tracks whether a signal has been refreshed within a timeout window."""

    def __init__(self, timeout_seconds: float) -> None:
        self.timeout_seconds = float(timeout_seconds)
        self.last_mark = 0.0

    def mark(self, now: float | None = None) -> None:
        self.last_mark = time.monotonic() if now is None else float(now)

    def age(self, now: float | None = None) -> float:
        if self.last_mark <= 0.0:
            return float("inf")
        check_time = time.monotonic() if now is None else float(now)
        return max(0.0, check_time - self.last_mark)

    def expired(self, now: float | None = None) -> bool:
        return self.age(now) > self.timeout_seconds
