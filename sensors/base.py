"""Shared base types for polled sensor telemetry producers."""

# sensors/base.py
from __future__ import annotations

from dataclasses import dataclass
import math
import time


@dataclass
class BaseSensor:
    """
    Small base class used by polled sensors.

    Kept in a hardware-free module so unit tests can import sensors without
    requiring Navigator libraries or physical hardware.
    """
    name: str
    rate_hz: float
    _next_t: float = 0.0

    def should_poll(self, now: float | None = None) -> bool:
        """Return True when it's time to poll the sensor again.

        This keeps the publisher loop simple and makes unit testing easier.
        """
        if now is None:
            now = time.time()
        return float(now) >= float(self._next_t)

    def mark_polled(self, now: float | None = None) -> None:
        """Advance the next scheduled poll time."""
        if now is None:
            now = time.time()
        hz = float(self.rate_hz)
        if hz <= 0:
            self._next_t = math.inf
        else:
            self._next_t = float(now) + (1.0 / hz)
