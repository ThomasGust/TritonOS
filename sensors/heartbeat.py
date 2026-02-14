# rov/sensors/heartbeat.py
from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

from sensors.base import BaseSensor


class HeartbeatSensor(BaseSensor):
    """
    Publishes a simple heartbeat message at a fixed rate.

    The state_fn should return a dict of extra fields to include, e.g.:
        {"armed": True, "pilot_age": 0.12, "pilot_seq": 123}
    """
    def __init__(self, state_fn: Optional[Callable[[], Dict[str, Any]]] = None, rate_hz: float = 1.0):
        super().__init__(name="heartbeat", rate_hz=rate_hz)
        self._state_fn = state_fn

    def read(self) -> Dict[str, Any]:
        data: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": "heartbeat",
            "type": "heartbeat",
        }
        if self._state_fn:
            try:
                extra = self._state_fn() or {}
                if isinstance(extra, dict):
                    data.update(extra)
            except Exception as e:
                data["state_error"] = str(e)
        return data
