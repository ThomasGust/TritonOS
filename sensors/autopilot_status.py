from __future__ import annotations

import time
from typing import Any, Dict

from sensors.base import BaseSensor


class AutopilotStatusSensor(BaseSensor):
    """Publishes the control/autopilot runtime state as telemetry.

    This is intentionally emitted on the normal sensor PUB stream so topside
    stream logs and raw CSV captures get a time-aligned record of what the ROV
    control loop believed, targeted, and commanded during a run.
    """

    def __init__(self, control_service: Any, rate_hz: float = 20.0):
        super().__init__(name="autopilot_status", rate_hz=float(rate_hz))
        self._control_service = control_service

    def read(self) -> Dict[str, Any]:
        now = time.time()
        data: Dict[str, Any] = {
            "ts": now,
            "sensor": "autopilot_status",
            "type": "autopilot_status",
            "source": "control_service",
        }
        try:
            snapshot = self._control_service.get_hold_status_snapshot()
        except Exception as exc:
            data["error"] = str(exc)
            return data
        if isinstance(snapshot, dict):
            data.update(snapshot)
        data["ts"] = now
        data["sensor"] = "autopilot_status"
        data["type"] = "autopilot_status"
        data["source"] = "control_service"
        return data
