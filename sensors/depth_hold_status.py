# sensors/depth_hold_status.py
from __future__ import annotations

import time
from typing import Any, Dict, Optional, Callable

from sensors.base import BaseSensor


class DepthHoldStatusSensor(BaseSensor):
    """Publishes depth-hold state for topside visibility.

    The ctrl_status_fn should return the dict produced by
    ControlService.get_depth_hold_status().
    """

    def __init__(
        self,
        ctrl_status_fn: Optional[Callable[[], Dict[str, Any]]] = None,
        rate_hz: float = 10.0,
    ):
        super().__init__(name="depth_hold", rate_hz=rate_hz)
        self._fn = ctrl_status_fn

    def read(self) -> Dict[str, Any]:
        msg: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": "depth_hold",
            "type": "depth_hold_status",
        }
        if self._fn is not None:
            try:
                st = self._fn() or {}
                if isinstance(st, dict):
                    msg.update(st)
            except Exception as e:
                msg["error"] = str(e)
        return msg
