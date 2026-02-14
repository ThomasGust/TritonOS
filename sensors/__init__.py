"""
sensors package

Keep this package importable on development machines without the Navigator
hardware library installed. Import hardware-backed sensors (NavigatorBoard, etc.)
directly from their modules when needed.
"""
from __future__ import annotations

from sensors.base import BaseSensor
from sensors.heartbeat import HeartbeatSensor

# Pure-Linux utility sensor (no Navigator dependencies)
try:
    from sensors.network import NetworkStatsSensor  # noqa: F401
except Exception:
    # Keep package importable on non-Linux dev machines.
    NetworkStatsSensor = None  # type: ignore
