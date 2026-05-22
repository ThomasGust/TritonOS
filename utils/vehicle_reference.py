"""Depth-reference persistence and capture helpers.

External pressure sensors need a surface-pressure reference before their depth
values are meaningful. This module owns the small JSON file used to store that
reference and provides helpers for capturing a fresh value through the active
depth-sensor configuration.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Optional


DEFAULT_DEPTH_REFERENCE_PATH = "calibration/depth_reference.json"


def resolve_path(path: str | Path) -> Path:
    """Resolve a user/config path without assuming it already exists."""

    return Path(path).expanduser()


def load_optional_json(path: str | Path) -> Optional[Dict[str, Any]]:
    """Load a JSON object if present, returning `None` for missing/bad files."""

    p = resolve_path(path)
    if not p.exists():
        return None
    try:
        data = json.loads(p.read_text())
    except Exception:
        return None
    return dict(data) if isinstance(data, dict) else None


def load_surface_pressure_reference_mbar(path: str | Path) -> Optional[float]:
    """Read the stored surface-pressure reference in millibar."""

    data = load_optional_json(path)
    if not data:
        return None
    try:
        v = float(data.get("surface_pressure_mbar"))
    except Exception:
        return None
    return v if math.isfinite(v) else None


def save_surface_pressure_reference(path: str | Path, surface_pressure_mbar: float, *, meta: Optional[Dict[str, Any]] = None) -> None:
    """Atomically write a surface-pressure reference JSON file."""

    payload: Dict[str, Any] = {
        "surface_pressure_mbar": float(surface_pressure_mbar),
        "created_ts": time.time(),
    }
    if meta:
        payload["meta"] = dict(meta)
    path_obj = resolve_path(path)
    path_obj.parent.mkdir(parents=True, exist_ok=True)
    tmp = path_obj.with_suffix(path_obj.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    tmp.replace(path_obj)


def build_depth_sensor_from_config(cfg: Any) -> Any:
    """Instantiate the configured external depth sensor wrapper."""

    from sensors.navigator import Bar02Sensor, Bar30Sensor, ExternalDepthSensor

    use_external = bool(getattr(cfg, "USE_EXTERNAL_DEPTH", False))
    use_bar02 = bool(getattr(cfg, "USE_BAR02", False))
    use_bar30 = bool(getattr(cfg, "USE_BAR30", False))

    def _get_buses(prefix: str, default_bus: int = 6):
        buses = getattr(cfg, f"{prefix}_I2C_BUSES", None)
        if buses is not None:
            return buses
        return int(getattr(cfg, f"{prefix}_I2C_BUS", getattr(cfg, "BAR30_I2C_BUS", default_bus)))

    kwargs = dict(
        surface_cal_samples=0,
        surface_cal_delay_s=0.0,
        surface_pressure_mbar=None,
        depth_offset_m=0.0,
    )

    if use_bar02:
        return Bar02Sensor(
            rate_hz=float(getattr(cfg, "BAR02_RATE_HZ", getattr(cfg, "BAR30_RATE_HZ", 5.0))),
            bus=_get_buses("BAR02"),
            model=getattr(cfg, "BAR02_MODEL", "02BA"),
            fluid_density=float(getattr(cfg, "BAR02_FLUID_DENSITY", getattr(cfg, "BAR30_FLUID_DENSITY", 1029))),
            osr=int(getattr(cfg, "BAR02_OSR", getattr(cfg, "BAR30_OSR", 5))),
            **kwargs,
        )
    if use_external:
        buses = getattr(cfg, "EXTERNAL_DEPTH_I2C_BUSES", None)
        if buses is None:
            buses = _get_buses("BAR30")
        return ExternalDepthSensor(
            rate_hz=float(getattr(cfg, "EXTERNAL_DEPTH_RATE_HZ", getattr(cfg, "BAR30_RATE_HZ", 5.0))),
            bus=buses,
            model=getattr(cfg, "EXTERNAL_DEPTH_MODEL", getattr(cfg, "BAR30_MODEL", "auto")),
            fluid_density=float(getattr(cfg, "EXTERNAL_DEPTH_FLUID_DENSITY", getattr(cfg, "BAR30_FLUID_DENSITY", 1029))),
            osr=int(getattr(cfg, "EXTERNAL_DEPTH_OSR", getattr(cfg, "BAR30_OSR", 5))),
            **kwargs,
        )
    if use_bar30:
        return Bar30Sensor(
            rate_hz=float(getattr(cfg, "BAR30_RATE_HZ", 5.0)),
            bus=_get_buses("BAR30"),
            model=getattr(cfg, "BAR30_MODEL", "auto"),
            fluid_density=float(getattr(cfg, "BAR30_FLUID_DENSITY", 1029)),
            osr=int(getattr(cfg, "BAR30_OSR", 5)),
            **kwargs,
        )
    raise RuntimeError("External depth sensor is not enabled in rov_config")


def capture_surface_pressure_reference(cfg: Any, *, samples: int, delay_s: float, sensor: Any | None = None) -> float:
    """Average one or more pressure readings for use as the surface reference."""

    sensor = sensor if sensor is not None else build_depth_sensor_from_config(cfg)
    pressures = []
    for _ in range(max(1, int(samples))):
        row = sensor.read()
        if row.get("error"):
            raise RuntimeError(f"Depth sensor read failed: {row.get('error')}")
        pressures.append(float(row["pressure_mbar"]))
        if delay_s > 0:
            time.sleep(float(delay_s))
    return sum(pressures) / float(len(pressures))


