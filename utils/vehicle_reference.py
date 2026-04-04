from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import numpy as np

from triton_ahrs.calibration import Mount, load_json, save_json
from triton_ahrs.quaternion import Quaternion


DEFAULT_DEPTH_REFERENCE_PATH = "calibration/depth_reference.json"
DEFAULT_FLAT_MOUNT_PATH = "calibration/flat_mount.json"


def _normalize3(v: Iterable[float]) -> Optional[np.ndarray]:
    a = np.array([float(x) for x in v], dtype=float).reshape(3)
    n = float(np.linalg.norm(a))
    if (not math.isfinite(n)) or n <= 1e-12:
        return None
    return a / n


def resolve_path(path: str | Path) -> Path:
    return Path(path).expanduser()


def load_optional_json(path: str | Path) -> Optional[Dict[str, Any]]:
    p = resolve_path(path)
    if not p.exists():
        return None
    try:
        data = load_json(p)
    except Exception:
        return None
    return dict(data) if isinstance(data, dict) else None


def load_surface_pressure_reference_mbar(path: str | Path) -> Optional[float]:
    data = load_optional_json(path)
    if not data:
        return None
    try:
        v = float(data.get("surface_pressure_mbar"))
    except Exception:
        return None
    return v if math.isfinite(v) else None


def save_surface_pressure_reference(path: str | Path, surface_pressure_mbar: float, *, meta: Optional[Dict[str, Any]] = None) -> None:
    payload: Dict[str, Any] = {
        "surface_pressure_mbar": float(surface_pressure_mbar),
        "created_ts": time.time(),
    }
    if meta:
        payload["meta"] = dict(meta)
    save_json(payload, resolve_path(path))


def compute_level_mount(accel_xyz: Iterable[float], *, yaw_deg: float = 0.0) -> Mount:
    a_n = _normalize3(accel_xyz)
    if a_n is None:
        return Mount.identity()

    z_b = np.array([0.0, 0.0, 1.0], dtype=float)
    q_level = Quaternion.from_two_vectors(a_n, z_b)
    yaw_rad = math.radians(float(yaw_deg))
    q_yaw = Quaternion.from_axis_angle((0.0, 0.0, 1.0), yaw_rad) if abs(yaw_rad) > 1e-12 else Quaternion.identity()
    q_extra = (q_yaw * q_level).normalized()
    return Mount(R=q_extra.to_rotation_matrix())


def average_accel_samples(board: Any, *, samples: int, delay_s: float) -> np.ndarray:
    acc = np.zeros(3, dtype=float)
    n = 0
    for _ in range(max(1, int(samples))):
        a = board.read_accel()
        acc += np.array([float(a.x), float(a.y), float(a.z)], dtype=float)
        n += 1
        if delay_s > 0:
            time.sleep(float(delay_s))
    if n <= 0:
        raise RuntimeError("No accelerometer samples captured")
    return acc / float(n)


def save_mount_reference(path: str | Path, mount: Mount, *, meta: Optional[Dict[str, Any]] = None) -> None:
    payload = mount.to_dict()
    payload["created_ts"] = time.time()
    if meta:
        payload["meta"] = dict(meta)
    save_json(payload, resolve_path(path))


def load_mount_reference(path: str | Path) -> Optional[Mount]:
    data = load_optional_json(path)
    if not data:
        return None
    try:
        return Mount.from_dict(data)
    except Exception:
        return None


def build_depth_sensor_from_config(cfg: Any) -> Any:
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


def capture_flat_mount_reference(board: Any, *, samples: int, delay_s: float, yaw_deg: float) -> tuple[Mount, np.ndarray]:
    accel_avg = average_accel_samples(board, samples=samples, delay_s=delay_s)
    return (compute_level_mount(accel_avg, yaw_deg=yaw_deg), accel_avg)
