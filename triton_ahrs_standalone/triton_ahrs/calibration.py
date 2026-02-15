from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np


def _to_vec3(v: Iterable[float]) -> np.ndarray:
    a = np.array([float(x) for x in v], dtype=float).reshape(3)
    return a


@dataclass
class GyroCalibration:
    """Simple constant gyro bias calibration (rad/s)."""

    bias_rad_s: np.ndarray  # shape (3,)

    def apply(self, gyro_rad_s: Iterable[float]) -> np.ndarray:
        g = _to_vec3(gyro_rad_s)
        return g - self.bias_rad_s

    def to_dict(self) -> dict:
        return {"bias_rad_s": self.bias_rad_s.tolist()}

    @staticmethod
    def from_dict(d: dict) -> "GyroCalibration":
        b = np.array(d["bias_rad_s"], dtype=float).reshape(3)
        return GyroCalibration(bias_rad_s=b)


@dataclass
class MagCalibration:
    """Mag calibration: m_cal = A @ (m_raw - bias). Units preserved (uT)."""

    bias_uT: np.ndarray  # shape (3,)
    A: np.ndarray        # shape (3,3)

    def apply(self, mag_uT: Iterable[float]) -> np.ndarray:
        m = _to_vec3(mag_uT)
        return self.A @ (m - self.bias_uT)

    def to_dict(self) -> dict:
        return {"bias_uT": self.bias_uT.tolist(), "A": self.A.tolist()}

    @staticmethod
    def from_dict(d: dict) -> "MagCalibration":
        b = np.array(d["bias_uT"], dtype=float).reshape(3)
        A = np.array(d["A"], dtype=float).reshape(3, 3)
        return MagCalibration(bias_uT=b, A=A)


@dataclass
class Mount:
    """Linear mapping from sensor axes to body axes: v_body = R @ v_sensor."""

    R: np.ndarray  # 3x3

    def apply(self, v: Iterable[float]) -> np.ndarray:
        return self.R @ _to_vec3(v)

    def to_dict(self) -> dict:
        return {"R": self.R.tolist()}

    @staticmethod
    def identity() -> "Mount":
        return Mount(R=np.eye(3, dtype=float))

    @staticmethod
    def from_dict(d: dict) -> "Mount":
        R = np.array(d["R"], dtype=float).reshape(3, 3)
        return Mount(R=R)


def save_json(obj_dict: dict, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj_dict, indent=2, sort_keys=True) + "\n")
    tmp.replace(path)


def load_json(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


# -------------------------
# Magnetometer calibration
# -------------------------

def _sphere_fit_center(samples: np.ndarray) -> np.ndarray:
    """Fit a sphere center to samples (Nx3) with linear least squares.

    Solves: ||m - c||^2 = r^2.

    Returns center c.
    """
    x = samples[:, 0]
    y = samples[:, 1]
    z = samples[:, 2]

    # x^2 + y^2 + z^2 - 2cx x - 2cy y - 2cz z + (c^2 - r^2) = 0
    # => [x y z 1] [cx cy cz d]^T = (x^2 + y^2 + z^2)/2
    A = np.stack([x, y, z, np.ones_like(x)], axis=1)
    b = 0.5 * (x*x + y*y + z*z)

    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, cz, _d = sol
    return np.array([cx, cy, cz], dtype=float)


def calibrate_mag_softiron(samples_uT: np.ndarray) -> MagCalibration:
    """Compute a robust hard-iron + soft-iron calibration.

    Method (practical + stable):
      1) Fit sphere center -> hard-iron bias.
      2) After bias removal, compute covariance and build a whitening transform.
         With good coverage of orientations, this maps the ellipsoid to a sphere.
      3) Scale the whitening matrix to roughly preserve field magnitude.

    Requires samples that cover many orientations (rotate slowly through all axes).
    """
    if samples_uT.ndim != 2 or samples_uT.shape[1] != 3:
        raise ValueError("samples_uT must be Nx3")
    if samples_uT.shape[0] < 200:
        raise ValueError("Need at least ~200 samples for a decent calibration")

    # 1) hard-iron bias
    bias = _sphere_fit_center(samples_uT)
    centered = samples_uT - bias[None, :]

    # 2) soft-iron via whitening of covariance
    # (Assumes near-uniform sampling over the sphere.)
    C = np.cov(centered.T)

    # Symmetrize + guard against numerical issues
    C = 0.5 * (C + C.T)
    w, V = np.linalg.eigh(C)

    # Avoid divide-by-zero if sampling was poor
    w = np.clip(w, 1e-12, None)

    # Whitening: V diag(1/sqrt(w)) V^T
    W = V @ np.diag(1.0 / np.sqrt(w)) @ V.T

    # 3) scale factor to keep magnitudes roughly in uT range
    # We use the mean axis stddev as a reasonable "radius".
    s = float(np.mean(np.sqrt(w)))
    A = s * W

    return MagCalibration(bias_uT=bias, A=A)


def mag_baseline_uT(mag_cal: MagCalibration, samples_uT: np.ndarray) -> float:
    """Estimate baseline field magnitude from samples (after calibration)."""
    cal = (mag_cal.A @ (samples_uT - mag_cal.bias_uT).T).T
    mags = np.linalg.norm(cal, axis=1)
    return float(np.median(mags))
