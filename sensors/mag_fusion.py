"""Magnetometer fusion utilities.

Goal: provide a *single*, more stable magnetometer vector from two sensors
(AK09915 + MMC5983) without breaking backward compatibility.

This is intentionally lightweight:
  - It does NOT attempt a full hard/soft-iron calibration.
  - It reduces *random* noise by averaging and prefers the cleaner sensor.
  - It detects gross disagreement (angle / norm mismatch) and downweights
    the outlier to avoid sudden heading jumps.

For best heading performance you still want:
  - proper mag calibration (hard/soft iron), and
  - an AHRS that uses gyro integration + mag corrections.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def as_dict(self) -> Dict[str, float]:
        return {"x": float(self.x), "y": float(self.y), "z": float(self.z)}


def _norm(v: Vec3) -> float:
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _dot(a: Vec3, b: Vec3) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z


def _add(a: Vec3, b: Vec3) -> Vec3:
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z)


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z)


def _scale(v: Vec3, s: float) -> Vec3:
    return Vec3(v.x * s, v.y * s, v.z * s)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class MagFusion:
    """Fuse two magnetometers into one vector.

    Strategy:
      1) Track an EMA low-pass per sensor.
      2) Track an EMA of squared residuals as a crude noise estimate.
      3) Compute weights ~ (preference / noise).
      4) If sensors disagree strongly, downweight the one farther from the
         learned baseline field magnitude.
      5) Optionally low-pass the fused output.
    """

    def __init__(
        self,
        *,
        enable: bool = True,
        prefer_mmc: float = 1.6,
        prefer_ak: float = 1.0,
        sensor_lpf_alpha: float = 0.20,
        noise_ema_beta: float = 0.05,
        agree_angle_deg: float = 15.0,
        agree_norm_frac: float = 0.12,
        outlier_angle_deg: float = 35.0,
        outlier_norm_frac: float = 0.25,
        output_lpf_tau_s: float = 0.15,
    ):
        self.enable = bool(enable)
        self.prefer_mmc = float(prefer_mmc)
        self.prefer_ak = float(prefer_ak)

        self.sensor_lpf_alpha = float(sensor_lpf_alpha)
        self.noise_ema_beta = float(noise_ema_beta)

        self.agree_angle_deg = float(agree_angle_deg)
        self.agree_norm_frac = float(agree_norm_frac)
        self.outlier_angle_deg = float(outlier_angle_deg)
        self.outlier_norm_frac = float(outlier_norm_frac)

        self.output_lpf_tau_s = float(output_lpf_tau_s)

        # State
        self._lp_ak: Optional[Vec3] = None
        self._lp_mmc: Optional[Vec3] = None
        self._noise_ak: float = 25.0  # uT^2-ish (non-zero warm-start)
        self._noise_mmc: float = 25.0
        self._baseline_B: Optional[float] = None
        self._fused_lp: Optional[Vec3] = None
        self._last_t: Optional[float] = None

    def _update_sensor_stats(self, name: str, m: Vec3) -> None:
        alpha = self.sensor_lpf_alpha
        beta = self.noise_ema_beta

        if name == "ak":
            if self._lp_ak is None:
                self._lp_ak = Vec3(m.x, m.y, m.z)
                return
            resid = _sub(m, self._lp_ak)
            # LPF
            self._lp_ak = _add(self._lp_ak, _scale(resid, alpha))
            # Noise estimate (scalar): EMA of squared residual norm
            r2 = resid.x * resid.x + resid.y * resid.y + resid.z * resid.z
            self._noise_ak = (1.0 - beta) * self._noise_ak + beta * float(r2)

        elif name == "mmc":
            if self._lp_mmc is None:
                self._lp_mmc = Vec3(m.x, m.y, m.z)
                return
            resid = _sub(m, self._lp_mmc)
            self._lp_mmc = _add(self._lp_mmc, _scale(resid, alpha))
            r2 = resid.x * resid.x + resid.y * resid.y + resid.z * resid.z
            self._noise_mmc = (1.0 - beta) * self._noise_mmc + beta * float(r2)

    def fuse(self, ak_uT: Vec3, mmc_uT: Optional[Vec3]) -> Tuple[Vec3, Dict[str, Any]]:
        """Return (fused_vec, meta)."""
        now = time.time()
        self._last_t = now if self._last_t is None else self._last_t

        # Always update AK stats (it is always present)
        self._update_sensor_stats("ak", ak_uT)

        if (not self.enable) or (mmc_uT is None):
            out = ak_uT
            meta = {
                "mode": "disabled" if not self.enable else "single",
                "source": "ak09915",
                "w_ak": 1.0,
                "w_mmc": 0.0,
            }
            return self._output_lpf(out, now), meta

        self._update_sensor_stats("mmc", mmc_uT)

        # Basic comparison metrics
        n_ak = max(_norm(ak_uT), 1e-9)
        n_mmc = max(_norm(mmc_uT), 1e-9)
        cosang = _clamp(_dot(ak_uT, mmc_uT) / (n_ak * n_mmc), -1.0, 1.0)
        ang_deg = math.degrees(math.acos(cosang))
        norm_frac = abs(n_ak - n_mmc) / max(n_ak, n_mmc, 1e-9)

        agree = (ang_deg <= self.agree_angle_deg) and (norm_frac <= self.agree_norm_frac)

        # Baseline magnetic field magnitude (learned when sensors agree)
        if self._baseline_B is None:
            self._baseline_B = 0.5 * (n_ak + n_mmc)
        elif agree:
            # Slow update to avoid chasing disturbances
            self._baseline_B = 0.98 * float(self._baseline_B) + 0.02 * (0.5 * (n_ak + n_mmc))

        B0 = float(self._baseline_B)

        # Weight by (preference / estimated noise)
        eps = 1e-6
        w_ak = self.prefer_ak / max(self._noise_ak, eps)
        w_mmc = self.prefer_mmc / max(self._noise_mmc, eps)

        # If sensors disagree strongly, downweight the one farther from baseline magnitude
        outlier = (ang_deg >= self.outlier_angle_deg) or (norm_frac >= self.outlier_norm_frac)
        if outlier:
            r_ak = abs(n_ak - B0)
            r_mmc = abs(n_mmc - B0)
            if r_ak > r_mmc:
                w_ak *= 0.15
            elif r_mmc > r_ak:
                w_mmc *= 0.15

        # Normalize weights
        w_sum = w_ak + w_mmc
        if w_sum <= 0:
            fused = mmc_uT
            w_ak, w_mmc = 0.0, 1.0
        else:
            fused = _scale(_add(_scale(ak_uT, w_ak), _scale(mmc_uT, w_mmc)), 1.0 / w_sum)

        meta = {
            "mode": "fused",
            "source": "fused",
            "w_ak": float(w_ak),
            "w_mmc": float(w_mmc),
            "angle_deg": float(ang_deg),
            "norm_frac": float(norm_frac),
            "agree": bool(agree),
            "outlier": bool(outlier),
            "baseline_uT": float(B0),
            "noise_ak": float(self._noise_ak),
            "noise_mmc": float(self._noise_mmc),
        }

        return self._output_lpf(fused, now), meta

    def _output_lpf(self, v: Vec3, now: float) -> Vec3:
        """Optional low-pass on the fused output to reduce jitter."""
        tau = self.output_lpf_tau_s
        if tau <= 0:
            return v
        if self._fused_lp is None:
            self._fused_lp = Vec3(v.x, v.y, v.z)
            self._last_t = now
            return self._fused_lp

        dt = max(1e-3, float(now - (self._last_t or now)))
        self._last_t = now
        alpha = dt / (tau + dt)
        resid = _sub(v, self._fused_lp)
        self._fused_lp = _add(self._fused_lp, _scale(resid, alpha))
        return self._fused_lp
