"""Attitude estimation (AHRS) sensor.

Publishes a high-level orientation message derived from the Navigator IMU.

Design goals:
- Very stable roll/pitch via 6DOF Madgwick (gyro + accel) with accel gating.
- Reliable yaw / heading via a *robust* magnetometer yaw correction path that:
    * rejects magnitude spikes / step jumps
    * uses hysteresis so mag doesn't rapidly toggle
    * applies yaw correction with a long time constant (smooth)
    * can learn a small Z-gyro bias from sustained mag yaw error

This is derived from the standalone AHRS in `triton_ahrs/run_ahrs.py`, adapted
into a polled sensor that fits TritonOS's SensorPublisherService.

Message format:
{
  "type": "attitude",
  "sensor": "attitude",
  "ts": <unix seconds>,
  "rpy_deg": {"roll":.., "pitch":.., "yaw":..},
  "q": {"w":..,"x":..,"y":..,"z":..},
  "health": {...},
  "mag": {"x":..,"y":..,"z":..},
  "mag_source": "mmc5983"|"ak09915"|"fused",
}

Topside can ignore any extra fields it doesn't know.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from sensors.base import BaseSensor
from sensors.mag_fusion import MagFusion, Vec3 as MFVec3

from triton_ahrs.calibration import GyroCalibration, MagCalibration, Mount, load_json
from triton_ahrs.madgwick import MadgwickAHRS, MadgwickConfig
from triton_ahrs.quaternion import Quaternion, quat_to_euler_deg, wrap_degrees

G = 9.80665


class EMA3:
    """3D exponential moving average with time-constant tau (seconds)."""

    def __init__(self, tau_s: float):
        self.tau = float(max(0.0, tau_s))
        self.x: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.x = None

    def update(self, v: np.ndarray, dt: float) -> np.ndarray:
        v = np.asarray(v, dtype=float)
        if self.tau <= 0.0:
            self.x = v
            return v
        if self.x is None:
            self.x = v
            return v
        a = float(dt / (self.tau + dt))
        self.x = (1.0 - a) * self.x + a * v
        return self.x


def _normalize3(v: np.ndarray) -> Optional[np.ndarray]:
    n = float(np.linalg.norm(v))
    if not math.isfinite(n) or n <= 1e-12:
        return None
    return v / n


def _normalize2(v: np.ndarray) -> Optional[np.ndarray]:
    n = float(np.linalg.norm(v))
    if not math.isfinite(n) or n <= 1e-12:
        return None
    return v / n


def _mag_health(
    mag_uT: np.ndarray,
    baseline_uT: Optional[float],
    tol_ratio: float,
    max_step_uT: float,
    prev_mag_norm_uT: Optional[float],
) -> Tuple[bool, float, float]:
    """Return (ok, mag_norm, step)."""
    mag_norm = float(np.linalg.norm(mag_uT))
    step = float(abs(mag_norm - prev_mag_norm_uT)) if prev_mag_norm_uT is not None else 0.0

    if not math.isfinite(mag_norm) or mag_norm <= 1e-9:
        return (False, mag_norm, step)

    if baseline_uT is not None and baseline_uT > 1e-9:
        lo = baseline_uT * (1.0 - tol_ratio)
        hi = baseline_uT * (1.0 + tol_ratio)
        if not (lo <= mag_norm <= hi):
            return (False, mag_norm, step)

    if max_step_uT > 0 and step > max_step_uT:
        return (False, mag_norm, step)

    return (True, mag_norm, step)


def _initial_quaternion_from_accel_mag(accel: np.ndarray, mag: Optional[np.ndarray]) -> Quaternion:
    """Seed quaternion from accel (+ optional mag).

    accel and mag are in BODY coordinates.

    WORLD +Z aligns with accel direction ("up" seen by IMU at rest).
    Yaw chosen so horizontal mag projection points along WORLD +X.
    """
    a = _normalize3(accel)
    if a is None:
        return Quaternion.identity()

    z_w = np.array([0.0, 0.0, 1.0], dtype=float)
    q_tilt = Quaternion.from_two_vectors(a, z_w)

    if mag is None:
        return q_tilt

    m = _normalize3(mag)
    if m is None:
        return q_tilt

    m_w = q_tilt.rotate(m)
    mh = np.array([m_w[0], m_w[1], 0.0], dtype=float)
    mh_n = float(np.linalg.norm(mh))
    if not math.isfinite(mh_n) or mh_n <= 1e-8:
        return q_tilt
    mh /= mh_n

    yaw = math.atan2(float(mh[1]), float(mh[0]))
    q_yaw = Quaternion.from_axis_angle((0.0, 0.0, 1.0), -yaw)

    return (q_yaw * q_tilt).normalized()


def _quaternion_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    """Build a BODY->WORLD quaternion from aerospace ZYX Euler angles."""
    q_roll = Quaternion.from_axis_angle((1.0, 0.0, 0.0), math.radians(float(roll_deg)))
    q_pitch = Quaternion.from_axis_angle((0.0, 1.0, 0.0), math.radians(float(pitch_deg)))
    q_yaw = Quaternion.from_axis_angle((0.0, 0.0, 1.0), math.radians(float(yaw_deg)))
    return (q_yaw * q_pitch * q_roll).normalized()


def _direct_mag_yaw_deg(mag_body_uT: np.ndarray, roll_deg: float, pitch_deg: float) -> Optional[float]:
    """Tilt-compensated yaw directly from magnetometer + current roll/pitch."""
    q_rp = _quaternion_from_rpy_deg(roll_deg, pitch_deg, 0.0)
    m_w = q_rp.rotate(mag_body_uT)
    mh = np.array([m_w[0], m_w[1]], dtype=float)
    mh = _normalize2(mh)
    if mh is None:
        return None
    return wrap_degrees(-math.degrees(math.atan2(float(mh[1]), float(mh[0]))))


def _load_optional_json(path: str) -> Optional[dict]:
    p = (path or "").strip()
    if not p:
        return None
    try:
        return load_json(p)
    except Exception:
        return None


class AttitudeSensor(BaseSensor):
    def __init__(self, board, rate_hz: float = 50.0):
        super().__init__("attitude", float(rate_hz))
        self.board = board

        # ---------------- config defaults ----------------
        fusion = "robust"  # robust (recommended) or madgwick
        init_seconds = 0.8

        # Madgwick beta scheduling
        beta = 0.08
        beta_init = 0.60
        beta_stationary = 0.12
        warmup_seconds = 1.5

        # Robust yaw correction
        yaw_tau = 8.0
        yaw_max_err_deg = 25.0
        yaw_ki = 0.02
        yaw_bias_max_dps = 5.0
        yaw_bias_adapt_err_deg = 10.0
        yaw_bias_adapt_gyro_rad = 0.35
        yaw_bias_adapt_gyro_norm = 0.5
        mag_ref_tau = 300.0

        # Sensor gating / health
        accel_g_tol = 0.20
        stationary_gyro_rad = 0.20
        bias_adapt_tau = 60.0

        mag_tol = 0.35
        mag_max_step = 8.0
        mag_enable_up = 0.75
        mag_enable_down = 0.35

        # Filtering
        accel_lpf_tau = 0.05
        mag_lpf_tau = 0.20
        gyro_lpf_tau = 0.00

        # Motion gating for yaw correction (prevents tilt-error-induced yaw corruption)
        yaw_dynamic_gate = 1.5    # m/s² dynamic accel threshold (0 disables)
        yaw_gyro_gate_dps = 10.0  # deg/s gyro rate threshold (0 disables)

        # Output smoothing
        output_lpf_tau = 0.15     # seconds (0 disables)

        # Presentation options
        accel_sign = "auto"  # auto|normal|invert
        zero_attitude = False
        yaw_zero = False
        yaw_mode = "fused"  # fused|direct_mag

        # Auto-mount (boot-time leveling)
        # If the IMU board (or Pi/Navigator) is mounted with some tilt inside the vehicle,
        # roll/pitch will be coupled and the attitude will not match the vehicle body axes.
        #
        # When the vehicle is known to be "level" at boot, we can estimate a *tilt-only*
        # correction that aligns the measured accel direction with +Z in the body frame.
        # This does **not** determine the remaining yaw-about-Z mounting angle (that
        # requires additional information), but in practice it fixes the big issue:
        # roll/pitch mixing from a non-level installation.
        auto_mount_enable = False
        auto_mount_yaw_deg = 0.0  # optional extra yaw rotation (deg) applied after leveling
        auto_mount_save_path = ""  # optional: save computed mount json for reuse
        auto_mount_with_saved_mount = False

        # Calibration files (optional)
        gyro_cal_path = ""
        mag_cal_path = ""
        mount_path = ""

        # Magnetometer selection (reuse the same settings as IMUSensor by default)
        mag_output_mode = "fused"  # fused|mmc|ak
        mag_fusion_enable = True
        mag_prefer_mmc = 1.6
        mag_prefer_ak = 1.0
        mag_sensor_lpf_alpha = 0.20
        mag_noise_ema_beta = 0.05
        mag_agree_angle_deg = 15.0
        mag_agree_norm_frac = 0.12
        mag_outlier_angle_deg = 35.0
        mag_outlier_norm_frac = 0.25
        mag_output_lpf_tau_s = 0.15
        mag_rate_hz = 25.0  # throttle mag reads (Hz) to reduce bus load

        # Read overrides from rov_config
        try:
            import rov_config as cfg  # type: ignore

            fusion = str(getattr(cfg, "ATTITUDE_FUSION", fusion)).strip().lower()
            init_seconds = float(getattr(cfg, "ATTITUDE_INIT_SECONDS", init_seconds))

            beta = float(getattr(cfg, "ATTITUDE_BETA", beta))
            beta_init = float(getattr(cfg, "ATTITUDE_BETA_INIT", beta_init))
            beta_stationary = float(getattr(cfg, "ATTITUDE_BETA_STATIONARY", beta_stationary))
            warmup_seconds = float(getattr(cfg, "ATTITUDE_WARMUP_SECONDS", warmup_seconds))

            yaw_tau = float(getattr(cfg, "ATTITUDE_YAW_TAU", yaw_tau))
            yaw_max_err_deg = float(getattr(cfg, "ATTITUDE_YAW_MAX_ERR_DEG", yaw_max_err_deg))
            yaw_ki = float(getattr(cfg, "ATTITUDE_YAW_KI", yaw_ki))
            yaw_bias_max_dps = float(getattr(cfg, "ATTITUDE_YAW_BIAS_MAX_DPS", yaw_bias_max_dps))
            yaw_bias_adapt_err_deg = float(getattr(cfg, "ATTITUDE_YAW_BIAS_ADAPT_ERR_DEG", yaw_bias_adapt_err_deg))
            yaw_bias_adapt_gyro_rad = float(getattr(cfg, "ATTITUDE_YAW_BIAS_ADAPT_GYRO_RAD", yaw_bias_adapt_gyro_rad))
            yaw_bias_adapt_gyro_norm = float(getattr(cfg, "ATTITUDE_YAW_BIAS_ADAPT_GYRO_NORM", yaw_bias_adapt_gyro_norm))
            mag_ref_tau = float(getattr(cfg, "ATTITUDE_MAG_REF_TAU", mag_ref_tau))

            accel_g_tol = float(getattr(cfg, "ATTITUDE_ACCEL_G_TOL", accel_g_tol))
            stationary_gyro_rad = float(getattr(cfg, "ATTITUDE_STATIONARY_GYRO_RAD", stationary_gyro_rad))
            bias_adapt_tau = float(getattr(cfg, "ATTITUDE_BIAS_ADAPT_TAU", bias_adapt_tau))

            mag_tol = float(getattr(cfg, "ATTITUDE_MAG_TOL", mag_tol))
            mag_max_step = float(getattr(cfg, "ATTITUDE_MAG_MAX_STEP", mag_max_step))
            mag_enable_up = float(getattr(cfg, "ATTITUDE_MAG_ENABLE_UP", mag_enable_up))
            mag_enable_down = float(getattr(cfg, "ATTITUDE_MAG_ENABLE_DOWN", mag_enable_down))

            accel_lpf_tau = float(getattr(cfg, "ATTITUDE_ACCEL_LPF_TAU_S", accel_lpf_tau))
            mag_lpf_tau = float(getattr(cfg, "ATTITUDE_MAG_LPF_TAU_S", mag_lpf_tau))
            gyro_lpf_tau = float(getattr(cfg, "ATTITUDE_GYRO_LPF_TAU_S", gyro_lpf_tau))

            accel_sign = str(getattr(cfg, "ATTITUDE_ACCEL_SIGN", accel_sign)).strip().lower()
            zero_attitude = bool(getattr(cfg, "ATTITUDE_ZERO_ATTITUDE_AT_START", zero_attitude))
            yaw_zero = bool(getattr(cfg, "ATTITUDE_YAW_ZERO_AT_START", yaw_zero))
            yaw_mode = str(getattr(cfg, "ATTITUDE_YAW_MODE", yaw_mode)).strip().lower()

            auto_mount_enable = bool(getattr(cfg, "ATTITUDE_AUTO_MOUNT_FROM_LEVEL", auto_mount_enable))
            auto_mount_yaw_deg = float(getattr(cfg, "ATTITUDE_AUTO_MOUNT_YAW_DEG", auto_mount_yaw_deg))
            auto_mount_save_path = str(getattr(cfg, "ATTITUDE_AUTO_MOUNT_SAVE_PATH", auto_mount_save_path))
            auto_mount_with_saved_mount = bool(
                getattr(cfg, "ATTITUDE_AUTO_MOUNT_WITH_SAVED_MOUNT", auto_mount_with_saved_mount)
            )

            gyro_cal_path = str(getattr(cfg, "ATTITUDE_GYRO_CAL", gyro_cal_path))
            mag_cal_path = str(getattr(cfg, "ATTITUDE_MAG_CAL", mag_cal_path))
            mount_path = str(getattr(cfg, "ATTITUDE_MOUNT", mount_path))

            # By default, follow the magnetometer mode used by IMUSensor
            mag_output_mode = str(getattr(cfg, "ATTITUDE_MAG_OUTPUT_MODE", getattr(cfg, "MAG_OUTPUT_MODE", mag_output_mode))).strip().lower()

            # Follow fusion settings too, unless explicitly overridden
            mag_fusion_enable = bool(getattr(cfg, "ATTITUDE_MAG_FUSION_ENABLE", getattr(cfg, "MAG_FUSION_ENABLE", mag_fusion_enable)))
            mag_prefer_mmc = float(getattr(cfg, "ATTITUDE_MAG_FUSION_PREFER_MMC", getattr(cfg, "MAG_FUSION_PREFER_MMC", mag_prefer_mmc)))
            mag_prefer_ak = float(getattr(cfg, "ATTITUDE_MAG_FUSION_PREFER_AK", getattr(cfg, "MAG_FUSION_PREFER_AK", mag_prefer_ak)))
            mag_sensor_lpf_alpha = float(getattr(cfg, "ATTITUDE_MAG_FUSION_SENSOR_LPF_ALPHA", getattr(cfg, "MAG_FUSION_SENSOR_LPF_ALPHA", mag_sensor_lpf_alpha)))
            mag_noise_ema_beta = float(getattr(cfg, "ATTITUDE_MAG_FUSION_NOISE_EMA_BETA", getattr(cfg, "MAG_FUSION_NOISE_EMA_BETA", mag_noise_ema_beta)))
            mag_agree_angle_deg = float(getattr(cfg, "ATTITUDE_MAG_FUSION_AGREE_ANGLE_DEG", getattr(cfg, "MAG_FUSION_AGREE_ANGLE_DEG", mag_agree_angle_deg)))
            mag_agree_norm_frac = float(getattr(cfg, "ATTITUDE_MAG_FUSION_AGREE_NORM_FRAC", getattr(cfg, "MAG_FUSION_AGREE_NORM_FRAC", mag_agree_norm_frac)))
            mag_outlier_angle_deg = float(getattr(cfg, "ATTITUDE_MAG_FUSION_OUTLIER_ANGLE_DEG", getattr(cfg, "MAG_FUSION_OUTLIER_ANGLE_DEG", mag_outlier_angle_deg)))
            mag_outlier_norm_frac = float(getattr(cfg, "ATTITUDE_MAG_FUSION_OUTLIER_NORM_FRAC", getattr(cfg, "MAG_FUSION_OUTLIER_NORM_FRAC", mag_outlier_norm_frac)))
            mag_output_lpf_tau_s = float(getattr(cfg, "ATTITUDE_MAG_FUSION_OUTPUT_LPF_TAU_S", getattr(cfg, "MAG_FUSION_OUTPUT_LPF_TAU_S", mag_output_lpf_tau_s)))
            mag_rate_hz = float(getattr(cfg, "ATTITUDE_MAG_RATE_HZ", mag_rate_hz))

            yaw_dynamic_gate = float(getattr(cfg, "ATTITUDE_YAW_DYNAMIC_GATE", yaw_dynamic_gate))
            yaw_gyro_gate_dps = float(getattr(cfg, "ATTITUDE_YAW_GYRO_GATE_DPS", yaw_gyro_gate_dps))
            output_lpf_tau = float(getattr(cfg, "ATTITUDE_OUTPUT_LPF_TAU_S", output_lpf_tau))
        except Exception:
            pass

        # Store config
        self._fusion = fusion if fusion in ("robust", "madgwick") else "robust"
        self._init_seconds = float(max(0.0, init_seconds))

        self._beta = float(beta)
        self._beta_init = float(beta_init)
        self._beta_stationary = float(beta_stationary)
        self._warmup_seconds = float(max(0.0, warmup_seconds))

        self._yaw_tau = float(max(1e-3, yaw_tau))
        self._yaw_max_err = math.radians(float(max(0.1, yaw_max_err_deg)))
        self._yaw_ki = float(max(0.0, yaw_ki))
        self._yaw_bias_max = math.radians(float(max(0.0, yaw_bias_max_dps)))
        self._yaw_bias_adapt_err = math.radians(float(max(0.0, yaw_bias_adapt_err_deg)))
        self._yaw_bias_adapt_gyro_rad = float(max(0.0, yaw_bias_adapt_gyro_rad))
        self._yaw_bias_adapt_gyro_norm = float(max(0.0, yaw_bias_adapt_gyro_norm))
        self._mag_ref_tau = float(max(0.0, mag_ref_tau))

        self._accel_g_tol = float(max(0.0, accel_g_tol))
        self._stationary_gyro_rad = float(max(0.0, stationary_gyro_rad))
        self._bias_adapt_tau = float(max(0.0, bias_adapt_tau))

        self._mag_tol = float(max(0.0, mag_tol))
        self._mag_max_step = float(max(0.0, mag_max_step))
        self._mag_enable_up = float(max(1e-3, mag_enable_up))
        self._mag_enable_down = float(max(1e-3, mag_enable_down))

        self._yaw_dynamic_gate = float(max(0.0, yaw_dynamic_gate))
        self._yaw_gyro_gate_rad = math.radians(float(max(0.0, yaw_gyro_gate_dps)))

        self._accel_sign_mode = accel_sign if accel_sign in ("auto", "normal", "invert") else "auto"
        self._zero_attitude = bool(zero_attitude)
        self._yaw_zero = bool(yaw_zero)
        self._yaw_mode = yaw_mode if yaw_mode in ("fused", "direct_mag") else "fused"

        self._auto_mount_enable = bool(auto_mount_enable)
        self._auto_mount_yaw_deg = float(auto_mount_yaw_deg)
        self._auto_mount_save_path = str(auto_mount_save_path)
        self._auto_mount_with_saved_mount = bool(auto_mount_with_saved_mount)

        # Filters
        self._lpf_a = EMA3(accel_lpf_tau)
        self._lpf_m = EMA3(mag_lpf_tau)
        self._lpf_g = EMA3(gyro_lpf_tau)

        # Output smoothing (applied to final Euler angles)
        self._output_lpf_tau = float(max(0.0, output_lpf_tau))
        self._out_roll: Optional[float] = None
        self._out_pitch: Optional[float] = None
        self._out_yaw: Optional[float] = None

        # Calibrations
        self._mount = Mount.identity()
        self._mount_loaded = False
        self._gyro_cal: Optional[GyroCalibration] = None
        self._mag_cal: Optional[MagCalibration] = None
        self._baseline_uT_from_cal: Optional[float] = None

        try:
            d = _load_optional_json(mount_path)
            if d is not None:
                self._mount = Mount.from_dict(d)
                self._mount_loaded = True
        except Exception:
            self._mount = Mount.identity()
            self._mount_loaded = False

        if self._mount_loaded and (not self._auto_mount_with_saved_mount):
            self._auto_mount_enable = False

        try:
            d = _load_optional_json(gyro_cal_path)
            if d is not None:
                self._gyro_cal = GyroCalibration.from_dict(d)
        except Exception:
            self._gyro_cal = None

        try:
            d = _load_optional_json(mag_cal_path)
            if d is not None:
                self._mag_cal = MagCalibration.from_dict(d)
                # Optional extra field produced by calibrate_mag.py
                try:
                    bu = d.get('baseline_uT', None)
                    self._baseline_uT_from_cal = float(bu) if bu is not None else None
                except Exception:
                    self._baseline_uT_from_cal = None
        except Exception:
            self._mag_cal = None
            self._baseline_uT_from_cal = None

        # Magnetometer selection / fusion
        self._mag_output_mode = (mag_output_mode or "fused").strip().lower()
        self._mag_fusion = MagFusion(
            enable=bool(mag_fusion_enable),
            prefer_mmc=float(mag_prefer_mmc),
            prefer_ak=float(mag_prefer_ak),
            sensor_lpf_alpha=float(mag_sensor_lpf_alpha),
            noise_ema_beta=float(mag_noise_ema_beta),
            agree_angle_deg=float(mag_agree_angle_deg),
            agree_norm_frac=float(mag_agree_norm_frac),
            outlier_angle_deg=float(mag_outlier_angle_deg),
            outlier_norm_frac=float(mag_outlier_norm_frac),
            output_lpf_tau_s=float(mag_output_lpf_tau_s),
        )

        self._mag_min_interval_s = 0.0 if float(mag_rate_hz) <= 0 else 1.0 / float(mag_rate_hz)
        self._mag_cache_perf: Optional[float] = None
        self._mag_cache: Optional[Tuple[Optional[np.ndarray], str, Dict[str, Any]]] = None

        # State
        self._filt = MadgwickAHRS(cfg=MadgwickConfig(beta=self._beta_init))
        self._start_perf = time.perf_counter()
        self._last_perf: Optional[float] = None

        self._gyro_bias = np.zeros(3, dtype=float)
        self._mag_ref_h: Optional[np.ndarray] = None  # 2D unit vector in world
        self._mag_enable = 0.0
        self._baseline_uT: Optional[float] = None
        self._prev_mag_norm: Optional[float] = None

        self._q_zero: Optional[Quaternion] = None
        self._yaw0: Optional[float] = None
        self._accel_sign_used = +1.0

        # Dynamic acceleration estimator (for yaw motion gating)
        self._prev_accel_v: Optional[np.ndarray] = None
        self._dynamic_accel_ema = 0.0

        # For debugging/telemetry
        self._mount_auto_applied = False
        self._mount_auto_R: Optional[np.ndarray] = None

        # Bootstrap initial attitude so the output is stable quickly.
        self._bootstrap_initial_alignment()

    # ---------------- internal helpers ----------------

    def _read_mag_selected(self) -> Tuple[Optional[np.ndarray], str, Dict[str, Any]]:
        """Return (mag_uT_vec3, mag_source, meta)."""
        # Throttle magnetometer reads: yaw correction does not need full AHRS rate,
        # and some buses can be sensitive to being polled too fast.
        nowp = time.perf_counter()
        if self._mag_cache is not None and self._mag_cache_perf is not None:
            if self._mag_min_interval_s > 0 and (nowp - float(self._mag_cache_perf)) < self._mag_min_interval_s:
                return self._mag_cache
        mags = self.board.read_mags()
        ak = mags.get("ak09915") or {"x": 0.0, "y": 0.0, "z": 0.0}
        mmc = mags.get("mmc5983")

        ak_v = MFVec3(float(ak["x"]), float(ak["y"]), float(ak["z"]))
        mmc_v = None if mmc is None else MFVec3(float(mmc["x"]), float(mmc["y"]), float(mmc["z"]))

        mode = self._mag_output_mode
        if mode in ("ak", "ak_only", "ak09915"):
            v = ak_v
            meta = {"mode": "ak_only", "source": "ak09915", "w_ak": 1.0, "w_mmc": 0.0}
        elif mode in ("mmc", "mmc_only", "mmc5983", "mmc5983_only"):
            if mmc_v is not None:
                v = mmc_v
                meta = {"mode": "mmc_only", "source": "mmc5983", "w_ak": 0.0, "w_mmc": 1.0}
            else:
                v = ak_v
                meta = {"mode": "mmc_only_fallback", "source": "ak09915", "w_ak": 1.0, "w_mmc": 0.0}
        else:
            v, meta = self._mag_fusion.fuse(ak_v, mmc_v)

        mv = np.array([float(v.x), float(v.y), float(v.z)], dtype=float)
        ret = (mv, str(meta.get("source", "ak09915")), {"mag_sources": mags, "mag_fusion": meta})
        self._mag_cache_perf = nowp
        self._mag_cache = ret
        return ret

    def _bootstrap_initial_alignment(self) -> None:
        secs = float(self._init_seconds)
        if secs <= 0.0:
            return

        t_end = time.perf_counter() + secs
        acc_a = np.zeros(3, dtype=float)
        acc_m = np.zeros(3, dtype=float)
        acc_g = np.zeros(3, dtype=float)
        k_a = 0
        k_m = 0
        k_g = 0

        # Small sleep to avoid hammering buses.
        dt = 0.01

        mag_src = "none"
        mag_meta: Dict[str, Any] = {}

        while time.perf_counter() < t_end:
            try:
                a = self.board.read_accel()
                g = self.board.read_gyro()

                av = self._mount.apply((a.x, a.y, a.z))
                gv = self._mount.apply((g.x, g.y, g.z))
                acc_a += av
                acc_g += gv
                k_a += 1
                k_g += 1

                mv, mag_src, mag_meta = self._read_mag_selected()
                if mv is not None:
                    mvb = self._mount.apply(mv)
                    if self._mag_cal is not None:
                        mvb = self._mag_cal.apply(mvb)
                    acc_m += mvb
                    k_m += 1
            except Exception:
                pass
            time.sleep(dt)

        if k_a <= 0 or k_g <= 0:
            return

        a_avg = acc_a / float(k_a)
        g_avg = acc_g / float(k_g)
        m_avg = (acc_m / float(k_m)) if k_m > 0 else None

        # Choose accel sign if requested (fixes 180° roll flips from wiring/axis conventions)
        accel_sign_used = +1.0
        a_use = a_avg

        if self._accel_sign_mode == "invert":
            accel_sign_used = -1.0
            a_use = -a_avg
        elif self._accel_sign_mode == "auto":
            # Pick the sign that yields smaller |roll|+|pitch| for the init quaternion.
            q1 = _initial_quaternion_from_accel_mag(a_avg, m_avg)
            r1, p1, _ = quat_to_euler_deg(q1)
            q2 = _initial_quaternion_from_accel_mag(-a_avg, m_avg)
            r2, p2, _ = quat_to_euler_deg(q2)
            if (abs(r2) + abs(p2)) < (abs(r1) + abs(p1)):
                accel_sign_used = -1.0
                a_use = -a_avg

        self._accel_sign_used = float(accel_sign_used)

        # Optional: estimate a tilt-only mount correction from the known "level" boot pose.
        # This updates self._mount so all subsequent sensor vectors are expressed in the
        # vehicle body axes (up to an unknown yaw-about-Z).
        if self._auto_mount_enable:
            a_n = _normalize3(a_use)
            if a_n is not None:
                z_b = np.array([0.0, 0.0, 1.0], dtype=float)
                q_level = Quaternion.from_two_vectors(a_n, z_b)  # old_body -> leveled_body

                # Optional extra yaw (about +Z in the leveled body frame)
                yaw_rad = math.radians(float(self._auto_mount_yaw_deg))
                q_yaw = (
                    Quaternion.from_axis_angle((0.0, 0.0, 1.0), yaw_rad)
                    if abs(yaw_rad) > 1e-12
                    else Quaternion.identity()
                )

                q_extra = (q_yaw * q_level).normalized()
                R_extra = q_extra.to_rotation_matrix()  # old_body -> new_body

                # Compose with any user-provided mount file: v_new_body = R_extra @ (R_user @ v_sensor)
                self._mount = Mount(R=R_extra @ self._mount.R)
                self._mount_auto_applied = True
                self._mount_auto_R = R_extra

                # Apply the same correction to our averages so the init quaternion is consistent.
                a_use = R_extra @ a_use
                g_avg = R_extra @ g_avg
                if m_avg is not None:
                    m_avg = R_extra @ m_avg

                # Optionally persist the computed mount matrix for reuse.
                p = (self._auto_mount_save_path or "").strip()
                if p:
                    try:
                        from triton_ahrs.calibration import save_json

                        save_json(
                            {
                                "R": self._mount.R.tolist(),
                                "meta": {
                                    "auto_from_level": True,
                                    "auto_yaw_deg": float(self._auto_mount_yaw_deg),
                                    "created_ts": time.time(),
                                },
                            },
                            p,
                        )
                    except Exception:
                        pass

        q_init = _initial_quaternion_from_accel_mag(a_use, m_avg)
        self._filt.q = q_init

        # Seed baseline mag norm for health gating (median-ish)
        if m_avg is not None:
            self._baseline_uT = float(self._baseline_uT_from_cal) if self._baseline_uT_from_cal is not None else float(np.linalg.norm(m_avg))
            # With the init quaternion we defined world X along horizontal mag,
            # so the reference vector is effectively +X.
            self._mag_ref_h = np.array([1.0, 0.0], dtype=float)
        else:
            self._baseline_uT = None
            self._mag_ref_h = None

        # Seed gyro bias very gently from the init window (helps reduce initial yaw drift)
        # Note: This is *not* a full calibration; it just avoids obviously wrong biases.
        self._gyro_bias = np.clip(g_avg, -0.2, 0.2)

        if self._zero_attitude:
            self._q_zero = q_init

        # Optionally set yaw-zero reference once we have a quaternion.
        if self._yaw_zero:
            r, p, y = quat_to_euler_deg(q_init if self._q_zero is None else Quaternion.identity())
            self._yaw0 = y

    # ---------------- main loop ----------------

    def read(self) -> Dict[str, Any]:
        now_perf = time.perf_counter()
        if self._last_perf is None:
            dt = 1.0 / max(1.0, float(self.rate_hz))
        else:
            dt = float(now_perf - self._last_perf)
        self._last_perf = now_perf

        # Clamp dt (handles stalls / debugger pauses)
        dt = max(1e-4, min(0.25, dt))

        # Read sensors (combined burst avoids accel/gyro temporal skew)
        try:
            a, g = self.board.read_imu()
        except AttributeError:
            a = self.board.read_accel()
            g = self.board.read_gyro()
        mv_raw, mag_src, mag_meta = self._read_mag_selected()

        av = self._mount.apply((a.x, a.y, a.z))
        gv_raw = self._mount.apply((g.x, g.y, g.z))

        # Apply accel sign
        av = float(self._accel_sign_used) * av

        # Apply calibrations
        if self._gyro_cal is not None:
            gv_raw = self._gyro_cal.apply(gv_raw)
        if mv_raw is not None:
            mv = self._mount.apply(mv_raw)
            if self._mag_cal is not None:
                mv = self._mag_cal.apply(mv)
        else:
            mv = None

        # LPFs
        av = self._lpf_a.update(av, dt)
        if mv is not None:
            mv = self._lpf_m.update(mv, dt)
        gv_raw = self._lpf_g.update(gv_raw, dt)

        # Bias-correct gyro for filter update
        gv = gv_raw - self._gyro_bias
        gyro_norm = float(np.linalg.norm(gv))

        # Estimate dynamic (non-gravitational) acceleration.
        # In the body frame, gravity changes as dg/dt = -omega x g due to rotation.
        # Any measured accel change beyond that is from vehicle thrust / drag.
        if self._prev_accel_v is not None:
            accel_change = av - self._prev_accel_v
            gravity_change = -np.cross(gv, self._prev_accel_v) * dt
            dynamic_impulse = accel_change - gravity_change
            dynamic_norm = float(np.linalg.norm(dynamic_impulse)) / max(dt, 1e-6)
            tau_dyn = 0.15  # seconds - smoothing on the estimate
            alpha_dyn = dt / (dt + tau_dyn)
            self._dynamic_accel_ema = (1.0 - alpha_dyn) * self._dynamic_accel_ema + alpha_dyn * dynamic_norm
        self._prev_accel_v = av.copy()

        # Stationary detection
        a_norm = float(np.linalg.norm(av))
        accel_ok = bool(abs(a_norm - G) <= float(self._accel_g_tol) * G)
        stationary = bool(accel_ok and gyro_norm <= float(self._stationary_gyro_rad))

        # Slow gyro bias adaptation (when stationary)
        if stationary and self._bias_adapt_tau > 0.0:
            alpha = float(dt / max(1e-3, float(self._bias_adapt_tau)))
            self._gyro_bias = (1.0 - alpha) * self._gyro_bias + alpha * gv_raw
            gv = gv_raw - self._gyro_bias
            gyro_norm = float(np.linalg.norm(gv))

        # Mag health + hysteresis
        mag_ok = False
        mag_norm = float("nan")
        mag_step = 0.0

        if mv is not None:
            mag_ok, mag_norm, mag_step = _mag_health(
                mv,
                self._baseline_uT,
                tol_ratio=float(self._mag_tol),
                max_step_uT=float(self._mag_max_step),
                prev_mag_norm_uT=self._prev_mag_norm,
            )
            self._prev_mag_norm = mag_norm

        if mag_ok:
            self._mag_enable = min(1.0, float(self._mag_enable) + dt / self._mag_enable_up)
        else:
            self._mag_enable = max(0.0, float(self._mag_enable) - dt / self._mag_enable_down)

        use_mag = bool(accel_ok and mv is not None and self._mag_enable >= 0.8)

        # Soft quality weight 0..1
        mag_qual = 0.0
        if mv is not None and mag_ok:
            q1 = 1.0
            if self._baseline_uT is not None and self._baseline_uT > 1e-6:
                rel = abs(mag_norm - float(self._baseline_uT)) / float(self._baseline_uT)
                q1 = max(0.0, 1.0 - rel / max(1e-6, float(self._mag_tol)))
            q2 = 1.0
            if float(self._mag_max_step) > 1e-6:
                q2 = max(0.0, 1.0 - (mag_step / float(self._mag_max_step)))
            mag_qual = max(0.0, min(1.0, q1 * q2))

        # Beta scheduling
        t_since = float(now_perf - self._start_perf)
        beta = float(self._beta)
        if t_since < float(self._warmup_seconds):
            beta = float(self._beta_init)
        elif stationary:
            beta = max(beta, float(self._beta_stationary))
        self._filt.cfg.beta = beta

        yaw_err = 0.0
        yaw_delta = 0.0
        mode = "gyro"

        if self._fusion == "madgwick":
            if accel_ok and use_mag and mv is not None:
                mode = "9dof"
                q = self._filt.update(gv, av, dt, mag_uT=mv)
            elif accel_ok:
                mode = "6dof"
                q = self._filt.update(gv, av, dt, mag_uT=None)
            else:
                q = self._filt.q.integrate_gyro(gv, dt)
                self._filt.q = q
        else:
            # Robust: stable roll/pitch from IMU-only Madgwick, yaw corrected via mag horizontal reference.
            if accel_ok:
                mode = "6dof"
                q = self._filt.update(gv, av, dt, mag_uT=None)
            else:
                q = self._filt.q.integrate_gyro(gv, dt)
                self._filt.q = q

            if mv is not None and use_mag:
                # Establish reference if missing (e.g., mag was unavailable at init)
                if self._mag_ref_h is None:
                    m_w = q.rotate(mv)
                    mh = _normalize2(np.array([m_w[0], m_w[1]], dtype=float))
                    if mh is not None:
                        self._mag_ref_h = mh

                # Slowly update reference when stationary to track very slow environment changes.
                if self._mag_ref_h is not None and stationary and self._mag_ref_tau > 0.0:
                    m_w = q.rotate(mv)
                    mh = _normalize2(np.array([m_w[0], m_w[1]], dtype=float))
                    if mh is not None:
                        alpha = float(dt / max(1e-3, float(self._mag_ref_tau)))
                        new_ref = _normalize2((1.0 - alpha) * self._mag_ref_h + alpha * mh)
                        if new_ref is not None:
                            self._mag_ref_h = new_ref

                # Apply yaw correction
                if self._mag_ref_h is not None:
                    m_w = q.rotate(mv)
                    mh = _normalize2(np.array([m_w[0], m_w[1]], dtype=float))
                    if mh is not None:
                        cross = float(mh[0] * self._mag_ref_h[1] - mh[1] * self._mag_ref_h[0])
                        dot = float(mh[0] * self._mag_ref_h[0] + mh[1] * self._mag_ref_h[1])
                        yaw_err = math.atan2(cross, dot)
                        yaw_err = max(-self._yaw_max_err, min(self._yaw_max_err, yaw_err))

                        # Down-weight correction when mag is borderline or
                        # vehicle is under dynamic acceleration / rotation
                        # (tilt estimate is unreliable during motion, which
                        # corrupts the tilt-compensated mag projection).
                        motion_w = 1.0
                        if self._yaw_dynamic_gate > 0:
                            motion_w *= max(0.0, 1.0 - self._dynamic_accel_ema / self._yaw_dynamic_gate)
                        if self._yaw_gyro_gate_rad > 0:
                            motion_w *= max(0.0, 1.0 - gyro_norm / self._yaw_gyro_gate_rad)
                        w = float(self._mag_enable) * float(mag_qual) * motion_w
                        k = float((dt / self._yaw_tau) * w)
                        yaw_delta = k * yaw_err

                        if abs(yaw_delta) > 1e-12:
                            q = (Quaternion.from_axis_angle((0.0, 0.0, 1.0), yaw_delta) * q).normalized()
                            self._filt.q = q
                            mode = "yc6"

                        # Learn small Z-gyro bias from sustained yaw error.
                        if self._yaw_ki > 0.0:
                            if (
                                abs(yaw_err) <= self._yaw_bias_adapt_err
                                and abs(float(gv[2])) <= self._yaw_bias_adapt_gyro_rad
                                and gyro_norm <= self._yaw_bias_adapt_gyro_norm
                                and mag_qual >= 0.3
                            ):
                                self._gyro_bias[2] += float(self._yaw_ki) * yaw_err * dt
                                self._gyro_bias[2] = max(-self._yaw_bias_max, min(self._yaw_bias_max, float(self._gyro_bias[2])))

        # Output relative to initial attitude if requested
        q_out = q
        if self._q_zero is not None:
            q_out = (self._q_zero.conj() * q).normalized()

        roll, pitch, yaw = quat_to_euler_deg(q_out)
        yaw = wrap_degrees(yaw)
        roll = wrap_degrees(roll)
        yaw_source = "fused"
        direct_mag_yaw = None

        if self._yaw_mode == "direct_mag" and mv is not None:
            direct_mag_yaw = _direct_mag_yaw_deg(mv, roll, pitch)
            if direct_mag_yaw is not None:
                yaw = direct_mag_yaw
                yaw_source = "direct_mag"

        if self._yaw_zero:
            if self._yaw0 is None and stationary:
                self._yaw0 = yaw
            if self._yaw0 is not None:
                yaw = wrap_degrees(yaw - float(self._yaw0))

        # Output low-pass filter (smooths final Euler angles)
        if self._output_lpf_tau > 0:
            a_out = dt / (dt + self._output_lpf_tau)
            if self._out_roll is None:
                self._out_roll = roll
                self._out_pitch = pitch
                self._out_yaw = yaw
            else:
                self._out_roll += a_out * (roll - self._out_roll)
                self._out_pitch += a_out * (pitch - self._out_pitch)
                # Wrap-safe yaw update: work on the shortest-path delta
                yaw_diff = wrap_degrees(yaw - self._out_yaw)
                self._out_yaw = wrap_degrees(self._out_yaw + a_out * yaw_diff)
            roll, pitch, yaw = self._out_roll, self._out_pitch, self._out_yaw

        out: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": self.name,
            "type": "attitude",
            "rpy_deg": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
            "q": {"w": float(q_out.w), "x": float(q_out.x), "y": float(q_out.y), "z": float(q_out.z)},
            "mag": {"x": float(mv[0]) if mv is not None else 0.0, "y": float(mv[1]) if mv is not None else 0.0, "z": float(mv[2]) if mv is not None else 0.0},
            "mag_source": mag_src,
            "health": {
                "mode": mode,
                "yaw_source": yaw_source,
                "accel_ok": bool(accel_ok),
                "mag_ok": bool(mag_ok),
                "mag_enable": float(self._mag_enable),
                "mag_qual": float(mag_qual),
                "stationary": bool(stationary),
                "gyro_norm": float(gyro_norm),
                "beta": float(beta),
                "yaw_err_deg": float(math.degrees(yaw_err)),
                "yaw_delta_deg": float(math.degrees(yaw_delta)),
                "direct_mag_yaw_deg": (None if direct_mag_yaw is None else float(direct_mag_yaw)),
                "mag_norm_uT": float(mag_norm) if math.isfinite(mag_norm) else None,
                "mag_step_uT": float(mag_step),
                "gyro_bias_rad_s": {"x": float(self._gyro_bias[0]), "y": float(self._gyro_bias[1]), "z": float(self._gyro_bias[2])},
                "accel_sign": float(self._accel_sign_used),
                "mount_auto": bool(self._mount_auto_applied),
                "dynamic_accel": float(self._dynamic_accel_ema),
            },
        }

        # Optional mount debug (small; safe for old pilots to ignore)
        if self._mount_auto_applied and self._mount_auto_R is not None:
            out["mount"] = {
                "R": [[float(x) for x in row] for row in self._mount.R.tolist()],
                "auto": True,
                "auto_yaw_deg": float(self._auto_mount_yaw_deg),
            }

        # Attach fusion debug (safe for older topsides to ignore)
        out.update(mag_meta)

        return out
