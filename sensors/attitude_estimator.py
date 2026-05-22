"""Onboard relative attitude estimator for raw IMU and magnetometer telemetry.

The estimator calibrates against the vehicle's startup/rest pose and publishes
roll, pitch, and relative yaw as derived sensor messages. It is intentionally
diagnostic rather than navigation-grade: the goal is stable operator feedback
and hold-controller input without requiring a fully mapped IMU mounting matrix.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable, Optional


Vec3 = tuple[float, float, float]


def _as_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return None
    return out if math.isfinite(out) else None


def _vec_from_msg(value: Any) -> Optional[Vec3]:
    if not isinstance(value, dict):
        return None
    x = _as_float(value.get("x"))
    y = _as_float(value.get("y"))
    z = _as_float(value.get("z"))
    if x is None or y is None or z is None:
        return None
    return (x, y, z)


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _normalize(a: Vec3) -> Optional[Vec3]:
    n = _norm(a)
    if n <= 1e-9 or not math.isfinite(n):
        return None
    return (a[0] / n, a[1] / n, a[2] / n)


def _mean_vec(samples: Iterable[Vec3]) -> Vec3:
    items = list(samples)
    if not items:
        return (0.0, 0.0, 0.0)
    inv = 1.0 / float(len(items))
    return (
        sum(v[0] for v in items) * inv,
        sum(v[1] for v in items) * inv,
        sum(v[2] for v in items) * inv,
    )


def _rotate_vector(v: Vec3, rot: Vec3) -> Vec3:
    """Rotate vector by a small/exact rotation vector in radians."""
    angle = _norm(rot)
    if angle <= 1e-12:
        return v
    axis = _scale(rot, 1.0 / angle)
    c = math.cos(angle)
    s = math.sin(angle)
    return _add(
        _add(_scale(v, c), _scale(_cross(axis, v), s)),
        _scale(axis, _dot(axis, v) * (1.0 - c)),
    )


def _project_axis(axis: Vec3, normal: Vec3) -> Optional[Vec3]:
    return _normalize(_sub(axis, _scale(normal, _dot(axis, normal))))


def _axis_from_name(name: str) -> Optional[Vec3]:
    text = str(name or "").strip().lower()
    sign = 1.0
    if text.startswith("+"):
        text = text[1:]
    elif text.startswith("-"):
        sign = -1.0
        text = text[1:]
    axes = {
        "x": (1.0, 0.0, 0.0),
        "sensor_x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "sensor_y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
        "sensor_z": (0.0, 0.0, 1.0),
    }
    axis = axes.get(text)
    if axis is None:
        return None
    return _scale(axis, sign)


def _angle_between_unit(a: Vec3, b: Vec3) -> float:
    dot = _clamp(_dot(a, b), -1.0, 1.0)
    return math.acos(dot)


def _wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _slerp_unit(a: Vec3, b: Vec3, amount: float) -> Optional[Vec3]:
    amount = _clamp(amount, 0.0, 1.0)
    angle = _angle_between_unit(a, b)
    if angle <= 1e-9:
        return _normalize(_add(_scale(a, 1.0 - amount), _scale(b, amount)))
    sin_angle = math.sin(angle)
    if abs(sin_angle) <= 1e-9:
        return _normalize(_add(_scale(a, 1.0 - amount), _scale(b, amount)))
    wa = math.sin((1.0 - amount) * angle) / sin_angle
    wb = math.sin(amount * angle) / sin_angle
    return _normalize(_add(_scale(a, wa), _scale(b, wb)))


def _stddev(values: Iterable[float]) -> float:
    items = list(values)
    if len(items) < 2:
        return 0.0
    mean = sum(items) / float(len(items))
    return math.sqrt(sum((v - mean) ** 2 for v in items) / float(len(items)))


def _rotate_between_unit(v: Vec3, src: Vec3, dst: Vec3) -> Vec3:
    """Rotate v by the smallest rotation that maps unit vector src to dst."""
    dot = _clamp(_dot(src, dst), -1.0, 1.0)
    if dot > 1.0 - 1e-9:
        return v
    if dot < -1.0 + 1e-9:
        axis = _project_axis((1.0, 0.0, 0.0), src)
        if axis is None:
            axis = _project_axis((0.0, 1.0, 0.0), src)
        if axis is None:
            axis = (0.0, 0.0, 1.0)
        return _rotate_vector(v, _scale(axis, math.pi))
    axis_raw = _cross(src, dst)
    axis = _normalize(axis_raw)
    if axis is None:
        return v
    return _rotate_vector(v, _scale(axis, math.acos(dot)))


@dataclass(frozen=True)
class RollPitchConfig:
    """Tuning parameters for startup calibration and IMU/mag fusion."""

    calibration_samples: int = 30
    max_dt_s: float = 0.25
    accel_correction: float | None = None
    accel_tau_s: float = 0.16
    accel_fast_tau_s: float = 0.055
    accel_fast_error_deg: float = 3.0
    accel_min_weight: float = 0.02
    accel_max_weight: float = 0.90
    accel_norm_gate: float = 0.18
    calibration_max_tilt_std_deg: float = 1.25
    calibration_max_gyro_rms_dps: float = 3.0
    vehicle_roll_axis: str = "x"
    roll_sign: float = 1.0
    pitch_sign: float = 1.0
    yaw_mag_source: str = "auto"
    yaw_tau_s: float = 0.45
    yaw_min_weight: float = 0.02
    yaw_max_weight: float = 0.65
    yaw_max_mag_age_s: float = 0.75
    yaw_mag_norm_gate: float = 0.45
    yaw_mag_smooth_tau_s: float = 0.65
    yaw_reference_samples: int = 30
    yaw_min_horizontal_norm: float = 1e-6
    stationary_bias_enable: bool = True
    stationary_bias_tau_s: float = 15.0
    stationary_gyro_max_dps: float = 1.0
    stationary_accel_error_max_deg: float = 1.5
    stationary_accel_norm_error_max: float = 0.05


class RollPitchEstimator:
    """Rest-relative roll/pitch/yaw diagnostic estimator.

    The estimator deliberately treats the current calibration pose as zero.
    That lets us produce useful roll/pitch telemetry before we know the exact
    IMU-to-vehicle mount transform. Yaw is relative to the magnetometer reading
    at that rest pose, not an absolute compass heading.
    """

    def __init__(self, config: RollPitchConfig | None = None):
        self.config = config or RollPitchConfig()
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        """Clear calibration, filtered attitude, yaw, and last-output state."""

        with self._lock:
            self._cal_accel_units: list[Vec3] = []
            self._cal_accel_raw: list[Vec3] = []
            self._cal_gyro: list[Vec3] = []
            self._cal_mag_samples: dict[str, list[Vec3]] = {}
            self._reference_accel: Optional[Vec3] = None
            self._reference_norm: Optional[float] = None
            self._gyro_bias: Vec3 = (0.0, 0.0, 0.0)
            self._roll_axis: Optional[Vec3] = None
            self._pitch_axis: Optional[Vec3] = None
            self._calibration_tilt_std_deg: Optional[float] = None
            self._calibration_gyro_rms_dps: Optional[float] = None
            self._gravity_est: Optional[Vec3] = None
            self._latest_mag: dict[str, tuple[float, Vec3, float]] = {}
            self._mag_filtered: dict[str, tuple[float, Vec3, float]] = {}
            self._latest_primary_mag_source: Optional[str] = None
            self._reference_mag: dict[str, Vec3] = {}
            self._reference_mag_norm: dict[str, float] = {}
            self._reference_mag_samples: dict[str, int] = {}
            self._yaw_est_rad: Optional[float] = None
            self._last_yaw_mag_correction: tuple[str, float] | None = None
            self._last_ts: Optional[float] = None
            self._last_output: Optional[dict[str, Any]] = None

    def status(self) -> dict[str, Any]:
        """Return calibration and latest-estimate diagnostics."""

        with self._lock:
            return {
                "calibration_state": "calibrated" if self._reference_accel is not None else "calibrating",
                "calibration_samples": len(self._cal_accel_units),
                "calibration_target_samples": int(self.config.calibration_samples),
                "reference_accel": self._reference_accel,
                "reference_norm": self._reference_norm,
                "gyro_bias": self._gyro_bias,
                "roll_axis": self._roll_axis,
                "pitch_axis": self._pitch_axis,
                "calibration_tilt_std_deg": self._calibration_tilt_std_deg,
                "calibration_gyro_rms_dps": self._calibration_gyro_rms_dps,
                "yaw_state": "ready" if self._yaw_est_rad is not None else "waiting_for_mag",
                "yaw_sources": sorted(self._reference_mag.keys()),
                "yaw_reference_samples": dict(self._reference_mag_samples),
                "latest_mag_sources": sorted(self._latest_mag.keys()),
                "last_output": dict(self._last_output) if self._last_output else None,
                "vehicle_roll_axis": str(self.config.vehicle_roll_axis),
                "roll_sign": float(self.config.roll_sign),
                "pitch_sign": float(self.config.pitch_sign),
            }

    def update_mag(self, mag_msg: dict[str, Any]) -> None:
        """Update the latest magnetometer samples used for relative yaw."""
        sensor_ts = _as_float((mag_msg or {}).get("ts"))
        if sensor_ts is None:
            sensor_ts = time.time()
        samples: dict[str, Vec3] = {}
        mag_sources = (mag_msg or {}).get("mag_sources")
        if isinstance(mag_sources, dict):
            for source, value in mag_sources.items():
                vec = _vec_from_msg(value)
                if vec is not None:
                    samples[str(source)] = vec
        primary = _vec_from_msg((mag_msg or {}).get("mag") or (mag_msg or {}).get("magnetometer"))
        primary_source = str((mag_msg or {}).get("mag_source") or "").strip()
        if primary is not None:
            if primary_source:
                samples.setdefault(primary_source, primary)
            samples["primary"] = primary

        if not samples:
            return
        with self._lock:
            if primary_source:
                self._latest_primary_mag_source = primary_source
            for source, vec in samples.items():
                if self._reference_accel is None:
                    self._record_cal_mag_sample_locked(source, vec)
                filtered, filtered_norm = self._smooth_mag_locked(source, float(sensor_ts), vec)
                self._latest_mag[source] = (float(sensor_ts), filtered, filtered_norm)
            self._seed_yaw_references_locked()

    def update(self, imu_msg: dict[str, Any], *, recv_time_s: float | None = None) -> Optional[dict[str, Any]]:
        """Consume one IMU message and return a derived attitude message if ready."""

        accel = _vec_from_msg((imu_msg or {}).get("accel"))
        gyro = _vec_from_msg((imu_msg or {}).get("gyro"))
        if accel is None or gyro is None:
            return None
        accel_norm = _norm(accel)
        accel_unit = _normalize(accel)
        if accel_unit is None or accel_norm < 1.0:
            return None
        sensor_ts = _as_float((imu_msg or {}).get("ts"))
        if sensor_ts is None:
            sensor_ts = time.time()
        if recv_time_s is None:
            recv_time_s = time.time()

        with self._lock:
            if self._reference_accel is None:
                self._cal_accel_units.append(accel_unit)
                self._cal_accel_raw.append(accel)
                self._cal_gyro.append(gyro)
                if len(self._cal_accel_units) >= int(self.config.calibration_samples):
                    if not self._finish_calibration(sensor_ts):
                        self._cal_accel_units.pop(0)
                        self._cal_accel_raw.pop(0)
                        self._cal_gyro.pop(0)
                return None

            dt = 0.0
            if self._last_ts is not None:
                dt = max(0.0, min(float(self.config.max_dt_s), sensor_ts - self._last_ts))
            self._last_ts = sensor_ts

            gyro_unbiased = _sub(gyro, self._gyro_bias)
            yaw_rate_dps = 0.0
            if self._gravity_est is None:
                self._gravity_est = self._reference_accel

            predicted = self._gravity_est
            if dt > 0.0:
                predicted = _normalize(_rotate_vector(predicted, _scale(gyro_unbiased, -dt))) or predicted
                if self._yaw_est_rad is not None:
                    yaw_rate_rad_s = _dot(gyro_unbiased, predicted)
                    yaw_rate_dps = math.degrees(yaw_rate_rad_s)
                    self._yaw_est_rad = _wrap_pi(self._yaw_est_rad + yaw_rate_rad_s * dt)

            accel_error_rad = _angle_between_unit(predicted, accel_unit)
            accel_error_deg = math.degrees(accel_error_rad)
            accel_weight = self._accel_weight(dt, accel_error_deg)
            norm_err = 0.0
            if self._reference_norm and self._reference_norm > 0:
                norm_err = abs(accel_norm - self._reference_norm) / self._reference_norm
                if norm_err > float(self.config.accel_norm_gate):
                    accel_weight = 0.0
                elif norm_err > float(self.config.accel_norm_gate) * 0.5:
                    accel_weight *= 0.35
            gyro_bias_alpha = self._adapt_gyro_bias(dt, gyro, gyro_unbiased, accel_error_deg, norm_err)

            corrected = _slerp_unit(predicted, accel_unit, accel_weight)
            self._gravity_est = corrected or predicted

            filtered = self._tilt_components(self._gravity_est)
            accel_only = self._tilt_components(accel_unit)
            if filtered is None or accel_only is None:
                return None

            yaw_info = self._update_yaw_locked(sensor_ts, self._gravity_est)
            roll_deg, pitch_deg, tilt_deg = filtered
            accel_roll_deg, accel_pitch_deg, accel_tilt_deg = accel_only
            out = {
                "ts": sensor_ts,
                "sensor": "roll_pitch_estimator",
                "type": "attitude",
                "source": "onboard_imu_mag_relative" if yaw_info.get("yaw_deg") is not None else "onboard_imu_6axis",
                "recv_time_s": float(recv_time_s),
                "sample_age_s": max(0.0, float(recv_time_s) - float(sensor_ts)),
                "roll_deg": roll_deg,
                "pitch_deg": pitch_deg,
                "tilt_deg": tilt_deg,
                "roll_pitch_ready": True,
                "attitude_ready": True,
                "vehicle_roll_axis": str(self.config.vehicle_roll_axis),
                "roll_sign": float(self.config.roll_sign),
                "pitch_sign": float(self.config.pitch_sign),
                "attitude_axes": {
                    "roll": {"x": self._roll_axis[0], "y": self._roll_axis[1], "z": self._roll_axis[2]},
                    "pitch": {"x": self._pitch_axis[0], "y": self._pitch_axis[1], "z": self._pitch_axis[2]},
                },
                "accel_roll_deg": accel_roll_deg,
                "accel_pitch_deg": accel_pitch_deg,
                "accel_tilt_deg": accel_tilt_deg,
                "accel_norm": accel_norm,
                "accel_weight": accel_weight,
                "accel_error_deg": accel_error_deg,
                "accel_norm_error": norm_err,
                "gyro_rate_dps": math.degrees(_norm(gyro_unbiased)),
                "gyro_bias_alpha": gyro_bias_alpha,
                "yaw_rate_dps": yaw_rate_dps,
                "gravity": {
                    "x": self._gravity_est[0],
                    "y": self._gravity_est[1],
                    "z": self._gravity_est[2],
                },
                "reference_accel": {
                    "x": self._reference_accel[0],
                    "y": self._reference_accel[1],
                    "z": self._reference_accel[2],
                    "norm": self._reference_norm,
                },
                "gyro_bias": {
                    "x": self._gyro_bias[0],
                    "y": self._gyro_bias[1],
                    "z": self._gyro_bias[2],
                },
                "gyro_unbiased": {
                    "x": gyro_unbiased[0],
                    "y": gyro_unbiased[1],
                    "z": gyro_unbiased[2],
                },
                "calibration_state": "calibrated",
                "calibration_samples": len(self._cal_accel_units),
                "calibration_tilt_std_deg": self._calibration_tilt_std_deg,
                "calibration_gyro_rms_dps": self._calibration_gyro_rms_dps,
            }
            out.update(yaw_info)
            out["yaw_ready"] = out.get("yaw_deg") is not None
            out["mag_ready"] = out.get("yaw_status") == "ready"
            self._last_output = out
            return dict(out)

    def _accel_weight(self, dt: float, accel_error_deg: float) -> float:
        fixed = self.config.accel_correction
        if fixed is not None:
            return _clamp(float(fixed), 0.0, 1.0)
        if dt <= 0.0:
            return _clamp(float(self.config.accel_max_weight), 0.0, 1.0)
        tau = float(self.config.accel_tau_s)
        if accel_error_deg >= float(self.config.accel_fast_error_deg):
            tau = min(tau, float(self.config.accel_fast_tau_s))
        tau = max(1e-3, tau)
        weight = 1.0 - math.exp(-dt / tau)
        return _clamp(weight, float(self.config.accel_min_weight), float(self.config.accel_max_weight))

    def _adapt_gyro_bias(
        self,
        dt: float,
        gyro_raw: Vec3,
        gyro_unbiased: Vec3,
        accel_error_deg: float,
        accel_norm_error: float,
    ) -> float:
        if not bool(self.config.stationary_bias_enable) or dt <= 0.0:
            return 0.0
        if accel_error_deg > float(self.config.stationary_accel_error_max_deg):
            return 0.0
        if accel_norm_error > float(self.config.stationary_accel_norm_error_max):
            return 0.0
        if math.degrees(_norm(gyro_unbiased)) > float(self.config.stationary_gyro_max_dps):
            return 0.0
        tau = max(1e-3, float(self.config.stationary_bias_tau_s))
        alpha = _clamp(1.0 - math.exp(-dt / tau), 0.0, 0.05)
        if alpha <= 0.0:
            return 0.0
        self._gyro_bias = _add(_scale(self._gyro_bias, 1.0 - alpha), _scale(gyro_raw, alpha))
        return alpha

    def _finish_calibration(self, sensor_ts: float) -> bool:
        ref = _normalize(_mean_vec(self._cal_accel_units))
        if ref is None:
            self._cal_accel_units.clear()
            self._cal_accel_raw.clear()
            self._cal_gyro.clear()
            return False

        tilt_errors = [math.degrees(_angle_between_unit(ref, sample)) for sample in self._cal_accel_units]
        tilt_std = _stddev(tilt_errors)
        gyro_mean = _mean_vec(self._cal_gyro)
        gyro_rms_dps = math.degrees(
            math.sqrt(
                sum(_dot(_sub(g, gyro_mean), _sub(g, gyro_mean)) for g in self._cal_gyro)
                / max(1, len(self._cal_gyro))
            )
        )
        if (
            tilt_std > float(self.config.calibration_max_tilt_std_deg)
            or gyro_rms_dps > float(self.config.calibration_max_gyro_rms_dps)
        ):
            return False

        raw_mean = _mean_vec(self._cal_accel_raw)
        self._reference_accel = ref
        self._reference_norm = _norm(raw_mean)
        self._gyro_bias = gyro_mean
        self._calibration_tilt_std_deg = tilt_std
        self._calibration_gyro_rms_dps = gyro_rms_dps

        roll_axis = None
        configured_roll_axis = _axis_from_name(str(self.config.vehicle_roll_axis))
        if configured_roll_axis is not None:
            roll_axis = _project_axis(configured_roll_axis, ref)
        if roll_axis is None:
            roll_axis = _project_axis((1.0, 0.0, 0.0), ref)
        if roll_axis is None:
            roll_axis = _project_axis((0.0, 1.0, 0.0), ref)
        if roll_axis is None:
            roll_axis = _project_axis((0.0, 0.0, 1.0), ref)
        if roll_axis is None:
            roll_axis = (1.0, 0.0, 0.0)
        pitch_axis = _normalize(_cross(ref, roll_axis)) or (0.0, 1.0, 0.0)

        self._roll_axis = roll_axis
        self._pitch_axis = pitch_axis
        self._gravity_est = ref
        self._last_ts = sensor_ts
        self._seed_yaw_references_locked()
        return True

    def _record_cal_mag_sample_locked(self, source: str, mag: Vec3) -> None:
        cap = max(1, int(self.config.yaw_reference_samples))
        samples = self._cal_mag_samples.setdefault(source, [])
        samples.append(mag)
        if len(samples) > cap:
            del samples[: len(samples) - cap]

    def _smooth_mag_locked(self, source: str, sensor_ts: float, mag: Vec3) -> tuple[Vec3, float]:
        tau = max(0.0, float(self.config.yaw_mag_smooth_tau_s))
        previous = self._mag_filtered.get(source)
        if previous is None or tau <= 1e-6:
            filtered = mag
        else:
            prev_ts, prev_mag, _prev_norm = previous
            dt = float(sensor_ts) - float(prev_ts)
            if dt > 0.0:
                alpha = 1.0 - math.exp(-dt / max(1e-6, tau))
            else:
                alpha = 0.05
            alpha = _clamp(alpha, 0.0, 1.0)
            filtered = _add(_scale(prev_mag, 1.0 - alpha), _scale(mag, alpha))
        filtered_norm = _norm(filtered)
        self._mag_filtered[source] = (float(sensor_ts), filtered, filtered_norm)
        return filtered, filtered_norm

    def _seed_yaw_references_locked(self) -> None:
        if self._reference_accel is None:
            return
        for source, (_ts, mag, mag_norm) in self._latest_mag.items():
            if source in self._reference_mag:
                continue
            ref_mag = mag
            ref_norm = mag_norm
            ref_sample_count = 1
            cal_samples = self._cal_mag_samples.get(source)
            if cal_samples:
                ref_mag = _mean_vec(cal_samples)
                ref_norm = _norm(ref_mag)
                ref_sample_count = len(cal_samples)
            horizontal = self._level_mag(ref_mag, self._reference_accel)
            if horizontal is None:
                continue
            self._reference_mag[source] = horizontal
            self._reference_mag_norm[source] = ref_norm
            self._reference_mag_samples[source] = ref_sample_count

    def _level_mag(self, mag: Vec3, gravity: Vec3) -> Optional[Vec3]:
        if self._reference_accel is None:
            return None
        gravity_unit = _normalize(gravity)
        if gravity_unit is None:
            return None
        leveled = _rotate_between_unit(mag, gravity_unit, self._reference_accel)
        horizontal = _sub(leveled, _scale(self._reference_accel, _dot(leveled, self._reference_accel)))
        if _norm(horizontal) <= float(self.config.yaw_min_horizontal_norm):
            return None
        return _normalize(horizontal)

    def _select_yaw_source_locked(self, sensor_ts: float | None = None) -> Optional[str]:
        if not self._reference_mag or not self._latest_mag:
            return None
        requested = str(self.config.yaw_mag_source or "auto").strip().lower()
        if requested and requested != "auto":
            for source in self._latest_mag:
                if source.lower() == requested and source in self._reference_mag:
                    return source
            return None

        candidates = ["mmc5983", "ak09915"]
        if self._latest_primary_mag_source:
            candidates.append(self._latest_primary_mag_source)
        candidates.append("primary")
        candidates.extend(sorted(self._latest_mag.keys()))
        candidates = list(dict.fromkeys(candidates))

        def usable(source: str) -> bool:
            if source in self._latest_mag and source in self._reference_mag:
                return True
            return False

        def fresh(source: str) -> bool:
            if sensor_ts is None:
                return True
            mag_ts, _mag, _mag_norm = self._latest_mag[source]
            return max(0.0, float(sensor_ts) - float(mag_ts)) <= float(self.config.yaw_max_mag_age_s)

        def clean_norm(source: str) -> bool:
            ref_norm = self._reference_mag_norm.get(source)
            if not ref_norm or ref_norm <= 0.0:
                return True
            _mag_ts, _mag, mag_norm = self._latest_mag[source]
            norm_error = abs(float(mag_norm) - float(ref_norm)) / float(ref_norm)
            return norm_error <= float(self.config.yaw_mag_norm_gate)

        if sensor_ts is not None:
            for source in candidates:
                if usable(source) and fresh(source) and clean_norm(source):
                    return source

            for source in candidates:
                if usable(source) and fresh(source):
                    return source

        for source in candidates:
            if usable(source) and clean_norm(source):
                return source
        for source in candidates:
            if usable(source):
                return source
        return None

    def _mag_yaw_locked(self, source: str, gravity: Vec3) -> Optional[dict[str, Any]]:
        if self._reference_accel is None or source not in self._latest_mag or source not in self._reference_mag:
            return None
        mag_ts, mag, mag_norm = self._latest_mag[source]
        current_h = self._level_mag(mag, gravity)
        reference_h = self._reference_mag.get(source)
        if current_h is None or reference_h is None:
            return None
        signed = math.atan2(
            _dot(self._reference_accel, _cross(reference_h, current_h)),
            _clamp(_dot(reference_h, current_h), -1.0, 1.0),
        )
        # A positive vehicle yaw rotates the world magnetic vector in the
        # opposite direction in body coordinates, so invert the relative angle.
        yaw_mag = _wrap_pi(-signed)
        ref_norm = self._reference_mag_norm.get(source)
        norm_err = 0.0
        if ref_norm and ref_norm > 0:
            norm_err = abs(mag_norm - ref_norm) / ref_norm
        return {
            "source": source,
            "mag_ts": mag_ts,
            "yaw_mag_rad": yaw_mag,
            "mag_norm": mag_norm,
            "mag_norm_error": norm_err,
            "reference_mag_samples": self._reference_mag_samples.get(source, 0),
            "reference_mag": reference_h,
            "current_mag": current_h,
        }

    def _update_yaw_locked(self, sensor_ts: float, gravity: Vec3) -> dict[str, Any]:
        self._seed_yaw_references_locked()
        source = self._select_yaw_source_locked(sensor_ts)
        out: dict[str, Any] = {
            "yaw_status": "waiting_for_mag",
            "yaw_source": source,
        }
        if source is None:
            if self._yaw_est_rad is not None:
                out["yaw_deg"] = math.degrees(self._yaw_est_rad)
                out["yaw_status"] = "gyro_only"
            return out

        mag_info = self._mag_yaw_locked(source, gravity)
        if mag_info is None:
            if self._yaw_est_rad is not None:
                out["yaw_deg"] = math.degrees(self._yaw_est_rad)
                out["yaw_status"] = "gyro_only"
            return out

        mag_age_s = max(0.0, float(sensor_ts) - float(mag_info["mag_ts"]))
        yaw_mag = float(mag_info["yaw_mag_rad"])
        yaw_weight = 0.0
        yaw_status = "mag_stale" if mag_age_s > float(self.config.yaw_max_mag_age_s) else "ready"
        if yaw_status == "ready" and float(mag_info["mag_norm_error"]) <= float(self.config.yaw_mag_norm_gate):
            if self._yaw_est_rad is None:
                self._yaw_est_rad = yaw_mag
                yaw_weight = 1.0
            elif self._last_yaw_mag_correction != (source, float(mag_info["mag_ts"])):
                tau = max(1e-3, float(self.config.yaw_tau_s))
                yaw_weight = 1.0 - math.exp(-max(0.0, mag_age_s) / tau)
                yaw_weight = _clamp(yaw_weight, float(self.config.yaw_min_weight), float(self.config.yaw_max_weight))
                error = _wrap_pi(yaw_mag - self._yaw_est_rad)
                self._yaw_est_rad = _wrap_pi(self._yaw_est_rad + yaw_weight * error)
            self._last_yaw_mag_correction = (source, float(mag_info["mag_ts"]))
        elif yaw_status == "ready":
            yaw_status = "mag_norm_gated"

        if self._yaw_est_rad is not None:
            out["yaw_deg"] = math.degrees(self._yaw_est_rad)
        out.update(
            {
                "yaw_mag_deg": math.degrees(yaw_mag),
                "yaw_weight": yaw_weight,
                "yaw_status": yaw_status,
                "yaw_source": source,
                "yaw_mag_age_s": mag_age_s,
                "yaw_mag_norm": mag_info["mag_norm"],
                "yaw_mag_norm_error": mag_info["mag_norm_error"],
                "yaw_reference_mag_samples": mag_info["reference_mag_samples"],
                "reference_mag": {
                    "x": mag_info["reference_mag"][0],
                    "y": mag_info["reference_mag"][1],
                    "z": mag_info["reference_mag"][2],
                },
                "leveled_mag": {
                    "x": mag_info["current_mag"][0],
                    "y": mag_info["current_mag"][1],
                    "z": mag_info["current_mag"][2],
                },
            }
        )
        return out

    def _tilt_components(self, gravity: Vec3) -> Optional[tuple[float, float, float]]:
        if self._reference_accel is None or self._roll_axis is None or self._pitch_axis is None:
            return None
        gx = _dot(gravity, self._roll_axis)
        gy = _dot(gravity, self._pitch_axis)
        gz = _clamp(_dot(gravity, self._reference_accel), -1.0, 1.0)
        roll = math.atan2(-gy, gz) * float(self.config.roll_sign)
        pitch = math.atan2(gx, math.sqrt(max(0.0, gy * gy + gz * gz))) * float(self.config.pitch_sign)
        tilt = math.atan2(math.sqrt(max(0.0, gx * gx + gy * gy)), gz)
        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(tilt),
        )


class AttitudeEstimatorProcessor:
    """Derive onboard attitude messages from the raw sensor publisher stream."""

    def __init__(self, estimator: RollPitchEstimator | None = None):
        self.estimator = estimator or RollPitchEstimator()

    def process(self, msg: dict[str, Any]) -> list[dict[str, Any]]:
        """Process one raw sensor-stream message and emit derived telemetry."""

        typ = str((msg or {}).get("type", ""))
        if typ == "mag":
            self.estimator.update_mag(dict(msg or {}))
            return []
        if typ != "imu":
            return []
        out = self.estimator.update(dict(msg or {}), recv_time_s=time.time())
        return [out] if isinstance(out, dict) else []
