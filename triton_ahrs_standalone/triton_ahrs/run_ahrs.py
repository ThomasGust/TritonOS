from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .calibration import GyroCalibration, MagCalibration, Mount, load_json, save_json
from .madgwick import MadgwickAHRS, MadgwickConfig
from .navigator import NavigatorIMU
from .quaternion import quat_to_euler_deg, wrap_degrees, Quaternion

G = 9.80665


UP_W = np.array([0.0, 0.0, 1.0], dtype=float)

def _normalize3(v: np.ndarray) -> Optional[np.ndarray]:
    n = float(np.linalg.norm(v))
    if not math.isfinite(n) or n <= 1e-12:
        return None
    return v / n

class LowPass3:
    """Simple 3D first-order low-pass filter (exponential smoothing) with time constant tau."""

    def __init__(self, tau_s: float):
        self.tau = float(max(0.0, tau_s))
        self.x: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.x = None

    def update(self, v: np.ndarray, dt: float) -> np.ndarray:
        v = np.asarray(v, dtype=float).reshape(3)
        if self.tau <= 0.0 or not math.isfinite(self.tau):
            self.x = v.copy()
            return v
        if self.x is None:
            self.x = v.copy()
            return v
        alpha = float(dt / (self.tau + dt))
        self.x = (1.0 - alpha) * self.x + alpha * v
        return self.x

def _quat_from_two_unit_vectors(a: np.ndarray, b: np.ndarray) -> Quaternion:
    """Return q such that q.rotate(a) == b (approximately), for unit a,b."""
    a = np.asarray(a, dtype=float).reshape(3)
    b = np.asarray(b, dtype=float).reshape(3)
    c = float(np.dot(a, b))
    v = np.cross(a, b)
    vn = float(np.linalg.norm(v))
    if vn <= 1e-12:
        if c > 0.0:
            return Quaternion.identity()
        axis = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0], dtype=float)
        axis = np.cross(a, axis)
        axis = axis / float(np.linalg.norm(axis))
        return Quaternion(0.0, float(axis[0]), float(axis[1]), float(axis[2])).normalized()
    q = Quaternion(1.0 + c, float(v[0]), float(v[1]), float(v[2])).normalized()
    return q

def initial_quaternion_from_accel_mag(accel_m_s2: np.ndarray, mag_uT: Optional[np.ndarray]) -> Quaternion:
    """Compute an initial BODY->WORLD quaternion from averaged accel (+optional mag)."""
    a = _normalize3(accel_m_s2)
    if a is None:
        return Quaternion.identity()

    z_b = a  # body 'up' direction expressed in body coordinates
    if mag_uT is None:
        return _quat_from_two_unit_vectors(z_b, UP_W)

    m = _normalize3(mag_uT)
    if m is None:
        return _quat_from_two_unit_vectors(z_b, UP_W)

    mh = m - z_b * float(np.dot(m, z_b))
    mh = _normalize3(mh)
    if mh is None:
        return _quat_from_two_unit_vectors(z_b, UP_W)

    x_b = mh
    y_b = np.cross(z_b, x_b)
    y_b = _normalize3(y_b)
    if y_b is None:
        return _quat_from_two_unit_vectors(z_b, UP_W)

    x_b = np.cross(y_b, z_b)
    x_b = _normalize3(x_b)
    if x_b is None:
        return _quat_from_two_unit_vectors(z_b, UP_W)

    B = np.stack([x_b, y_b, z_b], axis=1)
    R = B.T
    return Quaternion.from_rotation_matrix(R)


def _now() -> float:
    return time.time()


def _perf() -> float:
    return time.perf_counter()


def calibrate_gyro_bias(board: NavigatorIMU, seconds: float, rate_hz: float, mount: Mount) -> GyroCalibration:
    """Estimate gyro bias (rad/s) from a stationary window."""
    n = max(10, int(seconds * rate_hz))
    dt = 1.0 / rate_hz

    acc = np.zeros(3, dtype=float)
    t_end = _perf() + seconds
    k = 0
    while _perf() < t_end:
        g = board.read_gyro()
        gv = mount.apply((g.x, g.y, g.z))
        acc += gv
        k += 1
        time.sleep(dt)
    if k <= 0:
        raise RuntimeError("gyro calibration failed: no samples")
    bias = acc / float(k)
    return GyroCalibration(bias_rad_s=bias)


def _mag_health(
    mag_uT: np.ndarray,
    baseline_uT: Optional[float],
    tol_ratio: float,
    max_step_uT: float,
    prev_mag_norm_uT: Optional[float],
) -> tuple[bool, float, float]:
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

    if step > max_step_uT:
        return (False, mag_norm, step)

    return (True, mag_norm, step)


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Standalone AHRS for RPi + Navigator IMU")

    # Loop / output
    p.add_argument("--rate", type=float, default=200.0, help="AHRS update rate (Hz)")
    p.add_argument("--print-rate", type=float, default=20.0, help="Console print rate (Hz)")
    p.add_argument("--log-csv", type=str, default="", help="Write CSV log to this path")
    p.add_argument("--json", action="store_true", help="Emit JSON lines to stdout instead of human text")

    # Filter
    p.add_argument("--beta", type=float, default=0.08, help="Madgwick beta gain")

    # Startup convergence / stability
    p.add_argument("--init-seconds", type=float, default=0.8, help="Seconds of samples to compute an initial attitude seed")
    p.add_argument("--warmup-seconds", type=float, default=1.5, help="Seconds to use a higher beta for fast convergence")
    p.add_argument("--beta-init", type=float, default=0.6, help="Madgwick beta during warmup (higher = faster lock)")
    p.add_argument("--beta-stationary", type=float, default=0.12, help="Madgwick beta when stationary (helps keep lock)")

    # Simple low-pass filtering (sensor noise reduction)
    p.add_argument("--accel-lpf-tau", type=float, default=0.05, help="Accel low-pass tau seconds (0 disables)")
    p.add_argument("--mag-lpf-tau", type=float, default=0.20, help="Mag low-pass tau seconds (0 disables)")
    p.add_argument("--gyro-lpf-tau", type=float, default=0.00, help="Gyro low-pass tau seconds (0 disables)")

    # Stationary detection / bias refinement
    p.add_argument("--stationary-gyro-rad", type=float, default=0.03, help="Gyro norm (rad/s) below which we consider the system stationary")
    p.add_argument("--bias-adapt-tau", type=float, default=60.0, help="Seconds time constant for in-run gyro bias adaptation when stationary (0 disables)")

    # Orientation zeroing / accel sign
    p.add_argument("--zero-attitude", action="store_true", help="Zero roll/pitch/yaw at startup (relative attitude output)")
    p.add_argument("--accel-sign", choices=["auto", "normal", "invert"], default="auto", help="How to treat accel direction at rest (auto recommended)")

    # Calibration files
    p.add_argument("--gyro-cal", type=str, default="", help="Path to gyro calibration JSON")
    p.add_argument("--mag-cal", type=str, default="", help="Path to mag calibration JSON")
    p.add_argument("--mount", type=str, default="", help="Path to mount (axis mapping) JSON")

    # Auto calibration
    p.add_argument("--auto-gyro-cal", action="store_true", help="Auto-calibrate gyro bias at startup (recommended)")
    p.add_argument("--gyro-cal-seconds", type=float, default=3.0, help="Stationary seconds for gyro bias")
    p.add_argument("--save-gyro-cal", type=str, default="", help="Save estimated gyro cal JSON to this path")

    # Health gating
    p.add_argument("--accel-g-tol", type=float, default=0.20, help="Accel magnitude tolerance (fraction of 1g)")
    p.add_argument("--mag-tol", type=float, default=0.35, help="Mag magnitude tolerance ratio around baseline")
    p.add_argument("--mag-max-step", type=float, default=8.0, help="Max per-sample |B| step in uT before marking unhealthy")
    p.add_argument("--mag-baseline-seconds", type=float, default=2.0, help="Seconds to estimate baseline |B| at startup")

    # Yaw presentation
    p.add_argument("--yaw-zero", action="store_true", help="Zero yaw at startup (operator-friendly)")

    # Sensor options
    p.add_argument("--imu-i2c-bus", type=int, default=1)
    p.add_argument("--ak-i2c-bus", type=int, default=1)
    p.add_argument("--prefer-spi", action="store_true", help="Prefer SPI for ICM20602 auto-detect")
    p.add_argument("--disable-mmc5983", action="store_true", help="Do not try to use MMC5983")

    args = p.parse_args(argv)

    rate_hz = float(args.rate)
    dt_target = 1.0 / max(10.0, rate_hz)

    mount = Mount.identity()
    if args.mount:
        mount = Mount.from_dict(load_json(args.mount))

    gyro_cal: Optional[GyroCalibration] = None
    if args.gyro_cal:
        gyro_cal = GyroCalibration.from_dict(load_json(args.gyro_cal))

    mag_cal: Optional[MagCalibration] = None
    if args.mag_cal:
        mag_cal = MagCalibration.from_dict(load_json(args.mag_cal))

    board = NavigatorIMU(
        imu_i2c_bus=int(args.imu_i2c_bus),
        prefer_spi=bool(args.prefer_spi),
        ak_i2c_bus=int(args.ak_i2c_bus),
        enable_mmc5983=not bool(args.disable_mmc5983),
    )

    try:
        # Auto gyro cal if requested or if no file was provided
        if args.auto_gyro_cal or gyro_cal is None:
            if not args.json:
                print(f"[ahrs] Gyro bias calibration: hold still for {args.gyro_cal_seconds:.1f}s ...")
            gyro_cal = calibrate_gyro_bias(board, args.gyro_cal_seconds, rate_hz=min(rate_hz, 200.0), mount=mount)
            if not args.json:
                b = gyro_cal.bias_rad_s
                print(f"[ahrs] Gyro bias rad/s: x={b[0]:+.6f} y={b[1]:+.6f} z={b[2]:+.6f}")
            if args.save_gyro_cal:
                save_json(gyro_cal.to_dict(), args.save_gyro_cal)
                if not args.json:
                    print(f"[ahrs] Saved gyro cal -> {args.save_gyro_cal}")

        # Estimate baseline mag magnitude for gating (after applying mag cal + mount)
        baseline_uT: Optional[float] = None
        if args.mag_baseline_seconds > 0:
            mags = []
            t_end = _perf() + float(args.mag_baseline_seconds)
            while _perf() < t_end:
                _src, m = board.read_mag()
                mv = mount.apply((m.x, m.y, m.z))
                if mag_cal is not None:
                    mv = mag_cal.apply(mv)
                mags.append(float(np.linalg.norm(mv)))
                time.sleep(0.01)
            if mags:
                baseline_uT = float(np.median(np.array(mags)))

        # Low-pass filters (reduces sensor noise / jitter at rest)
        accel_lpf = LowPass3(float(args.accel_lpf_tau))
        mag_lpf = LowPass3(float(args.mag_lpf_tau))
        gyro_lpf = LowPass3(float(args.gyro_lpf_tau))

        # Mutable gyro bias vector (can be refined online when stationary)
        gyro_bias = gyro_cal.bias_rad_s.copy() if gyro_cal is not None else np.zeros(3, dtype=float)

        # Initial attitude seed from averaged accel (+mag if healthy).
        # This removes the long "settling" time you saw from starting at identity.
        accel_sign = 1.0
        if float(args.init_seconds) > 0.0:
            accs = []
            mags_seed = []
            t_end_seed = _perf() + float(args.init_seconds)
            while _perf() < t_end_seed:
                a0 = board.read_accel()
                _src0, m0 = board.read_mag()
                av0 = np.asarray(mount.apply((a0.x, a0.y, a0.z)), dtype=float)
                mv0 = np.asarray(mount.apply((m0.x, m0.y, m0.z)), dtype=float)
                if mag_cal is not None:
                    mv0 = mag_cal.apply(mv0)
                accs.append(av0)
                mags_seed.append(mv0)
                time.sleep(0.005)

            a_avg = np.median(np.stack(accs, axis=0), axis=0) if accs else np.array([0.0, 0.0, G], dtype=float)
            m_avg = np.median(np.stack(mags_seed, axis=0), axis=0) if mags_seed else None

            # Auto accel sign: if the board is "upright" but accel points strongly downward,
            # invert it so +Z_world corresponds to "up" and level reads near 0 instead of ~180.
            if args.accel_sign == "invert":
                accel_sign = -1.0
            elif args.accel_sign == "normal":
                accel_sign = 1.0
            else:
                au = _normalize3(a_avg)
                if au is not None and float(np.dot(au, UP_W)) < -0.7:
                    accel_sign = -1.0

            a_avg = a_avg * accel_sign

            mag_for_init = None
            if m_avg is not None:
                ok_init, _, _ = _mag_health(
                    m_avg,
                    baseline_uT,
                    tol_ratio=float(args.mag_tol),
                    max_step_uT=float(args.mag_max_step),
                    prev_mag_norm_uT=None,
                )
                if ok_init:
                    mag_for_init = m_avg

            q_init = initial_quaternion_from_accel_mag(a_avg, mag_for_init)
        else:
            q_init = Quaternion.identity()

        filt = MadgwickAHRS(cfg=MadgwickConfig(beta=float(args.beta)))
        filt.q = q_init

        yaw0: Optional[float] = None
        last_print = 0.0
        last_perf = _perf()
        prev_mag_norm: Optional[float] = None
        t_start = _perf()
        q_zero: Optional[Quaternion] = None

        # CSV logging
        csv_f = None
        csv_w = None
        if args.log_csv:
            Path(args.log_csv).parent.mkdir(parents=True, exist_ok=True)
            csv_f = open(args.log_csv, "w", newline="")
            csv_w = csv.writer(csv_f)
            csv_w.writerow([
                "ts",
                "roll_deg","pitch_deg","yaw_deg",
                "qw","qx","qy","qz",
                "accel_x","accel_y","accel_z",
                "gyro_x","gyro_y","gyro_z",
                "mag_x","mag_y","mag_z",
                "mag_src","accel_ok","mag_ok","mode",
                "mag_norm_uT","baseline_uT","mag_step_uT",
                "gyro_norm_rad_s","stationary","beta","gyro_bias_x","gyro_bias_y","gyro_bias_z","accel_sign",
            ])

        while True:
            t0 = _perf()
            dt = t0 - last_perf
            last_perf = t0
            # guard dt spikes
            if not math.isfinite(dt) or dt <= 0.0:
                dt = dt_target
            dt = min(max(dt, 1e-4), 0.05)

            a = board.read_accel()
            g = board.read_gyro()
            mag_src, m = board.read_mag()
            av_raw = np.asarray(mount.apply((a.x, a.y, a.z)), dtype=float) * accel_sign
            gv_raw = np.asarray(mount.apply((g.x, g.y, g.z)), dtype=float)
            mv_raw = np.asarray(mount.apply((m.x, m.y, m.z)), dtype=float)

            if mag_cal is not None:
                mv_raw = mag_cal.apply(mv_raw)

            # Low-pass to reduce jitter. (Does not change mean values, only noise.)
            av = accel_lpf.update(av_raw, dt)
            gv_f = gyro_lpf.update(gv_raw, dt)
            mv = mag_lpf.update(mv_raw, dt)

            gyro_norm = float(np.linalg.norm(gv_f))
            # Health checks / stationarity
            a_norm = float(np.linalg.norm(av))
            accel_ok = bool(abs(a_norm - G) <= float(args.accel_g_tol) * G)

            stationary = bool(accel_ok and (gyro_norm <= float(args.stationary_gyro_rad)))

            mag_ok, mag_norm, mag_step = _mag_health(
                mv,
                baseline_uT,
                tol_ratio=float(args.mag_tol),
                max_step_uT=float(args.mag_max_step),
                prev_mag_norm_uT=prev_mag_norm,
            )
            prev_mag_norm = mag_norm

            # Online gyro bias adaptation when stationary (helps real-world drift with minimal tuning).
            tau_bias = float(args.bias_adapt_tau)
            if tau_bias > 0.0 and stationary:
                alpha_b = float(dt / (tau_bias + dt))
                gyro_bias = (1.0 - alpha_b) * gyro_bias + alpha_b * gv_f

            gv = gv_f - gyro_bias

            # Beta schedule: fast converge at startup, then steady-state, with a small bump when stationary.
            t_since_start = float(t0 - t_start)
            beta_cur = float(args.beta)
            if float(args.warmup_seconds) > 0.0 and t_since_start < float(args.warmup_seconds):
                beta_cur = float(args.beta_init)
            elif stationary:
                beta_cur = float(args.beta_stationary)
            filt.cfg.beta = beta_cur

            # Choose update mode
            mode = "gyro"
            if accel_ok and mag_ok:
                mode = "9dof"
                q = filt.update(gv, av, dt, mag_uT=mv)
            elif accel_ok:
                mode = "6dof"
                q = filt.update(gv, av, dt, mag_uT=None)
            else:
                # high dynamics: don't trust accel/mag. integrate gyro only.
                q = filt.q.integrate_gyro(gv, dt)
                filt.q = q

            # Optional: zero the full attitude at startup (relative roll/pitch/yaw output).
            q_out = q
            if args.zero_attitude:
                if q_zero is None and accel_ok:
                    q_zero = q.conj()
                if q_zero is not None:
                    q_out = q_zero * q

            roll, pitch, yaw = quat_to_euler_deg(q_out)

            # Zero yaw if requested (operator-friendly). Applied after zero-attitude if both are enabled.
            if args.yaw_zero:
                if yaw0 is None and accel_ok:
                    yaw0 = yaw
                if yaw0 is not None:
                    yaw = wrap_degrees(yaw - yaw0)
            yaw = wrap_degrees(yaw)

            ts = _now()

            if csv_w is not None:
                csv_w.writerow([
                    f"{ts:.6f}",
                    f"{roll:.3f}", f"{pitch:.3f}", f"{yaw:.3f}",
                    f"{q_out.w:.8f}", f"{q_out.x:.8f}", f"{q_out.y:.8f}", f"{q_out.z:.8f}",
                    f"{av[0]:.6f}", f"{av[1]:.6f}", f"{av[2]:.6f}",
                    f"{gv[0]:.6f}", f"{gv[1]:.6f}", f"{gv[2]:.6f}",
                    f"{mv[0]:.3f}", f"{mv[1]:.3f}", f"{mv[2]:.3f}",
                    mag_src,
                    int(accel_ok), int(mag_ok), mode,
                    f"{mag_norm:.3f}", f"{baseline_uT if baseline_uT is not None else float('nan'):.3f}", f"{mag_step:.3f}",
                    f"{gyro_norm:.6f}", int(stationary), f"{beta_cur:.4f}",
                    f"{gyro_bias[0]:.6f}", f"{gyro_bias[1]:.6f}", f"{gyro_bias[2]:.6f}", f"{accel_sign:.1f}",
                ])
                csv_f.flush()

            # Print at print-rate
            if float(args.print_rate) > 0:
                if (t0 - last_print) >= (1.0 / float(args.print_rate)):
                    last_print = t0
                    if args.json:
                        out = {
                            "ts": ts,
                            "rpy_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
                            "q": {"w": q_out.w, "x": q_out.x, "y": q_out.y, "z": q_out.z},
                            "accel": {"x": float(av[0]), "y": float(av[1]), "z": float(av[2])},
                            "gyro": {"x": float(gv[0]), "y": float(gv[1]), "z": float(gv[2])},
                            "mag": {"x": float(mv[0]), "y": float(mv[1]), "z": float(mv[2]), "source": mag_src},
                            "health": {
                                "accel_ok": accel_ok,
                                "mag_ok": mag_ok,
                                "mode": mode,
                                "stationary": stationary,
                                "gyro_norm_rad_s": gyro_norm,
                                "beta": beta_cur,
                                "gyro_bias_rad_s": {"x": float(gyro_bias[0]), "y": float(gyro_bias[1]), "z": float(gyro_bias[2])},
                                "accel_sign": accel_sign,
                                "mag_norm_uT": mag_norm,
                                "mag_baseline_uT": baseline_uT,
                                "mag_step_uT": mag_step,
                            },
                        }
                        sys.stdout.write(json.dumps(out) + "\n")
                        sys.stdout.flush()
                    else:
                        print(
                            f"r={roll:+7.2f}  p={pitch:+7.2f}  y={yaw:+7.2f}  "
                            f"mode={mode:4s} stat={int(stationary)} beta={beta_cur:0.3f} "
                            f"accel_ok={int(accel_ok)} mag_ok={int(mag_ok)} "
                            f"|B|={mag_norm:6.1f}uT src={mag_src}"
                        )

            # Sleep to maintain loop rate
            elapsed = _perf() - t0
            to_sleep = dt_target - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        return 0
    finally:
        try:
            board.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
