from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .calibration import GyroCalibration, MagCalibration, Mount, load_json, save_json
from .madgwick import MadgwickAHRS, MadgwickConfig
from .navigator import NavigatorIMU
from .quaternion import Quaternion, quat_to_euler_deg, wrap_degrees

G = 9.80665


def _now() -> float:
    return time.time()


def _perf() -> float:
    return time.perf_counter()


class EMA3:
    """Simple 3D exponential moving average with time-constant tau."""

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


def calibrate_gyro_bias(board: NavigatorIMU, seconds: float, rate_hz: float, mount: Mount) -> GyroCalibration:
    """Estimate gyro bias (rad/s) from a stationary window."""
    dt = 1.0 / max(10.0, float(rate_hz))
    acc = np.zeros(3, dtype=float)
    t_end = _perf() + float(seconds)
    k = 0
    while _perf() < t_end:
        g = board.read_gyro()
        gv = mount.apply((g.x, g.y, g.z))
        acc += gv
        k += 1
        time.sleep(dt)
    if k <= 0:
        raise RuntimeError("gyro calibration failed: no samples")
    return GyroCalibration(bias_rad_s=acc / float(k))


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


def _normalize3(v: np.ndarray) -> Optional[np.ndarray]:
    n = float(np.linalg.norm(v))
    if not math.isfinite(n) or n <= 1e-12:
        return None
    return v / n


def _initial_quaternion_from_accel_mag(accel: np.ndarray, mag: Optional[np.ndarray]) -> Quaternion:
    """Seed quaternion from accel (+ optional mag).

    accel and mag are in BODY coordinates (after mount + calibration).

    We set WORLD +Z to align with accel direction (i.e., "up" as seen by the IMU at rest).
    Yaw is chosen so that the horizontal projection of mag points along WORLD +X.
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

    # Rotate around WORLD Z so that mh aligns with +X
    yaw = math.atan2(float(mh[1]), float(mh[0]))
    q_yaw = Quaternion.from_axis_angle((0.0, 0.0, 1.0), -yaw)

    return (q_yaw * q_tilt).normalized()


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Standalone AHRS for RPi + Navigator IMU (robust)")

    # Loop / output
    p.add_argument("--rate", type=float, default=200.0, help="AHRS update rate (Hz)")
    p.add_argument("--print-rate", type=float, default=20.0, help="Console print rate (Hz)")
    p.add_argument("--log-csv", type=str, default="", help="Write CSV log to this path")
    p.add_argument("--json", action="store_true", help="Emit JSON lines to stdout")

    # Filter
    p.add_argument("--beta", type=float, default=0.08, help="Madgwick beta (steady-state)")
    p.add_argument("--beta-init", type=float, default=0.60, help="Madgwick beta during warmup")
    p.add_argument("--beta-stationary", type=float, default=0.12, help="Madgwick beta when stationary")
    p.add_argument("--warmup-seconds", type=float, default=1.5, help="Seconds to use beta-init after startup")

    # Initialization / presentation
    p.add_argument("--init-seconds", type=float, default=0.8, help="Seconds to average accel/mag for initial alignment")
    p.add_argument("--yaw-zero", action="store_true", help="Zero yaw at startup (operator-friendly)")
    p.add_argument("--zero-attitude", action="store_true", help="Zero roll/pitch/yaw at startup (relative attitude output)")
    p.add_argument("--accel-sign", choices=["auto", "normal", "invert"], default="auto", help="Accel sign for gravity (fix 180deg roll issues)")

    # Calibration files
    p.add_argument("--gyro-cal", type=str, default="", help="Path to gyro calibration JSON")
    p.add_argument("--mag-cal", type=str, default="", help="Path to mag calibration JSON")
    p.add_argument("--mount", type=str, default="", help="Path to mount (axis mapping) JSON")

    # Auto gyro calibration
    p.add_argument("--auto-gyro-cal", action="store_true", help="Auto-calibrate gyro bias at startup")
    p.add_argument("--gyro-cal-seconds", type=float, default=3.0, help="Stationary seconds for gyro bias")
    p.add_argument("--save-gyro-cal", type=str, default="", help="Save estimated gyro cal JSON to this path")

    # Sensor filtering
    p.add_argument("--accel-lpf-tau", type=float, default=0.05, help="Accel LPF time-constant (s), 0 disables")
    p.add_argument("--mag-lpf-tau", type=float, default=0.20, help="Mag LPF time-constant (s), 0 disables")
    p.add_argument("--gyro-lpf-tau", type=float, default=0.00, help="Gyro LPF time-constant (s), 0 disables")

    # Stationary detection & bias adaptation
    p.add_argument("--stationary-gyro-rad", type=float, default=0.03, help="Stationary threshold on |gyro| (rad/s)")
    p.add_argument("--bias-adapt-tau", type=float, default=60.0, help="Gyro bias adaptation tau (s) when stationary, 0 disables")

    # Health gating
    p.add_argument("--accel-g-tol", type=float, default=0.20, help="Accel magnitude tolerance (fraction of 1g)")
    p.add_argument("--mag-tol", type=float, default=0.35, help="Mag magnitude tolerance ratio around baseline")
    p.add_argument("--mag-max-step", type=float, default=8.0, help="Max per-sample |B| step in uT")
    p.add_argument("--mag-baseline-seconds", type=float, default=2.0, help="Seconds to estimate baseline |B| at startup")

    # Mag hysteresis
    p.add_argument("--mag-enable-up", type=float, default=1.0, help="Seconds to ramp mag enable up")
    p.add_argument("--mag-enable-down", type=float, default=0.3, help="Seconds to ramp mag enable down")

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
        # ---- gyro bias init ----
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

        gyro_bias = np.array(gyro_cal.bias_rad_s, dtype=float) if gyro_cal is not None else np.zeros(3, dtype=float)

        # ---- estimate baseline mag norm for gating ----
        baseline_uT: Optional[float] = None
        if float(args.mag_baseline_seconds) > 0.0:
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

        # ---- LPFs ----
        lpf_a = EMA3(float(args.accel_lpf_tau))
        lpf_m = EMA3(float(args.mag_lpf_tau))
        lpf_g = EMA3(float(args.gyro_lpf_tau))

        # ---- filter ----
        filt = MadgwickAHRS(cfg=MadgwickConfig(beta=float(args.beta)))

        # ---- initial alignment ----
        q_zero: Optional[Quaternion] = None
        yaw0: Optional[float] = None
        accel_sign_used = "normal"

        if float(args.init_seconds) > 0.0:
            a_s, m_s = [], []
            t_end = _perf() + float(args.init_seconds)
            while _perf() < t_end:
                a = board.read_accel()
                _src, m = board.read_mag()
                av = mount.apply((a.x, a.y, a.z))
                mv = mount.apply((m.x, m.y, m.z))
                if mag_cal is not None:
                    mv = mag_cal.apply(mv)
                a_s.append(av)
                m_s.append(mv)
                time.sleep(0.005)

            a_avg = np.mean(np.array(a_s), axis=0) if a_s else np.array([0.0, 0.0, G])
            m_avg = np.mean(np.array(m_s), axis=0) if m_s else None

            # accel sign handling
            if args.accel_sign == "normal":
                a_use = a_avg
                accel_sign_used = "normal"
            elif args.accel_sign == "invert":
                a_use = -a_avg
                accel_sign_used = "invert"
            else:
                # Heuristic: pick the sign that yields euler closer to (0,0,*)
                q1 = _initial_quaternion_from_accel_mag(a_avg, m_avg)
                r1, p1, _ = quat_to_euler_deg(q1)
                q2 = _initial_quaternion_from_accel_mag(-a_avg, m_avg)
                r2, p2, _ = quat_to_euler_deg(q2)
                cost1 = abs(wrap_degrees(r1)) + abs(p1)
                cost2 = abs(wrap_degrees(r2)) + abs(p2)
                if cost2 < cost1:
                    a_use = -a_avg
                    accel_sign_used = "invert"
                else:
                    a_use = a_avg
                    accel_sign_used = "normal"

            # determine if mag is healthy for init
            mag_for_init: Optional[np.ndarray] = None
            if m_avg is not None and baseline_uT is not None:
                ok, _, _ = _mag_health(m_avg, baseline_uT, float(args.mag_tol), float(args.mag_max_step), None)
                if ok:
                    mag_for_init = m_avg

            q_init = _initial_quaternion_from_accel_mag(a_use, mag_for_init)
            filt.q = q_init

            if args.zero_attitude:
                q_zero = q_init

            # yaw-zero uses euler (after q init)
            if args.yaw_zero:
                _r, _p, y = quat_to_euler_deg(q_init)
                yaw0 = y

            if not args.json:
                r, p_, y = quat_to_euler_deg(q_init)
                print(
                    f"[ahrs] init q: r={wrap_degrees(r):+.2f} p={p_:+.2f} y={wrap_degrees(y):+.2f} "
                    f"(accel_sign={accel_sign_used}, mag_init={'yes' if mag_for_init is not None else 'no'})"
                )

        # ---- main loop ----
        start_perf = _perf()
        last_perf = start_perf
        last_print = 0.0
        prev_mag_norm: Optional[float] = None
        mag_enable = 1.0  # hysteresis state

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
                "gyro_norm","stationary","beta","mag_enable",
                "gyro_bias_x","gyro_bias_y","gyro_bias_z",
                "accel_sign",
            ])

        while True:
            t0 = _perf()
            dt = t0 - last_perf
            last_perf = t0
            if not math.isfinite(dt) or dt <= 0.0:
                dt = dt_target
            dt = min(max(dt, 1e-4), 0.05)

            # read sensors
            a = board.read_accel()
            g = board.read_gyro()
            mag_src, m = board.read_mag()

            av = mount.apply((a.x, a.y, a.z))
            gv_raw = mount.apply((g.x, g.y, g.z))
            mv = mount.apply((m.x, m.y, m.z))

            # apply accel sign (same as init)
            if accel_sign_used == "invert":
                av = -av

            # mag calibration
            if mag_cal is not None:
                mv = mag_cal.apply(mv)

            # LPF
            av = lpf_a.update(av, dt)
            mv = lpf_m.update(mv, dt)
            gv_raw = lpf_g.update(gv_raw, dt)

            # stationary detection uses bias-corrected gyro
            gv = gv_raw - gyro_bias
            gyro_norm = float(np.linalg.norm(gv))

            a_norm = float(np.linalg.norm(av))
            accel_ok = bool(abs(a_norm - G) <= float(args.accel_g_tol) * G)
            stationary = bool(accel_ok and gyro_norm <= float(args.stationary_gyro_rad))

            # gyro bias adaptation
            if stationary and float(args.bias_adapt_tau) > 0.0:
                alpha = float(dt / max(1e-3, float(args.bias_adapt_tau)))
                gyro_bias = (1.0 - alpha) * gyro_bias + alpha * gv_raw
                gv = gv_raw - gyro_bias

            # mag health
            mag_ok, mag_norm, mag_step = _mag_health(
                mv,
                baseline_uT,
                tol_ratio=float(args.mag_tol),
                max_step_uT=float(args.mag_max_step),
                prev_mag_norm_uT=prev_mag_norm,
            )
            prev_mag_norm = mag_norm

            # mag hysteresis (prevents rapid toggling)
            if mag_ok:
                mag_enable = min(1.0, mag_enable + dt / max(1e-3, float(args.mag_enable_up)))
            else:
                mag_enable = max(0.0, mag_enable - dt / max(1e-3, float(args.mag_enable_down)))

            use_mag = bool(accel_ok and mag_enable >= 0.8)

            # beta scheduling
            t_since = t0 - start_perf
            beta = float(args.beta)
            if t_since < float(args.warmup_seconds):
                beta = float(args.beta_init)
            elif stationary:
                beta = max(beta, float(args.beta_stationary))
            filt.cfg.beta = beta

            # choose update mode
            mode = "gyro"
            if accel_ok and use_mag:
                mode = "9dof"
                q = filt.update(gv, av, dt, mag_uT=mv)
            elif accel_ok:
                mode = "6dof"
                q = filt.update(gv, av, dt, mag_uT=None)
            else:
                q = filt.q.integrate_gyro(gv, dt)
                filt.q = q

            # output quaternion relative to initial attitude if requested
            q_out = q
            if q_zero is not None:
                q_out = (q_zero.conj() * q).normalized()

            roll, pitch, yaw = quat_to_euler_deg(q_out)

            # Yaw zeroing (operator-friendly)
            if args.yaw_zero:
                if yaw0 is None and stationary:
                    yaw0 = yaw
                if yaw0 is not None:
                    yaw = wrap_degrees(yaw - yaw0)
            yaw = wrap_degrees(yaw)
            roll = wrap_degrees(roll)

            ts = _now()

            # CSV
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
                    f"{gyro_norm:.6f}",
                    int(stationary),
                    f"{beta:.5f}",
                    f"{mag_enable:.3f}",
                    f"{gyro_bias[0]:+.8f}", f"{gyro_bias[1]:+.8f}", f"{gyro_bias[2]:+.8f}",
                    accel_sign_used,
                ])
                csv_f.flush()

            # Print
            if float(args.print_rate) > 0 and (t0 - last_print) >= (1.0 / float(args.print_rate)):
                last_print = t0
                if args.json:
                    out = {
                        "ts": ts,
                        "rpy_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
                        "q": {"w": q_out.w, "x": q_out.x, "y": q_out.y, "z": q_out.z},
                        "health": {
                            "accel_ok": accel_ok,
                            "mag_ok": mag_ok,
                            "mode": mode,
                            "stationary": stationary,
                            "gyro_norm": gyro_norm,
                            "beta": beta,
                            "mag_enable": mag_enable,
                            "accel_sign": accel_sign_used,
                        },
                    }
                    sys.stdout.write(json.dumps(out) + "\n")
                    sys.stdout.flush()
                else:
                    print(
                        f"r={roll:+7.2f}  p={pitch:+7.2f}  y={yaw:+7.2f}  "
                        f"mode={mode:4s} stat={int(stationary)} beta={beta:0.3f} "
                        f"mag_en={mag_enable:0.2f} |B|={mag_norm:6.1f}uT src={mag_src}"
                    )

            # rate control
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
