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
from .quaternion import quat_to_euler_deg, wrap_degrees

G = 9.80665


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

        filt = MadgwickAHRS(cfg=MadgwickConfig(beta=float(args.beta)))

        yaw0: Optional[float] = None
        last_print = 0.0
        last_perf = _perf()
        prev_mag_norm: Optional[float] = None

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

            av = mount.apply((a.x, a.y, a.z))
            gv = mount.apply((g.x, g.y, g.z))
            mv = mount.apply((m.x, m.y, m.z))

            if gyro_cal is not None:
                gv = gyro_cal.apply(gv)

            if mag_cal is not None:
                mv = mag_cal.apply(mv)

            # Health checks
            a_norm = float(np.linalg.norm(av))
            accel_ok = bool(abs(a_norm - G) <= float(args.accel_g_tol) * G)

            mag_ok, mag_norm, mag_step = _mag_health(
                mv,
                baseline_uT,
                tol_ratio=float(args.mag_tol),
                max_step_uT=float(args.mag_max_step),
                prev_mag_norm_uT=prev_mag_norm,
            )
            prev_mag_norm = mag_norm

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

            roll, pitch, yaw = quat_to_euler_deg(q)

            # Zero yaw if requested
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
                    f"{q.w:.8f}", f"{q.x:.8f}", f"{q.y:.8f}", f"{q.z:.8f}",
                    f"{av[0]:.6f}", f"{av[1]:.6f}", f"{av[2]:.6f}",
                    f"{gv[0]:.6f}", f"{gv[1]:.6f}", f"{gv[2]:.6f}",
                    f"{mv[0]:.3f}", f"{mv[1]:.3f}", f"{mv[2]:.3f}",
                    mag_src,
                    int(accel_ok), int(mag_ok), mode,
                    f"{mag_norm:.3f}", f"{baseline_uT if baseline_uT is not None else float('nan'):.3f}", f"{mag_step:.3f}",
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
                            "q": {"w": q.w, "x": q.x, "y": q.y, "z": q.z},
                            "accel": {"x": float(av[0]), "y": float(av[1]), "z": float(av[2])},
                            "gyro": {"x": float(gv[0]), "y": float(gv[1]), "z": float(gv[2])},
                            "mag": {"x": float(mv[0]), "y": float(mv[1]), "z": float(mv[2]), "source": mag_src},
                            "health": {
                                "accel_ok": accel_ok,
                                "mag_ok": mag_ok,
                                "mode": mode,
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
                            f"mode={mode:4s} accel_ok={int(accel_ok)} mag_ok={int(mag_ok)} "
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
