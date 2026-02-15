from __future__ import annotations

import argparse
import time
import sys

import numpy as np

from .calibration import Mount, calibrate_mag_softiron, load_json, mag_baseline_uT, save_json
from .navigator import NavigatorIMU


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Magnetometer calibration (hard + soft iron)")
    p.add_argument("--out", type=str, default="mag_cal.json", help="Output JSON")
    p.add_argument("--seconds", type=float, default=60.0, help="Sampling duration")
    p.add_argument("--rate", type=float, default=100.0, help="Sampling rate")
    p.add_argument("--mount", type=str, default="", help="Mount JSON (sensor->body)")
    p.add_argument("--imu-i2c-bus", type=int, default=1)
    p.add_argument("--ak-i2c-bus", type=int, default=1)
    p.add_argument("--prefer-spi", action="store_true")
    p.add_argument("--disable-mmc5983", action="store_true")
    args = p.parse_args(argv)

    mount = Mount.identity()
    if args.mount:
        mount = Mount.from_dict(load_json(args.mount))

    board = NavigatorIMU(
        imu_i2c_bus=int(args.imu_i2c_bus),
        prefer_spi=bool(args.prefer_spi),
        ak_i2c_bus=int(args.ak_i2c_bus),
        enable_mmc5983=not bool(args.disable_mmc5983),
    )

    dt = 1.0 / max(20.0, float(args.rate))

    try:
        print("Mag calibration capture")
        print("- Rotate the board through as many orientations as possible.")
        print("- Go slow. Try to cover all axes (like slowly tumbling a cube).")
        print("- Avoid running thrusters / high current loads during capture.")
        print(f"Capturing for {args.seconds:.1f}s at ~{args.rate:.0f} Hz ...")
        time.sleep(0.5)

        samples = []
        src_used = None
        t_end = time.perf_counter() + float(args.seconds)
        last_print = 0.0

        while time.perf_counter() < t_end:
            src, m = board.read_mag()
            src_used = src_used or src
            mv = mount.apply((m.x, m.y, m.z))
            samples.append(mv)

            now = time.perf_counter()
            if now - last_print > 1.0:
                last_print = now
                pct = 100.0 * (1.0 - (t_end - now) / float(args.seconds))
                sys_line = f"  {pct:5.1f}%  samples={len(samples)}  src={src}"
                print(sys_line)

            time.sleep(dt)

        data = np.array(samples, dtype=float)
        print(f"Captured {data.shape[0]} samples")

        cal = calibrate_mag_softiron(data)
        baseline = mag_baseline_uT(cal, data)

        # Quality report
        cal_data = (cal.A @ (data - cal.bias_uT).T).T
        mags = np.linalg.norm(cal_data, axis=1)
        print(f"Post-cal |B|: median={np.median(mags):.2f} uT  mean={np.mean(mags):.2f} uT  std={np.std(mags):.2f} uT")

        out = {
            **cal.to_dict(),
            "baseline_uT": float(baseline),
            "source": src_used,
            "captured_samples": int(data.shape[0]),
            "method": "sphere-center + covariance-whitening",
        }
        save_json(out, args.out)
        print(f"Saved -> {args.out}")
        return 0

    finally:
        board.close()


if __name__ == "__main__":
    raise SystemExit(main())
