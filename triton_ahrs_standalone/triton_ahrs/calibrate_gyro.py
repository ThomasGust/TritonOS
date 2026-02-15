from __future__ import annotations

import argparse
import time

from .calibration import Mount, GyroCalibration, load_json, save_json
from .navigator import NavigatorIMU
from .run_ahrs import calibrate_gyro_bias


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Gyro bias calibration (stationary)")
    p.add_argument("--out", type=str, default="gyro_cal.json", help="Output JSON")
    p.add_argument("--seconds", type=float, default=5.0, help="Stationary duration")
    p.add_argument("--rate", type=float, default=200.0, help="Sampling rate")
    p.add_argument("--mount", type=str, default="", help="Mount JSON (sensor->body)")
    p.add_argument("--imu-i2c-bus", type=int, default=1)
    p.add_argument("--ak-i2c-bus", type=int, default=1)
    p.add_argument("--prefer-spi", action="store_true")
    args = p.parse_args(argv)

    mount = Mount.identity()
    if args.mount:
        mount = Mount.from_dict(load_json(args.mount))

    board = NavigatorIMU(
        imu_i2c_bus=int(args.imu_i2c_bus),
        prefer_spi=bool(args.prefer_spi),
        ak_i2c_bus=int(args.ak_i2c_bus),
        enable_mmc5983=False,
    )

    try:
        print(f"Hold the board/ROV still for {args.seconds:.1f}s...")
        time.sleep(0.5)
        cal = calibrate_gyro_bias(board, seconds=float(args.seconds), rate_hz=float(args.rate), mount=mount)
        b = cal.bias_rad_s
        print(f"Gyro bias (rad/s): x={b[0]:+.6f} y={b[1]:+.6f} z={b[2]:+.6f}")
        save_json(cal.to_dict(), args.out)
        print(f"Saved -> {args.out}")
        return 0
    finally:
        board.close()


if __name__ == "__main__":
    raise SystemExit(main())
