#!/usr/bin/env python3
from __future__ import annotations

import argparse

import rov_config as cfg
from sensors.navigator import NavigatorBoard
from utils.vehicle_reference import (
    DEFAULT_DEPTH_REFERENCE_PATH,
    DEFAULT_FLAT_MOUNT_PATH,
    capture_flat_mount_reference,
    capture_surface_pressure_reference,
    save_mount_reference,
    save_surface_pressure_reference,
)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Capture and save TritonOS surface-pressure and flat-pose references."
    )
    ap.add_argument("--depth-only", action="store_true", help="Only save the surface-pressure reference.")
    ap.add_argument("--flat-only", action="store_true", help="Only save the flat-mount reference.")
    ap.add_argument("--depth-path", default=getattr(cfg, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
    ap.add_argument("--mount-path", default=getattr(cfg, "ATTITUDE_MOUNT", DEFAULT_FLAT_MOUNT_PATH))
    ap.add_argument("--pressure-samples", type=int, default=20, help="Depth-sensor samples to average.")
    ap.add_argument("--accel-samples", type=int, default=200, help="Accelerometer samples to average.")
    ap.add_argument("--sample-delay", type=float, default=0.02, help="Delay between captured samples.")
    ap.add_argument(
        "--yaw-deg",
        type=float,
        default=float(getattr(cfg, "ATTITUDE_AUTO_MOUNT_YAW_DEG", 0.0)),
        help="Extra yaw rotation to apply to the saved flat mount.",
    )
    args = ap.parse_args()

    capture_depth = not bool(args.flat_only)
    capture_flat = not bool(args.depth_only)

    if capture_depth:
        p0 = capture_surface_pressure_reference(cfg, samples=args.pressure_samples, delay_s=args.sample_delay)
        save_surface_pressure_reference(
            args.depth_path,
            p0,
            meta={
                "samples": int(args.pressure_samples),
                "sample_delay_s": float(args.sample_delay),
                "sensor_to_top_m": float(getattr(cfg, "EXTERNAL_DEPTH_SENSOR_TO_TOP_M", 0.0)),
            },
        )
        print(f"[set_vehicle_reference] saved surface pressure {p0:.3f} mbar -> {args.depth_path}")

    if capture_flat:
        board = NavigatorBoard()
        mount, accel_avg = capture_flat_mount_reference(
            board,
            samples=args.accel_samples,
            delay_s=args.sample_delay,
            yaw_deg=float(args.yaw_deg),
        )
        save_mount_reference(
            args.mount_path,
            mount,
            meta={
                "accel_avg": [float(x) for x in accel_avg.tolist()],
                "samples": int(args.accel_samples),
                "sample_delay_s": float(args.sample_delay),
                "yaw_deg": float(args.yaw_deg),
            },
        )
        print(f"[set_vehicle_reference] saved flat mount -> {args.mount_path}")

    if (not capture_depth) and (not capture_flat):
        print("[set_vehicle_reference] nothing selected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
