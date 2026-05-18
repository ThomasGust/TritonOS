#!/usr/bin/env python3
from __future__ import annotations

import argparse

import rov_config as cfg
from utils.vehicle_reference import (
    DEFAULT_DEPTH_REFERENCE_PATH,
    capture_surface_pressure_reference,
    save_surface_pressure_reference,
)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Capture and save TritonOS surface-pressure references."
    )
    ap.add_argument("--depth-path", default=getattr(cfg, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
    ap.add_argument("--pressure-samples", type=int, default=20, help="Depth-sensor samples to average.")
    ap.add_argument("--sample-delay", type=float, default=0.02, help="Delay between captured samples.")
    args = ap.parse_args()

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
