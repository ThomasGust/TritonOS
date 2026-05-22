#!/usr/bin/env python3
"""Run a simple sequential thruster test.

This tool uses the same ThrustWriter + rov_config settings as the full system,
but without any ZMQ/control dependencies.

Examples
--------
  # Gently spin each configured thruster for 2 seconds at 15% forward
  sudo .venv/bin/python -m tools.thruster_test

  # Only test the front-left horizontal thruster
  sudo .venv/bin/python -m tools.thruster_test --thruster H_FL

  # Reverse direction and increase power
  sudo .venv/bin/python -m tools.thruster_test --power 0.25 --reverse
"""

from __future__ import annotations

import argparse
import time

import rov_config as cfg
from motion.pwm import ThrustConfig, ThrustWriter


def main() -> None:
    """Run a sequential configured-thruster test with safe shutdown."""

    ap = argparse.ArgumentParser()
    ap.add_argument("--thruster", default="", help="Name to test (e.g. H_FL). Empty = all")
    ap.add_argument("--power", type=float, default=0.15, help="Normalized power (0..1)")
    ap.add_argument("--seconds", type=float, default=2.0, help="Duration per thruster")
    ap.add_argument("--reverse", action="store_true", help="Spin in reverse")
    args = ap.parse_args()

    power = max(0.0, min(1.0, float(args.power)))
    if args.reverse:
        power = -power

    thrust_cfg = ThrustConfig(
        freq_hz=getattr(cfg, "PWM_FREQ_HZ", 50.0),
        neutral_us=getattr(cfg, "PWM_NEUTRAL_US", 1500),
        span_us=getattr(cfg, "PWM_SPAN_US", 400),
        min_us=getattr(cfg, "PWM_MIN_US", 1100),
        max_us=getattr(cfg, "PWM_MAX_US", 1900),
        deadband_norm=getattr(cfg, "PWM_DEADBAND", 0.07),
        deadband_us=getattr(cfg, "PWM_DEADBAND_US", 25),
        trim_us=getattr(cfg, "PWM_TRIM_US", 0),
        esc_init_hold_s=getattr(cfg, "ESC_INIT_HOLD_S", 3.0),
        keep_pwm_enabled_on_disarm=getattr(cfg, "KEEP_PWM_ENABLED_ON_DISARM", True),
    )

    tw = ThrustWriter(
        thruster_channels=getattr(cfg, "THRUSTER_CHANNELS", None),
        cfg=thrust_cfg,
        reversed_map=getattr(cfg, "THRUSTER_REVERSED", None),
        debug=getattr(cfg, "DEBUG", False),
        auto_enable=True,
    )

    # We use the sink's arming gate so accidental writes remain neutral.
    tw.arm()

    names = list(tw.thruster_channels.keys())
    names.sort()
    if args.thruster:
        if args.thruster not in tw.thruster_channels:
            raise SystemExit(f"Unknown thruster '{args.thruster}'. Known: {names}")
        names = [args.thruster]

    print("[thruster_test] configured thrusters:")
    for n in names:
        print(f"  {n}: PWM channel {tw.thruster_channels[n]}")

    try:
        for n in names:
            print(f"[thruster_test] {n} -> {power:+.2f} for {args.seconds:.1f}s")
            cmd = {k: 0.0 for k in tw.thruster_channels.keys()}
            cmd[n] = power
            tw.write(cmd)
            time.sleep(float(args.seconds))
            print(f"[thruster_test] {n} -> neutral")
            tw.write({k: 0.0 for k in tw.thruster_channels.keys()})
            time.sleep(10)
    finally:
        print("[thruster_test] shutdown")
        tw.shutdown()

"""
HFL NONE
HFR HFR
HRL HRL
HRR HRR
VFL VFL
VFR VFR
VRL VRL
VRR
"""
if __name__ == "__main__":
    main()
