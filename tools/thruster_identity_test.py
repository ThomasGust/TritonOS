#!/usr/bin/env python3
"""
Thruster identity test (ROV-side).

Purpose:
- Verify that TritonOS -> ThrustWriter -> Navigator PWM is commanding the
  *same* channel numbers you verified with tools/native_motor_test.
- Helps diagnose "two thrusters never spin" issues caused by channel indexing
  mismatches or enum/int confusion.

Usage (on the ROV):
  sudo .venv/bin/python -m tools.thruster_identity_test

Options:
  --val 0.20      normalized thrust to apply
  --on  1.5       seconds on each thruster
  --off 0.6       seconds neutral between thrusters
  --only H_FR     test only one thruster name
"""

from __future__ import annotations
import argparse
import time

import rov_config as cfg
from motion.pwm import ThrustWriter, ThrustConfig

def main() -> None:
    """Cycle configured thruster names through the production ThrustWriter path."""

    ap = argparse.ArgumentParser()
    ap.add_argument("--val", type=float, default=0.20)
    ap.add_argument("--on", type=float, default=1.5)
    ap.add_argument("--off", type=float, default=0.6)
    ap.add_argument("--only", type=str, default=None, help="Test only this thruster name (e.g. H_FR).")
    args = ap.parse_args()

    tw = ThrustWriter(
        thruster_channels=getattr(cfg, "THRUSTER_CHANNELS", None),
        cfg=ThrustConfig(
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
            channel_base=1,
        ),
        reversed_map=getattr(cfg, "THRUSTER_REVERSED", None),
        debug=True,
        auto_enable=True,
    )

    try:
        tw.arm()
        names = list(tw.thruster_channels.keys())
        if args.only:
            if args.only not in tw.thruster_channels:
                raise SystemExit(f"--only {args.only!r} not in THRUSTER_CHANNELS keys: {names}")
            names = [args.only]

        print("[thruster_identity_test] armed; cycling thrusters:", names)
        for name in names:
            thr = {k: 0.0 for k in tw.thruster_channels.keys()}
            thr[name] = float(args.val)
            ch = tw.thruster_channels[name]
            print(f"[thruster_identity_test] {name}: user_channel={ch}  -> applying {args.val:+.2f} for {args.on:.1f}s")
            tw.write(thr)
            time.sleep(float(args.on))
            tw.write({k: 0.0 for k in tw.thruster_channels.keys()})
            time.sleep(float(args.off))
        print("[thruster_identity_test] done")
    finally:
        try:
            tw.disarm()
        except Exception:
            pass

if __name__ == "__main__":
    main()
