#!/usr/bin/env python3
"""
ROV-side thruster / PWM test.

This script is intentionally *low level* and can bypass the thruster-name map.

Why this exists:
- If your thruster map is wrong (e.g. motors plugged into PWM9-16),
  name-based tests can look "dead" even when PWM is fine.
- This script can also sweep *all* 16 PWM channels to help you map wiring.

Safety:
- REMOVE PROPS or secure vehicle before running.
- Start with small values, short durations.
"""

from __future__ import annotations

import argparse
import time
from typing import Dict

from motion.pwm import ThrustWriter, thrust_to_us

# Default names (if you use the built-in map)
# Canonical TritonOS thruster names:
#   H_FL, H_FR, H_RL, H_RR  (horizontal)
#   V_FL, V_FR, V_RL, V_RR  (vertical)
ALL = ["H_FL","H_FR","H_RL","H_RR","V_FL","V_FR","V_RL","V_RR"]


def one_thr(name: str, val: float) -> Dict[str, float]:
    d = {k: 0.0 for k in ALL}
    d[name] = float(val)
    return d


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--thr", choices=ALL, help="Thruster name (uses thruster map).")
    ap.add_argument("--channel", type=int, help="Raw PWM channel 0-15 (bypasses map).")
    ap.add_argument("--val", type=float, default=0.20, help="Normalized thrust (-1..1) when using --thr/--channel.")
    ap.add_argument("--us", type=int, help="Direct pulse width (µs). If set, bypasses --val scaling.")
    ap.add_argument("--secs", type=float, default=2.0, help="How long to hold the command.")
    ap.add_argument("--init-s", type=float, default=None, help="ESC neutral hold seconds (default from rov_config.ESC_INIT_S).")

    ap.add_argument("--scan", action="store_true", help="Sweep all 16 channels to find what is wired.")
    ap.add_argument("--scan-us", type=int, default=1700, help="Pulse width during scan (µs).")
    ap.add_argument("--scan-on", type=float, default=1.0, help="Seconds ON for each channel in scan.")
    ap.add_argument("--scan-off", type=float, default=0.5, help="Seconds neutral between channels in scan.")

    ap.add_argument("--i2c-bus", type=int, default=None)
    ap.add_argument("--i2c-addr", type=lambda x: int(x, 0), default=None)
    ap.add_argument("--freq", type=float, default=None)
    ap.add_argument("--osc-hz", type=float, default=None)
    ap.add_argument("--oe-chip", default=None)
    ap.add_argument("--oe-line", type=int, default=None)

    args = ap.parse_args()

    # Pull defaults from rov_config if present
    try:
        import rov_config as cfg  # type: ignore
        i2c_bus = int(getattr(cfg, "PWM_I2C_BUS", 4))
        i2c_addr = int(getattr(cfg, "PWM_I2C_ADDR", 0x40))
        freq_hz = float(getattr(cfg, "PWM_FREQ_HZ", 50.0))
        osc_hz = float(getattr(cfg, "PWM_OSC_HZ", 24_576_000.0))
        oe_chip = getattr(cfg, "PWM_OE_CHIP", "/dev/gpiochip0")
        oe_line = int(getattr(cfg, "PWM_OE_LINE", 26))
        init_s = float(getattr(cfg, "ESC_INIT_S", 5.0))
    except Exception:
        i2c_bus, i2c_addr, freq_hz, osc_hz = 4, 0x40, 50.0, 24_576_000.0
        oe_chip, oe_line = "/dev/gpiochip0", 26
        init_s = 5.0

    if args.i2c_bus is not None:
        i2c_bus = args.i2c_bus
    if args.i2c_addr is not None:
        i2c_addr = args.i2c_addr
    if args.freq is not None:
        freq_hz = args.freq
    if args.osc_hz is not None:
        osc_hz = args.osc_hz
    if args.oe_chip is not None:
        oe_chip = args.oe_chip
    if args.oe_line is not None:
        oe_line = args.oe_line
    if args.init_s is not None:
        init_s = args.init_s

    tw = ThrustWriter(
        i2c_bus=i2c_bus,
        i2c_addr=i2c_addr,
        oe_chip=oe_chip,
        oe_line=oe_line,
        freq_hz=freq_hz,
        osc_hz=osc_hz,
        debug=True,
    )

    try:
        info = tw.pwm.probe() if hasattr(tw.pwm, "probe") else {"ok": False}
        print(f"[thruster_test] PCA9685 probe: {info}")

        print(f"[thruster_test] enabling outputs (OE) + holding neutral for {init_s:.1f}s ...")
        tw.enable_outputs()
        tw.esc_init(init_s)

        if args.scan:
            print("[thruster_test] SCAN mode: sweeping channels 0..15")
            for ch in range(16):
                print(f"  channel {ch:02d}: {args.scan_us} µs for {args.scan_on:.1f}s")
                tw.pwm.set_servo_us(ch, args.scan_us)
                time.sleep(args.scan_on)
                tw.pwm.set_servo_us(ch, tw.neutral_us)
                time.sleep(args.scan_off)
            print("[thruster_test] scan complete")
            return

        # One-shot mode
        if args.channel is not None:
            if not (0 <= args.channel <= 15):
                raise SystemExit("--channel must be 0..15")
            if args.us is not None:
                us = int(args.us)
            else:
                # use ThrustWriter conversion
                us = thrust_to_us(float(args.val), neutral_us=tw.neutral_us, span_us=tw.span_us, deadband=tw.deadband, min_us=tw.min_us, max_us=tw.max_us)
            print(f"[thruster_test] channel {args.channel} -> {us} µs for {args.secs:.1f}s")
            tw.pwm.set_servo_us(args.channel, us)
            time.sleep(args.secs)
            tw.pwm.set_servo_us(args.channel, tw.neutral_us)
            return

        if args.thr is None:
            raise SystemExit("Specify --thr, --channel, or --scan")

        print(f"[thruster_test] {args.thr} at {args.val:+.2f} for {args.secs:.1f}s")
        tw.write(one_thr(args.thr, args.val))
        time.sleep(args.secs)
        tw.neutral()
    finally:
        try:
            tw.neutral()
        except Exception:
            pass
        tw.close()


if __name__ == "__main__":
    main()
