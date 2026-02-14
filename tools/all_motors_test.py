#!/usr/bin/env python3
"""Spin multiple Navigator PWM channels at the same time.

This is a *satisfaction* / bring-up tool: it lets you see all thrusters
turn together without going through the full control stack.

It talks directly to the Navigator PWM outputs via ``bluerobotics_navigator``
(similar to tools/native_motor_test.py), but supports a list of channels.

Examples
--------
  # Spin the common thruster set (1,2,3,5,6,7,8) forward then reverse
  sudo .venv/bin/python -m tools.all_motors_test

  # Use the channels from rov_config.MOTOR_PWM_CHANNELS
  sudo .venv/bin/python -m tools.all_motors_test --use-config

  # Specify channels explicitly
  sudo .venv/bin/python -m tools.all_motors_test --channels 0,1,2,3,5,6,7,8 --throttle 0.20

Safety
------
  * Start with the ROV out of the water or securely restrained.
  * This tool will enable PWM outputs (OE) during the test.
  * Always keep your hand near the power cutoff.
"""

from __future__ import annotations

import argparse
import time
from typing import List

import bluerobotics_navigator as nav


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Spin multiple Navigator PWM channels simultaneously.")
    ap.add_argument(
        "--channels",
        default="1,2,3,5,6,7,8",
        help="Comma-separated Navigator PWM channels to spin (default: %(default)s).",
    )
    ap.add_argument(
        "--use-config",
        action="store_true",
        help="Use rov_config.MOTOR_PWM_CHANNELS (and exclude LIGHTS_PWM_CHANNEL if present).",
    )
    ap.add_argument("--freq", type=float, default=50.0, help="PWM frequency in Hz (default: %(default)s)")
    ap.add_argument("--neutral-us", type=int, default=1500, help="Neutral pulse width (default: %(default)s)")
    ap.add_argument("--span-us", type=int, default=400, help="Span around neutral (default: %(default)s)")
    ap.add_argument("--min-us", type=int, default=1100, help="Minimum pulse width (default: %(default)s)")
    ap.add_argument("--max-us", type=int, default=1900, help="Maximum pulse width (default: %(default)s)")
    ap.add_argument("--throttle", type=float, default=0.20, help="Peak throttle (0..1), default: %(default)s")
    ap.add_argument("--hold-s", type=float, default=1.5, help="Hold time at peak (seconds)")
    ap.add_argument("--esc-init-s", type=float, default=3.0, help="Neutral hold for ESC init (seconds)")
    ap.add_argument("--ramp-step", type=float, default=0.02, help="Ramp step size (default: %(default)s)")
    ap.add_argument("--ramp-dt", type=float, default=0.10, help="Delay between ramp steps (default: %(default)s)")
    return ap.parse_args()


def us_to_count(pulse_us: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / float(freq_hz)
    value = round(4095.0 * (float(pulse_us) / period_us))
    return max(0, min(4095, int(value)))


def throttle_to_us(
    throttle: float,
    neutral_us: int,
    span_us: int,
    min_us: int,
    max_us: int,
) -> int:
    t = max(-1.0, min(1.0, float(throttle)))
    pulse = float(neutral_us) + float(span_us) * t
    pulse = max(float(min_us), min(float(max_us), pulse))
    return int(round(pulse))


def set_pwm_us(ch: int, pulse_us: float, freq_hz: float) -> None:
    nav.set_pwm_channel_value(int(ch), us_to_count(pulse_us, freq_hz))


def unique_ints(xs: List[int]) -> List[int]:
    out: List[int] = []
    for x in xs:
        if x not in out:
            out.append(x)
    return out


def main() -> None:
    args = parse_args()

    # Resolve channels
    channels: List[int]
    if args.use_config:
        try:
            import rov_config as cfg

            channels = [int(x) for x in getattr(cfg, "MOTOR_PWM_CHANNELS", [])]
            lights_ch = getattr(cfg, "LIGHTS_PWM_CHANNEL", None)
            if lights_ch is not None:
                channels = [c for c in channels if int(c) != int(lights_ch)]
        except Exception:
            channels = []
    else:
        channels = []

    if not channels:
        channels = [int(x.strip()) for x in str(args.channels).split(",") if x.strip()]

    channels = unique_ints(channels)
    if not channels:
        raise SystemExit("No channels specified.")

    channels = [0, 1, 2, 3, 5, 6, 7, 8]
    # Safety: show what we will do
    print("=== all_motors_test ===")
    print("Channels:", channels)
    print(f"freq={args.freq}Hz neutral={args.neutral_us}us span={args.span_us}us")
    print(f"throttle=±{args.throttle:.2f} (hold {args.hold_s:.1f}s), esc_init={args.esc_init_s:.1f}s")

    throttle = max(0.0, min(1.0, float(args.throttle)))
    ramp_step = max(0.005, float(args.ramp_step))
    ramp_dt = max(0.02, float(args.ramp_dt))

    # Optional: NeoPixel strip size should be set before init (if you use it).
    if hasattr(nav, "set_rgb_led_strip_size"):
        nav.set_rgb_led_strip_size(1)

    if hasattr(nav, "Raspberry") and hasattr(nav, "set_raspberry_pi_version"):
        try:
            nav.set_raspberry_pi_version(nav.Raspberry.Pi4)
        except Exception:
            pass

    if hasattr(nav, "init"):
        nav.init()

    nav.set_pwm_freq_hz(float(args.freq))

    # Neutral + enable
    for ch in channels:
        set_pwm_us(ch, float(args.neutral_us), float(args.freq))
    nav.set_pwm_enable(True)
    time.sleep(float(args.esc_init_s))

    def write_all(t: float) -> None:
        pulse = throttle_to_us(
            t,
            neutral_us=int(args.neutral_us),
            span_us=int(args.span_us),
            min_us=int(args.min_us),
            max_us=int(args.max_us),
        )
        for ch in channels:
            set_pwm_us(ch, pulse, float(args.freq))

    try:
        # Forward ramp
        print("[step] ramp forward")
        t = 0.0
        while t < throttle + 1e-6:
            write_all(t)
            time.sleep(ramp_dt)
            t += ramp_step

        print("[step] hold forward")
        time.sleep(float(args.hold_s))

        print("[step] back to neutral")
        write_all(0.0)
        time.sleep(0.75)

        # Reverse ramp
        print("[step] ramp reverse")
        t = 0.0
        while t > -throttle - 1e-6:
            write_all(t)
            time.sleep(ramp_dt)
            t -= ramp_step

        print("[step] hold reverse")
        time.sleep(float(args.hold_s))

    finally:
        print("[step] neutral + disable")
        write_all(0.0)
        time.sleep(0.25)
        try:
            nav.set_pwm_enable(False)
        except Exception:
            pass
        print("=== done ===")


if __name__ == "__main__":
    main()
