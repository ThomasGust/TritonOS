#!/usr/bin/env python3
"""Smooth servo sweep test for Navigator/PCA9685 PWM outputs.

Default settings are aimed at a Blue Trail Engineering SER-2030 on Navigator
PWM channel 5:

  sudo .venv/bin/python -m tools.servo_sweep_test

Run continuously until Ctrl+C:

  sudo .venv/bin/python -m tools.servo_sweep_test --loops 0

Try a narrower range first if the servo is attached to a linkage:

  sudo .venv/bin/python -m tools.servo_sweep_test --min-us 1100 --max-us 1900
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Callable, Optional, Tuple

DEFAULT_CHANNEL = 5
DEFAULT_MIN_US = 900.0
DEFAULT_CENTER_US = 1500.0
DEFAULT_MAX_US = 2100.0
DEFAULT_FREQ_HZ = 50.0
DEFAULT_RATE_US_PER_SEC = 300.0
DEFAULT_STEP_US = 10.0
MIN_UPDATE_S = 0.02

_stop_requested = False


def parse_addr(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Smoothly sweep one Navigator PWM channel through a servo pulse range.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument(
        "-c",
        "--channel",
        type=int,
        default=DEFAULT_CHANNEL,
        help="Navigator-labeled PWM channel, 1..16.",
    )
    ap.add_argument(
        "--min-us",
        type=float,
        default=DEFAULT_MIN_US,
        help="Minimum servo pulse width in microseconds.",
    )
    ap.add_argument(
        "--center-us",
        type=float,
        default=DEFAULT_CENTER_US,
        help="Center/neutral servo pulse width in microseconds.",
    )
    ap.add_argument(
        "--max-us",
        type=float,
        default=DEFAULT_MAX_US,
        help="Maximum servo pulse width in microseconds.",
    )
    ap.add_argument(
        "--rate-us-per-sec",
        type=float,
        default=DEFAULT_RATE_US_PER_SEC,
        help="Commanded pulse-width slew rate. Lower is slower/smoother.",
    )
    ap.add_argument(
        "--step-us",
        type=float,
        default=DEFAULT_STEP_US,
        help="Approximate pulse-width increment between updates.",
    )
    ap.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Number of full center->end->end->center cycles. Use 0 for continuous.",
    )
    ap.add_argument(
        "--first",
        choices=("max", "min"),
        default="max",
        help="Which end of travel to visit first.",
    )
    ap.add_argument(
        "--arm-seconds",
        type=float,
        default=1.0,
        help="Seconds to hold center before starting the sweep.",
    )
    ap.add_argument(
        "--end-dwell",
        type=float,
        default=0.25,
        help="Seconds to pause at min/max endpoints.",
    )
    ap.add_argument(
        "--center-dwell",
        type=float,
        default=0.5,
        help="Seconds to pause at center between loops.",
    )
    ap.add_argument(
        "--curve",
        choices=("smoothstep", "linear"),
        default="smoothstep",
        help="Pulse profile for each ramp.",
    )
    ap.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ, help="PWM frequency in Hz.")
    ap.add_argument("--bus", type=int, default=4, help="Preferred I2C bus number.")
    ap.add_argument("--addr", type=parse_addr, default=0x40, help="Preferred PCA9685 I2C address.")
    ap.add_argument("--osc-hz", type=float, default=25_000_000.0, help="PCA9685 oscillator Hz.")
    ap.add_argument("--oe-gpio", type=int, default=26, help="BCM GPIO used for PCA9685 OE.")
    ap.add_argument(
        "--oe-active-high",
        action="store_true",
        help="Treat OE as active-high instead of the default active-low.",
    )
    ap.add_argument("--no-oe", action="store_true", help="Skip OE GPIO control.")
    ap.add_argument("--scan-only", action="store_true", help="Only locate the PCA9685 and exit.")
    ap.add_argument("--dry-run", action="store_true", help="Validate arguments and print timing only.")
    ap.add_argument("--quiet", action="store_true", help="Reduce per-ramp status output.")
    return ap.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not (1 <= int(args.channel) <= 16):
        raise SystemExit(f"[error] --channel must be in 1..16 (got {args.channel})")
    if not (float(args.min_us) < float(args.center_us) < float(args.max_us)):
        raise SystemExit("[error] expected --min-us < --center-us < --max-us")
    if float(args.freq) <= 0:
        raise SystemExit("[error] --freq must be positive")
    if float(args.rate_us_per_sec) <= 0:
        raise SystemExit("[error] --rate-us-per-sec must be positive")
    if float(args.step_us) <= 0:
        raise SystemExit("[error] --step-us must be positive")
    if int(args.loops) < 0:
        raise SystemExit("[error] --loops must be >= 0")
    for name in ("arm_seconds", "end_dwell", "center_dwell"):
        if float(getattr(args, name)) < 0:
            raise SystemExit(f"[error] --{name.replace('_', '-')} must be >= 0")
    if float(args.min_us) < 500 or float(args.max_us) > 2500:
        raise SystemExit("[error] pulse range looks unsafe; keep values between 500us and 2500us")


def load_pwm_helpers():
    try:
        from tools.direct_i2c_pwm_test import OEController, PCA9685, find_pca9685

        return OEController, PCA9685, find_pca9685
    except ModuleNotFoundError as first_error:
        if first_error.name not in {"tools", "tools.direct_i2c_pwm_test"}:
            raise SystemExit(
                f"[error] missing dependency {first_error.name!r}; install requirements on the Pi."
            ) from first_error
        try:
            from direct_i2c_pwm_test import OEController, PCA9685, find_pca9685

            return OEController, PCA9685, find_pca9685
        except ModuleNotFoundError as second_error:
            raise SystemExit(
                f"[error] missing dependency {second_error.name!r}; install requirements on the Pi."
            ) from second_error


def request_stop(_signum, _frame) -> None:
    global _stop_requested
    _stop_requested = True
    print("\n[STOP] Stop requested; returning to center and disabling outputs...")


def ease_fraction(x: float, curve: str) -> float:
    x = max(0.0, min(1.0, float(x)))
    if curve == "linear":
        return x
    return x * x * (3.0 - 2.0 * x)


def ramp_duration_s(start_us: float, end_us: float, rate_us_per_sec: float) -> float:
    return abs(float(end_us) - float(start_us)) / float(rate_us_per_sec)


def sleep_interruptible(seconds: float) -> None:
    deadline = time.monotonic() + max(0.0, float(seconds))
    while not _stop_requested:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.05, remaining))


def ramp_servo(
    set_pulse_us: Callable[[float], None],
    start_us: float,
    end_us: float,
    *,
    rate_us_per_sec: float,
    step_us: float,
    curve: str,
    quiet: bool,
) -> float:
    distance = abs(float(end_us) - float(start_us))
    if distance <= 1e-6:
        set_pulse_us(float(end_us))
        return float(end_us)

    steps = max(1, int(math.ceil(distance / float(step_us))))
    duration = ramp_duration_s(start_us, end_us, rate_us_per_sec)
    dt = max(MIN_UPDATE_S, duration / steps)
    last_report_bucket: Optional[int] = None

    if not quiet:
        print(
            f"[RAMP] {start_us:.1f}us -> {end_us:.1f}us "
            f"({steps} updates, about {steps * dt:.1f}s)"
        )

    current = float(start_us)
    for index in range(1, steps + 1):
        if _stop_requested:
            break
        fraction = ease_fraction(index / steps, curve)
        current = float(start_us) + (float(end_us) - float(start_us)) * fraction
        set_pulse_us(current)

        if not quiet:
            bucket = int(round(current / 100.0))
            if index == steps or bucket != last_report_bucket:
                print(f"  -> {current:.1f}us")
                last_report_bucket = bucket

        sleep_interruptible(dt)

    return current


def print_dry_run(args: argparse.Namespace) -> None:
    end_a, end_b = (
        (float(args.max_us), float(args.min_us))
        if args.first == "max"
        else (float(args.min_us), float(args.max_us))
    )
    legs: Tuple[Tuple[float, float], ...] = (
        (float(args.center_us), end_a),
        (end_a, end_b),
        (end_b, float(args.center_us)),
    )
    cycle_s = sum(ramp_duration_s(a, b, float(args.rate_us_per_sec)) for a, b in legs)
    cycle_s += 2.0 * float(args.end_dwell) + float(args.center_dwell)
    loop_text = "continuous" if int(args.loops) == 0 else str(int(args.loops))
    print("=== servo_sweep_test dry run ===")
    print(f"channel: {args.channel}")
    print(f"range: {args.min_us:.1f}us .. {args.center_us:.1f}us .. {args.max_us:.1f}us")
    print(f"rate: {args.rate_us_per_sec:.1f}us/s, step: {args.step_us:.1f}us, curve: {args.curve}")
    print(f"loops: {loop_text}")
    print(f"one cycle: about {cycle_s:.1f}s plus {args.arm_seconds:.1f}s initial center hold")


def main() -> int:
    args = parse_args()
    validate_args(args)

    if args.dry_run:
        print_dry_run(args)
        return 0

    OEController, PCA9685, find_pca9685 = load_pwm_helpers()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    channel = int(args.channel)
    center_us = float(args.center_us)
    bus_num, addr = find_pca9685(args.bus, args.addr)
    print(f"[OK] Found PCA9685 at /dev/i2c-{bus_num} addr 0x{addr:02X}")
    if args.scan_only:
        return 0

    oe = None
    pca = None

    def set_pulse_us(pulse_us: float) -> None:
        if pca is None:
            return
        pca.set_pulse_us_nav_channel(channel, float(pulse_us))

    def safe_shutdown() -> None:
        print("[SAFE] Returning channel to center and disabling outputs")
        try:
            set_pulse_us(center_us)
            time.sleep(0.2)
        except Exception:
            pass
        try:
            if oe is not None:
                oe.set_enabled(False)
        except Exception:
            pass
        try:
            if pca is not None:
                pca.close()
        except Exception:
            pass
        try:
            if oe is not None:
                oe.close()
        except Exception:
            pass

    try:
        if not args.no_oe:
            oe = OEController(args.oe_gpio, active_low=not args.oe_active_high)
            oe.set_enabled(False)
            print(f"[OK] OE prepared on BCM{args.oe_gpio} (enabled=False)")
        else:
            print("[WARN] --no-oe: skipping OE control")

        pca = PCA9685(bus_num=bus_num, address=addr, freq_hz=args.freq, osc_hz=args.osc_hz)
        pca.init()
        print(f"[OK] PCA9685 initialized at {args.freq:.2f} Hz")

        set_pulse_us(center_us)
        print(f"[OK] Channel {channel} set to center {center_us:.1f}us")

        if oe is not None:
            oe.set_enabled(True)
            print("[OK] Outputs enabled via OE")

        print(f"[ARM] Holding center for {args.arm_seconds:.1f}s")
        sleep_interruptible(float(args.arm_seconds))

        end_a, end_b = (
            (float(args.max_us), float(args.min_us))
            if args.first == "max"
            else (float(args.min_us), float(args.max_us))
        )

        current = center_us
        loop_index = 0
        while not _stop_requested and (int(args.loops) == 0 or loop_index < int(args.loops)):
            loop_index += 1
            loop_label = f"{loop_index}" if int(args.loops) == 0 else f"{loop_index}/{int(args.loops)}"
            print(f"[LOOP] {loop_label}")

            for target, dwell in (
                (end_a, float(args.end_dwell)),
                (end_b, float(args.end_dwell)),
                (center_us, float(args.center_dwell)),
            ):
                current = ramp_servo(
                    set_pulse_us,
                    current,
                    target,
                    rate_us_per_sec=float(args.rate_us_per_sec),
                    step_us=float(args.step_us),
                    curve=str(args.curve),
                    quiet=bool(args.quiet),
                )
                if _stop_requested:
                    break
                sleep_interruptible(dwell)

        print("[DONE] Sweep complete")
        return 0
    finally:
        safe_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
