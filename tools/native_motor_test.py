#!/usr/bin/env python3
"""Native Navigator motor test (direct PWM).

This tool directly exercises a single Navigator PWM output using the official
``bluerobotics_navigator`` Python bindings. It is intentionally "close to the
metal" and does *not* use TritonOS control code.

Run on the Pi (default channel 1):

  sudo .venv/bin/python -m tools.native_motor_test

Specify a channel:

  sudo .venv/bin/python -m tools.native_motor_test --channel 4

If the motor spins here but not in the full system, the issue is almost
certainly in the control/mixing/thruster-mapping pipeline (not wiring).
"""

from __future__ import annotations

import importlib.metadata as md
import argparse
import math
import time

from utils.navigator_import import import_navigator_module

nav = None


# ----------------- Defaults / Config -----------------
# Navigator PWM outputs are numbered 1..16.
DEFAULT_PWM_CHANNEL = 1
PWM_FREQ_HZ = 50.0

# Basic ESC pulse widths (microseconds)
STOP_US = 1500
MAX_FWD_US = 1900
MAX_REV_US = 1100
DEADBAND_US = 25

# Keep the first test gentle
TEST_THROTTLE = 0.30
RAMP_STEP = 0.02
RAMP_DT = 0.15
INIT_SECONDS = 3.0
BETWEEN_SECONDS = 1.0
FINAL_STOP_SECONDS = 0.5
PEAK_HOLD_SECONDS = 0.0

def parse_args() -> argparse.Namespace:
    """Parse single-channel Navigator motor test options."""

    parser = argparse.ArgumentParser(
        description="Navigator native motor test (direct PWM).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-c",
        "--channel",
        type=int,
        default=DEFAULT_PWM_CHANNEL,
        help="Navigator PWM output channel/integer accepted by the Navigator binding.",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=PWM_FREQ_HZ,
        help="PWM frequency in Hz.",
    )
    parser.add_argument(
        "--stop-us",
        type=float,
        default=STOP_US,
        help="Stop/neutral pulse width in microseconds.",
    )
    parser.add_argument(
        "--max-fwd-us",
        type=float,
        default=MAX_FWD_US,
        help="Pulse width at +1.0 throttle.",
    )
    parser.add_argument(
        "--max-rev-us",
        type=float,
        default=MAX_REV_US,
        help="Pulse width at -1.0 throttle.",
    )
    parser.add_argument(
        "--deadband-us",
        type=float,
        default=DEADBAND_US,
        help="Pulse widths this close to stop are snapped to stop.",
    )
    parser.add_argument(
        "--throttle",
        type=float,
        default=TEST_THROTTLE,
        help="Default absolute throttle target for forward/reverse ramps.",
    )
    parser.add_argument(
        "--forward-throttle",
        type=float,
        default=None,
        help="Override forward throttle target, 0..1. Uses --throttle when omitted.",
    )
    parser.add_argument(
        "--reverse-throttle",
        type=float,
        default=None,
        help="Override reverse throttle target, 0..1. Uses --throttle when omitted.",
    )
    parser.add_argument(
        "--direction",
        choices=("both", "forward", "reverse"),
        default="both",
        help="Which direction(s) to test.",
    )
    parser.add_argument(
        "--ramp-step",
        type=float,
        default=RAMP_STEP,
        help="Throttle increment between ramp updates.",
    )
    parser.add_argument(
        "--ramp-dt",
        type=float,
        default=RAMP_DT,
        help="Seconds between ramp updates.",
    )
    parser.add_argument(
        "--init-seconds",
        type=float,
        default=INIT_SECONDS,
        help="Seconds to hold stop after enabling PWM before motion.",
    )
    parser.add_argument(
        "--between-seconds",
        type=float,
        default=BETWEEN_SECONDS,
        help="Seconds to hold stop between forward and reverse ramps.",
    )
    parser.add_argument(
        "--final-stop-seconds",
        type=float,
        default=FINAL_STOP_SECONDS,
        help="Seconds to hold stop before disabling PWM at the end.",
    )
    parser.add_argument(
        "--peak-hold",
        type=float,
        default=PEAK_HOLD_SECONDS,
        help="Seconds to hold at the end of each ramp before stopping/reversing.",
    )
    parser.add_argument(
        "--neopixel-size",
        type=int,
        default=1,
        help="NeoPixel strip size to configure before init. Use 0 to skip.",
    )
    parser.add_argument(
        "--no-leds",
        action="store_true",
        help="Do not use Navigator user LEDs for status.",
    )
    parser.add_argument(
        "--no-neopixel",
        action="store_true",
        help="Do not use NeoPixel status colors.",
    )
    parser.add_argument(
        "--no-init",
        action="store_true",
        help="Skip nav.init() even if the binding exposes it.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    """Validate command-line values before any hardware is touched."""

    if not (0 <= int(args.channel) <= 16):
        raise SystemExit(f"[error] --channel must be in 0..16 (got {args.channel})")
    if float(args.freq) <= 0:
        raise SystemExit("[error] --freq must be positive")
    if not (float(args.max_rev_us) < float(args.stop_us) < float(args.max_fwd_us)):
        raise SystemExit("[error] expected --max-rev-us < --stop-us < --max-fwd-us")
    if float(args.max_rev_us) < 500 or float(args.max_fwd_us) > 2500:
        raise SystemExit("[error] pulse range looks unsafe; keep values between 500us and 2500us")
    if float(args.deadband_us) < 0:
        raise SystemExit("[error] --deadband-us must be >= 0")
    if float(args.ramp_step) <= 0:
        raise SystemExit("[error] --ramp-step must be positive")
    if float(args.ramp_dt) < 0:
        raise SystemExit("[error] --ramp-dt must be >= 0")
    for name in ("throttle", "forward_throttle", "reverse_throttle"):
        value = getattr(args, name)
        if value is None:
            continue
        if not (0.0 <= float(value) <= 1.0):
            raise SystemExit(f"[error] --{name.replace('_', '-')} must be in 0..1")
    for name in ("init_seconds", "between_seconds", "final_stop_seconds", "peak_hold"):
        if float(getattr(args, name)) < 0:
            raise SystemExit(f"[error] --{name.replace('_', '-')} must be >= 0")
    if int(args.neopixel_size) < 0:
        raise SystemExit("[error] --neopixel-size must be >= 0")


def us_to_count(pulse_us: float, freq_hz: float) -> int:
    """Convert pulse width in microseconds to PCA9685 OFF count [0..4095]."""
    period_us = 1_000_000.0 / freq_hz
    value = round(4095.0 * (pulse_us / period_us))
    return max(0, min(4095, int(value)))


def throttle_to_us(
    throttle: float,
    *,
    stop_us: float,
    max_fwd_us: float,
    max_rev_us: float,
    deadband_us: float,
) -> float:
    """Map throttle [-1..+1] to a configured pulse-width range."""
    throttle = max(-1.0, min(1.0, float(throttle)))
    if throttle >= 0.0:
        pulse = stop_us + (max_fwd_us - stop_us) * throttle
    else:
        pulse = stop_us + (stop_us - max_rev_us) * throttle
    if abs(pulse - stop_us) < deadband_us:
        pulse = stop_us
    return pulse


def set_user_leds(on: bool) -> None:
    """Set Navigator user LEDs when the binding exposes that API."""

    # Uses onboard user LEDs (Blue, Green, Red)
    if nav is not None and hasattr(nav, "set_led_all"):
        nav.set_led_all(bool(on))


def try_set_neopixel_rgb(rgb=None, rgbw=None) -> None:
    """Optional NeoPixel feedback; safe to ignore if none connected."""
    try:
        if nav is not None and rgbw is not None and hasattr(nav, "set_neopixel_rgbw"):
            nav.set_neopixel_rgbw([rgbw])
        elif nav is not None and rgb is not None and hasattr(nav, "set_neopixel"):
            nav.set_neopixel([rgb])
    except Exception as e:
        print(f"[warn] neopixel call failed (ok if none connected): {e}")


def set_pwm_pulse_us(channel: int, pulse_us: float, freq_hz: float) -> int:
    """Write one PWM channel and return the computed PCA9685 count."""

    if nav is None:
        raise RuntimeError("Navigator module has not been initialized")
    count = us_to_count(pulse_us, freq_hz)
    nav.set_pwm_channel_value(int(channel), int(count))
    return count


def throttle_steps(target: float, step: float):
    """Yield ramp values from neutral to a target normalized throttle."""

    steps = max(1, int(math.ceil(abs(float(target)) / float(step))))
    for index in range(steps + 1):
        yield float(target) * (index / steps)


def set_neopixel_status(args: argparse.Namespace, rgb=None, rgbw=None) -> None:
    """Apply optional NeoPixel status feedback unless disabled."""

    if args.no_neopixel:
        return
    try_set_neopixel_rgb(rgb=rgb, rgbw=rgbw)


def run_ramp(label: str, target: float, rgb, ch: int, args: argparse.Namespace) -> None:
    """Ramp one channel toward a throttle target with printed telemetry."""

    print(f"[step] ramp {label} to {target * 100:.0f}%")
    set_neopixel_status(args, rgb=rgb)
    last_count = None
    last_pulse = None
    for throttle in throttle_steps(target, float(args.ramp_step)):
        pulse = throttle_to_us(
            throttle,
            stop_us=float(args.stop_us),
            max_fwd_us=float(args.max_fwd_us),
            max_rev_us=float(args.max_rev_us),
            deadband_us=float(args.deadband_us),
        )
        count = set_pwm_pulse_us(ch, pulse, float(args.freq))
        last_count = count
        last_pulse = pulse
        print(f"  throttle={throttle:+.2f}  pulse={pulse:.0f}us  count={count}")
        time.sleep(float(args.ramp_dt))

    if float(args.peak_hold) > 0:
        print(
            f"  holding peak throttle={target:+.2f} "
            f"pulse={last_pulse:.0f}us count={last_count} for {args.peak_hold:.2f}s"
        )
        time.sleep(float(args.peak_hold))


def safe_stop(ch: int, args: argparse.Namespace) -> None:
    """Return the channel to neutral, disable PWM, and clear indicators."""

    print("[step] STOP + disable PWM")
    try:
        set_pwm_pulse_us(ch, float(args.stop_us), float(args.freq))
        time.sleep(float(args.final_stop_seconds))
    except Exception as e:
        print(f"[warn] failed to send stop pulse: {e}")

    try:
        if nav is not None and hasattr(nav, "set_pwm_enable"):
            nav.set_pwm_enable(False)
    except Exception as e:
        print(f"[warn] failed to disable PWM: {e}")

    if not args.no_leds:
        try:
            set_user_leds(False)
        except Exception as e:
            print(f"[warn] failed to clear user LEDs: {e}")
    set_neopixel_status(args, rgb=[0, 0, 0])


def main() -> None:
    """Run the single-channel native Navigator motor test."""

    global nav
    args = parse_args()
    validate_args(args)
    nav = import_navigator_module()

    ch = int(args.channel)
    forward_throttle = float(args.forward_throttle if args.forward_throttle is not None else args.throttle)
    reverse_throttle = float(args.reverse_throttle if args.reverse_throttle is not None else args.throttle)

    print("=== Navigator native motor test ===")

    # Version/debug
    for dist in ("bluerobotics_navigator", "bluerobotics-navigator"):
        try:
            print(f"[info] {dist} version:", md.version(dist))
        except Exception:
            pass

    print("[info] has PwmChannel:", hasattr(nav, "PwmChannel"))
    print("[info] available PWM-related symbols:", [x for x in dir(nav) if "pwm" in x.lower()])

    # Optional: NeoPixel strip size should be set before init (if you use it).
    if int(args.neopixel_size) > 0 and not args.no_neopixel and hasattr(nav, "set_rgb_led_strip_size"):
        nav.set_rgb_led_strip_size(int(args.neopixel_size))

    if hasattr(nav, "Raspberry") and hasattr(nav, "set_raspberry_pi_version"):
        try:
            nav.set_raspberry_pi_version(nav.Raspberry.Pi4)
        except Exception:
            pass

    if not args.no_init and hasattr(nav, "init"):
        print("[step] init()")
        nav.init()

    try:
        if not args.no_leds:
            set_user_leds(True)
        set_neopixel_status(args, rgb=[0, 0, 50])  # dim blue

        print(f"[step] set PWM freq to {args.freq} Hz")
        nav.set_pwm_freq_hz(float(args.freq))

        print("[info] using integer PWM channel:", ch)

        print(f"[step] send STOP ({args.stop_us:.0f}us) and enable PWM (ESC init)")
        stop_count = set_pwm_pulse_us(ch, float(args.stop_us), float(args.freq))
        nav.set_pwm_enable(True)
        print(f"  stop_us={args.stop_us:.0f} => count={stop_count}  (expect ESC init tones after a few seconds)")

        # Hold neutral for ESC initialization
        set_neopixel_status(args, rgb=[50, 50, 0])
        time.sleep(float(args.init_seconds))

        if args.direction in ("both", "forward"):
            run_ramp("forward", forward_throttle, [0, 50, 0], ch, args)

        if args.direction == "both":
            print("[step] back to STOP")
            set_neopixel_status(args, rgb=[0, 0, 50])
            set_pwm_pulse_us(ch, float(args.stop_us), float(args.freq))
            time.sleep(float(args.between_seconds))

        if args.direction in ("both", "reverse"):
            run_ramp("reverse", -reverse_throttle, [50, 0, 0], ch, args)
    except KeyboardInterrupt:
        print("\n[warn] interrupted")
    finally:
        safe_stop(ch, args)
    print("=== done ===")


if __name__ == "__main__":
    main()
