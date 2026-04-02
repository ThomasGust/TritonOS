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

import argparse
import time
import importlib.metadata as md

from utils.navigator_import import import_navigator_module

nav = import_navigator_module()


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
TEST_THROTTLE = 0.30   # 20% of full range
RAMP_STEP = 0.02
RAMP_DT = 0.15

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Navigator native motor test (direct PWM)."
    )
    parser.add_argument(
        "-c",
        "--channel",
        type=int,
        default=DEFAULT_PWM_CHANNEL,
        help="Navigator PWM output channel (1..16). Default: %(default)s",
    )
    return parser.parse_args()


def us_to_count(pulse_us: float, freq_hz: float) -> int:
    """Convert pulse width in microseconds to PCA9685 OFF count [0..4095]."""
    period_us = 1_000_000.0 / freq_hz
    value = round(4095.0 * (pulse_us / period_us))
    return max(0, min(4095, int(value)))


def throttle_to_us(throttle: float) -> float:
    """Map throttle [-1..+1] to pulse width [1100..1900] with stop at 1500."""
    throttle = max(-1.0, min(1.0, float(throttle)))
    pulse = STOP_US + 400.0 * throttle  # 1500 +/- 400 => 1100..1900
    if abs(pulse - STOP_US) < DEADBAND_US:
        pulse = STOP_US
    return pulse


def set_user_leds(on: bool) -> None:
    # Uses onboard user LEDs (Blue, Green, Red)
    if hasattr(nav, "set_led_all"):
        nav.set_led_all(bool(on))


def try_set_neopixel_rgb(rgb=None, rgbw=None) -> None:
    """Optional NeoPixel feedback; safe to ignore if none connected."""
    try:
        if rgbw is not None and hasattr(nav, "set_neopixel_rgbw"):
            nav.set_neopixel_rgbw([rgbw])
        elif rgb is not None and hasattr(nav, "set_neopixel"):
            nav.set_neopixel([rgb])
    except Exception as e:
        print(f"[warn] neopixel call failed (ok if none connected): {e}")


def set_pwm_pulse_us(channel: int, pulse_us: float) -> int:
    count = us_to_count(pulse_us, PWM_FREQ_HZ)
    nav.set_pwm_channel_value(int(channel), int(count))
    return count


def main() -> None:
    args = parse_args()

    ch = int(args.channel)
    if not (0 <= ch <= 16):
        raise SystemExit(f"[error] --channel must be in 0..16 (got {ch})")

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
    if hasattr(nav, "set_rgb_led_strip_size"):
        nav.set_rgb_led_strip_size(1)

    if hasattr(nav, "Raspberry") and hasattr(nav, "set_raspberry_pi_version"):
        try:
            nav.set_raspberry_pi_version(nav.Raspberry.Pi4)
        except Exception:
            pass

    if hasattr(nav, "init"):
        print("[step] init()")
        nav.init()

    set_user_leds(True)
    try_set_neopixel_rgb(rgb=[0, 0, 50])  # dim blue

    print(f"[step] set PWM freq to {PWM_FREQ_HZ} Hz")
    nav.set_pwm_freq_hz(float(PWM_FREQ_HZ))

    print("[info] using integer PWM channel:", ch)

    print("[step] send STOP (1500us) and enable PWM (ESC init)")
    stop_count = set_pwm_pulse_us(ch, STOP_US)
    nav.set_pwm_enable(True)
    print(f"  STOP_US={STOP_US} => count={stop_count}  (expect ESC init tones after a few seconds)")

    # Hold neutral for ESC initialization
    try_set_neopixel_rgb(rgb=[50, 50, 0])
    time.sleep(3.0)

    # Gentle forward ramp
    print(f"[step] ramp forward to {TEST_THROTTLE*100:.0f}%")
    try_set_neopixel_rgb(rgb=[0, 50, 0])
    t = 0.0
    while t < TEST_THROTTLE + 1e-6:
        pulse = throttle_to_us(t)
        count = set_pwm_pulse_us(ch, pulse)
        print(f"  throttle={t:+.2f}  pulse={pulse:.0f}us  count={count}")
        time.sleep(RAMP_DT)
        t += RAMP_STEP

    print("[step] back to STOP")
    try_set_neopixel_rgb(rgb=[0, 0, 50])
    set_pwm_pulse_us(ch, STOP_US)
    time.sleep(1.0)

    # Gentle reverse ramp
    print(f"[step] ramp reverse to {-TEST_THROTTLE*100:.0f}%")
    try_set_neopixel_rgb(rgb=[50, 0, 0])
    t = 0.0
    while t > -TEST_THROTTLE - 1e-6:
        pulse = throttle_to_us(t)
        count = set_pwm_pulse_us(ch, pulse)
        print(f"  throttle={t:+.2f}  pulse={pulse:.0f}us  count={count}")
        time.sleep(RAMP_DT)
        t -= RAMP_STEP

    print("[step] STOP + disable PWM")
    set_pwm_pulse_us(ch, STOP_US)
    time.sleep(0.5)
    nav.set_pwm_enable(False)
    set_user_leds(False)
    try_set_neopixel_rgb(rgb=[0, 0, 0])
    print("=== done ===")


if __name__ == "__main__":
    main()
