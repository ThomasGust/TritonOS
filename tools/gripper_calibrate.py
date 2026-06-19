#!/usr/bin/env python3
"""Guided calibration for the differential wrist/arm servos.

The two BlueTrail servos form a 1:1 differential (see docs/MANIPULATOR_ARM.md):

    servo_left  pulse drives  (pitch + wrist)
    servo_right pulse drives  (pitch - wrist)

This wizard helps you measure, with the arm in front of you and a protractor:

  1. the neutral pulse (CENTER_US) that parks the arm at the pose you want at
     servo-center,
  2. the pulse-per-degree (US_PER_DEG) of output motion,
  3. the usable servo range in degrees (SERVO_RANGE_DEG),

then prints the `rov_config.py` values and the resulting full-wrist pitch band
for several candidate neutrals so you can choose where to bias range of motion.

SAFETY: this physically moves the arm. Keep it clear of obstructions and keep a
hand near Ctrl+C. Outputs are OE-gated and every move ramps smoothly.

Run on the Pi:

    ssh triton@tritonpi.local
    sudo .venv/bin/python -m tools.gripper_calibrate

Drive specific channels / a known center:

    sudo .venv/bin/python -m tools.gripper_calibrate --left-channel 10 --right-channel 11 --center-us 1500
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from typing import List, Optional, Tuple

DEFAULT_LEFT_CHANNEL = 10
DEFAULT_RIGHT_CHANNEL = 11
DEFAULT_CENTER_US = 1500.0
DEFAULT_FREQ_HZ = 50.0
DEFAULT_RATE_US_PER_SEC = 250.0   # smooth ramp speed
DEFAULT_STEP_US = 5.0
MIN_UPDATE_S = 0.02
SAFE_MIN_US = 500.0
SAFE_MAX_US = 2500.0

_stop_requested = False


def parse_addr(value: str) -> int:
    """Parse decimal or 0x-style integers (for --addr)."""

    return int(value, 0)


def parse_args() -> argparse.Namespace:
    """Parse channel, hardware, and timing options."""

    left_default = DEFAULT_LEFT_CHANNEL
    right_default = DEFAULT_RIGHT_CHANNEL
    center_default = DEFAULT_CENTER_US
    try:  # Prefer the live config so we calibrate the channels the ROV drives.
        from rov_config import (  # type: ignore
            GRIPPER_LEFT_PWM_CHANNEL,
            GRIPPER_RIGHT_PWM_CHANNEL,
            GRIPPER_SERVO_CENTER_US,
        )

        if GRIPPER_LEFT_PWM_CHANNEL is not None:
            left_default = int(GRIPPER_LEFT_PWM_CHANNEL)
        if GRIPPER_RIGHT_PWM_CHANNEL is not None:
            right_default = int(GRIPPER_RIGHT_PWM_CHANNEL)
        center_default = float(GRIPPER_SERVO_CENTER_US)
    except Exception:
        pass

    ap = argparse.ArgumentParser(
        description="Guided differential-wrist servo calibration.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--left-channel", type=int, default=left_default, help="Navigator channel (1..16) for servo_left.")
    ap.add_argument("--right-channel", type=int, default=right_default, help="Navigator channel (1..16) for servo_right.")
    ap.add_argument("--center-us", type=float, default=center_default, help="Starting neutral pulse for both servos.")
    ap.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ, help="PWM frequency in Hz.")
    ap.add_argument("--rate-us-per-sec", type=float, default=DEFAULT_RATE_US_PER_SEC, help="Ramp slew rate.")
    ap.add_argument("--step-us", type=float, default=DEFAULT_STEP_US, help="Ramp pulse increment.")
    ap.add_argument("--bus", type=int, default=4, help="Preferred I2C bus number.")
    ap.add_argument("--addr", type=parse_addr, default=0x40, help="Preferred PCA9685 I2C address.")
    ap.add_argument("--osc-hz", type=float, default=25_000_000.0, help="PCA9685 oscillator Hz.")
    ap.add_argument("--oe-gpio", type=int, default=26, help="BCM GPIO used for PCA9685 OE.")
    ap.add_argument("--oe-active-high", action="store_true", help="Treat OE as active-high (default active-low).")
    ap.add_argument("--no-oe", action="store_true", help="Skip OE GPIO control.")
    ap.add_argument("--servo-range-deg", type=float, default=70.0, help="Programmed servo half-range (deg).")
    ap.add_argument(
        "--align",
        action="store_true",
        help="Alignment mode: hold BOTH servos at center so you can mount the connector, then exit on Ctrl+C.",
    )
    return ap.parse_args()


def request_stop(_signum, _frame) -> None:
    """Signal handler that asks the wizard to exit cleanly."""

    global _stop_requested
    _stop_requested = True
    print("\n[STOP] Stop requested; returning to center and disabling outputs...")


def sleep_interruptible(seconds: float) -> None:
    """Sleep in short chunks so a stop request (Ctrl+C) is handled promptly."""

    deadline = time.monotonic() + max(0.0, float(seconds))
    while not _stop_requested:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.05, remaining))


def load_pwm_helpers():
    """Import the shared direct-I2C PWM helpers (module or script execution)."""

    try:
        from tools.direct_i2c_pwm_test import OEController, PCA9685, find_pca9685

        return OEController, PCA9685, find_pca9685
    except ModuleNotFoundError:
        from direct_i2c_pwm_test import OEController, PCA9685, find_pca9685

        return OEController, PCA9685, find_pca9685


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def prompt_float(msg: str) -> Optional[float]:
    """Read a float from the operator, or None on blank / EOF / stop."""

    if _stop_requested:
        return None
    try:
        raw = input(msg).strip()
    except (EOFError, KeyboardInterrupt):
        return None
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        print("  (not a number)")
        return prompt_float(msg)


def prompt_enter(msg: str) -> bool:
    """Wait for Enter; return False if the operator wants to stop."""

    if _stop_requested:
        return False
    try:
        input(msg)
    except (EOFError, KeyboardInterrupt):
        return False
    return not _stop_requested


class DualRamp:
    """Ramps two servo channels smoothly and tracks their current pulses."""

    def __init__(self, pca, left_ch: int, right_ch: int, rate_us_per_sec: float, step_us: float):
        self.pca = pca
        self.left_ch = int(left_ch)
        self.right_ch = int(right_ch)
        self.rate = float(rate_us_per_sec)
        self.step = float(step_us)
        self.cur_left = DEFAULT_CENTER_US
        self.cur_right = DEFAULT_CENTER_US

    def _write(self, left_us: float, right_us: float) -> None:
        self.pca.set_pulse_us_nav_channel(self.left_ch, float(left_us))
        self.pca.set_pulse_us_nav_channel(self.right_ch, float(right_us))
        self.cur_left = float(left_us)
        self.cur_right = float(right_us)

    def goto(self, left_us: float, right_us: float) -> None:
        """Ramp both channels to the target pulses with a smooth profile."""

        left_us = clamp(left_us, SAFE_MIN_US, SAFE_MAX_US)
        right_us = clamp(right_us, SAFE_MIN_US, SAFE_MAX_US)
        start_l, start_r = self.cur_left, self.cur_right
        distance = max(abs(left_us - start_l), abs(right_us - start_r))
        if distance <= 1e-6:
            self._write(left_us, right_us)
            return
        steps = max(1, int(round(distance / self.step)))
        dt = max(MIN_UPDATE_S, (distance / self.rate) / steps)
        for i in range(1, steps + 1):
            if _stop_requested:
                break
            f = i / steps
            f = f * f * (3.0 - 2.0 * f)  # smoothstep
            self._write(start_l + (left_us - start_l) * f, start_r + (right_us - start_r) * f)
            time.sleep(dt)


def full_wrist_band(neutral: float, servo_range: float, pitch_span: float, wrist_half: float) -> Tuple[float, float]:
    """Pitch range (deg) over which the full wrist span is available.

    Full wrist needs |dPitch| <= servo_range - wrist_half. Convert that to the
    absolute pitch angles, clipped to [0, pitch_span].
    """

    room = max(0.0, servo_range - wrist_half)
    lo = clamp(neutral - room, 0.0, pitch_span)
    hi = clamp(neutral + room, 0.0, pitch_span)
    return lo, hi


def recommend(center_us: float, us_per_deg: float, servo_range_deg: float, pitch_span: float, wrist_span: float) -> None:
    """Print the rov_config block and full-wrist bands for candidate neutrals."""

    wrist_half = wrist_span / 2.0
    min_us = int(round(center_us - servo_range_deg * us_per_deg))
    max_us = int(round(center_us + servo_range_deg * us_per_deg))
    print("\n================ RESULTS ================")
    print("Paste into rov_config.py (section 9, differential wrist/arm):\n")
    print(f"GRIPPER_SERVO_RANGE_DEG = {servo_range_deg:.1f}")
    print(f"GRIPPER_PITCH_SPAN_DEG = {pitch_span:.1f}")
    print(f"GRIPPER_WRIST_SPAN_DEG = {wrist_span:.1f}")
    print(f"GRIPPER_SERVO_CENTER_US = {int(round(center_us))}")
    print(f"GRIPPER_US_PER_DEG = {us_per_deg:.4f}")
    print(f"# -> derived GRIPPER_SERVO_MIN_US = {min_us}, GRIPPER_SERVO_MAX_US = {max_us}")
    print("\nFull-wrist pitch band vs. GRIPPER_PITCH_NEUTRAL_DEG choice:")
    print("  neutral    full-wrist band       wrist range at pitch=90")
    for neutral in (45.0, 55.0, 62.0, servo_range_deg):
        if neutral > pitch_span:
            continue
        lo, hi = full_wrist_band(neutral, servo_range_deg, pitch_span, wrist_half)
        room_at_90 = max(0.0, servo_range_deg - abs(pitch_span - neutral))
        wrist_at_90 = min(wrist_half, room_at_90) * 2.0
        print(f"  {neutral:6.1f}   {lo:5.1f}..{hi:5.1f} deg      ~{wrist_at_90:4.1f} deg")
    if servo_range_deg >= (pitch_span / 2.0 + wrist_half):
        print("\n  This range covers FULL wrist at FULL pitch everywhere (no compromise).")
    print("=========================================\n")


def alignment_hold(ramp: "DualRamp", center_us: float) -> None:
    """Drive both servos to center and hold so the operator can mount the connector.

    With both servos at their electrical center, the differential is at its neutral
    pose. Mount the connector + arm here so that this pose equals the chosen
    GRIPPER_PITCH_NEUTRAL_DEG (pitch) and a centered wrist; the +/-70 deg of each
    servo then spreads symmetrically about that neutral.
    """

    ramp.goto(center_us, center_us)
    print("\n================ ALIGNMENT HOLD ================")
    print(f"Both servos are driven to CENTER ({center_us:.0f}us) and held.")
    print("Mount the connector + arm so that at THIS pose the arm sits at:")
    print("  - PITCH = your chosen neutral (GRIPPER_PITCH_NEUTRAL_DEG), e.g. 45 deg from flat")
    print("  - WRIST = centered (mid of its 0..90 deg roll)")
    print("Make sure GRIPPER_PITCH_NEUTRAL_DEG in rov_config.py matches the mounted pitch.")
    print("Press Ctrl+C when the connector is mounted to release and exit.")
    print("===============================================\n")
    while not _stop_requested:
        # The PCA9685 keeps emitting the last pulse; re-assert periodically as a
        # safety net against any transient and to keep the process responsive.
        ramp.goto(center_us, center_us)
        sleep_interruptible(0.5)


def main() -> int:
    """Run the guided differential-wrist calibration wizard."""

    args = parse_args()
    for ch in (args.left_channel, args.right_channel):
        if not (1 <= int(ch) <= 16):
            raise SystemExit(f"[error] channel {ch} must be 1..16")
    if not (SAFE_MIN_US < float(args.center_us) < SAFE_MAX_US):
        raise SystemExit("[error] --center-us must be between 500 and 2500")

    OEController, PCA9685, find_pca9685 = load_pwm_helpers()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    bus_num, addr = find_pca9685(args.bus, args.addr)
    print(f"[OK] Found PCA9685 at /dev/i2c-{bus_num} addr 0x{addr:02X}")

    oe = None
    pca = None
    center_us = float(args.center_us)
    pitch_span = 90.0
    wrist_span = 90.0

    def safe_shutdown() -> None:
        print("[SAFE] Returning servos to center and disabling outputs")
        try:
            if pca is not None:
                pca.set_pulse_us_nav_channel(int(args.left_channel), center_us)
                pca.set_pulse_us_nav_channel(int(args.right_channel), center_us)
                time.sleep(0.3)
        except Exception:
            pass
        for obj in (oe, pca):
            try:
                if obj is not None and hasattr(obj, "set_enabled"):
                    obj.set_enabled(False)
            except Exception:
                pass
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass

    try:
        if not args.no_oe:
            oe = OEController(args.oe_gpio, active_low=not args.oe_active_high)
            oe.set_enabled(False)

        pca = PCA9685(bus_num=bus_num, address=addr, freq_hz=args.freq, osc_hz=args.osc_hz)
        pca.init()
        ramp = DualRamp(pca, args.left_channel, args.right_channel, args.rate_us_per_sec, args.step_us)
        ramp.cur_left = ramp.cur_right = center_us
        pca.set_pulse_us_nav_channel(int(args.left_channel), center_us)
        pca.set_pulse_us_nav_channel(int(args.right_channel), center_us)

        print(f"\nChannels: servo_left=Ch{args.left_channel}  servo_right=Ch{args.right_channel}")
        print("Confirm these are the differential wrist/arm servos (not thrusters!).")
        if not prompt_enter("Press Enter to ENABLE outputs at center (Ctrl+C to abort)... "):
            return 0
        if oe is not None:
            oe.set_enabled(True)
        print("[OK] Outputs enabled at center.\n")

        if args.align:
            alignment_hold(ramp, center_us)
            return 0

        # --- Step 1: choose the neutral pose --------------------------------
        print("STEP 1 — Neutral pose. Jog both servos together until the arm sits at the")
        print("pose you want at servo-center (e.g. pitch ~mid-arc, wrist centered).")
        while not _stop_requested:
            delta = prompt_float(f"  current center={center_us:.0f}us. Enter +/- us to jog (blank=done): ")
            if delta is None:
                break
            center_us = clamp(center_us + delta, SAFE_MIN_US, SAFE_MAX_US)
            ramp.goto(center_us, center_us)
        print(f"[OK] CENTER_US = {center_us:.0f}\n")

        # --- Step 2: pulse-per-degree (pitch axis) --------------------------
        print("STEP 2 — Pulse-per-degree. We move both servos the SAME way (pure pitch).")
        jog = prompt_float("  Enter a pitch test jog in us (e.g. 200): ") or 200.0
        jog = abs(jog)
        ramp.goto(center_us + jog, center_us + jog)
        d_plus = prompt_float(f"  Measured pitch change from center for +{jog:.0f}us (deg): ")
        ramp.goto(center_us, center_us)
        ramp.goto(center_us - jog, center_us - jog)
        d_minus = prompt_float(f"  Measured pitch change from center for -{jog:.0f}us (deg, magnitude): ")
        ramp.goto(center_us, center_us)
        degs = [abs(d) for d in (d_plus, d_minus) if d not in (None, 0.0)]
        if not degs:
            print("[warn] no angle entered; cannot compute US_PER_DEG. Aborting solve.")
            return 0
        us_per_deg = jog / (sum(degs) / len(degs))
        print(f"[OK] US_PER_DEG ~= {us_per_deg:.3f}\n")

        # --- Step 3: confirm / refine servo range ---------------------------
        servo_range_deg = float(args.servo_range_deg)
        print(f"STEP 3 — Servo range. Programmed half-range is assumed {servo_range_deg:.0f} deg.")
        meas = prompt_float("  Optional: jog to the mechanical limit and enter pitch there (deg, blank=keep): ")
        if meas:
            servo_range_deg = abs(meas)
        print(f"[OK] SERVO_RANGE_DEG = {servo_range_deg:.1f}\n")

        # --- Step 4: optional wrist sanity check ----------------------------
        print("STEP 4 (optional) — Wrist check. We move servos OPPOSITE (pure wrist).")
        if prompt_enter("  Press Enter to drive a small wrist move, or Ctrl+C to skip... "):
            ramp.goto(center_us + jog, center_us - jog)
            prompt_enter("  Confirm the WRIST rolled (not pitch). Press Enter to recenter... ")
            ramp.goto(center_us, center_us)

        recommend(center_us, us_per_deg, servo_range_deg, pitch_span, wrist_span)
        return 0
    finally:
        safe_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
