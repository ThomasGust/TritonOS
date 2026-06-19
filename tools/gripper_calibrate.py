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
    ap.add_argument(
        "--check-axes",
        action="store_true",
        help="Drive common-mode then differential-mode moves to learn how the differential maps pitch/roll.",
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


class PwmBackend:
    """PWM output that prefers the same backend the ROV uses.

    On the Navigator board the PWM output-enable is owned by the
    ``bluerobotics_navigator`` library (``set_pwm_enable``), NOT the PCA9685 OE
    GPIO. If we only toggle the OE GPIO, the chip emits the correct pulse but the
    output stage stays gated and the servos go limp. So prefer ``NavigatorPWM``;
    fall back to direct PCA9685 + OE GPIO only when the bindings are unavailable.

    Channels are given as 1-based config/Navigator numbers (e.g. 10, 11); for the
    Navigator integer API they are converted to the binding's base, matching
    ``motion.pwm.ThrustWriter``.
    """

    def __init__(self, args):
        self.name = "unknown"
        self._nav = None
        self._lib_base = 1
        self._pca = None
        self._oe = None

        try:
            from motion import pwm as motion_pwm

            self._nav = motion_pwm.NavigatorPWM(freq_hz=float(args.freq))
            self._lib_base = int(self._nav.lib_base)
            self.name = "navigator"
            print(f"[OK] PWM backend = navigator (lib_base={self._lib_base})")
            return
        except Exception as e:
            print(f"[info] Navigator backend unavailable ({e}); using direct I2C + OE GPIO.")

        OEController, PCA9685, find_pca9685 = load_pwm_helpers()
        bus_num, addr = find_pca9685(args.bus, args.addr)
        print(f"[OK] Found PCA9685 at /dev/i2c-{bus_num} addr 0x{addr:02X}")
        self._pca = PCA9685(bus_num=bus_num, address=addr, freq_hz=float(args.freq), osc_hz=float(args.osc_hz))
        self._pca.init()
        if not args.no_oe:
            self._oe = OEController(args.oe_gpio, active_low=not args.oe_active_high)
            self._oe.set_enabled(False)
        self.name = "direct_i2c"
        print("[OK] PWM backend = direct_i2c")

    def enable(self, state: bool) -> None:
        if self._nav is not None:
            self._nav.enable(bool(state))
        elif self._oe is not None:
            self._oe.set_enabled(bool(state))

    def set_us(self, nav_channel_one_based: int, pulse_us: float) -> None:
        if self._nav is not None:
            lib_ch = int(nav_channel_one_based) - 1 + self._lib_base
            self._nav.set_servo_us(lib_ch, float(pulse_us))
        else:
            self._pca.set_pulse_us_nav_channel(int(nav_channel_one_based), float(pulse_us))

    def close(self) -> None:
        for obj in (self._oe, self._pca):
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass


class DualRamp:
    """Ramps two servo channels smoothly and tracks their current pulses."""

    def __init__(self, backend: "PwmBackend", left_ch: int, right_ch: int, rate_us_per_sec: float, step_us: float):
        self.backend = backend
        self.left_ch = int(left_ch)
        self.right_ch = int(right_ch)
        self.rate = float(rate_us_per_sec)
        self.step = float(step_us)
        self.cur_left = DEFAULT_CENTER_US
        self.cur_right = DEFAULT_CENTER_US

    def _write(self, left_us: float, right_us: float) -> None:
        self.backend.set_us(self.left_ch, float(left_us))
        self.backend.set_us(self.right_ch, float(right_us))
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


def ask_motion(question: str) -> str:
    """Ask the operator whether a move PITCHed or ROLLed the arm."""

    while not _stop_requested:
        print(question)
        try:
            raw = input("  type 'pitch' or 'roll' (blank to skip): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return ""
        if raw in ("p", "pitch"):
            return "pitch"
        if raw in ("r", "roll"):
            return "roll"
        if not raw:
            return ""
        print("  (please type pitch or roll)")
    return ""


def check_axes(ramp: "DualRamp", center_us: float) -> None:
    """Drive common- then differential-mode moves and recommend the invert config.

    With a facing-servo bevel differential, both servos commanded the SAME way roll
    the output and OPPOSITE commands pitch it -- the reverse of the mixer default.
    """

    jog = 250.0
    print("\n================ AXIS CHECK ================")
    print("Watch the ARM. We drive two test moves so you can see how the gears map motion.")

    if not prompt_enter("Press Enter to drive BOTH servos the SAME direction... "):
        ramp.goto(center_us, center_us)
        return
    ramp.goto(center_us + jog, center_us + jog)
    obs_common = ask_motion("  BOTH servos same way -> did the arm PITCH (tilt) or ROLL (twist)?")
    ramp.goto(center_us, center_us)

    if not prompt_enter("Press Enter to drive the servos OPPOSITE directions... "):
        ramp.goto(center_us, center_us)
        return
    ramp.goto(center_us + jog, center_us - jog)
    obs_diff = ask_motion("  Servos OPPOSITE -> did the arm PITCH or ROLL?")
    ramp.goto(center_us, center_us)

    print("\n--- recommendation (edit rov_config.py) ---")
    if obs_common == "roll" and obs_diff == "pitch":
        print("Facing-servo geometry confirmed (both-same -> ROLL). Invert ONE servo:")
        print("    GRIPPER_RIGHT_INVERT = -1.0")
        print("Then re-run with the arm and flip GRIPPER_PITCH_INVERT / GRIPPER_YAW_INVERT")
        print("if pitch or roll moves the wrong direction.")
    elif obs_common == "pitch" and obs_diff == "roll":
        print("both-same -> PITCH: the mixer mapping is already correct (no servo invert).")
        print("If a single axis is reversed, flip GRIPPER_PITCH_INVERT or GRIPPER_YAW_INVERT.")
    else:
        print(f"Observed common={obs_common or '?'} , differential={obs_diff or '?'}.")
        print("If the two moves looked the same, the gears may not be meshing as a clean")
        print("differential -- check the mechanism before tuning software.")
    print("===========================================\n")


def main() -> int:
    """Run the guided differential-wrist calibration wizard."""

    args = parse_args()
    for ch in (args.left_channel, args.right_channel):
        if not (1 <= int(ch) <= 16):
            raise SystemExit(f"[error] channel {ch} must be 1..16")
    if not (SAFE_MIN_US < float(args.center_us) < SAFE_MAX_US):
        raise SystemExit("[error] --center-us must be between 500 and 2500")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    backend = PwmBackend(args)

    center_us = float(args.center_us)
    pitch_span = 90.0
    wrist_span = 90.0

    def safe_shutdown() -> None:
        print("[SAFE] Returning servos to center and disabling outputs")
        try:
            backend.set_us(int(args.left_channel), center_us)
            backend.set_us(int(args.right_channel), center_us)
            time.sleep(0.3)
        except Exception:
            pass
        try:
            backend.enable(False)
        except Exception:
            pass
        try:
            backend.close()
        except Exception:
            pass

    try:
        ramp = DualRamp(backend, args.left_channel, args.right_channel, args.rate_us_per_sec, args.step_us)
        ramp.cur_left = ramp.cur_right = center_us
        backend.set_us(int(args.left_channel), center_us)
        backend.set_us(int(args.right_channel), center_us)

        print(f"\nChannels: servo_left=Ch{args.left_channel}  servo_right=Ch{args.right_channel}")
        print("Confirm these are the differential wrist/arm servos (not thrusters!).")
        print("NOTE: enabling outputs enables ALL PWM channels; keep the ROV out of water.")
        if not prompt_enter("Press Enter to ENABLE outputs at center (Ctrl+C to abort)... "):
            return 0
        backend.enable(True)
        print("[OK] Outputs enabled at center.\n")

        if args.align:
            alignment_hold(ramp, center_us)
            return 0

        if args.check_axes:
            check_axes(ramp, center_us)
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
