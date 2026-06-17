#!/usr/bin/env python3
"""Horizontal surge-hook diagnostic.

This tool drives only the four horizontal thrusters through the same
ThrustWriter path used by the ROV runtime. It is meant for low-power, in-water
or restrained pool-side debugging when the vehicle hooks left/right during a
straight surge command.

Run on the ROV, with the normal ROV service stopped:

    sudo .venv/bin/python -m tools.surge_hook_debug

Watch the vehicle during each step and write down what actually happens.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Sequence

import rov_config as cfg
from motion.channel_map import ChannelMap
from motion.pwm import ThrustConfig, ThrustWriter


HORIZONTAL_THRUSTERS = ("H_FL", "H_FR", "H_RL", "H_RR")


@dataclass(frozen=True)
class Step:
    name: str
    commands: Mapping[str, float]
    note: str


def _thrust_config() -> ThrustConfig:
    return ThrustConfig(
        backend=str(getattr(cfg, "PWM_BACKEND", "auto")),
        freq_hz=float(getattr(cfg, "PWM_FREQ_HZ", 50.0)),
        neutral_us=int(getattr(cfg, "PWM_NEUTRAL_US", 1500)),
        span_us=int(getattr(cfg, "PWM_SPAN_US", 400)),
        min_us=int(getattr(cfg, "PWM_MIN_US", 1100)),
        max_us=int(getattr(cfg, "PWM_MAX_US", 1900)),
        deadband_norm=float(getattr(cfg, "PWM_DEADBAND", 0.07)),
        deadband_us=int(getattr(cfg, "PWM_DEADBAND_US", 25)),
        trim_us=getattr(cfg, "PWM_TRIM_US", 0),
        esc_init_hold_s=float(getattr(cfg, "ESC_INIT_HOLD_S", 3.0)),
        hardware_arm_disarm=bool(getattr(cfg, "HARDWARE_ARM_DISARM", True)),
        pwm_rearm_off_s=float(getattr(cfg, "PWM_REARM_OFF_S", 0.35)),
        pwm_disarm_hold_s=float(getattr(cfg, "PWM_DISARM_HOLD_S", 0.25)),
        disable_pwm_on_disarm=bool(getattr(cfg, "DISABLE_PWM_ON_DISARM", True)),
        keep_pwm_enabled_on_disarm=bool(getattr(cfg, "KEEP_PWM_ENABLED_ON_DISARM", True)),
        channel_base=getattr(cfg, "PWM_CHANNEL_BASE", "auto"),
        slew_rate_norm_per_s=float(getattr(cfg, "THRUSTER_SLEW_RATE_NORM_PER_S", 0.0)),
        slew_reverse_rate_norm_per_s=getattr(cfg, "THRUSTER_SLEW_REVERSE_RATE_NORM_PER_S", None),
        slew_dt_max_s=float(getattr(cfg, "THRUSTER_SLEW_DT_MAX_S", 0.10)),
        direct_i2c_bus=getattr(cfg, "PWM_DIRECT_I2C_BUS", 4),
        direct_i2c_addr=getattr(cfg, "PWM_DIRECT_I2C_ADDR", 0x40),
        direct_osc_hz=float(getattr(cfg, "PWM_DIRECT_OSC_HZ", 25_000_000.0)),
        direct_oe_gpio=getattr(cfg, "PWM_DIRECT_OE_GPIO", 26),
        direct_oe_active_low=bool(getattr(cfg, "PWM_DIRECT_OE_ACTIVE_LOW", True)),
    )


def _reversal_map() -> Dict[object, bool]:
    rev: Dict[object, bool] = {}
    rev.update(getattr(cfg, "THRUSTER_REVERSED", {}) or {})
    rev.update(getattr(cfg, "CHANNEL_REVERSED", {}) or {})
    return rev


def _scale(commands: Mapping[str, float], power: float) -> Dict[str, float]:
    return {name: float(value) * float(power) for name, value in commands.items()}


def _steps(power: float) -> List[Step]:
    return [
        Step("single_h_fl", _scale({"H_FL": 1.0}, power), "front-left alone; should have a forward component"),
        Step("single_h_fr", _scale({"H_FR": 1.0}, power), "front-right alone; should have a forward component"),
        Step("single_h_rl", _scale({"H_RL": 1.0}, power), "rear-left alone; should have a forward component"),
        Step("single_h_rr", _scale({"H_RR": 1.0}, power), "rear-right alone; should have a forward component"),
        Step("left_pair", _scale({"H_FL": 1.0, "H_RL": 1.0}, power), "left side pair; compare strength to right_pair"),
        Step("right_pair", _scale({"H_FR": 1.0, "H_RR": 1.0}, power), "right side pair; compare strength to left_pair"),
        Step("front_pair", _scale({"H_FL": 1.0, "H_FR": 1.0}, power), "front pair; compare to rear_pair"),
        Step("rear_pair", _scale({"H_RL": 1.0, "H_RR": 1.0}, power), "rear pair; compare to front_pair"),
        Step("all_forward", _scale({name: 1.0 for name in HORIZONTAL_THRUSTERS}, power), "straight surge request; should not yaw hard"),
        Step(
            "yaw_positive",
            _scale({"H_FL": 1.0, "H_FR": -1.0, "H_RL": 1.0, "H_RR": -1.0}, power),
            "legacy positive-yaw pattern; useful direction sanity check",
        ),
        Step(
            "yaw_negative",
            _scale({"H_FL": -1.0, "H_FR": 1.0, "H_RL": -1.0, "H_RR": 1.0}, power),
            "legacy negative-yaw pattern; should oppose yaw_positive",
        ),
    ]


def _neutral_command(names: Iterable[str]) -> Dict[str, float]:
    return {name: 0.0 for name in names}


def _apply_profile(channels: Dict[str, int], rev: Dict[object, bool], profile: str) -> List[str]:
    """Apply a temporary diagnostic profile without changing rov_config.py."""

    profile = str(profile or "current").strip().lower()
    notes: List[str] = []
    if profile == "current":
        return notes
    if profile in ("swap_rear", "rear_swapped"):
        channels["H_RL"], channels["H_RR"] = channels["H_RR"], channels["H_RL"]
        notes.append("swapped H_RL/H_RR PWM channels")
        return notes
    if profile in ("flip_rear_signs", "rear_signs_flipped"):
        rev["H_RL"] = not bool(rev.get("H_RL", False))
        rev["H_RR"] = not bool(rev.get("H_RR", False))
        notes.append("toggled H_RL/H_RR reversed flags")
        return notes
    if profile in ("recent_debug", "swap_rear_and_flip_signs"):
        channels["H_RL"], channels["H_RR"] = channels["H_RR"], channels["H_RL"]
        rev["H_RL"] = not bool(rev.get("H_RL", False))
        rev["H_RR"] = not bool(rev.get("H_RR", False))
        notes.append("swapped H_RL/H_RR PWM channels")
        notes.append("toggled H_RL/H_RR reversed flags")
        return notes
    raise SystemExit(
        "unknown --profile "
        f"{profile!r}; expected current, swap_rear, flip_rear_signs, or recent_debug"
    )


def _print_config(channels: Mapping[str, int], rev: Mapping[object, bool], notes: Sequence[str]) -> None:
    print("=== surge hook debug config ===")
    if notes:
        print("temporary overrides:")
        for note in notes:
            print(f"  - {note}")
    else:
        print("temporary overrides: none")
    print(f"mix_mode: {getattr(cfg, 'CONTROL_MIX_MODE', '<missing>')}")
    print(f"axis_surge: {getattr(cfg, 'AXIS_SURGE', '<missing>')} invert={getattr(cfg, 'AXIS_SURGE_INVERT', '<missing>')}")
    print(f"axis_yaw:   {getattr(cfg, 'AXIS_YAW', '<missing>')} invert={getattr(cfg, 'AXIS_YAW_INVERT', '<missing>')}")
    print("horizontal thrusters:")
    for name in HORIZONTAL_THRUSTERS:
        ch = channels.get(name)
        reversed_v = bool(rev.get(name, False) or rev.get(ch, False) or rev.get(str(ch), False))
        print(f"  {name}: pwm={ch} reversed={reversed_v}")
    print("================================")


def _select_steps(all_steps: Sequence[Step], requested: Sequence[str]) -> List[Step]:
    if not requested:
        return list(all_steps)
    by_name = {step.name: step for step in all_steps}
    unknown = [name for name in requested if name not in by_name]
    if unknown:
        raise SystemExit(f"unknown step(s): {unknown}. Known: {sorted(by_name)}")
    return [by_name[name] for name in requested]


def _wait_for_enter(message: str, *, assume_yes: bool) -> None:
    if assume_yes:
        return
    try:
        input(message)
    except EOFError:
        return


def _run_step(tw: ThrustWriter, step: Step, seconds: float, off_seconds: float, assume_yes: bool) -> None:
    command = _neutral_command(tw.thruster_channels.keys())
    command.update(step.commands)

    printable = {name: round(command[name], 3) for name in HORIZONTAL_THRUSTERS if abs(command.get(name, 0.0)) > 0.0}
    print(f"\n[step] {step.name}: {printable}")
    print(f"       {step.note}")
    _wait_for_enter("       Press Enter to pulse this step, or Ctrl+C to stop. ", assume_yes=assume_yes)

    tw.write(command)
    time.sleep(max(0.0, float(seconds)))
    tw.write(_neutral_command(tw.thruster_channels.keys()))
    time.sleep(max(0.0, float(off_seconds)))


def main() -> None:
    ap = argparse.ArgumentParser(description="Run horizontal thruster patterns to debug surge hook.")
    ap.add_argument("--power", type=float, default=0.14, help="Absolute command magnitude for each active thruster (0..1).")
    ap.add_argument("--seconds", type=float, default=1.2, help="Seconds to hold each pulse.")
    ap.add_argument("--off", type=float, default=1.0, help="Neutral time between pulses.")
    ap.add_argument("--step", action="append", default=[], help="Run only this step; may be repeated.")
    ap.add_argument(
        "--profile",
        default="current",
        help=(
            "Temporary rear-thruster override: current, swap_rear, "
            "flip_rear_signs, or recent_debug."
        ),
    )
    ap.add_argument("--list", action="store_true", help="List available steps and exit.")
    ap.add_argument("--yes", action="store_true", help="Do not prompt before arming or each pulse.")
    ap.add_argument("--debug-pwm", action="store_true", help="Enable verbose ThrustWriter PWM mapping logs.")
    args = ap.parse_args()

    power = max(0.0, min(1.0, abs(float(args.power))))
    all_steps = _steps(power)
    selected = _select_steps(all_steps, args.step)
    if args.list:
        for step in all_steps:
            print(f"{step.name:14s} {step.note}")
        return

    cm = ChannelMap.from_config(cfg)
    channels = dict(cm.thrusters)
    rev = _reversal_map()
    override_notes = _apply_profile(channels, rev, str(args.profile))
    _print_config(channels, rev, override_notes)
    print(f"pulse: power={power:.2f} seconds={float(args.seconds):.1f} off={float(args.off):.1f}")
    print("Stop the normal ROV service before running this tool. Keep power low and the vehicle clear.")
    _wait_for_enter("Press Enter to arm PWM and start the selected steps, or Ctrl+C to abort. ", assume_yes=bool(args.yes))

    tw = ThrustWriter(
        thruster_channels=channels,
        cfg=_thrust_config(),
        reversed_map=rev if rev else None,
        debug=bool(args.debug_pwm),
        auto_enable=bool(getattr(cfg, "PWM_AUTO_ENABLE", False)),
    )

    try:
        tw.arm()
        if not args.yes:
            print("Waiting through ESC neutral/arming hold before first pulse...")
        # The writer internally holds neutral until ESC_INIT_HOLD_S has elapsed.
        time.sleep(max(0.0, float(getattr(cfg, "ESC_INIT_HOLD_S", 0.0))))
        for step in selected:
            _run_step(tw, step, float(args.seconds), float(args.off), bool(args.yes))
        print("\n[done] All selected steps complete.")
    except KeyboardInterrupt:
        print("\n[abort] Interrupted; commanding neutral/shutdown.", file=sys.stderr)
    finally:
        try:
            tw.write(_neutral_command(tw.thruster_channels.keys()))
            time.sleep(0.25)
        except Exception:
            pass
        tw.shutdown()


if __name__ == "__main__":
    main()
