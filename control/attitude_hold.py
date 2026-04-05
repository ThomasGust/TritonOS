# rov/control/attitude_hold.py
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Dict, Any


def _clamp(x: float, lo: float, hi: float) -> float:
    x = float(x)
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


@dataclass
class AttitudeHoldConfig:
    """Configuration for attitude-hold (pitch & roll).

    Conventions assumed:
      - pitch_deg / roll_deg come from the AHRS in degrees
      - pitch command is positive nose-UP
      - roll command is positive right-side-DOWN (starboard down)

    The controller output is in the same normalized pitch/roll space as
    build_6dof(), typically [-1..+1] before final global limiting.
    """

    enabled_default: bool = False

    # Sensor validity
    sensor_stale_s: float = 2.0

    # Filtering
    lpf_tau_s: float = 0.15

    # Controller gains (command per degree / degree-second)
    kp: float = 0.020
    ki: float = 0.005
    kd: float = 0.002

    # Small error deadband to reduce thruster chatter (degrees)
    error_deadband_deg: float = 0.5

    # Integral anti-windup (in command units)
    i_limit: float = 0.20

    # Output clamp (safety while tuning)
    out_limit: float = 0.40

    # Sign flip hooks. With the conventions above, positive angle error should
    # command the opposite pitch/roll direction to drive the vehicle back
    # toward the target, so the default is -1.0.
    pitch_sign: float = -1.0
    roll_sign: float = -1.0

    # Target walking ("rotation rate" feel)
    walk_target: bool = True
    walk_deadband: float = 0.08
    walk_rate_dps: float = 15.0  # full stick => 15 deg/s target change

    # Optional clamp on target angles (degrees, None disables)
    target_min_deg: Optional[float] = -30.0
    target_max_deg: Optional[float] = 30.0


class _SingleAxisHold:
    """PID hold for a single rotational axis (pitch or roll)."""

    def __init__(self, sign: float):
        self.sign = float(sign)
        self._angle_f: Optional[float] = None
        self._angle_prev: Optional[float] = None
        self._target: Optional[float] = None
        self._i_state: float = 0.0
        self._last_enabled: bool = False

    def reset(self) -> None:
        self._angle_f = None
        self._angle_prev = None
        self._target = None
        self._i_state = 0.0
        self._last_enabled = False

    @property
    def target_deg(self) -> Optional[float]:
        return self._target

    def step(
        self,
        *,
        enabled: bool,
        manual_cmd: float,
        angle_deg: Optional[float],
        cfg: AttitudeHoldConfig,
        dt: float,
    ) -> tuple[float, Dict[str, Any]]:
        enabled = bool(enabled)
        manual_cmd = float(manual_cmd)
        dt = float(dt) if dt and dt > 0 else 0.02

        status: Dict[str, Any] = {
            "enabled_cmd": enabled,
            "active": False,
            "reason": "manual",
        }

        if not enabled:
            self.reset()
            return manual_cmd, status

        if angle_deg is None:
            status.update({"active": False, "reason": "no_sensor"})
            if self._target is not None:
                status["target_deg"] = float(self._target)
            return manual_cmd, status

        a = float(angle_deg)
        if not math.isfinite(a):
            self.reset()
            status["reason"] = "bad_sensor"
            return manual_cmd, status

        # Low-pass filter.
        if self._angle_f is None:
            self._angle_f = a
            self._angle_prev = a
        else:
            tau = max(0.0, float(cfg.lpf_tau_s))
            alpha = dt / (tau + dt) if (tau + dt) > 0 else 1.0
            self._angle_f = float(self._angle_f + alpha * (a - self._angle_f))

        a_f = float(self._angle_f)
        a_prev = float(self._angle_prev) if self._angle_prev is not None else a_f
        da = (a_f - a_prev) / dt
        self._angle_prev = a_f

        # Engage: on rising edge, capture target and zero integrator.
        if not self._last_enabled:
            self._target = a_f
            self._i_state = 0.0
        self._last_enabled = True

        if self._target is None:
            self._target = a_f

        # Walk target with manual stick.
        integrate_ok = True
        if abs(manual_cmd) > float(cfg.walk_deadband):
            if cfg.walk_target:
                self._target += manual_cmd * float(cfg.walk_rate_dps) * dt
                integrate_ok = False
            else:
                self._target = a_f
                self._i_state = 0.0
                status.update({"active": False, "reason": "manual_override"})
                return manual_cmd, status

        # Clamp target.
        if cfg.target_min_deg is not None:
            self._target = max(float(cfg.target_min_deg), float(self._target))
        if cfg.target_max_deg is not None:
            self._target = min(float(cfg.target_max_deg), float(self._target))

        t = float(self._target)

        # Error: positive when angle exceeds target.
        e = a_f - t
        if abs(e) < float(cfg.error_deadband_deg):
            e = 0.0

        kp = float(cfg.kp)
        ki = float(cfg.ki)
        kd = float(cfg.kd)

        u_raw = (kp * e) + (ki * self._i_state) + (kd * da)
        u = float(self.sign) * float(u_raw)
        u_clamped = _clamp(u, -float(cfg.out_limit), float(cfg.out_limit))

        saturated = (u != u_clamped)
        if integrate_ok and (not saturated) and ki != 0.0:
            self._i_state = _clamp(
                self._i_state + (e * dt),
                -float(cfg.i_limit) / abs(ki),
                float(cfg.i_limit) / abs(ki),
            )
        elif not integrate_ok:
            self._i_state *= 0.98

        # Recompute with updated I state.
        u_raw2 = (kp * e) + (ki * self._i_state) + (kd * da)
        u2 = float(self.sign) * float(u_raw2)
        u2_clamped = _clamp(u2, -float(cfg.out_limit), float(cfg.out_limit))

        status.update(
            {
                "active": True,
                "reason": "hold",
                "angle_deg": a,
                "angle_f_deg": a_f,
                "target_deg": t,
                "error_deg": e,
                "da_dps": da,
                "u_raw": u_raw2,
                "u_out": u2_clamped,
            }
        )
        return u2_clamped, status


class AttitudeHoldController:
    """Attitude-hold controller for pitch and roll.

    Each axis runs an independent PID loop.  Can be enabled/disabled
    independently of depth-hold so the pilot can stabilize attitude only,
    depth only, or both simultaneously.
    """

    def __init__(self, cfg: AttitudeHoldConfig):
        self.cfg = cfg
        self._pitch = _SingleAxisHold(sign=cfg.pitch_sign)
        self._roll = _SingleAxisHold(sign=cfg.roll_sign)

    def reset(self) -> None:
        self._pitch.reset()
        self._roll.reset()

    @property
    def target_pitch_deg(self) -> Optional[float]:
        return self._pitch.target_deg

    @property
    def target_roll_deg(self) -> Optional[float]:
        return self._roll.target_deg

    def step(
        self,
        *,
        enabled: bool,
        manual_pitch: float,
        manual_roll: float,
        pitch_deg: Optional[float],
        roll_deg: Optional[float],
        sensor_age_s: Optional[float],
        dt: float,
    ) -> tuple[float, float, Dict[str, Any]]:
        """Compute pitch and roll commands.

        Returns:
            (pitch_out, roll_out, status)
        """
        enabled = bool(enabled)
        dt = float(dt) if dt and dt > 0 else 0.02

        status: Dict[str, Any] = {
            "enabled_cmd": enabled,
            "active": False,
        }

        if not enabled:
            self.reset()
            return float(manual_pitch), float(manual_roll), status

        # Sensor staleness gate (applies to both axes together since they
        # come from the same AHRS).
        if sensor_age_s is not None and float(sensor_age_s) > float(self.cfg.sensor_stale_s):
            status.update({"active": False, "reason": "stale_sensor", "sensor_age_s": float(sensor_age_s)})
            return float(manual_pitch), float(manual_roll), status

        pitch_out, pitch_st = self._pitch.step(
            enabled=enabled,
            manual_cmd=manual_pitch,
            angle_deg=pitch_deg,
            cfg=self.cfg,
            dt=dt,
        )
        roll_out, roll_st = self._roll.step(
            enabled=enabled,
            manual_cmd=manual_roll,
            angle_deg=roll_deg,
            cfg=self.cfg,
            dt=dt,
        )

        active = pitch_st.get("active", False) or roll_st.get("active", False)
        status.update(
            {
                "active": active,
                "reason": "hold" if active else pitch_st.get("reason", "unknown"),
                "pitch": pitch_st,
                "roll": roll_st,
            }
        )
        return pitch_out, roll_out, status
