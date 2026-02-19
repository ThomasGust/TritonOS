# rov/control/depth_hold.py
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
class DepthHoldConfig:
    """Configuration for depth-hold.

    Conventions assumed:
      - depth_m is positive DOWN (increasing depth = going deeper)
      - heave command is positive UP (decreasing depth)

    The controller output is in the same normalized heave space as build_6dof(),
    typically [-1..+1] before final global limiting.
    """

    enabled_default: bool = False

    # Sensor validity
    # If valid depth telemetry is older than this, depth-hold pauses (inactive)
    # but stays enabled and keeps the target.
    sensor_stale_s: float = 2.0

    # Filtering
    depth_lpf_tau_s: float = 0.50

    # Controller gains (units: command per meter / meter-second)
    kp: float = 0.30
    ki: float = 0.05
    kd: float = 0.00

    # Small error deadband to reduce thruster chatter
    error_deadband_m: float = 0.03

    # Integral anti-windup
    i_limit: float = 0.25

    # Output clamp (safety while tuning)
    out_limit: float = 0.55

    # Sign flip hook (set to -1.0 if “too deep” makes it go deeper)
    sign: float = 1.0

    # Target walking ("vertical speed" feel)
    walk_target: bool = True
    walk_deadband: float = 0.08
    walk_rate_mps: float = 0.60

    # Optional clamp on target depth (None disables)
    target_min_m: Optional[float] = None
    target_max_m: Optional[float] = None


class DepthHoldController:
    """Depth-hold controller with sticky/"walk target" behavior."""

    def __init__(self, cfg: DepthHoldConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self._active = False
        self._z_f: Optional[float] = None
        self._z_prev: Optional[float] = None
        self._z_target: Optional[float] = None
        self._i_state: float = 0.0  # integral of error (meters*sec)
        self._last_enabled: bool = False

    @property
    def target_depth_m(self) -> Optional[float]:
        return self._z_target

    def step(
        self,
        *,
        enabled: bool,
        manual_heave: float,
        depth_m: Optional[float],
        depth_age_s: Optional[float],
        dt: float,
    ) -> tuple[float, Dict[str, Any]]:
        """Compute heave command.

        Returns:
            (heave_out, status)
        """
        enabled = bool(enabled)
        manual_heave = float(manual_heave)
        dt = float(dt) if dt and dt > 0 else 0.02

        status: Dict[str, Any] = {
            "enabled_cmd": enabled,
            "active": False,
            "reason": "manual",
        }

        # If not enabled, pass through manual and reset state.
        if not enabled:
            self.reset()
            return manual_heave, status

        # Sensor validity gate.
        # IMPORTANT: Do NOT reset controller state here.
        # We want the target depth to remain sticky across brief dropouts so the
        # setpoint doesn't "snap" back to 0.0 on the topside.
        if depth_m is None:
            status.update({"active": False, "reason": "no_depth"})
            if self._z_target is not None:
                status["target_m"] = float(self._z_target)
            if self._z_f is not None:
                status["depth_f_m"] = float(self._z_f)
            return manual_heave, status

        if depth_age_s is not None and float(depth_age_s) > float(self.cfg.sensor_stale_s):
            status.update({"active": False, "reason": "stale_depth", "depth_age_s": float(depth_age_s)})
            if self._z_target is not None:
                status["target_m"] = float(self._z_target)
            if self._z_f is not None:
                status["depth_f_m"] = float(self._z_f)
            return manual_heave, status

        z = float(depth_m)
        if not math.isfinite(z):
            self.reset()
            status["reason"] = "bad_depth"
            return manual_heave, status

        # Low-pass filter depth.
        if self._z_f is None:
            self._z_f = z
            self._z_prev = z
        else:
            tau = float(self.cfg.depth_lpf_tau_s)
            tau = max(0.0, tau)
            alpha = dt / (tau + dt) if (tau + dt) > 0 else 1.0
            self._z_f = float(self._z_f + alpha * (z - self._z_f))

        z_f = float(self._z_f)
        z_prev = float(self._z_prev) if self._z_prev is not None else z_f
        dz = (z_f - z_prev) / dt
        self._z_prev = z_f

        # Engage logic: on rising edge, capture target and zero integrator.
        if not self._last_enabled:
            self._z_target = z_f
            self._i_state = 0.0
            self._active = True
        self._last_enabled = True
        self._active = True

        # Ensure target exists.
        if self._z_target is None:
            self._z_target = z_f

        # Walk target with manual stick (optional), or sticky-manual override.
        integrate_ok = True
        if abs(manual_heave) > float(self.cfg.walk_deadband):
            if self.cfg.walk_target:
                # manual_heave > 0 means "UP" => target depth should DECREASE
                self._z_target += (-manual_heave) * float(self.cfg.walk_rate_mps) * dt
                integrate_ok = False  # avoid wind-up while pilot is actively commanding
            else:
                # "Sticky" mode: pilot is directly commanding heave. Keep the target
                # glued to the current depth so releasing the stick captures it.
                self._z_target = z_f
                self._i_state = 0.0
                status.update({"active": False, "reason": "manual_override"})
                return manual_heave, status

        # Clamp target if desired.
        if self.cfg.target_min_m is not None:
            self._z_target = max(float(self.cfg.target_min_m), float(self._z_target))
        if self.cfg.target_max_m is not None:
            self._z_target = min(float(self.cfg.target_max_m), float(self._z_target))

        z_t = float(self._z_target)

        # Error: positive when too deep.
        e = z_f - z_t
        if abs(e) < float(self.cfg.error_deadband_m):
            e = 0.0

        # PI(D)
        kp = float(self.cfg.kp)
        ki = float(self.cfg.ki)
        kd = float(self.cfg.kd)

        # Integrate only if we're not actively walking target and the output isn't saturated.
        # We apply anti-windup by only integrating when unclamped output equals clamped output.
        # (we compute unclamped first using the previous integral).
        u_raw = (kp * e) + (ki * self._i_state) + (kd * dz)
        u = float(self.cfg.sign) * float(u_raw)
        u_clamped = _clamp(u, -float(self.cfg.out_limit), float(self.cfg.out_limit))

        saturated = (u != u_clamped)
        if integrate_ok and (not saturated) and ki != 0.0:
            self._i_state = _clamp(
                self._i_state + (e * dt),
                -float(self.cfg.i_limit) / abs(ki),
                float(self.cfg.i_limit) / abs(ki),
            )
        elif (not integrate_ok):
            # Optional: gently decay integrator while actively walking target.
            self._i_state *= 0.98

        # Recompute with (possibly) updated I state and re-clamp.
        u_raw2 = (kp * e) + (ki * self._i_state) + (kd * dz)
        u2 = float(self.cfg.sign) * float(u_raw2)
        u2_clamped = _clamp(u2, -float(self.cfg.out_limit), float(self.cfg.out_limit))

        status.update(
            {
                "active": True,
                "reason": "hold",
                "depth_m": z,
                "depth_f_m": z_f,
                "target_m": z_t,
                "error_m": e,
                "dz_mps": dz,
                "u_raw": u_raw2,
                "u_out": u2_clamped,
            }
        )
        return u2_clamped, status
