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
    sensor_stale_s: float = 0.6

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

    # Constant heave trim (feed-forward) in normalized command space.
    # Use this to counteract small positive/negative buoyancy so the PID doesn't
    # need to “wind up” for seconds before holding depth.
    # Positive = UP.
    trim: float = 0.0

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

    def update_config(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        """Best-effort live config update.

        Intended for topside tuning. Values are sanity-clamped.
        Returns a dict describing what was applied.
        """
        u = updates or {}
        applied: Dict[str, Any] = {}

        def f(key: str, lo: float, hi: float) -> None:
            if key not in u:
                return
            try:
                v = float(u[key])
            except Exception:
                return
            v = _clamp(v, lo, hi)
            setattr(self.cfg, key, v)
            applied[key] = v

        def b(key: str) -> None:
            if key not in u:
                return
            try:
                v = bool(u[key])
            except Exception:
                return
            setattr(self.cfg, key, v)
            applied[key] = v

        # Gains and limits
        f("kp", 0.0, 5.0)
        f("ki", 0.0, 5.0)
        f("kd", 0.0, 5.0)
        f("out_limit", 0.0, 1.0)
        f("i_limit", 0.0, 2.0)
        f("error_deadband_m", 0.0, 0.5)
        f("sensor_stale_s", 0.05, 10.0)
        f("depth_lpf_tau_s", 0.0, 10.0)
        f("sign", -1.0, 1.0)
        f("trim", -1.0, 1.0)

        # Walk target
        b("walk_target")
        f("walk_deadband", 0.0, 1.0)
        f("walk_rate_mps", 0.0, 3.0)

        # Optional target clamps
        for key in ("target_min_m", "target_max_m"):
            if key in u:
                try:
                    vv = u[key]
                    if vv is None:
                        setattr(self.cfg, key, None)
                        applied[key] = None
                    else:
                        v = float(vv)
                        setattr(self.cfg, key, v)
                        applied[key] = v
                except Exception:
                    pass

        return applied

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
        if depth_m is None or (depth_age_s is not None and float(depth_age_s) > float(self.cfg.sensor_stale_s)):
            # Stay manual if depth is unavailable.
            self.reset()
            status["reason"] = "no_depth"
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
        # Apply constant trim in final command space (positive = UP).
        u2 = float(u2) + float(getattr(self.cfg, "trim", 0.0))
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
                "trim": float(getattr(self.cfg, "trim", 0.0)),
                # Echo key tuning parameters for topside visibility.
                "kp": float(kp),
                "ki": float(ki),
                "kd": float(kd),
                "out_limit": float(self.cfg.out_limit),
                "i_limit": float(self.cfg.i_limit),
                "error_deadband_m": float(self.cfg.error_deadband_m),
                "walk_deadband": float(self.cfg.walk_deadband),
                "walk_rate_mps": float(self.cfg.walk_rate_mps),
            }
        )
        return u2_clamped, status
