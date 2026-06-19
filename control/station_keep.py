"""Visual station-keeping controller (foundation for optical-tracking autopilot).

This is the *control* half of an optical station-keeping autopilot: hold position
against current by driving translational thrust to zero out a **visual error**
that a topside computer-vision module produces from a camera feed (e.g. the
transect/arm camera for the MATE RANGER "hold position watching the blue square,
see no red" task).

Design contract (the CV <-> control interface):
    The CV runs topside and publishes a normalized error in the pilot command
    ``modes["autopilot"]["visual"]`` with the schema:

        {
          "valid":     bool,    # CV currently has a confident target lock
          "ts":        float,   # producer timestamp (optional; diagnostics only)
          "ex":        float,   # horizontal target-center error,  [-1, 1]
          "ey":        float,   # vertical target-center error,    [-1, 1]
          "es":        float,   # scale/size (≈distance) error,    [-1, 1]
          "violation": float,   # 0..1 amount of "forbidden" content visible
                                #   (e.g. red border) -- 0 means none in frame
        }

    This module does NOT do any vision. It is deliberately decoupled so the CV
    can be developed/tuned independently and just fill in that dict.

Safety: like the rest of the autopilot, this controller is conservative -- if it
is disabled, has no valid lock, or the lock goes stale, it returns the pilot's
manual command untouched. It also yields any DOF the pilot is actively driving so
it never fights manual control.

The mapping from error components to thrust DOFs is fully config-driven (a list
of ``StationKeepAxis``), because the exact "what should the ROV do" policy is a
tuning problem the pilot will iterate on, not something to hard-code here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


_ERROR_KEYS = ("ex", "ey", "es", "violation")
_DOF_KEYS = ("surge", "sway", "heave", "yaw", "roll", "pitch")


def _clamp(x: float, lo: float, hi: float) -> float:
    x = float(x)
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        v = float(value)
    except Exception:
        return float(default)
    return v if math.isfinite(v) else float(default)


@dataclass
class StationKeepAxis:
    """One controlled thrust DOF driven by one visual error component."""

    dof: str                    # which output DOF: surge/sway/heave/yaw/...
    error_key: str              # which visual error drives it: ex/ey/es/violation
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    error_deadband: float = 0.03    # ignore tiny normalized errors
    i_limit: float = 0.20
    out_limit: float = 0.35         # cap this axis' contribution
    sign: float = 1.0               # flip to match camera/thruster geometry
    manual_deadband: float = 0.08   # pilot input above this yields the DOF


@dataclass
class StationKeepConfig:
    """Full visual station-keep configuration."""

    enable: bool = True
    stale_s: float = 0.5            # drop hold if no valid lock for this long
    axes: List[StationKeepAxis] = field(default_factory=list)
    # Safety cap on direct model thrust outputs (visual["command"][dof]).
    direct_limit: float = 0.75


def default_station_keep_axes() -> List[StationKeepAxis]:
    """Conservative default policy for the transect "hold position in current" task.

    Three translational DOFs, each tunable via ``STATION_KEEP_<DOF>_*`` config:

    - **sway  <- ex**  horizontal centering of the target in frame.
    - **surge <- es**  standoff (default error key kept for back-compat; the
      transect policy overrides it to ``ey`` -- fore/aft centering -- via
      ``STATION_KEEP_SURGE_ERROR_KEY = "ey"``, since for a down-looking camera
      vertical image position maps to fore/aft).
    - **heave <- es**  a *gentle* vision size-trim layered on top of depth hold,
      which owns bulk altitude. It naturally yields whenever depth hold is
      actively driving heave (manual-override path) and trims only when depth is
      settled, so keep its gain/limit small.

    Gains start at 0 so the controller is inert until the pilot tunes it (it
    reports "active" structurally but commands ~0 -- safe to enable while
    tuning). Heading (yaw) and leveling (roll/pitch) stay with attitude hold; add
    axes here to involve them.
    """
    return [
        StationKeepAxis(dof="sway", error_key="ex", kp=0.0, kd=0.0),
        StationKeepAxis(dof="surge", error_key="es", kp=0.0, kd=0.0),
        StationKeepAxis(dof="heave", error_key="es", kp=0.0, kd=0.0, out_limit=0.15),
    ]


class StationKeepController:
    """PID-per-DOF regulator that drives a visual error vector to zero."""

    def __init__(self, cfg: StationKeepConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self._i_state: Dict[str, float] = {}
        self._last_error: Dict[str, float] = {}
        self._stale_timer: float = 0.0
        self._ever_valid: bool = False
        self._last_enabled: bool = False
        self._last_ts: Any = None

    def _clear_integrators(self) -> None:
        self._i_state.clear()
        self._last_error.clear()

    def step(
        self,
        *,
        enabled: bool,
        manual_cmd: Dict[str, float],
        visual: Optional[Dict[str, Any]],
        dt: float,
    ) -> Tuple[Dict[str, float], Dict[str, Any]]:
        """Return (corrected command dict, status)."""

        dt = float(dt) if dt and dt > 0.0 else 0.02
        out = dict(manual_cmd or {})
        enabled = bool(enabled) and bool(self.cfg.enable)

        status: Dict[str, Any] = {
            "enabled_cmd": enabled,
            "active": False,
            "reason": "off",
            "axes": {},
        }

        if not enabled:
            if self._last_enabled:
                self.reset()
            self._last_enabled = False
            status["reason"] = "disabled"
            return out, status

        visual = dict(visual or {})
        valid = bool(visual.get("valid"))

        if not valid:
            # Explicit "no target" (or no visual at all): hold manual.
            self._stale_timer = 0.0
            self._clear_integrators()
            self._last_enabled = True
            status["reason"] = "no_lock"
            return out, status

        # Frozen-CV protection: if the producer keeps asserting valid with an
        # unchanging timestamp for longer than stale_s, treat the lock as stale.
        ts = visual.get("ts")
        if ts is not None and self._ever_valid and ts == self._last_ts:
            self._stale_timer += dt
        else:
            self._stale_timer = 0.0
        self._last_ts = ts
        self._ever_valid = True

        if self._stale_timer > float(self.cfg.stale_s):
            self._clear_integrators()
            self._last_enabled = True
            status["reason"] = "stale_lock"
            return out, status

        errors = {key: _finite(visual.get(key), 0.0) for key in _ERROR_KEYS}
        status["errors"] = errors
        any_active = False
        axes_status: Dict[str, Any] = {}

        # Capture the pilot's original command per DOF before we modify `out`, so
        # manual-override detection is consistent across the error-PID and direct
        # paths even when both target the same DOF.
        manual_in = {dof: _finite(out.get(dof, 0.0)) for dof in _DOF_KEYS}

        for idx, ax in enumerate(self.cfg.axes):
            if ax.dof not in _DOF_KEYS or ax.error_key not in _ERROR_KEYS:
                continue
            key = f"{idx}:{ax.dof}:{ax.error_key}"
            err = errors.get(ax.error_key, 0.0)
            if abs(err) < float(ax.error_deadband):
                err = 0.0

            manual = manual_in.get(ax.dof, 0.0)
            if abs(manual) > float(ax.manual_deadband):
                # Pilot is driving this DOF -- yield it and bleed the integrator.
                self._i_state[key] = 0.0
                self._last_error[key] = err
                axes_status[ax.dof] = {
                    "active": False,
                    "reason": "manual_override",
                    "error": err,
                    "manual": manual,
                }
                continue

            prev = self._last_error.get(key)
            d_err = ((err - prev) / dt) if prev is not None else 0.0
            self._last_error[key] = err

            i_state = self._i_state.get(key, 0.0)
            u_raw = float(ax.sign) * (
                float(ax.kp) * err + float(ax.ki) * i_state + float(ax.kd) * d_err
            )
            u = _clamp(u_raw, -float(ax.out_limit), float(ax.out_limit))

            saturated = abs(u - u_raw) > 1e-9
            if (not saturated) and float(ax.ki) != 0.0 and err != 0.0:
                lim = float(ax.i_limit) / abs(float(ax.ki))
                i_state = _clamp(i_state + err * dt, -lim, lim)
                self._i_state[key] = i_state
                u_raw = float(ax.sign) * (
                    float(ax.kp) * err + float(ax.ki) * i_state + float(ax.kd) * d_err
                )
                u = _clamp(u_raw, -float(ax.out_limit), float(ax.out_limit))

            out[ax.dof] = _clamp(manual + u, -1.0, 1.0)
            any_active = any_active or abs(u) > 1e-9
            axes_status[ax.dof] = {
                "active": True,
                "error": err,
                "u": u,
                "i_state": self._i_state.get(key, 0.0),
            }

        # Direct model outputs: the model can command any DOF straight through
        # (model-as-controller), bypassing the error->PID mapping for that DOF.
        # This is what gives the future ML policy full surge/sway/heave and
        # roll/pitch/yaw authority; dynamic depth/attitude *setpoints* go through
        # the autopilot depth/attitude holds (modes["autopilot"]["targets"]).
        command = visual.get("command")
        if isinstance(command, dict):
            limit = float(self.cfg.direct_limit)
            for dof, raw in command.items():
                if dof not in _DOF_KEYS:
                    continue
                if abs(manual_in.get(dof, 0.0)) > 0.08:
                    axes_status[dof] = {"active": False, "reason": "manual_override", "direct": True}
                    continue
                u = _clamp(_finite(raw, 0.0), -limit, limit)
                out[dof] = _clamp(u, -1.0, 1.0)
                any_active = any_active or abs(u) > 1e-9
                axes_status[dof] = {"active": True, "reason": "direct", "u": u}

        self._last_enabled = True
        status["active"] = any_active
        status["reason"] = "active" if any_active else "locked_idle"
        status["axes"] = axes_status
        status["stale_timer_s"] = self._stale_timer
        return out, status


def station_keep_config_from_module(cfg_mod: Any) -> StationKeepConfig:
    """Assemble StationKeepConfig from a ``rov_config``-like module.

    Per-axis gains come from ``STATION_KEEP_<DOF>_<PARAM>`` so the pilot can tune
    the policy in config without code changes. Defaults keep the controller inert
    (zero gains) until tuned.
    """

    axes: List[StationKeepAxis] = []
    for ax in default_station_keep_axes():
        prefix = f"STATION_KEEP_{ax.dof.upper()}"
        axes.append(
            StationKeepAxis(
                dof=ax.dof,
                error_key=str(getattr(cfg_mod, f"{prefix}_ERROR_KEY", ax.error_key)),
                kp=float(getattr(cfg_mod, f"{prefix}_KP", ax.kp)),
                ki=float(getattr(cfg_mod, f"{prefix}_KI", ax.ki)),
                kd=float(getattr(cfg_mod, f"{prefix}_KD", ax.kd)),
                error_deadband=float(getattr(cfg_mod, f"{prefix}_ERROR_DEADBAND", ax.error_deadband)),
                i_limit=float(getattr(cfg_mod, f"{prefix}_I_LIMIT", ax.i_limit)),
                out_limit=float(getattr(cfg_mod, f"{prefix}_OUT_LIMIT", ax.out_limit)),
                sign=float(getattr(cfg_mod, f"{prefix}_SIGN", ax.sign)),
                manual_deadband=float(getattr(cfg_mod, f"{prefix}_MANUAL_DEADBAND", ax.manual_deadband)),
            )
        )
    return StationKeepConfig(
        enable=bool(getattr(cfg_mod, "STATION_KEEP_ENABLE", True)),
        stale_s=float(getattr(cfg_mod, "STATION_KEEP_STALE_S", 0.5)),
        direct_limit=float(getattr(cfg_mod, "STATION_KEEP_DIRECT_LIMIT", 0.75)),
        axes=axes,
    )
