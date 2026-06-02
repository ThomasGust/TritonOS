"""Depth and attitude hold coordination.

The autopilot module produces corrected normalized DOF commands before final
thruster mixing. Depth hold owns the heave correction, while the per-axis
attitude controllers own roll, pitch, and yaw corrections. `ControlService`
decides when those corrections are allowed to influence the output based on
pilot modes, telemetry freshness, and vehicle arming state.

All controllers are conservative by design: missing or stale sensor data falls
back to manual pilot commands instead of inventing a correction from bad state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from control.depth_hold import DepthHoldConfig, DepthHoldController


def _clamp(x: float, lo: float, hi: float) -> float:
    x = float(x)
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _finite_float(value: Any) -> Optional[float]:
    try:
        v = float(value)
    except Exception:
        return None
    if not math.isfinite(v):
        return None
    return v


def _wrap_deg(deg: float) -> float:
    return ((float(deg) + 180.0) % 360.0) - 180.0


def _mode(value: Any, default: str = "off") -> str:
    if isinstance(value, bool):
        return "hold" if value else "off"
    text = str(value if value is not None else default).strip().lower()
    if text in ("", "none", "false", "0", "manual", "free"):
        return "off"
    if text in ("true", "1", "on"):
        return "hold"
    if text in ("level", "flat", "flatten"):
        return "level"
    if text in ("damp", "damping", "stabilize", "stabilise"):
        return "damp"
    if text in ("hold", "lock"):
        return "hold"
    return default


@dataclass
class AttitudeAxisConfig:
    """Tuning and behavior settings for one attitude-hold axis."""

    default_mode: str = "off"
    kp: float = 0.012
    ki: float = 0.0
    kd: float = 0.002
    error_deadband_deg: float = 0.5
    i_limit: float = 0.10
    out_limit: float = 0.16
    sign: float = 1.0
    manual_deadband: float = 0.08
    walk_rate_dps: float = 35.0


@dataclass
class AutopilotConfig:
    """Full autopilot configuration assembled from `rov_config`."""

    depth_enable: bool
    attitude_enable: bool
    attitude_stale_s: float
    depth: DepthHoldConfig
    roll: AttitudeAxisConfig
    pitch: AttitudeAxisConfig
    yaw: AttitudeAxisConfig


class AttitudeAxisController:
    """Small per-axis attitude controller.

    Modes:
      - off: pass manual command through
      - damp: pass manual command plus angular-rate damping
      - hold: capture current angle as target; manual input walks target
      - level: hold a fixed 0-degree target; manual input passes through
    """

    def __init__(self, axis: str, cfg: AttitudeAxisConfig):
        self.axis = str(axis)
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        """Clear captured target, integrator, and rate history for this axis."""

        self._last_enabled = False
        self._target_deg: Optional[float] = None
        self._i_state = 0.0
        self._last_angle_deg: Optional[float] = None

    @property
    def target_deg(self) -> Optional[float]:
        """Return the currently captured attitude target, if any."""

        return self._target_deg

    def step(
        self,
        *,
        mode: str,
        manual_cmd: float,
        angle_deg: Optional[float],
        ready: bool,
        stale: bool,
        dt: float,
        target_deg: Optional[float] = None,
        measured_rate_dps: Optional[float] = None,
    ) -> Tuple[float, Dict[str, Any]]:
        """Compute one corrected axis command and return status metadata."""

        dt = float(dt) if dt and dt > 0.0 else 0.02
        manual_cmd = float(manual_cmd)
        mode = _mode(mode, self.cfg.default_mode)
        target_cmd = _finite_float(target_deg)
        if target_cmd is not None:
            target_cmd = _wrap_deg(target_cmd)
        status: Dict[str, Any] = {
            "mode": mode,
            "enabled_cmd": mode != "off",
            "active": False,
            "reason": "manual",
            "manual_cmd": manual_cmd,
        }
        if target_cmd is not None:
            status["target_cmd_deg"] = float(target_cmd)

        if mode == "off":
            self.reset()
            return manual_cmd, status

        if angle_deg is None:
            status["reason"] = "no_attitude"
            self._last_enabled = False
            return manual_cmd, status
        if not ready:
            status["reason"] = "not_ready"
            self._last_enabled = False
            return manual_cmd, status
        if stale:
            status["reason"] = "stale_attitude"
            return manual_cmd, status

        angle = float(angle_deg)
        measured_rate = _finite_float(measured_rate_dps)
        rate_source = "measured" if measured_rate is not None else "angle_diff"
        rate_dps = float(measured_rate) if measured_rate is not None else 0.0
        if measured_rate is None and self._last_angle_deg is not None:
            rate_dps = _wrap_deg(angle - float(self._last_angle_deg)) / dt
        self._last_angle_deg = angle

        manual_active = abs(manual_cmd) > float(self.cfg.manual_deadband)
        if mode == "level":
            target = 0.0
            self._target_deg = target
            if manual_active:
                self._i_state = 0.0
                self._last_enabled = True
                status.update(
                    {
                        "active": False,
                        "reason": "manual_override",
                        "angle_deg": angle,
                        "target_deg": target,
                        "rate_dps": rate_dps,
                        "rate_source": rate_source,
                    }
                )
                return manual_cmd, status
        elif mode == "hold":
            if target_cmd is not None:
                if self._target_deg is None or abs(_wrap_deg(float(target_cmd) - float(self._target_deg))) > 1e-9:
                    self._target_deg = float(target_cmd)
                    self._i_state = 0.0
            elif (not self._last_enabled) or self._target_deg is None:
                self._target_deg = angle
                self._i_state = 0.0
            if manual_active:
                if target_cmd is not None:
                    self._i_state = 0.0
                    self._last_enabled = True
                    status.update(
                        {
                            "active": False,
                            "reason": "manual_override",
                            "angle_deg": angle,
                            "target_deg": float(self._target_deg),
                            "target_source": "command",
                            "rate_dps": rate_dps,
                            "rate_source": rate_source,
                        }
                    )
                    return manual_cmd, status
                self._target_deg = _wrap_deg(float(self._target_deg) + manual_cmd * float(self.cfg.walk_rate_dps) * dt)
        elif mode == "damp":
            u_damp = float(self.cfg.sign) * (-float(self.cfg.kd) * rate_dps)
            u = _clamp(manual_cmd + u_damp, -float(self.cfg.out_limit), float(self.cfg.out_limit))
            self._last_enabled = True
            status.update(
                {
                    "active": True,
                    "reason": "damp",
                    "angle_deg": angle,
                    "rate_dps": rate_dps,
                    "rate_source": rate_source,
                    "u_out": u,
                }
            )
            return u, status
        else:
            status["reason"] = "bad_mode"
            self._last_enabled = False
            return manual_cmd, status

        target = float(self._target_deg if self._target_deg is not None else angle)
        error_deg = _wrap_deg(target - angle)
        if abs(error_deg) < float(self.cfg.error_deadband_deg):
            error_deg = 0.0

        kp = float(self.cfg.kp)
        ki = float(self.cfg.ki)
        kd = float(self.cfg.kd)
        u_raw = float(self.cfg.sign) * ((kp * error_deg) + (ki * self._i_state) - (kd * rate_dps))
        u = _clamp(u_raw, -float(self.cfg.out_limit), float(self.cfg.out_limit))

        saturated = abs(u - u_raw) > 1e-9
        integrate_ok = (not saturated) and (not manual_active) and ki != 0.0
        if integrate_ok:
            self._i_state = _clamp(
                self._i_state + error_deg * dt,
                -float(self.cfg.i_limit) / abs(ki),
                float(self.cfg.i_limit) / abs(ki),
            )
            u_raw = float(self.cfg.sign) * ((kp * error_deg) + (ki * self._i_state) - (kd * rate_dps))
            u = _clamp(u_raw, -float(self.cfg.out_limit), float(self.cfg.out_limit))
        elif manual_active:
            self._i_state *= 0.98

        self._last_enabled = True
        status.update(
            {
                "active": True,
                "reason": "hold" if mode != "level" else "level",
                "angle_deg": angle,
                "target_deg": target,
                "target_source": "command" if target_cmd is not None else ("level" if mode == "level" else "capture"),
                "error_deg": error_deg,
                "rate_dps": rate_dps,
                "rate_source": rate_source,
                "u_raw": u_raw,
                "u_out": u,
            }
        )
        return u, status


class AutopilotController:
    """Coordinates depth and attitude holds before the vehicle mixer."""

    def __init__(self, cfg: AutopilotConfig):
        self.cfg = cfg
        self.depth_hold = DepthHoldController(cfg.depth)
        self.axes = {
            "roll": AttitudeAxisController("roll", cfg.roll),
            "pitch": AttitudeAxisController("pitch", cfg.pitch),
            "yaw": AttitudeAxisController("yaw", cfg.yaw),
        }

    def reset(self) -> None:
        """Reset depth hold and all attitude-axis controllers."""

        self.depth_hold.reset()
        for axis in self.axes.values():
            axis.reset()

    def step(
        self,
        *,
        modes: Dict[str, Any],
        cmd: Dict[str, float],
        depth_m: Optional[float],
        depth_age_s: Optional[float],
        attitude: Dict[str, Any],
        attitude_age_s: Optional[float],
        dt: float,
    ) -> Tuple[Dict[str, float], Dict[str, Any]]:
        """Apply enabled depth/attitude holds to a manual command dictionary."""

        out = dict(cmd or {})
        modes = dict(modes or {})
        ap_modes = modes.get("autopilot") if isinstance(modes.get("autopilot"), dict) else {}
        ap_modes = dict(ap_modes or {})
        targets = ap_modes.get("targets") if isinstance(ap_modes.get("targets"), dict) else {}
        targets = dict(targets or {})

        depth_enabled = bool(ap_modes.get("depth", modes.get("depth_hold", modes.get("depth_hold_enabled", False))))
        depth_target = self._target_value(targets, ap_modes, modes, "depth_m", ("depth_target_m", "target_depth_m"))
        depth_status: Dict[str, Any] = {"enabled_cmd": depth_enabled, "active": False, "reason": "disabled"}
        if self.cfg.depth_enable:
            heave_out, depth_status = self.depth_hold.step(
                enabled=depth_enabled,
                manual_heave=float(out.get("heave", 0.0) or 0.0),
                depth_m=depth_m,
                depth_age_s=depth_age_s,
                dt=dt,
                target_m=depth_target,
            )
            out["heave"] = float(heave_out)

        attitude_status = self._step_attitude(ap_modes, modes, targets, out, attitude, attitude_age_s, dt)
        status = {
            "enabled_cmd": bool(depth_status.get("enabled_cmd")) or bool(attitude_status.get("enabled_cmd")),
            "active": bool(depth_status.get("active")) or bool(attitude_status.get("active")),
            "depth_hold": depth_status,
            "attitude": attitude_status,
        }
        return out, status

    def _axis_mode(self, axis: str, ap_modes: Dict[str, Any], modes: Dict[str, Any]) -> str:
        rp_level = bool(
            ap_modes.get("roll_pitch_level", modes.get("roll_pitch_level", modes.get("attitude_rp_level", False)))
        )
        if axis in ("roll", "pitch") and rp_level:
            return "level"
        if axis in ap_modes:
            return _mode(ap_modes.get(axis), self.axes[axis].cfg.default_mode)
        for key in (f"attitude_{axis}", f"{axis}_hold"):
            if key in modes:
                return _mode(modes.get(key), self.axes[axis].cfg.default_mode)
        if modes.get("attitude_hold") is True:
            return "hold"
        return _mode(self.axes[axis].cfg.default_mode, "off")

    @staticmethod
    def _target_value(
        targets: Dict[str, Any],
        ap_modes: Dict[str, Any],
        modes: Dict[str, Any],
        target_key: str,
        fallback_keys: Tuple[str, ...],
    ) -> Optional[float]:
        if target_key in targets:
            return _finite_float(targets.get(target_key))
        for key in fallback_keys:
            if key in ap_modes:
                return _finite_float(ap_modes.get(key))
            if key in modes:
                return _finite_float(modes.get(key))
        return None

    def _step_attitude(
        self,
        ap_modes: Dict[str, Any],
        modes: Dict[str, Any],
        targets: Dict[str, Any],
        out: Dict[str, float],
        attitude: Dict[str, Any],
        attitude_age_s: Optional[float],
        dt: float,
    ) -> Dict[str, Any]:
        axes_status: Dict[str, Any] = {}
        if not self.cfg.attitude_enable:
            return {"enabled_cmd": False, "active": False, "reason": "disabled", "axes": axes_status}

        stale = attitude_age_s is None or float(attitude_age_s) > float(self.cfg.attitude_stale_s)
        roll_pitch_ready = bool((attitude or {}).get("roll_pitch_ready", (attitude or {}).get("attitude_ready", False)))
        yaw_ready = bool((attitude or {}).get("yaw_ready", False))
        any_enabled = False
        any_active = False

        for axis in ("roll", "pitch", "yaw"):
            mode = self._axis_mode(axis, ap_modes, modes)
            any_enabled = any_enabled or mode != "off"
            angle = _finite_float((attitude or {}).get(f"{axis}_deg"))
            ready = yaw_ready if axis == "yaw" else roll_pitch_ready
            target = self._target_value(
                targets,
                ap_modes,
                modes,
                f"{axis}_deg",
                (f"{axis}_target_deg", f"target_{axis}_deg"),
            )
            u, st = self.axes[axis].step(
                mode=mode,
                manual_cmd=float(out.get(axis, 0.0) or 0.0),
                angle_deg=angle,
                ready=ready,
                stale=stale,
                dt=dt,
                target_deg=target,
                measured_rate_dps=_finite_float((attitude or {}).get(f"{axis}_rate_dps")),
            )
            out[axis] = float(u)
            any_active = any_active or bool(st.get("active"))
            axes_status[axis] = st

        return {
            "enabled_cmd": any_enabled,
            "active": any_active,
            "reason": "active" if any_active else ("enabled" if any_enabled else "off"),
            "sample_age_s": attitude_age_s,
            "source": (attitude or {}).get("source"),
            "yaw_status": (attitude or {}).get("yaw_status"),
            "axes": axes_status,
        }


def autopilot_config_from_module(cfg_mod: Any) -> AutopilotConfig:
    """Assemble autopilot/depth-hold config from a ``rov_config``-like module."""

    def axis_config(axis: str, defaults: AttitudeAxisConfig) -> AttitudeAxisConfig:
        prefix = f"AUTOPILOT_{axis.upper()}"
        return AttitudeAxisConfig(
            default_mode=str(getattr(cfg_mod, f"{prefix}_MODE_DEFAULT", defaults.default_mode)),
            kp=float(getattr(cfg_mod, f"{prefix}_KP", defaults.kp)),
            ki=float(getattr(cfg_mod, f"{prefix}_KI", defaults.ki)),
            kd=float(getattr(cfg_mod, f"{prefix}_KD", defaults.kd)),
            error_deadband_deg=float(getattr(cfg_mod, f"{prefix}_ERROR_DEADBAND_DEG", defaults.error_deadband_deg)),
            i_limit=float(getattr(cfg_mod, f"{prefix}_I_LIMIT", defaults.i_limit)),
            out_limit=float(getattr(cfg_mod, f"{prefix}_OUT_LIMIT", defaults.out_limit)),
            sign=float(getattr(cfg_mod, f"{prefix}_SIGN", defaults.sign)),
            manual_deadband=float(getattr(cfg_mod, f"{prefix}_MANUAL_DEADBAND", defaults.manual_deadband)),
            walk_rate_dps=float(getattr(cfg_mod, f"{prefix}_WALK_RATE_DPS", defaults.walk_rate_dps)),
        )

    depth_cfg = DepthHoldConfig(
        sensor_stale_s=float(getattr(cfg_mod, "DEPTH_HOLD_SENSOR_STALE_S", 0.6)),
        depth_lpf_tau_s=float(getattr(cfg_mod, "DEPTH_HOLD_LPF_TAU_S", 0.50)),
        kp=float(getattr(cfg_mod, "DEPTH_HOLD_KP", 0.30)),
        ki=float(getattr(cfg_mod, "DEPTH_HOLD_KI", 0.05)),
        kd=float(getattr(cfg_mod, "DEPTH_HOLD_KD", 0.00)),
        error_deadband_m=float(getattr(cfg_mod, "DEPTH_HOLD_ERROR_DEADBAND_M", 0.03)),
        i_limit=float(getattr(cfg_mod, "DEPTH_HOLD_I_LIMIT", 0.25)),
        out_limit=float(getattr(cfg_mod, "DEPTH_HOLD_OUT_LIMIT", 0.55)),
        sign=float(getattr(cfg_mod, "DEPTH_HOLD_SIGN", 1.0)),
        walk_target=bool(getattr(cfg_mod, "DEPTH_HOLD_WALK_TARGET", False)),
        walk_deadband=float(getattr(cfg_mod, "DEPTH_HOLD_WALK_DEADBAND", 0.08)),
        walk_rate_mps=float(getattr(cfg_mod, "DEPTH_HOLD_WALK_RATE_MPS", 0.60)),
        target_min_m=getattr(cfg_mod, "DEPTH_HOLD_TARGET_MIN_M", None),
        target_max_m=getattr(cfg_mod, "DEPTH_HOLD_TARGET_MAX_M", None),
    )

    roll_defaults = AttitudeAxisConfig(kp=0.012, kd=0.002, out_limit=0.16)
    pitch_defaults = AttitudeAxisConfig(kp=0.012, kd=0.002, out_limit=0.16)
    yaw_defaults = AttitudeAxisConfig(kp=0.006, kd=0.0015, out_limit=0.12)
    return AutopilotConfig(
        depth_enable=bool(getattr(cfg_mod, "DEPTH_HOLD_ENABLE", True)),
        attitude_enable=bool(getattr(cfg_mod, "AUTOPILOT_ATTITUDE_ENABLE", True)),
        attitude_stale_s=float(getattr(cfg_mod, "AUTOPILOT_ATTITUDE_STALE_S", 0.50)),
        depth=depth_cfg,
        roll=axis_config("roll", roll_defaults),
        pitch=axis_config("pitch", pitch_defaults),
        yaw=axis_config("yaw", yaw_defaults),
    )
