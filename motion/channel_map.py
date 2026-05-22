"""Single source-of-truth for PWM channel mapping.

Edit only `rov_config.CHANNEL_MAP` (physical channels 1..16).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional


_REQUIRED_THRUSTER_NAMES = (
    "H_FL", "H_FR", "H_RL", "H_RR",
    "V_FL", "V_FR", "V_RL", "V_RR",
)

_DEFAULT_CHANNEL_MAP: Dict[str, Dict[str, int]] = {
    "thrusters": {
        "H_FL": 8,
        "H_FR": 6,
        "H_RL": 7,
        "H_RR": 2,
        "V_FL": 3,
        "V_FR": 4,
        "V_RL": 9,
        "V_RR": 1,
    },
    "aux": {
        "lights": 5,
        "wrist_rotate": 10,
        "gripper_left": 12,
        "gripper_right": 13,
    },
}


def _as_int_map(d: Mapping[Any, Any]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for k, v in (d or {}).items():
        out[str(k)] = int(v)
    return out


def _default_thrusters() -> Dict[str, int]:
    return dict(_DEFAULT_CHANNEL_MAP["thrusters"])


def _default_aux() -> Dict[str, int]:
    return dict(_DEFAULT_CHANNEL_MAP["aux"])


@dataclass(frozen=True)
class ChannelMap:
    """Validated mapping for thrusters + auxiliary PWM outputs."""

    thrusters: Dict[str, int]
    aux: Dict[str, int]

    @classmethod
    def from_config(cls, cfg: Any) -> "ChannelMap":
        """Load mapping from config.

        Preferred source is `cfg.CHANNEL_MAP`.
        Falls back to legacy variables for backward compatibility.
        """
        m = getattr(cfg, "CHANNEL_MAP", None)
        if isinstance(m, dict):
            thr = _as_int_map(m.get("thrusters", {}))
            aux = _as_int_map(m.get("aux", {}))
        else:
            thr = _as_int_map(getattr(cfg, "THRUSTER_CHANNELS", {}) or {})
            aux = {}
            lights = getattr(cfg, "LIGHTS_PWM_CHANNEL", None)
            if lights is not None:
                aux["lights"] = int(lights)

        # Backward-compatible safety net: if the active config is missing the
        # newer channel-map section entirely, fall back to the repo's standard
        # TritonOS layout so startup still succeeds with older configs.
        if not thr:
            thr = _default_thrusters()
        if not aux:
            aux = _default_aux()

        cm = cls(thrusters=thr, aux=aux)
        cm._validate()
        return cm

    # ---- derived convenience ----
    @property
    def horizontal_thrusters(self) -> List[str]:
        """Return configured thruster names that participate in horizontal motion."""

        # Convention: H_* are horizontals
        return [n for n in _REQUIRED_THRUSTER_NAMES if n.startswith("H_") and n in self.thrusters]

    @property
    def vertical_thrusters(self) -> List[str]:
        """Return configured thruster names that participate in vertical motion."""

        # Convention: V_* are verticals
        return [n for n in _REQUIRED_THRUSTER_NAMES if n.startswith("V_") and n in self.thrusters]

    @property
    def motor_channels(self) -> List[int]:
        """Return all physical PWM channels used by thrusters."""

        return sorted({int(v) for v in self.thrusters.values()})

    @property
    def lights_channel(self) -> Optional[int]:
        """Return the configured lights PWM channel, if one exists."""

        return int(self.aux["lights"]) if "lights" in self.aux else None

    # ---- validation ----
    def _validate(self) -> None:
        # Require the standard 8-thruster naming so the mixer is unambiguous.
        missing = [n for n in _REQUIRED_THRUSTER_NAMES if n not in self.thrusters]
        if missing:
            raise ValueError(
                "CHANNEL_MAP.thrusters must define all 8 required thruster names "
                f"{list(_REQUIRED_THRUSTER_NAMES)}. Missing: {missing}."
            )

        # Physical channel numbering only (1..16)
        all_ch = list(self.thrusters.values()) + list(self.aux.values())
        bad = [c for c in all_ch if int(c) < 1 or int(c) > 16]
        if bad:
            raise ValueError(f"PWM channels must be physical 1..16. Out of range: {sorted(set(bad))}")

        # Uniqueness / overlap checks
        thr_vals = [int(v) for v in self.thrusters.values()]
        if len(set(thr_vals)) != len(thr_vals):
            dup = sorted({v for v in thr_vals if thr_vals.count(v) > 1})
            raise ValueError(f"Duplicate thruster PWM channels: {dup}. Each thruster must be unique.")

        aux_vals = [int(v) for v in self.aux.values()]
        overlap = sorted(set(thr_vals).intersection(aux_vals))
        if overlap:
            raise ValueError(f"Aux PWM channel(s) overlap thrusters: {overlap}. Fix CHANNEL_MAP.")
