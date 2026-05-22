"""Named-thruster mixers for Triton's vehicle geometry.

Mixers produce commands keyed by logical thruster names, not PWM channel
numbers. The channel mapping lives in `rov_config.CHANNEL_MAP` and is validated
by `motion.channel_map`, which keeps geometry math separate from wiring.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Hashable, Iterable, List, Sequence, Tuple

import numpy as np


class EightThrusterMixer:
    """Standard 8-thruster mixer.

    Thruster naming (fixed):
      - 4 horizontals (X pattern):  H_FL, H_FR, H_RL, H_RR
      - 4 verticals:               V_FL, V_FR, V_RL, V_RR

    The *physical channel mapping* is defined elsewhere (rov_config.CHANNEL_MAP).
    """

    def mix(self, cmd: Dict[str, float]) -> Dict[str, float]:
        """Convert a full 6-DOF command into named thruster requests."""

        surge = cmd["surge"]
        sway = cmd["sway"]
        heave = cmd["heave"]
        yaw = cmd["yaw"]
        pitch = cmd["pitch"]
        roll = cmd["roll"]

        # horizontals
        h_fl = surge + sway + yaw
        h_fr = surge - sway - yaw
        h_rl = surge - sway + yaw
        h_rr = surge + sway - yaw

        # verticals
        v_fl = heave - pitch - roll
        v_fr = heave - pitch + roll
        v_rl = heave + pitch - roll
        v_rr = heave + pitch + roll

        return {
            "H_FL": h_fl,
            "H_FR": h_fr,
            "H_RL": h_rl,
            "H_RR": h_rr,
            "V_FL": v_fl,
            "V_FR": v_fr,
            "V_RL": v_rl,
            "V_RR": v_rr,
        }


@dataclass(frozen=True)
class ThrusterGeometry:
    """Physical contribution model for one named thruster.

    Coordinates are vehicle-relative:
      +x forward, +y right, +z down.

    `direction` is the physical force direction caused by a positive logical
    command after the configured thruster reversal is accounted for.
    """

    name: str
    position_m: Tuple[float, float, float]
    direction: Tuple[float, float, float]
    scale: float = 1.0


class GeometricThrusterMixer:
    """Least-squares thruster allocator built from vehicle geometry.

    The public command convention matches `build_6dof()`:
      surge +forward, sway +right, heave +up,
      roll/pitch/yaw matching the legacy mixer command signs.

    The allocation matrix is row-normalized so rotational commands live in the
    same normalized command space as translational commands.
    """

    AXES = ("surge", "sway", "heave", "roll", "pitch", "yaw")

    def __init__(
        self,
        thrusters: Sequence[ThrusterGeometry],
        *,
        axis_weights: Mapping[str, float] | None = None,
        regularization: float = 0.015,
    ):
        self.thrusters = [t for t in thrusters]
        if not self.thrusters:
            raise ValueError("GeometricThrusterMixer requires at least one thruster")
        self.names = [t.name for t in self.thrusters]
        self.axis_weights = {axis: float((axis_weights or {}).get(axis, 1.0)) for axis in self.AXES}
        self.regularization = max(0.0, float(regularization))
        self._matrix_raw = self._build_matrix(self.thrusters)
        self._row_scales = self._compute_row_scales(self._matrix_raw)
        self._matrix = self._matrix_raw / self._row_scales[:, None]
        self._last_diag: Dict[str, Any] = {}

    @classmethod
    def _build_matrix(cls, thrusters: Sequence[ThrusterGeometry]) -> np.ndarray:
        cols = []
        for t in thrusters:
            px, py, pz = [float(v) for v in t.position_m]
            dx, dy, dz = cls._unit(t.direction)
            scale = float(t.scale)
            fx = dx * scale
            fy = dy * scale
            fz = dz * scale

            # Physical moment in +x forward, +y right, +z down coordinates.
            mx = (py * fz) - (pz * fy)
            my = (pz * fx) - (px * fz)
            mz = (px * fy) - (py * fx)

            cols.append(
                [
                    fx,   # surge: +forward
                    fy,   # sway: +right
                    -fz,  # heave command is +up while z coordinate is +down
                    -mx,  # signs chosen to preserve the legacy command convention
                    -my,
                    mz,
                ]
            )
        return np.array(cols, dtype=float).T

    @staticmethod
    def _unit(values: Sequence[float]) -> Tuple[float, float, float]:
        x, y, z = [float(v) for v in values]
        mag = math.sqrt((x * x) + (y * y) + (z * z))
        if mag <= 1e-9:
            raise ValueError("thruster direction vector must be non-zero")
        return x / mag, y / mag, z / mag

    @staticmethod
    def _compute_row_scales(matrix: np.ndarray) -> np.ndarray:
        scales = np.max(np.abs(matrix), axis=1)
        scales[scales <= 1e-9] = 1.0
        return scales

    def mix(self, cmd: Dict[str, float]) -> Dict[str, float]:
        desired = np.array([float((cmd or {}).get(axis, 0.0) or 0.0) for axis in self.AXES], dtype=float)
        weights = np.array([max(0.0, float(self.axis_weights.get(axis, 1.0))) for axis in self.AXES], dtype=float)
        weighted_matrix = self._matrix * weights[:, None]
        weighted_desired = desired * weights

        if self.regularization > 0.0:
            damp = math.sqrt(self.regularization) * np.eye(len(self.names), dtype=float)
            lhs = np.vstack([weighted_matrix, damp])
            rhs = np.concatenate([weighted_desired, np.zeros(len(self.names), dtype=float)])
        else:
            lhs = weighted_matrix
            rhs = weighted_desired

        solution, *_ = np.linalg.lstsq(lhs, rhs, rcond=None)
        out = {name: float(solution[i]) for i, name in enumerate(self.names)}
        self._last_diag = self.diagnostics(cmd, out)
        return out

    def allocated_wrench(self, thr: Mapping[str, float]) -> Dict[str, float]:
        vec = np.array([float((thr or {}).get(name, 0.0) or 0.0) for name in self.names], dtype=float)
        achieved = self._matrix @ vec
        return {axis: float(achieved[i]) for i, axis in enumerate(self.AXES)}

    def diagnostics(self, cmd: Mapping[str, float], thr: Mapping[str, float]) -> Dict[str, Any]:
        desired = {axis: float((cmd or {}).get(axis, 0.0) or 0.0) for axis in self.AXES}
        achieved = self.allocated_wrench(thr or {})
        residual = {axis: float(desired[axis] - achieved.get(axis, 0.0)) for axis in self.AXES}
        return {
            "axes": list(self.AXES),
            "thrusters": list(self.names),
            "desired": desired,
            "achieved": achieved,
            "residual": residual,
            "regularization": float(self.regularization),
            "axis_weights": dict(self.axis_weights),
        }

    @property
    def last_diagnostics(self) -> Dict[str, Any]:
        return dict(self._last_diag)


def _default_horizontal_direction(name: str, angle_deg_from_forward: float) -> Tuple[float, float, float]:
    c = math.cos(math.radians(float(angle_deg_from_forward)))
    s = math.sin(math.radians(float(angle_deg_from_forward)))
    sway_sign = {
        "H_FL": 1.0,
        "H_FR": -1.0,
        "H_RL": -1.0,
        "H_RR": 1.0,
    }.get(str(name))
    if sway_sign is None:
        raise ValueError(f"cannot derive horizontal direction for {name!r}")
    return (c, sway_sign * s, 0.0)


def _default_direction(name: str, angle_deg_from_forward: float) -> Tuple[float, float, float]:
    if str(name).startswith("H_"):
        return _default_horizontal_direction(str(name), angle_deg_from_forward)
    if str(name).startswith("V_"):
        # Positive logical vertical command is vehicle-up.
        return (0.0, 0.0, -1.0)
    raise ValueError(f"cannot derive thruster direction for {name!r}")


def geometric_mixer_from_config(cfg_mod: Any) -> GeometricThrusterMixer:
    """Build a geometric mixer from `rov_config` values."""

    raw = getattr(cfg_mod, "THRUSTER_GEOMETRY", None)
    if not isinstance(raw, Mapping):
        raise ValueError("THRUSTER_GEOMETRY must be a mapping of thruster names to geometry entries")

    angle = float(getattr(cfg_mod, "GEOMETRIC_MIXER_HORIZONTAL_ANGLE_DEG_FROM_FORWARD", 30.0))
    thrusters: List[ThrusterGeometry] = []
    for name, entry_any in raw.items():
        entry = dict(entry_any or {})
        pos = entry.get("position_m")
        if pos is None:
            raise ValueError(f"THRUSTER_GEOMETRY[{name!r}] is missing position_m")
        direction = entry.get("direction")
        if direction is None or str(direction).strip().lower() == "auto":
            direction = _default_direction(str(name), angle)
        thrusters.append(
            ThrusterGeometry(
                name=str(name),
                position_m=tuple(float(v) for v in pos),  # type: ignore[arg-type]
                direction=tuple(float(v) for v in direction),  # type: ignore[arg-type]
                scale=float(entry.get("scale", 1.0)),
            )
        )

    weights = getattr(cfg_mod, "GEOMETRIC_MIXER_AXIS_WEIGHTS", {}) or {}
    regularization = float(getattr(cfg_mod, "GEOMETRIC_MIXER_REGULARIZATION", 0.015))
    return GeometricThrusterMixer(thrusters, axis_weights=weights, regularization=regularization)


def global_limit(thr: Mapping[Hashable, float], max_abs: float = 1.0) -> Dict[Hashable, float]:
    """Scale a set of thruster commands so none exceed `max_abs`.

    If any output would exceed the allowed magnitude, every command is scaled by
    the same factor. That preserves the commanded direction/shape while fitting
    inside the configured thrust envelope.
    """

    peak = max(abs(v) for v in thr.values()) if thr else 0.0
    if peak <= max_abs or peak == 0.0:
        return {k: max(-max_abs, min(max_abs, v)) for k, v in thr.items()}
    scale = max_abs / peak
    return {k: max(-max_abs, min(max_abs, v * scale)) for k, v in thr.items()}


class SimpleGroupMixer:
    """Bring-up mixer that is *name-based* (no channel numbers).

    This avoids the entire class of errors where channel numbering and mapping
    are duplicated in different places.

    - surge -> all horizontal thrusters (H_*)
    - heave -> all vertical thrusters (V_*)
    """

    def __init__(self, horizontal_thrusters: Iterable[str], vertical_thrusters: Iterable[str]):
        self.horizontal_thrusters: List[str] = [str(n) for n in horizontal_thrusters]
        self.vertical_thrusters: List[str] = [str(n) for n in vertical_thrusters]
        if len(self.horizontal_thrusters) != 4 or len(self.vertical_thrusters) != 4:
            raise ValueError(
                "SimpleGroupMixer expects exactly 4 horizontal and 4 vertical thruster names"
            )

    def mix(self, surge: float, heave: float) -> Dict[str, float]:
        """Map simple surge/heave commands to horizontal and vertical groups."""

        out: Dict[str, float] = {}
        for name in self.horizontal_thrusters:
            out[name] = float(surge)
        for name in self.vertical_thrusters:
            out[name] = float(heave)
        return out
