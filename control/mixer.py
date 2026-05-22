"""Named-thruster mixers for Triton's vehicle geometry.

Mixers produce commands keyed by logical thruster names, not PWM channel
numbers. The channel mapping lives in `rov_config.CHANNEL_MAP` and is validated
by `motion.channel_map`, which keeps geometry math separate from wiring.
"""

from __future__ import annotations

from typing import Dict, Mapping, Hashable, Iterable, List


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
