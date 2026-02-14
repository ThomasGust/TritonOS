# pilot_common.py
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Dict, Tuple, Any
import time

PILOT_SCHEMA_VERSION = 1


@dataclass
class PilotAxes:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0


@dataclass
class PilotButtons:
    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False
    lb: bool = False
    rb: bool = False
    win: bool = False
    menu: bool = False
    lstick: bool = False
    rstick: bool = False


@dataclass
class PilotFrame:
    schema: int = PILOT_SCHEMA_VERSION
    seq: int = 0
    ts: float = field(default_factory=lambda: time.time())
    axes: PilotAxes = field(default_factory=PilotAxes)
    buttons: PilotButtons = field(default_factory=PilotButtons)
    dpad: Tuple[int, int] = (0, 0)
    edges: Dict[str, str] = field(default_factory=dict)
    modes: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "type": "pilot",
            "schema": self.schema,
            "seq": self.seq,
            "ts": self.ts,
            "axes": asdict(self.axes),
            "buttons": asdict(self.buttons),
            "dpad": list(self.dpad),
            "edges": dict(self.edges),
            "modes": dict(self.modes),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "PilotFrame":
        return cls(
            schema=d.get("schema", PILOT_SCHEMA_VERSION),
            seq=d.get("seq", 0),
            ts=d.get("ts", time.time()),
            axes=PilotAxes(**d.get("axes", {})),
            buttons=PilotButtons(**d.get("buttons", {})),
            dpad=tuple(d.get("dpad", (0, 0))),
            edges=d.get("edges", {}) or {},
            modes=d.get("modes", {}) or {},
        )
