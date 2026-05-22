"""Import helpers for Blue Robotics Navigator binding layout differences."""

from __future__ import annotations

import importlib
import importlib.metadata as md
from types import ModuleType
from typing import Any, Dict, List


def import_navigator_module() -> ModuleType:
    """Import the real Navigator binding across package-layout variants.

    Some installs expose the API directly from ``bluerobotics_navigator``.
    Others ship an almost-empty package whose compiled extension lives at
    ``bluerobotics_navigator.bluerobotics_navigator``.
    """
    pkg = importlib.import_module("bluerobotics_navigator")
    if _has_navigator_api(pkg):
        return pkg

    try:
        sub = importlib.import_module("bluerobotics_navigator.bluerobotics_navigator")
    except Exception:
        return pkg
    return sub if _has_navigator_api(sub) else pkg


def navigator_api_summary(nav: Any) -> str:
    """Return a compact human-readable summary of a Navigator module."""

    pwm_symbols = sorted(x for x in dir(nav) if "pwm" in x.lower())

    versions: List[str] = []
    for dist in ("bluerobotics_navigator", "bluerobotics-navigator"):
        try:
            versions.append(f"{dist}={md.version(dist)}")
        except Exception:
            pass

    parts = [
        f"module={getattr(nav, '__file__', '<unknown>')}",
        f"versions={versions or ['<unknown>']}",
        f"has_init={hasattr(nav, 'init')}",
        f"has_set_pwm_freq_hz={hasattr(nav, 'set_pwm_freq_hz')}",
        f"has_set_pwm_enable={hasattr(nav, 'set_pwm_enable')}",
        f"has_set_pwm_channel_value={hasattr(nav, 'set_pwm_channel_value')}",
        f"has_PwmChannel={hasattr(nav, 'PwmChannel')}",
        f"pwm_symbols={pwm_symbols}",
    ]
    return "; ".join(parts)


def navigator_api_info(nav: Any) -> Dict[str, Any]:
    """Return structured feature/version details for a Navigator module."""

    versions: Dict[str, str | None] = {}
    for dist in ("bluerobotics_navigator", "bluerobotics-navigator"):
        try:
            versions[dist] = md.version(dist)
        except Exception:
            versions[dist] = None

    return {
        "module_file": getattr(nav, "__file__", "<unknown>"),
        "has_init": hasattr(nav, "init"),
        "has_PwmChannel": hasattr(nav, "PwmChannel"),
        "has_set_pwm_freq_hz": hasattr(nav, "set_pwm_freq_hz"),
        "has_set_pwm_enable": hasattr(nav, "set_pwm_enable"),
        "has_set_pwm_channel_value": hasattr(nav, "set_pwm_channel_value"),
        "pwm_symbols": sorted(x for x in dir(nav) if "pwm" in x.lower()),
        "versions": versions,
    }


def _has_navigator_api(mod: Any) -> bool:
    return any(
        hasattr(mod, attr)
        for attr in ("init", "set_pwm_freq_hz", "set_pwm_enable", "set_pwm_channel_value", "read_temp")
    )
