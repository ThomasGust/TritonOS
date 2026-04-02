#!/usr/bin/env python3
"""Inspect the installed bluerobotics_navigator PWM API without driving outputs.

Run on the Pi:
  sudo .venv/bin/python -m tools.navigator_api_probe
  sudo .venv/bin/python -m tools.navigator_api_probe --try-init
"""

from __future__ import annotations

import argparse
import importlib.metadata as md
import json
import sys
from typing import Any, Dict, List


def _versions() -> Dict[str, str]:
    out: Dict[str, str] = {}
    for dist in ("bluerobotics_navigator", "bluerobotics-navigator"):
        try:
            out[dist] = md.version(dist)
        except Exception:
            pass
    return out


def _probe_nav(try_init: bool) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "python": sys.version.replace("\n", " "),
        "versions": _versions(),
    }
    try:
        import bluerobotics_navigator as nav
    except Exception as e:
        result["ok"] = False
        result["import_error"] = str(e)
        return result

    pwm_symbols: List[str] = sorted(x for x in dir(nav) if "pwm" in x.lower())
    result.update(
        {
            "ok": True,
            "module_file": getattr(nav, "__file__", "<unknown>"),
            "has_init": hasattr(nav, "init"),
            "has_PwmChannel": hasattr(nav, "PwmChannel"),
            "has_set_pwm_freq_hz": hasattr(nav, "set_pwm_freq_hz"),
            "has_set_pwm_enable": hasattr(nav, "set_pwm_enable"),
            "has_set_pwm_channel_value": hasattr(nav, "set_pwm_channel_value"),
            "pwm_symbols": pwm_symbols,
        }
    )

    if try_init and hasattr(nav, "init"):
        try:
            nav.init()
            result["init_ok"] = True
        except Exception as e:
            result["init_ok"] = False
            result["init_error"] = str(e)

    return result


def main() -> None:
    ap = argparse.ArgumentParser(description="Inspect the installed bluerobotics_navigator PWM API.")
    ap.add_argument("--json", action="store_true", help="Emit JSON instead of human-readable text.")
    ap.add_argument("--try-init", action="store_true", help="Also call nav.init() as a smoke test.")
    args = ap.parse_args()

    result = _probe_nav(try_init=bool(args.try_init))
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return

    print("=== Navigator API Probe ===")
    print("Python:", result.get("python"))
    print("Versions:", result.get("versions") or "<unknown>")
    if not result.get("ok"):
        print("Import error:", result.get("import_error"))
        return

    print("Module:", result.get("module_file"))
    print("has init:", result.get("has_init"))
    print("has PwmChannel:", result.get("has_PwmChannel"))
    print("has set_pwm_freq_hz:", result.get("has_set_pwm_freq_hz"))
    print("has set_pwm_enable:", result.get("has_set_pwm_enable"))
    print("has set_pwm_channel_value:", result.get("has_set_pwm_channel_value"))
    print("PWM symbols:", result.get("pwm_symbols"))
    if "init_ok" in result:
        print("init ok:", result.get("init_ok"))
        if result.get("init_ok") is False:
            print("init error:", result.get("init_error"))


if __name__ == "__main__":
    main()
