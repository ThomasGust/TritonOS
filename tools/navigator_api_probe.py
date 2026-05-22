#!/usr/bin/env python3
"""Inspect the installed bluerobotics_navigator PWM API without driving outputs.

Run on the Pi:
  sudo .venv/bin/python -m tools.navigator_api_probe
  sudo .venv/bin/python -m tools.navigator_api_probe --try-init
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict

from utils.navigator_import import import_navigator_module, navigator_api_info


def _probe_nav(try_init: bool) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "python": sys.version.replace("\n", " "),
    }
    try:
        nav = import_navigator_module()
    except Exception as e:
        result["ok"] = False
        result["import_error"] = str(e)
        return result

    result.update({"ok": True, **navigator_api_info(nav)})

    if try_init and hasattr(nav, "init"):
        try:
            nav.init()
            result["init_ok"] = True
        except Exception as e:
            result["init_ok"] = False
            result["init_error"] = str(e)

    return result


def main() -> None:
    """Print or serialize Navigator binding capabilities."""

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
