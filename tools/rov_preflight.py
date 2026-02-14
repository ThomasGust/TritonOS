#!/usr/bin/env python3
"""
ROV preflight / hardware sanity checker.

Run this on the Pi before a pool test to quickly answer:
- Do we see the expected cameras? What formats/resolutions do they advertise?
- Are v4l2 tools installed? Is GStreamer installed?
- Are required Python modules importable?
- Are /dev resources (video, i2c, gpio) present?

This tool is intentionally "best effort": it never hard-crashes if a tool is missing.
It prints a human-readable report by default, and can emit JSON via --json.

Examples:
  python3 tools/rov_preflight.py
  python3 tools/rov_preflight.py --min-cameras 2
  python3 tools/rov_preflight.py --json > preflight.json
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


def which(cmd: str) -> Optional[str]:
    return shutil.which(cmd)


def run_cmd(cmd: List[str], timeout_s: float = 4.0) -> Dict[str, Any]:
    """
    Run a command and capture output. Never raises on missing binaries/timeouts.
    """
    out: Dict[str, Any] = {"cmd": cmd, "ok": False, "returncode": None, "stdout": "", "stderr": "", "error": None}
    try:
        cp = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        out["returncode"] = cp.returncode
        out["stdout"] = (cp.stdout or "").strip()
        out["stderr"] = (cp.stderr or "").strip()
        out["ok"] = (cp.returncode == 0)
    except FileNotFoundError as e:
        out["error"] = f"not_found: {e}"
    except subprocess.TimeoutExpired as e:
        out["error"] = f"timeout: {e}"
        # some partial output may exist
        out["stdout"] = ((e.stdout or "") if isinstance(e.stdout, str) else "").strip()  # type: ignore
        out["stderr"] = ((e.stderr or "") if isinstance(e.stderr, str) else "").strip()  # type: ignore
    except Exception as e:
        out["error"] = f"exception: {e}"
    return out


def list_dev_glob(pattern: str) -> List[str]:
    return sorted([str(p) for p in Path("/dev").glob(pattern)])


def collect_video_info(timeout_s: float = 4.0) -> Dict[str, Any]:
    info: Dict[str, Any] = {"devices": [], "v4l2ctl": None}
    devices = list_dev_glob("video*")
    info["devices"] = devices

    v4l2 = which("v4l2-ctl")
    info["v4l2ctl"] = v4l2
    if not v4l2:
        return info

    # overall listing
    info["list_devices"] = run_cmd(["v4l2-ctl", "--list-devices"], timeout_s=timeout_s)

    # per device details (formats tend to be the most useful)
    per_dev: Dict[str, Any] = {}
    for dev in devices:
        per_dev[dev] = {
            "all": run_cmd(["v4l2-ctl", "-d", dev, "--all"], timeout_s=timeout_s),
            "formats": run_cmd(["v4l2-ctl", "-d", dev, "--list-formats-ext"], timeout_s=timeout_s),
        }
    info["per_device"] = per_dev
    return info


def collect_gstreamer_info(timeout_s: float = 4.0) -> Dict[str, Any]:
    info: Dict[str, Any] = {"gst_launch": which("gst-launch-1.0"), "gst_inspect": which("gst-inspect-1.0")}
    if info["gst_launch"]:
        info["gst_launch_version"] = run_cmd(["gst-launch-1.0", "--version"], timeout_s=timeout_s)
    if info["gst_inspect"]:
        # Useful plugins for your pipelines
        info["inspect_v4l2src"] = run_cmd(["gst-inspect-1.0", "v4l2src"], timeout_s=timeout_s)
        info["inspect_rtph264pay"] = run_cmd(["gst-inspect-1.0", "rtph264pay"], timeout_s=timeout_s)
        info["inspect_udpsink"] = run_cmd(["gst-inspect-1.0", "udpsink"], timeout_s=timeout_s)
    return info


def collect_bus_info(timeout_s: float = 3.0) -> Dict[str, Any]:
    info: Dict[str, Any] = {
        "i2c_devices": list_dev_glob("i2c-*"),
        "gpiochips": list_dev_glob("gpiochip*"),
        "spidev": list_dev_glob("spidev*"),
    }
    if which("i2cdetect"):
        info["i2cdetect_list"] = run_cmd(["i2cdetect", "-l"], timeout_s=timeout_s)
        # If we see i2c devices, do a quick scan on the first one (low risk).
        if info["i2c_devices"]:
            try:
                bus = Path(info["i2c_devices"][0]).name.replace("i2c-", "")
                info["i2cdetect_scan_bus0"] = run_cmd(["i2cdetect", "-y", bus], timeout_s=timeout_s)
            except Exception as e:
                info["i2cdetect_scan_bus0"] = {"ok": False, "error": f"exception: {e}", "cmd": []}
    return info


def collect_python_imports() -> Dict[str, Any]:
    """
    We don't touch hardware here; we only verify imports so you catch missing deps fast.
    """
    modules = [
        "zmq",
        "numpy",
        "bluerobotics_navigator",
        "gpiod",
    ]
    results: Dict[str, Any] = {}
    for m in modules:
        try:
            __import__(m)
            results[m] = {"ok": True}
        except Exception as e:
            results[m] = {"ok": False, "error": str(e)}
    return results


def collect_navigator_smoke() -> Dict[str, Any]:
    """
    Best-effort check that the Navigator is reachable.
    This *may* touch hardware; safe to run on the Pi.
    """
    result: Dict[str, Any] = {"ok": False, "error": None}
    try:
        from sensors.navigator import NavigatorBoard  # type: ignore
        nav = NavigatorBoard()
        # small read; don't spam
        data = nav.read()
        # Keep output minimal but useful
        result["ok"] = True
        result["keys"] = sorted(list(data.keys()))
    except Exception as e:
        result["error"] = str(e)
    return result


def collect(min_cameras: int, include_navigator: bool) -> Dict[str, Any]:
    report: Dict[str, Any] = {
        "host": platform.node(),
        "platform": platform.platform(),
        "python": sys.version.replace("\n", " "),
        "cwd": os.getcwd(),
        "video": collect_video_info(),
        "gstreamer": collect_gstreamer_info(),
        "buses": collect_bus_info(),
        "python_imports": collect_python_imports(),
    }
    if include_navigator:
        report["navigator_smoke"] = collect_navigator_smoke()

    # simple verdict
    cams = report.get("video", {}).get("devices", []) or []
    report["verdict"] = {
        "ok": (len(cams) >= int(min_cameras)),
        "cameras_found": len(cams),
        "min_cameras": int(min_cameras),
    }
    return report


def _print_report_human(report: Dict[str, Any]) -> None:
    v = report["verdict"]
    print("=== TritonOS Preflight ===")
    print(f"Host: {report.get('host')}")
    print(f"Platform: {report.get('platform')}")
    print(f"Python: {report.get('python')}")
    print("")
    print(f"Cameras (/dev/video*): {v['cameras_found']} found (min required: {v['min_cameras']})")
    for dev in report.get("video", {}).get("devices", []):
        print(f"  - {dev}")
    if report.get("video", {}).get("v4l2ctl"):
        print("v4l2-ctl: OK")
    else:
        print("v4l2-ctl: NOT FOUND (install v4l-utils for better camera introspection)")

    gst = report.get("gstreamer", {})
    print(f"GStreamer gst-launch-1.0: {'OK' if gst.get('gst_launch') else 'NOT FOUND'}")
    print(f"GStreamer gst-inspect-1.0: {'OK' if gst.get('gst_inspect') else 'NOT FOUND'}")

    imports = report.get("python_imports", {})
    bad = [k for k, r in imports.items() if not r.get("ok")]
    if bad:
        print("Python imports missing/failing:")
        for k in bad:
            print(f"  - {k}: {imports[k].get('error')}")
    else:
        print("Python imports: OK")

    if "navigator_smoke" in report:
        ns = report["navigator_smoke"]
        print(f"Navigator smoke: {'OK' if ns.get('ok') else 'FAIL'}")
        if ns.get("error"):
            print(f"  error: {ns['error']}")
        if ns.get("keys"):
            print(f"  keys: {', '.join(ns['keys'])}")

    print("")
    print(f"VERDICT: {'OK' if v['ok'] else 'NOT READY'}")
    if not v["ok"]:
        print("  - Not enough camera devices detected.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-cameras", type=int, default=1, help="Minimum number of /dev/video* devices expected")
    ap.add_argument("--json", action="store_true", help="Print JSON instead of human report")
    ap.add_argument("--include-navigator", action="store_true", help="Also attempt a Navigator hardware read")
    args = ap.parse_args()

    report = collect(min_cameras=args.min_cameras, include_navigator=args.include_navigator)

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_report_human(report)

    return 0 if report["verdict"]["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
