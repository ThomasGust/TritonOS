#!/usr/bin/env python3
"""
ZeroMQ RPC wrapper for gst_streamer with device discovery and v4l2 capabilities.

Run on the Pi:
    python3 gst_streamer_rpc.py --bind tcp://0.0.0.0:5555
"""

import argparse
import json
import logging
import traceback
import glob
import os
import subprocess
from shutil import which
import re
import time
from collections import defaultdict


import zmq

from video.gst_streamer import StreamManager, StreamConfig
import rov_config as rov_cfg
from video.tether import (
    pick_tether_iface,
    iface_ipv4_addr,
    iface_operstate,
    is_wifi_iface,
    ensure_host_route,
)

logger = logging.getLogger("gst_streamer_rpc")
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")

FPS_RE = re.compile(r"\(([\d.]+)\s*fps\)")

# USB port hint like "1.3.4" embedded in by-path patterns
USB_PORT_RE = re.compile(r"(\d+(?:\.\d+){2,})")


def _extract_usb_port_hint(device: str) -> str | None:
    """Best-effort extraction of a hub port hint like '1.3.4' from a device string.

    Works with patterns like:
      /dev/v4l/by-path/*1.3.4*video-index0
      usb-0000:01:00.0-1.3.4
    """
    s = str(device or "")
    m = USB_PORT_RE.search(s)
    return m.group(1) if m else None


def _sysfs_write(path: str, value: str) -> None:
    with open(path, "w") as f:
        f.write(value)


def _find_usb_device_ids_for_port(port_hint: str) -> list[str]:
    """Find sysfs USB device IDs matching a port hint.

    On Pi/Linux these often look like:
      1-1.3.4
    We return the basename(s) to be written into unbind/bind.
    """
    base = "/sys/bus/usb/devices"
    if not port_hint:
        return []

    # Fast path: glob for "*-<port_hint>" (e.g. "1-1.3.4")
    matches: list[str] = []
    for p in glob.glob(os.path.join(base, f"*-{port_hint}")):
        bn = os.path.basename(p)
        if ":" in bn:
            continue
        matches.append(bn)

    # If that didn't work, scan directories (some kernels name the root hub differently)
    if not matches:
        try:
            for bn in os.listdir(base):
                if ":" in bn:
                    continue
                if bn.endswith(f"-{port_hint}"):
                    matches.append(bn)
        except Exception:
            pass

    return sorted(set(matches))


def usb_rebind_port(port_hint: str, messages: list[str] | None = None) -> bool:
    """Best-effort unbind/bind for a USB device on a given port hint.

    Returns True if we successfully issued unbind+bind to at least one matching
    device ID. This does not guarantee the camera will enumerate.
    """
    msgs = messages if messages is not None else []
    dev_ids = _find_usb_device_ids_for_port(port_hint)
    if not dev_ids:
        msgs.append(f"USB rebind: no /sys/bus/usb/devices/*-{port_hint} entry found")
        return False

    unbind_path = "/sys/bus/usb/drivers/usb/unbind"
    bind_path = "/sys/bus/usb/drivers/usb/bind"

    ok_any = False
    for dev_id in dev_ids:
        try:
            msgs.append(f"USB rebind: unbind {dev_id}")
            _sysfs_write(unbind_path, dev_id)
            time.sleep(0.25)
            msgs.append(f"USB rebind: bind {dev_id}")
            _sysfs_write(bind_path, dev_id)
            ok_any = True
        except PermissionError:
            msgs.append(
                "USB rebind: permission denied writing to sysfs (run TritonOS video service as root / with CAP_SYS_ADMIN)"
            )
        except FileNotFoundError:
            msgs.append("USB rebind: sysfs bind/unbind paths not found")
        except Exception as e:
            msgs.append(f"USB rebind: failed for {dev_id}: {e}")

    return ok_any


def usb_reset_all_cameras(port_hint: str, messages: list[str] | None = None) -> bool:
    """Broader best-effort USB reset intended to recover multiple cameras.

    Strategy (small + conservative):
      1) If we can infer a *parent hub* for the failing camera (e.g. 1.3 from 1.3.4),
         unbind/bind that hub device. This resets all downstream ports.
      2) Fallback: unbind/bind each discovered camera port from /dev/v4l/by-path/*video-index0.

    Returns True if we successfully issued at least one unbind+bind operation.
    """
    msgs = messages if messages is not None else []
    port_hint = str(port_hint or "")

    # 1) Try parent hub reset (e.g. 1.3 from 1.3.4)
    parts = port_hint.split(".")
    if len(parts) >= 2:
        parent = ".".join(parts[:-1])
        msgs.append(f"USB reset: attempting hub rebind on parent port {parent} (from {port_hint})")
        if usb_rebind_port(parent, messages=msgs):
            msgs.append("USB reset: hub rebind issued (downstream devices will re-enumerate)")
            return True

    # 2) Fallback: rebind all ports we can see in /dev/v4l/by-path
    ok_any = False
    paths = sorted(glob.glob("/dev/v4l/by-path/*video-index0"))
    port_hints: set[str] = set()
    for p in paths:
        h = _extract_usb_port_hint(p)
        if h:
            port_hints.add(h)

    if not port_hints:
        msgs.append("USB reset: no camera ports discovered under /dev/v4l/by-path/*video-index0")
        return False

    msgs.append(f"USB reset: rebinding all discovered camera ports: {', '.join(sorted(port_hints))}")
    for h in sorted(port_hints):
        ok_any = usb_rebind_port(h, messages=msgs) or ok_any

    return ok_any

def _intervals_to_fps(interval_lines: list[str]) -> list[float]:
    """
    Turn lines like:
        "Interval: Discrete 0.033s (30.000 fps)"
    into: [30.0]
    If we can't parse the (...) part, we try 1/seconds.
    """
    fps_vals: list[float] = []
    for line in interval_lines or []:
        m = FPS_RE.search(line)
        if m:
            try:
                fps_vals.append(float(m.group(1)))
                continue
            except ValueError:
                pass
        # fallback: try to grab the "... 0.033s" bit
        if "Discrete" in line and "s" in line:
            # e.g. "Interval: Discrete 0.033s"
            parts = line.split()
            for p in parts:
                if p.endswith("s"):
                    try:
                        sec = float(p[:-1])
                        if sec > 0:
                            fps_vals.append(round(1.0 / sec, 3))
                    except ValueError:
                        pass
                    break
    # dedupe + sort desc (high fps first is usually nicer for a UI)
    out = sorted(set(fps_vals), reverse=True)
    return out


def build_structured_modes(parsed_formats: list[dict]) -> list[dict]:
    """
    Turn the raw parsed formats from parse_v4l2_formats_ext(...) into a
    topside-friendly structure.

    Input (today):
        [
          { "pixelformat": "MJPG", "description": "...",
            "resolutions": [
               {"width": 640, "height": 480, "intervals": [...]},
               ...
            ]
          },
          ...
        ]

    Output (new):
        [
          {
            "format": "MJPG",
            "description": "Motion-JPEG",
            "sizes": [
              {
                "width": 640,
                "height": 480,
                "fps": [30.0, 15.0, 10.0]
              },
              ...
            ]
          },
          ...
        ]
    """
    out: list[dict] = []
    for fmt in parsed_formats or []:
        pf = (fmt.get("pixelformat") or "").upper()
        desc = fmt.get("description")
        sizes = []
        for r in fmt.get("resolutions", []):
            w = r.get("width")
            h = r.get("height")
            if not w or not h:
                continue
            fps = _intervals_to_fps(r.get("intervals", []))
            sizes.append({
                "width": w,
                "height": h,
                "fps": fps,
            })
        out.append({
            "format": pf,
            "description": desc,
            "sizes": sizes,
        })
    return out


# ---------------------------------------------------------------------------
# StreamConfig helper
# ---------------------------------------------------------------------------

def streamconfig_from_dict(d: dict) -> StreamConfig:
    """Build a ``StreamConfig`` from a JSON/RPC argument dictionary."""

    return StreamConfig(
        name=d["name"],
        device=d.get("device", "/dev/v4l/by-path/*video-index0"),
        width=d.get("width", 1280),
        height=d.get("height", 720),
        fps=d.get("fps", 30),
        video_format=d.get("video_format", "mjpeg"),
        encode=d.get("encode", None),
        h264_bitrate=d.get("h264_bitrate", 4_000_000),
        h264_gop=d.get("h264_gop", 30),
        transport=d.get("transport", "udp"),
        host=d.get("host", None),
        port=d.get("port", 5000),
        bind_address=d.get("bind_address", None),
        rtp_pt_jpeg=d.get("rtp_pt_jpeg", 26),
        rtp_pt_h264=d.get("rtp_pt_h264", 96),
        latency_ms=d.get("latency_ms", 60),
        sync=d.get("sync", False),
        extra=d.get("extra", {}),
    )


def _enforce_tether_for_video(scfg: StreamConfig) -> StreamConfig:
    """Best-effort tether enforcement for UDP video streams.

    - Sets scfg.bind_address to a tether IPv4 when configured.
    - Optionally installs a host route to the topside receive IP through tether.

    This does *not* disable Wi‑Fi; it only tries to keep video traffic on tether.
    """

    try:
        if not bool(getattr(rov_cfg, "VIDEO_ENFORCE_TETHER", False)):
            return scfg
        if (scfg.transport or "udp") != "udp":
            return scfg
        if not scfg.host:
            return scfg

        iface = pick_tether_iface(getattr(rov_cfg, "VIDEO_TETHER_IFACE", None))
        if not iface:
            return scfg

        src_ip = getattr(rov_cfg, "VIDEO_TETHER_SRC_IP", None) or iface_ipv4_addr(iface)
        if src_ip and not scfg.bind_address:
            scfg.bind_address = src_ip

        if bool(getattr(rov_cfg, "VIDEO_ENFORCE_HOST_ROUTE", False)):
            # Try to pin a /32 route to the topside receive host via tether.
            ensure_host_route(str(scfg.host), iface, src_ip=src_ip)
    except Exception:
        # Never fail the RPC due to tether enforcement issues.
        return scfg

    return scfg


# ---------------------------------------------------------------------------
# v4l2 probing / parsing
# ---------------------------------------------------------------------------

def has_v4l2ctl() -> bool:
    """Return True when the system has the v4l2 probing CLI installed."""

    return which("v4l2-ctl") is not None


def run_v4l2_all(dev_path: str) -> str | None:
    """Return ``v4l2-ctl --all`` output for a device, or None on failure."""

    if not has_v4l2ctl():
        return None
    try:
        return subprocess.check_output(
            ["v4l2-ctl", "-d", dev_path, "--all"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None


def run_v4l2_formats_ext(dev_path: str) -> str | None:
    """Return extended V4L2 format output for a device, or None on failure."""

    if not has_v4l2ctl():
        return None
    try:
        return subprocess.check_output(
            ["v4l2-ctl", "-d", dev_path, "--list-formats-ext"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None


def parse_v4l2_formats_ext(text: str) -> list[dict]:
    """
    Parse output of `v4l2-ctl --list-formats-ext` in BOTH styles:
    1) "Pixel Format: 'MJPG' (Motion-JPEG)"
    2) "[0]: 'MJPG' (Motion-JPEG)"

    Returns a list:
    [
      {
        "pixelformat": "MJPG",
        "description": "Motion-JPEG",
        "resolutions": [
          {"width": 640, "height": 480, "intervals": ["Interval: Discrete 0.033s (30.000 fps)"]},
          ...
        ]
      },
      ...
    ]
    """
    if not text:
        return []

    formats: list[dict] = []
    current_fmt: dict | None = None
    current_res: dict | None = None

    for line in text.splitlines():
        raw = line.rstrip("\n")
        if not raw.strip():
            continue

        s = raw.lstrip()

        # ----------- FORMAT LINES -----------
        # Style 1: "Pixel Format: 'MJPG' (Motion-JPEG)"
        if s.startswith("Pixel Format:"):
            # flush previous
            if current_fmt:
                if current_res:
                    current_fmt.setdefault("resolutions", []).append(current_res)
                    current_res = None
                formats.append(current_fmt)

            parts = s.split("Pixel Format:", 1)[1].strip()
            pix = None
            desc = None
            if "(" in parts:
                pix_part, desc_part = parts.split("(", 1)
                pix = pix_part.strip().strip("'").strip()
                desc = desc_part.strip("() ").strip()
            else:
                pix = parts.strip().strip("'").strip()

            current_fmt = {"pixelformat": pix}
            if desc:
                current_fmt["description"] = desc
            current_res = None
            continue

        # Style 2: "[0]: 'MJPG' (Motion-JPEG)"
        # Usually starts with "[N]:" and always has a quoted fourcc
        if s.startswith("[") and "]: " in s and "'" in s:
            # flush previous
            if current_fmt:
                if current_res:
                    current_fmt.setdefault("resolutions", []).append(current_res)
                    current_res = None
                formats.append(current_fmt)

            # example: "[1]: 'MJPG' (Motion-JPEG)"
            after_colon = s.split("]:", 1)[1].strip()
            # "'MJPG' (Motion-JPEG)"
            if "(" in after_colon:
                pix_part, desc_part = after_colon.split("(", 1)
                pix = pix_part.strip().strip("'").strip()
                desc = desc_part.strip("() ").strip()
            else:
                pix = after_colon.strip().strip("'").strip()
                desc = None

            current_fmt = {"pixelformat": pix}
            if desc:
                current_fmt["description"] = desc
            current_res = None
            continue

        # ----------- RESOLUTION LINES -----------
        # e.g. "Size: Discrete 640x480"
        if "Size:" in s and "Discrete" in s:
            if current_res:
                current_fmt.setdefault("resolutions", []).append(current_res)
            parts = s.split("Discrete", 1)[1].strip()
            if "x" in parts:
                w, h = parts.split("x", 1)
                current_res = {
                    "width": int(w),
                    "height": int(h),
                    "intervals": []
                }
            continue

        # ----------- INTERVAL LINES -----------
        if "Interval:" in s and "Discrete" in s and current_res is not None:
            current_res["intervals"].append(s)
            continue

    # flush tail
    if current_fmt:
        if current_res:
            current_fmt.setdefault("resolutions", []).append(current_res)
        formats.append(current_fmt)
    
    return formats



def device_label_from_sys(dev_path: str) -> str | None:
    """Read the Linux sysfs camera label for a ``/dev/video*`` node."""

    try:
        real = os.path.realpath(dev_path)
        base = os.path.basename(real)
        sys_name_path = f"/sys/class/video4linux/{base}/name"
        if os.path.exists(sys_name_path):
            with open(sys_name_path, "r") as f:
                return f.read().strip()
    except Exception:
        return None
    return None


def classify_formats(parsed_formats: list[dict]) -> dict:
    """Summarize parsed V4L2 formats into coarse capability booleans."""

    fmts = { (f.get("pixelformat") or "").upper() for f in parsed_formats }

    raw_candidates = {"YUYV", "YUY2", "UYVY", "NV12", "BGR3", "RGB3", "RGBP"}

    supports_mjpeg = any(x in fmts for x in ("MJPG", "MJPEG", "JPEG"))
    supports_h264 = any(x in fmts for x in ("H264", "H.264"))  # some cams show weird text
    supports_raw = any(x in fmts for x in raw_candidates)

    return {
        "supports_mjpeg": supports_mjpeg,
        "supports_h264": supports_h264,
        "supports_raw": supports_raw,
    }

def probe_v4l2_device(dev_path: str) -> dict:
    """Collect existence, labels, raw caps, parsed formats, and GUI modes."""

    info = {
        "device": dev_path,
        "exists": os.path.exists(dev_path),
    }

    label = device_label_from_sys(dev_path)
    if label:
        info["label"] = label

    v4l2_all = run_v4l2_all(dev_path)
    if v4l2_all:
        info["v4l2_all"] = v4l2_all

    fmts_text = run_v4l2_formats_ext(dev_path)
    if fmts_text:
        parsed = parse_v4l2_formats_ext(fmts_text)
        info["formats_ext_raw"] = fmts_text   # keep for debugging
        info["formats"] = parsed
        info["caps_flags"] = classify_formats(parsed)
        # 👇 NEW: structured, GUI-friendly block
        info["modes"] = build_structured_modes(parsed)
    else:
        info["formats"] = []
        info["caps_flags"] = {
            "supports_mjpeg": False,
            "supports_h264": False,
            "supports_raw": False,
        }
        info["modes"] = []

    return info


def list_video_devices() -> list[dict]:
    """Probe available camera devices, preferring stable by-path symlinks."""

    # Prefer stable, per-port symlinks (one per physical camera).
    by_path = sorted(glob.glob("/dev/v4l/by-path/*video-index0"))
    if by_path:
        return [probe_v4l2_device(d) for d in by_path]

    # Fallback: raw /dev/video* nodes (may include duplicates per camera).
    devices = sorted(glob.glob("/dev/video*"))
    return [probe_v4l2_device(d) for d in devices]


# ---------------------------------------------------------------------------
# main RPC loop
# ---------------------------------------------------------------------------

def start_video_rpc():
    """Run the blocking ZeroMQ REP loop that manages camera streams."""

    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="tcp://0.0.0.0:5555")
    args = ap.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(args.bind)
    logger.info("RPC server listening on %s", args.bind)

    mgr = StreamManager()

    while True:
        raw = sock.recv()
        try:
            req = json.loads(raw.decode("utf-8"))
        except Exception:
            sock.send_json({"ok": False, "error": "invalid json"})
            continue

        cmd = req.get("cmd")
        args = req.get("args", {}) or {}

        try:
            if cmd == "ping":
                sock.send_json({"ok": True, "data": "pong"})
                continue

            elif cmd == "net_info":
                # Helpful for topside diagnostics (which iface is tether, what IPs exist).
                try:
                    import os

                    ifaces = []
                    for iface in os.listdir("/sys/class/net"):
                        if iface == "lo":
                            continue
                        ifaces.append({
                            "iface": iface,
                            "is_wifi": bool(is_wifi_iface(iface)),
                            "state": iface_operstate(iface) or "unknown",
                            "ip": iface_ipv4_addr(iface),
                        })
                    tif = pick_tether_iface(getattr(rov_cfg, "VIDEO_TETHER_IFACE", None))
                    sock.send_json({
                        "ok": True,
                        "data": {
                            "ifaces": ifaces,
                            "tether_iface": tif,
                            "tether_ip": iface_ipv4_addr(tif) if tif else None,
                            "enforce_tether": bool(getattr(rov_cfg, "VIDEO_ENFORCE_TETHER", False)),
                        },
                    })
                except Exception:
                    sock.send_json({"ok": True, "data": {"ifaces": []}})
                continue

            elif cmd == "start_stream":
                scfg = streamconfig_from_dict(args)
                scfg = _enforce_tether_for_video(scfg)
                current = mgr.list_streams()
                if scfg.name in current:
                    logger.warning(
                        "start_stream: stream '%s' already exists, restarting it", scfg.name
                    )
                    try:
                        mgr.stop_stream(scfg.name)
                    except Exception:
                        logger.exception(
                            "start_stream: failed to stop existing stream '%s' before restart",
                            scfg.name,
                        )

                # Attempt start. If it fails (e.g., camera not enumerated yet),
                # try a best-effort USB unbind/bind on the inferred hub port.
                messages: list[str] = []
                last_err: str | None = None

                def _try_start_once() -> None:
                    mgr.start_stream(scfg)

                try:
                    _try_start_once()
                    sock.send_json({"ok": True, "data": {"name": scfg.name}})
                except Exception as e:
                    last_err = str(e)
                    port_hint = _extract_usb_port_hint(scfg.device)
                    if not port_hint:
                        sock.send_json({"ok": False, "error": last_err})
                        continue

                    retries = int(getattr(rov_cfg, "VIDEO_USB_REBIND_RETRIES", 3))
                    delay_s = float(getattr(rov_cfg, "VIDEO_USB_REBIND_DELAY_S", 1.0))

                    for i in range(max(0, retries)):
                        messages.append(
                            f"Video start failed for '{scfg.name}' (device={scfg.device}). "
                            f"Attempting USB rebind on port {port_hint} ({i+1}/{retries})…"
                        )
                        usb_rebind_port(port_hint, messages=messages)
                        time.sleep(max(0.0, delay_s))
                        try:
                            _try_start_once()
                            messages.append(f"Video stream '{scfg.name}' started after USB rebind")
                            sock.send_json({"ok": True, "data": {"name": scfg.name, "messages": messages}})
                            break
                        except Exception as e2:
                            last_err = str(e2)
                            # try again
                            continue
                    else:
                        # Rebind attempts exhausted. As a next (broader) step, try a hub-level
                        # reset that should cause *all* downstream cameras to re-enumerate.
                        messages.append(
                            f"Video stream '{scfg.name}' still failed after {retries} USB rebind attempts. "
                            f"Attempting broader USB reset…"
                        )
                        did_reset = usb_reset_all_cameras(port_hint, messages=messages)
                        if did_reset:
                            time.sleep(max(0.0, delay_s))
                            try:
                                _try_start_once()
                                messages.append(f"Video stream '{scfg.name}' started after broader USB reset")
                                sock.send_json({"ok": True, "data": {"name": scfg.name, "messages": messages}})
                                continue
                            except Exception as e3:
                                last_err = str(e3)
                        else:
                            messages.append("Broader USB reset could not be issued (no matching sysfs devices)")

                        sock.send_json({"ok": False, "error": last_err or "failed to start stream", "messages": messages})

            elif cmd == "stop_stream":
                name = args["name"]
                current = mgr.list_streams()
                if name not in current:
                    # be nice: stopping a non-existent stream is not an error
                    logger.info("stop_stream: stream '%s' not found; ignoring", name)
                    sock.send_json({"ok": True, "data": {"note": "not running"}})
                else:
                    try:
                        mgr.stop_stream(name)
                        sock.send_json({"ok": True})
                    except Exception:
                        # don't crash RPC — just report
                        logger.exception("stop_stream: failed to stop '%s'", name)
                        sock.send_json({"ok": False, "error": f"failed to stop '{name}'"})

            elif cmd == "update_stream":
                name = args["name"]
                updates = {k: v for k, v in args.items() if k != "name"}
                current = mgr.list_streams()
                if name not in current:
                    # we can either treat this as "nothing to update" or "start it"
                    # let's choose "nothing to update" to stay conservative
                    logger.info("update_stream: stream '%s' not found; ignoring update", name)
                    sock.send_json({"ok": True, "data": {"note": "not running"}})
                else:
                    try:
                        mgr.update_stream(name, **updates)
                        sock.send_json({"ok": True})
                    except Exception:
                        # e.g. update requires rebuild and something failed
                        logger.exception("update_stream: failed to update '%s'", name)
                        sock.send_json({"ok": False, "error": f"failed to update '{name}'"})

            elif cmd == "list_streams":
                current = mgr.list_streams()
                out = {n: vars(cfg) for n, cfg in current.items()}
                sock.send_json({"ok": True, "data": out})

            # NEW: list all devices, shallow+caps
            elif cmd == "list_devices":
                devices = list_video_devices()
                sock.send_json({"ok": True, "data": devices})

            # NEW: get caps for a single device
            elif cmd == "get_device_caps":
                dev_path = args.get("device", "/dev/v4l/by-path/*video-index0")
                info = probe_v4l2_device(dev_path)
                sock.send_json({"ok": True, "data": info})

            else:
                sock.send_json({"ok": False, "error": f"unknown cmd: {cmd}"})

        except Exception as e:
            logger.exception("command failed")
            sock.send_json({
                "ok": False,
                "error": str(e),
                "trace": traceback.format_exc()
            })


if __name__ == "__main__":
    start_video_rpc()
