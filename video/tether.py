"""ROV-side helpers for preferring the tether interface.

TritonOS runs on Linux. These helpers are designed to:
  - Identify a likely tether (wired) interface, and
  - (Optionally) pin a route to the topside video receive IP through that interface.

All operations are best-effort and should never raise exceptions to callers.
"""

from __future__ import annotations

import os
import socket
import struct
import subprocess
from typing import Optional


def is_wifi_iface(iface: str) -> bool:
    """Return True when Linux exposes the interface as wireless."""

    return os.path.isdir(f"/sys/class/net/{iface}/wireless")


def iface_operstate(iface: str) -> Optional[str]:
    """Read the Linux operstate string for an interface, if available."""

    try:
        with open(f"/sys/class/net/{iface}/operstate", "r", encoding="utf-8", errors="ignore") as f:
            return f.read().strip()
    except Exception:
        return None


def iface_ipv4_addr(iface: str) -> Optional[str]:
    """Best-effort IPv4 address lookup for an interface (Linux)."""
    try:
        import fcntl  # Linux-only

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ifreq = struct.pack("256s", iface.encode("utf-8")[:15])
        res = fcntl.ioctl(s.fileno(), 0x8915, ifreq)  # SIOCGIFADDR
        ip = socket.inet_ntoa(res[20:24])
        return ip
    except Exception:
        return None


def pick_tether_iface(preferred_iface: Optional[str] = None) -> Optional[str]:
    """Pick a likely tether interface.

    Preference order:
      1) preferred_iface if provided and looks UP
      2) any UP, non-loopback, non-wifi interface
      3) any UP non-loopback interface
    """
    try:
        if preferred_iface:
            st = iface_operstate(preferred_iface)
            if st in ("up", "unknown"):
                return preferred_iface
    except Exception:
        pass

    # First, look for wired.
    try:
        for iface in os.listdir("/sys/class/net"):
            if iface == "lo":
                continue
            if is_wifi_iface(iface):
                continue
            st = iface_operstate(iface)
            if st in ("up", "unknown"):
                if iface_ipv4_addr(iface):
                    return iface
    except Exception:
        pass

    # Fallback: any up iface with IPv4
    try:
        for iface in os.listdir("/sys/class/net"):
            if iface == "lo":
                continue
            st = iface_operstate(iface)
            if st in ("up", "unknown"):
                if iface_ipv4_addr(iface):
                    return iface
    except Exception:
        pass

    return None


def ensure_host_route(dest_ip: str, iface: str, src_ip: Optional[str] = None) -> bool:
    """Ensure a /32 host route to dest_ip via iface.

    Requires root. Returns True if the command ran successfully.
    """
    try:
        cmd = ["ip", "route", "replace", f"{dest_ip}/32", "dev", iface]
        if src_ip:
            cmd += ["src", src_ip]
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception:
        return False
