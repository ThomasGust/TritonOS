# sensors/network.py
from __future__ import annotations

"""Network telemetry sensor.

Publishes a lightweight "net" message onto the existing sensor PUB stream so
the topside can display:
  - which interface is being used (tether vs wifi)
  - link state + nominal link speed
  - IP address
  - live RX/TX throughput (bytes/sec)
  - error/drop counters

Designed to avoid third‑party deps (psutil, iperf, etc.) and work on small
embedded Linux images.
"""

import os
import socket
import struct
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from sensors.base import BaseSensor


def _read_text(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return f.read().strip()
    except Exception:
        return None


def _is_wifi_iface(iface: str) -> bool:
    # Common Linux hint: /sys/class/net/<iface>/wireless exists for Wi‑Fi.
    return os.path.isdir(f"/sys/class/net/{iface}/wireless")


def _iface_operstate(iface: str) -> Optional[str]:
    return _read_text(f"/sys/class/net/{iface}/operstate")


def _iface_speed_mbps(iface: str) -> Optional[int]:
    # For ethernet devices this is usually a file containing an integer (or -1).
    txt = _read_text(f"/sys/class/net/{iface}/speed")
    if not txt:
        return None
    try:
        v = int(float(txt))
        if v <= 0:
            return None
        return v
    except Exception:
        return None


def _default_route_iface() -> Optional[str]:
    """Return the interface used for the default route (Linux)."""
    try:
        with open("/proc/net/route", "r", encoding="utf-8", errors="ignore") as f:
            # Iface  Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT
            for line in f.readlines()[1:]:
                parts = line.strip().split()
                if len(parts) < 2:
                    continue
                iface, dest = parts[0], parts[1]
                if dest == "00000000":
                    return iface
    except Exception:
        return None
    return None


def _fallback_up_iface() -> Optional[str]:
    """Pick any non-loopback interface that looks up."""
    try:
        for iface in os.listdir("/sys/class/net"):
            if iface == "lo":
                continue
            st = _iface_operstate(iface)
            if st in ("up", "unknown"):
                return iface
    except Exception:
        return None
    return None


def _iface_ipv4_addr(iface: str) -> Optional[str]:
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


@dataclass
class _DevCounters:
    rx_bytes: int
    rx_errs: int
    rx_drop: int
    tx_bytes: int
    tx_errs: int
    tx_drop: int


def _read_dev_counters(iface: str) -> Optional[_DevCounters]:
    """Parse /proc/net/dev for interface counters."""
    try:
        with open("/proc/net/dev", "r", encoding="utf-8", errors="ignore") as f:
            for line in f.readlines()[2:]:
                if ":" not in line:
                    continue
                name, rest = line.split(":", 1)
                name = name.strip()
                if name != iface:
                    continue
                fields = rest.strip().split()
                # rx: bytes packets errs drop fifo frame compressed multicast
                # tx: bytes packets errs drop fifo colls carrier compressed
                if len(fields) < 16:
                    return None
                rx_bytes = int(fields[0])
                rx_errs = int(fields[2])
                rx_drop = int(fields[3])
                tx_bytes = int(fields[8])
                tx_errs = int(fields[10])
                tx_drop = int(fields[11])
                return _DevCounters(
                    rx_bytes=rx_bytes,
                    rx_errs=rx_errs,
                    rx_drop=rx_drop,
                    tx_bytes=tx_bytes,
                    tx_errs=tx_errs,
                    tx_drop=tx_drop,
                )
    except Exception:
        return None
    return None


class NetworkStatsSensor(BaseSensor):
    """Publishes link/interface + throughput counters."""

    def __init__(
        self,
        rate_hz: float = 1.0,
        iface: Optional[str] = None,
        tether_prefixes: Tuple[str, ...] = ("eth", "en", "eno", "enp", "enx", "usb", "rndis", "cdc"),
    ):
        super().__init__(name="network", rate_hz=float(rate_hz))
        self._iface_override = iface
        self._tether_prefixes = tuple(tether_prefixes)

        self._last_t: Optional[float] = None
        self._last: Optional[_DevCounters] = None

    def _pick_iface(self) -> Optional[str]:
        if self._iface_override:
            return self._iface_override
        iface = _default_route_iface()
        if iface:
            return iface
        return _fallback_up_iface()

    def read(self) -> Dict[str, Any]:
        ts = time.time()
        iface = self._pick_iface()
        if not iface:
            return {
                "ts": ts,
                "sensor": self.name,
                "type": "net",
                "error": "no_iface",
            }

        wifi = _is_wifi_iface(iface)
        oper = _iface_operstate(iface) or "unknown"
        speed = _iface_speed_mbps(iface)
        ip = _iface_ipv4_addr(iface)
        counters = _read_dev_counters(iface)

        rx_bps = None
        tx_bps = None
        if counters is not None and self._last is not None and self._last_t is not None:
            dt = max(1e-6, ts - float(self._last_t))
            rx_bps = (counters.rx_bytes - self._last.rx_bytes) / dt
            tx_bps = (counters.tx_bytes - self._last.tx_bytes) / dt

        # Update history
        if counters is not None:
            self._last = counters
            self._last_t = ts

        kind = "wifi" if wifi else "ethernet"
        is_tether = (not wifi) and any(iface.startswith(p) for p in self._tether_prefixes)

        msg: Dict[str, Any] = {
            "ts": ts,
            "sensor": self.name,
            "type": "net",
            "iface": iface,
            "ip": ip,
            "link": {
                "kind": kind,
                "state": oper,
                "speed_mbps": speed,
            },
            "is_tether": bool(is_tether),
        }

        if rx_bps is not None:
            msg["rx_bps"] = float(rx_bps)
        if tx_bps is not None:
            msg["tx_bps"] = float(tx_bps)

        if counters is not None:
            msg["counters"] = {
                "rx_drop": int(counters.rx_drop),
                "tx_drop": int(counters.tx_drop),
                "rx_errs": int(counters.rx_errs),
                "tx_errs": int(counters.tx_errs),
            }

        return msg
