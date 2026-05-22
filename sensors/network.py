"""Network telemetry sensor.

Publishes a lightweight "net" message onto the existing sensor PUB stream so
the topside can display tether-focused link stats even when Wi-Fi is enabled on
the Pi.

Highlights:
  - Prefers a tether/ethernet interface for stats (not just the default route)
  - Still reports the default-route interface for transparency
  - Exposes live RX/TX throughput + error/drop counters
"""

# sensors/network.py
from __future__ import annotations

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
    return os.path.isdir(f"/sys/class/net/{iface}/wireless")


def _iface_operstate(iface: str) -> Optional[str]:
    return _read_text(f"/sys/class/net/{iface}/operstate")


def _iface_speed_mbps(iface: str) -> Optional[int]:
    txt = _read_text(f"/sys/class/net/{iface}/speed")
    if not txt:
        return None
    try:
        v = int(float(txt))
        return v if v > 0 else None
    except Exception:
        return None


def _default_route_iface() -> Optional[str]:
    try:
        with open("/proc/net/route", "r", encoding="utf-8", errors="ignore") as f:
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


def _list_nonloop_ifaces() -> list[str]:
    try:
        return [n for n in os.listdir("/sys/class/net") if n and n != "lo"]
    except Exception:
        return []


def _iface_seems_up(iface: str) -> bool:
    st = (_iface_operstate(iface) or "").lower()
    return st in ("up", "unknown", "dormant")


def _fallback_up_iface() -> Optional[str]:
    for iface in _list_nonloop_ifaces():
        if _iface_seems_up(iface):
            return iface
    return None


def _iface_ipv4_addr(iface: str) -> Optional[str]:
    try:
        import fcntl

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ifreq = struct.pack("256s", iface.encode("utf-8")[:15])
        res = fcntl.ioctl(s.fileno(), 0x8915, ifreq)  # SIOCGIFADDR
        return socket.inet_ntoa(res[20:24])
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
    try:
        with open("/proc/net/dev", "r", encoding="utf-8", errors="ignore") as f:
            for line in f.readlines()[2:]:
                if ":" not in line:
                    continue
                name, rest = line.split(":", 1)
                if name.strip() != iface:
                    continue
                fields = rest.strip().split()
                if len(fields) < 16:
                    return None
                return _DevCounters(
                    rx_bytes=int(fields[0]),
                    rx_errs=int(fields[2]),
                    rx_drop=int(fields[3]),
                    tx_bytes=int(fields[8]),
                    tx_errs=int(fields[10]),
                    tx_drop=int(fields[11]),
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
        prefer_tether: bool = True,
    ):
        super().__init__(name="network", rate_hz=float(rate_hz))
        self._iface_override = iface
        self._tether_prefixes = tuple(tether_prefixes)
        self._prefer_tether = bool(prefer_tether)

        self._last_t: Optional[float] = None
        self._last: Optional[_DevCounters] = None
        self._last_iface: Optional[str] = None

    def _is_tether_candidate(self, iface: str) -> bool:
        if not iface or iface == "lo":
            return False
        if _is_wifi_iface(iface):
            return False
        return any(iface.startswith(p) for p in self._tether_prefixes)

    def _pick_tether_iface(self, default_iface: Optional[str]) -> Optional[str]:
        cands = []
        for iface in _list_nonloop_ifaces():
            if not self._is_tether_candidate(iface):
                continue
            st = (_iface_operstate(iface) or "").lower()
            if st not in ("up", "unknown", "dormant"):
                continue
            ip = _iface_ipv4_addr(iface)
            score = 0
            if iface == default_iface:
                score += 4
            if ip:
                score += 3
            sp = _iface_speed_mbps(iface)
            if isinstance(sp, int) and sp > 0:
                score += 1
            if st == "up":
                score += 1
            cands.append((score, iface))
        if not cands:
            return None
        cands.sort(key=lambda t: (t[0], t[1]), reverse=True)
        return cands[0][1]

    def _pick_iface(self) -> tuple[Optional[str], Optional[str], Optional[str], str]:
        if self._iface_override:
            default_iface = _default_route_iface()
            tether_iface = self._pick_tether_iface(default_iface)
            return self._iface_override, default_iface, tether_iface, "override"

        default_iface = _default_route_iface()
        tether_iface = self._pick_tether_iface(default_iface)

        if self._prefer_tether and tether_iface:
            if default_iface != tether_iface:
                return tether_iface, default_iface, tether_iface, "prefer_tether"
            return tether_iface, default_iface, tether_iface, "default_is_tether"

        if default_iface:
            return default_iface, default_iface, tether_iface, "default_route"

        fb = _fallback_up_iface()
        return fb, default_iface, tether_iface, "fallback_up"

    def read(self) -> Dict[str, Any]:
        """Return current selected-interface status and throughput counters."""

        ts = time.time()
        iface, default_iface, tether_iface, reason = self._pick_iface()
        if not iface:
            return {"ts": ts, "sensor": self.name, "type": "net", "error": "no_iface"}

        wifi = _is_wifi_iface(iface)
        oper = _iface_operstate(iface) or "unknown"
        speed = _iface_speed_mbps(iface)
        ip = _iface_ipv4_addr(iface)
        counters = _read_dev_counters(iface)

        # Reset delta history if the chosen interface changed.
        if self._last_iface and self._last_iface != iface:
            self._last = None
            self._last_t = None

        rx_bps = None
        tx_bps = None
        if counters is not None and self._last is not None and self._last_t is not None:
            dt = max(1e-6, ts - float(self._last_t))
            rx_bps = (counters.rx_bytes - self._last.rx_bytes) / dt
            tx_bps = (counters.tx_bytes - self._last.tx_bytes) / dt

        if counters is not None:
            self._last = counters
            self._last_t = ts
            self._last_iface = iface

        kind = "wifi" if wifi else "ethernet"
        is_tether = (not wifi) and self._is_tether_candidate(iface)

        msg: Dict[str, Any] = {
            "ts": ts,
            "sensor": self.name,
            "type": "net",
            # Backward compatible fields
            "iface": iface,
            "ip": ip,
            "link": {
                "kind": kind,
                "state": oper,
                "speed_mbps": speed,
            },
            "is_tether": bool(is_tether),
            # New transparency fields
            "selected_iface": iface,
            "default_iface": default_iface,
            "tether_iface": tether_iface,
            "selection_reason": reason,
        }

        if default_iface:
            try:
                msg["default_is_wifi"] = bool(_is_wifi_iface(default_iface))
            except Exception:
                pass

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
