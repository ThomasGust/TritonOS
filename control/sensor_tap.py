# rov/control/sensor_tap.py
from __future__ import annotations

import json
import time
from typing import Optional, Dict, Any

import zmq


def _normalize_local_endpoint(ep: str) -> str:
    """Turn a bind-ish endpoint into a connect-friendly localhost endpoint."""
    s = str(ep).strip()
    # Common patterns in this repo:
    #   tcp://0.0.0.0:6001
    #   tcp://*:6001
    if s.startswith("tcp://0.0.0.0:"):
        return "tcp://127.0.0.1:" + s.split(":")[-1]
    if s.startswith("tcp://*:"):
        return "tcp://127.0.0.1:" + s.split(":")[-1]
    return s


class DepthSensorTap:
    """Non-blocking subscriber that keeps the latest external_depth sample."""

    def __init__(self, endpoint: str, *, conflate: bool = True, rcv_hwm: int = 10):
        self.endpoint = _normalize_local_endpoint(endpoint)

        ctx = zmq.Context.instance()
        self.sock = ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.RCVHWM, int(rcv_hwm))
        try:
            self.sock.setsockopt(zmq.CONFLATE, 1 if conflate else 0)
        except Exception:
            pass
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.connect(self.endpoint)

        # Last VALID depth sample (depth_m present and error==False)
        self.last_depth_m: Optional[float] = None
        self.last_ts: Optional[float] = None  # local receive time of last valid sample

        # Last time we received ANY external_depth message (valid or error).
        # Useful for debugging "stream present but invalid" vs "no messages".
        self.last_rx_ts: Optional[float] = None
        self.last_sensor_name: Optional[str] = None
        self.last_raw: Dict[str, Any] = {}

    def poll(self, *, max_msgs: int = 50) -> None:
        """Drain available messages without blocking."""
        n = 0
        while n < max_msgs:
            n += 1
            try:
                raw = self.sock.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            except Exception:
                return

            try:
                msg = json.loads(raw)
            except Exception:
                continue

            if (msg or {}).get("type") != "external_depth":
                continue

            # Record that we saw the stream, even if this sample is an error.
            # (This helps distinguish: no sensor vs sensor returning errors.)
            self.last_rx_ts = time.time()
            self.last_raw = dict(msg)

            if (msg or {}).get("error"):
                # Do not update last_depth_m/last_ts on error samples.
                continue
            if "depth_m" not in (msg or {}):
                continue

            try:
                self.last_depth_m = float(msg.get("depth_m"))
                # Use LOCAL receive time for freshness checks; it is robust to
                # clock skew and still reflects whether data is arriving.
                self.last_ts = time.time()
                self.last_sensor_name = str(msg.get("sensor", "depth"))
            except Exception:
                continue

    def age_s(self, now: Optional[float] = None) -> Optional[float]:
        """Age since last VALID depth sample (seconds)."""
        if self.last_ts is None:
            return None
        if now is None:
            now = time.time()
        return float(now) - float(self.last_ts)

    def rx_age_s(self, now: Optional[float] = None) -> Optional[float]:
        """Age since last external_depth message of any kind (valid or error)."""
        if self.last_rx_ts is None:
            return None
        if now is None:
            now = time.time()
        return float(now) - float(self.last_rx_ts)
