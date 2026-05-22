"""Small ZeroMQ monitor helper for diagnostics and runtime connection state."""

# utils/zmq_monitor.py
from __future__ import annotations

import threading
import time
from typing import Callable, Optional, Dict, Any

import zmq

try:
    from zmq.utils.monitor import parse_monitor_message  # type: ignore
except Exception:  # pragma: no cover
    parse_monitor_message = None  # type: ignore


_EVENT_NAME = {
    getattr(zmq, "EVENT_CONNECTED", 0x0001): "connected",
    getattr(zmq, "EVENT_CONNECT_DELAYED", 0x0002): "connect_delayed",
    getattr(zmq, "EVENT_CONNECT_RETRIED", 0x0004): "connect_retried",
    getattr(zmq, "EVENT_LISTENING", 0x0008): "listening",
    getattr(zmq, "EVENT_BIND_FAILED", 0x0010): "bind_failed",
    getattr(zmq, "EVENT_ACCEPTED", 0x0020): "accepted",
    getattr(zmq, "EVENT_ACCEPT_FAILED", 0x0040): "accept_failed",
    getattr(zmq, "EVENT_CLOSED", 0x0080): "closed",
    getattr(zmq, "EVENT_CLOSE_FAILED", 0x0100): "close_failed",
    getattr(zmq, "EVENT_DISCONNECTED", 0x0200): "disconnected",
    getattr(zmq, "EVENT_MONITOR_STOPPED", 0x0400): "monitor_stopped",
}


def _parse_fallback(msg_parts):
    """Fallback parser for monitor messages if parse_monitor_message is unavailable."""
    # Monitor messages are (event, value, endpoint) but encoded in a binary struct.
    # pyzmq normally provides parse_monitor_message; this is a best-effort fallback.
    if not msg_parts:
        return {}
    first = msg_parts[0]
    if not isinstance(first, (bytes, bytearray)) or len(first) < 6:
        return {}
    # struct is: uint16 event, uint32 value, then endpoint as a frame
    import struct
    event_id, value = struct.unpack("=HI", first[:6])
    endpoint = None
    if len(msg_parts) >= 2 and isinstance(msg_parts[1], (bytes, bytearray)):
        try:
            endpoint = msg_parts[1].decode("utf-8", "replace")
        except Exception:
            endpoint = repr(msg_parts[1])
    return {"event": event_id, "value": value, "endpoint": endpoint}


class ZMQMonitor:
    """
    Lightweight ZMQ socket monitor helper.

    ZMQ doesn't give you a simple "connected?" API; monitor sockets provide events
    such as CONNECTED/DISCONNECTED/ACCEPTED.

    This helper:
      - attaches to a socket
      - starts a background reader thread
      - tracks a coarse connection state
      - optionally calls a callback with structured events

    Notes:
      - Connection "state" is best-effort for UX / logs. PUB/SUB can still drop messages
        even when "connected"; use application-layer heartbeats for real liveness.
    """

    def __init__(
        self,
        sock: zmq.Socket,
        *,
        name: str = "",
        on_event: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        self.sock = sock
        self.name = name or "zmq"
        self.on_event = on_event

        self._ctx = zmq.Context.instance()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._mon_sock: Optional[zmq.Socket] = None
        self._endpoint = f"inproc://{self.name}-mon-{id(sock)}-{time.time_ns()}"

        self._lock = threading.Lock()
        self._state = "unknown"
        self._connected = False
        self._peer_count = 0
        self._last_event: Dict[str, Any] = {}
        self._last_event_ts = 0.0

        self._start()

    def _start(self):
        try:
            # EVENT_ALL exists in recent pyzmq; fall back to a common mask.
            mask = getattr(zmq, "EVENT_ALL", 0xFFFF)
            self.sock.monitor(self._endpoint, mask)
        except Exception:
            # Monitoring is optional; do not fail the app if unsupported.
            return

        try:
            ms = self._ctx.socket(zmq.PAIR)
            ms.setsockopt(zmq.LINGER, 0)
            ms.connect(self._endpoint)
            self._mon_sock = ms
        except Exception:
            return

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop monitoring and close the internal monitor socket."""

        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        try:
            if hasattr(self.sock, "disable_monitor"):
                self.sock.disable_monitor()  # type: ignore[attr-defined]
        except Exception:
            pass
        if self._mon_sock is not None:
            try:
                self._mon_sock.close(0)
            except Exception:
                pass
            self._mon_sock = None

    def snapshot(self) -> Dict[str, Any]:
        """Return the latest coarse connection state and event metadata."""

        with self._lock:
            return {
                "name": self.name,
                "state": self._state,
                "connected": bool(self._connected),
                "peer_count": int(self._peer_count),
                "last_event_ts": float(self._last_event_ts),
                "last_event": dict(self._last_event) if self._last_event else {},
            }

    def _emit(self, evt: Dict[str, Any]):
        try:
            if self.on_event:
                self.on_event(evt)
        except Exception:
            pass

    def _apply_event(self, event_id: int, endpoint: Optional[str], value: int):
        name = _EVENT_NAME.get(event_id, f"event_{event_id}")
        now = time.time()

        with self._lock:
            self._last_event_ts = now
            self._last_event = {"event": name, "endpoint": endpoint, "value": value}
            # coarse state machine
            if name in ("connected", "accepted", "listening"):
                self._state = "connected"
                self._connected = True
                if name == "accepted":
                    self._peer_count += 1
                elif name == "connected":
                    self._peer_count = max(self._peer_count, 1)
            elif name in ("disconnected", "closed"):
                if name == "disconnected":
                    self._peer_count = max(0, self._peer_count - 1)
                if self._peer_count <= 0:
                    self._state = "disconnected"
                    self._connected = False
            elif name in ("connect_retried", "connect_delayed"):
                self._state = "retrying"
                self._connected = False
            elif name.endswith("failed"):
                self._state = "error"
                self._connected = False

        self._emit({
            "kind": "zmq",
            "name": self.name,
            "state": self._state,
            "event": name,
            "endpoint": endpoint,
            "value": value,
            "ts": now,
        })

    def _run(self):
        ms = self._mon_sock
        if ms is None:
            return

        poller = zmq.Poller()
        poller.register(ms, zmq.POLLIN)

        while not self._stop.is_set():
            try:
                events = dict(poller.poll(timeout=200))
            except Exception:
                time.sleep(0.2)
                continue

            if ms not in events:
                continue

            try:
                parts = ms.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            except Exception:
                break

            try:
                if parse_monitor_message is not None:
                    data = parse_monitor_message(parts)  # type: ignore[misc]
                else:
                    data = _parse_fallback(parts)
            except Exception:
                data = _parse_fallback(parts)

            if not data:
                continue

            event_id = int(data.get("event", 0))
            value = int(data.get("value", 0))
            endpoint = data.get("endpoint", None)
            if endpoint is not None:
                endpoint = str(endpoint)

            self._apply_event(event_id, endpoint, value)
