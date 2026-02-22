# rov/control/pilot_receiver.py
from __future__ import annotations

import argparse
import json
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

import zmq

from schema.pilot_common import PilotFrame, PilotButtons  # reuse your topside/schema :contentReference[oaicite:3]{index=3}


@dataclass
class PilotRxStats:
    received: int = 0
    parsed: int = 0
    bad_json: int = 0
    bad_type: int = 0
    bad_schema: int = 0
    out_of_order: int = 0
    duplicates: int = 0
    dropped_est: int = 0
    last_seq: int = -1
    last_arrival: float = 0.0
    last_remote_ts: float = 0.0


def _buttons_to_dict(b: PilotButtons) -> Dict[str, bool]:
    # PilotButtons is a dataclass; keep it explicit & stable
    return {
        "a": b.a, "b": b.b, "x": b.x, "y": b.y,
        "lb": b.lb, "rb": b.rb,
        "win": b.win, "menu": b.menu,
        "lstick": b.lstick, "rstick": b.rstick,
    }


def _compute_edges(prev: Optional[PilotButtons], cur: PilotButtons) -> Dict[str, str]:
    """
    Returns edges like {"menu": "down", "a": "up"}.
    Stored into PilotFrame.edges for convenience.
    """
    if prev is None:
        return {}

    p = _buttons_to_dict(prev)
    c = _buttons_to_dict(cur)
    edges: Dict[str, str] = {}
    for k, cv in c.items():
        pv = p.get(k, False)
        if (not pv) and cv:
            edges[k] = "down"
        elif pv and (not cv):
            edges[k] = "up"
    return edges


class PilotReceiver:
    """
    ROV-side subscriber: BINDs, receives pilot frames, keeps latest + arrival time.

    Improvements vs original:
      - Poller-based receive (no busy-wait loop) (original used NOBLOCK+sleep) :contentReference[oaicite:4]{index=4}
      - Drains backlog to keep the latest frame
      - Validates message type/schema
      - Computes button edges (down/up) and stores them in frame.edges
      - Tracks stats (drops, dupes, out-of-order, parse errors)
    """

    def __init__(
        self,
        bind_endpoint: str,
        debug: bool = False,
        poll_ms: int = 50,
        conflate: bool = True,
        rcv_hwm: int = 5,
        expected_schema: Optional[int] = None,
    ):
        self.bind_endpoint = bind_endpoint
        self.debug = debug
        self.poll_ms = int(poll_ms)
        self.expected_schema = expected_schema  # None means accept any

        ctx = zmq.Context.instance()
        self.sock = ctx.socket(zmq.SUB)

        # Best-effort low-latency / QoS hints for pilot control frames.
        for _opt, _val in [
            (getattr(zmq, "TCP_NODELAY", None), 1),
            (getattr(zmq, "TOS", None), 0xB8),   # EF / low latency
            (getattr(zmq, "PRIORITY", None), 6), # Linux socket priority (best-effort)
        ]:
            try:
                if _opt is not None:
                    self.sock.setsockopt(_opt, int(_val))
            except Exception:
                pass

        # Robust defaults
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.RCVHWM, int(rcv_hwm))
        try:
            # Keep only newest message (excellent for control loops)
            self.sock.setsockopt(zmq.CONFLATE, 1 if conflate else 0)
        except Exception:
            # Some zmq builds may not support CONFLATE on SUB; safe to ignore
            pass

        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.bind(self.bind_endpoint)

        self._poller = zmq.Poller()
        self._poller.register(self.sock, zmq.POLLIN)

        self._latest: Optional[PilotFrame] = None
        self._latest_arrival: float = 0.0
        self._lock = threading.Lock()

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._stats = PilotRxStats()
        self._prev_buttons: Optional[PilotButtons] = None

        self._last_log = 0.0

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if self.debug:
            print(f"[rov/pilot_rx] SUB bound on {self.bind_endpoint}")

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def stats(self) -> PilotRxStats:
        # return a copy-like snapshot
        s = self._stats
        return PilotRxStats(**s.__dict__)

    def _handle_message(self, raw: str, arrival: float) -> None:
        self._stats.received += 1

        try:
            d = json.loads(raw)
        except Exception:
            self._stats.bad_json += 1
            if self.debug:
                print(f"[rov/pilot_rx] bad json: {raw[:160]!r}")
            return

        msg_type = d.get("type", None)
        if msg_type not in (None, "pilot"):
            self._stats.bad_type += 1
            if self.debug:
                print(f"[rov/pilot_rx] unexpected type={msg_type!r}")
            return

        if self.expected_schema is not None and d.get("schema", None) != self.expected_schema:
            self._stats.bad_schema += 1
            if self.debug:
                print(f"[rov/pilot_rx] schema mismatch got={d.get('schema')} expected={self.expected_schema}")
            return

        try:
            frame = PilotFrame.from_dict(d)
        except Exception as e:
            self._stats.bad_json += 1
            if self.debug:
                print(f"[rov/pilot_rx] parse error: {e} raw={raw[:160]!r}")
            return

        # Track seq health
        last = self._stats.last_seq
        if frame.seq == last:
            self._stats.duplicates += 1
        elif frame.seq < last:
            self._stats.out_of_order += 1
        elif last >= 0 and frame.seq > last + 1:
            self._stats.dropped_est += (frame.seq - last - 1)

        self._stats.last_seq = frame.seq
        self._stats.last_arrival = arrival
        self._stats.last_remote_ts = frame.ts

        # Compute edges (buttons) on arrival
        edges = _compute_edges(self._prev_buttons, frame.buttons)
        if edges:
            # preserve any existing edges from sender, but prefer our computed edges
            merged = dict(frame.edges or {})
            merged.update(edges)
            frame.edges = merged
        self._prev_buttons = frame.buttons

        with self._lock:
            self._latest = frame
            self._latest_arrival = arrival

        self._stats.parsed += 1

        if self.debug and (arrival - self._last_log) > 0.2:
            a = frame.axes
            print(
                f"[rov/pilot_rx] seq={frame.seq} age=0.000 "
                f"axes(lx,ly,rx,ry,lt,rt)=({a.lx:+.2f},{a.ly:+.2f},{a.rx:+.2f},{a.ry:+.2f},{a.lt:+.2f},{a.rt:+.2f}) "
                f"dpad={frame.dpad} edges={frame.edges}"
            )
            self._last_log = arrival

    def _run(self):
        while not self._stop.is_set():
            events = dict(self._poller.poll(self.poll_ms))
            if self.sock not in events:
                continue

            # Drain backlog; keep latest
            while True:
                try:
                    raw = self.sock.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                arrival = time.time()
                self._handle_message(raw, arrival)

    def get_latest(self) -> Tuple[Optional[PilotFrame], float]:
        with self._lock:
            frame = self._latest
            arr = self._latest_arrival
        age = 0.0 if frame is None else (time.time() - arr)
        return frame, age

    def get_fresh(self, ttl: float) -> Tuple[Optional[PilotFrame], float]:
        frame, age = self.get_latest()
        if frame is None:
            return None, 0.0
        if age > ttl:
            return None, age
        return frame, age


def _cli_main() -> None:
    ap = argparse.ArgumentParser(description="ROV PilotReceiver debug (prints incoming pilot frames)")
    ap.add_argument("--bind", default="tcp://*:6000", help="ZMQ SUB bind endpoint (ROV side)")
    ap.add_argument("--ttl", type=float, default=0.5, help="freshness TTL seconds (for age display)")
    ap.add_argument("--print-rate", type=float, default=10.0, help="print summary rate (Hz)")
    ap.add_argument("--debug", action="store_true", help="verbose receiver logs")
    args = ap.parse_args()

    rx = PilotReceiver(bind_endpoint=args.bind, debug=args.debug)
    rx.start()
    print(f"[rov/pilot_rx] listening on {args.bind}")
    print("[rov/pilot_rx] (Ctrl+C to stop)")

    try:
        period = 1.0 / max(0.1, args.print_rate)
        tnext = time.time()
        while True:
            frame, age = rx.get_latest()
            now = time.time()
            if now >= tnext:
                st = rx.stats()
                if frame is None:
                    print(f"[rov/pilot_rx] no frames yet | stats={st}")
                else:
                    a = frame.axes
                    b = frame.buttons
                    print(
                        f"[rov/pilot_rx] latest seq={frame.seq} age={age:.3f}s "
                        f"lx={a.lx:+.2f} ly={a.ly:+.2f} rx={a.rx:+.2f} ry={a.ry:+.2f} lt={a.lt:+.2f} rt={a.rt:+.2f} "
                        f"dpad={frame.dpad} menu={b.menu} win={b.win} edges={frame.edges} "
                        f"(drops~{st.dropped_est} dup={st.duplicates} ooo={st.out_of_order})"
                    )
                tnext = now + period
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()


if __name__ == "__main__":
    _cli_main()
