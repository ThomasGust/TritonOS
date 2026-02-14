#!/usr/bin/env python3
"""Minimal network diagnostics server (ROV-side).

This intentionally avoids external deps like iperf so it can run on small
embedded Linux images.

It serves two lightweight tests on the same port:

1) UDP echo  (RTT / loss / jitter)
2) TCP throughput
   - mode=rx : client sends bytes -> server measures bytes received
   - mode=tx : server sends bytes -> client measures bytes received

Protocol (TCP): client sends a single JSON line (utf-8) and then switches to
binary streaming.

  {"mode":"rx","seconds":5,"chunk_size":65536}\n
  {"mode":"tx","seconds":5,"chunk_size":65536}\n
In tx mode, the server streams zero-bytes for the requested duration and then
appends a metadata marker + JSON so the client can parse server-side timing.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Optional


META_MARKER = b"\n__NETDIAG_META__\n"


def _json_line(obj: dict) -> bytes:
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")


def _udp_echo_loop(sock: socket.socket, stop: threading.Event, verbose: bool) -> None:
    sock.settimeout(0.5)
    if verbose:
        print(f"[netdiag] UDP echo ready on {sock.getsockname()}")
    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except Exception:
            continue
        try:
            sock.sendto(data, addr)
        except Exception:
            pass


def _recv_line(conn: socket.socket, timeout_s: float = 2.0, max_len: int = 16_384) -> bytes:
    conn.settimeout(timeout_s)
    buf = b""
    while b"\n" not in buf and len(buf) < max_len:
        chunk = conn.recv(4096)
        if not chunk:
            break
        buf += chunk
    if b"\n" in buf:
        line, _rest = buf.split(b"\n", 1)
        return line
    return buf


def _tcp_client_handler(conn: socket.socket, addr, verbose: bool) -> None:
    try:
        line = _recv_line(conn)
        req = json.loads(line.decode("utf-8", errors="replace") or "{}")
    except Exception:
        try:
            conn.sendall(_json_line({"ok": False, "error": "bad_request"}))
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        return

    mode = str(req.get("mode", "")).lower().strip()
    seconds = float(req.get("seconds", 5.0))
    seconds = max(0.25, min(seconds, 60.0))
    chunk_size = int(req.get("chunk_size", 65536))
    chunk_size = max(1024, min(chunk_size, 1024 * 1024))

    if verbose:
        print(f"[netdiag] TCP from {addr}: mode={mode} seconds={seconds} chunk={chunk_size}")

    if mode == "rx":
        # Client sends -> server measures.
        start = time.time()
        end_at = start + seconds
        hard_end = end_at + 2.0
        total = 0
        conn.settimeout(0.5)
        while True:
            now = time.time()
            if now >= hard_end:
                break
            try:
                data = conn.recv(65536)
            except socket.timeout:
                if now >= end_at:
                    break
                continue
            except Exception:
                break
            if not data:
                break
            total += len(data)
            if now >= end_at:
                # Continue draining briefly so kernel buffers don't skew (optional).
                continue
        dur = max(1e-6, time.time() - start)
        resp = {
            "ok": True,
            "mode": "rx",
            "bytes": int(total),
            "duration_s": float(dur),
        }
        try:
            conn.sendall(_json_line(resp))
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        return

    if mode == "tx":
        # Server sends -> client measures.
        start = time.time()
        end_at = start + seconds
        total = 0
        payload = b"\x00" * chunk_size
        conn.settimeout(1.0)
        try:
            while time.time() < end_at:
                conn.sendall(payload)
                total += len(payload)
        except Exception:
            pass
        dur = max(1e-6, time.time() - start)
        meta = {
            "ok": True,
            "mode": "tx",
            "bytes": int(total),
            "duration_s": float(dur),
        }
        try:
            conn.sendall(META_MARKER + _json_line(meta))
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        return

    try:
        conn.sendall(_json_line({"ok": False, "error": "unknown_mode"}))
    except Exception:
        pass
    try:
        conn.close()
    except Exception:
        pass


def _tcp_accept_loop(sock: socket.socket, stop: threading.Event, verbose: bool) -> None:
    sock.settimeout(0.5)
    if verbose:
        print(f"[netdiag] TCP ready on {sock.getsockname()}")
    while not stop.is_set():
        try:
            conn, addr = sock.accept()
        except socket.timeout:
            continue
        except Exception:
            continue
        t = threading.Thread(target=_tcp_client_handler, args=(conn, addr, verbose), daemon=True)
        t.start()


def start_in_thread(bind_host: str = "0.0.0.0", port: int = 7700, verbose: bool = False) -> threading.Event:
    """Start UDP echo + TCP throughput server in background threads.

    Returns a stop Event. Set it to request shutdown.
    """
    stop = threading.Event()

    # UDP
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind((bind_host, int(port)))

    # TCP
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp.bind((bind_host, int(port)))
    tcp.listen(8)

    threading.Thread(target=_udp_echo_loop, args=(udp, stop, verbose), daemon=True).start()
    threading.Thread(target=_tcp_accept_loop, args=(tcp, stop, verbose), daemon=True).start()

    return stop


def main() -> None:
    ap = argparse.ArgumentParser(description="ROV netdiag server (UDP echo + TCP throughput)")
    ap.add_argument("--bind", default="0.0.0.0", help="bind host (default: 0.0.0.0)")
    ap.add_argument("--port", type=int, default=7700, help="port for UDP+TCP (default: 7700)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    stop = start_in_thread(bind_host=args.bind, port=args.port, verbose=args.verbose)
    print(f"[netdiag] running on {args.bind}:{args.port} (UDP+TCP). Ctrl-C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()
        print("[netdiag] stopped")


if __name__ == "__main__":
    main()
