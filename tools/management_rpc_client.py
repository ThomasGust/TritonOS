#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

import zmq


def _request(endpoint: str, payload: dict) -> dict:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(endpoint)
    sock.send_json(payload)
    return sock.recv_json()


def main() -> int:
    ap = argparse.ArgumentParser(description="Client for TritonOS management RPC.")
    ap.add_argument("--endpoint", default="tcp://127.0.0.1:5556")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("get-state")

    p_set = sub.add_parser("set-config")
    p_set.add_argument("updates_json", help='JSON object, e.g. {"DEPTH_HOLD_KP": 0.6}')

    p_surface = sub.add_parser("capture-surface")
    p_surface.add_argument("--samples", type=int, default=20)
    p_surface.add_argument("--delay-s", type=float, default=0.02)

    args = ap.parse_args()

    if args.cmd == "get-state":
        req = {"cmd": "get_state"}
    elif args.cmd == "set-config":
        req = {"cmd": "set_config", "args": {"updates": json.loads(args.updates_json)}}
    elif args.cmd == "capture-surface":
        req = {"cmd": "capture_surface_reference", "args": {"samples": args.samples, "delay_s": args.delay_s}}
    else:
        raise AssertionError(f"unhandled command: {args.cmd}")

    print(json.dumps(_request(args.endpoint, req), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
