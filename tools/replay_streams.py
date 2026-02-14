#!/usr/bin/env python3
"""Replay a recorded streams.jsonl file onto ZMQ endpoints (useful to simulate pilot input).

Example (simulate topside sending pilot frames to a local ROV process):
  python tools/replay_streams.py recordings/.../streams.jsonl --pilot-endpoint tcp://*:6000

Use --speed 2.0 for 2x faster.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import zmq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", help="Path to streams.jsonl")
    ap.add_argument("--pilot-endpoint", default=None, help="PUB endpoint for pilot frames (tcp://*:6000)")
    ap.add_argument("--sensor-endpoint", default=None, help="PUB endpoint for sensor/heartbeat frames (tcp://*:6001)")
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed (1.0 = real time)")
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    pub_pilot = None
    pub_sensors = None
    if args.pilot_endpoint:
        pub_pilot = ctx.socket(zmq.PUB)
        pub_pilot.bind(args.pilot_endpoint)
    if args.sensor_endpoint:
        pub_sensors = ctx.socket(zmq.PUB)
        pub_sensors.bind(args.sensor_endpoint)

    t0 = None
    wall0 = None

    for line in Path(args.jsonl).read_text().splitlines():
        if not line.strip():
            continue
        ev = json.loads(line)
        t = float(ev.get("t", time.time()))
        stream = ev.get("stream")
        msg = ev.get("msg", {})

        if t0 is None:
            t0 = t
            wall0 = time.time()

        dt = (t - t0) / max(args.speed, 1e-9)
        while time.time() - wall0 < dt:
            time.sleep(0.001)

        raw = json.dumps(msg)
        if stream == "pilot" and pub_pilot is not None:
            pub_pilot.send_string(raw)
        elif stream == "sensors" and pub_sensors is not None:
            pub_sensors.send_string(raw)

    print("Replay finished.")

if __name__ == "__main__":
    main()
