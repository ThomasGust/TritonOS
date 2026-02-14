#!/usr/bin/env python3
"""Publish sensor telemetry over ZMQ (ROV-side test).

Run this on the Raspberry Pi to publish ONLY sensor telemetry over ZMQ PUB.
This isolates the sensor streaming path end-to-end (Pi -> topside) without
starting video or control.

The message shapes match what the topside UI expects (see
`TritonPilot/gui/sensor_panel.py`).

Examples
--------
  # Bind using rov_config.SENSOR_PUB_ENDPOINT (default tcp://0.0.0.0:6001)
  python3 tests/sensor_stream_pub_test.py

  # Force fake sensors (no hardware required)
  python3 tests/sensor_stream_pub_test.py --fake

  # Include Bar30 on I2C bus 6
  python3 tests/sensor_stream_pub_test.py --bar30 --bar30-bus 6

On the topside computer, run:
  python3 tests/sensor_stream_sub_test.py --endpoint tcp://<pi-ip>:6001
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import zmq

# Ensure repo root is on sys.path when run as `python3 tests/...`
_THIS = Path(__file__).resolve()
_REPO_ROOT = _THIS.parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

try:
    import rov_config as cfg  # type: ignore
except Exception:
    cfg = None

try:
    from sensors.navigator import BaseSensor  # type: ignore
except Exception:
    # In fake-only mode we don't need Navigator installed, but keep a compatible base.
    @dataclass
    class BaseSensor:  # type: ignore
        name: str
        rate_hz: float
        _next_t: float = 0.0

        def should_poll(self, now: float) -> bool:
            return now >= self._next_t

        def mark_polled(self, now: float):
            self._next_t = now + 1.0 / self.rate_hz

        def read(self) -> Dict[str, Any]:
            raise NotImplementedError


class FakeIMUSensor(BaseSensor):
    """Synthetic IMU readings with smooth sinusoids (good for end-to-end tests)."""

    def __init__(self, rate_hz: float = 20.0):
        super().__init__("imu", rate_hz)
        self._t0 = time.time()

    def read(self) -> Dict[str, Any]:
        t = time.time() - self._t0
        ax = 0.10 * math.sin(2 * math.pi * 0.7 * t)
        ay = 0.10 * math.sin(2 * math.pi * 0.5 * t + 1.0)
        az = 1.00 + 0.02 * math.sin(2 * math.pi * 0.2 * t)
        gx = 0.01 * math.sin(2 * math.pi * 0.3 * t)
        gy = 0.01 * math.sin(2 * math.pi * 0.4 * t + 0.3)
        gz = 0.01 * math.sin(2 * math.pi * 0.2 * t + 0.7)
        mx, my, mz = 0.2, 0.0, 0.5
        return {
            "ts": time.time(),
            "sensor": self.name,
            "type": "imu",
            "accel": {"x": ax, "y": ay, "z": az},
            "gyro": {"x": gx, "y": gy, "z": gz},
            "mag": {"x": mx, "y": my, "z": mz},
        }


class FakeEnvSensor(BaseSensor):
    """Synthetic temperature + pressure, shaped like Navigator's env messages."""

    def __init__(self, rate_hz: float = 2.0):
        super().__init__("env", rate_hz)
        self._t0 = time.time()

    def read(self) -> Dict[str, Any]:
        t = time.time() - self._t0
        temp_c = 22.0 + 0.5 * math.sin(2 * math.pi * 0.02 * t)
        pressure_kpa = 101.3 + 0.8 * math.sin(2 * math.pi * 0.01 * t + 0.5)
        return {
            "ts": time.time(),
            "sensor": self.name,
            "type": "env",
            "temperature_c": float(temp_c),
            "pressure_kpa": float(pressure_kpa),
        }


class FakeBar30Sensor(BaseSensor):
    """Synthetic external depth message matching Bar30Sensor output."""

    def __init__(self, rate_hz: float = 5.0):
        super().__init__("bar30", rate_hz)
        self._t0 = time.time()

    def read(self) -> Dict[str, Any]:
        t = time.time() - self._t0
        depth_m = 1.0 + 0.3 * math.sin(2 * math.pi * 0.05 * t)
        temp_c = 18.0 + 0.2 * math.sin(2 * math.pi * 0.02 * t)
        pressure_mbar = 1013.25 + depth_m * 100.0  # rough-ish
        return {
            "ts": time.time(),
            "sensor": self.name,
            "type": "external_depth",
            "depth_m": float(depth_m),
            "temperature_c": float(temp_c),
            "pressure_mbar": float(pressure_mbar),
        }


class _PubSockProxy:
    """Proxy around a ZMQ socket to report publish rates + occasional samples."""

    def __init__(self, sock: zmq.Socket, quiet: bool = False, sample_every_s: float = 5.0):
        self._sock = sock
        self.quiet = quiet
        self.sample_every_s = max(0.5, sample_every_s)

        self._lock = threading.Lock()
        self._total = 0
        self._since = 0
        self._by_type_since: Dict[str, int] = {}
        self._last_print_t = time.time()
        self._last_sample_t = 0.0
        self._last_sample: Optional[dict] = None

    def send_string(self, s: str):
        self._sock.send_string(s)
        if self.quiet:
            return

        now = time.time()
        msg_type = "?"
        sample: Optional[dict] = None
        try:
            sample = json.loads(s)
            msg_type = str(sample.get("type", "?"))
        except Exception:
            msg_type = "bad_json"

        with self._lock:
            self._total += 1
            self._since += 1
            self._by_type_since[msg_type] = self._by_type_since.get(msg_type, 0) + 1
            self._last_sample = sample or {"raw": s}

            if now - self._last_print_t >= 1.0:
                dt = now - self._last_print_t
                rate = self._since / max(dt, 1e-6)
                parts = [f"{k}:{v/dt:.1f}Hz" for k, v in sorted(self._by_type_since.items())]
                print(f"[sensor-pub] total={self._total} rate={rate:.1f}Hz  " + " ".join(parts), flush=True)
                self._since = 0
                self._by_type_since.clear()
                self._last_print_t = now

            if self._last_sample is not None and (now - self._last_sample_t) >= self.sample_every_s:
                print(f"[sensor-pub] sample: {self._last_sample}", flush=True)
                self._last_sample_t = now


def _build_sensors(args) -> Tuple[List[BaseSensor], bool, Optional[str]]:
    """Return (sensors, using_fake, error_reason_if_any)."""

    if args.fake:
        sensors: List[BaseSensor] = [
            FakeIMUSensor(rate_hz=args.imu_rate),
            FakeEnvSensor(rate_hz=args.env_rate),
        ]
        if args.bar30:
            sensors.append(FakeBar30Sensor(rate_hz=args.bar30_rate))
        return sensors, True, None

    # Real hardware path.
    try:
        from sensors.navigator import NavigatorBoard, IMUSensor, EnvSensor, Bar30Sensor  # type: ignore

        board = NavigatorBoard()
        sensors = [
            IMUSensor(board, rate_hz=args.imu_rate),
            EnvSensor(board, rate_hz=args.env_rate),
        ]
        if args.bar30:
            sensors.append(Bar30Sensor(rate_hz=args.bar30_rate, bus=args.bar30_bus))
        return sensors, False, None

    except Exception as e:
        reason = f"{type(e).__name__}: {e}"
        if args.require_hw:
            raise
        sensors = [
            FakeIMUSensor(rate_hz=args.imu_rate),
            FakeEnvSensor(rate_hz=args.env_rate),
        ]
        if args.bar30:
            sensors.append(FakeBar30Sensor(rate_hz=args.bar30_rate))
        return sensors, True, reason


def main() -> int:
    default_bind = "tcp://0.0.0.0:6001"
    if cfg is not None:
        default_bind = getattr(cfg, "SENSOR_PUB_ENDPOINT", default_bind)

    ap = argparse.ArgumentParser(description="ROV-side sensor streaming publisher test")
    ap.add_argument("--bind", default=default_bind, help="ZMQ endpoint to bind, e.g. tcp://0.0.0.0:6001")
    ap.add_argument("--imu-rate", type=float, default=20.0, help="IMU publish rate (Hz)")
    ap.add_argument("--env-rate", type=float, default=2.0, help="Temperature/pressure publish rate (Hz)")
    ap.add_argument("--bar30", action="store_true", help="Include external depth sensor (Bar30)")
    ap.add_argument("--bar30-rate", type=float, default=5.0, help="Bar30 publish rate (Hz)")
    ap.add_argument(
        "--bar30-bus",
        type=int,
        default=getattr(cfg, "BAR30_I2C_BUS", 6) if cfg else 6,
        help="I2C bus for Bar30 (Navigator external bus is often 6)",
    )
    ap.add_argument("--fake", action="store_true", help="Publish synthetic data (no hardware required)")
    ap.add_argument("--require-hw", action="store_true", help="Exit if hardware sensors cannot be initialized")
    ap.add_argument("--quiet", action="store_true", help="Don't print publish stats")
    args = ap.parse_args()

    sensors, using_fake, reason = _build_sensors(args)
    if using_fake:
        if args.fake:
            print("[sensor-pub] using FAKE sensors (forced)", flush=True)
        else:
            print(f"[sensor-pub] hardware init failed; falling back to FAKE sensors ({reason})", flush=True)
    else:
        print("[sensor-pub] using hardware sensors", flush=True)

    # Bind ZMQ PUB socket and stream readings.
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.bind(args.bind)
    pub = _PubSockProxy(sock, quiet=args.quiet)

    print(f"[sensor-pub] PUB bound on {args.bind}", flush=True)
    print("[sensor-pub] Ctrl+C to stop", flush=True)

    # Give subscribers a moment to connect (PUB/SUB has a 'slow joiner' behavior).
    time.sleep(0.2)

    try:
        while True:
            now = time.time()
            for s in sensors:
                if s.should_poll(now):
                    try:
                        reading = s.read()
                    except Exception as e:
                        reading = {
                            "ts": time.time(),
                            "sensor": getattr(s, "name", "unknown"),
                            "type": "error",
                            "error": str(e),
                        }
                    s.mark_polled(now)
                    pub.send_string(json.dumps(reading))
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[sensor-pub] stopping...", flush=True)
    finally:
        try:
            sock.close(linger=0)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
