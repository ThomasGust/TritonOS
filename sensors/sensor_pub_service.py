# rov/sensors/sensor_pub_service.py
from __future__ import annotations
import time
import json
import os
import threading
from typing import Any, List

import zmq

from sensors.navigator import (
    NavigatorBoard,
    IMUSensor,
    MagSensor,
    EnvSensor,
    LeakSensor,
    ADCSensor,
    Bar30Sensor,
    BaseSensor,
)


def _zmq_best_effort_qos(sock: zmq.Socket) -> None:
    """Best-effort low-latency / QoS hints for telemetry sockets."""
    try:
        snd_hwm = int(os.environ.get("TRITON_SENSOR_SNDHWM", "50"))
    except Exception:
        snd_hwm = 50
    for opt, val in [
        (getattr(zmq, "LINGER", None), 0),
        (getattr(zmq, "SNDHWM", None), max(1, snd_hwm)),
        (getattr(zmq, "SNDTIMEO", None), 0),
    ]:
        try:
            if opt is not None:
                sock.setsockopt(opt, int(val))
        except Exception:
            pass
    for opt, val in [
        (getattr(zmq, "IMMEDIATE", None), 1),
        (getattr(zmq, "TCP_NODELAY", None), 1),
        (getattr(zmq, "TCP_KEEPALIVE", None), 1),
        (getattr(zmq, "TCP_KEEPALIVE_IDLE", None), 10),
        (getattr(zmq, "TCP_KEEPALIVE_INTVL", None), 5),
        (getattr(zmq, "TCP_KEEPALIVE_CNT", None), 3),
        (getattr(zmq, "HEARTBEAT_IVL", None), 1000),
        (getattr(zmq, "HEARTBEAT_TIMEOUT", None), 3000),
        (getattr(zmq, "HEARTBEAT_TTL", None), 6000),
        (getattr(zmq, "TOS", None), 0x88),   # AF41 telemetry
        (getattr(zmq, "PRIORITY", None), 5), # Linux socket priority (best-effort)
    ]:
        try:
            if opt is not None:
                sock.setsockopt(opt, int(val))
        except Exception:
            pass


class SensorPublisherService:
    def __init__(self,
                 bind_endpoint: str,
                 sensors: List[BaseSensor],
                 derived_processors: List[Any] | None = None,
                 debug: bool = False):
        self.bind_endpoint = bind_endpoint
        self.sensors = sensors
        self.derived_processors = list(derived_processors or [])
        self.debug = debug

        ctx = zmq.Context.instance()
        self.sock = ctx.socket(zmq.PUB)
        _zmq_best_effort_qos(self.sock)
        self.sock.bind(self.bind_endpoint)

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if self.debug:
            print(f"[rov/sensors] PUB bound on {self.bind_endpoint}")

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        while not self._stop.is_set():
            now = time.time()
            for s in self.sensors:
                if s.should_poll(now):
                    try:
                        reading = s.read()
                    except Exception as e:
                        reading = {
                            "ts": time.time(),
                            "sensor": s.name,
                            "type": "error",
                            "error": str(e),
                        }
                    s.mark_polled(now)
                    self._publish(reading)
                    for derived in self._derive(reading):
                        self._publish(derived)
                    #if self.debug:
                    #    print("[rov/sensors]", reading)
            time.sleep(0.01)

    def _publish(self, reading: dict) -> None:
        try:
            self.sock.send_string(json.dumps(reading, separators=(",", ":")), flags=zmq.NOBLOCK)
        except zmq.Again:
            # Drop stale telemetry rather than blocking sensor threads under congestion.
            pass

    def _derive(self, reading: dict) -> list[dict]:
        out: list[dict] = []
        for processor in self.derived_processors:
            try:
                derived = processor.process(reading)
            except Exception as e:
                if self.debug:
                    print("[rov/sensors] derived telemetry failed:", e)
                continue
            if isinstance(derived, dict):
                out.append(derived)
            elif isinstance(derived, list):
                out.extend(item for item in derived if isinstance(item, dict))
        return out
