# rov/sensors/sensor_pub_service.py
from __future__ import annotations
import time
import json
import threading
from typing import List

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
    for opt, val in [
        (getattr(zmq, "LINGER", None), 0),
        (getattr(zmq, "SNDHWM", None), 1000),
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
                 debug: bool = False):
        self.bind_endpoint = bind_endpoint
        self.sensors = sensors
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
                    try:
                        self.sock.send_string(json.dumps(reading), flags=zmq.NOBLOCK)
                    except zmq.Again:
                        # Drop stale telemetry rather than blocking sensor threads under congestion.
                        pass
                    #if self.debug:
                    #    print("[rov/sensors]", reading)
            time.sleep(0.01)
