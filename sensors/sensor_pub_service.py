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
    EnvSensor,
    LeakSensor,
    ADCSensor,
    Bar30Sensor,
    BaseSensor,
)


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
        try:
            self.sock.setsockopt(zmq.LINGER, 0)
            self.sock.setsockopt(zmq.SNDHWM, 1000)
        except Exception:
            pass
        self.sock.bind(self.bind_endpoint)

        self._mon: ZMQMonitor | None = None
        try:
            self._mon = ZMQMonitor(self.sock, name="sensor_pub")
        except Exception:
            self._mon = None

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

        try:
            if self._mon is not None:
                self._mon.stop()
        except Exception:
            pass
        self._mon = None

        try:
            self.sock.close(0)
        except Exception:
            pass

    def connection_snapshot(self) -> dict:
        try:
            if self._mon is not None:
                return self._mon.snapshot()
        except Exception:
            pass
        return {"name": "sensor_pub", "state": "unknown", "connected": False, "peer_count": 0}

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
                    self.sock.send_string(json.dumps(reading))
                    #if self.debug:
                    #    print("[rov/sensors]", reading)
            time.sleep(0.01)
