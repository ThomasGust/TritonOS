from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional

import zmq

import rov_config as cfg
from sensors.navigator import NavigatorBoard
from utils.config_store import (
    load_runtime_config_snapshot,
    reload_runtime_config_module,
    update_config_values,
)
from utils.vehicle_reference import (
    DEFAULT_DEPTH_REFERENCE_PATH,
    DEFAULT_FLAT_MOUNT_PATH,
    capture_flat_mount_reference,
    capture_surface_pressure_reference,
    load_mount_reference,
    load_surface_pressure_reference_mbar,
    resolve_path,
    save_mount_reference,
    save_surface_pressure_reference,
)


class ManagementRpcService:
    def __init__(self, bind_endpoint: str, debug: bool = False):
        self.bind_endpoint = str(bind_endpoint)
        self.debug = bool(debug)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if self.debug:
            print(f"[rov/mgmt] RPC listening on {self.bind_endpoint}")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _config_module(self) -> Any:
        return reload_runtime_config_module()

    def _reference_state(self, cfg_mod: Any) -> Dict[str, Any]:
        depth_path = str(getattr(cfg_mod, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
        mount_path = str(getattr(cfg_mod, "ATTITUDE_MOUNT", DEFAULT_FLAT_MOUNT_PATH))
        pressure = load_surface_pressure_reference_mbar(depth_path)
        mount = load_mount_reference(mount_path)
        return {
            "depth_reference_path": depth_path,
            "depth_reference_exists": resolve_path(depth_path).exists(),
            "surface_pressure_mbar": pressure,
            "depth_sensor_to_top_m": float(getattr(cfg_mod, "EXTERNAL_DEPTH_SENSOR_TO_TOP_M", 0.0)),
            "mount_path": mount_path,
            "mount_exists": resolve_path(mount_path).exists(),
            "mount_loaded": mount is not None,
        }

    def _handle_request(self, req: Dict[str, Any]) -> Dict[str, Any]:
        cmd = str(req.get("cmd", "") or "").strip().lower()
        args = req.get("args", {}) or {}
        cfg_mod = self._config_module()

        if cmd == "ping":
            return {"ok": True, "data": "pong"}

        if cmd in ("get_state", "state"):
            return {
                "ok": True,
                "data": {
                    "config_path": getattr(cfg_mod, "__file__", None),
                    "config": load_runtime_config_snapshot(),
                    "references": self._reference_state(cfg_mod),
                    "commands": [
                        "get_state",
                        "set_config",
                        "set_surface_reference",
                        "capture_surface_reference",
                        "capture_flat_reference",
                    ],
                },
            }

        if cmd == "set_config":
            updates = args.get("updates", {}) or {}
            if not isinstance(updates, dict) or not updates:
                return {"ok": False, "error": "args.updates must be a non-empty object"}
            written = update_config_values(dict(updates))
            cfg_mod = self._config_module()
            return {
                "ok": True,
                "data": {
                    "updated": written,
                    "references": self._reference_state(cfg_mod),
                    "restart_required": True,
                },
            }

        if cmd == "set_surface_reference":
            try:
                pressure_mbar = float(args["surface_pressure_mbar"])
            except Exception:
                return {"ok": False, "error": "args.surface_pressure_mbar is required"}
            depth_path = str(args.get("path") or getattr(cfg_mod, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
            save_surface_pressure_reference(
                depth_path,
                pressure_mbar,
                meta={"source": "rpc", "set_ts": time.time()},
            )
            return {
                "ok": True,
                "data": {
                    "surface_pressure_mbar": pressure_mbar,
                    "path": depth_path,
                    "restart_required": True,
                },
            }

        if cmd == "capture_surface_reference":
            samples = int(args.get("samples", 20))
            delay_s = float(args.get("delay_s", 0.02))
            depth_path = str(args.get("path") or getattr(cfg_mod, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
            pressure_mbar = capture_surface_pressure_reference(cfg_mod, samples=samples, delay_s=delay_s)
            save_surface_pressure_reference(
                depth_path,
                pressure_mbar,
                meta={
                    "source": "rpc_capture",
                    "samples": samples,
                    "delay_s": delay_s,
                    "sensor_to_top_m": float(getattr(cfg_mod, "EXTERNAL_DEPTH_SENSOR_TO_TOP_M", 0.0)),
                },
            )
            return {
                "ok": True,
                "data": {
                    "surface_pressure_mbar": pressure_mbar,
                    "path": depth_path,
                    "restart_required": True,
                },
            }

        if cmd == "capture_flat_reference":
            samples = int(args.get("samples", 200))
            delay_s = float(args.get("delay_s", 0.02))
            yaw_deg = float(args.get("yaw_deg", getattr(cfg_mod, "ATTITUDE_AUTO_MOUNT_YAW_DEG", 0.0)))
            mount_path = str(args.get("path") or getattr(cfg_mod, "ATTITUDE_MOUNT", DEFAULT_FLAT_MOUNT_PATH))
            board = NavigatorBoard()
            mount, accel_avg = capture_flat_mount_reference(
                board,
                samples=samples,
                delay_s=delay_s,
                yaw_deg=yaw_deg,
            )
            save_mount_reference(
                mount_path,
                mount,
                meta={
                    "source": "rpc_capture",
                    "samples": samples,
                    "delay_s": delay_s,
                    "yaw_deg": yaw_deg,
                    "accel_avg": [float(x) for x in accel_avg.tolist()],
                },
            )
            return {
                "ok": True,
                "data": {
                    "path": mount_path,
                    "yaw_deg": yaw_deg,
                    "restart_required": True,
                },
            }

        return {"ok": False, "error": f"unknown cmd '{cmd}'"}

    def _run(self) -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REP)
        sock.setsockopt(zmq.LINGER, 0)
        sock.bind(self.bind_endpoint)

        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)

        while not self._stop.is_set():
            try:
                events = dict(poller.poll(100))
            except Exception:
                continue
            if sock not in events:
                continue

            try:
                raw = sock.recv()
            except Exception:
                continue

            try:
                req = json.loads(raw.decode("utf-8"))
            except Exception:
                sock.send_json({"ok": False, "error": "invalid json"})
                continue

            try:
                resp = self._handle_request(req)
            except Exception as e:
                resp = {"ok": False, "error": str(e)}

            try:
                sock.send_json(resp)
            except Exception:
                pass

