"""Management RPC service for ROV maintenance and runtime state.

This ZeroMQ REP service is separate from the high-rate pilot/control path. It
handles infrequent management operations requested by topside tooling: reading
the active configuration, updating selected config constants, capturing or
setting depth-reference pressure, reporting hold/autopilot state, invoking the
code-update script, and scheduling a service restart.

Every handler returns a JSON-serializable `{"ok": bool, ...}` envelope so the
topside UI can show actionable errors instead of parsing tracebacks.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import zmq

import rov_config as cfg
from utils.config_store import (
    load_runtime_config_snapshot,
    reload_runtime_config_module,
    update_config_values,
)
from utils.vehicle_reference import (
    DEFAULT_ATTITUDE_REFERENCE_PATH,
    DEFAULT_DEPTH_REFERENCE_PATH,
    capture_surface_pressure_reference,
    load_attitude_reference,
    load_surface_pressure_reference_mbar,
    resolve_path,
    save_attitude_reference,
    save_surface_pressure_reference,
)


class ManagementRpcService:
    """Threaded ZeroMQ REP server for low-rate management commands."""

    def __init__(
        self,
        bind_endpoint: str,
        debug: bool = False,
        depth_sensor: Any | None = None,
        control_service: Any | None = None,
        attitude_estimator: Any | None = None,
    ):
        self.bind_endpoint = str(bind_endpoint)
        self.debug = bool(debug)
        self._depth_sensor = depth_sensor
        self._control_service = control_service
        self._attitude_estimator = attitude_estimator
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @staticmethod
    def _repo_root() -> Path:
        return Path(__file__).resolve().parents[1]

    def start(self) -> None:
        """Start the background management RPC thread."""

        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if self.debug:
            print(f"[rov/mgmt] RPC listening on {self.bind_endpoint}")

    def stop(self) -> None:
        """Request management RPC shutdown and wait briefly for the thread."""

        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _config_module(self) -> Any:
        return reload_runtime_config_module()

    def _reference_state(self, cfg_mod: Any) -> Dict[str, Any]:
        depth_path = str(getattr(cfg_mod, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
        attitude_path = str(getattr(cfg_mod, "ATTITUDE_REFERENCE_PATH", DEFAULT_ATTITUDE_REFERENCE_PATH))
        pressure = load_surface_pressure_reference_mbar(depth_path)
        attitude_ref = load_attitude_reference(attitude_path)
        attitude_status = {}
        if self._attitude_estimator is not None and hasattr(self._attitude_estimator, "status"):
            try:
                attitude_status = dict(self._attitude_estimator.status())
            except Exception:
                attitude_status = {}
        return {
            "depth_reference_path": depth_path,
            "depth_reference_exists": resolve_path(depth_path).exists(),
            "surface_pressure_mbar": pressure,
            "depth_sensor_to_top_m": float(getattr(cfg_mod, "EXTERNAL_DEPTH_SENSOR_TO_TOP_M", 0.0)),
            "attitude_reference_path": attitude_path,
            "attitude_reference_exists": resolve_path(attitude_path).exists(),
            "attitude_reference_loaded": bool(attitude_ref),
            "attitude_calibration_state": attitude_status.get("calibration_state"),
            "attitude_yaw_sources": attitude_status.get("yaw_sources", []),
        }

    def _runtime_state(self) -> Dict[str, Any]:
        empty_runtime = {
            "control_loop_available": False,
            "armed": False,
            "updated_ts": None,
            "autopilot": {
                "available": False,
                "sensor_available": False,
                "status": {},
                "status_age_s": None,
                "attitude_sensor": {},
            },
            "depth_hold": {
                "available": False,
                "sensor_available": False,
                "target_m": None,
                "status": {},
                "status_age_s": None,
                "sensor": {},
            },
        }
        if self._control_service is None or not hasattr(self._control_service, "get_hold_status_snapshot"):
            return empty_runtime
        snapshot = self._control_service.get_hold_status_snapshot()
        if not isinstance(snapshot, dict):
            return empty_runtime
        out = dict(snapshot)
        out["control_loop_available"] = True
        return out

    def _schedule_self_restart(self, delay_s: float = 1.0) -> None:
        def _restart() -> None:
            time.sleep(max(0.1, float(delay_s)))
            os._exit(0)

        threading.Thread(target=_restart, daemon=True).start()

    def _apply_surface_pressure_live(self, pressure_mbar: float) -> bool:
        if self._depth_sensor is None or not hasattr(self._depth_sensor, "set_surface_pressure_mbar"):
            return False
        self._depth_sensor.set_surface_pressure_mbar(float(pressure_mbar))
        return True

    def _capture_attitude_reference(self, cfg_mod: Any, args: Dict[str, Any]) -> Dict[str, Any]:
        if self._attitude_estimator is None or not hasattr(self._attitude_estimator, "capture_current_reference"):
            return {"ok": False, "error": "attitude estimator is not available"}
        attitude_path = str(args.get("path") or getattr(cfg_mod, "ATTITUDE_REFERENCE_PATH", DEFAULT_ATTITUDE_REFERENCE_PATH))
        reference = self._attitude_estimator.capture_current_reference()
        save_attitude_reference(
            attitude_path,
            reference,
            meta={"source": "rpc_capture", "set_ts": time.time()},
        )
        return {
            "ok": True,
            "data": {
                "attitude_reference_path": attitude_path,
                "attitude_reference": reference,
                "restart_required": False,
            },
        }

    def _run_update_code(self, args: Dict[str, Any]) -> Dict[str, Any]:
        repo_root = self._repo_root()
        branch = str(args.get("branch") or getattr(cfg, "TRITONOS_BRANCH", "main") or "main").strip() or "main"
        force = bool(args.get("force", True))
        restart = bool(args.get("restart", False))
        timeout_s = float(args.get("timeout_s", 180.0))
        script = repo_root / "bin" / "update_code.sh"
        if not script.exists():
            return {"ok": False, "error": f"update script not found: {script}"}

        cmd = ["bash", str(script), "--dir", str(repo_root), "--branch", branch]
        if force:
            cmd.append("--force")
        if bool(args.get("with_apt", False)):
            cmd.append("--with-apt")

        try:
            proc = subprocess.run(
                cmd,
                cwd=str(repo_root),
                text=True,
                capture_output=True,
                timeout=max(5.0, timeout_s),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            return {
                "ok": False,
                "error": f"update timed out after {timeout_s:.0f}s",
                "stdout": exc.stdout or "",
                "stderr": exc.stderr or "",
            }
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

        data = {
            "returncode": int(proc.returncode),
            "stdout": proc.stdout[-12000:],
            "stderr": proc.stderr[-12000:],
            "branch": branch,
            "force": force,
            "restart_requested": restart,
        }
        if proc.returncode != 0:
            return {"ok": False, "error": f"update failed with exit code {proc.returncode}", "data": data}
        if restart:
            self._schedule_self_restart(1.0)
            data["restart_scheduled"] = True
        return {"ok": True, "data": data}

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
                    "runtime": self._runtime_state(),
                    "commands": [
                        "get_state",
                        "get_hold_status",
                        "set_config",
                        "set_surface_reference",
                        "capture_surface_reference",
                        "capture_attitude_reference",
                        "capture_local_rest",
                        "update_code",
                        "restart_service",
                    ],
                },
            }

        if cmd in ("get_hold_status", "get_runtime_state", "hold_status"):
            return {
                "ok": True,
                "data": self._runtime_state(),
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

        if cmd == "update_code":
            return self._run_update_code(args)

        if cmd in ("restart_service", "restart"):
            self._schedule_self_restart(float(args.get("delay_s", 1.0)))
            return {
                "ok": True,
                "data": {
                    "restart_scheduled": True,
                    "delay_s": float(args.get("delay_s", 1.0)),
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
            applied_live = self._apply_surface_pressure_live(pressure_mbar)
            return {
                "ok": True,
                "data": {
                    "surface_pressure_mbar": pressure_mbar,
                    "path": depth_path,
                    "applied_live": applied_live,
                    "restart_required": not applied_live,
                },
            }

        if cmd == "capture_surface_reference":
            samples = int(args.get("samples", 20))
            delay_s = float(args.get("delay_s", 0.02))
            depth_path = str(args.get("path") or getattr(cfg_mod, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
            pressure_mbar = capture_surface_pressure_reference(
                cfg_mod,
                samples=samples,
                delay_s=delay_s,
                sensor=self._depth_sensor,
            )
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
            applied_live = self._apply_surface_pressure_live(pressure_mbar)
            return {
                "ok": True,
                "data": {
                    "surface_pressure_mbar": pressure_mbar,
                    "path": depth_path,
                    "applied_live": applied_live,
                    "restart_required": not applied_live,
                },
            }

        if cmd == "capture_attitude_reference":
            return self._capture_attitude_reference(cfg_mod, args)

        if cmd == "capture_local_rest":
            out: Dict[str, Any] = {}
            errors: Dict[str, str] = {}
            attitude_resp = self._capture_attitude_reference(cfg_mod, args)
            if attitude_resp.get("ok"):
                out["attitude"] = dict(attitude_resp.get("data") or {})
            else:
                errors["attitude"] = str(attitude_resp.get("error") or "attitude capture failed")

            if bool(args.get("include_depth", True)):
                try:
                    samples = int(args.get("samples", 20))
                    delay_s = float(args.get("delay_s", 0.02))
                    depth_path = str(args.get("depth_path") or getattr(cfg_mod, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH))
                    pressure_mbar = capture_surface_pressure_reference(
                        cfg_mod,
                        samples=samples,
                        delay_s=delay_s,
                        sensor=self._depth_sensor,
                    )
                    save_surface_pressure_reference(
                        depth_path,
                        pressure_mbar,
                        meta={
                            "source": "rpc_local_rest",
                            "samples": samples,
                            "delay_s": delay_s,
                            "sensor_to_top_m": float(getattr(cfg_mod, "EXTERNAL_DEPTH_SENSOR_TO_TOP_M", 0.0)),
                        },
                    )
                    applied_live = self._apply_surface_pressure_live(pressure_mbar)
                    out["depth"] = {
                        "surface_pressure_mbar": pressure_mbar,
                        "path": depth_path,
                        "applied_live": applied_live,
                        "restart_required": not applied_live,
                    }
                except Exception as exc:
                    errors["depth"] = str(exc)

            if not out:
                return {"ok": False, "error": "; ".join(f"{k}: {v}" for k, v in errors.items()) or "local rest capture failed"}
            out["errors"] = errors
            out["restart_required"] = any(bool((v or {}).get("restart_required")) for v in out.values() if isinstance(v, dict))
            return {"ok": True, "data": out}

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

