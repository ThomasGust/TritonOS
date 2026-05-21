import subprocess
import shutil
import uuid
from pathlib import Path
from types import SimpleNamespace

from control.management_rpc import ManagementRpcService


def _scratch_dir() -> Path:
    p = Path("tests") / "_tmp_management_rpc" / str(uuid.uuid4())
    p.mkdir(parents=True, exist_ok=True)
    return p


def test_management_rpc_get_state_and_set_surface_reference(monkeypatch):
    root = _scratch_dir()
    try:
        depth_path = root / "depth_reference.json"

        cfg_stub = SimpleNamespace(
            __file__="sample_rov_config.py",
            EXTERNAL_DEPTH_REFERENCE_PATH=str(depth_path),
            EXTERNAL_DEPTH_SENSOR_TO_TOP_M=0.15,
        )

        runtime_snapshot = {
            "armed": False,
            "updated_ts": 1234.5,
            "depth_hold": {
                "available": True,
                "sensor_available": True,
                "target_m": 1.2,
                "status": {"enabled_cmd": True, "active": True, "reason": "hold"},
                "status_age_s": 0.05,
                "sensor": {"depth_m": 1.18},
            },
        }

        class _ControlStub:
            def get_hold_status_snapshot(self):
                return runtime_snapshot

        svc = ManagementRpcService(bind_endpoint="tcp://127.0.0.1:0", control_service=_ControlStub())
        monkeypatch.setattr(svc, "_config_module", lambda: cfg_stub)
        monkeypatch.setattr("control.management_rpc.load_runtime_config_snapshot", lambda: {"DEPTH_HOLD_KP": 0.55})

        resp = svc._handle_request({"cmd": "get_state"})
        assert resp["ok"] is True
        assert resp["data"]["config"]["DEPTH_HOLD_KP"] == 0.55
        assert resp["data"]["runtime"]["control_loop_available"] is True
        assert resp["data"]["runtime"]["depth_hold"]["target_m"] == 1.2
        assert "get_hold_status" in resp["data"]["commands"]
        assert "update_code" in resp["data"]["commands"]
        assert "restart_service" in resp["data"]["commands"]

        resp_runtime = svc._handle_request({"cmd": "get_hold_status"})
        assert resp_runtime["ok"] is True
        assert resp_runtime["data"]["control_loop_available"] is True
        assert resp_runtime["data"]["depth_hold"]["status"]["reason"] == "hold"

        resp2 = svc._handle_request(
            {"cmd": "set_surface_reference", "args": {"surface_pressure_mbar": 1014.5}}
        )
        assert resp2["ok"] is True
        assert depth_path.exists()
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_management_rpc_update_code_runs_force_update(monkeypatch, tmp_path):
    svc = ManagementRpcService(bind_endpoint="tcp://127.0.0.1:0")
    monkeypatch.setattr(svc, "_repo_root", lambda: tmp_path)
    script = tmp_path / "bin" / "update_code.sh"
    script.parent.mkdir(parents=True)
    script.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")

    calls = []

    def _run(cmd, **kwargs):
        calls.append((cmd, kwargs))
        return subprocess.CompletedProcess(cmd, 0, stdout="ok", stderr="")

    monkeypatch.setattr("control.management_rpc.subprocess.run", _run)
    resp = svc._handle_request({"cmd": "update_code", "args": {"branch": "main", "force": True}})

    assert resp["ok"] is True
    assert "--force" in calls[0][0]
    assert calls[0][0][:2] == ["bash", str(script)]
