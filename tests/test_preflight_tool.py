from tools.rov_preflight import run_cmd

def test_run_cmd_handles_missing_binary():
    res = run_cmd(["definitely-not-a-real-command-xyz"], timeout_s=0.1)
    assert res["ok"] is False
    assert res["returncode"] is None
    assert res["error"] is not None
