"""Unit tests for the USB-recovery policy in start_stream_with_recovery.

The key safety guarantee under test: the broad parent-hub reset (which
re-enumerates every camera on the hub) must NEVER run while another camera is
streaming, so a late/stuck camera can never knock the working ones offline.
"""

import sys
import types

import pytest


def _install_fake_gi(monkeypatch):
    """Let video.gst_streamer import without a real GStreamer/gi present."""
    fake_gi = types.ModuleType("gi")
    fake_gi.require_version = lambda *a, **k: None
    fake_repo = types.ModuleType("gi.repository")
    fake_gst = types.SimpleNamespace(
        init=lambda *a, **k: None,
        SECOND=1_000_000_000,
        CLOCK_TIME_NONE=-1,
        State=types.SimpleNamespace(PLAYING="PLAYING", NULL="NULL"),
        parse_launch=lambda d: object(),
    )
    fake_repo.Gst = fake_gst
    fake_repo.GObject = types.SimpleNamespace(threads_init=lambda *a, **k: None)
    monkeypatch.setitem(sys.modules, "gi", fake_gi)
    monkeypatch.setitem(sys.modules, "gi.repository", fake_repo)


@pytest.fixture
def rpc(monkeypatch):
    _install_fake_gi(monkeypatch)
    for name in ("video.gst_streamer", "video.gst_streamer_rpc"):
        monkeypatch.delitem(sys.modules, name, raising=False)
    import importlib

    return importlib.import_module("video.gst_streamer_rpc")


class FakeMgr:
    """Minimal StreamManager stand-in with a scripted start outcome sequence."""

    def __init__(self, running=None, start_results=None):
        # running: {name: cfg} already-live streams
        self.running = dict(running or {})
        # start_results: list consumed per start_stream call; None=success,
        # Exception instance=raise it.
        self.start_results = list(start_results or [])
        self.start_calls = []
        self.stop_calls = []

    def list_streams(self):
        return dict(self.running)

    def stop_stream(self, name):
        self.stop_calls.append(name)
        self.running.pop(name, None)

    def start_stream(self, scfg):
        self.start_calls.append(scfg.name)
        outcome = self.start_results.pop(0) if self.start_results else None
        if isinstance(outcome, Exception):
            raise outcome
        self.running[scfg.name] = scfg


def _scfg(rpc, name="Primary Camera", device="/dev/v4l/by-path/*1.2.4*video-index2"):
    return rpc.streamconfig_from_dict({"name": name, "device": device})


def _deps(rpc, *, rebind_calls, reset_calls, hub_reset_enable=True, reset_ok=True, retries=2):
    return rpc.StartStreamDeps(
        rebind_port=lambda hint, msgs: rebind_calls.append(hint),
        reset_all=lambda hint, msgs: (reset_calls.append(hint) or True) and reset_ok,
        sleep=lambda _s: None,
        rebind_retries=retries,
        rebind_delay_s=0.0,
        hub_reset_enable=hub_reset_enable,
    )


def test_first_try_success_does_no_usb_recovery(rpc):
    rebind_calls, reset_calls = [], []
    mgr = FakeMgr(start_results=[None])
    result = rpc.start_stream_with_recovery(mgr, _scfg(rpc), _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls))
    assert result["ok"] is True
    assert mgr.start_calls == ["Primary Camera"]
    assert rebind_calls == [] and reset_calls == []
    assert result.get("messages") == []


def test_recovers_after_narrow_rebind(rpc):
    rebind_calls, reset_calls = [], []
    # initial fail, then succeed after first rebind
    mgr = FakeMgr(start_results=[RuntimeError("no device"), None])
    result = rpc.start_stream_with_recovery(mgr, _scfg(rpc), _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls))
    assert result["ok"] is True
    assert rebind_calls == ["1.2.4"]  # narrow, this camera's port only
    assert reset_calls == []  # never needed the hub reset
    assert any("USB rebind" in m for m in result["messages"])


def test_hub_reset_skipped_when_other_cameras_live(rpc):
    rebind_calls, reset_calls = [], []
    # Aux is already streaming; Primary fails initial + both rebinds.
    mgr = FakeMgr(
        running={"Aux Camera": object()},
        start_results=[RuntimeError("x"), RuntimeError("x"), RuntimeError("x")],
    )
    result = rpc.start_stream_with_recovery(mgr, _scfg(rpc), _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls))
    assert result["ok"] is False
    assert reset_calls == []  # the hub reset must NOT run with a camera live
    assert rebind_calls == ["1.2.4", "1.2.4"]
    assert any("Skipping broad USB hub reset" in m for m in result["messages"])


def test_hub_reset_allowed_at_cold_start(rpc):
    rebind_calls, reset_calls = [], []
    # Nothing else running; initial + 2 rebinds fail, then reset rescues it.
    mgr = FakeMgr(start_results=[RuntimeError("x"), RuntimeError("x"), RuntimeError("x"), None])
    result = rpc.start_stream_with_recovery(mgr, _scfg(rpc), _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls))
    assert result["ok"] is True
    assert reset_calls == ["1.2.4"]  # cold start: hub reset is safe and used
    assert any("broader USB reset" in m for m in result["messages"])


def test_hub_reset_can_be_disabled(rpc):
    rebind_calls, reset_calls = [], []
    mgr = FakeMgr(start_results=[RuntimeError("x"), RuntimeError("x"), RuntimeError("x")])
    deps = _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls, hub_reset_enable=False)
    result = rpc.start_stream_with_recovery(mgr, _scfg(rpc), deps)
    assert result["ok"] is False
    assert reset_calls == []
    assert any("disabled" in m for m in result["messages"])


def test_no_port_hint_fails_fast(rpc):
    rebind_calls, reset_calls = [], []
    mgr = FakeMgr(start_results=[RuntimeError("boom")])
    scfg = _scfg(rpc, device="/dev/video0")  # no USB port hint
    result = rpc.start_stream_with_recovery(mgr, scfg, _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls))
    assert result["ok"] is False
    assert mgr.start_calls == ["Primary Camera"]  # exactly one attempt
    assert rebind_calls == [] and reset_calls == []


def test_existing_stream_is_restarted(rpc):
    rebind_calls, reset_calls = [], []
    mgr = FakeMgr(running={"Primary Camera": object()}, start_results=[None])
    result = rpc.start_stream_with_recovery(mgr, _scfg(rpc), _deps(rpc, rebind_calls=rebind_calls, reset_calls=reset_calls))
    assert result["ok"] is True
    assert mgr.stop_calls == ["Primary Camera"]  # stopped before rebuild
