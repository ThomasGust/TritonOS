import importlib.util
import sys
import types
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "video" / "gst_streamer.py"


def _load_gst_streamer(monkeypatch):
    fake_gi = types.ModuleType("gi")
    fake_gi.require_version = lambda *args, **kwargs: None

    fake_repo = types.ModuleType("gi.repository")
    fake_gst = types.SimpleNamespace(
        init=lambda *args, **kwargs: None,
        SECOND=1_000_000_000,
        CLOCK_TIME_NONE=-1,
        State=types.SimpleNamespace(PLAYING="PLAYING", NULL="NULL"),
        parse_launch=lambda desc: object(),
    )
    fake_gobject = types.SimpleNamespace(threads_init=lambda *args, **kwargs: None)
    fake_repo.Gst = fake_gst
    fake_repo.GObject = fake_gobject

    monkeypatch.setitem(sys.modules, "gi", fake_gi)
    monkeypatch.setitem(sys.modules, "gi.repository", fake_repo)

    spec = importlib.util.spec_from_file_location("gst_streamer_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def _build_description(module, monkeypatch, cfg):
    captured = {}

    def fake_parse_launch(desc):
        captured["desc"] = desc
        return object()

    module.Gst.parse_launch = fake_parse_launch
    monkeypatch.setattr(module, "resolve_v4l2_device", lambda device, prefer_h264=False: device)
    monkeypatch.setattr(module, "apply_h264_quality_controls", lambda *args, **kwargs: {})
    monkeypatch.setattr(module.GstStream, "_apply_udp_sink_qos", lambda self, pipeline, cfg: None)

    module.GstStream(cfg)._build_pipeline(cfg)
    return captured["desc"]


def test_h264_pipeline_defaults_to_stable_nonleaky_sender_path(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    cfg = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
    )

    desc = _build_description(gst_streamer, monkeypatch, cfg)

    assert "v4l2src device=/dev/video2 do-timestamp=true" in desc
    assert "h264parse config-interval=-1 disable-passthrough=true" in desc
    assert "queue name=" not in desc
    assert desc.index("h264parse") < desc.index("rtph264pay")
    assert "multiudpsink" in desc
    assert "clients=192.168.1.1:5000" in desc
    assert "tee name=snapshot_tee" in desc
    assert "appsink name=snapshot_sink" in desc
    assert "decodebin" in desc
    assert "jpegenc quality=90" in desc


def test_sender_low_latency_options_can_enable_leaky_queues(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    cfg = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
        extra={"sender_leaky_queues": True},
    )

    desc = _build_description(gst_streamer, monkeypatch, cfg)

    assert "queue name=q_capture max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream" in desc
    assert "queue name=q_pay max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream" in desc
    assert desc.index("queue name=q_pay") < desc.index("rtph264pay")


def test_onboard_snapshot_branch_can_be_disabled(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    cfg = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
        extra={"rov_snapshot_enabled": False},
    )

    desc = _build_description(gst_streamer, monkeypatch, cfg)

    assert "tee name=snapshot_tee" not in desc
    assert "appsink name=snapshot_sink" not in desc


def test_capture_snapshot_pulls_jpeg_from_appsink(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)

    class _FakeBuffer:
        def __init__(self, data):
            self.data = bytes(data)

        def get_size(self):
            return len(self.data)

        def extract_dup(self, offset, size):
            copy_started["value"] = True
            return self.data[offset : offset + size]

    class _FakeCaps:
        def to_string(self):
            return "image/jpeg,width=32,height=24"

    class _FakeSample:
        def __init__(self, data):
            self._buffer = _FakeBuffer(data)

        def get_buffer(self):
            return self._buffer

        def get_caps(self):
            return _FakeCaps()

    class _FakeSink:
        def __init__(self):
            self.calls = []

        def emit(self, signal_name, timeout_ns):
            self.calls.append((signal_name, timeout_ns))
            return _FakeSample(b"\xff\xd8snapshot\xff\xd9")

    class _FakePipeline:
        def __init__(self, sink):
            self.sink = sink

        def get_by_name(self, name):
            return self.sink if name == "snapshot_sink" else None

    sink = _FakeSink()
    copy_started = {"value": False}
    stream = gst_streamer.GstStream(
        gst_streamer.StreamConfig(name="Primary Camera", video_format="h264", host="192.168.1.1")
    )
    stream._pipeline = _FakePipeline(sink)
    monkeypatch.setattr(gst_streamer.time, "monotonic", lambda: 20.0 if copy_started["value"] else 10.0)

    frame = stream.capture_snapshot(timeout_s=0.25)

    assert frame.stream == "Primary Camera"
    assert frame.mime_type == "image/jpeg"
    assert frame.caps == "image/jpeg,width=32,height=24"
    assert frame.data == b"\xff\xd8snapshot\xff\xd9"
    assert frame.monotonic_ts == 10.0
    assert sink.calls == [("try-pull-sample", 250_000_000)]


def test_capture_stereo_pair_uses_fresh_parallel_snapshots(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)

    class _FakeStream:
        def __init__(self, name, monotonic_ts):
            self.name = name
            self.monotonic_ts = monotonic_ts
            self.calls = []

        def capture_snapshot(self, *, timeout_s=1.5, fresh=False):
            self.calls.append({"timeout_s": timeout_s, "fresh": fresh})
            return gst_streamer.SnapshotFrame(
                stream=self.name,
                data=f"{self.name}-jpg".encode("ascii"),
                mime_type="image/jpeg",
                caps="image/jpeg,width=32,height=24",
                wall_ts=1000.0 + self.monotonic_ts,
                monotonic_ts=self.monotonic_ts,
                seq=len(self.calls),
            )

    manager = gst_streamer.StreamManager()
    left = _FakeStream("Left", 50.000)
    right = _FakeStream("Right", 50.008)
    manager._streams = {"Left": left, "Right": right}

    pair = manager.capture_stereo_pair("Left", "Right", timeout_s=0.5, max_pair_delta_ms=20.0)

    assert pair.left.stream == "Left"
    assert pair.right.stream == "Right"
    assert pair.pair_delta_ms == pytest.approx(8.0)
    assert pair.timestamp_source == "rov_snapshot_appsink_fresh_pull_monotonic"
    assert left.calls and left.calls[0]["fresh"] is True
    assert right.calls and right.calls[0]["fresh"] is True


def test_capture_stereo_pair_uses_cached_frames_and_source_timing(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    monkeypatch.setattr(gst_streamer.time, "monotonic", lambda: 1000.0)

    left = gst_streamer.GstStream(
        gst_streamer.StreamConfig(
            name="Left",
            video_format="h264",
            host="192.168.1.1",
            extra={"rov_snapshot_cache_enabled": True},
        )
    )
    right = gst_streamer.GstStream(
        gst_streamer.StreamConfig(
            name="Right",
            video_format="h264",
            host="192.168.1.1",
            extra={"rov_snapshot_cache_enabled": True},
        )
    )
    left._snapshot_cache_frames.append(
        gst_streamer.SnapshotFrame(
            stream="Left",
            data=b"left-jpg",
            mime_type="image/jpeg",
            caps="image/jpeg,width=32,height=24",
            wall_ts=1000.0,
            monotonic_ts=1000.010,
            seq=1,
            source_monotonic_ts=50.000,
            capture_source="rov_snapshot_cache",
        )
    )
    right._snapshot_cache_frames.append(
        gst_streamer.SnapshotFrame(
            stream="Right",
            data=b"right-jpg",
            mime_type="image/jpeg",
            caps="image/jpeg,width=32,height=24",
            wall_ts=1000.0,
            monotonic_ts=1000.018,
            seq=2,
            source_monotonic_ts=50.004,
            capture_source="rov_snapshot_cache",
        )
    )
    manager = gst_streamer.StreamManager()
    manager._streams = {"Left": left, "Right": right}

    pair = manager.capture_stereo_pair("Left", "Right", timeout_s=0.5, max_pair_delta_ms=20.0)

    assert pair.left.stream == "Left"
    assert pair.right.stream == "Right"
    assert pair.pair_delta_ms == pytest.approx(4.0)
    assert pair.timestamp_source == "rov_snapshot_cache_source_monotonic"
    assert pair.attempts == 1


def test_sender_low_latency_options_can_be_disabled_explicitly(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    cfg = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
        extra={
            "sender_leaky_queues": False,
            "sender_v4l2_do_timestamp": False,
        },
    )

    desc = _build_description(gst_streamer, monkeypatch, cfg)

    assert "do-timestamp=true" not in desc
    assert "queue name=" not in desc


def test_udp_mirror_ports_use_multiudpsink(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    cfg = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
        port=5000,
        extra={"udp_mirror_ports": [6000]},
    )

    desc = _build_description(gst_streamer, monkeypatch, cfg)

    assert "multiudpsink" in desc
    assert "clients=192.168.1.1:5000,192.168.1.1:6000" in desc
    assert "udpsink host=192.168.1.1 port=5000" not in desc


def test_udp_mirror_port_changes_are_live_updates(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    old = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
        port=5000,
        extra={"sender_leaky_queues": True},
    )
    new = old.clone_with_updates(extra={"sender_leaky_queues": True, "udp_mirror_ports": [61000]})

    live, changes = gst_streamer.GstStream._is_live_update(old, new)

    assert live is True
    assert set(changes) == {"extra"}


def test_non_mirror_extra_changes_still_rebuild(monkeypatch):
    gst_streamer = _load_gst_streamer(monkeypatch)
    old = gst_streamer.StreamConfig(
        name="Primary Camera",
        device="/dev/video2",
        width=1920,
        height=1080,
        fps=30,
        video_format="h264",
        host="192.168.1.1",
        extra={"sender_leaky_queues": True},
    )
    new = old.clone_with_updates(extra={"sender_leaky_queues": False})

    live, changes = gst_streamer.GstStream._is_live_update(old, new)

    assert live is False
    assert set(changes) == {"extra"}
