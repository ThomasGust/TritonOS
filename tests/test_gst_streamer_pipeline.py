import importlib.util
import sys
import types
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "video" / "gst_streamer.py"


def _load_gst_streamer(monkeypatch):
    fake_gi = types.ModuleType("gi")
    fake_gi.require_version = lambda *args, **kwargs: None

    fake_repo = types.ModuleType("gi.repository")
    fake_gst = types.SimpleNamespace(init=lambda *args, **kwargs: None)
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


def test_h264_pipeline_uses_capture_timestamps_and_leaky_sender_queues(monkeypatch):
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
    assert "queue name=q_capture max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream" in desc
    assert "h264parse config-interval=-1 disable-passthrough=true" in desc
    assert "queue name=q_pay max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream" in desc
    assert desc.index("queue name=q_pay") < desc.index("rtph264pay")


def test_sender_low_latency_options_can_be_disabled(monkeypatch):
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
