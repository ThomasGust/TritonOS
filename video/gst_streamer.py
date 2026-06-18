"""
OO GStreamer wrapper for Raspberry Pi streaming (MJPEG & H.264)

This version explicitly matches:
  Pi (192.168.1.2) -> RTP/JPEG PT=26 -> Windows (192.168.1.1) listening on UDP/5000

Requirements (Pi):
  sudo apt update && sudo apt install -y python3-gi python3-gst-1.0 \
      gstreamer1.0-tools gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav

Example usage:

from gst_streamer import StreamConfig, StreamManager

mgr = StreamManager()

cfg0 = StreamConfig(
    name="cam0",
    device="/dev/v4l/by-path/*video-index0",
    width=1280,
    height=720,
    fps=30,
    transport="udp",
    host="192.168.1.1",   # Windows box
    port=5000,
    video_format="mjpeg"  # camera outputs MJPEG natively
)
mgr.start_stream(cfg0)
"""
from __future__ import annotations

import glob
import logging
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Dict, Optional, Any, Tuple

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GObject", "2.0")
from gi.repository import Gst, GObject

from video.v4l2_controls import apply_h264_quality_controls

# Initialize once
GObject.threads_init()
Gst.init(None)

logger = logging.getLogger("gst_streamer")
if not logger.handlers:
    _h = logging.StreamHandler()
    _fmt = logging.Formatter("[%(asctime)s] %(levelname)s %(name)s: %(message)s")
    _h.setFormatter(_fmt)
    logger.addHandler(_h)
logger.setLevel(logging.INFO)


_FALSEY_EXTRA = {"0", "false", "no", "off"}
_TRUTHY_EXTRA = {"1", "true", "yes", "on"}


@dataclass
class StreamConfig:
    """Configuration for one camera stream and its network transport."""

    # Identification
    name: str

    # Capture
    device: str = "/dev/v4l/by-path/*video-index2"
    width: int = 1280
    height: int = 720
    fps: int = 30

    # Input video format from camera: "mjpeg", "raw", or "h264"
    video_format: str = "h264"

    # Encoding control
    # - If video_format == "raw": encode is required ("h264" or "mjpeg")
    # - If video_format == "mjpeg": encode may be set to "h264" to transcode MJPEG->H.264
    encode: Optional[str] = None  # None, "h264", "mjpeg"
    # H.264 encoder options
    h264_bitrate: int = 4_000_000
    h264_gop: int = 30  # keyframe interval

    # Transport
    transport: str = "udp"  # "udp" or "tcp"
    host: Optional[str] = None  # required for udp
    port: int = 5000

    # When set, bind the UDP sender to a specific local interface address.
    # This helps keep video on the tether even if Wiâ€‘Fi is enabled.
    bind_address: Optional[str] = None

    # RTP payload types
    rtp_pt_jpeg: int = 26
    rtp_pt_h264: int = 96
    rtp_mtu: int = 1200
    udp_buffer_size: int = 1024 * 1024

    # Jitter buffer (receiver-side item; kept here for symmetry)
    latency_ms: int = 60

    # Misc
    sync: bool = False

    # Extensions
    extra: Dict[str, Any] = field(default_factory=dict)

    def clone_with_updates(self, **updates) -> "StreamConfig":
        """Return a copy with selected dataclass fields replaced."""

        d = asdict(self)
        d.update(updates)
        return StreamConfig(**d)


class GstError(RuntimeError):
    """Raised when a GStreamer pipeline cannot be built or controlled."""

    pass


@dataclass(frozen=True)
class SnapshotFrame:
    """One onboard still image captured from a live stream pipeline."""

    stream: str
    data: bytes
    mime_type: str
    caps: str
    wall_ts: float
    monotonic_ts: float
    seq: int = 0
    source_pts_ns: int | None = None
    source_dts_ns: int | None = None
    source_duration_ns: int | None = None
    source_monotonic_ts: float | None = None
    source_clock_ns: int | None = None
    extension: str = "jpg"
    capture_source: str = "rov_snapshot_appsink"


@dataclass(frozen=True)
class StereoSnapshotPair:
    """A best-effort simultaneous onboard still-image pair."""

    left: SnapshotFrame
    right: SnapshotFrame
    pair_delta_ms: float
    timestamp_source: str
    attempts: int = 1


@dataclass(frozen=True)
class _CompressedAU:
    """One stored H.264 access unit for the on-demand snapshot ring.

    Storing compressed access units is cheap (a byte copy, no decode) so the ring
    can hold every source frame at full rate without loading the Pi or
    back-pressuring the display tee. Frames are decoded on demand only at capture
    time, starting from the most recent keyframe so references are intact.
    """

    data: bytes
    pts_ns: int | None
    clock_ns: int | None  # base_time + pts: comparable across pipelines
    keyframe: bool
    wall_ts: float
    seq: int


def _candidate_h264_sibling_patterns(dev_or_pattern: str) -> list[str]:
    """Return likely sibling node patterns for UVC cameras where H.264 lives on a non-zero video-index."""
    s = (dev_or_pattern or "").strip()
    if not s or "video-index0" not in s:
        return []
    # DWE ExploreHD cameras expose native H.264 on the 3rd V4L2 node (video-index2).
    # Try that first, then a couple of nearby indices as a safe fallback.
    return [
        s.replace("video-index0", "video-index2"),
        s.replace("video-index0", "video-index3"),
        s.replace("video-index0", "video-index1"),
    ]


def resolve_v4l2_device(device: str, *, prefer_h264: bool = False) -> str:
    """Resolve a V4L2 device path, expanding /dev/v4l/by-path globs if present.

    When prefer_h264=True and a config points at a *video-index0 by-path symlink, try
    sibling nodes first (especially *video-index2) so we can use native camera H.264.
    """
    dev = (device or "").strip()
    if not dev:
        return dev

    # Allow configs like /dev/v4l/by-path/*1.3.4*video-index0
    if any(ch in dev for ch in "*?[]"):
        if prefer_h264:
            for cand in _candidate_h264_sibling_patterns(dev):
                matches = sorted(glob.glob(cand))
                if matches:
                    if matches[0] != dev:
                        logger.info("Using H.264-capable sibling node for pattern %s -> %s", dev, matches[0])
                    return matches[0]
        matches = sorted(glob.glob(dev))
        if not matches:
            raise GstError(f"No V4L2 device matches pattern: {dev}")
        return matches[0]

    # Exact path case (e.g. /dev/v4l/by-path/...video-index0)
    if prefer_h264:
        for cand in _candidate_h264_sibling_patterns(dev):
            if os.path.exists(cand):
                if cand != dev:
                    logger.info("Using H.264-capable sibling node %s instead of %s", cand, dev)
                return cand

    return dev


def _extra_bool(extra: Dict[str, Any], *names: str, default: bool = False) -> bool:
    for name in names:
        if name not in extra:
            continue
        value = extra.get(name)
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in _FALSEY_EXTRA:
            return False
        if text in _TRUTHY_EXTRA:
            return True
        return bool(value)
    return bool(default)


def _extra_int(
    extra: Dict[str, Any],
    *names: str,
    default: int = 0,
    minimum: int | None = None,
) -> int:
    value = default
    for name in names:
        if name not in extra or extra.get(name) is None:
            continue
        try:
            value = int(float(extra.get(name)))
            break
        except Exception:
            value = default
            break
    if minimum is not None:
        value = max(int(minimum), int(value))
    return int(value)


def _extra_float(
    extra: Dict[str, Any],
    *names: str,
    default: float = 0.0,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    value = float(default)
    for name in names:
        if name not in extra or extra.get(name) is None:
            continue
        try:
            value = float(extra.get(name))
            break
        except Exception:
            value = float(default)
            break
    if minimum is not None:
        value = max(float(minimum), float(value))
    if maximum is not None:
        value = min(float(maximum), float(value))
    return float(value)


def _extra_str(extra: Dict[str, Any], *names: str, default: str = "") -> str:
    for name in names:
        value = extra.get(name)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return str(default)


def _extra_int_list(extra: Dict[str, Any], *names: str) -> list[int]:
    for name in names:
        if name not in extra or extra.get(name) is None:
            continue
        value = extra.get(name)
        raw_values: list[Any]
        if isinstance(value, str):
            raw_values = [part.strip() for part in value.split(",") if part.strip()]
        elif isinstance(value, (list, tuple, set)):
            raw_values = list(value)
        else:
            raw_values = [value]
        ports: list[int] = []
        for raw in raw_values:
            try:
                port = int(float(raw))
            except Exception:
                continue
            if 0 < port <= 65535 and port not in ports:
                ports.append(port)
        return ports
    return []


def _v4l2src_part(dev: str, cfg: StreamConfig) -> str:
    props = [f"device={dev}"]
    if _extra_bool(cfg.extra, "sender_v4l2_do_timestamp", "v4l2_do_timestamp", default=True):
        props.append("do-timestamp=true")
    io_mode = _extra_str(cfg.extra, "sender_v4l2_io_mode", "v4l2_io_mode")
    if io_mode:
        props.append(f"io-mode={io_mode}")
    return "v4l2src " + " ".join(props)


def _sender_queue_parts(cfg: StreamConfig, name: str) -> list[str]:
    """Return optional leaky sender queues for explicit low-latency profiles."""

    if not _extra_bool(cfg.extra, "sender_leaky_queues", "leaky_queues", default=False):
        return []
    buffers = _extra_int(cfg.extra, "sender_queue_max_buffers", default=1, minimum=1)
    bytes_ = _extra_int(cfg.extra, "sender_queue_max_bytes", default=0, minimum=0)
    time_ms = _extra_int(cfg.extra, "sender_queue_max_time_ms", default=0, minimum=0)
    return [
        (
            f"queue name={name} max-size-buffers={buffers} "
            f"max-size-bytes={bytes_} max-size-time={time_ms * 1_000_000} "
            "leaky=downstream"
        )
    ]


def _udp_destination_ports(cfg: StreamConfig) -> list[int]:
    ports = [int(cfg.port)]
    for port in _extra_int_list(
        cfg.extra,
        "udp_mirror_ports",
        "mirror_udp_ports",
    ):
        if port not in ports:
            ports.append(port)
    return ports


def _udp_clients(cfg: StreamConfig) -> str:
    return ",".join(f"{cfg.host}:{port}" for port in _udp_destination_ports(cfg))


def _extra_without_udp_mirrors(extra: Dict[str, Any]) -> Dict[str, Any]:
    stripped = dict(extra or {})
    stripped.pop("udp_mirror_ports", None)
    stripped.pop("mirror_udp_ports", None)
    return stripped


def _only_udp_mirrors_changed(old: Dict[str, Any], new: Dict[str, Any]) -> bool:
    return _extra_without_udp_mirrors(old) == _extra_without_udp_mirrors(new)


def _snapshot_enabled(cfg: StreamConfig) -> bool:
    return _extra_bool(cfg.extra, "rov_snapshot_enabled", "snapshot_enabled", default=True)


def _snapshot_fps(cfg: StreamConfig) -> int:
    return _extra_int(cfg.extra, "rov_snapshot_fps", "snapshot_fps", default=4, minimum=1)


def _snapshot_quality(cfg: StreamConfig) -> int:
    return _extra_int(cfg.extra, "rov_snapshot_jpeg_quality", "snapshot_jpeg_quality", default=90, minimum=1)


def _snapshot_cache_enabled(cfg: StreamConfig) -> bool:
    return _extra_bool(cfg.extra, "rov_snapshot_cache_enabled", "snapshot_cache_enabled", default=False)


def _snapshot_cache_frames(cfg: StreamConfig) -> int:
    return _extra_int(cfg.extra, "rov_snapshot_cache_frames", "snapshot_cache_frames", default=16, minimum=2)


def _snapshot_ondemand(cfg: StreamConfig) -> bool:
    """On-demand compressed-GOP snapshot path (decode only at capture time).

    Defaults ON for H.264: the alternative branch runs a continuous software
    H.264 decode (openh264dec) just to feed a snapshot appsink, which burns a
    whole core per camera even when nobody is capturing -- that pegged the Pi and
    caused thermal-throttle stutter on the display. The on-demand AU ring only
    copies compressed access units (cheap) and decodes a frame at capture time.
    """
    default = str(cfg.video_format).lower() == "h264"
    return _extra_bool(cfg.extra, "rov_snapshot_ondemand", "snapshot_ondemand", default=default)


def _snapshot_ring_aus(cfg: StreamConfig) -> int:
    """Number of compressed access units to retain (~3s at source fps)."""
    return _extra_int(cfg.extra, "rov_snapshot_ring_aus", "snapshot_ring_aus", default=120, minimum=16)


def _snapshot_decoder(cfg: StreamConfig) -> str:
    """GStreamer decoder element for the isolated on-demand decode pipeline.

    Independent of the display path, so it can be tuned for capture latency
    without affecting live video. ``avdec_h264 max-threads=N`` uses libav's
    multi-threaded software decode (the Pi5 has no HW H.264 decode), which can
    decode a 1080p GOP much faster than single-threaded openh264dec.
    """
    return _extra_str(
        cfg.extra, "rov_snapshot_decoder", "snapshot_decoder", default="openh264dec"
    ) or "openh264dec"


def _snapshot_appsink_part() -> str:
    return "appsink name=snapshot_sink emit-signals=false sync=false max-buffers=1 drop=true"


def _snapshot_raw_to_jpeg_parts(cfg: StreamConfig) -> list[str]:
    fps = _snapshot_fps(cfg)
    quality = min(100, _snapshot_quality(cfg))
    return [
        "videoconvert",
        "videorate drop-only=true",
        f"video/x-raw,framerate={fps}/1",
        f"jpegenc quality={quality}",
        _snapshot_appsink_part(),
    ]


def _snapshot_au_branch_parts(cfg: StreamConfig) -> list[str]:
    """Compressed-AU ring branch: copy H.264 access units, no in-pipeline decode.

    The queue and appsink both stay leaky/drop so this branch can never
    back-pressure the tee and stall the display, even under heavy capture load.
    Byte copies keep up trivially, so drops are effectively never hit.
    """
    ring = _snapshot_ring_aus(cfg)
    max_buffers = max(16, ring + 16)
    return [
        "queue max-size-buffers=16 max-size-bytes=0 max-size-time=0 leaky=downstream",
        "h264parse config-interval=-1",
        "video/x-h264,alignment=au,stream-format=byte-stream",
        f"appsink name=snapshot_sink emit-signals=false sync=false max-buffers={max_buffers} drop=true",
    ]


def _snapshot_branch_parts(cfg: StreamConfig) -> list[str]:
    vf = cfg.video_format.lower()
    queue = "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream"
    if vf == "mjpeg":
        return [queue, _snapshot_appsink_part()]
    if vf == "h264" and _snapshot_ondemand(cfg):
        return _snapshot_au_branch_parts(cfg)
    if vf == "h264":
        # Interim (display-safe) snapshot branch: pinned openh264dec gives clean
        # frames (decodebin selected a corrupting decoder on this Pi). The queue
        # stays leaky so a slow decode never back-pressures the tee / display.
        # NOTE: leaking compressed access units starves openh264dec of references
        # so it only emits clean frames near keyframe rate (~1 fps) -- fine for
        # single stills, too slow for tight stereo pairing. The production fix is
        # the on-demand compressed-GOP decode path (decode only at capture time).
        return [
            queue,
            "h264parse config-interval=-1 disable-passthrough=true",
            "openh264dec",
            *_snapshot_raw_to_jpeg_parts(cfg),
        ]
    if vf == "raw":
        return [queue, *_snapshot_raw_to_jpeg_parts(cfg)]
    return []


def _snapshot_stream_queue_part() -> str:
    return "queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream"


def _gst_time_to_ns(value: Any) -> int | None:
    try:
        numeric = int(value)
    except Exception:
        return None
    if numeric < 0:
        return None
    try:
        if numeric == int(Gst.CLOCK_TIME_NONE):
            return None
    except Exception:
        pass
    return numeric


class GstStream:
    """A single streaming pipeline with lifecycle management."""
    def __init__(self, config: StreamConfig):
        self.config = config
        self._pipeline: Optional[Gst.Pipeline] = None
        self._bus_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._last_error: Optional[str] = None
        self._started_wall_ts: Optional[float] = None
        self._started_monotonic_ts: Optional[float] = None
        self._state_lock = threading.Lock()
        self._snapshot_lock = threading.Lock()
        self._snapshot_seq = 0
        self._snapshot_cache_cond = threading.Condition()
        self._snapshot_cache_frames = deque(maxlen=_snapshot_cache_frames(config))
        self._snapshot_cache_thread: threading.Thread | None = None
        self._ondemand = _snapshot_ondemand(config)
        self._au_ring: deque[_CompressedAU] = deque(maxlen=_snapshot_ring_aus(config))
        # Persistent on-demand decode + encode pipelines (built lazily, reused).
        # Decode emits raw frames; only the single target frame is JPEG-encoded,
        # so we never waste an encode on the other GOP frames we decode for refs.
        self._decode_pipeline: Optional[Gst.Pipeline] = None
        self._decode_src: Optional[Gst.Element] = None
        self._decode_out: Optional[Gst.Element] = None
        self._encode_pipeline: Optional[Gst.Pipeline] = None
        self._encode_src: Optional[Gst.Element] = None
        self._encode_out: Optional[Gst.Element] = None
        self._decode_lock = threading.Lock()
        self._decode_pts_ns = 0
        self._encode_pts_ns = 0

    # ------------- Public ------------- #
    def start(self) -> None:
        """Build and start the GStreamer pipeline."""

        with self._state_lock:
            if self._pipeline is not None:
                logger.warning("Stream '%s' already running", self.config.name)
                return
            self._pipeline = self._build_pipeline(self.config)
            self._set_state(Gst.State.PLAYING)
            self._started_wall_ts = time.time()
            self._started_monotonic_ts = time.monotonic()
            self._start_bus_watcher()
            self._start_snapshot_cache()
            logger.info("Stream '%s' started", self.config.name)

    def stop(self, timeout: float = 5.0) -> None:
        """Stop the pipeline, send EOS best-effort, and release resources."""

        with self._state_lock:
            if self._pipeline is None:
                return
            logger.info("Stopping stream '%s'", self.config.name)
            self._stop_event.set()
            self._stop_snapshot_cache()
            self._stop_decode_pipeline()
            try:
                self._send_eos(timeout=1.0)
            except Exception:
                pass
            self._set_state(Gst.State.NULL)
            self._teardown_pipeline()
            self._started_wall_ts = None
            self._started_monotonic_ts = None
            logger.info("Stream '%s' stopped", self.config.name)

    def restart(self) -> None:
        """Stop and immediately rebuild/start the stream."""

        logger.info("Restarting stream '%s'", self.config.name)
        self.stop()
        time.sleep(0.1)
        self.start()

    def update(self, **updates) -> None:
        """Apply stream config updates live when possible, otherwise restart."""

        new_cfg = self.config.clone_with_updates(**updates)
        live_ok, changes = self._is_live_update(self.config, new_cfg)
        if live_ok and self._pipeline is not None:
            logger.info("Live-updating stream '%s': %s", self.config.name, changes)
            self._apply_live_updates(new_cfg)
            self.config = new_cfg
            return
        logger.info("Rebuilding stream '%s' due to changes: %s", self.config.name, changes)
        self.config = new_cfg
        self.restart()

    def is_running(self) -> bool:
        """Return True when a pipeline object is currently active."""

        return self._pipeline is not None

    def last_error(self) -> Optional[str]:
        """Return the latest GStreamer bus error string, if any."""

        return self._last_error

    def status(self) -> Dict[str, Any]:
        """Return stream config plus lightweight timing diagnostics."""

        return {
            "config": asdict(self.config),
            "running": self.is_running(),
            "started_wall_ts": self._started_wall_ts,
            "started_monotonic_ts": self._started_monotonic_ts,
            "last_error": self._last_error,
            "snapshot_ready": self._snapshot_sink() is not None,
            "snapshot_cache_enabled": _snapshot_cache_enabled(self.config),
            "snapshot_cache_frames": len(self.snapshot_cache_frames()),
            "snapshot_ondemand": self._ondemand,
            "snapshot_ring_aus": len(self.au_ring_frames()),
        }

    def capture_snapshot(self, *, timeout_s: float = 1.5, fresh: bool = False) -> SnapshotFrame:
        """Return one still image from the configured ROV capture path."""

        if self._ondemand:
            return self.capture_ondemand_snapshot(timeout_s=timeout_s)

        if _snapshot_cache_enabled(self.config):
            after_ts = time.monotonic() if fresh else None
            return self.capture_cached_snapshot(timeout_s=timeout_s, after_monotonic_ts=after_ts)

        sink = self._snapshot_sink()
        if sink is None:
            raise GstError(f"Stream '{self.config.name}' has no onboard snapshot sink")

        timeout_ns = int(max(0.0, float(timeout_s)) * Gst.SECOND)
        sample = None
        try:
            if fresh:
                while True:
                    stale = sink.emit("try-pull-sample", 0)
                    if stale is None:
                        break
            sample = sink.emit("try-pull-sample", timeout_ns)
            pulled_monotonic_ts = time.monotonic()
        except Exception as exc:
            raise GstError(f"Snapshot pull failed for '{self.config.name}': {exc}") from exc
        if sample is None:
            raise TimeoutError(f"No onboard snapshot frame available for '{self.config.name}'")

        return self._snapshot_frame_from_sample(
            sample,
            pulled_monotonic_ts=pulled_monotonic_ts,
            capture_source="rov_snapshot_appsink",
        )

    def _snapshot_frame_from_sample(
        self,
        sample,
        *,
        pulled_monotonic_ts: float,
        mime_type: str = "image/jpeg",
        extension: str = "jpg",
        capture_source: str = "rov_snapshot_appsink",
    ) -> SnapshotFrame:
        buf = sample.get_buffer()
        if buf is None:
            raise GstError(f"Snapshot sample for '{self.config.name}' had no buffer")
        source_pts_ns = _gst_time_to_ns(getattr(buf, "pts", None))
        source_dts_ns = _gst_time_to_ns(getattr(buf, "dts", None))
        source_duration_ns = _gst_time_to_ns(getattr(buf, "duration", None))
        source_monotonic_ts: float | None = None
        if source_pts_ns is not None and self._started_monotonic_ts is not None:
            source_monotonic_ts = float(self._started_monotonic_ts) + (float(source_pts_ns) / float(Gst.SECOND))
        # Absolute capture instant on the shared GstSystemClock: base_time + PTS.
        # Unlike started_monotonic_ts (a time.monotonic() snapshot taken after
        # set_state(PLAYING) returns), base_time is the real clock origin the
        # buffer PTS is measured against, so it is directly comparable across the
        # left/right pipelines and removes the per-stream startup-latency bias.
        source_clock_ns: int | None = None
        base_time_ns = self._pipeline_base_time_ns()
        if source_pts_ns is not None and base_time_ns is not None:
            source_clock_ns = base_time_ns + source_pts_ns
        size = int(buf.get_size())
        if size <= 0:
            raise GstError(f"Snapshot sample for '{self.config.name}' was empty")
        try:
            data = bytes(buf.extract_dup(0, size))
        except Exception as exc:
            raise GstError(f"Could not copy snapshot buffer for '{self.config.name}': {exc}") from exc
        if not data:
            raise GstError(f"Snapshot sample for '{self.config.name}' copied no bytes")
        caps = sample.get_caps()
        caps_text = caps.to_string() if caps is not None else ""
        with self._snapshot_lock:
            self._snapshot_seq += 1
            seq = self._snapshot_seq
        return SnapshotFrame(
            stream=self.config.name,
            data=data,
            mime_type=mime_type,
            caps=caps_text,
            wall_ts=time.time(),
            monotonic_ts=pulled_monotonic_ts,
            seq=seq,
            source_pts_ns=source_pts_ns,
            source_dts_ns=source_dts_ns,
            source_duration_ns=source_duration_ns,
            source_monotonic_ts=source_monotonic_ts,
            source_clock_ns=source_clock_ns,
            extension=extension,
            capture_source=capture_source,
        )

    def _start_snapshot_cache(self) -> None:
        if self._ondemand:
            self._start_au_ring()
            return
        if not _snapshot_cache_enabled(self.config):
            return
        sink = self._snapshot_sink()
        if sink is None:
            logger.warning("Snapshot cache requested for '%s' but no snapshot sink is ready", self.config.name)
            return
        with self._snapshot_cache_cond:
            self._snapshot_cache_frames = deque(maxlen=_snapshot_cache_frames(self.config))
            self._snapshot_cache_cond.notify_all()
        self._snapshot_cache_thread = threading.Thread(
            target=self._snapshot_cache_loop,
            name=f"snapshot-cache-{self.config.name}",
            daemon=True,
        )
        self._snapshot_cache_thread.start()

    def _start_au_ring(self) -> None:
        sink = self._snapshot_sink()
        if sink is None:
            logger.warning("On-demand snapshot ring requested for '%s' but no snapshot sink is ready", self.config.name)
            return
        with self._snapshot_cache_cond:
            self._au_ring = deque(maxlen=_snapshot_ring_aus(self.config))
            self._snapshot_cache_cond.notify_all()
        self._snapshot_cache_thread = threading.Thread(
            target=self._au_ring_loop,
            name=f"snapshot-au-{self.config.name}",
            daemon=True,
        )
        self._snapshot_cache_thread.start()

    def _stop_snapshot_cache(self) -> None:
        thread = self._snapshot_cache_thread
        self._snapshot_cache_thread = None
        with self._snapshot_cache_cond:
            self._snapshot_cache_cond.notify_all()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.75)

    def _snapshot_cache_loop(self) -> None:
        sink = self._snapshot_sink()
        if sink is None:
            return
        timeout_ns = int(0.2 * Gst.SECOND)
        while not self._stop_event.is_set():
            try:
                sample = sink.emit("try-pull-sample", timeout_ns)
                if sample is None:
                    continue
                pulled_monotonic_ts = time.monotonic()
                frame = self._snapshot_frame_from_sample(
                    sample,
                    pulled_monotonic_ts=pulled_monotonic_ts,
                    capture_source="rov_snapshot_cache",
                )
            except Exception as exc:
                if not self._stop_event.is_set():
                    logger.warning("Snapshot cache pull failed for '%s': %s", self.config.name, exc)
                time.sleep(0.05)
                continue
            with self._snapshot_cache_cond:
                self._snapshot_cache_frames.append(frame)
                self._snapshot_cache_cond.notify_all()

    def snapshot_cache_frames(self) -> list[SnapshotFrame]:
        with self._snapshot_cache_cond:
            return list(self._snapshot_cache_frames)

    def capture_cached_snapshot(
        self,
        *,
        timeout_s: float = 1.5,
        after_monotonic_ts: float | None = None,
    ) -> SnapshotFrame:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._snapshot_cache_cond:
            while True:
                frames = list(self._snapshot_cache_frames)
                if after_monotonic_ts is not None:
                    frames = [frame for frame in frames if float(frame.monotonic_ts) >= float(after_monotonic_ts)]
                if frames:
                    return frames[-1]
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._snapshot_cache_cond.wait(timeout=min(remaining, 0.1))
        raise TimeoutError(f"No cached onboard snapshot frame available for '{self.config.name}'")

    # ------------- On-demand compressed-GOP snapshot path ------------- #
    def _au_from_sample(self, sample) -> "_CompressedAU | None":
        buf = sample.get_buffer()
        if buf is None:
            return None
        size = int(buf.get_size())
        if size <= 0:
            return None
        try:
            data = bytes(buf.extract_dup(0, size))
        except Exception:
            return None
        if not data:
            return None
        pts_ns = _gst_time_to_ns(getattr(buf, "pts", None))
        base_ns = self._pipeline_base_time_ns()
        clock_ns = (base_ns + pts_ns) if (base_ns is not None and pts_ns is not None) else None
        try:
            keyframe = not bool(buf.has_flags(Gst.BufferFlags.DELTA_UNIT))
        except Exception:
            keyframe = False
        with self._snapshot_lock:
            self._snapshot_seq += 1
            seq = self._snapshot_seq
        return _CompressedAU(
            data=data,
            pts_ns=pts_ns,
            clock_ns=clock_ns,
            keyframe=keyframe,
            wall_ts=time.time(),
            seq=seq,
        )

    def _au_ring_loop(self) -> None:
        sink = self._snapshot_sink()
        if sink is None:
            return
        timeout_ns = int(0.2 * Gst.SECOND)
        while not self._stop_event.is_set():
            try:
                sample = sink.emit("try-pull-sample", timeout_ns)
                if sample is None:
                    continue
                au = self._au_from_sample(sample)
            except Exception as exc:
                if not self._stop_event.is_set():
                    logger.warning("Snapshot AU pull failed for '%s': %s", self.config.name, exc)
                time.sleep(0.05)
                continue
            if au is None:
                continue
            with self._snapshot_cache_cond:
                self._au_ring.append(au)
                self._snapshot_cache_cond.notify_all()

    def au_ring_frames(self) -> list["_CompressedAU"]:
        with self._snapshot_cache_cond:
            return list(self._au_ring)

    @staticmethod
    def _gop_segment(aus: list["_CompressedAU"], target_idx: int) -> "list[_CompressedAU] | None":
        """Return the contiguous run from the latest keyframe up to target_idx.

        Walks back from the target; the first keyframe reached is the latest one
        at/under the target. Returns None if a ring gap (non-consecutive seq) is
        hit before a keyframe, so a partial/broken GOP is never decoded.
        """
        if not aus or not (0 <= target_idx < len(aus)):
            return None
        i = target_idx
        while True:
            if aus[i].keyframe:
                return aus[i:target_idx + 1]
            if i == 0 or aus[i].seq != aus[i - 1].seq + 1:
                return None
            i -= 1

    def _ensure_decode_pipeline(self) -> None:
        """Build the persistent on-demand H.264->raw decode pipeline (kept PLAYING)."""
        if self._decode_pipeline is not None:
            return
        decoder = _snapshot_decoder(self.config)
        desc = (
            "appsrc name=src is-live=false format=time block=false max-bytes=0 "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
            f"h264parse config-interval=-1 ! {decoder} ! "
            "appsink name=out emit-signals=false sync=false max-buffers=16 drop=false"
        )
        pipeline = Gst.parse_launch(desc)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            raise GstError(f"on-demand decode pipeline failed to start for '{self.config.name}'")
        self._decode_pipeline = pipeline
        self._decode_src = pipeline.get_by_name("src")
        self._decode_out = pipeline.get_by_name("out")

    def _ensure_encode_pipeline(self, caps) -> None:
        """Build the persistent raw->JPEG encode pipeline once (target frame only)."""
        if self._encode_pipeline is not None:
            return
        q = min(100, max(1, _snapshot_quality(self.config)))
        desc = (
            "appsrc name=esrc is-live=false format=time block=true max-bytes=0 ! "
            "videoconvert ! "
            f"jpegenc quality={q} ! "
            "appsink name=eout emit-signals=false sync=false max-buffers=4 drop=false"
        )
        pipeline = Gst.parse_launch(desc)
        esrc = pipeline.get_by_name("esrc")
        try:
            esrc.set_property("caps", caps)
        except Exception:
            pass
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            raise GstError(f"on-demand encode pipeline failed to start for '{self.config.name}'")
        self._encode_pipeline = pipeline
        self._encode_src = esrc
        self._encode_out = pipeline.get_by_name("eout")

    def _stop_decode_pipeline(self) -> None:
        with self._decode_lock:
            pipelines = [self._decode_pipeline, self._encode_pipeline]
            self._decode_pipeline = None
            self._decode_src = None
            self._decode_out = None
            self._encode_pipeline = None
            self._encode_src = None
            self._encode_out = None
        for pipeline in pipelines:
            if pipeline is not None:
                try:
                    pipeline.set_state(Gst.State.NULL)
                except Exception:
                    pass

    def _encode_raw_sample_to_jpeg(self, sample, *, timeout_s: float = 2.0) -> bytes:
        self._ensure_encode_pipeline(sample.get_caps())
        esrc = self._encode_src
        eout = self._encode_out
        if esrc is None or eout is None:
            raise GstError("on-demand encode pipeline unavailable")
        while eout.emit("try-pull-sample", 0) is not None:
            pass
        # Copy (not extract_dup): copy() preserves GstVideoMeta so the encoder
        # reads the correct chroma plane strides. A raw byte copy loses the meta
        # and videoconvert then mis-strides the planes -> chroma speckle.
        gbuf = sample.get_buffer().copy()
        gbuf.pts = self._encode_pts_ns
        gbuf.dts = self._encode_pts_ns
        self._encode_pts_ns += 33_333_333
        if esrc.emit("push-buffer", gbuf) != Gst.FlowReturn.OK:
            raise GstError("encode push-buffer failed")
        deadline = time.monotonic() + max(0.5, float(timeout_s))
        while time.monotonic() < deadline:
            jpeg = eout.emit("try-pull-sample", int(0.2 * Gst.SECOND))
            if jpeg is None:
                continue
            jbuf = jpeg.get_buffer()
            data = bytes(jbuf.extract_dup(0, int(jbuf.get_size())))
            if not data:
                raise GstError("on-demand encode produced an empty frame")
            return data
        raise GstError("on-demand encode produced no frame")

    def _decode_aus_to_jpeg(self, aus: list["_CompressedAU"], *, quality: int, timeout_s: float = 3.0) -> bytes:
        """Decode a [keyframe..target] H.264 segment via the persistent per-stream
        decoder and JPEG-encode only the final (target) frame.

        Each segment starts with an IDR, which resets the decoder's references, so
        reusing the pipeline across captures is clean and avoids rebuilding
        openh264dec every time. Only the target frame is encoded, so the GOP
        frames decoded purely for references never pay an encode cost."""
        if not aus:
            raise GstError("no access units to decode")
        with self._decode_lock:
            self._ensure_decode_pipeline()
            src = self._decode_src
            out = self._decode_out
            if src is None or out is None:
                raise GstError("on-demand decode pipeline unavailable")
            # Drop any stale decoded frames from a previous capture.
            while out.emit("try-pull-sample", 0) is not None:
                pass
            for au in aus:
                gbuf = Gst.Buffer.new_allocate(None, len(au.data), None)
                gbuf.fill(0, au.data)
                # Monotonic synthetic PTS so the appsrc segment never goes
                # backward across captures (real AU order is preserved anyway).
                gbuf.pts = self._decode_pts_ns
                gbuf.dts = self._decode_pts_ns
                self._decode_pts_ns += 33_333_333
                if src.emit("push-buffer", gbuf) != Gst.FlowReturn.OK:
                    break
            expected = len(aus)
            pulled = 0
            last = None
            deadline = time.monotonic() + max(0.5, float(timeout_s))
            while pulled < expected and time.monotonic() < deadline:
                sample = out.emit("try-pull-sample", int(0.2 * Gst.SECOND))
                if sample is None:
                    continue
                last = sample
                pulled += 1
            if last is None:
                raise GstError("on-demand decode produced no frame")
            return self._encode_raw_sample_to_jpeg(last)

    def _frame_from_au(self, aus: list["_CompressedAU"], idx: int, *, quality: int) -> SnapshotFrame:
        segment = self._gop_segment(aus, idx)
        if segment is None:
            raise GstError(f"target access unit for '{self.config.name}' is not GOP-decodable yet")
        target = aus[idx]
        jpeg = self._decode_aus_to_jpeg(segment, quality=quality)
        with self._snapshot_lock:
            self._snapshot_seq += 1
            seq = self._snapshot_seq
        source_monotonic_ts: float | None = None
        if target.pts_ns is not None and self._started_monotonic_ts is not None:
            source_monotonic_ts = float(self._started_monotonic_ts) + (float(target.pts_ns) / float(Gst.SECOND))
        return SnapshotFrame(
            stream=self.config.name,
            data=jpeg,
            mime_type="image/jpeg",
            caps="",
            wall_ts=time.time(),
            monotonic_ts=time.monotonic(),
            seq=seq,
            source_pts_ns=target.pts_ns,
            source_dts_ns=None,
            source_duration_ns=None,
            source_monotonic_ts=source_monotonic_ts,
            source_clock_ns=target.clock_ns,
            extension="jpg",
            capture_source="rov_snapshot_ondemand",
        )

    @staticmethod
    def _select_target_index(aus: list["_CompressedAU"], target_clock_ns: int | None) -> int:
        if target_clock_ns is None:
            return len(aus) - 1
        best_i = len(aus) - 1
        best_d: int | None = None
        for i, au in enumerate(aus):
            if au.clock_ns is None:
                continue
            d = abs(int(au.clock_ns) - int(target_clock_ns))
            if best_d is None or d < best_d:
                best_d = d
                best_i = i
        return best_i

    def capture_ondemand_snapshot(self, *, timeout_s: float = 1.5, target_clock_ns: int | None = None) -> SnapshotFrame:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        quality = min(100, _snapshot_quality(self.config))
        while True:
            aus = self.au_ring_frames()
            if aus:
                idx: int | None
                if target_clock_ns is None:
                    # Newest *decodable* frame (the bleeding edge may have a tail gap).
                    idx = None
                    for i in range(len(aus) - 1, -1, -1):
                        if aus[i].clock_ns is not None and self._gop_segment(aus, i) is not None:
                            idx = i
                            break
                else:
                    idx = self._select_target_index(aus, target_clock_ns)
                    if idx is not None and self._gop_segment(aus, idx) is None:
                        idx = None
                if idx is not None:
                    return self._frame_from_au(aus, idx, quality=quality)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            with self._snapshot_cache_cond:
                self._snapshot_cache_cond.wait(timeout=min(remaining, 0.1))
        raise TimeoutError(f"No on-demand snapshot frame available for '{self.config.name}'")

    # ------------- Internals ------------- #
    def _start_bus_watcher(self):
        self._stop_event.clear()
        self._bus_thread = threading.Thread(target=self._bus_loop, name=f"bus-{self.config.name}", daemon=True)
        self._bus_thread.start()

    def _bus_loop(self):
        assert self._pipeline is not None
        bus = self._pipeline.get_bus()
        while not self._stop_event.is_set():
            msg = bus.timed_pop_filtered(
                200 * Gst.MSECOND,
                Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.STATE_CHANGED,
            )
            if msg is None:
                continue
            t = msg.type
            if t == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self._last_error = f"{err.message} | debug={dbg}"
                logger.error("Stream '%s' error: %s", self.config.name, self._last_error)
                self._set_state(Gst.State.NULL)
                self._teardown_pipeline()
                break
            elif t == Gst.MessageType.EOS:
                logger.info("Stream '%s' EOS", self.config.name)
                self._set_state(Gst.State.NULL)
                self._teardown_pipeline()
                break
            elif t == Gst.MessageType.STATE_CHANGED and msg.src == self._pipeline:
                old, new, pend = msg.parse_state_changed()
                logger.debug("Stream '%s' state: %s -> %s", self.config.name, old.value_nick, new.value_nick)

    def _send_eos(self, timeout: float = 1.5):
        if not self._pipeline:
            return
        self._pipeline.send_event(Gst.Event.new_eos())
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.05)

    def _teardown_pipeline(self):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
        if self._bus_thread is not None:
            self._stop_event.set()
            self._bus_thread = None

    def _set_state(self, state: Gst.State):
        if not self._pipeline:
            return
        ret = self._pipeline.set_state(state)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise GstError(f"Failed to set pipeline state to {state}")

    def _pipeline_base_time_ns(self) -> int | None:
        """Return the pipeline's GstClock base time in ns (shared across streams)."""
        pipeline = self._pipeline
        if pipeline is None:
            return None
        try:
            base_time = pipeline.get_base_time()
        except Exception:
            return None
        return _gst_time_to_ns(base_time)

    def _snapshot_sink(self) -> Optional[Gst.Element]:
        pipeline = self._pipeline
        if pipeline is None:
            return None
        try:
            sink = pipeline.get_by_name("snapshot_sink")
            if sink is not None:
                return sink
        except Exception:
            pass
        return self._find_element(pipeline, "appsink")

    # ------------- Pipeline build ------------- #
    def _build_pipeline(self, cfg: StreamConfig) -> Gst.Pipeline:
        if cfg.transport not in {"udp", "tcp"}:
            raise ValueError("transport must be 'udp' or 'tcp'")
        if cfg.transport == "udp" and not cfg.host:
            raise ValueError("host is required for UDP transport")

        source_parts = []
        stream_parts = []
        vf = cfg.video_format.lower()
        dev = resolve_v4l2_device(cfg.device, prefer_h264=(vf == "h264"))
        rtp_mtu = max(576, int(cfg.rtp_mtu))

        if vf == "h264":
            apply_h264_quality_controls(
                dev,
                h264_bitrate=int(cfg.h264_bitrate),
                h264_gop=int(cfg.h264_gop),
                extra=cfg.extra,
                logger=logger,
            )

        source_parts.append(_v4l2src_part(dev, cfg))

        if vf == "mjpeg":
            # Camera outputs MJPEG.
            # Either send as RTP/JPEG or transcode to H.264 (software, x264enc).
            source_parts.append(f"image/jpeg,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1")
            source_parts += _sender_queue_parts(cfg, "q_capture")
            if (cfg.encode or "").lower() == "h264":
                kbps = max(1, int(cfg.h264_bitrate // 1000))
                stream_parts += [
                    "jpegdec",
                    "videoconvert",
                    f"video/x-raw,format=I420,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1",
                    *_sender_queue_parts(cfg, "q_encode"),
                    f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={kbps} key-int-max={cfg.h264_gop}",
                    "h264parse config-interval=-1 disable-passthrough=true",
                    *_sender_queue_parts(cfg, "q_pay"),
                    f"rtph264pay config-interval=1 pt={cfg.rtp_pt_h264} mtu={rtp_mtu}",
                ]
            else:
                stream_parts.append(f"rtpjpegpay pt={cfg.rtp_pt_jpeg} mtu={rtp_mtu}")
        elif vf == "h264":
            # Camera outputs H.264
            # Be permissive about stream-format; some cameras output byte-stream.
            source_parts.append(f"video/x-h264,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1,alignment=au")
            source_parts += _sender_queue_parts(cfg, "q_capture")
            stream_parts.append("h264parse config-interval=-1 disable-passthrough=true")
            stream_parts += _sender_queue_parts(cfg, "q_pay")
            stream_parts.append(f"rtph264pay config-interval=1 pt={cfg.rtp_pt_h264} mtu={rtp_mtu}")
        elif vf == "raw":
            # Camera outputs raw â†’ must encode
            if cfg.encode == "h264":
                kbps = max(1, int(cfg.h264_bitrate // 1000))
                source_parts.append(f"video/x-raw,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1")
                source_parts += _sender_queue_parts(cfg, "q_capture")
                stream_parts += [
                    "videoconvert",
                    f"video/x-raw,format=I420,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1",
                    *_sender_queue_parts(cfg, "q_encode"),
                    f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={kbps} key-int-max={cfg.h264_gop}",
                    "h264parse config-interval=-1 disable-passthrough=true",
                    *_sender_queue_parts(cfg, "q_pay"),
                    f"rtph264pay config-interval=1 pt={cfg.rtp_pt_h264} mtu={rtp_mtu}",
                ]
            elif cfg.encode == "mjpeg":
                source_parts.append(f"video/x-raw,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1")
                source_parts += _sender_queue_parts(cfg, "q_capture")
                stream_parts.append("jpegenc")
                stream_parts.append(f"rtpjpegpay pt={cfg.rtp_pt_jpeg} mtu={rtp_mtu}")
            else:
                raise ValueError("For video_format='raw', encode must be 'h264' or 'mjpeg'")
        else:
            raise ValueError("video_format must be 'mjpeg', 'raw', or 'h264'")

        if cfg.transport == "udp":
            # udpsink supports bind-address to select source interface.
            # Note: This is best-effort; if the address is not present on the
            # system, GStreamer will error and the stream will fail to start.
            sync_prop = f"sync={'true' if cfg.sync else 'false'}"
            udp_buffer = f"buffer-size={max(0, int(cfg.udp_buffer_size))}"
            props = [
                f"clients={_udp_clients(cfg)}",
                udp_buffer,
                sync_prop,
                "async=false",
            ]
            if cfg.bind_address:
                props.append(f"bind-address={cfg.bind_address}")
            stream_parts.append("multiudpsink " + " ".join(props))
        else:
            stream_parts.append(f"tcpserversink host=0.0.0.0 port={cfg.port}")

        if _snapshot_enabled(cfg):
            tee_name = "snapshot_tee"
            snapshot_parts = _snapshot_branch_parts(cfg)
            if not snapshot_parts:
                desc = " ! ".join(source_parts + stream_parts)
            else:
                prefix = " ! ".join(source_parts + [f"tee name={tee_name}"])
                main_parts = [_snapshot_stream_queue_part(), *stream_parts]
                desc = (
                    f"{prefix} {tee_name}. ! {' ! '.join(main_parts)} "
                    f"{tee_name}. ! {' ! '.join(snapshot_parts)}"
                )
        else:
            desc = " ! ".join(source_parts + stream_parts)
        logger.debug("Pipeline(%s): %s", cfg.name, desc)
        try:
            pipeline = Gst.parse_launch(desc)
        except Exception as e:
            raise GstError(f"Failed to build pipeline: {e}\nDesc: {desc}")
        if cfg.transport == "udp":
            self._apply_udp_sink_qos(pipeline, cfg)
        return pipeline


    def _apply_udp_sink_qos(self, pipeline: Gst.Pipeline, cfg: StreamConfig) -> None:
        """Best-effort DSCP marking for video UDP traffic so control/telemetry can be prioritized."""
        try:
            udpsink = self._find_element(pipeline, "udpsink") or self._find_element(pipeline, "multiudpsink")
            if not udpsink:
                return
            dscp = None
            try:
                dscp = cfg.extra.get("udp_qos_dscp")
            except Exception:
                dscp = None
            if dscp is None:
                # CS1 (8) = low priority / scavenger class on many networks.
                dscp = 8
            dscp = int(dscp)
            if dscp < 0:
                return
            # Some GStreamer builds expose this on udpsink/udpsrc. Best-effort only.
            try:
                udpsink.set_property("qos-dscp", dscp)
                logger.info("Stream '%s' udpsink qos-dscp=%s", cfg.name, dscp)
            except Exception:
                pass
        except Exception:
            pass

    @staticmethod
    def _is_live_update(old: StreamConfig, new: StreamConfig) -> Tuple[bool, Dict[str, Tuple[Any, Any]]]:
        changes: Dict[str, Tuple[Any, Any]] = {}
        for f in asdict(old):
            ov, nv = getattr(old, f), getattr(new, f)
            if ov != nv:
                changes[f] = (ov, nv)

        live_keys = {"host", "port", "h264_bitrate", "extra"}
        if changes and all(k in live_keys for k in changes.keys()):
            if "extra" in changes and not _only_udp_mirrors_changed(old.extra, new.extra):
                return False, changes
            return True, changes
        return False, changes

    def _apply_live_updates(self, cfg: StreamConfig) -> None:
        assert self._pipeline is not None
        multiudpsink = self._find_element(self._pipeline, "multiudpsink")
        if multiudpsink:
            try:
                clients = _udp_clients(cfg)
                multiudpsink.set_property("clients", clients)
                logger.info("Updated multiudpsink clients -> %s", clients)
            except Exception as e:
                logger.warning("Failed to live-set multiudpsink clients: %s", e)

        udpsink = None if multiudpsink else self._find_element(self._pipeline, "udpsink")
        if udpsink:
            if self.config.host != cfg.host:
                try:
                    udpsink.set_property("host", cfg.host)
                    logger.info("Updated udpsink host -> %s", cfg.host)
                except Exception as e:
                    logger.warning("Failed to live-set udpsink host: %s", e)
            if self.config.port != cfg.port:
                try:
                    udpsink.set_property("port", cfg.port)
                    logger.info("Updated udpsink port -> %s", cfg.port)
                except Exception as e:
                    logger.warning("Failed to live-set udpsink port: %s", e)

        # Update x264 bitrate live (x264enc bitrate is in kbit/sec)
        is_h264_path = (
            (self.config.video_format == "raw" and self.config.encode == "h264")
            or (self.config.video_format == "mjpeg" and (self.config.encode or "").lower() == "h264")
        )
        if is_h264_path and (self.config.h264_bitrate != cfg.h264_bitrate):
            enc = self._find_element(self._pipeline, "x264enc")
            if enc:
                try:
                    enc.set_property("bitrate", max(1, int(cfg.h264_bitrate // 1000)))
                    logger.info("Updated x264 bitrate -> %s kbps", max(1, int(cfg.h264_bitrate // 1000)))
                except Exception as e:
                    logger.warning("Failed to live-set x264 bitrate: %s", e)

    @staticmethod
    def _find_element(pipeline: Gst.Pipeline, factory_name: str) -> Optional[Gst.Element]:
        it = pipeline.iterate_elements()
        while True:
            ok, elem = it.next()
            if ok == Gst.IteratorResult.DONE:
                break
            if ok != Gst.IteratorResult.OK:
                continue
            try:
                if elem.get_factory().get_name() == factory_name:
                    return elem
            except Exception:
                continue
        return None


class StreamManager:
    """Thread-safe registry for named ``GstStream`` instances."""

    def __init__(self):
        self._streams: Dict[str, GstStream] = {}
        self._lock = threading.Lock()

    def start_stream(self, config: StreamConfig) -> GstStream:
        """Start and register a new named stream."""

        with self._lock:
            if config.name in self._streams:
                raise ValueError(f"Stream '{config.name}' already exists")
            st = GstStream(config)
            st.start()
            self._streams[config.name] = st
            return st

    def stop_stream(self, name: str) -> None:
        """Stop and unregister a stream by name."""

        with self._lock:
            st = self._streams.pop(name, None)
        if st:
            st.stop()

    def stop_all(self) -> None:
        """Stop every registered stream."""

        with self._lock:
            names = list(self._streams.keys())
        for n in names:
            self.stop_stream(n)

    def get_stream(self, name: str) -> Optional[GstStream]:
        """Return a stream by name, if it exists."""

        with self._lock:
            return self._streams.get(name)

    def list_streams(self) -> Dict[str, StreamConfig]:
        """Return stream configurations keyed by stream name."""

        with self._lock:
            return {n: s.config for n, s in self._streams.items()}

    def list_stream_status(self) -> Dict[str, Dict[str, Any]]:
        """Return stream configurations and diagnostic timing metadata."""

        with self._lock:
            return {n: s.status() for n, s in self._streams.items()}

    def update_stream(self, name: str, **updates) -> None:
        """Update one registered stream by name."""

        st = self.get_stream(name)
        if not st:
            raise KeyError(f"No such stream: {name}")
        st.update(**updates)

    def capture_snapshot(self, name: str, *, timeout_s: float = 1.5) -> SnapshotFrame:
        """Capture one onboard still image from a running stream."""

        st = self.get_stream(name)
        if not st:
            raise KeyError(f"No such stream: {name}")
        return st.capture_snapshot(timeout_s=timeout_s)

    @staticmethod
    def _capture_snapshot_pair_once(
        left_stream: GstStream,
        right_stream: GstStream,
        *,
        timeout_s: float,
    ) -> tuple[SnapshotFrame, SnapshotFrame]:
        start = threading.Event()
        results: dict[str, SnapshotFrame] = {}
        errors: dict[str, BaseException] = {}

        def _worker(side: str, stream: GstStream) -> None:
            try:
                start.wait()
                results[side] = stream.capture_snapshot(timeout_s=timeout_s, fresh=True)
            except BaseException as exc:
                errors[side] = exc

        threads = [
            threading.Thread(target=_worker, args=("left", left_stream), name="stereo-snapshot-left", daemon=True),
            threading.Thread(target=_worker, args=("right", right_stream), name="stereo-snapshot-right", daemon=True),
        ]
        for thread in threads:
            thread.start()
        start.set()
        join_timeout = max(0.0, float(timeout_s)) + 0.25
        for thread in threads:
            thread.join(join_timeout)
        alive = [thread.name for thread in threads if thread.is_alive()]
        if alive:
            raise TimeoutError(f"Timed out waiting for stereo snapshot workers: {', '.join(alive)}")
        if errors:
            side, exc = next(iter(errors.items()))
            raise GstError(f"{side} stereo snapshot failed: {exc}") from exc
        if "left" not in results or "right" not in results:
            raise TimeoutError("Stereo snapshot workers returned no pair")
        return results["left"], results["right"]

    @staticmethod
    def _frame_pair_time(frame: SnapshotFrame) -> tuple[float, str]:
        # Prefer the shared-clock capture instant (base_time + PTS); it is the
        # only timestamp directly comparable across the two pipelines.
        clock_ns = getattr(frame, "source_clock_ns", None)
        if clock_ns is not None:
            try:
                return float(clock_ns) / 1e9, "source_clock"
            except Exception:
                pass
        source_ts = getattr(frame, "source_monotonic_ts", None)
        if source_ts is not None:
            try:
                return float(source_ts), "source_monotonic"
            except Exception:
                pass
        return float(frame.monotonic_ts), "pull_monotonic"

    @classmethod
    def _pair_delta_ms(cls, left_frame: SnapshotFrame, right_frame: SnapshotFrame) -> tuple[float, str]:
        left_ts, left_source = cls._frame_pair_time(left_frame)
        right_ts, right_source = cls._frame_pair_time(right_frame)
        source = "source_monotonic" if left_source == right_source == "source_monotonic" else "pull_monotonic"
        return abs(left_ts - right_ts) * 1000.0, source

    @classmethod
    def _best_frame_pair(
        cls,
        left_frames: list[SnapshotFrame],
        right_frames: list[SnapshotFrame],
    ) -> tuple[float, str, SnapshotFrame, SnapshotFrame] | None:
        best: tuple[float, str, SnapshotFrame, SnapshotFrame] | None = None
        for left_frame in left_frames:
            for right_frame in right_frames:
                delta_ms, source = cls._pair_delta_ms(left_frame, right_frame)
                if best is None or delta_ms < best[0]:
                    best = (delta_ms, source, left_frame, right_frame)
        return best

    @classmethod
    def _capture_cached_snapshot_pair(
        cls,
        left_stream: GstStream,
        right_stream: GstStream,
        *,
        timeout_s: float,
        max_pair_delta_ms: float,
    ) -> StereoSnapshotPair:
        request_ts = time.monotonic()
        deadline = request_ts + max(0.0, float(timeout_s))
        max_delta_ms = max(0.0, float(max_pair_delta_ms))
        best: tuple[float, str, SnapshotFrame, SnapshotFrame] | None = None
        attempts = 0
        while True:
            attempts += 1
            left_frames = [frame for frame in left_stream.snapshot_cache_frames() if frame.monotonic_ts >= request_ts]
            right_frames = [frame for frame in right_stream.snapshot_cache_frames() if frame.monotonic_ts >= request_ts]
            current = cls._best_frame_pair(left_frames, right_frames)
            if current is not None:
                if best is None or current[0] < best[0]:
                    best = current
                if current[0] <= max_delta_ms:
                    delta_ms, source, left_frame, right_frame = current
                    return StereoSnapshotPair(
                        left=left_frame,
                        right=right_frame,
                        pair_delta_ms=delta_ms,
                        timestamp_source=f"rov_snapshot_cache_{source}",
                        attempts=attempts,
                    )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.01, remaining))

        if best is not None:
            delta_ms, source, left_frame, right_frame = best
            raise TimeoutError(
                f"Could not capture cached stereo pair within {max_delta_ms:.1f} ms "
                f"(best delta {delta_ms:.1f} ms across {attempts} attempt(s); "
                f"source={source}; left_seq={left_frame.seq} right_seq={right_frame.seq})"
            )
        raise TimeoutError(f"No cached stereo frames arrived within {max(0.0, float(timeout_s)):.2f} s")

    @staticmethod
    def _newest_decodable_index(stream: "GstStream", aus: list["_CompressedAU"]) -> int | None:
        """Newest AU (with a clock) whose GOP is complete in the ring.

        The bleeding-edge AU is where the leaky ring occasionally drops a frame
        (seq gap) under load, so the very newest is sometimes not yet
        GOP-decodable. Backing off to the newest decodable AU avoids spinning
        while a fresh keyframe settles the tail.
        """
        for i in range(len(aus) - 1, -1, -1):
            if aus[i].clock_ns is not None and stream._gop_segment(aus, i) is not None:
                return i
        return None

    @staticmethod
    def _closest_decodable_index(stream: "GstStream", aus: list["_CompressedAU"], target_clock_ns: int) -> int | None:
        best_i = None
        best_d: int | None = None
        for i, au in enumerate(aus):
            if au.clock_ns is None:
                continue
            if stream._gop_segment(aus, i) is None:
                continue
            d = abs(int(au.clock_ns) - int(target_clock_ns))
            if best_d is None or d < best_d:
                best_d = d
                best_i = i
        return best_i

    @classmethod
    def _pick_decodable_au_pair(
        cls,
        left_stream: "GstStream",
        right_stream: "GstStream",
        left_aus: list["_CompressedAU"],
        right_aus: list["_CompressedAU"],
    ) -> "tuple[float, int, int] | None":
        """Pick the most recent shared instant where BOTH sides are decodable.

        Targets ``min(newest_decodable_left, newest_decodable_right)`` then takes
        the closest decodable AU on each side. Both returned indices are
        guaranteed GOP-decodable, so the caller never decodes a frame that isn't
        ready (which previously caused a ~400ms retry spin). Returns
        ``(delta_ms, left_idx, right_idx)`` or None.
        """
        li0 = cls._newest_decodable_index(left_stream, left_aus)
        ri0 = cls._newest_decodable_index(right_stream, right_aus)
        if li0 is None or ri0 is None:
            return None
        lc0 = left_aus[li0].clock_ns
        rc0 = right_aus[ri0].clock_ns
        if lc0 is None or rc0 is None:
            return None
        target = min(int(lc0), int(rc0))
        li = cls._closest_decodable_index(left_stream, left_aus, target)
        ri = cls._closest_decodable_index(right_stream, right_aus, target)
        if li is None or ri is None:
            return None
        lc = left_aus[li].clock_ns
        rc = right_aus[ri].clock_ns
        if lc is None or rc is None:
            return None
        return abs(int(lc) - int(rc)) / 1e6, li, ri

    @classmethod
    def _capture_ondemand_pair(
        cls,
        left_stream: GstStream,
        right_stream: GstStream,
        *,
        timeout_s: float,
        max_pair_delta_ms: float,
    ) -> StereoSnapshotPair:
        """Pair two compressed-AU rings on the shared clock and decode on demand."""
        quality_l = min(100, _snapshot_quality(left_stream.config))
        quality_r = min(100, _snapshot_quality(right_stream.config))
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        max_delta_ms = max(0.0, float(max_pair_delta_ms))
        attempts = 0
        best_delta: float | None = None

        def _decode_pair(li, ri, left_aus, right_aus) -> "tuple[SnapshotFrame, SnapshotFrame] | None":
            # Decode sequentially: software openh264dec saturates the Pi's cores,
            # so two concurrent 1080p decodes contend and run far slower than the
            # sum of two sequential decodes. The frames were already selected by
            # shared clock, so pairing accuracy does not depend on decode timing.
            try:
                left_frame = left_stream._frame_from_au(left_aus, li, quality=quality_l)
                right_frame = right_stream._frame_from_au(right_aus, ri, quality=quality_r)
            except GstError:
                return None
            return left_frame, right_frame

        while True:
            attempts += 1
            left_aus = left_stream.au_ring_frames()
            right_aus = right_stream.au_ring_frames()
            pick = cls._pick_decodable_au_pair(left_stream, right_stream, left_aus, right_aus)
            if pick is not None:
                delta_ms, li, ri = pick
                if best_delta is None or delta_ms < best_delta:
                    best_delta = delta_ms
                if delta_ms <= max_delta_ms:
                    # Both indices are already guaranteed decodable by the picker.
                    decoded = _decode_pair(li, ri, left_aus, right_aus)
                    if decoded is not None:
                        left_frame, right_frame = decoded
                        return StereoSnapshotPair(
                            left=left_frame,
                            right=right_frame,
                            pair_delta_ms=delta_ms,
                            timestamp_source="rov_snapshot_ondemand_source_clock",
                            attempts=attempts,
                        )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.01, remaining))

        best_txt = f"{best_delta:.1f} ms" if best_delta is not None else "n/a"
        raise TimeoutError(
            f"Could not capture on-demand stereo pair within {max_delta_ms:.1f} ms "
            f"(best delta {best_txt} across {attempts} attempt(s))"
        )

    def capture_stereo_pair(
        self,
        left: str,
        right: str,
        *,
        timeout_s: float = 2.0,
        max_pair_delta_ms: float = 50.0,
    ) -> StereoSnapshotPair:
        """Capture a fresh left/right onboard JPEG pair with common-process timing."""

        left_name = str(left or "").strip()
        right_name = str(right or "").strip()
        if not left_name or not right_name:
            raise ValueError("left and right stream names are required")
        if left_name == right_name:
            raise ValueError("left and right streams must be different")
        with self._lock:
            left_stream = self._streams.get(left_name)
            right_stream = self._streams.get(right_name)
        if left_stream is None:
            raise KeyError(f"No such stream: {left_name}")
        if right_stream is None:
            raise KeyError(f"No such stream: {right_name}")

        left_cfg = getattr(left_stream, "config", None)
        right_cfg = getattr(right_stream, "config", None)
        if left_cfg is not None and right_cfg is not None and _snapshot_ondemand(left_cfg) and _snapshot_ondemand(right_cfg):
            return self._capture_ondemand_pair(
                left_stream,
                right_stream,
                timeout_s=timeout_s,
                max_pair_delta_ms=max_pair_delta_ms,
            )
        if left_cfg is not None and right_cfg is not None and _snapshot_cache_enabled(left_cfg) and _snapshot_cache_enabled(right_cfg):
            return self._capture_cached_snapshot_pair(
                left_stream,
                right_stream,
                timeout_s=timeout_s,
                max_pair_delta_ms=max_pair_delta_ms,
            )

        deadline = time.monotonic() + max(0.0, float(timeout_s))
        max_delta_ms = max(0.0, float(max_pair_delta_ms))
        attempts = 0
        best: tuple[float, str, SnapshotFrame, SnapshotFrame] | None = None
        last_error: BaseException | None = None
        while True:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                break
            attempts += 1
            try:
                left_frame, right_frame = self._capture_snapshot_pair_once(
                    left_stream,
                    right_stream,
                    timeout_s=remaining_s,
                )
            except BaseException as exc:
                last_error = exc
                break
            delta_ms, pair_time_source = self._pair_delta_ms(left_frame, right_frame)
            if best is None or delta_ms < best[0]:
                best = (delta_ms, pair_time_source, left_frame, right_frame)
            if delta_ms <= max_delta_ms:
                return StereoSnapshotPair(
                    left=left_frame,
                    right=right_frame,
                    pair_delta_ms=delta_ms,
                    timestamp_source=f"rov_snapshot_appsink_fresh_{pair_time_source}",
                    attempts=attempts,
                )

        if best is not None:
            delta_ms, pair_time_source, left_frame, right_frame = best
            left_source = str(getattr(left_frame, "capture_source", "") or "unknown")
            right_source = str(getattr(right_frame, "capture_source", "") or "unknown")
            raise TimeoutError(
                f"Could not capture stereo pair within {max_delta_ms:.1f} ms "
                f"(best delta {delta_ms:.1f} ms across {attempts} attempt(s); "
                f"time_source={pair_time_source}; sources left={left_source} right={right_source})"
            )
        if last_error is not None:
            raise TimeoutError(f"Could not capture stereo pair: {last_error}") from last_error
        raise TimeoutError(f"Could not capture stereo pair within {max(0.0, float(timeout_s)):.2f} s")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Raspberry Pi GStreamer sender")
    ap.add_argument("--name", default="cam0")
    ap.add_argument("--device", default="/dev/v4l/by-path/*video-index2")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--video-format", default="h264", choices=["mjpeg", "raw", "h264"])
    ap.add_argument("--encode", default=None, choices=[None, "h264", "mjpeg"])
    ap.add_argument("--h264-bitrate", type=int, default=4_000_000)
    ap.add_argument("--h264-gop", type=int, default=30)
    ap.add_argument("--rtp-mtu", type=int, default=1200)
    ap.add_argument("--udp-buffer-size", type=int, default=1024 * 1024)
    ap.add_argument("--transport", default="udp", choices=["udp", "tcp"])
    ap.add_argument("--host", default="192.168.1.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)

    cfg = StreamConfig(
        name=args.name,
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        video_format=args.video_format,
        encode=args.encode,
        h264_bitrate=args.h264_bitrate,
        h264_gop=args.h264_gop,
        rtp_mtu=args.rtp_mtu,
        udp_buffer_size=args.udp_buffer_size,
        transport=args.transport,
        host=args.host,
        port=args.port,
    )

    mgr = StreamManager()
    try:
        mgr.start_stream(cfg)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mgr.stop_all()
