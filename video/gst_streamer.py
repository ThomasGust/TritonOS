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
    # This helps keep video on the tether even if Wi‑Fi is enabled.
    bind_address: Optional[str] = None

    # RTP payload types
    rtp_pt_jpeg: int = 26
    rtp_pt_h264: int = 96

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
            logger.info("Stream '%s' started", self.config.name)

    def stop(self, timeout: float = 5.0) -> None:
        """Stop the pipeline, send EOS best-effort, and release resources."""

        with self._state_lock:
            if self._pipeline is None:
                return
            logger.info("Stopping stream '%s'", self.config.name)
            self._stop_event.set()
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
        }

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

    # ------------- Pipeline build ------------- #
    def _build_pipeline(self, cfg: StreamConfig) -> Gst.Pipeline:
        if cfg.transport not in {"udp", "tcp"}:
            raise ValueError("transport must be 'udp' or 'tcp'")
        if cfg.transport == "udp" and not cfg.host:
            raise ValueError("host is required for UDP transport")

        parts = []
        vf = cfg.video_format.lower()
        dev = resolve_v4l2_device(cfg.device, prefer_h264=(vf == "h264"))

        if vf == "h264":
            apply_h264_quality_controls(
                dev,
                h264_bitrate=int(cfg.h264_bitrate),
                h264_gop=int(cfg.h264_gop),
                extra=cfg.extra,
                logger=logger,
            )

        parts.append(f"v4l2src device={dev}")

        if vf == "mjpeg":
            # Camera outputs MJPEG.
            # Either send as RTP/JPEG or transcode to H.264 (software, x264enc).
            parts.append(f"image/jpeg,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1")
            if (cfg.encode or "").lower() == "h264":
                kbps = max(1, int(cfg.h264_bitrate // 1000))
                parts += [
                    "jpegdec",
                    "videoconvert",
                    f"video/x-raw,format=I420,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1",
                    f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={kbps} key-int-max={cfg.h264_gop}",
                    "h264parse",
                    f"rtph264pay config-interval=1 pt={cfg.rtp_pt_h264}",
                ]
            else:
                parts.append(f"rtpjpegpay pt={cfg.rtp_pt_jpeg}")
        elif vf == "h264":
            # Camera outputs H.264
            # Be permissive about stream-format; some cameras output byte-stream.
            parts.append(f"video/x-h264,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1,alignment=au")
            parts.append("h264parse")
            parts.append(f"rtph264pay config-interval=1 pt={cfg.rtp_pt_h264}")
        elif vf == "raw":
            # Camera outputs raw → must encode
            if cfg.encode == "h264":
                kbps = max(1, int(cfg.h264_bitrate // 1000))
                parts += [
                    f"video/x-raw,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1",
                    "videoconvert",
                    f"video/x-raw,format=I420,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1",
                    f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={kbps} key-int-max={cfg.h264_gop}",
                    "h264parse",
                    f"rtph264pay config-interval=1 pt={cfg.rtp_pt_h264}",
                ]
            elif cfg.encode == "mjpeg":
                parts.append(f"video/x-raw,width={cfg.width},height={cfg.height},framerate={cfg.fps}/1")
                parts.append("jpegenc")
                parts.append(f"rtpjpegpay pt={cfg.rtp_pt_jpeg}")
            else:
                raise ValueError("For video_format='raw', encode must be 'h264' or 'mjpeg'")
        else:
            raise ValueError("video_format must be 'mjpeg', 'raw', or 'h264'")

        if cfg.transport == "udp":
            # udpsink supports bind-address to select source interface.
            # Note: This is best-effort; if the address is not present on the
            # system, GStreamer will error and the stream will fail to start.
            sync_prop = f"sync={'true' if cfg.sync else 'false'}"
            if cfg.bind_address:
                parts.append(
                    f"udpsink host={cfg.host} port={cfg.port} bind-address={cfg.bind_address} "
                    f"{sync_prop} async=false"
                )
            else:
                parts.append(f"udpsink host={cfg.host} port={cfg.port} {sync_prop} async=false")
        else:
            parts.append(f"tcpserversink host=0.0.0.0 port={cfg.port}")

        desc = " ! ".join(parts)
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
            udpsink = self._find_element(pipeline, "udpsink")
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

        # allow live updates only for a few settings
        live_keys = {"host", "port", "h264_bitrate"}
        if changes and all(k in live_keys for k in changes.keys()):
            return True, changes
        return False, changes

    def _apply_live_updates(self, cfg: StreamConfig) -> None:
        assert self._pipeline is not None
        udpsink = self._find_element(self._pipeline, "udpsink")
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
