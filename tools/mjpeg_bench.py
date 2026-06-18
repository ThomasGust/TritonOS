#!/usr/bin/env python3
"""Bench: can the exploreHD cameras serve H.264 (display) + MJPEG (capture) at once?

Opens several v4l2 capture pipelines simultaneously and measures the sustained
delivery rate (fps + MB/s) of each, plus CPU and SoC temperature/throttling. This
is the decision gate for adding a dedicated low-latency MJPEG capture stream
alongside the existing H.264 display streams (see the media-capture plan).

Standalone: only needs python3-gi + the GStreamer v4l2/jpeg plugins (already on the
Pi). It does NOT import TritonOS, so it can be scp'd to the Pi on its own.

Examples (run on the Pi, with the ROV NOT actively streaming so the cameras are free):
    python3 mjpeg_bench.py --scenario pair --seconds 12
    python3 mjpeg_bench.py --scenario full --seconds 12 --mjpeg-size 1280x720

Scenarios:
    capture   Primary+Aux MJPEG only (2 streams)
    pair      Primary+Aux: H.264 + MJPEG each (4 streams)
    full      all 4 cams H.264 (display sim) + Primary+Aux MJPEG (6 streams)  [default]
"""
from __future__ import annotations

import argparse
import glob
import subprocess
import threading
import time
from dataclasses import dataclass, field

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

# USB hub port suffix per camera (matches data/streams.json device globs).
CAM_PORTS = {
    "Arm": "1.2.3",
    "BackGripper": "1.2.4",
    "Primary": "1.2.5",
    "Aux": "1.2.6",
}


def resolve_node(port: str, index: int) -> str | None:
    """Return the /dev/videoN for a camera USB port + v4l2 node index."""
    pattern = f"/dev/v4l/by-path/*usb-0:{port}:1.0-video-index{index}"
    matches = sorted(glob.glob(pattern))
    return matches[0] if matches else None


@dataclass
class StreamSpec:
    label: str
    device: str
    fmt: str            # "h264" or "mjpeg"
    width: int
    height: int
    fps: int


@dataclass
class StreamStat:
    spec: StreamSpec
    pipeline: Gst.Pipeline | None = None
    frames: int = 0
    total_bytes: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0
    error: str = ""
    lock: threading.Lock = field(default_factory=threading.Lock)

    def on_sample(self, appsink) -> Gst.FlowReturn:
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        size = buf.get_size() if buf is not None else 0
        now = time.monotonic()
        with self.lock:
            if self.first_ts == 0.0:
                self.first_ts = now
            self.last_ts = now
            self.frames += 1
            self.total_bytes += int(size)
        return Gst.FlowReturn.OK


def build_pipeline(spec: StreamSpec, stat: StreamStat) -> Gst.Pipeline:
    if spec.fmt == "h264":
        caps = (
            f"video/x-h264,width={spec.width},height={spec.height},"
            f"framerate={spec.fps}/1,alignment=au"
        )
        chain = f"{caps} ! h264parse"
    elif spec.fmt == "mjpeg":
        caps = f"image/jpeg,width={spec.width},height={spec.height},framerate={spec.fps}/1"
        chain = caps
    else:
        raise ValueError(spec.fmt)
    desc = (
        f"v4l2src device={spec.device} do-timestamp=true ! {chain} ! "
        f"appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true"
    )
    pipeline = Gst.parse_launch(desc)
    sink = pipeline.get_by_name("sink")
    sink.connect("new-sample", stat.on_sample)

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def _on_msg(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, _dbg = msg.parse_error()
            with stat.lock:
                if not stat.error:
                    stat.error = str(err)

    bus.connect("message", _on_msg)
    return pipeline


def read_cpu_times() -> tuple[int, int]:
    """Return (idle, total) jiffies from /proc/stat aggregate line."""
    try:
        with open("/proc/stat", "r") as f:
            parts = f.readline().split()
        nums = [int(x) for x in parts[1:]]
        idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
        return idle, sum(nums)
    except Exception:
        return 0, 0


def soc_status() -> str:
    out = []
    for arg in ("measure_temp", "get_throttled"):
        try:
            r = subprocess.run(
                ["vcgencmd", arg], capture_output=True, text=True, timeout=2
            )
            out.append(r.stdout.strip())
        except Exception:
            pass
    return " ".join(out)


def scenario_specs(name: str, mjpeg_w: int, mjpeg_h: int, fps: int) -> list[StreamSpec]:
    specs: list[StreamSpec] = []

    def h264(cam: str) -> StreamSpec | None:
        dev = resolve_node(CAM_PORTS[cam], 2)
        return StreamSpec(f"{cam}/H264", dev, "h264", 1920, 1080, fps) if dev else None

    def mjpeg(cam: str) -> StreamSpec | None:
        dev = resolve_node(CAM_PORTS[cam], 0)
        return StreamSpec(f"{cam}/MJPG", dev, "mjpeg", mjpeg_w, mjpeg_h, fps) if dev else None

    if name == "capture":
        builders = [mjpeg("Primary"), mjpeg("Aux")]
    elif name == "pair":
        builders = [h264("Primary"), mjpeg("Primary"), h264("Aux"), mjpeg("Aux")]
    elif name == "full":
        builders = [
            h264("Arm"), h264("BackGripper"), h264("Primary"), h264("Aux"),
            mjpeg("Primary"), mjpeg("Aux"),
        ]
    else:
        raise SystemExit(f"unknown scenario {name!r}")

    for b in builders:
        if b is None:
            print("WARNING: a camera node could not be resolved; skipping it")
        else:
            specs.append(b)
    return specs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="full", choices=["capture", "pair", "full"])
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mjpeg-size", default="1920x1080", help="WxH for MJPEG streams")
    args = ap.parse_args()

    mjpeg_w, mjpeg_h = (int(x) for x in args.mjpeg_size.lower().split("x"))

    Gst.init(None)
    specs = scenario_specs(args.scenario, mjpeg_w, mjpeg_h, args.fps)
    if not specs:
        print("No streams to run.")
        return 2

    print(f"Scenario={args.scenario} streams={len(specs)} duration={args.seconds}s")
    for s in specs:
        print(f"  - {s.label:16s} {s.fmt:5s} {s.width}x{s.height}@{s.fps}  {s.device}")
    print(f"SoC before: {soc_status()}")

    stats = [StreamStat(spec=s) for s in specs]
    for st in stats:
        st.pipeline = build_pipeline(st.spec, st)

    idle0, total0 = read_cpu_times()
    for st in stats:
        st.pipeline.set_state(Gst.State.PLAYING)

    loop = GLib.MainLoop()
    GLib.timeout_add_seconds(int(max(1, round(args.seconds))), loop.quit)
    t_start = time.monotonic()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - t_start

    for st in stats:
        st.pipeline.set_state(Gst.State.NULL)
    idle1, total1 = read_cpu_times()

    print(f"SoC after:  {soc_status()}")
    dtotal = max(1, total1 - total0)
    cpu_pct = 100.0 * (1.0 - (idle1 - idle0) / dtotal)
    print(f"Aggregate CPU during run: {cpu_pct:.0f}% of {nproc()} cores\n")

    print(f"{'stream':16s} {'fmt':5s} {'frames':>7s} {'fps':>7s} {'MB/s':>7s}  status")
    ok = True
    target = args.fps * 0.9
    for st in stats:
        with st.lock:
            frames, nbytes, err = st.frames, st.total_bytes, st.error
        fps = frames / elapsed if elapsed > 0 else 0.0
        mbps = (nbytes / elapsed) / 1e6 if elapsed > 0 else 0.0
        status = "ERROR: " + err if err else ("ok" if fps >= target else "LOW FPS")
        if err or fps < target:
            ok = False
        print(f"{st.spec.label:16s} {st.spec.fmt:5s} {frames:7d} {fps:7.1f} {mbps:7.1f}  {status}")

    print("\nVERDICT:", "PASS - simultaneous H.264 + MJPEG sustained" if ok else
          "FAIL - at least one stream dropped below target / errored")
    return 0 if ok else 1


def nproc() -> int:
    try:
        import os
        return os.cpu_count() or 1
    except Exception:
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
