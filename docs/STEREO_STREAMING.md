# Stereo Streaming Notes

TritonOS streams the two physical cameras. TritonPilot pairs decoded frames and
TritonAnalysis calibrates the saved image pairs.

## Sync Model

The current exploreHD stereo plan is software-synchronized:

- Both cameras stream independently through the normal GStreamer video service.
- For still stereo capture, TritonPilot asks TritonOS to capture both streams on
  the ROV through one paired RPC.
- When configured, TritonOS tries each camera's MJPEG/raw still source first so
  stills are captured before H.264 display compression. If that source is busy,
  it falls back to the running display pipeline's onboard snapshot branch.
- A stereo pair is accepted when the left/right ROV-side monotonic timestamp
  delta is below the configured threshold.

This is not true hardware sync. The exploreHD is a rolling-shutter UVC camera
without an external trigger path in this stack. For calibration and still
measurements, keep the board and ROV steady during capture. For moving targets,
expect stereo error to grow with target speed, vehicle motion, exposure time,
and pair timestamp delta.

## Stream Configuration

Use matching settings for the stereo pair:

- Same resolution.
- Same frame rate.
- Same codec family and bitrate target.
- Same exposure/white-balance behavior when camera controls are available.
- Stable by-path device names so left and right do not swap after USB
  re-enumeration.

H.264 is usually the practical choice on the tether because two 1080p streams
fit comfortably at controlled bitrates. For still snapshots, prefer a configured
MJPEG/raw still source or a pre-H.264 tee so motion artifacts from H.264 are not
baked into calibration images.

## Diagnostics

The video RPC still supports `list_streams`. A new `list_stream_status` command
also returns timing diagnostics:

```json
{
  "Primary Camera": {
    "running": true,
    "started_wall_ts": 1779490000.0,
    "started_monotonic_ts": 12345.67,
    "last_error": null,
    "config": {}
  }
}
```

These timestamps describe when TritonOS put each GStreamer pipeline into
`PLAYING`; they do not timestamp individual sensor exposures. They are useful
for debugging startup order, restarts, and accidental stream mismatches.

## Future Hardware Sync Path

If the team later moves to cameras with global shutter and external frame sync,
do not hide that behind the current software pairing model. Add:

- A trigger source owned by TritonOS.
- Per-frame hardware timestamps or frame IDs.
- A Pilot manifest field that records trigger IDs.
- Analysis validation that rejects unmatched trigger IDs.

Until then, the highest-quality path is rigid mounting, underwater calibration,
high-resolution matching streams, slow/stationary capture, and honest timestamp
metadata.
