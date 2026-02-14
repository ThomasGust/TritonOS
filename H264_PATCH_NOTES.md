# H.264 Streaming Patch Notes

This patched build focuses on making H.264 streaming start reliably and stay low-latency over constrained links (e.g. a tether).

## What changed

### ROV-side (TritonOS)
- Added **MJPEG -> H.264 transcode** support:
  - `video_format="mjpeg", encode="h264"`
- Made H.264 encoding more robust:
  - Automatic encoder selection (hardware-first, software fallback)
  - Adds `videoconvert`/`videoscale` and chooses a raw format (NV12/I420) the encoder advertises to prevent *not-negotiated* start failures.
  - Names the encoder element (`name=h264enc`) and sets bitrate/GOP best-effort across common encoder plugins.
- Added leaky queues (default on) so the system **drops frames instead of buffering latency** when bandwidth is limited.
- Safer RTP MTU default (`rtp_mtu=1200`) to reduce fragmentation issues.

### Pilot-side (TritonPilot)
- Streams.json options are now passed through to the ROV:
  - `encode`, `h264_bitrate`, `h264_gop`, `rtp_mtu`, etc.
- Receiver H.264 pipeline includes `h264parse` and a leaky queue to reduce start errors and buffering.

## Recommended stream configs

### Best (camera supports raw at desired mode)
```json
{
  "name": "main",
  "device": "/dev/v4l/by-path/*video-index0",
  "width": 1280,
  "height": 720,
  "fps": 30,
  "video_format": "raw",
  "encode": "h264",
  "h264_bitrate": 3000000,
  "h264_gop": 30,
  "rtp_mtu": 1200,
  "port": 5000
}
```

### If camera only supports MJPEG at desired mode
```json
{
  "name": "main",
  "device": "/dev/v4l/by-path/*video-index0",
  "width": 1280,
  "height": 720,
  "fps": 30,
  "video_format": "mjpeg",
  "encode": "h264",
  "h264_bitrate": 3000000,
  "h264_gop": 30,
  "rtp_mtu": 1200,
  "port": 5000
}
```

## Notes
- For lowest latency, prefer UDP and keep GOP around ~fps (1 second) or smaller if you see loss.
- If you still see start failures, the most common cause is missing GStreamer plugins. Confirm the encoder exists:
  - `gst-inspect-1.0 v4l2h264enc` (or whichever hardware encoder you expect)
  - Software fallback: `gst-inspect-1.0 x264enc`
