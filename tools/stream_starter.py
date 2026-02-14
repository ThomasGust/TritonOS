from video.gst_streamer import StreamConfig, StreamManager
import time

if __name__ == "__main__":
    # Pi is 192.168.1.2, Windows is 192.168.1.1
    RX_IP = "192.168.1.1"

    mgr = StreamManager()
    cfg = StreamConfig(
        name="cam0",
        device="/dev/v4l/by-path/*video-index0",
        width=1280,
        height=720,
        fps=30,
        video_format="mjpeg",   # Pi webcam gives MJPEG
        transport="udp",
        host=RX_IP,
        port=5000,
        rtp_pt_jpeg=26,         # match Windows
    )
    mgr.start_stream(cfg)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        mgr.stop_all()