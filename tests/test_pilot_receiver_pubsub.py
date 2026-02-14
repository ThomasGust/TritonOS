import json
import socket
import time

import zmq

from control.pilot_receiver import PilotReceiver
from schema.pilot_common import PilotFrame, PilotAxes, PilotButtons


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_pilot_receiver_receives_latest_frame():
    port = _free_port()
    ep = f"tcp://127.0.0.1:{port}"

    rx = PilotReceiver(bind_endpoint=ep, debug=False, poll_ms=20, conflate=True, rcv_hwm=10, expected_schema=1)
    rx.start()

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.connect(ep)

    # Slow-joiner mitigation
    time.sleep(0.15)

    for seq in range(1, 4):
        frame = PilotFrame(
            seq=seq,
            axes=PilotAxes(lx=0.1 * seq, ly=0.0, rx=0.0, ry=0.0, lt=0.0, rt=0.0),
            buttons=PilotButtons(menu=(seq == 2)),
        )
        pub.send_string(json.dumps(frame.to_dict()))
        time.sleep(0.02)

    # Wait a moment for rx thread to process
    t0 = time.time()
    last = None
    while time.time() - t0 < 1.0:
        f, age = rx.get_latest()
        if f and f.seq == 3:
            last = f
            break
        time.sleep(0.02)

    rx.stop()
    pub.close(0)

    assert last is not None
    assert last.seq == 3
    assert abs(last.axes.lx - 0.3) < 1e-6
