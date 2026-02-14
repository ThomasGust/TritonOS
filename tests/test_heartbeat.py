from sensors.heartbeat import HeartbeatSensor

def test_heartbeat_includes_fields():
    hb = HeartbeatSensor(state_fn=lambda: {"armed": True, "pilot_age": 0.1}, rate_hz=1.0)
    msg = hb.read()
    assert msg["sensor"] == "heartbeat"
    assert msg["type"] == "heartbeat"
    assert msg["armed"] is True
    assert abs(msg["pilot_age"] - 0.1) < 1e-6
