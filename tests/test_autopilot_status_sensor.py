from sensors.autopilot_status import AutopilotStatusSensor


def test_autopilot_status_sensor_wraps_control_snapshot():
    class _Control:
        def get_hold_status_snapshot(self):
            return {
                "armed": True,
                "control": {"status": {"reason": "armed_apply"}},
                "autopilot": {"status": {"active": True}},
            }

    sensor = AutopilotStatusSensor(_Control(), rate_hz=10.0)
    msg = sensor.read()

    assert msg["type"] == "autopilot_status"
    assert msg["sensor"] == "autopilot_status"
    assert msg["source"] == "control_service"
    assert msg["armed"] is True
    assert msg["control"]["status"]["reason"] == "armed_apply"
