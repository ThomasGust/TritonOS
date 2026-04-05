import numpy as np

from sensors.attitude import _direct_mag_yaw_deg, _mag_debug_summary, _quaternion_from_rpy_deg
from triton_ahrs.calibration import Mount


def test_direct_mag_yaw_level_axes():
    assert _direct_mag_yaw_deg(np.array([1.0, 0.0, 0.0]), 0.0, 0.0) == 0.0
    assert _direct_mag_yaw_deg(np.array([0.0, 1.0, 0.0]), 0.0, 0.0) == -90.0


def test_direct_mag_yaw_recovers_heading_with_tilt():
    for roll_deg, pitch_deg, yaw_deg in ((0.0, 0.0, 30.0), (10.0, -15.0, 45.0), (-20.0, 25.0, -60.0)):
        q = _quaternion_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
        mag_body = q.inv_rotate((1.0, 0.0, 0.0))
        got = _direct_mag_yaw_deg(mag_body, roll_deg, pitch_deg)
        assert got == yaw_deg


def test_mag_debug_summary_reports_heading_delta_between_sensors():
    debug = _mag_debug_summary(
        {
            "ak09915": {"x": 1.0, "y": 0.0, "z": 0.0},
            "mmc5983": {"x": 0.0, "y": 1.0, "z": 0.0},
        },
        roll_deg=0.0,
        pitch_deg=0.0,
        mount=Mount.identity(),
        mag_cal=None,
        selected_source="mmc5983",
    )

    assert debug["selected_source"] == "mmc5983"
    assert debug["ak09915"]["heading_deg"] == 0.0
    assert debug["mmc5983"]["heading_deg"] == -90.0
    assert debug["heading_delta_deg"] == 90.0
    assert debug["body_angle_deg"] == 90.0
