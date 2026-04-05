import numpy as np

from sensors.attitude import _direct_mag_yaw_deg, _quaternion_from_rpy_deg


def test_direct_mag_yaw_level_axes():
    assert _direct_mag_yaw_deg(np.array([1.0, 0.0, 0.0]), 0.0, 0.0) == 0.0
    assert _direct_mag_yaw_deg(np.array([0.0, 1.0, 0.0]), 0.0, 0.0) == -90.0


def test_direct_mag_yaw_recovers_heading_with_tilt():
    for roll_deg, pitch_deg, yaw_deg in ((0.0, 0.0, 30.0), (10.0, -15.0, 45.0), (-20.0, 25.0, -60.0)):
        q = _quaternion_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
        mag_body = q.inv_rotate((1.0, 0.0, 0.0))
        got = _direct_mag_yaw_deg(mag_body, roll_deg, pitch_deg)
        assert got == yaw_deg
