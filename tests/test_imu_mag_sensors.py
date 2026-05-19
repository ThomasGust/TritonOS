from sensors.navigator import IMUSensor, MagSensor, Vec3


class StubBoard:
    def __init__(self):
        self.read_imu_calls = 0
        self.read_mags_calls = 0

    def read_imu(self):
        self.read_imu_calls += 1
        return Vec3(1.0, 2.0, 3.0), Vec3(0.1, 0.2, 0.3)

    def read_mags(self):
        self.read_mags_calls += 1
        return {
            "ak09915": {"x": 50.0, "y": 10.0, "z": -5.0, "ts": 1.0},
            "mmc5983": {"x": 35.0, "y": -44.0, "z": -9.0, "ts": 1.0},
        }


def test_imu_sensor_reads_accel_gyro_without_mag_by_default():
    board = StubBoard()
    msg = IMUSensor(board, rate_hz=20.0).read()

    assert msg["type"] == "imu"
    assert msg["accel"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert msg["gyro"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert "mag_sources" not in msg
    assert board.read_imu_calls == 1
    assert board.read_mags_calls == 0


def test_mag_sensor_publishes_both_raw_magnetometers():
    board = StubBoard()
    msg = MagSensor(board, rate_hz=5.0).read()

    assert msg["sensor"] == "mag"
    assert msg["type"] == "mag"
    assert msg["mag_source"] == "ak09915"
    assert msg["mag"] == {"x": 50.0, "y": 10.0, "z": -5.0}
    assert msg["mag_sources"]["mmc5983"]["y"] == -44.0
    assert board.read_mags_calls == 1
