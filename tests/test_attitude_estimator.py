import math

import pytest

from sensors.attitude_estimator import AttitudeEstimatorProcessor, RollPitchConfig, RollPitchEstimator


def _imu(ts, accel, gyro=(0.0, 0.0, 0.0)):
    return {
        "ts": ts,
        "sensor": "imu",
        "type": "imu",
        "accel": {"x": accel[0], "y": accel[1], "z": accel[2]},
        "gyro": {"x": gyro[0], "y": gyro[1], "z": gyro[2]},
    }


def _mag(ts, mag, source="ak09915"):
    return {
        "ts": ts,
        "sensor": "mag",
        "type": "mag",
        "mag": {"x": mag[0], "y": mag[1], "z": mag[2]},
        "mag_source": source,
        "mag_sources": {
            source: {"x": mag[0], "y": mag[1], "z": mag[2]},
        },
    }


def _mag_sources(ts, sources, primary="ak09915"):
    primary_mag = sources[primary]
    return {
        "ts": ts,
        "sensor": "mag",
        "type": "mag",
        "mag": {"x": primary_mag[0], "y": primary_mag[1], "z": primary_mag[2]},
        "mag_source": primary,
        "mag_sources": {
            source: {"x": mag[0], "y": mag[1], "z": mag[2]}
            for source, mag in sources.items()
        },
    }


def _config(**kwargs):
    kwargs.setdefault("vehicle_roll_axis", "x")
    return RollPitchConfig(**kwargs)


def test_roll_pitch_estimator_default_uses_standard_vehicle_axes():
    est = RollPitchEstimator(RollPitchConfig(calibration_samples=5, accel_correction=1.0))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    theta = math.radians(12.0)
    sensor_x_tilt = (math.sin(theta) * 9.80665, 0.0, math.cos(theta) * 9.80665)
    out = est.update(_imu(5.05, sensor_x_tilt))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(0.0, abs=0.15)
    assert out["pitch_deg"] == pytest.approx(12.0, abs=0.15)

    sensor_y_tilt = (0.0, math.sin(theta) * 9.80665, math.cos(theta) * 9.80665)
    out = est.update(_imu(5.10, sensor_y_tilt))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(-12.0, abs=0.15)
    assert out["pitch_deg"] == pytest.approx(0.0, abs=0.15)


def test_roll_pitch_estimator_can_swap_vehicle_roll_axis_to_sensor_y():
    est = RollPitchEstimator(
        RollPitchConfig(
            calibration_samples=5,
            accel_correction=1.0,
            vehicle_roll_axis="y",
        )
    )
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    theta = math.radians(12.0)
    sensor_x_tilt = (math.sin(theta) * 9.80665, 0.0, math.cos(theta) * 9.80665)
    out = est.update(_imu(5.05, sensor_x_tilt))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(12.0, abs=0.15)
    assert out["pitch_deg"] == pytest.approx(0.0, abs=0.15)

    sensor_y_tilt = (0.0, math.sin(theta) * 9.80665, math.cos(theta) * 9.80665)
    out = est.update(_imu(5.10, sensor_y_tilt))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(0.0, abs=0.15)
    assert out["pitch_deg"] == pytest.approx(12.0, abs=0.15)


def test_roll_pitch_estimator_zeros_current_rest_pose():
    est = RollPitchEstimator(_config(calibration_samples=8, accel_correction=1.0))
    rest = (3.379, -9.315, -0.232)
    bias = (-0.0178, -0.0102, 0.0077)

    out = None
    for i in range(12):
        out = est.update(_imu(10.0 + i * 0.05, rest, bias))

    assert out is not None
    assert out["source"] == "onboard_imu_6axis"
    assert out["roll_pitch_ready"] is True
    assert out["attitude_ready"] is True
    assert abs(out["roll_deg"]) < 0.01
    assert abs(out["pitch_deg"]) < 0.01
    assert out["calibration_state"] == "calibrated"
    assert math.isclose(out["gyro_bias"]["x"], bias[0])


def test_roll_pitch_estimator_reports_known_tilt_after_reference():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    theta = math.radians(10.0)
    roll_accel = (0.0, -math.sin(theta) * 9.80665, math.cos(theta) * 9.80665)
    out = est.update(_imu(5.05, roll_accel))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(10.0, abs=0.15)
    assert out["pitch_deg"] == pytest.approx(0.0, abs=0.15)


def test_roll_pitch_estimator_reports_exact_rest_frame_pitch():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    theta = math.radians(12.0)
    pitch_accel = (math.sin(theta) * 9.80665, 0.0, math.cos(theta) * 9.80665)
    out = est.update(_imu(5.05, pitch_accel))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(0.0, abs=0.15)
    assert out["pitch_deg"] == pytest.approx(12.0, abs=0.15)
    assert out["accel_weight"] == pytest.approx(1.0)


def test_roll_pitch_estimator_reports_combined_roll_pitch_without_axis_bleed():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    roll = math.radians(-15.0)
    pitch = math.radians(20.0)
    accel = (
        math.sin(pitch) * 9.80665,
        -math.sin(roll) * math.cos(pitch) * 9.80665,
        math.cos(roll) * math.cos(pitch) * 9.80665,
    )
    out = est.update(_imu(5.05, accel))

    assert out is not None
    assert out["roll_deg"] == pytest.approx(-15.0, abs=0.2)
    assert out["pitch_deg"] == pytest.approx(20.0, abs=0.2)


def test_roll_pitch_estimator_waits_for_stable_calibration():
    est = RollPitchEstimator(
        _config(
            calibration_samples=5,
            calibration_max_tilt_std_deg=0.5,
            calibration_max_gyro_rms_dps=3.0,
        )
    )
    moving = [
        (0.0, 0.0, 9.80665),
        (1.0, 0.0, 9.75552),
        (-1.0, 0.0, 9.75552),
        (0.0, 1.0, 9.75552),
        (0.0, -1.0, 9.75552),
    ]
    for i, accel in enumerate(moving):
        assert est.update(_imu(float(i), accel)) is None

    assert est.status()["calibration_state"] == "calibrating"

    out = None
    for i in range(12):
        out = est.update(_imu(10.0 + i * 0.05, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert out["calibration_state"] == "calibrated"


def test_roll_pitch_estimator_refines_gyro_bias_while_stationary():
    est = RollPitchEstimator(
        _config(
            calibration_samples=5,
            accel_correction=1.0,
            stationary_bias_tau_s=0.1,
            stationary_gyro_max_dps=2.0,
        )
    )
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    drift = math.radians(0.5)
    out = None
    for i in range(40):
        out = est.update(_imu(5.0 + i * 0.05, (0.0, 0.0, 9.80665), (0.0, 0.0, drift)))

    assert out is not None
    assert out["gyro_bias_alpha"] > 0.0
    assert out["gyro_bias"]["z"] == pytest.approx(drift, rel=0.25)


def test_roll_pitch_estimator_reports_relative_mag_yaw():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    est.update_mag(_mag(0.0, (1.0, 0.0, 0.0)))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    yaw = math.radians(30.0)
    # Positive vehicle yaw rotates the world magnetic vector negative in body coordinates.
    est.update_mag(_mag(5.05, (math.cos(yaw), -math.sin(yaw), 0.0)))
    out = est.update(_imu(5.05, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert out["yaw_status"] == "ready"
    assert out["yaw_source"] == "ak09915"
    assert out["yaw_deg"] == pytest.approx(30.0, abs=0.2)
    assert out["yaw_mag_deg"] == pytest.approx(30.0, abs=0.2)


def test_roll_pitch_estimator_gyro_propagates_yaw_between_mag_samples():
    est = RollPitchEstimator(
        _config(
            calibration_samples=5,
            accel_correction=1.0,
            max_dt_s=2.0,
        )
    )
    est.update_mag(_mag(0.0, (1.0, 0.0, 0.0)))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    est.update_mag(_mag(5.0, (1.0, 0.0, 0.0)))
    out0 = est.update(_imu(5.0, (0.0, 0.0, 9.80665)))
    assert out0 is not None
    assert out0["yaw_deg"] == pytest.approx(0.0, abs=0.2)

    rate = math.radians(10.0)
    out1 = est.update(_imu(6.0, (0.0, 0.0, 9.80665), (0.0, 0.0, rate)))
    assert out1 is not None
    assert out1["yaw_deg"] == pytest.approx(10.0, abs=0.2)


def test_roll_pitch_estimator_tilt_compensates_mag_yaw():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    est.update_mag(_mag(0.0, (1.0, 0.0, 0.0)))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    yaw = math.radians(25.0)
    pitch = math.radians(18.0)
    # World magnetic north and gravity, expressed in a body frame with
    # positive yaw and positive pitch.
    mag_body = (math.cos(yaw) * math.cos(pitch), -math.sin(yaw), -math.cos(yaw) * math.sin(pitch))
    accel_body = (math.sin(pitch) * 9.80665, 0.0, math.cos(pitch) * 9.80665)
    est.update_mag(_mag(5.05, mag_body))
    out = est.update(_imu(5.05, accel_body))

    assert out is not None
    assert out["pitch_deg"] == pytest.approx(18.0, abs=0.3)
    assert out["yaw_mag_deg"] == pytest.approx(25.0, abs=0.4)


def test_roll_pitch_estimator_auto_prefers_mmc_yaw_when_available():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    est.update_mag(_mag_sources(0.0, {"ak09915": (1.0, 0.0, 0.0), "mmc5983": (1.0, 0.0, 0.0)}))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    ak_yaw = math.radians(10.0)
    mmc_yaw = math.radians(40.0)
    est.update_mag(
        _mag_sources(
            5.05,
            {
                "ak09915": (math.cos(ak_yaw), -math.sin(ak_yaw), 0.0),
                "mmc5983": (math.cos(mmc_yaw), -math.sin(mmc_yaw), 0.0),
            },
        )
    )
    out = est.update(_imu(5.05, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert out["yaw_source"] == "mmc5983"
    assert out["yaw_mag_deg"] == pytest.approx(40.0, abs=0.2)


def test_roll_pitch_estimator_auto_uses_fresh_yaw_source():
    est = RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    est.update_mag(_mag_sources(0.0, {"ak09915": (1.0, 0.0, 0.0), "mmc5983": (1.0, 0.0, 0.0)}))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    yaw = math.radians(15.0)
    est.update_mag(_mag_sources(5.05, {"ak09915": (math.cos(yaw), -math.sin(yaw), 0.0)}))
    out = est.update(_imu(5.05, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert out["yaw_source"] == "ak09915"
    assert out["yaw_status"] == "ready"
    assert out["yaw_mag_deg"] == pytest.approx(15.0, abs=0.2)


def test_roll_pitch_estimator_auto_uses_clean_yaw_source():
    est = RollPitchEstimator(
        _config(
            calibration_samples=5,
            accel_correction=1.0,
            yaw_mag_norm_gate=0.2,
        )
    )
    est.update_mag(_mag_sources(0.0, {"ak09915": (1.0, 0.0, 0.0), "mmc5983": (1.0, 0.0, 0.0)}))
    for i in range(5):
        assert est.update(_imu(float(i), (0.0, 0.0, 9.80665))) is None

    ak_yaw = math.radians(12.0)
    mmc_yaw = math.radians(45.0)
    est.update_mag(
        _mag_sources(
            5.05,
            {
                "ak09915": (math.cos(ak_yaw), -math.sin(ak_yaw), 0.0),
                "mmc5983": (2.0 * math.cos(mmc_yaw), -2.0 * math.sin(mmc_yaw), 0.0),
            },
        )
    )
    out = est.update(_imu(5.05, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert out["yaw_source"] == "ak09915"
    assert out["yaw_status"] == "ready"
    assert out["yaw_mag_deg"] == pytest.approx(12.0, abs=0.2)


def test_roll_pitch_estimator_averages_yaw_reference_samples():
    est = RollPitchEstimator(
        _config(
            calibration_samples=5,
            accel_correction=1.0,
            yaw_mag_smooth_tau_s=0.0,
            yaw_reference_samples=5,
        )
    )
    reference_noise_deg = [-1.5, 1.5, -1.5, 1.5, 1.5]
    for i, yaw_deg in enumerate(reference_noise_deg):
        yaw = math.radians(yaw_deg)
        ts = i * 0.05
        est.update_mag(_mag(ts, (math.cos(yaw), -math.sin(yaw), 0.0)))
        assert est.update(_imu(ts, (0.0, 0.0, 9.80665))) is None

    est.update_mag(_mag(0.30, (1.0, 0.0, 0.0)))
    out = est.update(_imu(0.30, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert abs(out["yaw_mag_deg"]) < 0.5
    assert out["yaw_reference_mag_samples"] == 5


def test_roll_pitch_estimator_smooths_short_mag_spikes():
    est = RollPitchEstimator(
        _config(
            calibration_samples=5,
            accel_correction=1.0,
            yaw_mag_smooth_tau_s=1.0,
            yaw_reference_samples=5,
        )
    )
    for i in range(5):
        ts = i * 0.05
        est.update_mag(_mag(ts, (1.0, 0.0, 0.0)))
        assert est.update(_imu(ts, (0.0, 0.0, 9.80665))) is None

    est.update_mag(_mag(0.25, (1.0, 0.0, 0.0)))
    out0 = est.update(_imu(0.25, (0.0, 0.0, 9.80665)))
    assert out0 is not None
    assert out0["yaw_deg"] == pytest.approx(0.0, abs=0.2)

    spike_yaw = math.radians(30.0)
    est.update_mag(_mag(0.30, (math.cos(spike_yaw), -math.sin(spike_yaw), 0.0)))
    out = est.update(_imu(0.30, (0.0, 0.0, 9.80665)))

    assert out is not None
    assert abs(out["yaw_mag_deg"]) < 3.0
    assert abs(out["yaw_deg"]) < 0.2


def test_attitude_estimator_processor_derives_onboard_attitude():
    processor = AttitudeEstimatorProcessor(
        RollPitchEstimator(_config(calibration_samples=5, accel_correction=1.0))
    )
    assert processor.process(_mag(0.0, (1.0, 0.0, 0.0))) == []

    out = []
    for i in range(5):
        out = processor.process(_imu(float(i), (0.0, 0.0, 9.80665)))
    assert out == []

    assert processor.process(_mag(5.0, (1.0, 0.0, 0.0))) == []
    out = processor.process(_imu(5.0, (0.0, 0.0, 9.80665)))

    assert len(out) == 1
    msg = out[0]
    assert msg["type"] == "attitude"
    assert msg["source"] == "onboard_imu_mag_relative"
    assert msg["roll_pitch_ready"] is True
    assert msg["yaw_ready"] is True
    assert msg["sample_age_s"] >= 0.0
