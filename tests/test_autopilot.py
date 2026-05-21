import pytest

from control.autopilot import (
    AttitudeAxisConfig,
    AutopilotConfig,
    AutopilotController,
)
from control.depth_hold import DepthHoldConfig


def _config() -> AutopilotConfig:
    return AutopilotConfig(
        depth_enable=True,
        attitude_enable=True,
        attitude_stale_s=0.5,
        depth=DepthHoldConfig(kp=0.5, ki=0.0, kd=0.0, out_limit=0.5, depth_lpf_tau_s=0.0),
        roll=AttitudeAxisConfig(kp=0.01, kd=0.0, out_limit=0.2),
        pitch=AttitudeAxisConfig(kp=0.01, kd=0.0, out_limit=0.2),
        yaw=AttitudeAxisConfig(kp=0.01, kd=0.0, out_limit=0.2),
    )


def _cmd(**overrides):
    base = {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    base.update(overrides)
    return base


def _attitude(**overrides):
    base = {
        "type": "attitude",
        "roll_pitch_ready": True,
        "yaw_ready": True,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": 0.0,
        "source": "onboard_imu_mag_relative",
    }
    base.update(overrides)
    return base


def test_autopilot_combines_depth_and_roll_pitch_without_disabling_depth():
    autopilot = AutopilotController(_config())

    cmd, status0 = autopilot.step(
        modes={"autopilot": {"depth": True, "roll_pitch_level": True}},
        cmd=_cmd(),
        depth_m=1.00,
        depth_age_s=0.0,
        attitude=_attitude(roll_deg=0.0, pitch_deg=0.0),
        attitude_age_s=0.0,
        dt=0.02,
    )
    assert status0["depth_hold"]["active"] is True
    assert status0["attitude"]["axes"]["roll"]["active"] is True

    cmd, status = autopilot.step(
        modes={"autopilot": {"depth": True, "roll_pitch_level": True}},
        cmd=_cmd(),
        depth_m=1.20,
        depth_age_s=0.0,
        attitude=_attitude(roll_deg=10.0, pitch_deg=-5.0),
        attitude_age_s=0.0,
        dt=0.02,
    )

    assert status["depth_hold"]["active"] is True
    assert status["attitude"]["axes"]["roll"]["mode"] == "level"
    assert cmd["heave"] == pytest.approx(0.1)
    assert cmd["roll"] == pytest.approx(-0.1)
    assert cmd["pitch"] == pytest.approx(0.05)
    assert cmd["yaw"] == pytest.approx(0.0)


def test_autopilot_keeps_yaw_free_for_roll_pitch_level():
    autopilot = AutopilotController(_config())

    cmd, status = autopilot.step(
        modes={"roll_pitch_level": True},
        cmd=_cmd(yaw=0.25),
        depth_m=None,
        depth_age_s=None,
        attitude=_attitude(yaw_deg=40.0),
        attitude_age_s=0.0,
        dt=0.02,
    )

    assert status["attitude"]["axes"]["yaw"]["enabled_cmd"] is False
    assert cmd["yaw"] == pytest.approx(0.25)


def test_autopilot_holds_yaw_without_forcing_roll_pitch():
    autopilot = AutopilotController(_config())

    cmd, status0 = autopilot.step(
        modes={"autopilot": {"yaw": "hold"}},
        cmd=_cmd(),
        depth_m=None,
        depth_age_s=None,
        attitude=_attitude(yaw_deg=10.0),
        attitude_age_s=0.0,
        dt=0.02,
    )
    assert status0["attitude"]["axes"]["yaw"]["target_deg"] == pytest.approx(10.0)
    assert cmd["yaw"] == pytest.approx(0.0)

    cmd, status = autopilot.step(
        modes={"autopilot": {"yaw": "hold"}},
        cmd=_cmd(),
        depth_m=None,
        depth_age_s=None,
        attitude=_attitude(yaw_deg=25.0),
        attitude_age_s=0.0,
        dt=0.02,
    )

    assert status["attitude"]["axes"]["yaw"]["mode"] == "hold"
    assert status["attitude"]["axes"]["roll"]["enabled_cmd"] is False
    assert cmd["yaw"] == pytest.approx(-0.15)
    assert cmd["roll"] == pytest.approx(0.0)
    assert cmd["pitch"] == pytest.approx(0.0)


def test_autopilot_accepts_independent_manual_targets_for_all_axes():
    autopilot = AutopilotController(_config())

    cmd, status = autopilot.step(
        modes={
            "autopilot": {
                "depth": True,
                "roll": "hold",
                "pitch": "hold",
                "yaw": "hold",
                "targets": {
                    "depth_m": 1.5,
                    "roll_deg": 5.0,
                    "pitch_deg": -3.0,
                    "yaw_deg": 90.0,
                },
            }
        },
        cmd=_cmd(),
        depth_m=1.0,
        depth_age_s=0.0,
        attitude=_attitude(roll_deg=10.0, pitch_deg=0.0, yaw_deg=100.0),
        attitude_age_s=0.0,
        dt=0.02,
    )

    assert status["depth_hold"]["target_m"] == pytest.approx(1.5)
    assert status["depth_hold"]["target_source"] == "command"
    assert status["attitude"]["axes"]["roll"]["target_deg"] == pytest.approx(5.0)
    assert status["attitude"]["axes"]["pitch"]["target_deg"] == pytest.approx(-3.0)
    assert status["attitude"]["axes"]["yaw"]["target_deg"] == pytest.approx(90.0)
    assert cmd["heave"] == pytest.approx(-0.25)
    assert cmd["roll"] == pytest.approx(-0.05)
    assert cmd["pitch"] == pytest.approx(-0.03)
    assert cmd["yaw"] == pytest.approx(-0.10)


def test_explicit_attitude_target_fails_open_to_manual_input():
    autopilot = AutopilotController(_config())

    cmd, status = autopilot.step(
        modes={"autopilot": {"yaw": "hold", "targets": {"yaw_deg": 90.0}}},
        cmd=_cmd(yaw=0.25),
        depth_m=None,
        depth_age_s=None,
        attitude=_attitude(yaw_deg=100.0),
        attitude_age_s=0.0,
        dt=0.02,
    )

    assert status["attitude"]["axes"]["yaw"]["reason"] == "manual_override"
    assert cmd["yaw"] == pytest.approx(0.25)


def test_autopilot_fails_open_to_manual_when_attitude_stale():
    autopilot = AutopilotController(_config())

    cmd, status = autopilot.step(
        modes={"roll_pitch_level": True},
        cmd=_cmd(roll=0.12, pitch=-0.08),
        depth_m=None,
        depth_age_s=None,
        attitude=_attitude(roll_deg=20.0, pitch_deg=20.0),
        attitude_age_s=2.0,
        dt=0.02,
    )

    assert status["attitude"]["axes"]["roll"]["reason"] == "stale_attitude"
    assert cmd["roll"] == pytest.approx(0.12)
    assert cmd["pitch"] == pytest.approx(-0.08)
