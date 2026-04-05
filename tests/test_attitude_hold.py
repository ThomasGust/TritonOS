from control.attitude_hold import AttitudeHoldConfig, AttitudeHoldController
from control.depth_hold import DepthHoldConfig, DepthHoldController
from control.mixer import EightThrusterMixer


def test_attitude_hold_corrects_pitch_and_roll_toward_level():
    dt = 0.02
    controller = AttitudeHoldController(
        AttitudeHoldConfig(
            lpf_tau_s=0.0,
            kp=0.05,
            ki=0.0,
            kd=0.0,
            error_deadband_deg=0.0,
            out_limit=1.0,
        )
    )

    controller.step(
        enabled=True,
        manual_pitch=0.0,
        manual_roll=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        sensor_age_s=0.0,
        dt=dt,
    )

    pitch_out, roll_out, status = controller.step(
        enabled=True,
        manual_pitch=0.0,
        manual_roll=0.0,
        pitch_deg=6.0,
        roll_deg=4.0,
        sensor_age_s=0.0,
        dt=dt,
    )

    assert status["active"] is True
    assert pitch_out < 0.0
    assert roll_out < 0.0


def test_attitude_hold_walks_target_with_manual_input():
    dt = 0.02
    controller = AttitudeHoldController(
        AttitudeHoldConfig(
            lpf_tau_s=0.0,
            kp=0.05,
            ki=0.0,
            kd=0.0,
            error_deadband_deg=0.0,
            walk_target=True,
            walk_deadband=0.08,
            walk_rate_dps=20.0,
        )
    )

    controller.step(
        enabled=True,
        manual_pitch=0.0,
        manual_roll=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        sensor_age_s=0.0,
        dt=dt,
    )

    for _ in range(10):
        pitch_out, _, status = controller.step(
            enabled=True,
            manual_pitch=0.5,
            manual_roll=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            sensor_age_s=0.0,
            dt=dt,
        )

    assert status["active"] is True
    assert controller.target_pitch_deg is not None
    assert controller.target_pitch_deg > 0.0
    assert pitch_out > 0.0


def test_depth_and_attitude_hold_can_stabilize_together():
    dt = 0.02

    depth_controller = DepthHoldController(
        DepthHoldConfig(
            depth_lpf_tau_s=0.0,
            kp=0.6,
            ki=0.0,
            kd=0.0,
            error_deadband_m=0.0,
            out_limit=1.0,
        )
    )
    attitude_controller = AttitudeHoldController(
        AttitudeHoldConfig(
            lpf_tau_s=0.0,
            kp=0.05,
            ki=0.0,
            kd=0.0,
            error_deadband_deg=0.0,
            out_limit=1.0,
        )
    )

    depth_controller.step(enabled=True, manual_heave=0.0, depth_m=1.0, depth_age_s=0.0, dt=dt)
    attitude_controller.step(
        enabled=True,
        manual_pitch=0.0,
        manual_roll=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        sensor_age_s=0.0,
        dt=dt,
    )

    heave_out, depth_status = depth_controller.step(
        enabled=True,
        manual_heave=0.0,
        depth_m=1.1,
        depth_age_s=0.0,
        dt=dt,
    )
    pitch_out, roll_out, attitude_status = attitude_controller.step(
        enabled=True,
        manual_pitch=0.0,
        manual_roll=0.0,
        pitch_deg=5.0,
        roll_deg=-3.0,
        sensor_age_s=0.0,
        dt=dt,
    )

    assert depth_status["active"] is True
    assert attitude_status["active"] is True
    assert heave_out > 0.0
    assert pitch_out < 0.0
    assert roll_out > 0.0

    thr = EightThrusterMixer().mix(
        {
            "surge": 0.0,
            "sway": 0.0,
            "heave": heave_out,
            "yaw": 0.0,
            "pitch": pitch_out,
            "roll": roll_out,
        }
    )

    assert len({round(thr[name], 6) for name in ("V_FL", "V_FR", "V_RL", "V_RR")}) > 1
