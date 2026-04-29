from types import SimpleNamespace

from control.control_service import ControlService


def test_mix_gripper_axes_preserves_pitch_and_limits_yaw():
    pitch, yaw = ControlService._limit_gripper_axes_preserve_pitch(0.25, -1.0)
    left, right = ControlService._mix_gripper_axes(0.25, -1.0)

    assert abs(pitch - 0.25) < 1e-6
    assert abs(yaw + 0.75) < 1e-6
    assert abs(left + 0.5) < 1e-6
    assert abs(right - 1.0) < 1e-6


def test_compute_gripper_diff_holds_park_pose_without_input():
    svc = object.__new__(ControlService)
    svc._gripper_enabled = True
    svc._gripper_pitch_key = "gripper_pitch"
    svc._gripper_yaw_key = "gripper_yaw"
    svc._gripper_pitch_scale = 0.5
    svc._gripper_yaw_scale = 0.5
    svc._gripper_pitch_invert = 1.0
    svc._gripper_yaw_invert = 1.0
    svc._gripper_deadzone = 0.01
    svc._gripper_hold_last = True
    svc._gripper_last_left = 0.0
    svc._gripper_last_right = -1.0

    left, right = ControlService._compute_gripper_diff(svc, SimpleNamespace(aux={}))
    assert abs(left - 0.0) < 1e-6
    assert abs(right + 1.0) < 1e-6


def test_compute_gripper_diff_holds_pitch_when_rotating():
    svc = object.__new__(ControlService)
    svc._gripper_enabled = True
    svc._gripper_pitch_key = "gripper_pitch"
    svc._gripper_yaw_key = "gripper_yaw"
    svc._gripper_pitch_scale = 0.5
    svc._gripper_yaw_scale = 1.0
    svc._gripper_pitch_invert = 1.0
    svc._gripper_yaw_invert = 1.0
    svc._gripper_deadzone = 0.01
    svc._gripper_hold_last = True
    svc._gripper_last_pitch = 0.25
    svc._gripper_last_yaw = 0.0
    svc._gripper_last_left = 0.25
    svc._gripper_last_right = 0.25

    left, right = ControlService._compute_gripper_diff(
        svc,
        SimpleNamespace(aux={"gripper_yaw": 1.0}),
    )

    assert abs(svc._gripper_last_pitch - 0.25) < 1e-6
    assert abs(svc._gripper_last_yaw - 0.75) < 1e-6
    assert abs(left - 1.0) < 1e-6
    assert abs(right + 0.5) < 1e-6
