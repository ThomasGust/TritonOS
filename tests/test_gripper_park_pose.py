from types import SimpleNamespace

from control.control_service import ControlService


def test_mix_gripper_axes_soft_tucked_pose():
    left, right = ControlService._mix_gripper_axes(-0.95, 0.95)
    assert abs(left - 0.0) < 1e-6
    assert abs(right + 1.0) < 1e-6


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
