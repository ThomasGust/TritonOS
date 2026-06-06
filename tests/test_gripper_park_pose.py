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


def test_compute_gripper_diff_scales_live_arm_gain():
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
    svc._gripper_last_pitch = 0.0
    svc._gripper_last_yaw = 0.0
    svc._gripper_last_left = 0.0
    svc._gripper_last_right = 0.0

    left, right = ControlService._compute_gripper_diff(
        svc,
        SimpleNamespace(aux={"gripper_pitch": 1.0, "gripper_yaw": 0.0}, modes={"arm_gain": 0.4}),
    )

    assert svc._last_arm_gain == 0.4
    assert abs(left - 0.2) < 1e-6
    assert abs(right - 0.2) < 1e-6


def test_compute_wrist_rotate_scales_live_back_gripper_gain():
    svc = object.__new__(ControlService)
    svc._wrist_rotate_enabled = True
    svc._wrist_rotate_right_axis = "rt"
    svc._wrist_rotate_left_axis = "lt"
    svc._wrist_rotate_trigger_deadzone = 0.10
    svc._wrist_rotate_speed = 0.50

    cmd = ControlService._compute_wrist_rotate(
        svc,
        SimpleNamespace(axes=SimpleNamespace(rt=1.0, lt=0.0), modes={"back_gripper_gain": 0.4}),
    )

    assert svc._last_back_gripper_gain == 0.4
    assert abs(cmd - 0.2) < 1e-6


def test_send_gripper_park_pose_writes_folded_servo_targets():
    class FakeSink:
        def __init__(self):
            self.writes = []

        def write(self, payload):
            self.writes.append(dict(payload))

    svc = object.__new__(ControlService)
    svc._gripper_enabled = True
    svc._gripper_park_on_arm_disarm = True
    svc._gripper_park_pitch, svc._gripper_park_yaw = ControlService._limit_gripper_axes_preserve_pitch(0.20, -1.0)
    svc._gripper_park_left, svc._gripper_park_right = ControlService._mix_gripper_axes(0.20, -1.0)
    svc._gripper_left_key = "gripper_left"
    svc._gripper_right_key = "gripper_right"
    svc._gripper_last_pitch = 0.0
    svc._gripper_last_yaw = 0.0
    svc._gripper_last_left = 0.0
    svc._gripper_last_right = 0.0
    svc.dry_run = False
    svc._hw_sink = FakeSink()
    svc._sink_armed = True
    svc._warned_dry_run = False
    svc._warned_no_sink = False
    svc._warned_sink_disarmed = False

    ControlService._send_gripper_park_pose(svc, settle_s=0.0)

    assert len(svc._hw_sink.writes) == 1
    assert abs(svc._hw_sink.writes[0]["gripper_left"] + 0.6) < 1e-6
    assert abs(svc._hw_sink.writes[0]["gripper_right"] - 1.0) < 1e-6
