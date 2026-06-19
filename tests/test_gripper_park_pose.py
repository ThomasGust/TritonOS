from types import SimpleNamespace

from control.control_service import ControlService


# Geometry used across the gripper tests (matches rov_config defaults).
GEO = dict(
    servo_range_deg=70.0,
    pitch_span_deg=90.0,
    wrist_span_deg=90.0,
    pitch_neutral_deg=45.0,
    wrist_neutral_deg=45.0,
)

# Full pitch (or full wrist) alone uses 45 deg of the 70 deg budget -> 45/70.
FULL_AXIS = 45.0 / 70.0


def _make_gripper_svc(**overrides):
    svc = object.__new__(ControlService)
    svc._gripper_enabled = True
    svc._gripper_pitch_key = "gripper_pitch"
    svc._gripper_yaw_key = "gripper_yaw"
    svc._gripper_left_key = "gripper_left"
    svc._gripper_right_key = "gripper_right"
    svc._gripper_pitch_invert = 1.0
    svc._gripper_yaw_invert = 1.0
    svc._gripper_left_invert = 1.0
    svc._gripper_right_invert = 1.0
    svc._gripper_deadzone = 0.01
    svc._gripper_servo_range_deg = GEO["servo_range_deg"]
    svc._gripper_pitch_span_deg = GEO["pitch_span_deg"]
    svc._gripper_wrist_span_deg = GEO["wrist_span_deg"]
    svc._gripper_pitch_neutral_deg = GEO["pitch_neutral_deg"]
    svc._gripper_wrist_neutral_deg = GEO["wrist_neutral_deg"]
    svc._gripper_hold_last = True
    svc._gripper_last_pitch = 0.0
    svc._gripper_last_yaw = 0.0
    svc._gripper_last_left = 0.0
    svc._gripper_last_right = 0.0
    svc._last_arm_gain = 1.0
    for k, v in overrides.items():
        setattr(svc, k, v)
    return svc


# --- differential kinematics ------------------------------------------------

def test_diff_mix_neutral_inputs_are_centered():
    left, right = ControlService._diff_mix_norm_deg(0.0, 0.0, **GEO)
    assert abs(left) < 1e-9
    assert abs(right) < 1e-9


def test_diff_mix_pure_pitch_drives_both_servos_together():
    left, right = ControlService._diff_mix_norm_deg(1.0, 0.0, **GEO)
    assert abs(left - FULL_AXIS) < 1e-6
    assert abs(right - FULL_AXIS) < 1e-6  # pitch reaches 90 deg without saturating


def test_diff_mix_pure_wrist_drives_servos_opposite():
    left, right = ControlService._diff_mix_norm_deg(0.0, 1.0, **GEO)
    assert abs(left - FULL_AXIS) < 1e-6
    assert abs(right + FULL_AXIS) < 1e-6


def test_diff_mix_keeps_wrist_at_full_pitch_with_pitch_priority():
    # Full pitch + full wrist exceeds the +/-70 budget: pitch is preserved and
    # wrist is tapered to the remaining 25 deg (left saturates, right does not).
    left, right = ControlService._diff_mix_norm_deg(1.0, 1.0, **GEO)
    assert abs(left - 1.0) < 1e-6           # 45 + 25 deg = 70 deg
    assert abs(right - (20.0 / 70.0)) < 1e-6  # 45 - 25 deg = 20 deg
    # Wrist authority is non-zero at full pitch (the old limiter zeroed it).
    assert abs(left - right) > 1e-3


def test_diff_mix_right_invert_unswaps_pitch_and_roll():
    # Pure pitch with no invert -> both servos move the SAME way (mixer default).
    left, right = ControlService._diff_mix_norm_deg(1.0, 0.0, **GEO)
    assert abs(left - right) < 1e-9
    # On a facing-servo bevel differential that mapping rolls the output, so we
    # invert one servo. Pure pitch then drives the servos OPPOSITE, same magnitude.
    li, ri = ControlService._diff_mix_norm_deg(1.0, 0.0, right_invert=-1.0, **GEO)
    assert abs(li - left) < 1e-9
    assert abs(ri + right) < 1e-9
    assert abs(li + ri) < 1e-9


def test_diff_mix_recovers_pitch_and_wrist_from_outputs_midband():
    # In the unsaturated band, (left+right)/2 and (left-right)/2 recover the
    # pitch/wrist deviations scaled by range.
    left, right = ControlService._diff_mix_norm_deg(0.5, 0.25, **GEO)
    d_pitch = (left + right) * 0.5 * GEO["servo_range_deg"]
    d_wrist = (left - right) * 0.5 * GEO["servo_range_deg"]
    # pitch_norm 0.5 -> 67.5 deg -> +22.5 from neutral; wrist 0.25 -> 56.25 -> +11.25
    assert abs(d_pitch - 22.5) < 1e-6
    assert abs(d_wrist - 11.25) < 1e-6


# --- live command path ------------------------------------------------------

def test_compute_gripper_diff_applies_absolute_position():
    svc = _make_gripper_svc()
    left, right = ControlService._compute_gripper_diff(
        svc, SimpleNamespace(aux={"gripper_pitch": 1.0, "gripper_yaw": 0.0}, modes={})
    )
    exp_l, exp_r = ControlService._diff_mix_norm_deg(1.0, 0.0, **GEO)
    assert abs(left - exp_l) < 1e-6
    assert abs(right - exp_r) < 1e-6


def test_compute_gripper_diff_arm_gain_does_not_cap_range():
    # arm_gain is now a pilot-side speed knob; it must NOT scale the ROV output.
    svc = _make_gripper_svc()
    left, right = ControlService._compute_gripper_diff(
        svc,
        SimpleNamespace(aux={"gripper_pitch": 1.0, "gripper_yaw": 0.0}, modes={"arm_gain": 0.4}),
    )
    assert svc._last_arm_gain == 0.4            # still tracked for telemetry
    assert abs(left - FULL_AXIS) < 1e-6         # full pitch despite low arm_gain
    assert abs(right - FULL_AXIS) < 1e-6


def test_compute_gripper_diff_holds_last_when_arm_keys_absent():
    # No arm keys on the wire -> hold the last commanded servo pose.
    svc = _make_gripper_svc(_gripper_last_left=0.3, _gripper_last_right=-0.1)
    left, right = ControlService._compute_gripper_diff(
        svc, SimpleNamespace(aux={}, modes={})
    )
    assert abs(left - 0.3) < 1e-9
    assert abs(right + 0.1) < 1e-9


def test_compute_gripper_diff_applies_commanded_center():
    # A present 0.0 command is the centered pose, not "hold last".
    svc = _make_gripper_svc(_gripper_last_left=0.3, _gripper_last_right=-0.1)
    left, right = ControlService._compute_gripper_diff(
        svc, SimpleNamespace(aux={"gripper_pitch": 0.0, "gripper_yaw": 0.0}, modes={})
    )
    assert abs(left) < 1e-9
    assert abs(right) < 1e-9


def test_send_gripper_park_pose_writes_folded_servo_targets():
    class FakeSink:
        def __init__(self):
            self.writes = []

        def write(self, payload):
            self.writes.append(dict(payload))

    park_left, park_right = ControlService._diff_mix_norm_deg(-1.0, 0.0, **GEO)

    svc = _make_gripper_svc(
        _gripper_park_on_arm_disarm=True,
        _gripper_park_pitch=-1.0,
        _gripper_park_yaw=0.0,
        _gripper_park_left=park_left,
        _gripper_park_right=park_right,
        dry_run=False,
        _sink_armed=True,
        _warned_dry_run=False,
        _warned_no_sink=False,
        _warned_sink_disarmed=False,
    )
    svc._hw_sink = FakeSink()

    ControlService._send_gripper_park_pose(svc, settle_s=0.0)

    assert len(svc._hw_sink.writes) == 1
    # pitch -1 == 0 deg (folded flat), wrist centered -> both servos at -45/70.
    assert abs(svc._hw_sink.writes[0]["gripper_left"] + FULL_AXIS) < 1e-6
    assert abs(svc._hw_sink.writes[0]["gripper_right"] + FULL_AXIS) < 1e-6


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
