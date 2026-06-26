from types import SimpleNamespace

import pytest

from control.control_service import ControlService


# Symmetric geometry used across the gripper tests for the core differential math.
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
    svc._gripper_pitch_min = -1.0
    svc._gripper_pitch_max = 1.0
    svc._gripper_yaw_min = -1.0
    svc._gripper_yaw_max = 1.0
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


def test_current_repo_geometry_reaches_full_pitch_wrist_square():
    import rov_config as cfg

    # The +/-100 servos make the whole (pitch, wrist) square reachable, so the
    # config uses a symmetric 45 deg neutral with no pitch/wrist trade-off.
    assert cfg.GRIPPER_SERVO_RANGE_DEG == 100.0
    assert cfg.GRIPPER_PITCH_SPAN_DEG == 90.0
    assert cfg.GRIPPER_PITCH_NEUTRAL_DEG == 45.0
    assert cfg.GRIPPER_PITCH_MIN == pytest.approx(-1.0)
    assert cfg.GRIPPER_PITCH_MAX == pytest.approx(1.0)
    assert cfg.GRIPPER_YAW_MIN == pytest.approx(-1.0)
    assert cfg.GRIPPER_YAW_MAX == pytest.approx(1.0)

    geo = dict(
        servo_range_deg=cfg.GRIPPER_SERVO_RANGE_DEG,
        pitch_span_deg=cfg.GRIPPER_PITCH_SPAN_DEG,
        wrist_span_deg=cfg.GRIPPER_WRIST_SPAN_DEG,
        pitch_neutral_deg=cfg.GRIPPER_PITCH_NEUTRAL_DEG,
        wrist_neutral_deg=cfg.GRIPPER_WRIST_NEUTRAL_DEG,
    )
    left, right = ControlService._diff_mix_norm_deg(1.0, 1.0, **geo)
    d_pitch = (left + right) * 0.5 * cfg.GRIPPER_SERVO_RANGE_DEG
    d_wrist = (left - right) * 0.5 * cfg.GRIPPER_SERVO_RANGE_DEG

    # Full pitch AND full wrist are both delivered (45 deg each) -- no clip.
    assert abs(d_pitch - 45.0) < 1e-6
    assert abs(d_wrist - 45.0) < 1e-6


def test_servo_reprogram_keeps_pwm_endpoints_constant():
    import rov_config as cfg

    halfspan = cfg.GRIPPER_SERVO_RANGE_DEG * cfg.GRIPPER_US_PER_DEG
    assert halfspan == cfg.GRIPPER_SERVO_PULSE_HALFSPAN_US
    assert cfg.GRIPPER_SERVO_MIN_US == cfg.GRIPPER_SERVO_CENTER_US - cfg.GRIPPER_SERVO_PULSE_HALFSPAN_US
    assert cfg.GRIPPER_SERVO_MAX_US == cfg.GRIPPER_SERVO_CENTER_US + cfg.GRIPPER_SERVO_PULSE_HALFSPAN_US


def test_hundred_degree_servos_cover_full_pitch_wrist_square():
    geo = dict(
        servo_range_deg=100.0,
        pitch_span_deg=90.0,
        wrist_span_deg=90.0,
        pitch_neutral_deg=45.0,
        wrist_neutral_deg=45.0,
    )
    for pitch in (-1.0, 1.0):
        for wrist in (-1.0, 1.0):
            left, right = ControlService._diff_mix_norm_deg(pitch, wrist, **geo)
            assert abs(left) <= 0.90
            assert abs(right) <= 0.90


def test_gripper_calibrate_flat_alignment_pose_matches_current_config():
    import rov_config as cfg
    from tools.gripper_calibrate import mix_pitch_wrist_to_servo_deg

    params = dict(
        servo_range_deg=float(cfg.GRIPPER_SERVO_RANGE_DEG),
        pitch_neutral=float(cfg.GRIPPER_PITCH_NEUTRAL_DEG),
        wrist_neutral=float(cfg.GRIPPER_WRIST_NEUTRAL_DEG),
        left_invert=float(cfg.GRIPPER_LEFT_INVERT),
        right_invert=float(cfg.GRIPPER_RIGHT_INVERT),
    )

    left_90, right_90 = mix_pitch_wrist_to_servo_deg(0.0, 90.0, params)
    assert left_90 == pytest.approx(0.0)
    assert right_90 == pytest.approx(90.0)

    left_0, right_0 = mix_pitch_wrist_to_servo_deg(0.0, 0.0, params)
    assert left_0 == pytest.approx(-90.0)
    assert right_0 == pytest.approx(0.0)


def test_current_repo_disarm_pose_is_flat_wrist_90_and_held():
    import rov_config as cfg

    assert cfg.GRIPPER_DISARM_PITCH == pytest.approx(-1.0)
    assert cfg.GRIPPER_DISARM_YAW == pytest.approx(1.0)
    assert cfg.GRIPPER_ARM_PITCH == pytest.approx(cfg.GRIPPER_DISARM_PITCH)
    assert cfg.GRIPPER_ARM_YAW == pytest.approx(cfg.GRIPPER_DISARM_YAW)
    assert cfg.GRIPPER_HOLD_PWM_ON_DISARM is True

    left, right = ControlService._diff_mix_norm_deg(
        cfg.GRIPPER_DISARM_PITCH,
        cfg.GRIPPER_DISARM_YAW,
        servo_range_deg=cfg.GRIPPER_SERVO_RANGE_DEG,
        pitch_span_deg=cfg.GRIPPER_PITCH_SPAN_DEG,
        wrist_span_deg=cfg.GRIPPER_WRIST_SPAN_DEG,
        pitch_neutral_deg=cfg.GRIPPER_PITCH_NEUTRAL_DEG,
        wrist_neutral_deg=cfg.GRIPPER_WRIST_NEUTRAL_DEG,
        left_invert=cfg.GRIPPER_LEFT_INVERT,
        right_invert=cfg.GRIPPER_RIGHT_INVERT,
    )
    assert left == pytest.approx(0.0)
    assert right == pytest.approx(0.9)


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


def test_compute_gripper_diff_applies_live_right_invert_override():
    # Without override, pure pitch drives both servos the same way.
    base_l, base_r = ControlService._compute_gripper_diff(
        _make_gripper_svc(), SimpleNamespace(aux={"gripper_pitch": 1.0, "gripper_yaw": 0.0}, modes={})
    )
    assert abs(base_l - base_r) < 1e-9
    # A live modes["arm_tune"] right_invert flips one servo without a restart.
    left, right = ControlService._compute_gripper_diff(
        _make_gripper_svc(),
        SimpleNamespace(
            aux={"gripper_pitch": 1.0, "gripper_yaw": 0.0},
            modes={"arm_tune": {"right_invert": -1.0}},
        ),
    )
    assert abs(left - base_l) < 1e-9
    assert abs(right + base_r) < 1e-9
    assert abs(left + right) < 1e-9


def test_compute_gripper_diff_applies_live_pitch_neutral_override():
    # neutral override 45 -> 25 deg shifts where full pitch lands (Dpitch 45 -> 65).
    left, right = ControlService._compute_gripper_diff(
        _make_gripper_svc(),
        SimpleNamespace(
            aux={"gripper_pitch": 1.0, "gripper_yaw": 0.0},
            modes={"arm_tune": {"pitch_neutral_deg": 25.0}},
        ),
    )
    assert abs(left - 65.0 / 70.0) < 1e-6
    assert abs(right - 65.0 / 70.0) < 1e-6


def test_compute_gripper_diff_applies_configured_pitch_roll_limits():
    svc = _make_gripper_svc(
        _gripper_pitch_min=-0.50,
        _gripper_pitch_max=0.25,
        _gripper_yaw_min=-0.25,
        _gripper_yaw_max=0.50,
    )
    left, right = ControlService._compute_gripper_diff(
        svc, SimpleNamespace(aux={"gripper_pitch": 1.0, "gripper_yaw": -1.0}, modes={})
    )
    exp_l, exp_r = ControlService._diff_mix_norm_deg(0.25, -0.25, **GEO)

    assert svc._gripper_last_pitch == pytest.approx(0.25)
    assert svc._gripper_last_yaw == pytest.approx(-0.25)
    assert left == pytest.approx(exp_l)
    assert right == pytest.approx(exp_r)


def test_compute_gripper_diff_applies_live_pitch_roll_limit_override():
    svc = _make_gripper_svc()
    left, right = ControlService._compute_gripper_diff(
        svc,
        SimpleNamespace(
            aux={"gripper_pitch": -1.0, "gripper_yaw": 1.0},
            modes={
                "arm_tune": {
                    "pitch_min": -0.20,
                    "pitch_max": 0.20,
                    "yaw_min": -0.40,
                    "yaw_max": 0.40,
                }
            },
        ),
    )
    exp_l, exp_r = ControlService._diff_mix_norm_deg(-0.20, 0.40, **GEO)

    assert svc._gripper_last_pitch == pytest.approx(-0.20)
    assert svc._gripper_last_yaw == pytest.approx(0.40)
    assert left == pytest.approx(exp_l)
    assert right == pytest.approx(exp_r)


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


def test_send_gripper_park_pose_slews_toward_target(monkeypatch):
    class FakeSink:
        def __init__(self):
            self.writes = []

        def write(self, payload):
            self.writes.append(dict(payload))

    monkeypatch.setattr("control.control_service.time.sleep", lambda _seconds: None)

    svc = _make_gripper_svc(
        _gripper_park_on_arm_disarm=True,
        _gripper_park_pitch=1.0,
        _gripper_park_yaw=0.0,
        _gripper_park_left=FULL_AXIS,
        _gripper_park_right=FULL_AXIS,
        _gripper_park_slew_norm_per_s=0.5,
        _gripper_last_left=0.0,
        _gripper_last_right=0.0,
        period=0.05,
        dry_run=False,
        _sink_armed=True,
        _warned_dry_run=False,
        _warned_no_sink=False,
        _warned_sink_disarmed=False,
    )
    svc._hw_sink = FakeSink()

    ControlService._send_gripper_park_pose(svc, settle_s=0.1)

    assert len(svc._hw_sink.writes) > 1
    assert 0.0 < svc._hw_sink.writes[0]["gripper_left"] < FULL_AXIS
    assert svc._hw_sink.writes[-1]["gripper_left"] == pytest.approx(FULL_AXIS)
    assert svc._gripper_last_left == pytest.approx(FULL_AXIS)
    assert svc._gripper_last_right == pytest.approx(FULL_AXIS)


def test_arm_and_disarm_use_slow_park_settle(monkeypatch):
    class State:
        def __init__(self):
            self.armed = False

        def set_armed(self, value):
            self.armed = bool(value)

    park_calls = []
    sync_calls = []
    set_park_calls = []

    svc = object.__new__(ControlService)
    svc._autopilot = None
    svc.state = State()
    svc._gripper_park_settle_s = 0.85
    svc._armed_since = None
    svc._sync_sink_armed = lambda force=False: sync_calls.append(bool(force))
    svc._set_gripper_park_pose = lambda: set_park_calls.append(True)
    monkeypatch.setattr(
        ControlService,
        "_send_gripper_park_pose",
        lambda self, *, settle_s=0.0: park_calls.append(float(settle_s)),
    )

    ControlService._arm_with_gripper_park(svc)
    assert svc.state.armed is True

    ControlService._disarm_with_gripper_park(svc)
    assert svc.state.armed is False

    assert park_calls == pytest.approx([0.85, 0.85])
    assert sync_calls == [True, True]
    assert set_park_calls == [True]


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
