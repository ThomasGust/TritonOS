import pytest

from control.station_keep import (
    StationKeepAxis,
    StationKeepConfig,
    StationKeepController,
    station_keep_config_from_module,
)


def _ctrl(**axis_overrides):
    axis = StationKeepAxis(dof="sway", error_key="ex", kp=0.5, error_deadband=0.02, out_limit=0.4)
    for k, v in axis_overrides.items():
        setattr(axis, k, v)
    return StationKeepController(StationKeepConfig(enable=True, stale_s=0.5, axes=[axis]))


def test_disabled_passes_manual_through_untouched():
    c = _ctrl()
    out, st = c.step(enabled=False, manual_cmd={"sway": 0.3}, visual={"valid": True, "ex": 0.4}, dt=0.02)
    assert out["sway"] == 0.3
    assert st["enabled_cmd"] is False
    assert st["reason"] == "disabled"


def test_no_lock_holds_manual():
    c = _ctrl()
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": False}, dt=0.02)
    assert out["sway"] == 0.0
    assert st["enabled_cmd"] is True
    assert st["active"] is False
    assert st["reason"] == "no_lock"


def test_missing_visual_holds_manual():
    c = _ctrl()
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual=None, dt=0.02)
    assert out["sway"] == 0.0
    assert st["reason"] == "no_lock"


def test_valid_lock_produces_proportional_correction():
    c = _ctrl()
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": True, "ex": 0.4}, dt=0.02)
    assert out["sway"] == pytest.approx(0.2)  # sign=1 * kp=0.5 * ex=0.4
    assert st["active"] is True
    assert st["axes"]["sway"]["active"] is True


def test_sign_flips_correction_direction():
    c = _ctrl(sign=-1.0)
    out, _ = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": True, "ex": 0.4}, dt=0.02)
    assert out["sway"] == pytest.approx(-0.2)


def test_output_is_clamped_to_axis_limit():
    c = _ctrl(kp=5.0, out_limit=0.3)
    out, _ = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": True, "ex": 0.9}, dt=0.02)
    assert out["sway"] == pytest.approx(0.3)


def test_error_deadband_suppresses_tiny_corrections():
    c = _ctrl(error_deadband=0.1)
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": True, "ex": 0.05}, dt=0.02)
    assert out["sway"] == 0.0
    assert st["active"] is False


def test_pilot_manual_input_yields_the_dof():
    c = _ctrl()
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.5}, visual={"valid": True, "ex": 0.4}, dt=0.02)
    assert out["sway"] == 0.5  # untouched
    assert st["axes"]["sway"]["reason"] == "manual_override"


def test_frozen_producer_goes_stale():
    c = _ctrl()
    # Same timestamp held for > stale_s while still "valid" -> stale fallback.
    frozen = {"valid": True, "ex": 0.4, "ts": 123.0}
    reason = None
    for _ in range(40):  # 40 * 0.02 = 0.8s > 0.5s
        out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual=frozen, dt=0.02)
        reason = st["reason"]
    assert reason == "stale_lock"
    assert out["sway"] == 0.0


def test_fresh_timestamps_stay_active():
    c = _ctrl()
    out = st = None
    for i in range(40):
        visual = {"valid": True, "ex": 0.4, "ts": float(i)}
        out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual=visual, dt=0.02)
    assert st["reason"] == "active"
    assert out["sway"] == pytest.approx(0.2)


def test_config_from_module_uses_defaults_and_overrides():
    class _Mod:
        STATION_KEEP_ENABLE = True
        STATION_KEEP_STALE_S = 0.75
        STATION_KEEP_SWAY_KP = 0.8
        STATION_KEEP_SURGE_KP = 0.0

    cfg = station_keep_config_from_module(_Mod())
    assert cfg.enable is True
    assert cfg.stale_s == 0.75
    by_dof = {ax.dof: ax for ax in cfg.axes}
    assert by_dof["sway"].error_key == "ex"
    assert by_dof["sway"].kp == 0.8
    assert by_dof["surge"].error_key == "es"


def test_transect_config_maps_surge_to_ey_and_adds_heave_size_trim():
    """The transect policy: surge centers fore/aft (ey) and heave trims size (es)."""

    class _Mod:
        STATION_KEEP_SURGE_ERROR_KEY = "ey"
        STATION_KEEP_SURGE_KP = 0.45
        STATION_KEEP_HEAVE_ERROR_KEY = "es"
        STATION_KEEP_HEAVE_KP = 0.12
        STATION_KEEP_HEAVE_OUT_LIMIT = 0.15
        STATION_KEEP_YAW_ERROR_KEY = "er"
        STATION_KEEP_YAW_KP = 0.25

    cfg = station_keep_config_from_module(_Mod())
    by_dof = {ax.dof: ax for ax in cfg.axes}
    assert by_dof["sway"].error_key == "ex"
    assert by_dof["surge"].error_key == "ey"   # overridden from the "es" default
    assert by_dof["heave"].error_key == "es"   # additive gentle size trim
    assert by_dof["heave"].kp == 0.12
    assert by_dof["heave"].out_limit == 0.15
    assert by_dof["yaw"].error_key == "er"     # square-up axis
    assert by_dof["yaw"].kp == 0.25


def test_er_rotation_error_drives_yaw():
    cfg = StationKeepConfig(
        enable=True, stale_s=0.5,
        axes=[StationKeepAxis(dof="yaw", error_key="er", kp=0.5, out_limit=0.4)],
    )
    c = StationKeepController(cfg)
    out, st = c.step(enabled=True, manual_cmd={"yaw": 0.0}, visual={"valid": True, "er": 0.4}, dt=0.02)
    assert out["yaw"] == pytest.approx(0.2)   # sign=1 * kp=0.5 * er=0.4
    assert st["axes"]["yaw"]["active"] is True


def test_slew_rate_limits_the_ramp():
    # Big error wants the full out_limit immediately; slew caps the per-step change
    # so it ramps in instead of stepping (softens the engage lurch).
    c = _ctrl(kp=5.0, out_limit=0.4, slew=2.0, error_deadband=0.01)
    v = {"valid": True, "ex": 0.4}
    out1, _ = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual=v, dt=0.1)
    assert out1["sway"] == pytest.approx(0.2)   # 0 + slew(2.0)*dt(0.1) = 0.2, not 0.4
    out2, _ = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual=v, dt=0.1)
    assert out2["sway"] == pytest.approx(0.4)   # ramps the rest of the way, capped


def test_slew_memory_resets_on_no_lock():
    c = _ctrl(kp=5.0, out_limit=0.4, slew=2.0, error_deadband=0.01)
    c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": True, "ex": 0.4}, dt=0.1)
    c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": False}, dt=0.1)  # drops lock
    # Re-acquire: ramp starts from zero again, not the held 0.2.
    out, _ = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": True, "ex": 0.4}, dt=0.1)
    assert out["sway"] == pytest.approx(0.2)


def test_config_from_module_reads_slew():
    class _Mod:
        STATION_KEEP_SWAY_SLEW = 0.7

    cfg = station_keep_config_from_module(_Mod())
    by_dof = {ax.dof: ax for ax in cfg.axes}
    assert by_dof["sway"].slew == 0.7
    assert by_dof["surge"].slew == 0.0   # unset -> axis default (unlimited)


def test_rov_config_disables_yaw_align():
    """Regression: yaw-align is off (its rotation input is currently noise)."""
    import rov_config

    assert rov_config.STATION_KEEP_YAW_KP == 0.0


def test_direct_command_drives_dof_and_is_clamped():
    cfg = StationKeepConfig(enable=True, stale_s=0.5, direct_limit=0.6, axes=[])
    c = StationKeepController(cfg)
    visual = {"valid": True, "command": {"surge": 0.4, "sway": 0.9, "yaw": -0.9}}
    out, st = c.step(enabled=True, manual_cmd={"surge": 0.0, "sway": 0.0, "yaw": 0.0}, visual=visual, dt=0.02)
    assert out["surge"] == pytest.approx(0.4)
    assert out["sway"] == pytest.approx(0.6)   # clamped to direct_limit
    assert out["yaw"] == pytest.approx(-0.6)   # clamped
    assert st["active"] is True
    assert st["axes"]["surge"]["reason"] == "direct"


def test_direct_command_overrides_error_pid_for_same_dof():
    cfg = StationKeepConfig(
        enable=True, stale_s=0.5, direct_limit=1.0,
        axes=[StationKeepAxis(dof="sway", error_key="ex", kp=0.5, out_limit=0.4)],
    )
    c = StationKeepController(cfg)
    visual = {"valid": True, "ex": 0.4, "command": {"sway": -0.3}}
    out, _ = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual=visual, dt=0.02)
    assert out["sway"] == pytest.approx(-0.3)  # direct wins over PID's +0.2


def test_direct_command_yields_to_pilot_manual():
    cfg = StationKeepConfig(enable=True, stale_s=0.5, direct_limit=1.0, axes=[])
    c = StationKeepController(cfg)
    visual = {"valid": True, "command": {"sway": 0.5}}
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.5}, visual=visual, dt=0.02)
    assert out["sway"] == 0.5
    assert st["axes"]["sway"]["reason"] == "manual_override"


def test_direct_command_ignored_without_valid_lock():
    cfg = StationKeepConfig(enable=True, stale_s=0.5, direct_limit=1.0, axes=[])
    c = StationKeepController(cfg)
    out, st = c.step(enabled=True, manual_cmd={"sway": 0.0}, visual={"valid": False, "command": {"sway": 0.5}}, dt=0.02)
    assert out["sway"] == 0.0
    assert st["reason"] == "no_lock"


def test_autopilot_step_applies_station_keep():
    from control.autopilot import AutopilotConfig, AutopilotController, AttitudeAxisConfig
    from control.depth_hold import DepthHoldConfig

    cfg = AutopilotConfig(
        depth_enable=False,
        attitude_enable=False,
        attitude_stale_s=0.5,
        depth=DepthHoldConfig(),
        roll=AttitudeAxisConfig(),
        pitch=AttitudeAxisConfig(),
        yaw=AttitudeAxisConfig(),
        station_keep=StationKeepConfig(
            enable=True, stale_s=0.5,
            axes=[StationKeepAxis(dof="sway", error_key="ex", kp=0.5, out_limit=0.4)],
        ),
    )
    ap = AutopilotController(cfg)
    cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0}
    modes = {"autopilot": {"station_keep": True, "visual": {"valid": True, "ex": 0.4}}}
    out, st = ap.step(
        modes=modes, cmd=cmd, depth_m=None, depth_age_s=None,
        attitude={}, attitude_age_s=None, dt=0.02,
    )
    assert out["sway"] == pytest.approx(0.2)
    assert st["station_keep"]["active"] is True
    assert st["active"] is True


def _alt_autopilot(**over):
    from control.autopilot import AutopilotConfig, AttitudeAxisConfig, AutopilotController
    from control.depth_hold import DepthHoldConfig

    base = dict(
        depth_enable=True, attitude_enable=False, attitude_stale_s=0.5,
        depth=DepthHoldConfig(kp=0.5, ki=0.0, kd=0.0, out_limit=0.5, depth_lpf_tau_s=0.0),
        roll=AttitudeAxisConfig(), pitch=AttitudeAxisConfig(), yaw=AttitudeAxisConfig(),
        station_keep=StationKeepConfig(enable=True, stale_s=0.5, axes=[]),
        alt_from_es=True, alt_kp=0.15, alt_max_offset_m=0.7, alt_sign=-1.0, alt_deadband=0.1,
    )
    base.update(over)
    return AutopilotController(AutopilotConfig(**base))


def _alt_step(ap, es, depth_m, n=1, valid=True):
    modes = {"autopilot": {"depth": True, "station_keep": True,
                           "visual": {"valid": valid, "es": es}}}
    st = None
    for _ in range(n):
        _out, st = ap.step(modes=modes, cmd={"heave": 0.0}, depth_m=depth_m, depth_age_s=0.0,
                           attitude={}, attitude_age_s=None, dt=0.1)
    return st


def test_alt_from_es_servos_depth_setpoint_down_when_too_high():
    ap = _alt_autopilot()
    st = _alt_step(ap, es=-1.0, depth_m=1.0, n=5)   # es<0 = too high -> descend
    alt = st["alt_hold"]
    assert alt["active"] is True
    assert alt["base_m"] == pytest.approx(1.0)      # engage depth captured as base
    assert alt["offset_m"] > 0.0                    # walked the setpoint deeper
    assert alt["target_m"] > 1.0


def test_alt_from_es_climbs_when_too_close():
    ap = _alt_autopilot()
    st = _alt_step(ap, es=1.0, depth_m=2.0, n=5)     # es>0 = too close -> climb
    assert st["alt_hold"]["offset_m"] < 0.0
    assert st["alt_hold"]["target_m"] < 2.0


def test_alt_from_es_offset_is_clamped_for_safety():
    ap = _alt_autopilot()
    st = _alt_step(ap, es=-1.0, depth_m=1.0, n=200)
    assert st["alt_hold"]["offset_m"] == pytest.approx(0.7)   # never beyond max offset


def test_alt_from_es_inactive_when_disabled():
    ap = _alt_autopilot(alt_from_es=False)
    st = _alt_step(ap, es=-1.0, depth_m=1.0, n=3)
    assert st["alt_hold"]["active"] is False
    assert st["alt_hold"]["target_m"] is None


def test_alt_from_es_holds_offset_when_es_in_deadband():
    ap = _alt_autopilot()
    _alt_step(ap, es=-1.0, depth_m=1.0, n=3)          # build some offset
    off = ap._alt_offset
    st = _alt_step(ap, es=0.03, depth_m=1.0, n=5)     # within deadband -> no change
    assert st["alt_hold"]["offset_m"] == pytest.approx(off)


def test_autopilot_combines_dynamic_depth_setpoint_with_direct_translation():
    """Model drives depth via the depth-hold setpoint AND translation directly."""
    from control.autopilot import AutopilotConfig, AutopilotController, AttitudeAxisConfig
    from control.depth_hold import DepthHoldConfig

    cfg = AutopilotConfig(
        depth_enable=True,
        attitude_enable=False,
        attitude_stale_s=0.5,
        depth=DepthHoldConfig(kp=0.5, ki=0.0, kd=0.0, out_limit=0.5, depth_lpf_tau_s=0.0),
        roll=AttitudeAxisConfig(),
        pitch=AttitudeAxisConfig(),
        yaw=AttitudeAxisConfig(),
        station_keep=StationKeepConfig(enable=True, stale_s=0.5, direct_limit=1.0, axes=[]),
    )
    ap = AutopilotController(cfg)
    cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0}
    modes = {
        "autopilot": {
            "depth": True,
            "station_keep": True,
            "targets": {"depth_m": 1.0},
            "visual": {"valid": True, "command": {"surge": 0.3, "sway": -0.2}},
        }
    }
    # Vehicle is above target depth -> depth hold should drive heave.
    out, st = ap.step(
        modes=modes, cmd=cmd, depth_m=0.5, depth_age_s=0.0,
        attitude={}, attitude_age_s=None, dt=0.02,
    )
    assert out["surge"] == pytest.approx(0.3)   # direct translation from model
    assert out["sway"] == pytest.approx(-0.2)
    assert abs(out["heave"]) > 0.0              # depth hold tracking the setpoint
    assert st["depth_hold"]["active"] is True
    assert st["station_keep"]["active"] is True
