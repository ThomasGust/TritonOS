import pytest

import rov_config as cfg
from control.mixer import EightThrusterMixer, geometric_mixer_from_config, global_limit

def test_mixer_basic_surge():
    mix = EightThrusterMixer()
    thr = mix.mix({"surge": 1.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0})
    assert thr["H_FL"] == 1.0
    assert thr["H_FR"] == 1.0
    assert thr["H_RL"] == 1.0
    assert thr["H_RR"] == 1.0
    assert thr["V_FL"] == 0.0

def test_global_limit_scales():
    thr = {"H_FL": 2.0, "H_FR": -1.0}
    limited = global_limit(thr, max_abs=1.0)
    # peak was 2.0, so scale by 0.5
    assert abs(limited["H_FL"] - 1.0) < 1e-6
    assert abs(limited["H_FR"] + 0.5) < 1e-6


def test_geometric_mixer_pure_yaw_has_minimal_translation():
    mix = geometric_mixer_from_config(cfg)
    cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 1.0, "pitch": 0.0, "roll": 0.0}
    thr = mix.mix(cmd)
    achieved = mix.allocated_wrench(thr)

    assert achieved["yaw"] == pytest.approx(1.0, abs=0.08)
    assert achieved["surge"] == pytest.approx(0.0, abs=0.05)
    assert achieved["sway"] == pytest.approx(0.0, abs=0.05)
    assert achieved["heave"] == pytest.approx(0.0, abs=0.05)


def test_geometric_mixer_unit_axes_use_full_thruster_authority():
    mix = geometric_mixer_from_config(cfg)
    for axis in ("surge", "sway", "heave", "roll", "pitch", "yaw"):
        cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
        cmd[axis] = 1.0
        thr = mix.mix(cmd)
        assert max(abs(v) for v in thr.values()) == pytest.approx(1.0, abs=0.03)
        achieved = mix.allocated_wrench(thr)
        assert achieved[axis] == pytest.approx(1.0, abs=0.08)


def test_geometric_mixer_can_blend_depth_and_nonlevel_attitude_targets():
    mix = geometric_mixer_from_config(cfg)
    cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.35, "yaw": 0.25, "pitch": -0.20, "roll": 0.15}
    thr = mix.mix(cmd)
    limited = global_limit(thr, max_abs=1.0)
    achieved = mix.allocated_wrench(limited)

    assert max(abs(v) for v in limited.values()) <= 1.0
    assert achieved["heave"] == pytest.approx(cmd["heave"], abs=0.08)
    assert achieved["yaw"] == pytest.approx(cmd["yaw"], abs=0.08)
    assert achieved["pitch"] == pytest.approx(cmd["pitch"], abs=0.08)
    assert achieved["roll"] == pytest.approx(cmd["roll"], abs=0.08)
