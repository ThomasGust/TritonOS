from control.mixer import EightThrusterMixer, global_limit

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
