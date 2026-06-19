"""Aux (servo) output slew-rate limiting in ThrustWriter."""

from motion.pwm import AuxOutputConfig, ThrustConfig, ThrustWriter, us_to_count


def _bare_writer(slew_norm_per_s: float) -> ThrustWriter:
    """Build a ThrustWriter without touching PWM hardware, with one signed aux."""

    w = object.__new__(ThrustWriter)
    w.cfg = ThrustConfig()
    w._aux_order = ["gripper_left"]
    w.aux_cfg = {
        "gripper_left": AuxOutputConfig(
            input_mode="signed",
            min_us=500,
            max_us=2500,
            center_us=1500,
            deadband_norm=0.0,
            slew_norm_per_s=slew_norm_per_s,
        )
    }
    w._last_aux_counts = [w._aux_norm_to_count("gripper_left", 0.0)]
    w._last_aux_norm = [None]
    w._last_aux_write_t = None
    return w


def test_aux_slew_limits_step_change():
    w = _bare_writer(slew_norm_per_s=3.0)

    # First sample snaps (no previous history) but seeds the limiter at 0.0.
    w._extract_aux_counts({"gripper_left": 0.0}, now=0.0)
    # 0.1 s later a full-scale command is requested; slew caps the step at 3.0*0.1.
    counts = w._extract_aux_counts({"gripper_left": 1.0}, now=0.10)

    assert abs(w._last_aux_norm[0] - 0.30) < 1e-9
    assert counts[0] == us_to_count(1500 + (2500 - 1500) * 0.30, w.cfg.freq_hz)


def test_aux_slew_disabled_passes_through():
    w = _bare_writer(slew_norm_per_s=0.0)

    w._extract_aux_counts({"gripper_left": 0.0}, now=0.0)
    counts = w._extract_aux_counts({"gripper_left": 1.0}, now=0.10)

    assert abs(w._last_aux_norm[0] - 1.0) < 1e-9
    assert counts[0] == us_to_count(2500, w.cfg.freq_hz)


def test_aux_missing_key_holds_last_count():
    w = _bare_writer(slew_norm_per_s=3.0)
    w._last_aux_counts = [4242]

    counts = w._extract_aux_counts({}, now=0.0)

    assert counts[0] == 4242
