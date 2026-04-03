import rov_config as cfg

from control.depth_hold import DepthHoldConfig, DepthHoldController


def _effective_low_end_command_threshold() -> float:
    pwm_norm_deadband = float(getattr(cfg, "PWM_DEADBAND", 0.0))
    pwm_us_deadband = float(getattr(cfg, "PWM_DEADBAND_US", 0.0))
    pwm_span_us = float(getattr(cfg, "PWM_SPAN_US", 400.0))
    mix_deadband = float(getattr(cfg, "DEPTH_HOLD_MIX_DEADBAND", 0.0))
    pulse_deadband_as_norm = (pwm_us_deadband / pwm_span_us) if pwm_span_us > 0 else 0.0
    return max(pwm_norm_deadband, pulse_deadband_as_norm, mix_deadband)


def test_depth_hold_small_error_overcomes_output_deadband_quickly():
    dt = 1.0 / float(getattr(cfg, "CONTROL_RATE_HZ", 50.0))
    controller = DepthHoldController(
        DepthHoldConfig(
            sensor_stale_s=float(getattr(cfg, "DEPTH_HOLD_SENSOR_STALE_S", 2.0)),
            depth_lpf_tau_s=float(getattr(cfg, "DEPTH_HOLD_LPF_TAU_S", 0.50)),
            kp=float(getattr(cfg, "DEPTH_HOLD_KP", 0.30)),
            ki=float(getattr(cfg, "DEPTH_HOLD_KI", 0.05)),
            kd=float(getattr(cfg, "DEPTH_HOLD_KD", 0.00)),
            error_deadband_m=float(getattr(cfg, "DEPTH_HOLD_ERROR_DEADBAND_M", 0.03)),
            i_limit=float(getattr(cfg, "DEPTH_HOLD_I_LIMIT", 0.25)),
            out_limit=float(getattr(cfg, "DEPTH_HOLD_OUT_LIMIT", 0.55)),
            sign=float(getattr(cfg, "DEPTH_HOLD_SIGN", 1.0)),
            walk_target=bool(getattr(cfg, "DEPTH_HOLD_WALK_TARGET", True)),
            walk_deadband=float(getattr(cfg, "DEPTH_HOLD_WALK_DEADBAND", 0.08)),
            walk_rate_mps=float(getattr(cfg, "DEPTH_HOLD_WALK_RATE_MPS", 0.60)),
            target_min_m=getattr(cfg, "DEPTH_HOLD_TARGET_MIN_M", None),
            target_max_m=getattr(cfg, "DEPTH_HOLD_TARGET_MAX_M", None),
        )
    )

    # Engage hold at 1.00 m depth, then simulate a small but persistent sink of 5 cm.
    controller.step(enabled=True, manual_heave=0.0, depth_m=1.00, depth_age_s=0.0, dt=dt)

    outputs = []
    for _ in range(int(round(2.0 / dt))):
        u, status = controller.step(
            enabled=True,
            manual_heave=0.0,
            depth_m=1.05,
            depth_age_s=0.0,
            dt=dt,
        )
        assert status["active"] is True
        outputs.append(float(u))

    # A small sustained depth error should now break through the low-end command
    # deadbands quickly enough to be visible on the vertical thrusters.
    assert max(outputs) > _effective_low_end_command_threshold()
