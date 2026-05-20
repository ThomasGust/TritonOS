#Later, this is the main file we start all the services/modules from
#!/usr/bin/env python3
"""
Main entry point for the ROV.

Starts:
  - video RPC server
  - pilot/control loop
  - sensor publisher

Assumes your packages are laid out like:

  control/
    pilot_receiver.py
    control_service.py
    mixer.py
  sensors/
    navigator.py
    sensor_pub_service.py
    ms5837.py
  video/
    gst_streamer_rpc.py
  motion/
    pwm.py            (optional)

And that schema/pilot_common.py is the same one the topside uses.
"""

#UPD
from __future__ import annotations
import time
import threading
import sys
import traceback

import rov_config as cfg
from utils.vehicle_reference import DEFAULT_DEPTH_REFERENCE_PATH, load_surface_pressure_reference_mbar


DEFAULT_PILOT_SUB_ENDPOINT = "tcp://0.0.0.0:6000"
DEFAULT_SENSOR_PUB_ENDPOINT = "tcp://0.0.0.0:6001"
DEFAULT_VIDEO_RPC_ENDPOINT = "tcp://0.0.0.0:5555"
DEFAULT_CONTROL_RATE_HZ = 50.0
DEFAULT_PILOT_TTL_S = 0.5


# --- 1) video --------------------------------------------------------
def start_video_service():
    """
    Start the existing gst_streamer RPC in a thread.
    We assume video/gst_streamer_rpc.py exposes a start_video_rpc()
    like in your original code.
    """
    try:
        from video import gst_streamer_rpc
    except Exception as e:
        print("[rov/main] video: could not import video.gst_streamer_rpc:", e)
        traceback.print_exc()
        return

    def _runner():
        try:
            # your original code just called start_video_rpc() and blocked
            gst_streamer_rpc.start_video_rpc()
        except Exception:
            print("[rov/main] video thread crashed:")
            traceback.print_exc()

    t = threading.Thread(target=_runner, daemon=True)
    t.start()
    video_rpc_endpoint = getattr(cfg, "VIDEO_RPC_ENDPOINT", DEFAULT_VIDEO_RPC_ENDPOINT)
    print(f"[rov/main] video RPC started on {video_rpc_endpoint}")


def start_management_service(*, depth_sensor=None, control_service=None):
    if not bool(getattr(cfg, "MANAGEMENT_RPC_ENABLE", True)):
        return None

    try:
        from control.management_rpc import ManagementRpcService
    except Exception as e:
        print("[rov/main] management RPC disabled:", e)
        traceback.print_exc()
        return None

    bind = str(getattr(cfg, "MANAGEMENT_RPC_ENDPOINT", "tcp://0.0.0.0:5556"))
    svc = ManagementRpcService(
        bind_endpoint=bind,
        debug=bool(getattr(cfg, "DEBUG", False)),
        depth_sensor=depth_sensor,
        control_service=control_service,
    )
    svc.start()
    print(f"[rov/main] management RPC started on {bind}")
    return svc


# --- 2) sensors ------------------------------------------------------
def start_sensor_service(ctrl=None, pilot_rx=None, state=None):
    """
    Create Navigator sensors and start the pub service.
    Matches sensors/navigator.py + sensors/sensor_pub_service.py in your tree.
    """
    try:
        from sensors.navigator import (
            NavigatorBoard,
            IMUSensor,
            MagSensor,
            EnvSensor,
            LeakSensor,
            ADCSensor,
            ExternalDepthSensor,
            Bar02Sensor,
            Bar30Sensor,
        )
        from sensors.sensor_pub_service import SensorPublisherService
    except Exception as e:
        print("[rov/main] sensors: could not import sensors modules:", e)
        traceback.print_exc()
        return

    board = NavigatorBoard()

    sensor_list = [
        IMUSensor(board, rate_hz=float(getattr(cfg, "IMU_RATE_HZ", 20.0))),
        MagSensor(board, rate_hz=float(getattr(cfg, "MAG_RATE_HZ", 5.0))),
        EnvSensor(board, rate_hz=2.0),
        LeakSensor(board, rate_hz=2.0),
        ADCSensor(board, rate_hz=5.0),
    ]

    # Power Sense Module -> publish converted voltage/current telemetry.
    if bool(getattr(cfg, "POWER_SENSE_ENABLE", False)):
        try:
            from sensors.power_sense import PowerSenseSensor

            sensor_list.append(
                PowerSenseSensor(
                    board,
                    rate_hz=float(getattr(cfg, "POWER_SENSE_RATE_HZ", 2.0)),
                    volt_mult=float(getattr(cfg, "POWER_SENSE_VOLT_MULT", 11.0)),
                    amps_per_volt=float(getattr(cfg, "POWER_SENSE_AMPS_PER_VOLT", 37.8788)),
                    amps_offset_v=float(getattr(cfg, "POWER_SENSE_AMPS_OFFSET_V", 0.330)),
                    volt_ch=getattr(cfg, "POWER_SENSE_VOLT_CH", None),
                    curr_ch=getattr(cfg, "POWER_SENSE_CURR_CH", None),
                    v_batt_min=float(getattr(cfg, "POWER_SENSE_V_BATT_MIN", 6.0)),
                    v_batt_max=float(getattr(cfg, "POWER_SENSE_V_BATT_MAX", 26.0)),
                    i_min=float(getattr(cfg, "POWER_SENSE_I_MIN", -5.0)),
                    i_max=float(getattr(cfg, "POWER_SENSE_I_MAX", 150.0)),
                    samples_per_read=int(getattr(cfg, "POWER_SENSE_SAMPLES_PER_READ", 5)),
                    ema_alpha=float(getattr(cfg, "POWER_SENSE_EMA_ALPHA", 0.30)),
                    voltage_step_max_v=float(getattr(cfg, "POWER_SENSE_VOLTAGE_STEP_MAX_V", 3.0)),
                    current_step_max_a=float(getattr(cfg, "POWER_SENSE_CURRENT_STEP_MAX_A", 25.0)),
                    negative_current_clamp_a=float(getattr(cfg, "POWER_SENSE_NEGATIVE_CURRENT_CLAMP_A", 0.75)),
                    hold_last_good=bool(getattr(cfg, "POWER_SENSE_HOLD_LAST_GOOD", True)),
                    track_channels=bool(getattr(cfg, "POWER_SENSE_TRACK_CHANNELS", True)),
                    switch_penalty=float(getattr(cfg, "POWER_SENSE_SWITCH_PENALTY", 80.0)),
                    reselect_after_bad=int(getattr(cfg, "POWER_SENSE_RESELECT_AFTER_BAD", 0)),
                )
            )
        except Exception as e:
            print("[rov/main] power sense sensor disabled:", e)

    # Heartbeat (lets topside show link + armed state)
    try:
        from sensors.heartbeat import HeartbeatSensor

        def _hb_state():
            armed = bool(state.is_armed()) if state is not None else False
            seq = None
            age = None
            if pilot_rx is not None:
                p, a = pilot_rx.get_latest()
                age = float(a) if a is not None else None
                seq = int(p.seq) if p is not None else None
            return {
                "armed": armed,
                "pilot_age": age,
                "pilot_seq": seq,
            }

        sensor_list.append(HeartbeatSensor(state_fn=_hb_state, rate_hz=1.0))
    except Exception as e:
        if getattr(cfg, "DEBUG", False):
            print("[rov/main] heartbeat disabled:", e)
    # External depth sensor (Blue Robotics MS5837: Bar30 / Bar02)
    use_external = bool(getattr(cfg, "USE_EXTERNAL_DEPTH", False))
    use_bar02 = bool(getattr(cfg, "USE_BAR02", False))
    use_bar30 = bool(getattr(cfg, "USE_BAR30", False))

    depth_sensor = None

    if use_external or use_bar02 or use_bar30:
        try:
            depth_reference_path = str(
                getattr(cfg, "EXTERNAL_DEPTH_REFERENCE_PATH", DEFAULT_DEPTH_REFERENCE_PATH)
            )
            fixed_surface_pressure = getattr(cfg, "EXTERNAL_DEPTH_FIXED_SURFACE_PRESSURE_MBAR", None)
            if fixed_surface_pressure is not None:
                surface_pressure_mbar = float(fixed_surface_pressure)
            else:
                surface_pressure_mbar = load_surface_pressure_reference_mbar(depth_reference_path)
            depth_offset_m = float(getattr(cfg, "EXTERNAL_DEPTH_SENSOR_TO_TOP_M", 0.0))

            def _get_buses(prefix: str, default_bus: int = 6):
                buses = getattr(cfg, f"{prefix}_I2C_BUSES", None)
                if buses is not None:
                    return buses
                # fall back to a single bus number
                return int(getattr(cfg, f"{prefix}_I2C_BUS", getattr(cfg, "BAR30_I2C_BUS", default_bus)))

            if use_bar02:
                buses = _get_buses("BAR02")
                depth_sensor = Bar02Sensor(
                        rate_hz=float(getattr(cfg, "BAR02_RATE_HZ", getattr(cfg, "BAR30_RATE_HZ", 5.0))),
                        bus=buses,
                        model=getattr(cfg, "BAR02_MODEL", "02BA"),
                        fluid_density=float(getattr(cfg, "BAR02_FLUID_DENSITY", getattr(cfg, "BAR30_FLUID_DENSITY", 1029))),
                        osr=int(getattr(cfg, "BAR02_OSR", getattr(cfg, "BAR30_OSR", 5))),
                        surface_cal_samples=int(getattr(cfg, "BAR02_SURFACE_CAL_SAMPLES", getattr(cfg, "BAR30_SURFACE_CAL_SAMPLES", 15))),
                        surface_cal_delay_s=float(getattr(cfg, "BAR02_SURFACE_CAL_DELAY_S", getattr(cfg, "BAR30_SURFACE_CAL_DELAY_S", 0.02))),
                        surface_pressure_mbar=surface_pressure_mbar,
                        depth_offset_m=depth_offset_m,
                    )
                sensor_list.append(depth_sensor)
            elif use_external:
                buses = getattr(cfg, "EXTERNAL_DEPTH_I2C_BUSES", None)
                if buses is None:
                    buses = _get_buses("BAR30")
                depth_sensor = ExternalDepthSensor(
                        rate_hz=float(getattr(cfg, "EXTERNAL_DEPTH_RATE_HZ", getattr(cfg, "BAR30_RATE_HZ", 5.0))),
                        bus=buses,
                        model=getattr(cfg, "EXTERNAL_DEPTH_MODEL", getattr(cfg, "BAR30_MODEL", "auto")),
                        fluid_density=float(getattr(cfg, "EXTERNAL_DEPTH_FLUID_DENSITY", getattr(cfg, "BAR30_FLUID_DENSITY", 1029))),
                        osr=int(getattr(cfg, "EXTERNAL_DEPTH_OSR", getattr(cfg, "BAR30_OSR", 5))),
                        surface_cal_samples=int(getattr(cfg, "EXTERNAL_DEPTH_SURFACE_CAL_SAMPLES", getattr(cfg, "BAR30_SURFACE_CAL_SAMPLES", 15))),
                        surface_cal_delay_s=float(getattr(cfg, "EXTERNAL_DEPTH_SURFACE_CAL_DELAY_S", getattr(cfg, "BAR30_SURFACE_CAL_DELAY_S", 0.02))),
                        surface_pressure_mbar=surface_pressure_mbar,
                        depth_offset_m=depth_offset_m,
                    )
                sensor_list.append(depth_sensor)
            else:
                buses = _get_buses("BAR30")
                depth_sensor = Bar30Sensor(
                        rate_hz=float(getattr(cfg, "BAR30_RATE_HZ", 5.0)),
                        bus=buses,
                        model=getattr(cfg, "BAR30_MODEL", "auto"),
                        fluid_density=float(getattr(cfg, "BAR30_FLUID_DENSITY", 1029)),
                        osr=int(getattr(cfg, "BAR30_OSR", 5)),
                        surface_cal_samples=int(getattr(cfg, "BAR30_SURFACE_CAL_SAMPLES", 15)),
                        surface_cal_delay_s=float(getattr(cfg, "BAR30_SURFACE_CAL_DELAY_S", 0.02)),
                        surface_pressure_mbar=surface_pressure_mbar,
                        depth_offset_m=depth_offset_m,
                    )
                sensor_list.append(depth_sensor)
        except Exception as e:
            print("[rov/main] sensors: external depth enabled but failed to init:", e)

    # Network stats sensor (tether/wifi visibility + throughput)
    if getattr(cfg, "NET_STATS_ENABLE", False):
        try:
            from sensors.network import NetworkStatsSensor

            sensor_list.append(
                NetworkStatsSensor(
                    rate_hz=float(getattr(cfg, "NET_STATS_RATE_HZ", 1.0)),
                    iface=getattr(cfg, "NET_STATS_IFACE", None),
                )
            )
        except Exception as e:
            print("[rov/main] network stats sensor disabled:", e)

    derived_processors = []
    if bool(getattr(cfg, "ATTITUDE_ESTIMATOR_ENABLE", True)):
        try:
            from sensors.attitude_estimator import AttitudeEstimatorProcessor, RollPitchConfig, RollPitchEstimator

            attitude_cfg = RollPitchConfig(
                calibration_samples=int(getattr(cfg, "ATTITUDE_CALIBRATION_SAMPLES", 30)),
                max_dt_s=float(getattr(cfg, "ATTITUDE_MAX_DT_S", 0.25)),
                accel_tau_s=float(getattr(cfg, "ATTITUDE_ACCEL_TAU_S", 0.16)),
                accel_fast_tau_s=float(getattr(cfg, "ATTITUDE_ACCEL_FAST_TAU_S", 0.055)),
                accel_fast_error_deg=float(getattr(cfg, "ATTITUDE_ACCEL_FAST_ERROR_DEG", 3.0)),
                accel_min_weight=float(getattr(cfg, "ATTITUDE_ACCEL_MIN_WEIGHT", 0.02)),
                accel_max_weight=float(getattr(cfg, "ATTITUDE_ACCEL_MAX_WEIGHT", 0.90)),
                accel_norm_gate=float(getattr(cfg, "ATTITUDE_ACCEL_NORM_GATE", 0.18)),
                calibration_max_tilt_std_deg=float(getattr(cfg, "ATTITUDE_CALIBRATION_MAX_TILT_STD_DEG", 1.25)),
                calibration_max_gyro_rms_dps=float(getattr(cfg, "ATTITUDE_CALIBRATION_MAX_GYRO_RMS_DPS", 3.0)),
                yaw_mag_source=str(getattr(cfg, "ATTITUDE_YAW_MAG_SOURCE", "auto")),
                yaw_tau_s=float(getattr(cfg, "ATTITUDE_YAW_TAU_S", 0.45)),
                yaw_min_weight=float(getattr(cfg, "ATTITUDE_YAW_MIN_WEIGHT", 0.02)),
                yaw_max_weight=float(getattr(cfg, "ATTITUDE_YAW_MAX_WEIGHT", 0.65)),
                yaw_max_mag_age_s=float(getattr(cfg, "ATTITUDE_YAW_MAX_MAG_AGE_S", 0.75)),
                yaw_mag_norm_gate=float(getattr(cfg, "ATTITUDE_YAW_MAG_NORM_GATE", 0.45)),
                stationary_bias_enable=bool(getattr(cfg, "ATTITUDE_STATIONARY_BIAS_ENABLE", True)),
                stationary_bias_tau_s=float(getattr(cfg, "ATTITUDE_STATIONARY_BIAS_TAU_S", 15.0)),
                stationary_gyro_max_dps=float(getattr(cfg, "ATTITUDE_STATIONARY_GYRO_MAX_DPS", 1.0)),
                stationary_accel_error_max_deg=float(getattr(cfg, "ATTITUDE_STATIONARY_ACCEL_ERROR_MAX_DEG", 1.5)),
                stationary_accel_norm_error_max=float(getattr(cfg, "ATTITUDE_STATIONARY_ACCEL_NORM_ERROR_MAX", 0.05)),
            )
            derived_processors.append(AttitudeEstimatorProcessor(RollPitchEstimator(attitude_cfg)))
        except Exception as e:
            print("[rov/main] onboard attitude estimator disabled:", e)

    srv = SensorPublisherService(
        bind_endpoint=getattr(cfg, "SENSOR_PUB_ENDPOINT", DEFAULT_SENSOR_PUB_ENDPOINT),
        sensors=sensor_list,
        derived_processors=derived_processors,
        debug=bool(getattr(cfg, "DEBUG", False)),
    )
    srv.start()
    print(f"[rov/main] sensor PUB started on {getattr(cfg, 'SENSOR_PUB_ENDPOINT', DEFAULT_SENSOR_PUB_ENDPOINT)}")
    return srv, depth_sensor


# --- 3) control / pilot ----------------------------------------------
def start_control_service():
    """
    Start the pilot SUB + control loop.
    We expect control/control_service.py to contain:
        - ControlService
        - ControlGains
        - ROVControlState
    and control/pilot_receiver.py to contain:
        - PilotReceiver
    """
    try:
        from control.pilot_receiver import PilotReceiver
        from control.control_service import ControlService, ControlGains, ROVControlState
    except Exception as e:
        print("[rov/main] control: could not import control modules:", e)
        traceback.print_exc()
        return None, None, None

    # optional: hardware sink
    hw_sink = None
    try:
        from motion import pwm
        from motion.channel_map import ChannelMap
    except Exception as e:
        print("[rov/main] motion: not using hardware PWM (import failed):", e)
    else:
        try:
            # expect pwm.py to give us something like write_thrust(dict)
            if hasattr(pwm, "ThrustWriter"):
                # The ThrustWriter uses the official BlueRobotics Navigator Python
                # bindings (bluerobotics_navigator). PWM outputs can be kept disabled until ARM
                # and will hold neutral for ESC initialization when armed.
                chanmap = ChannelMap.from_config(cfg)

                thrust_cfg = pwm.ThrustConfig(
                    backend=str(getattr(cfg, "PWM_BACKEND", "auto")),
                    freq_hz=getattr(cfg, "PWM_FREQ_HZ", 50.0),
                    neutral_us=getattr(cfg, "PWM_NEUTRAL_US", 1500),
                    span_us=getattr(cfg, "PWM_SPAN_US", 400),
                    min_us=getattr(cfg, "PWM_MIN_US", 1100),
                    max_us=getattr(cfg, "PWM_MAX_US", 1900),
                    deadband_norm=getattr(cfg, "PWM_DEADBAND", 0.07),
                    deadband_us=getattr(cfg, "PWM_DEADBAND_US", 25),
                    trim_us=getattr(cfg, "PWM_TRIM_US", 0),
                    esc_init_hold_s=getattr(cfg, "ESC_INIT_HOLD_S", 3.0),
                    hardware_arm_disarm=bool(getattr(cfg, "HARDWARE_ARM_DISARM", True)),
                    pwm_rearm_off_s=float(getattr(cfg, "PWM_REARM_OFF_S", 0.35)),
                    pwm_disarm_hold_s=float(getattr(cfg, "PWM_DISARM_HOLD_S", 0.25)),
                    disable_pwm_on_disarm=bool(getattr(cfg, "DISABLE_PWM_ON_DISARM", True)),
                    keep_pwm_enabled_on_disarm=getattr(cfg, "KEEP_PWM_ENABLED_ON_DISARM", True),
                    channel_base=getattr(cfg, "PWM_CHANNEL_BASE", "auto"),
                    slew_rate_norm_per_s=float(getattr(cfg, "THRUSTER_SLEW_RATE_NORM_PER_S", 0.0)),
                    slew_reverse_rate_norm_per_s=getattr(cfg, "THRUSTER_SLEW_REVERSE_RATE_NORM_PER_S", None),
                    slew_dt_max_s=float(getattr(cfg, "THRUSTER_SLEW_DT_MAX_S", 0.10)),
                    direct_i2c_bus=getattr(cfg, "PWM_DIRECT_I2C_BUS", 4),
                    direct_i2c_addr=getattr(cfg, "PWM_DIRECT_I2C_ADDR", 0x40),
                    direct_osc_hz=float(getattr(cfg, "PWM_DIRECT_OSC_HZ", 25_000_000.0)),
                    direct_oe_gpio=getattr(cfg, "PWM_DIRECT_OE_GPIO", 26),
                    direct_oe_active_low=bool(getattr(cfg, "PWM_DIRECT_OE_ACTIVE_LOW", True)),
                )

                # Merge reversal maps: allow either thruster-name keys or raw channel keys.
                rev_map = {}
                rev_map.update(getattr(cfg, "THRUSTER_REVERSED", {}) or {})
                rev_map.update(getattr(cfg, "CHANNEL_REVERSED", {}) or {})

                # Optional aux PWM outputs (e.g. lights, differential wrist servos).
                # The ControlService provides normalized values by name.
                aux_channels = {}
                aux_cfg = {}

                def _register_aux_output(name: str, channel_attr: str, output_cfg: "pwm.AuxOutputConfig") -> None:
                    ch = getattr(cfg, channel_attr, None)
                    if ch is None:
                        return
                    aux_channels[str(name)] = int(ch)
                    aux_cfg[str(name)] = output_cfg

                def _clamp_signed_norm(v: float) -> float:
                    return max(-1.0, min(1.0, float(v)))

                try:
                    _register_aux_output(
                        "lights",
                        "LIGHTS_PWM_CHANNEL",
                        pwm.AuxOutputConfig(
                            min_us=int(getattr(cfg, "LIGHTS_US_MIN", getattr(cfg, "LIGHTS_MIN_US", 1100))),
                            max_us=int(getattr(cfg, "LIGHTS_US_MAX", getattr(cfg, "LIGHTS_MAX_US", 1900))),
                            off_us=int(getattr(cfg, "LIGHTS_US_OFF", getattr(cfg, "LIGHTS_OFF_US", 1100))),
                            deadband_norm=float(getattr(cfg, "LIGHTS_DEADBAND_NORM", getattr(cfg, "LIGHTS_DEADZONE", 0.02))),
                            trim_us=int(getattr(cfg, "LIGHTS_TRIM_US", 0)),
                            allow_when_disarmed=bool(getattr(cfg, "LIGHTS_ALLOW_WHEN_DISARMED", True)),
                            force_off_on_disarm=bool(getattr(cfg, "LIGHTS_FORCE_OFF_ON_DISARM", False)),
                        ),
                    )

                    if bool(getattr(cfg, "GRIPPER_ENABLE", True)):
                        disarm_pitch = getattr(cfg, "GRIPPER_DISARM_PITCH", getattr(cfg, "GRIPPER_DISARM_NORM", None))
                        disarm_yaw = getattr(cfg, "GRIPPER_DISARM_YAW", 0.0)
                        left_disarm = None
                        right_disarm = None
                        if disarm_pitch is not None:
                            pitch_v = float(disarm_pitch)
                            yaw_v = float(disarm_yaw or 0.0)
                            pitch_v, yaw_v = ControlService._limit_gripper_axes_preserve_pitch(pitch_v, yaw_v)
                            left_disarm, right_disarm = ControlService._mix_gripper_axes(pitch_v, yaw_v)
                            left_disarm = _clamp_signed_norm(left_disarm)
                            right_disarm = _clamp_signed_norm(right_disarm)

                        gripper_cfg_base = dict(
                            min_us=int(getattr(cfg, "GRIPPER_SERVO_MIN_US", 500)),
                            max_us=int(getattr(cfg, "GRIPPER_SERVO_MAX_US", 2500)),
                            off_us=int(getattr(cfg, "GRIPPER_SERVO_CENTER_US", 1500)),
                            deadband_norm=float(getattr(cfg, "GRIPPER_DEADBAND", 0.01)),
                            trim_us=int(getattr(cfg, "GRIPPER_TRIM_US", 0)),
                            input_mode="signed",
                            center_us=int(getattr(cfg, "GRIPPER_SERVO_CENTER_US", 1500)),
                            allow_when_disarmed=bool(getattr(cfg, "GRIPPER_ALLOW_WHEN_DISARMED", False)),
                            force_off_on_disarm=bool(getattr(cfg, "GRIPPER_FORCE_OFF_ON_DISARM", False)),
                            center_on_disarm=bool(getattr(cfg, "GRIPPER_CENTER_ON_DISARM", True)),
                            hold_pwm_on_disarm=bool(getattr(cfg, "GRIPPER_HOLD_PWM_ON_DISARM", False)),
                        )
                        _register_aux_output(
                            "gripper_left",
                            "GRIPPER_LEFT_PWM_CHANNEL",
                            pwm.AuxOutputConfig(disarm_norm=left_disarm, **gripper_cfg_base),
                        )
                        _register_aux_output(
                            "gripper_right",
                            "GRIPPER_RIGHT_PWM_CHANNEL",
                            pwm.AuxOutputConfig(disarm_norm=right_disarm, **gripper_cfg_base),
                        )
                except Exception:
                    aux_channels = {}
                    aux_cfg = {}

                # Optional wrist rotate motor (T200) on an aux-mapped channel.
                # We intentionally drive it as a *thruster-style* channel (neutral 1500us,
                # bidirectional [-1..1]) rather than as a unidirectional aux output like lights.
                thruster_channels = dict(chanmap.thrusters)
                wrist_ch = getattr(cfg, "WRIST_ROTATE_PWM_CHANNEL", None)
                if wrist_ch is None:
                    try:
                        wrist_ch = chanmap.aux.get("wrist_rotate")
                    except Exception:
                        wrist_ch = None
                if getattr(cfg, "WRIST_ROTATE_ENABLE", True) and (wrist_ch is not None):
                    thruster_channels[str(getattr(cfg, "WRIST_ROTATE_CMD_KEY", "wrist_rotate"))] = int(wrist_ch)

                hw_sink = pwm.ThrustWriter(
                    thruster_channels=thruster_channels,
                    cfg=thrust_cfg,
                    reversed_map=rev_map if rev_map else None,
                    aux_channels=aux_channels if aux_channels else None,
                    aux_cfg=aux_cfg if aux_cfg else None,
                    debug=getattr(cfg, "DEBUG", False),
                    auto_enable=bool(getattr(cfg, "PWM_AUTO_ENABLE", False)),
                )
                backend_name = str(getattr(hw_sink, "backend_name", "unknown"))
                print(f"[rov/main] motion: using PWM backend={backend_name}")
            elif hasattr(pwm, "write_thrust"):
                hw_sink = pwm.write_thrust
                print("[rov/main] motion: using pwm.write_thrust(...)")
            else:
                print("[rov/main] motion: pwm.py imported but no known sink found; will print thrusters")
        except Exception as e:
            print("[rov/main] motion: hardware PWM init failed:", e)
            hw_sink = None


    # Make it extremely clear whether we are in dry_run mode.
    if hw_sink is None:
        print('[rov/main] motion: NO hardware PWM sink -> motors will NOT run (dry_run mode).')
        if bool(getattr(cfg, 'REQUIRE_HARDWARE_PWM', False)):
            raise SystemExit('[rov/main] FATAL: REQUIRE_HARDWARE_PWM is set, but hardware PWM could not be initialized. If tools/native_motor_test works, you are likely missing permissions here (run with sudo or fix /dev/i2c-* access).')
    else:
        print('[rov/main] motion: hardware PWM sink ready.')


    # pilot receiver
    pilot_rx = PilotReceiver(
        bind_endpoint=getattr(cfg, "PILOT_SUB_ENDPOINT", DEFAULT_PILOT_SUB_ENDPOINT),
        debug=bool(getattr(cfg, "PILOT_RX_DEBUG", False)),
    )
    pilot_rx.start()

    # shared state (armed flag)
    state = ROVControlState()
    # Safety: start DISARMED. Use MENU on the controller to toggle armed.
    state.set_armed(False)


    gains = ControlGains(
        surge=1.0,
        sway=1.0,
        heave=1.0,
        # 4-DOF layout: surge/sway (horizontals) + heave/pitch (verticals).
        # Yaw is disabled by default; map it later if you want turning.
        yaw=1.0,
        pitch=1.0,
        roll=1.0,
        power_scale=float(getattr(cfg, "POWER_SCALE", 1.0)),
    )
    ctrl = ControlService(
        pilot_rx=pilot_rx,
        gains=gains,
        control_state=state,
        rate_hz=float(getattr(cfg, "CONTROL_RATE_HZ", DEFAULT_CONTROL_RATE_HZ)),
        ttl=float(getattr(cfg, "PILOT_TTL", DEFAULT_PILOT_TTL_S)),
        debug=bool(getattr(cfg, "CONTROL_DEBUG", False)),
        dry_run=(hw_sink is None),
    )

    # Attach hardware sink (thrusters) once, if available.
    if hw_sink is not None and hasattr(ctrl, "set_hw_sink"):
        try:
            ctrl.set_hw_sink(hw_sink)
            # once we have a sink, disable dry_run so commands actually hit hardware
            ctrl.dry_run = False
        except Exception as e:
            print("[rov/main] warning: could not attach hw sink:", e)

    ctrl.start()
    print(f"[rov/main] control loop started (rate={float(getattr(cfg, 'CONTROL_RATE_HZ', DEFAULT_CONTROL_RATE_HZ))} Hz)")

    return ctrl, pilot_rx, state


def main():
    print("[rov/main] starting services…")

    # Optional: start netdiag server (UDP echo + TCP throughput)
    if getattr(cfg, "NETDIAG_ENABLE", False):
        try:
            from tools import netdiag_server

            netdiag_server.start_in_thread(
                bind_host=str(getattr(cfg, "NETDIAG_BIND_HOST", "0.0.0.0")),
                port=int(getattr(cfg, "NETDIAG_PORT", 7700)),
                verbose=bool(getattr(cfg, "DEBUG", False)),
            )
            print(f"[rov/main] netdiag server started on {getattr(cfg, 'NETDIAG_BIND_HOST', '0.0.0.0')}:{getattr(cfg, 'NETDIAG_PORT', 7700)}")
        except Exception as e:
            print("[rov/main] netdiag server disabled:", e)

    # start each service in turn
    start_video_service()
    ctrl, pilot_rx, state = start_control_service()
    _, depth_sensor = start_sensor_service(ctrl=ctrl, pilot_rx=pilot_rx, state=state)
    start_management_service(depth_sensor=depth_sensor, control_service=ctrl)

    print("[rov/main] all services started.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[rov/main] shutting down…")
        sys.exit(0)


if __name__ == "__main__":
    main()
