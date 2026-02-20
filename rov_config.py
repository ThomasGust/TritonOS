# rov_config.py
#
# SINGLE SOURCE OF TRUTH:
#   - Edit ONLY CHANNEL_MAP (physical channels 1..16).
#   - Everything else derives from it.
#
# This avoids the “mapping garbage” where different parts of the codebase
# use different lists / numbering bases / overlapping channels.

CONFIG_VERSION = "simple_channelmap_single_source_2026-01-30"

# ---------------------------------------------------------------------------
# 1) network endpoints
# ---------------------------------------------------------------------------

PILOT_SUB_ENDPOINT = "tcp://0.0.0.0:6000"   # topside → ROV pilot data
SENSOR_PUB_ENDPOINT = "tcp://0.0.0.0:6001"  # ROV → topside sensor data
VIDEO_RPC_ENDPOINT = "tcp://0.0.0.0:5555"   # ROV video RPC (gst streamer)

# ---------------------------------------------------------------------------
# 2) control loop
# ---------------------------------------------------------------------------

CONTROL_RATE_HZ = 50.0
PILOT_TTL = 0.5  # seconds before we consider pilot stale

# Control mixing mode:
#   - "simple_groups": bring-up mode (surge drives all horizontals, heave drives all verticals)
#   - "six_dof": full mixer (surge/sway/yaw on horizontals; heave/pitch/roll on verticals)
CONTROL_MIX_MODE = "six_dof"

# Controller axis mapping (PilotFrame.axes fields).
# Desired mapping:
#   left stick Y (ly) -> surge
#   left stick X (lx) -> pitch
#   right stick Y (ry) -> heave
#   right stick X (rx) -> sway
#
# Notes:
#   - Set AXIS_YAW = "none" to disable yaw if you don't want turning.
#   - AXIS_PITCH/AXIS_ROLL override D-pad pitch/roll when set.
AXIS_SURGE = "ly"
AXIS_YAW   = "lx"

# Put pitch/roll somewhere “secondary” for now:
AXIS_PITCH = "dpad_y"   # or "none" to fully disable pitch
AXIS_ROLL  = "dpad_x"   # optional; leaving it unset also defaults to dpad
AXIS_HEAVE = "ry"
AXIS_SWAY  = "rx"
AXIS_YAW   = "lx"
AXIS_ROLL  = "dpad_x"

# Invert scalars (+1.0 or -1.0)
AXIS_SURGE_INVERT = 1.0
AXIS_SWAY_INVERT  = 1.0
AXIS_HEAVE_INVERT = 1.0
AXIS_PITCH_INVERT = 1.0
AXIS_YAW_INVERT   = 1.0

AXIS_DEADZONE = 0.10  # 10% stick deadzone

# Overall thrust cap (0..1). 1.0 = full output.
# Set this lower (e.g. 0.6) to retduce peak current draw / brownouts while tuning.
THRUSTER_MAX_ABS = 1.0

# Per-thruster deadband applied after mixing (extra protection against creep).
# This is applied on the ROV side even if the pilot deadzones look good.
MIX_OUTPUT_DEADBAND = 0.05

# When depth-hold is enabled, allow smaller vertical corrections to avoid the
# controller being "nulled out" near setpoint.
DEPTH_HOLD_MIX_DEADBAND = 0.02

# Additional global scaling applied to pilot DOFs before mixing (0..1).
# You can usually leave this at 1.0 and only use THRUSTER_MAX_ABS.
POWER_SCALE = 1.0

# ---------------------------------------------------------------------------
# 2c) depth hold (sticky / "walk target")
# ---------------------------------------------------------------------------
# Depth hold is enabled/disabled by the pilot via PilotFrame.modes["depth_hold"].
# These parameters tune the onboard controller.

# Master switch to compile/initialize depth-hold support.
DEPTH_HOLD_ENABLE = True

# If depth telemetry is older than this, depth-hold will disengage to manual.
DEPTH_HOLD_SENSOR_STALE_S = 2.0

# Low-pass filter time constant on depth (seconds).
DEPTH_HOLD_LPF_TAU_S = 0.50

# PI(D) gains (heave-command per meter / meter-second)
DEPTH_HOLD_KP = 0.30
DEPTH_HOLD_KI = 0.05
DEPTH_HOLD_KD = 0.00

# Error deadband in meters (reduces thruster chatter near setpoint)
DEPTH_HOLD_ERROR_DEADBAND_M = 0.03

# Integrator clamp (in heave-command units)
DEPTH_HOLD_I_LIMIT = 0.25

# Output clamp (in heave command units; keep < 1.0 while tuning)
DEPTH_HOLD_OUT_LIMIT = 0.55

# If the controller pushes the wrong way, flip this to -1.0.
DEPTH_HOLD_SIGN = 1.0

# "Walk target" behavior: stick commands move the target depth; releasing holds.
DEPTH_HOLD_WALK_TARGET = True
DEPTH_HOLD_WALK_DEADBAND = 0.08
DEPTH_HOLD_WALK_RATE_MPS = 0.60  # full stick => ~0.6 m/s target change

# Optional clamp on target depth (meters, depth positive down). Set to None to disable.
DEPTH_HOLD_TARGET_MIN_M = None
DEPTH_HOLD_TARGET_MAX_M = None

# ---------------------------------------------------------------------------
# 2b) arming safety
# ---------------------------------------------------------------------------
# If True, the ROV will refuse to ARM unless sticks are centered and triggers
# are at rest. This prevents "ARM -> instant max thrust" when the topside axis
# mapping is wrong (for example, a trigger axis accidentally mapped into ry).
ARM_REQUIRE_NEUTRAL = True

# Max allowed absolute stick deflection (lx/ly/rx/ry) to permit arming.
ARM_CENTER_TOL = 0.18

# Max allowed trigger values (lt/rt) to permit arming (topside normalizes to [0..1]).
ARM_TRIGGER_TOL = 0.10

# Seconds to ramp from neutral to commanded output after a successful ARM.
ARM_RAMP_S = 0.35

# ---------------------------------------------------------------------------
# 3) sensors
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Power Sense Module (Blue Robotics PSM)
# ---------------------------------------------------------------------------
# TritonOS already publishes raw ADC voltages as `type='adc'`.
# Enable this to also publish a converted `type='power'` message containing
# battery voltage (V) and current (A), using the Blue Robotics formulas.
#
# Notes:
# - The PSM provides two analog outputs:
#     * VOLTAGE sense:  V_batt = V_adc * VOLT_MULT
#     * CURRENT sense:  I_amps = (V_adc - AMPS_OFFSET) * AMPS_PER_VOLT
# - Different PSM revisions have different AMPS_PER_VOLT values. The current
#   Blue Robotics store listing specifies 37.8788 A/V with 0.330 V offset.
#   The older R1 PSM uses 56.81818 A/V with the same 0.330 V offset.
# - If you don't know which ADS1115 channels are wired, leave *_CH = None.
#   TritonOS will auto-detect a plausible mapping and will re-detect if the
#   mapping becomes invalid (helps when the channel ordering appears to change).

POWER_SENSE_ENABLE = True
POWER_SENSE_RATE_HZ = 2.0

# Optional fixed ADS1115 channels (0..3). Set to None for auto-detect.
POWER_SENSE_VOLT_CH = 3
POWER_SENSE_CURR_CH = 2

# Conversion constants (defaults match the current Blue Robotics PSM revision).
POWER_SENSE_VOLT_MULT = 11.0
POWER_SENSE_AMPS_PER_VOLT = 37.8788
POWER_SENSE_AMPS_OFFSET_V = 0#0.330

# Plausibility bounds for validity checks.
# Tip: Set these tighter if you know your pack range; it makes the mapping
# selection and spike rejection more robust.
POWER_SENSE_V_BATT_MIN = 6.0
POWER_SENSE_V_BATT_MAX = 26.0
POWER_SENSE_I_MIN = -5.0
POWER_SENSE_I_MAX = 150.0

# Reliability / filtering
POWER_SENSE_SAMPLES_PER_READ = 5             # Median of N samples (rejects spikes)
POWER_SENSE_EMA_ALPHA = 0.30                 # 0 disables smoothing
POWER_SENSE_VOLTAGE_STEP_MAX_V = 3.0         # Larger jumps treated as outliers
POWER_SENSE_CURRENT_STEP_MAX_A = 25.0        # Larger jumps treated as outliers
POWER_SENSE_NEGATIVE_CURRENT_CLAMP_A = 0.75  # Clamp small negatives to 0
POWER_SENSE_HOLD_LAST_GOOD = True            # Hold last good value on outliers

# Channel handling
POWER_SENSE_TRACK_CHANNELS = True            # Re-evaluate mapping with hysteresis
POWER_SENSE_SWITCH_PENALTY = 80.0            # Larger => less likely to switch
POWER_SENSE_RESELECT_AFTER_BAD = 0           # 0 disables full reselect (recommended)

# External BlueRobotics MS5837 pressure sensor (Bar30 / Bar02).
# When enabled, TritonOS will publish an `external_depth` message that includes:
#   - depth_m (relative to a surface reference measured at startup)
#   - pressure_mbar (absolute)
#   - temperature_c
#
# Notes:
#   - The driver can auto-detect Bar30 vs Bar02 from the PROM. When auto-detect
#     works, TritonOS will publish the sensor row name as `bar30` or `bar02`.
#   - You can also force the model by setting *_MODEL to "30BA" or "02BA".
#
# Preferred (new) switches:
USE_EXTERNAL_DEPTH = True
USE_BAR02 = False  # set True to force Bar02 naming/model
USE_BAR30 = USE_EXTERNAL_DEPTH  # backward-compat alias (old configs used USE_BAR30)

# I2C bus selection
# Navigator external I2C bus is commonly 6, but many Pi setups use bus 1.
# You can provide a *list/tuple* to try multiple buses at startup:
EXTERNAL_DEPTH_I2C_BUSES = (6, 1)
EXTERNAL_DEPTH_I2C_BUS = 6  # legacy single-bus form (used if *_I2C_BUSES not set)

# Model detection/forcing:
#   - "auto" reads PROM and auto-detects 30BA vs 02BA
#   - "30BA" or "02BA" forces
EXTERNAL_DEPTH_MODEL = "auto"

# Water density in kg/m^3 (997 freshwater, 1029 seawater).
EXTERNAL_DEPTH_FLUID_DENSITY = 1029

# Oversampling (0..5) = 256..8192. Higher = less noise, slower reads.
EXTERNAL_DEPTH_OSR = 5

# Surface reference calibration at startup.
EXTERNAL_DEPTH_SURFACE_CAL_SAMPLES = 15
EXTERNAL_DEPTH_SURFACE_CAL_DELAY_S = 0.02

# -----------------------------------------------------------------------------
# Backward-compatible aliases (older code/configs use BAR30_* names)
# -----------------------------------------------------------------------------
BAR30_I2C_BUS = EXTERNAL_DEPTH_I2C_BUS
BAR30_I2C_BUSES = EXTERNAL_DEPTH_I2C_BUSES
BAR30_MODEL = EXTERNAL_DEPTH_MODEL
BAR30_FLUID_DENSITY = EXTERNAL_DEPTH_FLUID_DENSITY
BAR30_OSR = EXTERNAL_DEPTH_OSR
BAR30_SURFACE_CAL_SAMPLES = EXTERNAL_DEPTH_SURFACE_CAL_SAMPLES
BAR30_SURFACE_CAL_DELAY_S = EXTERNAL_DEPTH_SURFACE_CAL_DELAY_S

# Optional Bar02-specific overrides (used when USE_BAR02=True)
BAR02_I2C_BUS = EXTERNAL_DEPTH_I2C_BUS
BAR02_I2C_BUSES = EXTERNAL_DEPTH_I2C_BUSES
BAR02_MODEL = "02BA"

# ---------------------------------------------------------------------------
# 4) misc / diagnostics

# ---------------------------------------------------------------------------

DEBUG = True

# Control-loop / pilot RX verbose printing (keep OFF in normal ops).
CONTROL_DEBUG = False
PILOT_RX_DEBUG = False

# If True, prints file path + channel map when rov_config is imported.
PRINT_CONFIG_ON_IMPORT = True

# Publish live network stats onto SENSOR_PUB_ENDPOINT stream
NET_STATS_ENABLE = True
NET_STATS_RATE_HZ = 1.0
NET_STATS_IFACE = None

# Optional lightweight speed-test server
NETDIAG_ENABLE = True
NETDIAG_BIND_HOST = "0.0.0.0"
NETDIAG_PORT = 7700

# ---------------------------------------------------------------------------
# 5) thruster PWM (Navigator)
# ---------------------------------------------------------------------------

PWM_FREQ_HZ = 50.0

# Pulse shaping (tune if ESCs creep)
PWM_NEUTRAL_US = 1500
PWM_SPAN_US = 400      # +/- span around neutral (400 => ~1100-1900)
PWM_MIN_US = 1100
PWM_MAX_US = 1900

# Deadbands
PWM_DEADBAND = 0.07     # normalized thrust deadband
PWM_DEADBAND_US = 25    # microsecond deadband around neutral

# Trim can be:
#   - int: global trim in microseconds applied to all thrusters
#   - dict: per-thruster trims, e.g. {"H_FL": -8, "H_FR": +5}
PWM_TRIM_US = 0

# ESC initialization neutral hold when (re)enabling outputs (seconds)
ESC_INIT_HOLD_S = 3.0

# Hardware arming/disarming:
#   - If PWM_AUTO_ENABLE is False, PWM outputs stay disabled at boot and are only
#     enabled when you ARM from the controller (recommended).
#   - If HARDWARE_ARM_DISARM is True, each ARM/DISARM physically toggles Navigator
#     PWM enable (OE) so ESCs re-acquire signal and produce obvious tones.
PWM_AUTO_ENABLE = False
HARDWARE_ARM_DISARM = True
PWM_REARM_OFF_S = 0.35
PWM_DISARM_HOLD_S = 0.25
DISABLE_PWM_ON_DISARM = True

# Shift joystick ramp start by this many seconds after ARM (prevents a jump right
# after the ESC init/arm window). Default matches ESC_INIT_HOLD_S when hardware
# arming is enabled.
ARM_HW_INIT_HOLD_S = ESC_INIT_HOLD_S if HARDWARE_ARM_DISARM else 0.0

# Legacy behavior (soft disarm): keep PWM enabled and actively drive neutral.
# Ignored when HARDWARE_ARM_DISARM is True and DISABLE_PWM_ON_DISARM is True.
KEEP_PWM_ENABLED_ON_DISARM = False

# If True, TritonOS will exit if hardware PWM cannot be initialized
REQUIRE_HARDWARE_PWM = False

# ---------------------------------------------------------------------------
# 6) CHANNEL MAP (EDIT ONLY THIS SECTION)
# ---------------------------------------------------------------------------
#
# Physical Navigator channel numbering (1..16), same numbering as native_motor_test.
#
# Your wiring:
#   Lights: physical channel 5
#   Motors 0,1,2,3: physical channels 1,2,3,4
#   Motors 5,6,7,8: physical channels 6,7,8,9
#
# Your desired grouping behavior:
#   Horizontals should be motors: 7,5,1,6  => channels 8,6,2,7
#   Verticals should be motors:   3,2,0,8  => channels 4,3,1,9
#
# IMPORTANT:
#   Thruster naming is functional (H_FL/H_FR/H_RL/H_RR and V_FL/V_FR/V_RL/V_RR).
#   If yaw/sway directions feel wrong later, you’ll either:
#     - swap which physical motor is assigned to which name here, or
#     - set THRUSTER_REVERSED for that named thruster.

CHANNEL_MAP = {
    "thrusters": {
        # Horizontals (surge/sway/yaw) — should be motors 7,5,1,6
        "H_FL": 8,  # motor7 -> physical channel 8
        "H_FR": 6,  # motor5 -> physical channel 6
        "H_RL": 7,  # motor6 -> physical channel 7
        "H_RR": 2,  # motor1 -> physical channel 2

        # Verticals (heave/pitch/roll)
        # Your wiring (physical Navigator channels):
        #   V_FL (front-left)  = 3
        #   V_FR (front-right) = 4
        #   V_RL (rear-left)   = 9
        #   V_RR (rear-right)  = 1
        "V_FL": 3,
        "V_FR": 4,
        "V_RL": 9,
        "V_RR": 1,
    },
    "aux": {
        "lights": 5,  # physical channel 5
    },
}

# ---- Derived aliases (do not edit) -----------------------------------------

THRUSTER_CHANNELS = dict(CHANNEL_MAP["thrusters"])
AUX_PWM_CHANNELS = dict(CHANNEL_MAP.get("aux", {}))
LIGHTS_PWM_CHANNEL = AUX_PWM_CHANNELS.get("lights")
MOTOR_PWM_CHANNELS = sorted(THRUSTER_CHANNELS.values())

# Optional per-thruster direction flips.
# Keys should be thruster names (preferred) or raw channel numbers.
THRUSTER_REVERSED = {
      "H_FL": True,
    # "H_FR": True,
      "H_RL": True,
    # "H_RR": True,
    # "V_FL": True,
      "V_FR": True,
    # "V_RL": True,
      "V_RR": True,
}
CHANNEL_REVERSED = {
    # 8: True,
}

# ---------------------------------------------------------------------------
# 7) Lights control (kept separate from heave)
# ---------------------------------------------------------------------------

LIGHTS_ENABLE = True

# Lights control mode:
#   - "toggle": fixed brightness controlled by a button edge (recommended)
#   - "axis": legacy brightness control using a trigger/axis
LIGHTS_MODE = "toggle"

# Toggle mode: press the LEFT stick (L3) to toggle.
LIGHTS_TOGGLE_BUTTON = "lstick"
LIGHTS_ON_BY_DEFAULT = True
LIGHTS_DEFAULT = 0.75  # 75% brightness

# Axis mode (kept for compatibility if you switch LIGHTS_MODE back to "axis")
LIGHTS_AXIS = "rt"  # keep lights on the trigger, NOT ry
LIGHTS_INVERT = False
LIGHTS_SCALE = 1.0
LIGHTS_DEADZONE = 0.02

LIGHTS_ALLOW_WHEN_DISARMED = True
LIGHTS_FAILSAFE_OFF = False

LIGHTS_US_MIN = 1100
LIGHTS_US_MAX = 1900
LIGHTS_US_OFF = 1100
LIGHTS_TRIM_US = 0

# ---------------------------------------------------------------------------
# 8) Navigator sensors (leave defaults unless you know you need changes)
# ---------------------------------------------------------------------------

NAV_IMU_I2C_BUS = 1
NAV_IMU_SPI_DEVICES = None

NAV_MAG_I2C_BUS = 1
NAV_ENV_I2C_BUS = 1
NAV_ADC_I2C_BUS = 1

LEAK_GPIO_CHIP = "/dev/gpiochip0"
LEAK_GPIO_LINE = None
LEAK_GPIO_INVERT = False

MMC5983_ENABLE = True
MMC5983_USE_SET_RESET = True
MMC5983_I2C_BUSES = (6, 1)
MMC5983_SPI_DEVICES = ((0, 0), (0, 1), (1, 0), (1, 1))

# ---------------------------------------------------------------------------
# Magnetometer fusion (AK09915 + MMC5983)
# ---------------------------------------------------------------------------
# TritonOS can publish a *single* `imu.mag` vector that is a robust blend of
# both onboard magnetometers. This mainly reduces *random* sensor noise and
# provides some outlier rejection when the sensors disagree.
#
# Notes:
#   - This does NOT replace proper hard/soft-iron calibration.
#   - For heading, you still want an AHRS (gyro integration + mag correction).

MAG_FUSION_ENABLE = True


# Which magnetometer to publish as `imu.mag`:
#   "fused" (default): blend AK09915 + MMC5983 for lower random noise.
#   "mmc":   publish MMC5983 only (falls back to AK if MMC is unavailable).
#   "ak":    publish AK09915 only.
#
# Note: Setting MAG_FUSION_PREFER_AK = 0.0 is *almost* equivalent to "mmc"
# when MMC is present, but MAG_OUTPUT_MODE is clearer and sets `mag_source`
# appropriately.
MAG_OUTPUT_MODE = "mmc"

# Relative preference (higher => more weight). Default: trust MMC slightly more.
MAG_FUSION_PREFER_MMC = 1.6
MAG_FUSION_PREFER_AK = 1.0

# Per-sensor statistics:
MAG_FUSION_SENSOR_LPF_ALPHA = 0.20   # 0..1 (larger = faster, less smoothing)
MAG_FUSION_NOISE_EMA_BETA = 0.05     # 0..1 (larger = faster noise tracking)

# Agreement / outlier thresholds:
MAG_FUSION_AGREE_ANGLE_DEG = 15.0
MAG_FUSION_AGREE_NORM_FRAC = 0.12
MAG_FUSION_OUTLIER_ANGLE_DEG = 35.0
MAG_FUSION_OUTLIER_NORM_FRAC = 0.25

# Output smoothing (seconds). Set to 0 to disable.
MAG_FUSION_OUTPUT_LPF_TAU_S = 0.15
# ---------------------------------------------------------------------------
# Attitude estimation (AHRS)
# ---------------------------------------------------------------------------
# TritonOS can publish an additional `type='attitude'` message that contains
# a fused orientation estimate (roll/pitch/yaw + quaternion) computed *on the
# ROV* and streamed topside.
#
# Recommended settings:
#   - Keep ATTITUDE_FUSION='robust' for stable yaw (heading) with mag spike rejection.
#   - Tune ATTITUDE_YAW_TAU larger for smoother yaw, smaller for faster response.
#   - If your heading still jitters, ensure your magnetometers are calibrated
#     (hard/soft iron) and that the IMU is mounted away from high-current wiring.

ATTITUDE_ENABLE = True
ATTITUDE_RATE_HZ = 50.0

# Fusion mode:
#   - 'robust'  : 6DOF Madgwick for roll/pitch + smooth magnetometer yaw correction (recommended)
#   - 'madgwick': classic 6DOF/9DOF Madgwick (more sensitive to mag disturbances)
ATTITUDE_FUSION = 'robust'

# Initial alignment averaging window (seconds)
ATTITUDE_INIT_SECONDS = 2.0

# Madgwick beta scheduling (higher = faster convergence but noisier)
ATTITUDE_BETA = 0.08
ATTITUDE_BETA_INIT = 0.60
ATTITUDE_BETA_STATIONARY = 0.12
ATTITUDE_WARMUP_SECONDS = 1.5

# Accel gating (disable accel correction when vehicle is accelerating hard)
ATTITUDE_ACCEL_G_TOL = 0.20          # allow +/- 0.20g deviation from 1g
ATTITUDE_STATIONARY_GYRO_RAD = 0.20  # rad/s threshold for "stationary"
ATTITUDE_BIAS_ADAPT_TAU = 60.0       # seconds (0 disables stationary gyro bias learning)

# Robust yaw correction tuning
ATTITUDE_YAW_TAU = 8.0               # seconds (larger = smoother yaw)
ATTITUDE_YAW_MAX_ERR_DEG = 25.0      # clamp mag yaw error to reject spikes
ATTITUDE_YAW_KI = 0.02               # 1/s (0 disables Z-gyro bias learning from yaw error)
ATTITUDE_YAW_BIAS_MAX_DPS = 5.0      # clamp learned Z-bias magnitude
ATTITUDE_YAW_BIAS_ADAPT_ERR_DEG = 10.0
ATTITUDE_YAW_BIAS_ADAPT_GYRO_RAD = 0.35
ATTITUDE_YAW_BIAS_ADAPT_GYRO_NORM = 0.50
ATTITUDE_MAG_REF_TAU = 300.0         # seconds (0 disables slow ref tracking)

# Magnetometer health gating (magnitude + step)
ATTITUDE_MAG_TOL = 0.35              # fractional tolerance on |B| relative to baseline
ATTITUDE_MAG_MAX_STEP = 8.0          # uT: max allowed |B| step between samples
ATTITUDE_MAG_ENABLE_UP = 0.75        # seconds to enable mag after it becomes healthy
ATTITUDE_MAG_ENABLE_DOWN = 0.35      # seconds to disable mag after it becomes unhealthy

# Optional sensor filtering (seconds; 0 disables)
ATTITUDE_ACCEL_LPF_TAU_S = 0.05
ATTITUDE_MAG_LPF_TAU_S = 0.20
ATTITUDE_GYRO_LPF_TAU_S = 0.00

# Accel sign handling:
#   - 'auto'   : choose sign that yields smallest initial roll/pitch
#   - 'normal' : use accel as read
#   - 'invert' : flip accel
ATTITUDE_ACCEL_SIGN = 'auto'

# Output zeroing (presentation):
#   - ZERO_ATTITUDE: output is relative to startup attitude (startup becomes 0,0,0)
#   - YAW_ZERO:      subtract an initial yaw reference (operator-friendly)
ATTITUDE_ZERO_ATTITUDE_AT_START = False
ATTITUDE_YAW_ZERO_AT_START = False

# Auto-mount (boot-time leveling)
#
# If the Navigator/Pi is mounted with some *tilt* relative to the vehicle body,
# roll/pitch will be coupled and the attitude stream won't match the robot axes.
#
# When you can guarantee the vehicle boots in a known "level" pose, enabling this
# will compute a tilt-only correction from the averaged accelerometer vector during
# the init window and apply it as an additional mount transform.
#
# Notes:
#   - This fixes the common case where the electronics stack is installed "crooked".
#   - It does NOT automatically determine any yaw-about-Z mounting angle.
#     If your board is also rotated in yaw (not pointing straight ahead), you can
#     either set ATTITUDE_YAW_ZERO_AT_START=True (to make yaw=0 at boot) or use
#     ATTITUDE_AUTO_MOUNT_YAW_DEG below to rotate axes.
ATTITUDE_AUTO_MOUNT_FROM_LEVEL = True

# Optional extra yaw rotation (degrees) applied after leveling. Use this if your
# board is mounted rotated left/right relative to the vehicle forward axis.
ATTITUDE_AUTO_MOUNT_YAW_DEG = 90

# Optional: save the computed mount matrix (JSON) so you can reuse it later.
# Example: '/home/pi/triton_mount.json'
ATTITUDE_AUTO_MOUNT_SAVE_PATH = ''

# Optional calibration files (JSON) produced by tools in triton_ahrs/.
# Leave blank to disable.
ATTITUDE_GYRO_CAL = ''
ATTITUDE_MAG_CAL = ''
ATTITUDE_MOUNT = ''

# Magnetometer input selection for attitude (defaults to MAG_OUTPUT_MODE)
#   'fused' | 'mmc' | 'ak'
ATTITUDE_MAG_OUTPUT_MODE = MAG_OUTPUT_MODE
# Throttle magnetometer reads for the AHRS (Hz). Set <=0 to read at full AHRS rate.
ATTITUDE_MAG_RATE_HZ = 25.0


# ---------------------------------------------------------------------------
# Print config identity + channel map on import (debugging “wrong file” issues)
# ---------------------------------------------------------------------------

if PRINT_CONFIG_ON_IMPORT:
    try:
        print(f"[rov_config] CONFIG_VERSION={CONFIG_VERSION}")
        print(f"[rov_config] loaded from: {__file__}")
        print(f"[rov_config] CONTROL_MIX_MODE={CONTROL_MIX_MODE}")
        print(f"[rov_config] AXES surge={AXIS_SURGE} pitch={AXIS_PITCH} heave={AXIS_HEAVE} sway={AXIS_SWAY} yaw={AXIS_YAW}")
        print(f"[rov_config] THRUSTER_CHANNELS={THRUSTER_CHANNELS}")
        print(f"[rov_config] LIGHTS_PWM_CHANNEL={LIGHTS_PWM_CHANNEL}")
    except Exception:
        pass
