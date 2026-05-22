# rov_config.py
"""Runtime configuration for the onboard ROV process.

This file is intentionally plain Python because the vehicle needs to read it
without a separate config parser, and the management RPC can safely edit a
limited set of top-level uppercase constants. Treat it as the operator-tunable
source of truth for endpoints, control gains, sensor choices, PWM channel
mapping, and safety limits.

Most subsystems import these values at startup. Settings that are changed while
the ROV is running usually require a service restart unless the management RPC
or the owning service explicitly documents live reload behavior.
"""
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
MANAGEMENT_RPC_ENABLE = True
MANAGEMENT_RPC_ENDPOINT = "tcp://0.0.0.0:5556"  # ROV config/reference management RPC

# ---------------------------------------------------------------------------
# 2) control loop
# ---------------------------------------------------------------------------

CONTROL_RATE_HZ = 50.0
PILOT_TTL = 0.5  # seconds before we consider pilot stale

# Control mixing mode:
#   - "simple_groups": bring-up mode (surge drives all horizontals, heave drives all verticals)
#   - "six_dof": full mixer (surge/sway/yaw on horizontals; heave/pitch/roll on verticals)
#   - "geometric": physical-geometry least-squares mixer; set back to "six_dof" if testing is poor
CONTROL_MIX_MODE = "geometric"

# Geometric mixer model.
# Coordinates are vehicle-relative, measured from the approximate frame center:
#   +x forward, +y right, +z down.
#
# Positions below are derived from motoref.md:
#   frame 20.5 in long, 16.5 in wide, 11 in tall.
#   front/back and left/right values use the listed insets from frame walls.
#   height is listed as distance down from the top of frame, then converted to
#   a centered z coordinate.
#
# The horizontal motors were described as "60 deg to bias surge". We interpret
# that as 60 deg from the lateral/sway axis, i.e. 30 deg off the forward axis.
# If the physical convention is the opposite, change this to 60.0 and retest.
GEOMETRIC_MIXER_HORIZONTAL_ANGLE_DEG_FROM_FORWARD = 30.0
GEOMETRIC_MIXER_REGULARIZATION = 0.015
GEOMETRIC_MIXER_AUTO_SCALE_UNIT_AXES = True
GEOMETRIC_MIXER_AXIS_COMMAND_SCALES = {
    # Leave empty for automatic scaling. Add per-axis overrides here if a pool
    # test shows one axis should intentionally be softer or stronger.
}
GEOMETRIC_MIXER_AXIS_WEIGHTS = {
    "surge": 1.0,
    "sway": 1.0,
    "heave": 1.0,
    "roll": 1.0,
    "pitch": 1.0,
    "yaw": 1.0,
}
THRUSTER_GEOMETRY = {
    "H_FL": {"position_m": (0.2032, -0.12065, -0.0381), "direction": "auto", "scale": 1.0},
    "H_FR": {"position_m": (0.2032, 0.12065, -0.0381), "direction": "auto", "scale": 1.0},
    "H_RL": {"position_m": (-0.2032, -0.12065, -0.0381), "direction": "auto", "scale": 1.0},
    "H_RR": {"position_m": (-0.2032, 0.12065, -0.0381), "direction": "auto", "scale": 1.0},
    "V_FL": {"position_m": (0.08255, -0.1524, -0.1397), "direction": "auto", "scale": 1.0},
    "V_FR": {"position_m": (0.08255, 0.1524, -0.1397), "direction": "auto", "scale": 1.0},
    "V_RL": {"position_m": (-0.1016, -0.1524, -0.1397), "direction": "auto", "scale": 1.0},
    "V_RR": {"position_m": (-0.1016, 0.1524, -0.1397), "direction": "auto", "scale": 1.0},
}

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
DEPTH_HOLD_MIX_DEADBAND = 0.01

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
# Keep enough smoothing to avoid pressure-sensor chatter without adding much lag.
DEPTH_HOLD_LPF_TAU_S = 0.30

# PI(D) gains (heave-command per meter / meter-second)
DEPTH_HOLD_KP = 0.55
DEPTH_HOLD_KI = 0.06
DEPTH_HOLD_KD = 0.08

# Error deadband in meters (reduces thruster chatter near setpoint)
DEPTH_HOLD_ERROR_DEADBAND_M = 0.010

# Integrator clamp (in heave-command units)
DEPTH_HOLD_I_LIMIT = 0.15

# Output clamp (in heave command units; keep < 1.0 while tuning)
DEPTH_HOLD_OUT_LIMIT = 0.45

# If the controller pushes the wrong way, flip this to -1.0.
DEPTH_HOLD_SIGN = 1.0

# "Walk target" behavior: stick commands move the target depth; releasing holds.
DEPTH_HOLD_WALK_TARGET = True
DEPTH_HOLD_WALK_DEADBAND = 0.10
DEPTH_HOLD_WALK_RATE_MPS = 0.45  # full stick => ~0.45 m/s target change

# Optional clamp on target depth (meters, depth positive down). Set to None to disable.
DEPTH_HOLD_TARGET_MIN_M = None
DEPTH_HOLD_TARGET_MAX_M = None

# ---------------------------------------------------------------------------
# 2c.5) autopilot coordination
# ---------------------------------------------------------------------------
# The autopilot owns the combined depth/attitude hold path. Depth hold still
# uses the DEPTH_HOLD_* gains above and the legacy PilotFrame.modes["depth_hold"]
# command, but it now runs through the same coordinator as attitude hold so the
# vertical-thruster commands are composed before final mixing.
AUTOPILOT_ENABLE = True
AUTOPILOT_ATTITUDE_ENABLE = True
AUTOPILOT_ATTITUDE_STALE_S = 0.50
AUTOPILOT_MIX_DEADBAND = 0.02
AUTOPILOT_STATUS_ENABLE = True
AUTOPILOT_STATUS_RATE_HZ = 20.0

# First attitude-hold modes to tune: roll/pitch level and yaw hold.
# Keep these deliberately conservative while bench/water testing.
AUTOPILOT_ROLL_MODE_DEFAULT = "off"
AUTOPILOT_ROLL_KP = 0.012
AUTOPILOT_ROLL_KI = 0.0
AUTOPILOT_ROLL_KD = 0.002
AUTOPILOT_ROLL_ERROR_DEADBAND_DEG = 0.5
AUTOPILOT_ROLL_I_LIMIT = 0.10
AUTOPILOT_ROLL_OUT_LIMIT = 0.16
AUTOPILOT_ROLL_SIGN = 1.0
AUTOPILOT_ROLL_MANUAL_DEADBAND = 0.08
AUTOPILOT_ROLL_WALK_RATE_DPS = 35.0

AUTOPILOT_PITCH_MODE_DEFAULT = "off"
AUTOPILOT_PITCH_KP = 0.012
AUTOPILOT_PITCH_KI = 0.0
AUTOPILOT_PITCH_KD = 0.002
AUTOPILOT_PITCH_ERROR_DEADBAND_DEG = 0.5
AUTOPILOT_PITCH_I_LIMIT = 0.10
AUTOPILOT_PITCH_OUT_LIMIT = 0.16
AUTOPILOT_PITCH_SIGN = 1.0
AUTOPILOT_PITCH_MANUAL_DEADBAND = 0.08
AUTOPILOT_PITCH_WALK_RATE_DPS = 35.0

AUTOPILOT_YAW_MODE_DEFAULT = "off"
AUTOPILOT_YAW_KP = 0.009
AUTOPILOT_YAW_KI = 0.0008
AUTOPILOT_YAW_KD = 0.0015
AUTOPILOT_YAW_ERROR_DEADBAND_DEG = 1.0
AUTOPILOT_YAW_I_LIMIT = 0.05
AUTOPILOT_YAW_OUT_LIMIT = 0.14
# Positive yaw-hold error must command the vehicle back toward increasing yaw.
# The current horizontal thruster/mixer convention needs the inverted sign here.
AUTOPILOT_YAW_SIGN = -1.0
AUTOPILOT_YAW_MANUAL_DEADBAND = 0.08
AUTOPILOT_YAW_WALK_RATE_DPS = 35.0

# ---------------------------------------------------------------------------
# 2d) arming safety
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

# Persisted surface-pressure reference captured with tools/set_vehicle_reference.py.
# If the file exists, TritonOS will use that pressure instead of assuming boot
# happened at the surface. Leave the fixed override as None in normal use.
EXTERNAL_DEPTH_REFERENCE_PATH = "calibration/depth_reference.json"
EXTERNAL_DEPTH_FIXED_SURFACE_PRESSURE_MBAR = None

# Report depth from the *top* of the ROV instead of from the pressure sensor
# itself. Positive means the sensor sits that far below the top reference point.
EXTERNAL_DEPTH_SENSOR_TO_TOP_M = 0.15

# Publish external depth faster than the fallback default so depth-hold sees new
# pressure samples sooner and does not feel "sticky" after a depth change.
EXTERNAL_DEPTH_RATE_HZ = 10.0

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
# Keep these low enough that depth-hold can make small vertical corrections near
# setpoint. The control loop still applies its own higher mix deadbands for
# manual driving, so reducing the PWM deadband mainly helps closed-loop trim.
PWM_DEADBAND = 0.03     # normalized thrust deadband
PWM_DEADBAND_US = 12    # microsecond deadband around neutral

# Thruster slew limiting (software band-aid for power spikes / brownouts)
# Units are normalized thrust per second (1.0 == full-scale change in ~1s).
# At 50 Hz, 3.0 => ~0.06 max change per control tick.
THRUSTER_SLEW_RATE_NORM_PER_S = 15
# Optional slower limit when reversing direction across neutral (None -> use base rate).
THRUSTER_SLEW_REVERSE_RATE_NORM_PER_S = 15
# Cap dt used by the slew limiter so a delayed loop does not instantly jump.
THRUSTER_SLEW_DT_MAX_S = 0.10

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
# PWM backend:
#   - "auto"      : prefer bluerobotics_navigator, fall back to direct PCA9685 I2C
#   - "navigator" : require bluerobotics_navigator
#   - "direct"    : bypass bluerobotics_navigator entirely
PWM_BACKEND = "auto"
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

# Direct PCA9685 fallback settings (used by PWM_BACKEND="direct" or auto fallback)
PWM_DIRECT_I2C_BUS = 4
PWM_DIRECT_I2C_ADDR = 0x40
PWM_DIRECT_OSC_HZ = 25_000_000.0
PWM_DIRECT_OE_GPIO = 26
PWM_DIRECT_OE_ACTIVE_LOW = True

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
"""
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
        "lights": 5,         # physical channel 5
        "wrist_rotate": 10,  # physical channel 10 (T200 wrist rotate motor)
        "gripper_left": 12,  # physical channel 12 (differential servo)
        "gripper_right": 13, # physical channel 13 (differential servo)
    },
}
"""


CHANNEL_MAP = {
    "thrusters": {
        # Horizontals (surge/sway/yaw) — should be motors 7,5,1,6
        "H_FL": 12,  # motor7 -> physical channel 8
        "H_FR": 2,  # motor5 -> physical channel 6
        "H_RL": 3,  # motor6 -> physical channel 7
        "H_RR": 14,  # motor1 -> physical channel 2

        # Verticals (heave/pitch/roll)
        # Your wiring (physical Navigator channels):
        #   V_FL (front-left)  = 3
        #   V_FR (front-right) = 4
        #   V_RL (rear-left)   = 9
        #   V_RR (rear-right)  = 1
        "V_FL": 13,
        "V_FR": 1,
        "V_RL": 4,
        "V_RR": 15,
    },
    "aux": {
        "lights": 5,         # physical channel 5
        "wrist_rotate": 16,  # physical channel 10 (T200 wrist rotate motor)
        "gripper_left": 10,  # physical channel 12 (differential servo)
        "gripper_right": 11, # physical channel 13 (differential servo)
    },
}

# ---- Derived aliases (do not edit) -----------------------------------------

THRUSTER_CHANNELS = dict(CHANNEL_MAP["thrusters"])
AUX_PWM_CHANNELS = dict(CHANNEL_MAP.get("aux", {}))
LIGHTS_PWM_CHANNEL = AUX_PWM_CHANNELS.get("lights")
WRIST_ROTATE_PWM_CHANNEL = AUX_PWM_CHANNELS.get("wrist_rotate")
GRIPPER_LEFT_PWM_CHANNEL = AUX_PWM_CHANNELS.get("gripper_left")
GRIPPER_RIGHT_PWM_CHANNEL = AUX_PWM_CHANNELS.get("gripper_right")
MOTOR_PWM_CHANNELS = sorted(THRUSTER_CHANNELS.values())

# Optional per-thruster direction flips.
# Keys should be thruster names (preferred) or raw channel numbers.
"""
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
"""

THRUSTER_REVERSED = {
      "H_FL": True,
    # "H_FR": True,
      "H_RL": True,
    # "H_RR": True,
    # "V_FL": True,
      "V_FR": True,
      "V_RL": True,
      #"V_RR": True,
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

# Toggle mode: topside maps the pilot keyboard `L` key onto this synthetic edge.
LIGHTS_TOGGLE_BUTTON = "lights"
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
# 8) Wrist rotate (T200 on aux channel, driven like a thruster)
# ---------------------------------------------------------------------------
# Uses the controller triggers for a proportional wrist rotation command:
#   - RT = rotate right
#   - LT = rotate left
# The output is sent as a *thruster-style* command (normalized [-1..1]) to the
# channel configured in CHANNEL_MAP["aux"]["wrist_rotate"].

WRIST_ROTATE_ENABLE = True
WRIST_ROTATE_CMD_KEY = "wrist_rotate"

# 9) Differential-servo gripper head (pitch/yaw servos on channels 11/12)
# Topside sends keyboard-derived normalized commands in PilotFrame.aux:
#   W/S -> gripper_pitch in [-1..1]
#   A/D -> gripper_yaw   in [-1..1]
# The ROV mixes those into two signed servo outputs:
#   left  = pitch + yaw
#   right = pitch - yaw
# Then main_rov configures those outputs as bidirectional servos centered at 1500 us.
GRIPPER_ENABLE = True
GRIPPER_PITCH_CMD_KEY = "gripper_pitch"
GRIPPER_YAW_CMD_KEY = "gripper_yaw"
GRIPPER_LEFT_CMD_KEY = "gripper_left"
GRIPPER_RIGHT_CMD_KEY = "gripper_right"
GRIPPER_PITCH_SCALE = 0.5
GRIPPER_YAW_SCALE = 1.0
GRIPPER_PITCH_INVERT = 1.0
GRIPPER_YAW_INVERT = 1.0
GRIPPER_DEADBAND = 0.01
GRIPPER_HOLD_LAST_POSITION = True
GRIPPER_SERVO_MIN_US = 500
GRIPPER_SERVO_MAX_US = 2500
GRIPPER_SERVO_CENTER_US = 1500
GRIPPER_ALLOW_WHEN_DISARMED = False
GRIPPER_CENTER_ON_DISARM = True
# Keep the differential wrist servos powered on disarm so the arm stays folded in.
GRIPPER_HOLD_PWM_ON_DISARM = False # if False, the servos will be unpowered on disarm (arm will go limp)
# Explicitly command the folded pose when arming and right before disarming.
GRIPPER_PARK_ON_ARM_DISARM = True
GRIPPER_PARK_SETTLE_S = 0.50
# Park the differential wrist on transitions. Pitch is preserved first; yaw is
# limited if the requested pitch+yaw pose exceeds the two-servo range.
GRIPPER_DISARM_PITCH = 0.20
GRIPPER_DISARM_YAW = -1.0
# On arm, default back to the same tucked pose instead of reviving the last live command.
GRIPPER_ARM_PITCH = GRIPPER_DISARM_PITCH
GRIPPER_ARM_YAW = GRIPPER_DISARM_YAW

WRIST_ROTATE_RIGHT_AXIS = "rt"
WRIST_ROTATE_LEFT_AXIS = "lt"
WRIST_ROTATE_TRIGGER_DEADZONE = 0.10
WRIST_ROTATE_SPEED = 0.50  # max normalized command at full trigger

# ---------------------------------------------------------------------------
# 9) Navigator sensors (leave defaults unless you know you need changes)
# ---------------------------------------------------------------------------

NAV_IMU_I2C_BUS = 1
NAV_IMU_SPI_DEVICES = None
IMU_RATE_HZ = 20.0

NAV_MAG_I2C_BUS = 1
MAG_RATE_HZ = 5.0
NAV_ENV_I2C_BUS = 1
NAV_ADC_I2C_BUS = 1

LEAK_GPIO_CHIP = "/dev/gpiochip0"
LEAK_GPIO_LINE = None
LEAK_GPIO_INVERT = False

MMC5983_ENABLE = True
MMC5983_USE_SET_RESET = True
MMC5983_I2C_BUSES = (6, 1)
MMC5983_SPI_DEVICES = ((0, 0), (0, 1), (1, 0), (1, 1))

# IMU telemetry publishes accel/gyro at IMU_RATE_HZ. Magnetometers publish as
# a separate raw mag stream at MAG_RATE_HZ so they can be visualized without
# slowing down the accel/gyro cadence.

# Onboard attitude estimator.
# This publishes type='attitude' telemetry derived from the local IMU/mag stream.
# It is diagnostic-only for now; future attitude hold should consume this onboard
# estimate directly instead of sending raw IMU data to topside first.
ATTITUDE_ESTIMATOR_ENABLE = True
ATTITUDE_CALIBRATION_SAMPLES = 30
ATTITUDE_MAX_DT_S = 0.25
ATTITUDE_ACCEL_TAU_S = 0.16
ATTITUDE_ACCEL_FAST_TAU_S = 0.055
ATTITUDE_ACCEL_FAST_ERROR_DEG = 3.0
ATTITUDE_ACCEL_MIN_WEIGHT = 0.02
ATTITUDE_ACCEL_MAX_WEIGHT = 0.90
ATTITUDE_ACCEL_NORM_GATE = 0.18
ATTITUDE_CALIBRATION_MAX_TILT_STD_DEG = 1.25
ATTITUDE_CALIBRATION_MAX_GYRO_RMS_DPS = 3.0
# Navigator mount/body-frame convention on this ROV: at rest gravity is mostly
# on sensor -Y, so the usable horizontal body axes are sensor X and Z. Sensor Z
# is the vehicle forward/roll axis, which makes bow pitch appear as pitch and
# side roll appear as roll.
ATTITUDE_VEHICLE_ROLL_AXIS = "z"
ATTITUDE_ROLL_SIGN = 1.0
ATTITUDE_PITCH_SIGN = 1.0
ATTITUDE_YAW_MAG_SOURCE = "auto"  # auto prefers MMC5983 when available/clean
ATTITUDE_YAW_TAU_S = 0.45
ATTITUDE_YAW_MIN_WEIGHT = 0.02
ATTITUDE_YAW_MAX_WEIGHT = 0.65
ATTITUDE_YAW_MAX_MAG_AGE_S = 0.75
ATTITUDE_YAW_MAG_NORM_GATE = 0.45
ATTITUDE_YAW_MAG_SMOOTH_TAU_S = 0.65
ATTITUDE_YAW_REFERENCE_SAMPLES = 30
ATTITUDE_STATIONARY_BIAS_ENABLE = True
ATTITUDE_STATIONARY_BIAS_TAU_S = 15.0
ATTITUDE_STATIONARY_GYRO_MAX_DPS = 1.0
ATTITUDE_STATIONARY_ACCEL_ERROR_MAX_DEG = 1.5
ATTITUDE_STATIONARY_ACCEL_NORM_ERROR_MAX = 0.05


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
