# Configuration Guide

TritonOS runtime configuration lives in `rov_config.py`. It is plain Python so
startup remains simple and selected values can be edited by management tooling.

## Editing Rules

1. Prefer changing `rov_config.py` over hard-coding values in modules.
2. Restart `tritonos-rov.service` after most edits.
3. Edit `CHANNEL_MAP` for physical wiring. Do not edit derived aliases.
4. Keep safety limits conservative until the vehicle has been tested in water.
5. Treat schema or endpoint changes as cross-repository changes because
   TritonPilot must agree with them.

Restart after editing:

```bash
sudo systemctl restart tritonos-rov.service
```

Follow logs:

```bash
sudo journalctl -u tritonos-rov.service -f
```

## Network Endpoints

Important defaults:

```python
PILOT_SUB_ENDPOINT = "tcp://0.0.0.0:6000"
SENSOR_PUB_ENDPOINT = "tcp://0.0.0.0:6001"
VIDEO_RPC_ENDPOINT = "tcp://0.0.0.0:5555"
MANAGEMENT_RPC_ENDPOINT = "tcp://0.0.0.0:5556"
```

The ROV binds to `0.0.0.0`; TritonPilot connects to the ROV's actual tether IP,
normally `192.168.1.4`.

## Channel Map

`CHANNEL_MAP` is the single source of truth for physical PWM wiring:

```python
CHANNEL_MAP = {
    "thrusters": {
        "H_FL": 12,
        "H_FR": 2,
        "H_RL": 3,
        "H_RR": 14,
        "V_FL": 13,
        "V_FR": 1,
        "V_RL": 4,
        "V_RR": 15,
    },
    "aux": {
        "lights": 5,
        "wrist_rotate": 16,
        "gripper_left": 10,
        "gripper_right": 11,
    },
}
```

Thruster names are logical vehicle positions. Values are physical Navigator PWM
channels `1..16`.

Derived aliases are created after `CHANNEL_MAP`:

```python
THRUSTER_CHANNELS = dict(CHANNEL_MAP["thrusters"])
AUX_PWM_CHANNELS = dict(CHANNEL_MAP.get("aux", {}))
MOTOR_PWM_CHANNELS = sorted(THRUSTER_CHANNELS.values())
```

Do not edit these aliases by hand.

Check the active map:

```bash
python -m tools.print_channel_map
```

## Thruster Direction

Use `THRUSTER_REVERSED` for named direction flips:

```python
THRUSTER_REVERSED = {
    "H_FL": True,
    "H_RL": True,
    "V_FR": True,
}
```

Prefer named keys over raw channel numbers. Named keys survive wiring changes
better.

## Control Loop Settings

Core values:

```python
CONTROL_RATE_HZ = 50.0
PILOT_TTL = 0.5
CONTROL_MIX_MODE = "simple_groups"
POWER_SCALE = 1.0
THRUSTER_MAX_ABS = 1.0
MIX_OUTPUT_DEADBAND = 0.05
```

Use `CONTROL_MIX_MODE = "simple_groups"` for bring-up. In this mode surge drives
only the horizontal thruster group and heave drives only the vertical thruster
group. Normal operation should use `six_dof` after logical channel names and
signs are proven.

Pilot axis mapping:

```python
AXIS_SURGE = "ly"
AXIS_SWAY = "lx"
AXIS_HEAVE = "ry"
AXIS_YAW = "rx"
AXIS_PITCH = "dpad_y"
AXIS_ROLL = "dpad_x"
```

Set an axis to `"none"` if it should be disabled.

Manipulator runtime gains:

```python
WRIST_ROTATE_SPEED = 0.50
GRIPPER_PITCH_SCALE = 0.5
GRIPPER_YAW_SCALE = 1.0
```

TritonPilot sends `PilotFrame.modes["back_gripper_gain"]` and
`PilotFrame.modes["arm_gain"]` at runtime. TritonOS clamps those values and
applies them on top of the fixed wrist/arm calibration values above.

## Arming Safety

Important safety gates:

```python
ARM_REQUIRE_NEUTRAL = True
ARM_CENTER_TOL = 0.18
ARM_TRIGGER_TOL = 0.10
ARM_RAMP_S = 0.35
PILOT_TTL = 0.5
```

Keep `ARM_REQUIRE_NEUTRAL` enabled for normal operation. It protects against
bad axis mappings and non-centered controller input at arm time.

## PWM Backend

TritonOS can use Navigator bindings or direct PCA9685 access:

```python
PWM_BACKEND = "auto"
PWM_AUTO_ENABLE = False
HARDWARE_ARM_DISARM = True
DISABLE_PWM_ON_DISARM = True
```

Backend choices:

- `"auto"` - prefer Blue Robotics Navigator bindings, fall back to direct
  PCA9685.
- `"navigator"` - require Navigator bindings.
- `"direct"` - bypass Navigator bindings and use direct I2C PCA9685 access.

Normal competition operation should keep outputs disabled until arming:

```python
PWM_AUTO_ENABLE = False
HARDWARE_ARM_DISARM = True
DISABLE_PWM_ON_DISARM = True
```

## Depth Sensor And Depth Hold

External depth settings:

```python
USE_EXTERNAL_DEPTH = True
EXTERNAL_DEPTH_I2C_BUSES = (6, 1)
EXTERNAL_DEPTH_MODEL = "auto"
EXTERNAL_DEPTH_RATE_HZ = 10.0
EXTERNAL_DEPTH_REFERENCE_PATH = "calibration/depth_reference.json"
EXTERNAL_DEPTH_SENSOR_TO_TOP_M = 0.15
```

Capture surface pressure:

```bash
python -m tools.set_vehicle_reference --pressure-samples 20
```

Depth-hold controller settings:

```python
DEPTH_HOLD_ENABLE = True
DEPTH_HOLD_SENSOR_STALE_S = 2.0
DEPTH_HOLD_KP = 0.55
DEPTH_HOLD_KI = 0.06
DEPTH_HOLD_KD = 0.08
DEPTH_HOLD_OUT_LIMIT = 0.45
DEPTH_HOLD_WALK_TARGET = False
```

With `DEPTH_HOLD_WALK_TARGET = False`, manual vertical stick input passes
through normally while depth hold is armed, and the controller latches the
current depth as the target when the stick returns to center. Set it to `True`
only if you want the older behavior where stick input walks the setpoint.

Tune these slowly. If depth hold drives in the wrong direction, verify sensor
sign and then use:

```python
DEPTH_HOLD_SIGN = -1.0
```

## Attitude Estimator And Autopilot

The onboard attitude estimator publishes relative attitude from the local IMU
and magnetometer stream:

```python
ATTITUDE_ESTIMATOR_ENABLE = True
ATTITUDE_CALIBRATION_SAMPLES = 30
ATTITUDE_VEHICLE_ROLL_AXIS = "z"
ATTITUDE_YAW_MAG_SOURCE = "auto"
```

Autopilot coordination:

```python
AUTOPILOT_ENABLE = True
AUTOPILOT_ATTITUDE_ENABLE = True
AUTOPILOT_ATTITUDE_STALE_S = 0.50
AUTOPILOT_STATUS_ENABLE = True
AUTOPILOT_YAW_MANUAL_LATCH = True
```

Roll, pitch, and yaw hold defaults are intentionally conservative. Leave their
default modes `"off"` until tuning and validation are complete.

With `AUTOPILOT_YAW_MANUAL_LATCH = True`, manual yaw input turns the ROV
normally while yaw hold is armed, and the controller latches the current yaw as
the target when the stick returns to center.

## Power, Network, And Diagnostics

Power sense:

```python
POWER_SENSE_ENABLE = True
POWER_SENSE_VOLT_CH = 3
POWER_SENSE_CURR_CH = 2
POWER_SENSE_VOLT_MULT = 11.0
POWER_SENSE_AMPS_PER_VOLT = 37.8788
```

Network stats:

```python
NET_STATS_ENABLE = True
NET_STATS_RATE_HZ = 1.0
```

Network diagnostics server:

```python
NETDIAG_ENABLE = True
NETDIAG_PORT = 7700
```

Debug logs:

```python
DEBUG = True
CONTROL_DEBUG = False
PILOT_RX_DEBUG = False
PRINT_CONFIG_ON_IMPORT = True
```

Turn verbose control logs on only while diagnosing behavior.

## Video Quality

TritonPilot sends per-stream video targets such as `h264_bitrate`,
`h264_gop`, and `rtp_mtu` from its `data/streams.json`. For native H.264 UVC
cameras, TritonOS applies the bitrate/GOP values as V4L2 controls when the
camera exposes common controls such as `video_bitrate` and
`h264_i_frame_period`. Unsupported controls are skipped so video startup still
succeeds with camera defaults. Explicit `v4l2_controls` values may include `0`,
which is useful for controls such as `exposure_dynamic_framerate=0`.

The current pilot-side default for four 1080p30 streams is 8 Mbps per stream
with a 1200-byte RTP MTU. Sender-side leaky queues are enabled by default so
the Pi drops stale whole frames before RTP packetization instead of building
latency. Per-stream `extra` options can override this with
`sender_leaky_queues`, `sender_queue_max_buffers`, `sender_queue_max_time_ms`,
and `sender_v4l2_do_timestamp`.

## Management RPC

Use the CLI client:

```bash
python -m tools.management_rpc_client --endpoint tcp://127.0.0.1:5556 get-state
```

Set a safe config value:

```bash
python -m tools.management_rpc_client \
  --endpoint tcp://127.0.0.1:5556 \
  set-config '{"DEPTH_HOLD_KP": 0.55}'
```

Capture surface pressure through RPC:

```bash
python -m tools.management_rpc_client \
  --endpoint tcp://127.0.0.1:5556 \
  capture-surface --samples 20 --delay-s 0.02
```

The management RPC contract is summarized in
`docs/TOPSIDE_CONFIG_REFERENCE_HANDOFF.md`.
