# Architecture Overview

TritonOS is a set of cooperating onboard services started by `main_rov.py`.
Each service owns one runtime concern: control, telemetry, video, management,
or diagnostics. The services share configuration from `rov_config.py` and
communicate with TritonPilot over the tether network.

## System Boundary

TritonOS owns:

- Receiving pilot input.
- Enforcing arming and failsafe behavior.
- Mixing commands into thruster outputs.
- Driving Navigator/PCA9685 PWM channels.
- Reading onboard sensors.
- Publishing telemetry.
- Starting/stopping ROV camera streams.
- Handling runtime config/reference RPC commands.

TritonOS does not own:

- Topside controller UI and operator panels. Those live in TritonPilot.
- Mission-specific computer vision and scoring workflows. Those live in
  TritonAnalysis.
- Long-form documentation generation or competition reporting. Those should use
  this repository's docs as source material, not run on the ROV.

## Runtime Services

`main_rov.py` starts these services:

```text
start_video_service()
  video.gst_streamer_rpc.start_video_rpc()

start_control_service()
  control.pilot_receiver.PilotReceiver
  control.control_service.ControlService
  motion.pwm.ThrustWriter

start_sensor_service()
  sensors.navigator.NavigatorBoard
  sensors.sensor_pub_service.SensorPublisherService
  optional derived processors

start_management_service()
  control.management_rpc.ManagementRpcService
```

Optional network diagnostics are started from `tools.netdiag_server` when
`NETDIAG_ENABLE` is true.

## Control Flow

```text
TritonPilot
  publishes PilotFrame JSON
        |
        v
control.pilot_receiver.PilotReceiver
  parses, validates, computes button edges, stores latest frame
        |
        v
control.control_service.ControlService
  checks freshness, handles arming/failsafe, builds DOF commands
        |
        v
control.autopilot.AutopilotController
  composes depth/attitude hold corrections when enabled
        |
        v
control.mixer.EightThrusterMixer
  converts surge/sway/heave/yaw/pitch/roll into named thruster outputs
        |
        v
motion.pwm.ThrustWriter
  maps named outputs to physical channels, applies limits/deadbands/slew,
  writes Navigator/PCA9685 PWM counts
```

The control loop is deliberately hardware-isolated. Unit tests can validate
pilot parsing, arming behavior, hold controllers, mixing, and payload creation
without a Navigator attached.

## Sensor Flow

```text
sensors.navigator.NavigatorBoard
  owns physical sensor access
        |
        v
BaseSensor wrappers
  IMU, mag, env, ADC, leak, power, external depth, heartbeat
        |
        v
sensors.sensor_pub_service.SensorPublisherService
  polls each sensor at its rate and publishes JSON telemetry
        |
        v
TritonPilot sensor subscriber and logs
```

Derived telemetry can be produced by processors that observe raw readings. The
current important derived streams are:

- `attitude` from `sensors.attitude_estimator.AttitudeEstimatorProcessor`
- `autopilot_status` from `sensors.autopilot_status.AutopilotStatusSensor`
- `power` from `sensors.power_sense.PowerSenseSensor`
- `net` from `sensors.network.NetworkStatsSensor`

## Video Flow

```text
TritonPilot video controls
        |
        v
video.gst_streamer_rpc
  receives ZeroMQ RPC commands
        |
        v
video.gst_streamer.StreamManager
  starts/stops named GStreamer pipelines
        |
        v
camera device -> RTP/UDP or TCP stream -> pilot computer
```

Video control RPC is separate from video payload transport. This makes it
possible for RPC to succeed while payload networking or camera formats still
need debugging.

Stereo camera work uses the same video service. TritonOS can report stream
startup timing through `list_stream_status`, but per-frame pairing happens on
the topside receiver because the current exploreHD cameras do not provide a
hardware trigger or exposure timestamp in this stack.

## Management Flow

```text
tools.management_rpc_client or TritonPilot
        |
        v
control.management_rpc.ManagementRpcService
        |
        v
utils.config_store and utils.vehicle_reference
```

Management RPC provides safe operational hooks:

- Read runtime state.
- Read hold-controller status.
- Update selected config values.
- Set or capture external depth surface pressure reference.
- Request controlled update/restart workflows when implemented.

The contract is documented in
`docs/TOPSIDE_CONFIG_REFERENCE_HANDOFF.md`.

## Configuration Model

`rov_config.py` is the operator-tunable source of truth. It is plain Python so
the runtime can import it directly and the management RPC can edit selected
uppercase constants.

Important rules:

- Edit `CHANNEL_MAP` for physical wiring.
- Treat generated aliases such as `THRUSTER_CHANNELS` as derived values.
- Restart the service after changing most config values.
- Use management RPC only for values that are explicitly safe to change at
  runtime.

See [Configuration Guide](CONFIGURATION.md).

## Safety Model

The control service protects the hardware path with several gates:

- TritonOS starts disarmed.
- Pilot frames must be fresh.
- Arming can require neutral sticks and triggers.
- Failsafe disarms can occur when pilot input goes stale.
- Thruster outputs are neutral while disarmed.
- PWM outputs can be physically disabled on disarm.
- Slew limiting can soften command changes.

Hardware diagnostic scripts can bypass parts of this stack. They are useful,
but they must be treated as direct hardware tools.

## Testability

The unit tests exercise logic that does not require hardware:

- Control service payload behavior.
- Mixers.
- Depth hold and autopilot controllers.
- Attitude estimator.
- Channel map validation.
- Management RPC request handling.
- Sensor wrappers that can be simulated.

Run:

```bash
python -m pytest
```

Hardware tests are intentionally separate under `tools/` and `bin/`.
