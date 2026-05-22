# Subsystem Reference

This document explains the major pieces of TritonOS at the code level. Use it
when you need to understand where behavior lives before making a change.

## Entry Point

### `main_rov.py`

The normal onboard entry point. It starts video RPC, control, sensors, and
management RPC. Startup is defensive: optional hardware or service failures are
reported, and the rest of the process continues when safe.

Key functions:

- `start_video_service()` - runs video RPC in a daemon thread.
- `start_control_service()` - builds pilot receiver, control service, hardware
  PWM sink, arming state, and control gains.
- `start_sensor_service()` - builds physical sensors, derived processors, and
  starts the sensor publisher.
- `start_management_service()` - starts management RPC when enabled.
- `main()` - service supervisor loop.

## Configuration

### `rov_config.py`

Plain-Python runtime configuration. This file defines endpoints, control gains,
sensor settings, PWM behavior, channel maps, and diagnostics flags.

Most values are read at startup. Restart `tritonos-rov.service` after editing
unless a specific management RPC command documents live behavior.

## Control Package

### `control/pilot_receiver.py`

ROV-side ZeroMQ SUB receiver for pilot frames. It validates incoming JSON,
keeps only the latest frame, tracks link health counters, and computes local
button edges.

Used by: `ControlService`

### `control/control_service.py`

The main control-loop orchestrator. It:

- Pulls the latest pilot frame.
- Enforces freshness and arming behavior.
- Applies pilot gain caps.
- Builds manual 6-DOF or simple-group commands.
- Applies depth and attitude hold corrections.
- Mixes commands into named thruster outputs.
- Applies lights, wrist rotate, and gripper aux commands.
- Sends the final payload to the hardware sink.
- Records status snapshots for telemetry and management RPC.

### `control/mixer.py`

Named-thruster mixers. The primary mixer is `EightThrusterMixer`, which maps:

- `surge`, `sway`, `yaw` to horizontal thrusters.
- `heave`, `pitch`, `roll` to vertical thrusters.

It outputs logical names like `H_FL` and `V_RR`, not physical channels.

### `control/depth_hold.py`

Sticky depth-hold controller. It captures a target depth when enabled and can
"walk" the target with manual heave input. It uses external-depth telemetry and
outputs a normalized heave correction.

### `control/autopilot.py`

Coordinator for depth hold and attitude hold. It composes hold corrections
before final thruster mixing and falls back to manual commands when telemetry is
missing, stale, or not ready.

### `control/sensor_tap.py`

Local subscriber for sensor telemetry used by onboard hold controllers. It lets
the control loop observe depth and attitude messages without routing them
through TritonPilot.

### `control/management_rpc.py`

ZeroMQ REP service for runtime management. It exposes state snapshots,
hold-status inspection, selected config edits, and surface-pressure reference
commands.

## Motion Package

### `motion/channel_map.py`

Validates the physical PWM channel map. This is the bridge between logical
thruster names and Navigator physical channels.

Important convention:

- `CHANNEL_MAP` in `rov_config.py` uses physical channels `1..16`.
- The rest of the control stack should use thruster names.

### `motion/pwm.py`

PWM backend and high-level `ThrustWriter`.

Responsibilities:

- Select Navigator binding backend or direct PCA9685 backend.
- Convert normalized thrust `[-1.0, 1.0]` to microsecond pulses.
- Convert pulses to PCA9685 counts.
- Apply deadband, trim, reversal, slew limiting, and arming behavior.
- Handle auxiliary PWM outputs such as lights and gripper servos.

## Sensors Package

### `sensors/base.py`

Defines `BaseSensor`, the small polling abstraction used by the sensor
publisher.

### `sensors/navigator.py`

High-level access to Navigator-connected sensors. It prefers direct sensor
drivers to avoid Navigator binding conflicts with PWM resources, but can use
the official bindings if `TRITON_USE_NAV_BINDINGS=1`.

It also defines sensor wrappers for IMU, magnetometer, environment, leak, ADC,
and external depth telemetry.

### Hardware drivers

- `sensors/icm20602.py` - minimal I2C IMU driver.
- `sensors/icm20602_auto.py` - IMU driver with I2C/SPI auto-detection.
- `sensors/ak09915.py` - primary magnetometer driver.
- `sensors/mmc5983.py` - optional magnetometer driver.
- `sensors/bmp280.py` - barometer/temperature driver.
- `sensors/ads1115.py` - ADC driver.
- `sensors/ms5837.py` - Blue Robotics Bar02/Bar30 pressure sensor driver.
- `sensors/leak_gpio.py` - GPIO leak detector helper.

### Runtime telemetry modules

- `sensors/sensor_pub_service.py` - polls sensors and publishes JSON telemetry.
- `sensors/attitude_estimator.py` - derives relative attitude from IMU/mag.
- `sensors/autopilot_status.py` - publishes control/hold state.
- `sensors/power_sense.py` - converts ADC readings into voltage/current.
- `sensors/network.py` - publishes tether/network stats.
- `sensors/heartbeat.py` - publishes process liveness and coarse state.

## Video Package

### `video/gst_streamer.py`

Object-oriented GStreamer wrapper. It builds camera pipelines from
`StreamConfig`, manages lifecycle, handles live updates where possible, and
tracks bus errors.

### `video/gst_streamer_rpc.py`

ZeroMQ RPC service for video operations. It supports stream start/stop/update,
camera discovery, V4L2 capability probing, and best-effort USB rebind/reset
recovery for flaky camera enumeration.

### `video/tether.py`

Linux helpers for selecting a tether interface and optionally pinning routes to
the pilot video receiver.

## Schema Package

### `schema/pilot_common.py`

Shared pilot wire schema. `PilotFrame` is the JSON payload sent by TritonPilot
and consumed by TritonOS. Treat changes here as protocol changes and keep the
topside copy in sync.

## Utils Package

### `utils/config_store.py`

Loads, reloads, and updates selected runtime config values for management RPC.

### `utils/vehicle_reference.py`

Loads and saves vehicle reference data, especially external-depth surface
pressure references.

### `utils/navigator_import.py`

Imports Blue Robotics Navigator bindings across package layout variants and
reports API capability information.

### `utils/zmq_monitor.py`

Small helper for monitoring ZeroMQ socket connection events.

## Tools

The `tools/` directory contains operator and developer utilities. These should
not be imported into normal runtime paths unless specifically designed for that
purpose.

Common tools:

- `tools/rov_preflight.py` - broad hardware/software sanity report.
- `tools/print_channel_map.py` - show validated physical channel map.
- `tools/management_rpc_client.py` - call management RPC manually.
- `tools/sensor_stream_pub_test.py` - publish real or fake telemetry.
- `tools/native_motor_test.py` - direct Navigator single-channel PWM test.
- `tools/direct_i2c_pwm_test.py` - direct PCA9685 test without Navigator
  bindings.
- `tools/thruster_test.py` - sequential configured-thruster test.
- `tools/thruster_identity_test.py` - confirm named thruster to PWM mapping.
- `tools/servo_sweep_test.py` - smooth servo sweep test.
- `tools/netdiag_server.py` - lightweight network diagnostics server.

## Bin Scripts

The `bin/` directory contains shell scripts intended for the ROV:

- `install_configure.sh` - initial install/provisioning.
- `update_code.sh` - fast field code updater.
- `configure_tether_gateway.sh` - Pi route setup through pilot computer.
- `rov_debug.sh` - debug helper for running service-related commands.
- `pwm_diag.py` and `thruster_test.py` - low-level diagnostics.

## Tests

The `tests/` directory focuses on hardware-free behavior. Add tests here when
changing control logic, config persistence, channel maps, telemetry processors,
or RPC contracts.
