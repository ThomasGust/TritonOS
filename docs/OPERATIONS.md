# Operations Guide

This guide is for bench checks, pool tests, and competition operation. It
assumes TritonOS is already installed on the ROV computer.

## Roles During Operation

TritonOS runs on the ROV and should stay focused on vehicle runtime:

- Receive pilot commands.
- Drive motors and auxiliary actuators.
- Publish telemetry.
- Stream cameras.
- Expose management/calibration commands.

TritonPilot runs on the operator computer. TritonAnalysis runs mission-specific
analysis scripts on a separate computer when needed.

## Before Powering Thrusters

Before any test that can move a motor:

- Remove props or secure the vehicle if on the bench.
- Confirm the kill/disarm path is understood.
- Confirm the operator knows whether the ROV is armed.
- Keep hands away from thrusters and moving mechanisms.
- Start with low power limits when tuning.

Hardware tools can bypass the normal arming stack. Treat every motor/servo tool
as direct hardware control.

## Field Preflight

Run on the ROV:

```bash
cd /home/TritonOS
python -m tools.rov_preflight --min-cameras 1
python -m tools.print_channel_map
```

Check service health:

```bash
sudo systemctl status tritonos-rov.service
sudo journalctl -u tritonos-rov.service -n 80
```

Check network:

```bash
ip -br addr
ip route
ping -c 2 192.168.1.1
```

If internet updates are needed through the pilot computer:

```bash
curl -4 -I --connect-timeout 5 --max-time 8 https://github.com
```

## Startup Sequence

1. Connect tether and confirm link lights.
2. Power the ROV computer.
3. Confirm the pilot computer can ping the ROV.
4. Start TritonPilot on the pilot computer.
5. Watch TritonOS logs:

   ```bash
   sudo journalctl -u tritonos-rov.service -f
   ```

6. Confirm sensor telemetry appears in TritonPilot.
7. Confirm video devices are listed and streams can be started.
8. Keep the vehicle disarmed until the operator is ready.

## Surface Pressure Reference

Before using external-depth hold, capture surface pressure while the ROV is at
the surface in the same water/air pressure conditions as the run:

```bash
cd /home/TritonOS
python -m tools.set_vehicle_reference --pressure-samples 20 --sample-delay 0.02
sudo systemctl restart tritonos-rov.service
```

The reference is stored under `calibration/` by default. Do not delete it during
a field recovery unless you intend to recalibrate.

## Control Validation

Before entering water or before enabling full power:

```bash
python -m tools.print_channel_map
```

If mapping or direction is uncertain, validate one layer at a time:

1. Direct single-channel Navigator output:

   ```bash
   sudo .venv/bin/python -m tools.native_motor_test --channel 1 --throttle 0.15
   ```

2. Direct PCA9685 output without Navigator bindings:

   ```bash
   sudo .venv/bin/python -m tools.direct_i2c_pwm_test --channels 1 --pulse-us 1550 --hold 1
   ```

3. Configured thruster names through `ThrustWriter`:

   ```bash
   sudo .venv/bin/python -m tools.thruster_test --power 0.15 --seconds 1
   ```

4. Full TritonPilot -> TritonOS control path.

Only move to the next layer when the current layer behaves as expected.

## Sensor Validation

Run a fake stream when validating topside connectivity without hardware:

```bash
python -m tools.sensor_stream_pub_test --fake
```

Run hardware stream isolation when checking sensor publishing without starting
the full control/video stack:

```bash
python -m tools.sensor_stream_pub_test --require-hw
```

Expected high-level telemetry types include:

- `heartbeat`
- `imu`
- `mag`
- `env`
- `adc`
- `external_depth`
- `attitude`
- `autopilot_status`
- `power`
- `net`

Some telemetry depends on configuration and detected hardware.

## Video Validation

Run preflight first:

```bash
python -m tools.rov_preflight --min-cameras 1
```

If video RPC is running but streams fail:

```bash
sudo journalctl -u tritonos-rov.service -f
```

Look for GStreamer errors, missing V4L2 devices, unsupported formats, or USB
rebind messages.

## During A Run

Monitor these signs:

- TritonPilot still receives heartbeat and sensor telemetry.
- Pilot frame age stays low.
- ROV remains disarmed until the operator intentionally arms.
- Power telemetry stays within expected battery/current range.
- Depth-hold and attitude status are visible when enabled.
- Video latency and packet loss are acceptable.

If something looks wrong, disarm first, then diagnose.

## Shutdown

1. Disarm in TritonPilot.
2. Stop mission actions and camera streams.
3. Stop the service if bench work will continue:

   ```bash
   sudo systemctl stop tritonos-rov.service
   ```

4. Power down the ROV computer cleanly when possible:

   ```bash
   sudo shutdown now
   ```

## After A Run

Record anything that changed:

- `rov_config.py` changes.
- Channel map or reversal changes.
- Surface pressure reference recapture.
- Hardware swaps.
- Observed symptoms and matching logs.

Then run tests on the development machine before committing code changes:

```bash
python -m pytest
```
