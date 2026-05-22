# Testing And Troubleshooting

This guide gives a recommended debug order for TritonOS. The main idea is to
test one layer at a time: code logic, service startup, network, hardware
interfaces, then full integrated control.

## Quick Trust Check

Run from the repository root:

```bash
python -m pytest
```

If the test tooling is not installed, run `python -m pip install -r requirements-dev.txt`.

The default suite is designed to run on a development machine without ROV
hardware or active network services. Tests marked `network`, `hardware`, `slow`,
or `groundtruth` are skipped unless explicitly requested.

The helper script exposes the standard tiers:

```bash
python tools/trust_check.py quick
python tools/trust_check.py network
python tools/trust_check.py extended
python tools/trust_check.py hardware
python tools/trust_check.py full
```

Equivalent direct pytest commands:

| Goal | Command |
| --- | --- |
| Fast software-only check | `python -m pytest` |
| Local socket/ZMQ tests | `python -m pytest --run-network -m network` |
| All non-hardware optional tiers | `python -m pytest --run-extended` |
| Physical ROV/hardware tests | `python -m pytest --run-hardware -m hardware` |
| Everything | `python -m pytest --run-all-trust` |
| Coverage report, if `pytest-cov` is installed | `python tools/trust_check.py coverage` |

Environment variables work for CI or shell profiles:

- `TRITON_RUN_NETWORK=1`
- `TRITON_RUN_GROUNDTRUTH=1`
- `TRITON_RUN_SLOW=1`
- `TRITON_RUN_HARDWARE=1`

The software-only tests cover:

- Pilot schema and receiver behavior.
- Control service safety and payload generation.
- Mixers and channel map validation.
- Depth hold and attitude/autopilot logic.
- Management RPC behavior.
- Vehicle reference storage.
- Preflight report logic.

## Test Marker Policy

Use the default suite for deterministic tests that can run on any developer
machine. Mark new tests when they leave that boundary:

- `network`: opens sockets, uses ZMQ over TCP, or depends on active networking.
- `hardware`: touches physical ROV hardware or live system services.
- `slow`: intentionally takes long enough that it should not block quick checks.
- `groundtruth`: depends on optional saved media or datasets outside the normal
  repository fixtures.
- `integration`: crosses module/service boundaries but remains deterministic and
  hardware-free.

Pytest artifacts are repo-local and ignored by Git:

- `.pytest_cache/`
- `.pytest-tmp/`
- `.pytest-work/`

## Preflight Report

Run on the ROV:

```bash
python -m tools.rov_preflight --min-cameras 1
```

Use JSON output when attaching a report to an issue or competition log:

```bash
python -m tools.rov_preflight --json > preflight.json
```

Use Navigator smoke only when hardware is connected and it is safe to touch the
Navigator sensor stack:

```bash
python -m tools.rov_preflight --include-navigator
```

## Debug Order

When something fails, use this order:

1. Confirm the service is running.
2. Confirm the tether network works.
3. Confirm config values and channel map.
4. Test the specific hardware interface directly.
5. Test the TritonOS wrapper for that hardware.
6. Test the full TritonPilot -> TritonOS path.

Jumping straight to the full system makes it harder to tell whether the failure
is network, config, hardware, or logic.

## Service Problems

Check status:

```bash
sudo systemctl status tritonos-rov.service
```

Follow logs:

```bash
sudo journalctl -u tritonos-rov.service -f
```

Run foreground for stack traces:

```bash
cd /home/TritonOS
sudo -u triton /home/TritonOS/.venv/bin/python /home/TritonOS/main_rov.py
```

Common causes:

- `.venv` missing or stale.
- Missing hardware permissions for `i2c`, `gpio`, or `video`.
- Navigator bindings installed incorrectly.
- GStreamer missing.
- `rov_config.py` syntax error.
- Camera or sensor hardware absent during startup.

Repair dependency state:

```bash
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --recreate-venv
```

## Network Problems

Check ROV addresses and routes:

```bash
ip -br addr
ip route
```

Check tether gateway:

```bash
ping -c 2 192.168.1.1
sudo bash bin/configure_tether_gateway.sh --probe
```

Check internet from the ROV:

```bash
curl -4 -I --connect-timeout 5 --max-time 8 https://github.com
```

If SSH works from the pilot computer but GitHub does not work from the ROV, the
problem is ROV outbound routing/DNS, not SSH.

## Control Problems

Print the channel map:

```bash
python -m tools.print_channel_map
```

Check whether pilot frames are arriving by watching service logs:

```bash
sudo journalctl -u tritonos-rov.service -f
```

If the ROV arms but motors do not move:

1. Confirm `dry_run` is false in logs.
2. Confirm `REQUIRE_HARDWARE_PWM` is not masking a startup issue.
3. Confirm `PWM_BACKEND` initialized.
4. Confirm the PWM sink is armed.
5. Run a direct hardware test.

Direct Navigator binding test:

```bash
sudo .venv/bin/python -m tools.native_motor_test --channel 1 --throttle 0.15
```

Direct PCA9685 test:

```bash
sudo .venv/bin/python -m tools.direct_i2c_pwm_test --channels 1 --pulse-us 1550 --hold 1
```

Configured thruster test:

```bash
sudo .venv/bin/python -m tools.thruster_test --power 0.15 --seconds 1
```

## Thruster Direction Or Mapping Problems

Symptoms:

- A named thruster spins the wrong physical motor.
- Surge/sway/yaw directions are wrong.
- Vertical control creates roll or pitch.
- Lights or aux outputs move when a thruster should move.

Debug order:

1. Use direct channel tests to identify physical channel wiring.
2. Update `CHANNEL_MAP` in `rov_config.py`.
3. Run `python -m tools.print_channel_map`.
4. Use `tools.thruster_identity_test` to confirm logical names.
5. Use `THRUSTER_REVERSED` for direction flips.

Do not fix physical channel mapping by changing mixer math.

## Sensor Problems

Fake stream test:

```bash
python -m tools.sensor_stream_pub_test --fake
```

Hardware stream test:

```bash
python -m tools.sensor_stream_pub_test --require-hw
```

External depth specific:

```bash
python -m tools.bar02_i2c_test --bus 6 --once
```

If a sensor fails in the full service but works alone, compare:

- I2C/SPI bus settings in `rov_config.py`.
- Whether another process is using the bus.
- Whether Navigator bindings are conflicting with direct drivers.
- Whether the service user has hardware group permissions.

## Depth Hold Problems

Check that `external_depth` telemetry exists and is fresh. Then check hold
status through the management state snapshot:

```bash
python -m tools.management_rpc_client --endpoint tcp://127.0.0.1:5556 get-state
```

If depth hold drives the wrong way:

1. Confirm depth increases when the ROV goes down.
2. Confirm heave sign in manual control.
3. Flip `DEPTH_HOLD_SIGN` only after the first two checks.

If depth hold jitters:

- Increase `DEPTH_HOLD_ERROR_DEADBAND_M`.
- Increase `DEPTH_HOLD_LPF_TAU_S`.
- Lower `DEPTH_HOLD_KP` or `DEPTH_HOLD_KD`.
- Verify pressure sensor noise and surface reference.

## Video Problems

Run:

```bash
python -m tools.rov_preflight --min-cameras 1
```

Check GStreamer logs:

```bash
sudo journalctl -u tritonos-rov.service -f
```

Common causes:

- Camera not enumerated under `/dev/v4l/by-path`.
- Format/resolution unsupported by the camera node.
- H.264 stream is on a sibling `video-index` device.
- Pilot firewall blocks UDP video packets.
- Stream host points to the wrong pilot IP.
- USB camera needs rebind/reset.

## Management RPC Problems

Local state check:

```bash
python -m tools.management_rpc_client --endpoint tcp://127.0.0.1:5556 get-state
```

Remote state check from pilot computer:

```bash
python -m tools.management_rpc_client --endpoint tcp://192.168.1.4:5556 get-state
```

If local works but remote fails, check firewall and tether routing. If both
fail, check whether `MANAGEMENT_RPC_ENABLE` is true and read service logs.

## When To Add Tests

Add or update tests when changing:

- Control-service arming/failsafe behavior.
- Mixer math or channel-map validation.
- Depth hold, attitude estimator, or autopilot logic.
- Pilot schema.
- Config persistence or management RPC behavior.
- Any bug that was hard to diagnose from logs.

Keep hardware-specific tests as explicit tools unless they can be safely
simulated.
