# TritonOS

TritonOS is the onboard runtime for Triton's ROV. It runs on the vehicle
computer, listens for pilot commands from TritonPilot, drives the vehicle's PWM
outputs, publishes sensor telemetry, manages camera streams, and exposes a
small management RPC surface for configuration and calibration tasks.

TritonOS is intentionally separate from mission analysis code. During
competition, mission-specific detection and scoring workflows should run from
the TritonAnalysis repository on a separate computer. TritonOS should stay
focused on safe vehicle operation, telemetry, and hardware control.

## What Runs On The ROV

`main_rov.py` starts the onboard services:

- Pilot receiver and control loop on `tcp://0.0.0.0:6000`
- Sensor telemetry publisher on `tcp://0.0.0.0:6001`
- Video stream RPC server on `tcp://0.0.0.0:5555`
- Management RPC server on `tcp://0.0.0.0:5556`
- Optional network diagnostics server on port `7700`

The most important runtime flow is:

```text
TritonPilot controller input
        |
        v
PilotFrame JSON over ZeroMQ
        |
        v
PilotReceiver -> ControlService -> Mixer/Autopilot -> ThrustWriter
        |
        v
Navigator/PCA9685 PWM outputs
```

Sensor and video flows are separate so telemetry, control, and camera problems
can be diagnosed independently.

## Repository Layout

```text
main_rov.py          ROV service supervisor and normal entry point
rov_config.py        Operator-tunable runtime configuration
control/             Pilot intake, arming, control loop, hold controllers, RPC
motion/              PWM backends, thruster writer, channel mapping
sensors/             Hardware drivers, telemetry wrappers, derived processors
video/               GStreamer stream manager and video RPC server
schema/              Shared pilot-control wire schema
utils/               Shared config, Navigator import, reference, ZMQ helpers
tools/               Operator diagnostics and bench/field utilities
bin/                 Install, update, tether, and debug shell scripts
tests/               Hardware-free unit tests
docs/                Setup, networking, architecture, and subsystem docs
```

## Start Here

- [Documentation Index](docs/README.md)
- [Setup Guide](docs/SETUP.md)
- [Network Guide](docs/NETWORKING.md)
- [Operations Guide](docs/OPERATIONS.md)
- [Architecture Overview](docs/ARCHITECTURE.md)
- [Subsystem Reference](docs/SUBSYSTEMS.md)
- [Configuration Guide](docs/CONFIGURATION.md)
- [Testing And Troubleshooting](docs/TESTING_AND_TROUBLESHOOTING.md)

## Development Quick Start

From a development machine:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python -m pytest
```

The test suite is designed to run without physical ROV hardware. Hardware-only
checks live in `tools/` and should be run deliberately on the vehicle.

## ROV Install And Update

Initial provisioning on the Raspberry Pi:

```bash
sudo bash bin/install_configure.sh --project-dir /home/TritonOS
```

Normal code update on the Pi:

```bash
sudo bash bin/update_code.sh --dir /home/TritonOS
```

Check the service:

```bash
sudo systemctl status tritonos-rov.service
sudo journalctl -u tritonos-rov.service -f
```

See [Setup Guide](docs/SETUP.md) for the full install path, expected hardware
dependencies, and recovery options.

## Network Defaults

The normal tethered layout is:

- Pilot computer: `192.168.1.1`
- ROV Ethernet: `192.168.1.4`
- Pilot commands: ROV port `6000`
- Sensor stream: ROV port `6001`
- Video RPC: ROV port `5555`
- Management RPC: ROV port `5556`
- Network diagnostics: ROV port `7700`

See [Network Guide](docs/NETWORKING.md) for tether setup, internet sharing,
route configuration, video routing, and troubleshooting.

## Safety Notes

TritonOS starts disarmed. The control loop only commands non-neutral thruster
outputs after an arm command, fresh pilot input, and configured arming checks.
Hardware scripts in `tools/` can bypass pieces of the normal control stack, so
use them with props removed or the vehicle secured.

Before any water test, run:

```bash
python -m tools.rov_preflight
python -m tools.print_channel_map
```

Then use the operations checklist in [Operations Guide](docs/OPERATIONS.md).
