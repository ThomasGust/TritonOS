# TritonOS

TritonOS is the onboard runtime for the ROV. It starts the pilot/control loop,
hardware output sink, sensor publisher, video RPC service, and management RPC
service used by the topside TritonPilot app.

Current stability software is intentionally limited to depth hold. Raw IMU
telemetry publishes accelerometer and gyroscope samples at the IMU rate, while
raw magnetometers publish separately so AK09915 and optional MMC5983 samples
can be compared without slowing down accel/gyro telemetry.

## Raw Sensor Stream

The ROV publishes sensor telemetry on `rov_config.SENSOR_PUB_ENDPOINT`, usually
`tcp://0.0.0.0:6001`. IMU messages include raw accel/gyro only. Raw
magnetometer messages use `type: "mag"` and include the primary AK09915 vector
plus `mag_sources` for all detected raw magnetometers.

For isolated stream testing on the ROV:

```bash
python tools/sensor_stream_pub_test.py --fake
```

Then connect from the TritonPilot checkout with:

```bash
python tools/sensor_stream_sub_test.py --endpoint tcp://<rov-ip>:6001
```

## Test

```bash
pytest
```

The pytest configuration uses a repo-local `.pytest-tmp` directory and skips
tests marked `hardware` by default so the suite can run on a development
machine without touching physical ROV devices.

## ROV Setup And Updates

For normal code updates on the Pi, use:

```bash
sudo bash bin/update_code.sh
```

That path avoids `apt-get update` and full repository checks by default so it
stays fast on the bench network. Use `--with-apt` only when you actually want
the script to refresh/install base OS tools, and `--fsck` only when you suspect
repository corruption.

Initial provisioning remains:

```bash
sudo bash bin/install_configure.sh
```

Both scripts force apt to IPv4 with short network timeouts, which avoids long
IPv6 stalls on networks where the Pi has no IPv6 route.

If the repo is private and the update fails at Git fetch/pull, the updater will
try to copy an existing root GitHub credential into the `triton` user's
credential store. If no credential exists, set a fresh read-only token once:

```bash
export TRITON_GITHUB_TOKEN='...'
sudo -E bash bin/update_code.sh
```

For an offline-ish repair when the Pi already has dependencies installed:

```bash
sudo bash bin/install_configure.sh --skip-os-packages --skip-python-deps --no-navigator-overlay
```

## Tether Internet Gateway

The ROV can route updates through the pilot computer instead of joining Wi-Fi.
The expected tether addresses are pilot `192.168.1.1/24` and ROV `eth0`
`192.168.1.4/24`.

First configure the Windows side from the TritonPilot checkout:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\setup_tether_nat.ps1 -TuneAdapter -ResetAdapter
```

Then test and enable the Pi route:

```bash
sudo bash bin/configure_tether_gateway.sh --probe
sudo bash bin/configure_tether_gateway.sh --persistent
```

The Pi script refuses to install the default route unless the pilot gateway
responds on Ethernet first, which prevents a broken tether link from stealing
the working Wi-Fi default route.
