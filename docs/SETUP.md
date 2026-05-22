# Setup Guide

This guide covers installing TritonOS on the ROV computer, starting the onboard
service, updating code, and recovering from a partial install.

## Target Machine

TritonOS is designed for the onboard Raspberry Pi / Linux computer connected to
the Blue Robotics Navigator and vehicle hardware. Development and unit tests can
run on a laptop, but the install scripts assume a Pi-style Linux environment.

The expected production checkout path is:

```bash
/home/TritonOS
```

The install script creates a Python virtual environment at:

```bash
/home/TritonOS/.venv
```

The boot service is:

```bash
tritonos-rov.service
```

## Fresh Install

Run these commands on the ROV computer.

```bash
cd /home
sudo git clone https://github.com/ThomasGust/TritonOS.git /home/TritonOS
cd /home/TritonOS
sudo bash bin/install_configure.sh --project-dir /home/TritonOS
```

The installer:

- Installs OS packages for Python, I2C, GPIO, video, and GStreamer.
- Adds the target user to `i2c`, `gpio`, and `video` groups.
- Enables I2C.
- Optionally runs Blue Robotics Navigator board overlay setup.
- Optionally enables legacy V4L2 camera support.
- Creates `.venv` with system site packages.
- Installs Python dependencies and Navigator bindings.
- Installs and starts `tritonos-rov.service`.

Hardware interface changes usually need a reboot:

```bash
sudo reboot
```

## Useful Installer Flags

Use these only when you know why you need them.

```bash
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --recreate-venv
```

Rebuilds `.venv` if Python dependency state is broken.

```bash
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --skip-os-packages
```

Skips apt package work. Useful when the Pi has no internet but OS packages are
already installed.

```bash
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --skip-python-deps
```

Skips pip work. Useful when you are repairing service files or code only.

```bash
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --no-navigator-overlay
```

Skips the Blue Robotics board overlay setup. Only use this if the overlay was
already configured or you are not on Navigator hardware.

## Service Management

Check whether TritonOS is running:

```bash
sudo systemctl status tritonos-rov.service
```

Follow logs:

```bash
sudo journalctl -u tritonos-rov.service -f
```

Restart after changing code or config:

```bash
sudo systemctl restart tritonos-rov.service
```

Stop the service for bench diagnostics:

```bash
sudo systemctl stop tritonos-rov.service
```

Run foreground for a clearer stack trace:

```bash
cd /home/TritonOS
sudo -u triton /home/TritonOS/.venv/bin/python /home/TritonOS/main_rov.py
```

If the user is not named `triton`, replace it with the account that owns the
checkout.

## Normal Code Update

Run this on the ROV computer:

```bash
cd /home/TritonOS
sudo bash bin/update_code.sh --dir /home/TritonOS
sudo systemctl restart tritonos-rov.service
```

The updater defaults to a code-first path. It avoids apt work unless requested
and keeps Git operations bounded so a poor field network does not hang forever.

Use apt refresh only when dependencies or base tools are missing:

```bash
sudo bash bin/update_code.sh --dir /home/TritonOS --with-apt
```

Use Git connectivity checks only when repository corruption is suspected:

```bash
sudo bash bin/update_code.sh --dir /home/TritonOS --fsck
```

## Private Repository Credentials

If GitHub fetch fails because the repository is private, set a read-only token
once:

```bash
export TRITON_GITHUB_TOKEN='...'
sudo -E bash bin/update_code.sh --dir /home/TritonOS
```

The updater stores that credential for future pulls by the target user.

## Development Machine Setup

On a laptop or workstation:

```bash
cd TritonOS
python -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python -m pytest
```

The tests avoid physical hardware by default. Hardware diagnostics are separate
tools and should be run deliberately on the ROV.

## First Boot Validation

After install and reboot:

```bash
cd /home/TritonOS
python -m tools.rov_preflight --min-cameras 1
python -m tools.print_channel_map
sudo systemctl status tritonos-rov.service
```

If cameras are not connected yet, the preflight camera verdict may fail while
the rest of the system is still usable.

## Recovery Install

If the checkout exists but dependencies are broken:

```bash
cd /home/TritonOS
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --recreate-venv
sudo reboot
```

If internet is unavailable but the Pi already has dependencies:

```bash
cd /home/TritonOS
sudo bash bin/install_configure.sh \
  --project-dir /home/TritonOS \
  --skip-os-packages \
  --skip-python-deps \
  --no-navigator-overlay
```

If the checkout itself is unusable, move it aside before recloning so any local
calibration files can still be recovered:

```bash
cd /home
sudo mv TritonOS TritonOS.broken.$(date +%Y%m%d-%H%M%S)
sudo git clone https://github.com/ThomasGust/TritonOS.git /home/TritonOS
cd /home/TritonOS
sudo bash bin/install_configure.sh --project-dir /home/TritonOS
```

## Expected Runtime Files

The runtime may create local data that should not be treated as source code:

- `.venv/` - Python virtual environment.
- `.pytest_cache/`, `.pytest-tmp/`, `.pytest-work/` - test artifacts.
- `calibration/` - depth reference and similar vehicle-specific data.
- `logs/` or `recordings/` - optional runtime captures.

Do not delete calibration data during field recovery unless you intentionally
want to recalibrate.
