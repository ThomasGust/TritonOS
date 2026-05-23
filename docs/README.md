# TritonOS Documentation

This folder is the maintained documentation set for TritonOS. It is meant to
be read as repository documentation, not as a separate wiki. Keep pages focused,
cross-linked, and close to the code they describe.

## Guides

- [Setup Guide](SETUP.md) - Install TritonOS on the ROV computer, manage the
  systemd service, update code, and recover a broken install.
- [Network Guide](NETWORKING.md) - Tether addressing, Windows internet
  sharing, Pi routing, ZeroMQ endpoints, video ports, and diagnostics.
- [Operations Guide](OPERATIONS.md) - Field workflow for preflight, startup,
  calibration, arming, runtime monitoring, and shutdown.
- [Architecture Overview](ARCHITECTURE.md) - How the onboard services fit
  together and where data moves through the system.
- [Subsystem Reference](SUBSYSTEMS.md) - What each package/module does and how
  the pieces are expected to be used.
- [Configuration Guide](CONFIGURATION.md) - How to edit `rov_config.py`, channel
  maps, safety limits, hold-controller settings, and management RPC values.
- [Stereo Streaming Notes](STEREO_STREAMING.md) - ROV-side camera streaming,
  software-sync limits, diagnostics, and future hardware-sync path.
- [Testing And Troubleshooting](TESTING_AND_TROUBLESHOOTING.md) - Unit tests,
  hardware diagnostics, failure symptoms, and recommended debug order.

## Existing Handoff Notes

- [Topside Configuration And Reference Handoff](TOPSIDE_CONFIG_REFERENCE_HANDOFF.md)
  records the current management RPC contract used by TritonPilot.

## Documentation Style

When adding docs:

- Prefer command examples that can be run from the repository root.
- Say which computer a command runs on: ROV, pilot computer, or development
  machine.
- Keep safety-critical notes near the command that can move hardware.
- Link to code paths with repository-relative paths.
- Update this index when adding or removing a maintained guide.
