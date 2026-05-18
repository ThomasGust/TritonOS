# Topside Configuration And Reference Handoff

This document describes the current management-RPC surface shared between
TritonOS and TritonPilot after resetting the vehicle stability stack.

## Runtime State

`get_state` returns:

```json
{
  "config_path": "rov_config.py",
  "config": {},
  "references": {
    "depth_reference_path": "calibration/depth_reference.json",
    "depth_reference_exists": true,
    "surface_pressure_mbar": 1014.5,
    "depth_sensor_to_top_m": 0.0
  },
  "runtime": {
    "control_loop_available": true,
    "armed": false,
    "updated_ts": 0.0,
    "depth_hold": {
      "available": true,
      "sensor_available": true,
      "target_m": null,
      "status": {},
      "status_age_s": null,
      "sensor": {}
    }
  },
  "commands": [
    "get_state",
    "get_hold_status",
    "set_config",
    "set_surface_reference",
    "capture_surface_reference"
  ]
}
```

## Commands

`get_hold_status` returns the live `runtime` shape.

`set_config` accepts:

```json
{
  "cmd": "set_config",
  "args": {
    "updates": {
      "DEPTH_HOLD_KP": 0.55
    }
  }
}
```

`set_surface_reference` writes an explicit pressure reference:

```json
{
  "cmd": "set_surface_reference",
  "args": {
    "surface_pressure_mbar": 1014.5
  }
}
```

`capture_surface_reference` samples the configured depth sensor and persists
the averaged surface pressure:

```json
{
  "cmd": "capture_surface_reference",
  "args": {
    "samples": 20,
    "delay_s": 0.02
  }
}
```

## Notes

Depth hold remains the only closed-loop hold mode exposed through management
RPC. Raw IMU and magnetometer data are still published by the sensor service;
logging, visualization, merged orientation messages, estimator work, and any
future hold mode should be layered back in deliberately.
