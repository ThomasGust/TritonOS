# TritonOS Topside Handoff: ROV Config and Reference Management

This document is for a model or developer working on the **topside GUI**.

It explains how the **ROV-side config/reference management** works today and
what a new topside page should do to interact with it.


## Goal

The ROV now exposes a dedicated **management RPC service** that lets topside:

- inspect the currently loaded `rov_config`
- inspect the saved reference/calibration state
- persist selected config changes into `rov_config.py`
- capture and save a **surface pressure reference**
- capture and save a **flat mount reference**

This is separate from the pilot control stream and separate from the video RPC.


## Important Concepts

### 1. Config vs runtime

The management RPC writes persistent changes to the ROV filesystem.

- Config values are persisted into `rov_config.py`
- references are persisted into JSON files under `calibration/`

Most updates are **not live-reloaded into already-running subsystems**.
The RPC replies explicitly include:

- `"restart_required": true`

So the GUI should assume:

- changes are saved immediately
- a TritonOS restart is usually required before the running system fully uses them


### 2. Depth reference behavior

The ROV no longer has to boot at the water surface.

Depth now works like this:

1. If a saved reference pressure file exists, TritonOS uses it
2. Otherwise it falls back to the older boot-time surface sampling behavior

Relevant config keys:

- `EXTERNAL_DEPTH_REFERENCE_PATH`
- `EXTERNAL_DEPTH_FIXED_SURFACE_PRESSURE_MBAR`
- `EXTERNAL_DEPTH_SENSOR_TO_TOP_M`

Important semantic detail:

- published `depth_m` is now measured from the **top of the ROV**
- raw sensor depth is also published as `depth_sensor_m`
- `EXTERNAL_DEPTH_SENSOR_TO_TOP_M` defaults to `0.15`


### 3. Flat pose behavior

The ROV can save a mount file representing a known “flat” vehicle pose.

Relevant config keys:

- `ATTITUDE_MOUNT`
- `ATTITUDE_AUTO_MOUNT_SAVE_PATH`
- `ATTITUDE_AUTO_MOUNT_WITH_SAVED_MOUNT`

Current intended behavior:

- if a saved mount file exists at `ATTITUDE_MOUNT`, the attitude system uses it
- startup no longer has to happen in a known flat pose after that file is captured


## ROV-side Files

These are the main files the topside model should know about:

- `control/management_rpc.py`
  ROV management RPC server
- `tools/management_rpc_client.py`
  simple client / reference for request format
- `utils/config_store.py`
  persistent config-file writer for `rov_config.py`
- `utils/vehicle_reference.py`
  shared depth/flat reference helpers
- `tools/set_vehicle_reference.py`
  on-ROV CLI tool that captures the same references manually
- `rov_config.py`
  persistent configuration file
- `main_rov.py`
  starts the management RPC service


## RPC Endpoint

Configured in `rov_config.py`:

- `MANAGEMENT_RPC_ENABLE = True`
- `MANAGEMENT_RPC_ENDPOINT = "tcp://0.0.0.0:5556"`

Topside should usually connect to:

- `tcp://<rov-host>:5556`

Protocol:

- ZeroMQ `REQ` / `REP`
- request body is JSON
- response body is JSON


## RPC Commands

All requests are shaped like:

```json
{
  "cmd": "some_command",
  "args": {}
}
```

All responses are shaped like:

```json
{
  "ok": true,
  "data": {}
}
```

or

```json
{
  "ok": false,
  "error": "message"
}
```


### `ping`

Request:

```json
{"cmd": "ping"}
```

Response:

```json
{"ok": true, "data": "pong"}
```


### `get_state`

Primary read API for the GUI.

Request:

```json
{"cmd": "get_state"}
```

Response shape:

```json
{
  "ok": true,
  "data": {
    "config_path": "/home/TritonOS/rov_config.py",
    "config": {
      "DEPTH_HOLD_KP": 0.55,
      "EXTERNAL_DEPTH_SENSOR_TO_TOP_M": 0.15
    },
    "references": {
      "depth_reference_path": "calibration/depth_reference.json",
      "depth_reference_exists": true,
      "surface_pressure_mbar": 1014.82,
      "depth_sensor_to_top_m": 0.15,
      "mount_path": "calibration/flat_mount.json",
      "mount_exists": true,
      "mount_loaded": true
    },
    "commands": [
      "get_state",
      "set_config",
      "set_surface_reference",
      "capture_surface_reference",
      "capture_flat_reference"
    ]
  }
}
```

Use this to populate the GUI page.


### `set_config`

Persists one or more config values into `rov_config.py`.

Request:

```json
{
  "cmd": "set_config",
  "args": {
    "updates": {
      "DEPTH_HOLD_KP": 0.6,
      "DEPTH_HOLD_KI": 0.14,
      "EXTERNAL_DEPTH_SENSOR_TO_TOP_M": 0.15
    }
  }
}
```

Response:

```json
{
  "ok": true,
  "data": {
    "updated": {
      "DEPTH_HOLD_KP": 0.6,
      "DEPTH_HOLD_KI": 0.14,
      "EXTERNAL_DEPTH_SENSOR_TO_TOP_M": 0.15
    },
    "references": { ... },
    "restart_required": true
  }
}
```

Current limitations:

- only updates **existing uppercase assignment keys** already present in `rov_config.py`
- this is intended for known/configured GUI fields, not arbitrary file editing


### `set_surface_reference`

Directly sets the saved surface pressure value without reading sensors.

Request:

```json
{
  "cmd": "set_surface_reference",
  "args": {
    "surface_pressure_mbar": 1014.8
  }
}
```

Optional:

- `args.path`

Response:

```json
{
  "ok": true,
  "data": {
    "surface_pressure_mbar": 1014.8,
    "path": "calibration/depth_reference.json",
    "restart_required": true
  }
}
```


### `capture_surface_reference`

Asks the ROV to read the external depth sensor and save the current pressure as
the surface reference.

Use this when the operator has the ROV positioned so the **top of the vehicle is
at the water surface**.

Request:

```json
{
  "cmd": "capture_surface_reference",
  "args": {
    "samples": 20,
    "delay_s": 0.02
  }
}
```

Optional:

- `args.path`

Response:

```json
{
  "ok": true,
  "data": {
    "surface_pressure_mbar": 1014.82,
    "path": "calibration/depth_reference.json",
    "restart_required": true
  }
}
```


### `capture_flat_reference`

Asks the ROV to read accelerometer data and save the current vehicle pose as
the flat/reference mount.

Request:

```json
{
  "cmd": "capture_flat_reference",
  "args": {
    "samples": 200,
    "delay_s": 0.02,
    "yaw_deg": 90
  }
}
```

Optional:

- `args.path`

Response:

```json
{
  "ok": true,
  "data": {
    "path": "calibration/flat_mount.json",
    "yaw_deg": 90,
    "restart_required": true
  }
}
```

`yaw_deg` should usually default to the current ROV config value:

- `ATTITUDE_AUTO_MOUNT_YAW_DEG`


## Saved Files

Default persisted files:

- `calibration/depth_reference.json`
- `calibration/flat_mount.json`

The GUI should treat these as ROV-owned implementation details and use the RPC,
not direct file access.


## Recommended GUI Page

Suggested page sections:

### 1. Reference Status

Show:

- active config path
- management RPC connected/disconnected
- whether depth reference file exists
- current saved surface pressure
- current `EXTERNAL_DEPTH_SENSOR_TO_TOP_M`
- whether flat mount file exists


### 2. Depth Reference Controls

Controls:

- numeric field for `EXTERNAL_DEPTH_SENSOR_TO_TOP_M`
- numeric field for manual `surface_pressure_mbar`
- button: `Capture Surface Reference`
- button: `Save Manual Surface Pressure`

UX note:

- explain that capture should be done with the **top of the ROV at the water surface**
- explain that published depth is measured from the top of the ROV, not from the sensor


### 3. Flat Pose Controls

Controls:

- numeric field for `yaw_deg`
- button: `Capture Flat Pose`

UX note:

- explain that the ROV should be held in the pose that should count as “flat”


### 4. Config Controls

Expose selected safe/configured fields, not every value in `rov_config.py`.

Good first candidates:

- `DEPTH_HOLD_KP`
- `DEPTH_HOLD_KI`
- `DEPTH_HOLD_KD`
- `DEPTH_HOLD_LPF_TAU_S`
- `DEPTH_HOLD_ERROR_DEADBAND_M`
- `DEPTH_HOLD_OUT_LIMIT`
- `EXTERNAL_DEPTH_SENSOR_TO_TOP_M`
- possibly `EXTERNAL_DEPTH_RATE_HZ`

The page should likely batch-save these through one `set_config` request.


### 5. Restart Banner

After any successful mutating operation, show a clear banner:

- “Saved on ROV. TritonOS restart required to fully apply.”

Do not imply that all changes are live.


## Suggested Topside Client Behavior

On page load:

1. connect to management RPC
2. send `get_state`
3. populate the form from `data.config` and `data.references`

On save:

1. send mutating RPC
2. if `ok`, refresh with `get_state`
3. show success + restart notice

On error:

- show the returned `error`
- keep the page state intact so the operator does not lose input


## Safety and UX Guidance

- confirm before capture operations
- disable capture buttons while the request is in flight
- give operator instructions before each capture
- show the exact returned values after capture
- never edit `rov_config.py` from topside directly; always go through RPC
- avoid exposing arbitrary free-form file/path editing in the GUI


## Current Non-goals / Limitations

- no authentication or authorization layer yet
- no full live-reconfigure for running subsystems
- no built-in “restart TritonOS” RPC yet
- no topside GUI implementation in this repo

If the topside app wants restart support later, add it as a new explicit RPC
command instead of overloading config writes.


## Reference CLI

There is already a simple client in this repo:

- `tools/management_rpc_client.py`

Examples:

```bash
python tools/management_rpc_client.py --endpoint tcp://tritonpi:5556 get-state
python tools/management_rpc_client.py --endpoint tcp://tritonpi:5556 set-config "{\"DEPTH_HOLD_KP\": 0.6}"
python tools/management_rpc_client.py --endpoint tcp://tritonpi:5556 capture-surface
python tools/management_rpc_client.py --endpoint tcp://tritonpi:5556 capture-flat --yaw-deg 90
```

This is the easiest executable reference for the topside model to follow.

