# Visual Station-Keeping (optical-tracking autopilot)

Foundation for the MATE RANGER "hold position in current" task: keep a target
(blue square) framed through the transect/arm camera while keeping forbidden
content (the surrounding red square) out of frame. This document describes the
control foundation that exists today and the contract for the computer vision
(CV) that will drive it later.

## Split of responsibilities

```
 topside (TritonPilot)                         ROV (TritonOS)
 ┌─────────────────────────┐   visual error   ┌──────────────────────────┐
 │ OpticalTracker (CV, TBD)│ ───────────────▶ │ StationKeepController      │
 │  tracking/optical_*.py  │  modes[...]      │  control/station_keep.py  │
 └─────────────────────────┘  ["autopilot"]   │  → surge/sway corrections │
                              ["visual"]       └──────────────────────────┘
```

- **Perception (topside, not built yet):** a CV tracker watches the transect
  feed and produces a normalized *visual error*. Lives topside because that is
  where the video and GPU are. Interface: `TritonPilot/tracking/optical_tracker.py`
  (`OpticalTracker.process(frame) -> VisualTargetError`). `NullOpticalTracker`
  is the inert placeholder.
- **Control (ROV, built):** `control/station_keep.py` turns that error into
  thrust corrections, folded into the autopilot after depth/attitude in
  `control/autopilot.py`. Conservative: disabled / no-lock / stale / pilot
  override all fall back to manual.

## Interface (keep both sides in sync)

The CV publishes, inside the pilot command, `modes["autopilot"]`:

```python
{
  "station_keep": True,            # mode toggle
  "visual": {
    "valid": bool,                 # confident lock this frame
    "ts": float,                   # producer timestamp (frozen-CV detection)
    "ex": float,  # [-1,1] horizontal target-center error (+ = right)
    "ey": float,  # [-1,1] vertical error               (+ = below)
    "es": float,  # [-1,1] scale/size error             (+ = too close)
    "violation": float,  # 0..1 forbidden (red) content visible; 0 = none
  }
}
```

`VisualTargetError.to_visual_payload()` produces exactly this dict;
`station_keep_modes(err, enabled=...)` wraps it for merging into the command.

### Full model authority (translation, attitude, dynamic depth)

The model is not limited to emitting an error for the ROV PID to chase. It has
three freely-combinable, per-DOF outputs (topside `StationKeepCommand` →
`to_autopilot_modes()`):

1. **Error → ROV PID** (above): a hand-tuned classical baseline.
2. **Direct DOF thrust** — `modes["autopilot"]["visual"]["command"] =
   {surge, sway, heave, roll, pitch, yaw}` (normalized, only with a valid lock).
   The model *is* the controller; `StationKeepController` passes these straight
   through (capped by `STATION_KEEP_DIRECT_LIMIT`), overriding the error-PID for
   those DOFs, yielding to any DOF the pilot is actively driving. This is the
   path for a learned policy and gives full surge/sway/heave + roll/pitch/yaw.
3. **Dynamic setpoints** — `modes["autopilot"]["targets"] = {depth_m, yaw_deg,
   roll_deg, pitch_deg}` plus the hold-enable flags (`depth`, `yaw`,
   `roll_pitch_level`). These drive the existing drift-free depth/attitude holds,
   so the model can command "track to depth 1.5 m / heading 30°" instead of
   fighting them with raw thrust.

So **dynamic depth control, full translation, and roll/pitch/yaw are all
supported**, per-DOF, mixing direct thrust and setpoints however the model wants.

## Control behaviour & tuning

`StationKeepController` runs one PID per configured `StationKeepAxis`, each
mapping one error component to one thrust DOF. Defaults (in
`default_station_keep_axes`) control only **sway←ex** and **surge←es** (the DOFs
not owned by depth/attitude hold) and ship with **zero gains** — inert until
tuned, so it is safe to enable while iterating.

Tune via `rov_config` (no code changes), e.g.:

```python
STATION_KEEP_ENABLE = True
STATION_KEEP_STALE_S = 0.5
STATION_KEEP_SWAY_KP = 0.6      # ex -> sway
STATION_KEEP_SWAY_SIGN = 1.0    # flip if it strafes the wrong way
STATION_KEEP_SURGE_KP = 0.5     # es -> surge (standoff distance)
# ..._KI, ..._KD, ..._ERROR_DEADBAND, ..._I_LIMIT, ..._OUT_LIMIT, ..._MANUAL_DEADBAND
```

To involve more DOFs (e.g. yaw←ex, heave←ey, or a surge bias from `violation`),
extend `default_station_keep_axes()` / the `STATION_KEEP_*` config. The pilot is
expected to iterate on this policy ("what should the ROV do to hold position")
without touching the controller code.

## Status / diagnostics

`AutopilotController.step` returns `status["station_keep"]` with `reason`
(`disabled` / `no_lock` / `stale_lock` / `active` / `locked_idle`), per-axis
errors and outputs, and `stale_timer_s`. Surfaced through the existing autopilot
status path.

## Capturing pool data for model iteration

A single-camera video recording (controller **B** in standard capture mode, with
the transect/arm camera selected) now **bundles a synchronized dataset** under
one `recordings/<session>/` folder, topside:

- `video/<camera>-<ts>.mp4` — the camera feed (native H.264, no re-encode).
- `<ts>_streams.jsonl` — the state log: `pilot` (commands + modes), `sensors`
  (incl. `autopilot_status` → depth/attitude/**station_keep** status), `attitude`,
  and `tracking` (model error/command samples once the CV runs, via
  `MainWindow.record_tracking_sample(payload)`).
- `capture_manifest.json` — ties the mp4 to the log; align by wall clock
  (`video.started_wall_ts` ≈ t0 of the mp4).

So one button-press in the pool yields aligned (video, vehicle state, pilot
action) tuples — the raw material for imitation learning / model dev. Extra
low-level diagnostics: set `TRITON_CAPTURE_TRACE=1` for the `capture_trace`
JSONL event log.

## Not done yet (next steps)
- The CV model itself (`OpticalTracker` implementation) — topside.
- Wiring a pilot control (button/key) to toggle `station_keep` and to start
  publishing `visual` from the tracker into the outgoing command.
- Pool tuning of the `STATION_KEEP_*` gains and the error→DOF policy.
