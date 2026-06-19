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
    "er": float,  # [-1,1] rotation error               (+ = rotated CW; 0 = squared-on)
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

## The transect model (geometry → setpoints → mapping)

The "model" layer lives topside in `TritonPilot/tracking/transect_policy.py`
(`TransectPolicy`, `TransectModel`, `TransectObservation`, `TransectEstimate`).
It turns geometric detections of the inscribed-square target into the normalized
`VisualTargetError` above. It does **no** vision — a future `OpticalTracker`
detects the blue square + red presence, packs a `TransectObservation`, and calls
`TransectPolicy.evaluate(obs)`.

**Geometry (setpoints fall out of the dimensions).** Blue (50 cm) is concentric
inside red (130 cm). Model the camera floor footprint as a window of half-size
`s`, center offset `d` (per axis, cm): contain all blue ⇒ `s ≥ 25 + |d|`; exclude
all red ⇒ `s ≤ 65 − |d|`. Maximizing the minimum margin gives, independent of `d`:

```
target footprint  W* = (blue + red)/2 = 90 cm
position tol      d_max = (red − blue)/4 = 20 cm     (min margin = 20 − |d| cm)
size  tol         (red − blue)/2 = 40 cm of footprint
```

So the loop is deliberately **forgiving** (±20 cm slack at the sweet spot) — and
because blue is concentric in red, *keeping blue centered at the target size
keeps red out automatically*. The primary loop is the safety mechanism;
`violation` (red seen) is the backup/abort signal surfaced to the pilot.

We regulate in **image space** (no calibration / 3-D): `|ex|`,`|ey|` reach 1.0 at
the `d_max` position-failure boundary; `|es|` reaches 1.0 where blue fills the
frame / red touches it. `es` is anchored to the on-station apparent blue size, so
calibrating `target_cy` / `target_blue_fraction` for the **oblique** arm cam
keeps `es == 0` on-station. `TransectPolicy` also smooths the centroid/size and
runs a confidence-hysteresis lock FSM (`no_target`/`acquiring`/`lock`/`lost`) —
that is the "good lock" indicator the pilot needs *before* engaging.

**Default DOF mapping (this task), all tunable via `STATION_KEEP_*`:**

| DOF   | error | role                                                            |
|-------|-------|-----------------------------------------------------------------|
| sway  | `ex`  | horizontal centering — PI (integral rejects steady current)     |
| surge | `ey`  | fore/aft centering — PI (`STATION_KEEP_SURGE_ERROR_KEY="ey"`)    |
| heave | `es`  | **gentle** size trim layered on depth hold (owns bulk altitude) |
| yaw   | `er`  | square the target up (rotation→0) to maximize the margin        |

Roll/pitch stay level via attitude hold (stable camera geometry). The integral
terms on sway/surge are what actually *hold* against a steady current instead of
drooping downstream. **Yaw squares the target up** — at 0° the see-all-blue/
no-red centering tolerance is ±20 cm; at a 45° diamond it collapses to ±~5 cm, so
aligning buys ~4× the margin (low gain; overrides heading hold while engaged; the
square is 90°-symmetric so `er` is reported in ±45° and drives to the nearest of
4 equivalent orientations — no spin). Park the arm at a fixed pose during the
hold so the camera extrinsics stay constant.

## Control behaviour & tuning

`StationKeepController` runs one PID per configured `StationKeepAxis`, each
mapping one error component to one thrust DOF. Defaults (in
`default_station_keep_axes`) provide **sway←ex**, **surge←es**, a gentle
**heave←es** size-trim axis, and a **yaw←er** square-up axis, all shipping with
**zero gains** — inert until tuned, so it is safe to enable while iterating. For
the transect task the recommended `rov_config` block overrides surge to `ey` and
gives the four axes their starting gains (see the mapping table above).

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

## Pilot controls (topside, built)

The operator UI already has the CV-era controls wired:

- **Engage/disengage:** `Autopilot > Optical Hold (Station-Keep)` menu item or the
  **K** key. Engaging is safe with no CV running -- the ROV stays inert (NO LOCK ->
  manual) until a valid lock arrives. Backed by
  `PilotService.set_station_keep_enabled` / `toggle_station_keep`, which sets
  `modes["autopilot"]["station_keep"]` in the published pilot command.
- **Status readout:** the drive-status bar shows `Optical Hold: OFF / ON (no data)
  / NO LOCK / STALE / LOCK / ACTIVE`, driven by the ROV's
  `status["station_keep"]["reason"]`.
- **CV integration point:** the future tracker calls
  `MainWindow.publish_visual_target(sample)` each frame (accepts a
  `VisualTargetError`, a `StationKeepCommand`, or a raw payload dict). It calls
  `PilotService.set_visual_target(...)` (rides the normal pilot frame) and logs the
  sample to the capture `tracking` stream. A `NullOpticalTracker` placeholder is
  instantiated until the real model is dropped in.

## Not done yet (next steps)
- The **CV detector** (`OpticalTracker` implementation): pixels -> blue-square
  geometry + red presence -> `TransectObservation`. Plan: classical first
  (HSV/Lab blue-square fit; red detection with the fixed lower-frame **gripper
  ROI masked out**, since the gripper is the same orange-red and must not trip
  `violation`); a learned model later via the direct-thrust path. The
  `TransectPolicy` (geometry -> error + lock) is built and tested, so the
  detector only has to fill a `TransectObservation`.
- Calibrate `TransectModel.target_cy` / `target_blue_fraction` for the oblique
  arm cam from recorded on-station frames (defaults are nadir-correct).
- A topside **frame source** for the tracker: the live display path is
  gst-launch -> d3d11 (no pixel access in Python), so the CV needs its own raw
  receiver on the transect/arm camera (e.g. a mirror-port raw pull, like the
  recorder) before `publish_visual_target` can be driven from real frames.
- A **transect-tab overlay** (blue box, target box, margins, green lock light)
  driven by `TransectEstimate`, so the pilot can trust the lock before engaging.
- Pool tuning of the `STATION_KEEP_*` gains/signs (verify each DOF's direction).
