# Differential wrist / arm

Two BlueTrail Engineering servos drive a **1:1 differential** that gives the
manipulator two degrees of freedom:

- **pitch** — the arm pitches from `0°` (flat/folded against the ROV) to `90°`
  (pointing straight down),
- **wrist** — roll of the end effector, `0°…90°`.

The continuous-rotation T200 "wrist rotate" motor (aux channel 10) is a separate
output driven by the triggers and is not part of this differential.

## Channels & control path

| Output | Wire key | Config channel | Role |
|---|---|---|---|
| servo_left  | `gripper_left`  | `GRIPPER_LEFT_PWM_CHANNEL`  | `pitch + wrist` |
| servo_right | `gripper_right` | `GRIPPER_RIGHT_PWM_CHANNEL` | `pitch − wrist` |

```
TritonPilot                              TritonOS
-----------                              --------
RB + right stick / W A S D
  -> arm position integrator
  -> PilotFrame.aux["gripper_pitch"]     ControlService._compute_gripper_diff
                    ["gripper_yaw"]   ->   _diff_mix_norm (degrees, pitch-priority clip)
       (absolute position, -1..+1)     ->  gripper_left / gripper_right  (-1..+1)
                                        ->  ThrustWriter aux (signed PWM + slew limit)
                                        ->  servo µs
```

`gripper_pitch` / `gripper_yaw` are **absolute position** commands in `[-1, +1]`:

- `gripper_pitch` −1 → `0°` (flat), +1 → `GRIPPER_PITCH_SPAN_DEG` (90° down in the current config).
- `gripper_yaw`   −1 → `0°`, +1 → `GRIPPER_WRIST_SPAN_DEG` (90° roll).

`arm_gain` (pilot keys 6/7) scales the *speed* of the pilot-side integrator. It no
longer caps the reachable range on the ROV.

## Kinematics & reachable range

With a 1:1 differential each servo travels `pitch ± wrist` degrees and is limited
to `±SERVO_RANGE` (`GRIPPER_SERVO_RANGE_DEG`, 70° now / 100° after reprogramming).
Working in deviations from the servo-center pose:

```
s_L = Δpitch + Δwrist
s_R = Δpitch − Δwrist
reachable region:  |Δpitch| + |Δwrist| ≤ SERVO_RANGE      (a diamond)
```

The pitch arc (0–90°) needs `±45°` and the wrist span (0–90°) needs `±45°`. The
only unreachable combination is the corner (full pitch **and** full wrist at once),
which needs `45 + 45 = 90° > 70°`.

With **±70°** and a 90° pitch span, a symmetric `N = 45°` neutral gives:

- **Full 90° wrist is available wherever `|Δpitch| ≤ 25°`** — a 50°-wide pitch band.
- Toward the pitch extremes wrist tapers to `±25°` (a 50° wrist range at the limit).
- **Pitch can always reach 90°** (wrist sacrificed there) — the *pitch-priority* clip.

The current build deliberately uses `GRIPPER_PITCH_SPAN_DEG = 90.0` and
`GRIPPER_PITCH_NEUTRAL_DEG = 70.0`. That trades away wrist while folded, but keeps
full wrist through the working/down part of the arm travel. The older 140° pitch
span used the whole servo budget for pitch and made wrist vanish at command
endpoints.

With **±100°** the whole `(pitch, wrist)` square is reachable: full wrist at full
pitch everywhere. After reprogramming, set `GRIPPER_SERVO_RANGE_DEG = 100.0` and
`GRIPPER_PITCH_NEUTRAL_DEG = 45.0`. Keep `GRIPPER_SERVO_PULSE_HALFSPAN_US` at the measured pulse half-span so the PWM endpoints stay fixed while physical degrees change.

### Mechanism & per-servo inversion (important)

The drive is a **bevel-gear differential**: the two servos face each other and turn
two side gears; a perpendicular bevel gear meshing between them is the output (the
arm shaft). Because the servos are **mirrored**, the raw mapping is the reverse of
the mixer's default:

- both servos commanded the **same** way → output **rolls** (wrist),
- servos commanded **opposite** → output **pitches**.

So one servo must be inverted to un-swap pitch and roll. Set `GRIPPER_RIGHT_INVERT`
(or `GRIPPER_LEFT_INVERT`) `= -1.0` for exactly one servo; then use
`GRIPPER_PITCH_INVERT` / `GRIPPER_YAW_INVERT` for per-axis direction. Determine all
three on the bench:

```
sudo .venv/bin/python -m tools.gripper_calibrate --check-axes
```

It drives a same-direction move then an opposite move and tells you what to set
based on whether each pitched or rolled. Symptoms of a missing inversion: pitch and
roll feel swapped, the arm can't reach flat, and range feels wrong.

### Choosing the neutral (range-of-motion lever)

There is 140° of pitch authority (±70°) for a 90° span, so the spare 50° slides the
full-wrist band along the arc via `GRIPPER_PITCH_NEUTRAL_DEG` (the pitch angle at
servo-center; set it physically with the horn index and trim it in software):

| `GRIPPER_PITCH_NEUTRAL_DEG` | Full-wrist pitch band | Wrist at pitch 90° |
|---|---|---|
| 45 (symmetric)   | 20°–70° (widest) | ±25° |
| ~65 (down-biased)| 40°–90°          | full ±45° |
| 70 (current)     | 45°–90°          | full ±45° |

Bias the neutral toward the angles where you actually need full wrist (usually
pointing down for manipulation); accept reduced wrist where the arm is stowed.

The clip is implemented in `ControlService._diff_mix_norm_deg`
([control/control_service.py](../control/control_service.py)) and is the single
source of truth (live commands, init, and the arm/disarm park pose all use it).

## Calibration

Run with the arm clear of obstructions and a hand near Ctrl+C:

```
ssh triton@tritonpi.local
sudo .venv/bin/python -m tools.gripper_calibrate
```

The wizard (`tools/gripper_calibrate.py`) ramps the servos smoothly and walks you
through: (1) set the neutral pulse `GRIPPER_SERVO_CENTER_US`, (2) measure
`GRIPPER_US_PER_DEG` from a known pitch jog, (3) confirm `GRIPPER_SERVO_RANGE_DEG`,
(4) sanity-check the wrist axis. It then prints the `rov_config.py` block and the
full-wrist band for several candidate neutrals. Paste the values into
[rov_config.py](../rov_config.py) section 9 and re-run to confirm.

`GRIPPER_SERVO_MIN_US` / `MAX_US` are derived as
`CENTER_US ± SERVO_RANGE_DEG · US_PER_DEG`; the ThrustWriter aux mapping turns the
normalized `±1` servo command into those endpoints.

Servo: the SER-2010 is a Hitec **D954SW** programmed to **±70°**. This build's
measured pulse range is about 700-2300 us, so the config keeps
`GRIPPER_SERVO_PULSE_HALFSPAN_US = 800.0` and derives `GRIPPER_US_PER_DEG` from the programmed servo range. If the arm cannot reach the commanded endpoint,
re-measure the pulse half-span; if the last bit of stick is dead, reduce it.

## Assembly: aligning the servos & mounting the connector

The single rule: **at both servos' electrical center (1500 µs) the differential is
at its neutral pose.** So mount the connector + arm while the servos are centered,
positioning the arm at the neutral you want — then `±70°` of each servo spreads
symmetrically about that neutral (`φ_L = Δpitch + Δwrist`, `φ_R = Δpitch − Δwrist`).

1. **Pick the neutral pitch `N`** (permanent mechanical choice). The full-wrist band
   is ~50° wide for any `N` in 25°–65°; `N` just slides it along the arc. The
   current `N = 70°` choice clips the high side at 90° so the working/down pose
   keeps full wrist:
   - `N = 25°` → full wrist over pitch **0°–50°** (flat → mid, the shallow / reaching-out half).
   - `N = 45°` → full wrist 20°–70°.
   - **This build uses `N = 70°`** → full wrist over pitch **45°–90°** (the down/working half).
   The wrist neutral is always centered (mid of its 0–90° roll).
2. **Center and hold both servos** so you can bolt the connector on with them locked:
   ```
   ssh triton@tritonpi.local
   sudo .venv/bin/python -m tools.gripper_calibrate --align
   ```
   Both servos drive to center and hold until Ctrl+C.
3. **Mount the connector + arm** so that, at this centered pose, the arm sits at
   pitch `N` and wrist centered. If the horns are splined and you can't land exactly
   on `N`, get as close as possible and trim the rest with `GRIPPER_TRIM_US` /
   `GRIPPER_SERVO_CENTER_US`.
4. **Set `GRIPPER_PITCH_NEUTRAL_DEG = N`** in [rov_config.py](../rov_config.py) so the
   software neutral matches the mounted neutral (they must agree or the pitch-priority
   clip and ROM will be skewed).
5. **Verify:** arm the vehicle and sweep pitch 0°→90° (should reach both ends) and
   wrist 0°→90° across the arc (full in the band, tapering near the pitch limits).

If you later reprogram the servos to ±100°, set `GRIPPER_SERVO_RANGE_DEG = 100` and `GRIPPER_PITCH_NEUTRAL_DEG = 45`; the
centered-mount procedure changes to the middle of the square: pitch 45° and wrist 45°.

## Smoothness

The servo (aux) outputs are rate-limited by `GRIPPER_SLEW_NORM_PER_S` (normalized
units/sec) in `ThrustWriter` ([motion/pwm.py](../motion/pwm.py)). It is set high
enough to feel instant (~full travel in 0.33 s) while absorbing per-frame jitter
and park jumps; the slew history resets on arm/disarm so deliberate park moves are
not throttled. Primary motion smoothing comes from the pilot-side position
integrator; this slew is a backstop against wire jitter.
