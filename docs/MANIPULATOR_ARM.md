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
RB + right stick
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

`arm_gain` (pilot keys 6/7) scales the *speed* of the pilot-side controller-stick
integrator. It no longer caps the reachable range on the ROV. Keyboard `A`
walks the configured park pose through a slower Pilot-side park rate; keyboard
`W`/`S`/`D` are not bound to manipulator motion.

## Kinematics & reachable range

With a 1:1 differential each servo travels `pitch ± wrist` degrees and is limited
to `±SERVO_RANGE` (`GRIPPER_SERVO_RANGE_DEG`, **100° now** that the servos are
reprogrammed; 70° on the older config). Working in deviations from the
servo-center pose:

```
s_L = Δpitch + Δwrist
s_R = Δpitch − Δwrist
reachable region:  |Δpitch| + |Δwrist| ≤ SERVO_RANGE      (a diamond)
```

The pitch arc (0–90°) needs `±45°` and the wrist span (0–90°) needs `±45°`, so the
worst-case corner (full pitch **and** full wrist at once) needs `45 + 45 = 90°` of
servo deviation. At **±100°** that fits with ~10° to spare, so the whole square is
reachable; at the legacy ±70° it did not (`90° > 70°`).

With **±100°** and a 90° pitch span, a symmetric `N = 45°` neutral gives **full 90°
wrist at every pitch angle** — no taper, because the worst-case deviation (45° pitch
+ 45° wrist = 90°) stays inside the ±100° budget. **Pitch always reaches 90°** too.
The *pitch-priority* clip still runs but effectively never bites.

The current build therefore uses the symmetric, center-of-square neutral:

- `GRIPPER_SERVO_RANGE_DEG = 100.0`
- `GRIPPER_PITCH_SPAN_DEG = 90.0`, `GRIPPER_PITCH_NEUTRAL_DEG = 45.0`
- `GRIPPER_WRIST_SPAN_DEG = 90.0`, `GRIPPER_WRIST_NEUTRAL_DEG = 45.0`

`GRIPPER_SERVO_PULSE_HALFSPAN_US` stays at the measured pulse half-span (800 µs), so
reprogramming the servo to ±100° only remaps the fixed 700–2300 µs travel to more
physical degrees — the PWM endpoints do not move.

> **Legacy ±70° config.** With only ±70° of travel the corner was unreachable
> (`90° > 70°`), so the build used a *down-biased* `N = 70°` neutral to keep full
> wrist through the working/down half of the arc and traded away wrist while folded
> (a symmetric `N = 45°` there gives full wrist only over a 50°-wide band,
> `|Δpitch| ≤ 25°`, tapering to `±25°` at the pitch extremes). To revert, set
> `GRIPPER_SERVO_RANGE_DEG = 70.0` and `GRIPPER_PITCH_NEUTRAL_DEG = 70.0`.

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

At **±100°** the neutral is no longer a trade-off: a symmetric
`GRIPPER_PITCH_NEUTRAL_DEG = 45.0` already reaches full wrist across the entire
0–90° pitch arc, so just center it (45°) and you have the whole square. The lever
below only matters on the **legacy ±70°** servos, where there isn't enough budget
for the full square and `GRIPPER_PITCH_NEUTRAL_DEG` (the pitch angle at servo-center)
slides a 50°-wide full-wrist band along the arc:

| `GRIPPER_PITCH_NEUTRAL_DEG` (at ±70°) | Full-wrist pitch band | Wrist at pitch 90° |
|---|---|---|
| 45 (symmetric)   | 20°–70° (widest) | ±25° |
| ~65 (down-biased)| 40°–90°          | full ±45° |
| 70               | 45°–90°          | full ±45° |

On ±70° bias the neutral toward the angles where you actually need full wrist (usually
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

Servo: the SER-2010 is a Hitec **D954SW** reprogrammed to **±100°** (was ±70°). The
measured pulse range is about 700-2300 us and does not change with the reprogramming,
so the config keeps `GRIPPER_SERVO_PULSE_HALFSPAN_US = 800.0` and derives
`GRIPPER_US_PER_DEG` from the programmed servo range (800/100 ≈ 8.0 µs/deg). If the arm
cannot reach the commanded endpoint, re-measure the pulse half-span; if the last bit of
stick is dead, reduce it.

## Assembly: aligning the servos & mounting the connector

The single rule: **at both servos' electrical center (1500 µs) the differential is
at its neutral pose.** So mount the connector + arm while the servos are centered,
positioning the arm at the neutral you want — then `±100°` of each servo spreads
symmetrically about that neutral (`φ_L = Δpitch + Δwrist`, `φ_R = Δpitch − Δwrist`).

With the servos reprogrammed to **±100°** the neutral is simply the **middle of the
square**: pitch `N = 45°` and wrist centered. At that neutral the full `(pitch, wrist)`
square is reachable, so there is no band to position — just center everything.

1. **Pick the neutral pitch `N = 45°`** (permanent mechanical choice; the middle of
   the 0°–90° arc). The wrist neutral is always centered (mid of its 0–90° roll). On
   ±100° any `N` reaches full wrist everywhere, so 45° is chosen simply to keep the
   clip and travel symmetric about center.
2. **Center and hold both servos** so you can bolt the connector on with them locked:
   ```
   ssh triton@tritonpi.local
   sudo .venv/bin/python -m tools.gripper_calibrate --align
   ```
   Both servos drive to center and hold until Ctrl+C. The same alignment poses
   are now available on TritonPilot's Vehicle Setup page. The terminal tool also
   accepts named references:
   ```
   sudo .venv/bin/python -m tools.gripper_calibrate --align --align-pose center
   sudo .venv/bin/python -m tools.gripper_calibrate --align --align-pose flat-wrist-90
   sudo .venv/bin/python -m tools.gripper_calibrate --align --align-pose flat-wrist-0
   ```
3. **Mount the connector + arm** so that, at this centered pose, the arm sits at
   pitch `45°` (halfway between flat and straight-down) and wrist centered. If the
   horns are splined and you can't land exactly on `45°`, get as close as possible and
   trim the rest with `GRIPPER_TRIM_US` / `GRIPPER_SERVO_CENTER_US`.
4. **Confirm `GRIPPER_PITCH_NEUTRAL_DEG = 45.0`** in [rov_config.py](../rov_config.py)
   (already the default) so the software neutral matches the mounted neutral — they
   must agree or the pitch-priority clip and ROM will be skewed.
5. **Verify:** arm the vehicle and sweep pitch 0°→90° (should reach both ends) and
   wrist 0°→90° across the **whole** arc — at ±100° wrist stays full even at the pitch
   extremes (no taper).

If you ever revert the servos to ±70°, set `GRIPPER_SERVO_RANGE_DEG = 70` and a
down-biased `GRIPPER_PITCH_NEUTRAL_DEG = 70`; the centered-mount pitch then becomes
70° and wrist tapers near the pitch limits (see "Choosing the neutral").

### Flat reference alignment poses

For the current +/-100 deg setup with pitch neutral 45 deg, wrist neutral 45 deg,
servo center 1500 us, and pulse half-span 800 us:

| Pose | Mechanism target | Servo angle targets before pulse trim |
| --- | --- | --- |
| `center` | pitch 45 deg, wrist 45 deg | left 0 deg, right 0 deg -> 1500 us / 1500 us |
| `flat-wrist-90` | pitch 0 deg, wrist 90 deg | left 0 deg, right +90 deg with the current right-servo invert -> 1500 us / 2220 us |
| `flat-wrist-0` | pitch 0 deg, wrist 0 deg | left -90 deg, right 0 deg with the current right-servo invert -> 780 us / 1500 us |

The raw differential before the configured right-servo invert is:
`servo_left = delta_pitch + delta_wrist`, `servo_right = delta_pitch - delta_wrist`.
At pitch 0 / wrist 90, that is `0 deg` and `-90 deg`; the current
`GRIPPER_RIGHT_INVERT = -1.0` turns the right command into a +90 deg pulse command.

The configured arm/disarm park pose is `flat-wrist-90`
(`GRIPPER_DISARM_PITCH = -1.0`, `GRIPPER_DISARM_YAW = 1.0`) and
`GRIPPER_HOLD_PWM_ON_DISARM = True`, so the servos keep holding that tucked pose
after a disarm. If wrist 0 packages better on the real hardware, switch
`GRIPPER_DISARM_YAW` and `GRIPPER_ARM_YAW` to `-1.0`; Vehicle Setup can now edit
and save those normalized park-pose values. Vehicle Setup also refreshes the
topside keyboard `A` park target from the ROV config. For a fallback before the
management page has refreshed, set TritonPilot's `TRITON_ARM_PARK_WRIST` and
`TRITON_ARM_INIT_WRIST` to the same value.

## Smoothness

The servo (aux) outputs are rate-limited by `GRIPPER_SLEW_NORM_PER_S` (normalized
units/sec) in `ThrustWriter` ([motion/pwm.py](../motion/pwm.py)). It is set high
enough to feel instant (~full travel in 0.33 s) while absorbing per-frame jitter.
Explicit TritonOS arm/disarm park writes use `GRIPPER_PARK_SLEW_NORM_PER_S` and
`GRIPPER_PARK_SETTLE_S` so the tucked pose can move more gently without slowing
every live arm command. Primary live-motion smoothing still comes from the
pilot-side position integrator.
