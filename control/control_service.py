# rov/control/control_service.py
from __future__ import annotations

import argparse
import math
import time
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, Any, Mapping, Hashable

from schema.pilot_common import PilotFrame
from control.pilot_receiver import PilotReceiver
import rov_config as cfg
from motion.channel_map import ChannelMap

from control.mixer import EightThrusterMixer, SimpleGroupMixer, global_limit
from control.depth_hold import DepthHoldController, DepthHoldConfig
from control.sensor_tap import DepthSensorTap


@dataclass
class ControlGains:
    surge: float = 1.0
    sway: float = 1.0
    heave: float = 1.0
    yaw: float = 0.8
    pitch: float = 0.5
    roll: float = 0.5
    power_scale: float = 1.0


class ROVControlState:
    """
    Shared flags: armed, etc.
    """
    def __init__(self):
        self._armed = False
        self._lock = threading.Lock()

    def set_armed(self, val: bool):
        with self._lock:
            self._armed = bool(val)

    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

    def toggle_armed(self) -> bool:
        with self._lock:
            self._armed = not self._armed
            return self._armed


def build_6dof(pilot: PilotFrame, gains: ControlGains) -> Dict[str, float]:
    """Build a 6-DOF command dict from a PilotFrame.

    Axis mapping is intentionally configurable via rov_config.py so you can
    match whatever your controller/driver reports.

    Defaults preserve the original behavior:
      surge <- ly
      sway  <- lx
      yaw   <- rx
      heave <- ry

    You can override with:
      AXIS_SURGE, AXIS_SWAY, AXIS_YAW, AXIS_HEAVE
      and optional *_INVERT (1.0 or -1.0) plus AXIS_DEADZONE.

    Pitch/roll can be controlled via the D-pad (default) or by axes by setting:
      AXIS_PITCH, AXIS_ROLL  (e.g. "lx", "ry", "rt")

    Special axis names like "none"/"off" disable that DOF and return 0.0.
    """

    def dz(x: float, d: float) -> float:
        return 0.0 if abs(float(x)) < d else float(x)

    dzv = float(getattr(cfg, "AXIS_DEADZONE", 0.10))

    # Axis names (PilotFrame.axes fields)
    surge_axis = str(getattr(cfg, "AXIS_SURGE", "ly"))
    sway_axis = str(getattr(cfg, "AXIS_SWAY", "lx"))
    yaw_axis = str(getattr(cfg, "AXIS_YAW", "rx"))
    heave_axis = str(getattr(cfg, "AXIS_HEAVE", "ry"))

    # Invert scalars (+1 or -1)
    surge_inv = float(getattr(cfg, "AXIS_SURGE_INVERT", 1.0))
    sway_inv = float(getattr(cfg, "AXIS_SWAY_INVERT", 1.0))
    yaw_inv = float(getattr(cfg, "AXIS_YAW_INVERT", 1.0))
    heave_inv = float(getattr(cfg, "AXIS_HEAVE_INVERT", 1.0))

    def a(name: str) -> float:
        """Read an axis by name.

        Special values like "none" disable the axis and return 0.0.
        """
        try:
            s = str(name).strip().lower()
        except Exception:
            s = ""
        if s in ("", "none", "null", "off", "disabled"):
            return 0.0
        try:
            return float(getattr(pilot.axes, s, 0.0))
        except Exception:
            return 0.0

    surge = dz(a(surge_axis), dzv) * surge_inv * gains.surge
    sway = dz(a(sway_axis), dzv) * sway_inv * gains.sway
    yaw = dz(a(yaw_axis), dzv) * yaw_inv * gains.yaw
    heave = dz(a(heave_axis), dzv) * heave_inv * gains.heave

    # Pitch/roll can come from the D-pad (default) or from configurable axes.
    pitch_axis = getattr(cfg, "AXIS_PITCH", None)
    roll_axis = getattr(cfg, "AXIS_ROLL", None)
    pitch_inv = float(getattr(cfg, "AXIS_PITCH_INVERT", 1.0))
    roll_inv = float(getattr(cfg, "AXIS_ROLL_INVERT", 1.0))

    dpx, dpy = pilot.dpad

    if pitch_axis is None or str(pitch_axis).strip().lower() in ("dpad", "dpad_y", "hat", "hat_y"):
        pitch = float(dpy) * gains.pitch
    else:
        pitch = dz(a(str(pitch_axis)), dzv) * pitch_inv * gains.pitch

    if roll_axis is None or str(roll_axis).strip().lower() in ("dpad", "dpad_x", "hat", "hat_x"):
        roll = float(dpx) * gains.roll
    else:
        roll = dz(a(str(roll_axis)), dzv) * roll_inv * gains.roll

    k = gains.power_scale
    return {
        "surge": surge * k,
        "sway": sway * k,
        "heave": heave * k,
        "yaw": yaw * k,
        "pitch": pitch * k,
        "roll": roll * k,
    }


def build_2axis(pilot: PilotFrame, gains: ControlGains) -> Dict[str, float]:
    """Build a minimal command dict for bring-up.

    Uses only:
      - surge: axis cfg.AXIS_SURGE (default ly)
      - heave: axis cfg.AXIS_HEAVE (default ry)

    This is intentionally simple so you can validate all channels and motor directions.
    """
    def dz(x: float, d: float) -> float:
        return 0.0 if abs(float(x)) < d else float(x)

    surge_axis = getattr(cfg, "AXIS_SURGE", "ly")
    heave_axis = getattr(cfg, "AXIS_HEAVE", "ry")
    dzv = float(getattr(cfg, "AXIS_DEADZONE", 0.10))

    surge_raw = float(getattr(pilot.axes, surge_axis, 0.0))
    heave_raw = float(getattr(pilot.axes, heave_axis, 0.0))

    surge = dz(surge_raw, dzv) * float(getattr(cfg, "AXIS_SURGE_INVERT", 1.0)) * gains.surge
    heave = dz(heave_raw, dzv) * float(getattr(cfg, "AXIS_HEAVE_INVERT", 1.0)) * gains.heave

    k = gains.power_scale
    return {"surge": surge * k, "heave": heave * k}


def clamp01(x: float) -> float:
    x = float(x)
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


def axis_to_01(axis_name: str, raw: float) -> float:
    """Convert an axis value into [0..1] for aux controls.

    Triggers (lt/rt) are already normalized to [0..1] by TritonPilot.
    Sticks are typically [-1..1] and we map them to [0..1] via (x+1)/2.
    """
    axis_name = str(axis_name).strip().lower()
    v = float(raw)
    if axis_name in ("lt", "rt"):
        return clamp01(v)
    return clamp01((v + 1.0) * 0.5)



class ControlService:
    """
    Periodic loop (your original structure) :contentReference[oaicite:5]{index=5}, with:
      - robust arming toggle based on button edges
      - dry-run printing for easy ROV-side testing
      - clearer logs (pilot age, reason, cmd6, thr)
    """
    def __init__(
        self,
        pilot_rx: PilotReceiver,
        gains: ControlGains,
        control_state: ROVControlState,
        rate_hz: float = 50.0,
        ttl: float = 0.5,
        debug: bool = False,
        dry_run: bool = True,
        log_every_s: float = 0.25,
        # Buttons can be single names or comma-separated lists.
        # Example: arm_toggle_button="menu,start" kill_button="win,b"
        arm_toggle_button: str = "menu",  # press to toggle armed
        kill_button: str = "win",         # press to force disarm (B is reserved for camera switching topside)
        # If set, the ROV will only remain armed while this button is held.
        # Example: deadman_button="rb". Set to "" to disable.
        deadman_button: str = "",
        # If pilot frames go stale while armed, automatically disarm after this many seconds.
        failsafe_disarm_s: float = 2.0,
    ):
        self.pilot_rx = pilot_rx
        self.gains = gains
        self.state = control_state
        # Baseline from rov_config/main_rov; pilot may apply a 0..1 cap multiplier on top.
        self._base_power_scale = float(getattr(gains, 'power_scale', 1.0))
        self._last_pilot_max_gain = 1.0
        self.period = 1.0 / float(rate_hz)
        self.ttl = float(ttl)
        self.debug = bool(debug)
        self.dry_run = bool(dry_run)
        self.log_every_s = float(log_every_s)

        # Parse configurable button lists (comma-separated)
        self.arm_toggle_button = arm_toggle_button
        self.kill_button = kill_button
        self._arm_buttons = [b.strip() for b in str(arm_toggle_button).split(",") if b.strip()]
        self._kill_buttons = [b.strip() for b in str(kill_button).split(",") if b.strip()]
        self.deadman_button = (deadman_button or "").strip()
        self.failsafe_disarm_s = float(failsafe_disarm_s)

        # Single source-of-truth for channel mapping (physical channels 1..16)
        self._chanmap = ChannelMap.from_config(cfg)

        self._mix_mode = str(getattr(cfg, 'CONTROL_MIX_MODE', 'six_dof')).strip().lower()
        if self._mix_mode == 'simple_groups':
            # Name-based bring-up mixing: surge -> all horizontals, heave -> all verticals
            self.mixer = SimpleGroupMixer(
                self._chanmap.horizontal_thrusters,
                self._chanmap.vertical_thrusters,
            )
        else:
            self.mixer = EightThrusterMixer()

        # Reverse lookup for bring-up modes that work in raw PWM channel space.
        # This lets us convert {channel: thrust} into {thruster_name: thrust} when possible.
        self._thruster_name_by_channel: Dict[int, str] = {}
        try:
            tc = getattr(cfg, "THRUSTER_CHANNELS", {}) or {}
            for name, ch in dict(tc).items():
                try:
                    ch_i = int(ch)
                except Exception:
                    continue
                if ch_i not in self._thruster_name_by_channel:
                    self._thruster_name_by_channel[ch_i] = str(name)
        except Exception:
            pass

        # Optional lights (aux PWM) control. The hardware mapping is handled by the PWM sink.
        self._lights_enabled = bool(getattr(cfg, "LIGHTS_ENABLE", hasattr(cfg, "LIGHTS_PWM_CHANNEL")))
        # Modes:
        #   - "toggle" (default for this project): L3 toggles a fixed brightness.
        #   - "axis": legacy trigger/axis brightness control.
        self._lights_mode = str(getattr(cfg, "LIGHTS_MODE", "toggle")).strip().lower()

        # Toggle-mode config
        self._lights_toggle_button = str(getattr(cfg, "LIGHTS_TOGGLE_BUTTON", "lstick")).strip()
        self._lights_default = float(getattr(cfg, "LIGHTS_DEFAULT", getattr(cfg, "LIGHTS_DEFAULT_BRIGHTNESS", 0.75)))
        self._lights_on = bool(getattr(cfg, "LIGHTS_ON_BY_DEFAULT", True))

        # Axis-mode config (kept for backwards-compat)
        self._lights_axis = str(getattr(cfg, "LIGHTS_AXIS", getattr(cfg, "LIGHTS_INPUT_AXIS", "rt")))
        self._lights_scale = float(getattr(cfg, "LIGHTS_SCALE", 1.0))
        self._lights_invert = bool(getattr(cfg, "LIGHTS_INVERT", False))
        self._lights_deadzone = float(getattr(cfg, "LIGHTS_DEADZONE", 0.02))

        # Whether we *include* lights commands while disarmed. Note: if your PWM sink
        # physically disables outputs while disarmed (recommended for thruster safety),
        # the lights will only actually illuminate when PWM outputs are enabled.
        self._lights_allow_when_disarmed = bool(getattr(cfg, "LIGHTS_ALLOW_WHEN_DISARMED", True))
        self._lights_failsafe_off = bool(getattr(cfg, "LIGHTS_FAILSAFE_OFF", False))

        # Wrist rotate (T200 on a dedicated channel). This is driven as a *thruster-style*
        # output (normalized [-1..1], neutral at 1500us) using trigger buttons for a fixed speed.
        wrist_ch = None
        try:
            wrist_ch = self._chanmap.aux.get("wrist_rotate")
        except Exception:
            wrist_ch = None
        self._wrist_rotate_enabled = bool(getattr(cfg, "WRIST_ROTATE_ENABLE", True)) and (wrist_ch is not None)
        self._wrist_rotate_cmd_key = str(getattr(cfg, "WRIST_ROTATE_CMD_KEY", "wrist_rotate"))
        self._wrist_rotate_right_axis = str(getattr(cfg, "WRIST_ROTATE_RIGHT_AXIS", "rt"))
        self._wrist_rotate_left_axis = str(getattr(cfg, "WRIST_ROTATE_LEFT_AXIS", "lt"))
        self._wrist_rotate_trigger_deadzone = float(getattr(cfg, "WRIST_ROTATE_TRIGGER_DEADZONE", 0.10))
        self._wrist_rotate_speed = float(getattr(cfg, "WRIST_ROTATE_SPEED", 0.20))

        self._gripper_enabled = bool(getattr(cfg, "GRIPPER_ENABLE", True))
        self._gripper_pitch_key = str(getattr(cfg, "GRIPPER_PITCH_CMD_KEY", "gripper_pitch"))
        self._gripper_yaw_key = str(getattr(cfg, "GRIPPER_YAW_CMD_KEY", "gripper_yaw"))
        self._gripper_left_key = str(getattr(cfg, "GRIPPER_LEFT_CMD_KEY", "gripper_left"))
        self._gripper_right_key = str(getattr(cfg, "GRIPPER_RIGHT_CMD_KEY", "gripper_right"))
        self._gripper_pitch_scale = float(getattr(cfg, "GRIPPER_PITCH_SCALE", 1.0))
        self._gripper_yaw_scale = float(getattr(cfg, "GRIPPER_YAW_SCALE", 1.0))
        self._gripper_pitch_invert = float(getattr(cfg, "GRIPPER_PITCH_INVERT", 1.0))
        self._gripper_yaw_invert = float(getattr(cfg, "GRIPPER_YAW_INVERT", 1.0))
        self._gripper_deadzone = float(getattr(cfg, "GRIPPER_DEADBAND", 0.01))
        arm_pitch = float(getattr(cfg, "GRIPPER_ARM_PITCH", getattr(cfg, "GRIPPER_DISARM_PITCH", 0.0)) or 0.0)
        arm_yaw = float(getattr(cfg, "GRIPPER_ARM_YAW", getattr(cfg, "GRIPPER_DISARM_YAW", 0.0)) or 0.0)
        self._gripper_park_left, self._gripper_park_right = self._mix_gripper_axes(arm_pitch, arm_yaw)
        # For servos, "no input" should normally mean "hold last commanded position"
        # rather than springing back to center. This latches the last mixed differential
        # output until a new pitch/yaw command arrives.
        self._gripper_hold_last = bool(getattr(cfg, "GRIPPER_HOLD_LAST_POSITION", True))
        self._gripper_last_left = self._gripper_park_left
        self._gripper_last_right = self._gripper_park_right

        # Optional hardware sink. If set, it will be called with a dict
        # like {"H_FL": 0.1, ...} every control tick.
        #
        # Supported forms:
        #   - callable(thr_dict)
        #   - object with .write(thr_dict)
        #   - object with .neutral() (optional)
        self._hw_sink = None

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_log = 0.0
        self._last_seq_seen: int = -1

        # Used for stale-frame auto-disarm.
        self._stale_since: Optional[float] = None

        # Track whether we have physically enabled PWM outputs (OE) when using
        # a ThrustWriter-like sink that supports arm/disarm.
        self._sink_armed: bool = False

        # One-time warnings to make failures obvious in logs
        self._warned_no_sink: bool = False
        self._warned_dry_run: bool = False
        self._warned_sink_disarmed: bool = False

        # --- arming safety -------------------------------------------------
        # When you press ARM, we require that sticks are centered and triggers
        # are at rest. This prevents the classic “ARM -> instant full thrust”
        # problem caused by mismatched SDL axis numbering (e.g., a trigger axis
        # accidentally mapped into a stick).
        self._arm_require_neutral: bool = bool(getattr(cfg, "ARM_REQUIRE_NEUTRAL", True))
        self._arm_center_tol: float = float(getattr(cfg, "ARM_CENTER_TOL", 0.18))
        self._arm_trigger_tol: float = float(getattr(cfg, "ARM_TRIGGER_TOL", 0.10))
        self._arm_ramp_s: float = float(getattr(cfg, "ARM_RAMP_S", 0.35))
        self._armed_since: Optional[float] = None

        # Track & log lights toggles (edge-based)
        self._last_lights_event: Optional[str] = None

        # --- depth hold ----------------------------------------------------
        self._depth_hold: Optional[DepthHoldController] = None
        self._depth_tap: Optional[DepthSensorTap] = None
        self._last_depth_status: Dict[str, Any] = {}

        if bool(getattr(cfg, "DEPTH_HOLD_ENABLE", True)):
            try:
                # Subscribe locally to the sensor PUB stream.
                self._depth_tap = DepthSensorTap(getattr(cfg, "SENSOR_PUB_ENDPOINT", "tcp://0.0.0.0:6001"))
            except Exception as e:
                if self.debug:
                    print("[rov/control] depth hold disabled (sensor tap init failed):", e)
                self._depth_tap = None

            try:
                dh_cfg = DepthHoldConfig(
                    sensor_stale_s=float(getattr(cfg, "DEPTH_HOLD_SENSOR_STALE_S", 0.6)),
                    depth_lpf_tau_s=float(getattr(cfg, "DEPTH_HOLD_LPF_TAU_S", 0.50)),
                    kp=float(getattr(cfg, "DEPTH_HOLD_KP", 0.30)),
                    ki=float(getattr(cfg, "DEPTH_HOLD_KI", 0.05)),
                    kd=float(getattr(cfg, "DEPTH_HOLD_KD", 0.00)),
                    error_deadband_m=float(getattr(cfg, "DEPTH_HOLD_ERROR_DEADBAND_M", 0.03)),
                    i_limit=float(getattr(cfg, "DEPTH_HOLD_I_LIMIT", 0.25)),
                    out_limit=float(getattr(cfg, "DEPTH_HOLD_OUT_LIMIT", 0.55)),
                    sign=float(getattr(cfg, "DEPTH_HOLD_SIGN", 1.0)),
                    walk_target=bool(getattr(cfg, "DEPTH_HOLD_WALK_TARGET", True)),
                    walk_deadband=float(getattr(cfg, "DEPTH_HOLD_WALK_DEADBAND", 0.08)),
                    walk_rate_mps=float(getattr(cfg, "DEPTH_HOLD_WALK_RATE_MPS", 0.60)),
                    target_min_m=getattr(cfg, "DEPTH_HOLD_TARGET_MIN_M", None),
                    target_max_m=getattr(cfg, "DEPTH_HOLD_TARGET_MAX_M", None),
                )
                self._depth_hold = DepthHoldController(dh_cfg)
            except Exception as e:
                if self.debug:
                    print("[rov/control] depth hold disabled (config/init failed):", e)
                self._depth_hold = None

    def _handle_lights_toggle(self, pilot: PilotFrame) -> Optional[str]:
        """Handle edge-based lights toggling.

        We intentionally keep this separate from arming so you can toggle lights
        without affecting vehicle safety state.
        """
        if not self._lights_enabled:
            return None
        if self._lights_mode not in ("toggle", "button", "l3"):
            return None
        b = (self._lights_toggle_button or "").strip()
        if not b:
            return None
        edges = pilot.edges or {}
        if edges.get(b) == "down":
            self._lights_on = not self._lights_on
            return f"LIGHTS ({b}) -> {'ON' if self._lights_on else 'OFF'}"
        return None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        sink = self._hw_sink
        sink_name = type(sink).__name__ if sink is not None else 'None'
        print(f"[rov/control] started mode={self._mix_mode} rate={1.0/self.period:.1f}Hz ttl={self.ttl:.2f}s dry_run={self.dry_run} sink={sink_name}")

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

        # Best-effort: drive neutral on shutdown.
        try:
            self._send_to_hw({})
        except Exception:
            pass

        # Best-effort: physically disable outputs.
        try:
            self.state.set_armed(False)
            self._set_gripper_park_pose()
            self._sync_sink_armed(force=True)
        except Exception:
            pass

    def set_hw_sink(self, sink) -> None:
        """Attach a hardware sink.

        Example:
            from motion.pwm import ThrustWriter
            tw = ThrustWriter(); tw.arm()
            ctrl.set_hw_sink(tw)
        """
        self._hw_sink = sink

        sink_name = type(sink).__name__
        print(f"[rov/control] hardware sink attached: {sink_name} (dry_run={self.dry_run})")

        # Ensure physical outputs start disabled unless we're already armed.
        self._sync_sink_armed(force=True)

    def _sync_sink_armed(self, force: bool = False) -> None:
        """If the sink supports arm/disarm, keep it in sync with control_state.

        This controls physical enabling (e.g., Navigator PWM_OE). When disarmed
        we also try to send neutral before disabling outputs.
        """
        sink = self._hw_sink
        if sink is None:
            return
        has_arm = hasattr(sink, "arm") and hasattr(sink, "disarm")
        if not has_arm:
            return

        desired = bool(self.state.is_armed()) and (not self.dry_run)
        if (not force) and desired == self._sink_armed:
            return

        try:
            if desired:
                sink.arm()  # type: ignore[attr-defined]
            else:
                # best-effort: neutral then disable outputs
                try:
                    if hasattr(sink, "write"):
                        sink.write({})  # type: ignore[attr-defined]
                except Exception:
                    pass
                sink.disarm()  # type: ignore[attr-defined]
        except Exception as e:
            if self.debug:
                print("[rov/control] sink arm/disarm failed:", e)
        else:
            self._sink_armed = desired

    def _handle_arming(self, pilot: PilotFrame) -> Optional[str]:
        """
        Returns a human-readable arming event string, or None.
        Uses pilot.edges if available (PilotReceiver now computes them).
        """
        edges = pilot.edges or {}

        def _axes_centered() -> tuple[bool, str]:
            """Return (ok, reason)."""
            a = pilot.axes
            # Sticks should be near 0.0, triggers should be near 0.0 (topside normalizes to [0..1]).
            sticks = {
                "lx": float(getattr(a, "lx", 0.0)),
                "ly": float(getattr(a, "ly", 0.0)),
                "rx": float(getattr(a, "rx", 0.0)),
                "ry": float(getattr(a, "ry", 0.0)),
            }
            lt = float(getattr(a, "lt", 0.0))
            rt = float(getattr(a, "rt", 0.0))

            # Non-finite values are always unsafe.
            if any((not math.isfinite(v)) for v in list(sticks.values()) + [lt, rt]):
                return False, "non-finite axis value(s)"

            max_stick = max(abs(v) for v in sticks.values()) if sticks else 0.0
            ok = (max_stick <= self._arm_center_tol) and (lt <= self._arm_trigger_tol) and (rt <= self._arm_trigger_tol)
            reason = f"sticks max={max_stick:.2f} (tol={self._arm_center_tol:.2f}) lt={lt:.2f} rt={rt:.2f}"
            return ok, reason

        # Helper: read current button level (for cases where an edge is missed)
        def is_pressed(name: str) -> bool:
            return bool(getattr(pilot.buttons, name, False))

        # KILL / DISARM: if any kill button is pressed OR edge-down detected.
        for b in self._kill_buttons:
            if edges.get(b) == "down" or is_pressed(b):
                if self.state.is_armed():
                    self.state.set_armed(False)
                    self._armed_since = None
                    self._set_gripper_park_pose()
                    self._sync_sink_armed(force=True)
                    return f"KILL ({b}) -> DISARMED"
                return f"KILL ({b}) (already disarmed)"

        # Toggle arm: edge-down on any configured toggle button
        for b in self._arm_buttons:
            if edges.get(b) == "down":
                # If we are currently DISARMED and about to arm, enforce a "sticks centered" gate.
                if not self.state.is_armed():
                    if self._arm_require_neutral:
                        ok, why = _axes_centered()
                        if not ok:
                            # Stay disarmed.
                            self.state.set_armed(False)
                            self._armed_since = None
                            self._set_gripper_park_pose()
                            self._sync_sink_armed(force=True)
                            return f"REFUSED ARM ({b}): {why}"
                    self.state.set_armed(True)
                    hold_s = float(getattr(cfg, "ARM_HW_INIT_HOLD_S", 0.0) or 0.0)
                    self._armed_since = time.time() + hold_s
                    self._set_gripper_park_pose()
                    self._sync_sink_armed(force=True)
                    return f"TOGGLE ({b}) -> ARMED"
                else:
                    self.state.set_armed(False)
                    self._armed_since = None
                    self._set_gripper_park_pose()
                    self._sync_sink_armed(force=True)
                    return f"TOGGLE ({b}) -> DISARMED"

        return None

    def _run(self):
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            pilot, age = self.pilot_rx.get_latest()  # we want edges even if stale

            arming_event = None
            lights_event = None
            if pilot is not None and pilot.seq != self._last_seq_seen:
                arming_event = self._handle_arming(pilot)
                if arming_event:
                    print(f"[rov/control] {arming_event}")

                lights_event = self._handle_lights_toggle(pilot)
                if lights_event:
                    print(f"[rov/control] {lights_event}")
                self._last_seq_seen = pilot.seq

            fresh_pilot = None
            fresh_age = age
            if pilot is not None and age <= self.ttl:
                fresh_pilot = pilot

            # Auto-disarm if we're armed but pilot frames go stale for too long.
            if self.state.is_armed() and self.failsafe_disarm_s > 0:
                if fresh_pilot is None:
                    if self._stale_since is None:
                        self._stale_since = now
                    elif (now - self._stale_since) >= self.failsafe_disarm_s:
                        self.state.set_armed(False)
                        self._armed_since = None
                        self._set_gripper_park_pose()
                        self._sync_sink_armed(force=True)
                        arming_event = f"FAILSAFE (stale>{self.failsafe_disarm_s:.1f}s) -> DISARMED"
                else:
                    self._stale_since = None

            # Deadman: if enabled, you must hold the button to remain armed.
            if self.state.is_armed() and self.deadman_button and fresh_pilot is not None:
                if not bool(getattr(fresh_pilot.buttons, self.deadman_button, False)):
                    self.state.set_armed(False)
                    self._armed_since = None
                    self._set_gripper_park_pose()
                    self._sync_sink_armed(force=True)
                    arming_event = f"DEADMAN ({self.deadman_button}) released -> DISARMED"

            if fresh_pilot is None or not self.state.is_armed():
                payload: Dict[Any, float] = {}
                lights_val: Optional[float] = self._compute_lights(fresh_pilot)
                if lights_val is not None and (self.state.is_armed() or self._lights_allow_when_disarmed):
                    payload["lights"] = float(lights_val)
                self._send_to_hw(payload)

                if self.debug and (now - self._last_log) > self.log_every_s:
                    reason = "no frames" if pilot is None else ("stale" if fresh_pilot is None else "DISARMED")
                    msg = f"[rov/control] NEUTRAL ({reason}, age={fresh_age:.3f}s armed={self.state.is_armed()})"
                    if arming_event:
                        msg += f" | {arming_event}"
                    if lights_event:
                        msg += f" | {lights_event}"
                    print(msg)
                    self._last_log = now
            else:
                # Pilot runtime max gain cap (Y/A on topside) scales overall power.
                self._apply_pilot_gain(fresh_pilot)

                if self._mix_mode == 'simple_groups':
                    cmd2 = build_2axis(fresh_pilot, self.gains)
                    raw_thr = self.mixer.mix(cmd2['surge'], cmd2['heave'])
                    thr = global_limit(raw_thr, max_abs=float(getattr(cfg, 'THRUSTER_MAX_ABS', 1.0)))
                    thr = self._channels_to_named(thr)
                else:
                    cmd6 = build_6dof(fresh_pilot, self.gains)

                    # Depth hold: modify heave command in-place before mixing.
                    if self._depth_tap is not None:
                        try:
                            self._depth_tap.poll()
                        except Exception:
                            pass
                    if self._depth_hold is not None:
                        modes = fresh_pilot.modes or {}
                        enabled_cmd = bool(modes.get("depth_hold", modes.get("depth_hold_enabled", False)))
                        depth_m = self._depth_tap.last_depth_m if self._depth_tap is not None else None
                        depth_age = self._depth_tap.age_s(now) if self._depth_tap is not None else None
                        try:
                            heave_out, st = self._depth_hold.step(
                                enabled=enabled_cmd,
                                manual_heave=float(cmd6.get("heave", 0.0)),
                                depth_m=depth_m,
                                depth_age_s=depth_age,
                                dt=self.period,
                            )
                            cmd6["heave"] = float(heave_out)
                            self._last_depth_status = st
                        except Exception as e:
                            # Fail open to manual heave if anything goes wrong.
                            self._last_depth_status = {"enabled_cmd": enabled_cmd, "active": False, "reason": f"err:{e}"}

                    raw_thr = self.mixer.mix(cmd6)
                    thr = global_limit(raw_thr, max_abs=float(getattr(cfg, 'THRUSTER_MAX_ABS', 1.0)))

                # Per-thruster deadband at the mix output (extra protection against creep)
                base_db = float(getattr(cfg, "MIX_OUTPUT_DEADBAND", 0.05))
                dh_db = float(getattr(cfg, "DEPTH_HOLD_MIX_DEADBAND", 0.02))
                dh_cmd = False
                try:
                    modes = fresh_pilot.modes or {}
                    dh_cmd = bool(modes.get("depth_hold", modes.get("depth_hold_enabled", False)))
                except Exception:
                    dh_cmd = False

                for k, v in list(thr.items()):
                    if isinstance(k, str) and k.strip().lower() == "lights":
                        continue

                    db = base_db
                    # When depth-hold is enabled, allow smaller vertical corrections.
                    if dh_cmd and isinstance(k, str) and k.strip().upper().startswith("V_"):
                        db = dh_db

                    if abs(float(v)) < float(db):
                        thr[k] = 0.0

                # Smooth ramp-in right after arming so even if a joystick axis is
                # slightly off-center you don't get a violent jump.
                ramp = 1.0
                if self._armed_since is not None and self._arm_ramp_s > 0.0:
                    dt = now - float(self._armed_since)
                    if dt <= 0.0:
                        ramp = 0.0
                    else:
                        ramp = max(0.0, min(1.0, dt / float(self._arm_ramp_s)))

                if ramp < 1.0:
                    for k in list(thr.keys()):
                        if isinstance(k, str) and k.strip().lower() == "lights":
                            continue
                        thr[k] = float(thr[k]) * float(ramp)
                payload: Dict[Any, float] = dict(thr)
                lights_val = self._compute_lights(fresh_pilot)
                if lights_val is not None:
                    payload["lights"] = float(lights_val)
                wrist_cmd = self._compute_wrist_rotate(fresh_pilot)
                if self._wrist_rotate_enabled:
                    payload[self._wrist_rotate_cmd_key] = float(wrist_cmd)
                if self._gripper_enabled:
                    gripper_left, gripper_right = self._compute_gripper_diff(fresh_pilot)
                    payload[self._gripper_left_key] = float(gripper_left)
                    payload[self._gripper_right_key] = float(gripper_right)
                self._send_to_hw(payload)

                if self.debug and (now - self._last_log) > self.log_every_s:
                    if self._mix_mode == "simple_groups":
                        msg = f"[rov/control] APPLY seq={fresh_pilot.seq} age={fresh_age:.3f}s cmd2={cmd2} thr={thr} | gain={self._last_pilot_max_gain*100:.0f}% (k={self.gains.power_scale:.2f})"
                    else:
                        msg = f"[rov/control] APPLY seq={fresh_pilot.seq} age={fresh_age:.3f}s cmd6={cmd6} thr={thr} | gain={self._last_pilot_max_gain*100:.0f}% (k={self.gains.power_scale:.2f})"
                        try:
                            st = self._last_depth_status or {}
                            if bool(st.get("enabled_cmd", False)):
                                msg += f" | depth_hold={'ON' if st.get('active') else 'OFF'}"
                                if st.get("active") and ("target_m" in st):
                                    msg += f" z={float(st.get('depth_f_m', 0.0)):.2f}m->t={float(st.get('target_m', 0.0)):.2f}m"
                        except Exception:
                            pass
                    if arming_event:
                        msg += f" | {arming_event}"
                    if lights_event:
                        msg += f" | {lights_event}"
                    print(msg)
                    self._last_log = now

            next_t += self.period
            sleep_for = next_t - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.time()

    def _pilot_gain_multiplier(self, pilot: Optional[PilotFrame]) -> float:
        """Return pilot-requested max gain multiplier in [0.05..1.0] (default 1.0)."""
        if pilot is None:
            return 1.0
        try:
            modes = pilot.modes or {}
        except Exception:
            return 1.0
        raw = None
        # Preferred key from TritonPilot. Accept a few aliases for compatibility.
        for k in ("max_gain", "pilot_max_gain", "power_scale_max"):
            if k in modes:
                raw = modes.get(k)
                break
        if raw is None:
            return 1.0
        try:
            v = float(raw)
        except Exception:
            return 1.0
        # Keep a non-zero floor to avoid confusing "armed but no thrust" behavior.
        if v < 0.05:
            v = 0.05
        if v > 1.0:
            v = 1.0
        return v

    def _apply_pilot_gain(self, pilot: Optional[PilotFrame]) -> None:
        """Update effective power scale using the pilot's runtime max gain cap."""
        mult = self._pilot_gain_multiplier(pilot)
        self._last_pilot_max_gain = float(mult)
        self.gains.power_scale = float(self._base_power_scale) * float(mult)

    @staticmethod
    def _neutral() -> Dict[str, float]:
        return {
            "H_FL": 0.0, "H_FR": 0.0, "H_RL": 0.0, "H_RR": 0.0,
            "V_FL": 0.0, "V_FR": 0.0, "V_RL": 0.0, "V_RR": 0.0,
        }

    def _channels_to_named(self, thr: Dict[Any, float]) -> Dict[Any, float]:
        """Convert channel-keyed thrust dicts into thruster-name keyed dicts when possible."""
        if not thr or not self._thruster_name_by_channel:
            return thr

        out: Dict[Any, float] = {}
        for k, v in thr.items():
            ch: Optional[int] = None
            if isinstance(k, int):
                ch = k
            elif isinstance(k, str) and k.strip().isdigit():
                try:
                    ch = int(k.strip())
                except Exception:
                    ch = None

            if ch is not None and ch in self._thruster_name_by_channel:
                out[self._thruster_name_by_channel[ch]] = float(v)
            else:
                out[k] = float(v)
        return out

    def _compute_wrist_rotate(self, pilot: Optional[PilotFrame]) -> float:
        """Return normalized wrist rotation command in [-1..1].

        Positive = rotate right (RT), negative = rotate left (LT).
        Trigger pressure proportionally scales the command, capped by
        WRIST_ROTATE_SPEED (e.g. 0.20 max when fully pressed).
        """
        if (not self._wrist_rotate_enabled) or pilot is None:
            return 0.0

        try:
            rt = float(getattr(pilot.axes, self._wrist_rotate_right_axis, 0.0) or 0.0)
        except Exception:
            rt = 0.0
        try:
            lt = float(getattr(pilot.axes, self._wrist_rotate_left_axis, 0.0) or 0.0)
        except Exception:
            lt = 0.0

        def _trigger_mag(v: float, deadzone: float) -> float:
            # Pilot trigger axes are normalized to [0..1], but clamp defensively.
            x = max(0.0, min(1.0, float(v)))
            if x <= deadzone:
                return 0.0
            # Re-map post-deadzone travel back to [0..1] for smooth throttle.
            span = max(1e-6, 1.0 - deadzone)
            return max(0.0, min(1.0, (x - deadzone) / span))

        dz = float(self._wrist_rotate_trigger_deadzone)
        rt_mag = _trigger_mag(rt, dz)
        lt_mag = _trigger_mag(lt, dz)

        # Net command lets pilots feather both triggers; equal pressure cancels.
        cmd = (rt_mag - lt_mag) * float(self._wrist_rotate_speed)
        return max(-1.0, min(1.0, cmd))


    def _compute_gripper_diff(self, pilot: Optional[PilotFrame]) -> Tuple[float, float]:
        if not self._gripper_enabled:
            return 0.0, 0.0
        if pilot is None:
            return self._gripper_last_left, self._gripper_last_right

        aux = getattr(pilot, "aux", {}) or {}
        try:
            pitch = float(aux.get(self._gripper_pitch_key, 0.0) or 0.0)
        except Exception:
            pitch = 0.0
        try:
            yaw = float(aux.get(self._gripper_yaw_key, 0.0) or 0.0)
        except Exception:
            yaw = 0.0

        pitch = max(-1.0, min(1.0, pitch * float(self._gripper_pitch_scale) * float(self._gripper_pitch_invert)))
        yaw = max(-1.0, min(1.0, yaw * float(self._gripper_yaw_scale) * float(self._gripper_yaw_invert)))

        has_input = (abs(pitch) > float(self._gripper_deadzone)) or (abs(yaw) > float(self._gripper_deadzone))
        if (not has_input) and self._gripper_hold_last:
            return self._gripper_last_left, self._gripper_last_right

        left, right = self._mix_gripper_axes(pitch, yaw)

        # Preserve the commanded differential direction when combined wrist
        # sweep + rotation would otherwise overrun one servo. Independent
        # clipping makes opposition rotation feel like it only works near the
        # center of sweep, because one side hits the limit early and the pair
        # stops moving symmetrically. Scale both sides together instead so the
        # wrist keeps the requested motion as much as the mechanism allows.
        peak = max(1.0, abs(left), abs(right))
        left /= peak
        right /= peak

        self._gripper_last_left = left
        self._gripper_last_right = right
        return left, right

    @staticmethod
    def _mix_gripper_axes(pitch: float, yaw: float) -> Tuple[float, float]:
        left = float(pitch) + float(yaw)
        right = float(pitch) - float(yaw)

        peak = max(1.0, abs(left), abs(right))
        return (left / peak, right / peak)

    def _set_gripper_park_pose(self) -> None:
        self._gripper_last_left = float(self._gripper_park_left)
        self._gripper_last_right = float(self._gripper_park_right)

    def _compute_lights(self, pilot: Optional[PilotFrame]) -> Optional[float]:
        """Return a normalized lights value in [0..1], or None if lights disabled."""
        if not self._lights_enabled:
            return None

        # Safety: if configured to fail-safe lights off when pilot frames are stale/missing.
        if pilot is None and self._lights_failsafe_off:
            return 0.0

        # Toggle mode: fixed brightness controlled by a button edge.
        if self._lights_mode in ("toggle", "button"):
            v = float(self._lights_default) if bool(self._lights_on) else 0.0
            return clamp01(v)

        # Axis mode (legacy): brightness from a trigger/axis.
        if pilot is None:
            return None
        axis_name = self._lights_axis
        raw = float(getattr(pilot.axes, axis_name, 0.0))
        v = axis_to_01(axis_name, raw)

        if v < self._lights_deadzone:
            v = 0.0

        if self._lights_invert:
            v = 1.0 - v

        v = clamp01(v * self._lights_scale)
        return v

    def _send_to_hw(self, thr: Dict[Any, float]):
        # dry_run: no hardware IO
        if self.dry_run:
            if (not self._warned_dry_run) and self.state.is_armed() and _has_nonzero(thr):
                print('[rov/control] WARNING: dry_run=True, so PWM outputs are suppressed. If motors spin in tools/native_motor_test but not here, PWM init likely failed or you are not running with sufficient permissions.')
                self._warned_dry_run = True
            return

        sink = self._hw_sink
        if sink is None:
            if (not self._warned_no_sink) and self.state.is_armed() and _has_nonzero(thr):
                print('[rov/control] WARNING: no hardware sink attached, so PWM outputs are suppressed. Check that motion/pwm.py imported and ThrustWriter initialized successfully in main_rov.py.')
                self._warned_no_sink = True
            return

        # If the sink supports arm/disarm but isn't armed while we are, warn.
        if hasattr(sink, 'arm') and hasattr(sink, 'disarm'):
            if self.state.is_armed() and (not self._sink_armed) and (not self._warned_sink_disarmed):
                print('[rov/control] WARNING: control is ARMED but PWM sink is not armed (outputs likely neutral). This usually means an arming event was missed or dry_run toggling prevented sink.arm().')
                self._warned_sink_disarmed = True

        # Prefer a .write(...) method, otherwise treat as callable.
        if hasattr(sink, 'write'):
            sink.write(thr)  # type: ignore[attr-defined]
        else:
            sink(thr)

def _cli_main() -> None:
    ap = argparse.ArgumentParser(description="ROV ControlService debug (receive pilot + mix thrusters)")
    ap.add_argument("--bind", default="tcp://*:6000", help="ZMQ SUB bind endpoint")
    ap.add_argument("--rate", type=float, default=50.0, help="control loop rate Hz")
    ap.add_argument("--ttl", type=float, default=0.5, help="pilot freshness TTL seconds")
    ap.add_argument("--debug", action="store_true", help="extra logs")
    ap.add_argument("--dry-run", action="store_true", help="do not talk to hardware (prints)")
    ap.add_argument("--log-every", type=float, default=0.25, help="print interval seconds")
    args = ap.parse_args()

    rx = PilotReceiver(bind_endpoint=args.bind, debug=args.debug)
    rx.start()

    gains = ControlGains()
    state = ROVControlState()

    svc = ControlService(
        pilot_rx=rx,
        gains=gains,
        control_state=state,
        rate_hz=args.rate,
        ttl=args.ttl,
        debug=args.debug,
        dry_run=bool(args.dry_run),  # default: False unless --dry-run
        log_every_s=args.log_every,
        arm_toggle_button="menu",
        kill_button="win",
    )

    print(f"[rov/control] listening on {args.bind}")
    print("[rov/control] Press MENU to toggle ARMED/DISARMED. Press WIN to KILL (disarm).")
    print("[rov/control] (Ctrl+C to stop)")
    svc.start()

    try:
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        svc.stop()
        rx.stop()


if __name__ == "__main__":
    _cli_main()
