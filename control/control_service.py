"""ROV control-loop orchestration.

`ControlService` is the bridge between the topside pilot stream and physical
PWM outputs. It receives fresh `PilotFrame` values, applies controller-axis
configuration, composes optional depth/attitude-hold corrections, mixes the
result into named thruster outputs, and finally sends safe normalized commands
to the hardware adapter.

The module keeps control math intentionally separate from hardware writes. That
separation lets unit tests validate command building, hold behavior, arming, and
auxiliary outputs without needing a Navigator board attached.
"""

from __future__ import annotations

import argparse
import copy
import math
import time
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, Any, Mapping, Hashable

from schema.pilot_common import PilotFrame
from control.pilot_receiver import PilotReceiver
import rov_config as cfg
from motion.channel_map import ChannelMap

from control.mixer import EightThrusterMixer, SimpleGroupMixer, geometric_mixer_from_config, global_limit
from control.autopilot import AutopilotController, autopilot_config_from_module
from control.sensor_tap import AutopilotSensorTap


@dataclass
class ControlGains:
    """Scalar gains applied before mixing normalized pilot DOF commands."""

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
        """Set the shared armed flag."""

        with self._lock:
            self._armed = bool(val)

    def is_armed(self) -> bool:
        """Return the current shared armed flag."""

        with self._lock:
            return self._armed

    def toggle_armed(self) -> bool:
        """Flip and return the shared armed flag."""

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
        pitch = float(dpy) * pitch_inv * gains.pitch
    else:
        pitch = dz(a(str(pitch_axis)), dzv) * pitch_inv * gains.pitch

    if roll_axis is None or str(roll_axis).strip().lower() in ("dpad", "dpad_x", "hat", "hat_x"):
        roll = float(dpx) * roll_inv * gains.roll
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
    """Clamp a numeric value to the inclusive [0.0, 1.0] range."""

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


def _has_nonzero(values: Mapping[Any, Any], eps: float = 1e-6) -> bool:
    """Return True when any mapping value is meaningfully non-zero."""
    for v in values.values():
        try:
            if abs(float(v)) > float(eps):
                return True
        except Exception:
            continue
    return False


def _float_map(values: Mapping[Any, Any] | None) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not isinstance(values, Mapping):
        return out
    for key, value in values.items():
        try:
            v = float(value)
        except Exception:
            continue
        if math.isfinite(v):
            out[str(key)] = v
    return out


def _pilot_summary(pilot: Optional[PilotFrame], age_s: Optional[float], fresh: bool) -> Dict[str, Any]:
    if pilot is None:
        return {
            "available": False,
            "fresh": False,
            "age_s": None if age_s is None else float(age_s),
        }
    out = pilot.to_dict()
    out.pop("type", None)
    out["available"] = True
    out["fresh"] = bool(fresh)
    out["age_s"] = None if age_s is None else float(age_s)
    return out



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
        self._last_back_gripper_gain = 1.0
        self._last_arm_gain = 1.0
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
        elif self._mix_mode == 'geometric':
            self.mixer = geometric_mixer_from_config(cfg)
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

        # --- optional feed-forward current budget (fuse protection) --------
        # Default OFF. When disabled this is a true no-op: the model is not even
        # loaded and _apply_current_budget() returns its input unchanged.
        self._current_budget_enabled = bool(getattr(cfg, "CURRENT_BUDGET_ENABLE", False))
        self._current_budget_max_a = float(getattr(cfg, "CURRENT_BUDGET_MAX_A", 22.0))
        self._current_budget_reserve_a = float(getattr(cfg, "CURRENT_BUDGET_RESERVE_A", 0.0))
        self._current_budget_voltage_v = float(getattr(cfg, "CURRENT_BUDGET_VOLTAGE_V", 14.0))
        self._current_budget_min_scale = float(getattr(cfg, "CURRENT_BUDGET_MIN_SCALE", 0.0))
        self._current_model = None
        self._current_budget_warned = False
        if self._current_budget_enabled:
            try:
                from control.current_model import T200CurrentModel
                path = str(getattr(cfg, "CURRENT_BUDGET_MODEL_PATH", "") or "").strip()
                self._current_model = (
                    T200CurrentModel.from_json(path) if path else T200CurrentModel.bundled()
                )
                budget = max(0.0, self._current_budget_max_a - self._current_budget_reserve_a)
                print(
                    f"[rov/control] current budget ENABLED: budget={budget:.1f}A "
                    f"(max={self._current_budget_max_a:.1f}A reserve={self._current_budget_reserve_a:.1f}A) "
                    f"assumed_V={self._current_budget_voltage_v:.1f} min_scale={self._current_budget_min_scale:.2f}"
                )
            except Exception as e:
                # Fail open: never let model loading break startup.
                self._current_model = None
                self._current_budget_enabled = False
                print(f"[rov/control] current budget disabled (model load failed): {e}")

        # Optional lights (aux PWM) control. The hardware mapping is handled by the PWM sink.
        self._lights_enabled = bool(getattr(cfg, "LIGHTS_ENABLE", hasattr(cfg, "LIGHTS_PWM_CHANNEL")))
        # Modes:
        #   - "toggle" (default for this project): a named button edge toggles a fixed brightness.
        #   - "axis": legacy trigger/axis brightness control.
        self._lights_mode = str(getattr(cfg, "LIGHTS_MODE", "toggle")).strip().lower()

        # Toggle-mode config
        self._lights_toggle_button = str(getattr(cfg, "LIGHTS_TOGGLE_BUTTON", "lights")).strip()
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
        self._gripper_pitch_invert = float(getattr(cfg, "GRIPPER_PITCH_INVERT", 1.0))
        self._gripper_yaw_invert = float(getattr(cfg, "GRIPPER_YAW_INVERT", 1.0))
        self._gripper_pitch_min, self._gripper_pitch_max = self._ordered_norm_pair(
            getattr(cfg, "GRIPPER_PITCH_MIN", -1.0),
            getattr(cfg, "GRIPPER_PITCH_MAX", 1.0),
        )
        self._gripper_yaw_min, self._gripper_yaw_max = self._ordered_norm_pair(
            getattr(cfg, "GRIPPER_YAW_MIN", -1.0),
            getattr(cfg, "GRIPPER_YAW_MAX", 1.0),
        )
        # Per-servo inversion for the facing-servo bevel differential (invert one
        # servo to un-swap pitch/roll). See rov_config.GRIPPER_*_INVERT.
        self._gripper_left_invert = float(getattr(cfg, "GRIPPER_LEFT_INVERT", 1.0))
        self._gripper_right_invert = float(getattr(cfg, "GRIPPER_RIGHT_INVERT", 1.0))
        self._gripper_deadzone = float(getattr(cfg, "GRIPPER_DEADBAND", 0.01))
        # Differential geometry in degrees (see _diff_mix_norm / docs/MANIPULATOR_ARM.md).
        self._gripper_servo_range_deg = max(1.0, float(getattr(cfg, "GRIPPER_SERVO_RANGE_DEG", 100.0)))
        self._gripper_pitch_span_deg = float(getattr(cfg, "GRIPPER_PITCH_SPAN_DEG", 90.0))
        self._gripper_wrist_span_deg = float(getattr(cfg, "GRIPPER_WRIST_SPAN_DEG", 90.0))
        self._gripper_pitch_neutral_deg = float(getattr(cfg, "GRIPPER_PITCH_NEUTRAL_DEG", 45.0))
        self._gripper_wrist_neutral_deg = float(getattr(cfg, "GRIPPER_WRIST_NEUTRAL_DEG", 45.0))
        arm_pitch = float(getattr(cfg, "GRIPPER_ARM_PITCH", getattr(cfg, "GRIPPER_DISARM_PITCH", 0.0)) or 0.0)
        arm_yaw = float(getattr(cfg, "GRIPPER_ARM_YAW", getattr(cfg, "GRIPPER_DISARM_YAW", 0.0)) or 0.0)
        self._gripper_park_pitch = max(self._gripper_pitch_min, min(self._gripper_pitch_max, arm_pitch))
        self._gripper_park_yaw = max(self._gripper_yaw_min, min(self._gripper_yaw_max, arm_yaw))
        self._gripper_park_left, self._gripper_park_right = self._diff_mix_norm(
            self._gripper_park_pitch,
            self._gripper_park_yaw,
        )
        # For servos, "no input" should normally mean "hold last commanded position"
        # rather than springing back to center. This latches the last mixed differential
        # output until a new pitch/yaw command arrives.
        self._gripper_hold_last = bool(getattr(cfg, "GRIPPER_HOLD_LAST_POSITION", True))
        self._gripper_park_on_arm_disarm = bool(getattr(cfg, "GRIPPER_PARK_ON_ARM_DISARM", True))
        self._gripper_park_settle_s = float(getattr(cfg, "GRIPPER_PARK_SETTLE_S", 0.50))
        self._gripper_park_slew_norm_per_s = max(
            0.0,
            float(getattr(cfg, "GRIPPER_PARK_SLEW_NORM_PER_S", getattr(cfg, "GRIPPER_SLEW_NORM_PER_S", 0.0))),
        )
        self._gripper_last_pitch = self._gripper_park_pitch
        self._gripper_last_yaw = self._gripper_park_yaw
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

        # --- autopilot -----------------------------------------------------
        self._autopilot: Optional[AutopilotController] = None
        self._autopilot_tap: Optional[AutopilotSensorTap] = None
        # Back-compat aliases used by management/status code and older tests.
        self._depth_hold = None
        self._depth_tap: Optional[AutopilotSensorTap] = None
        self._hold_status_lock = threading.Lock()
        self._last_autopilot_status: Dict[str, Any] = {}
        self._last_autopilot_status_ts: Optional[float] = None
        self._last_depth_status: Dict[str, Any] = {}
        self._last_depth_status_ts: Optional[float] = None
        self._last_control_status: Dict[str, Any] = {}
        self._last_control_status_ts: Optional[float] = None

        if bool(getattr(cfg, "AUTOPILOT_ENABLE", True)):
            try:
                # Subscribe locally to the sensor PUB stream.
                self._autopilot_tap = AutopilotSensorTap(getattr(cfg, "SENSOR_PUB_ENDPOINT", "tcp://0.0.0.0:6001"))
                self._depth_tap = self._autopilot_tap
            except Exception as e:
                if self.debug:
                    print("[rov/control] autopilot sensor tap disabled:", e)
                self._autopilot_tap = None
                self._depth_tap = None

            try:
                self._autopilot = AutopilotController(autopilot_config_from_module(cfg))
                self._depth_hold = self._autopilot.depth_hold
            except Exception as e:
                if self.debug:
                    print("[rov/control] autopilot disabled (config/init failed):", e)
                self._autopilot = None
                self._depth_hold = None

    def _set_depth_status(self, status: Dict[str, Any]) -> None:
        with self._hold_status_lock:
            self._last_depth_status = dict(status or {})
            self._last_depth_status_ts = time.time()

    def _set_autopilot_status(self, status: Dict[str, Any]) -> None:
        now = time.time()
        with self._hold_status_lock:
            self._last_autopilot_status = copy.deepcopy(dict(status or {}))
            self._last_autopilot_status_ts = now
            depth_status = dict((status or {}).get("depth_hold") or {})
            self._last_depth_status = depth_status
            self._last_depth_status_ts = now

    def _set_control_status(self, status: Dict[str, Any]) -> None:
        now = time.time()
        payload = copy.deepcopy(dict(status or {}))
        payload.setdefault("updated_ts", now)
        with self._hold_status_lock:
            self._last_control_status = payload
            self._last_control_status_ts = now

    def get_hold_status_snapshot(self) -> Dict[str, Any]:
        """Return a JSON-friendly snapshot of live hold state for topside/debugging."""
        now = time.time()
        with self._hold_status_lock:
            depth_status = copy.deepcopy(self._last_depth_status)
            depth_status_ts = self._last_depth_status_ts
            autopilot_status = copy.deepcopy(self._last_autopilot_status)
            autopilot_status_ts = self._last_autopilot_status_ts
            control_status = copy.deepcopy(self._last_control_status)
            control_status_ts = self._last_control_status_ts

        depth_target_m: Optional[float] = None
        if self._depth_hold is not None and self._depth_hold.target_depth_m is not None:
            depth_target_m = float(self._depth_hold.target_depth_m)

        depth_sensor: Dict[str, Any] = {
            "depth_m": None,
            "sample_age_s": None,
            "stream_age_s": None,
            "sensor_name": None,
            "raw": {},
        }
        if self._depth_tap is not None:
            depth_sensor = {
                "depth_m": (None if self._depth_tap.last_depth_m is None else float(self._depth_tap.last_depth_m)),
                "sample_age_s": self._depth_tap.age_s(now),
                "stream_age_s": self._depth_tap.rx_age_s(now),
                "sensor_name": self._depth_tap.last_sensor_name,
                "raw": copy.deepcopy(self._depth_tap.last_raw),
            }

        attitude_sensor: Dict[str, Any] = {
            "available": False,
            "sample_age_s": None,
            "source": None,
            "raw": {},
        }
        if self._autopilot_tap is not None:
            attitude_sensor = {
                "available": bool(self._autopilot_tap.last_attitude),
                "sample_age_s": self._autopilot_tap.attitude_age_s(now),
                "source": self._autopilot_tap.last_attitude_source,
                "raw": copy.deepcopy(self._autopilot_tap.last_attitude),
            }

        return {
            "armed": bool(self.state.is_armed()),
            "updated_ts": now,
            "control": {
                "status": control_status,
                "status_age_s": (None if control_status_ts is None else float(now - control_status_ts)),
            },
            "autopilot": {
                "available": self._autopilot is not None,
                "sensor_available": self._autopilot_tap is not None,
                "status": autopilot_status,
                "status_age_s": (None if autopilot_status_ts is None else float(now - autopilot_status_ts)),
                "attitude_sensor": attitude_sensor,
            },
            "depth_hold": {
                "available": self._depth_hold is not None,
                "sensor_available": self._depth_tap is not None,
                "target_m": depth_target_m,
                "status": depth_status,
                "status_age_s": (None if depth_status_ts is None else float(now - depth_status_ts)),
                "sensor": depth_sensor,
            },
        }

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
        """Start the control-loop background thread."""

        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        sink = self._hw_sink
        sink_name = type(sink).__name__ if sink is not None else 'None'
        print(f"[rov/control] started mode={self._mix_mode} rate={1.0/self.period:.1f}Hz ttl={self.ttl:.2f}s dry_run={self.dry_run} sink={sink_name}")

    def stop(self):
        """Stop the control loop and command safe hardware shutdown."""

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
            if self.state.is_armed():
                self._send_gripper_park_pose(settle_s=self._gripper_park_settle_s)
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
                    self._disarm_with_gripper_park()
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
                    self._arm_with_gripper_park()
                    return f"TOGGLE ({b}) -> ARMED"
                else:
                    self._disarm_with_gripper_park()
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
                        self._disarm_with_gripper_park()
                        arming_event = f"FAILSAFE (stale>{self.failsafe_disarm_s:.1f}s) -> DISARMED"
                else:
                    self._stale_since = None

            # Deadman: if enabled, you must hold the button to remain armed.
            if self.state.is_armed() and self.deadman_button and fresh_pilot is not None:
                if not bool(getattr(fresh_pilot.buttons, self.deadman_button, False)):
                    self._disarm_with_gripper_park()
                    arming_event = f"DEADMAN ({self.deadman_button}) released -> DISARMED"

            if self._autopilot_tap is not None:
                try:
                    self._autopilot_tap.poll()
                except Exception:
                    pass

            if fresh_pilot is None or not self.state.is_armed():
                payload: Dict[Any, float] = {}
                lights_val: Optional[float] = self._compute_lights(fresh_pilot)
                if lights_val is not None and (self.state.is_armed() or self._lights_allow_when_disarmed):
                    payload["lights"] = float(lights_val)
                self._send_to_hw(payload)

                if fresh_pilot is None:
                    control_reason = "no_pilot" if pilot is None else "stale_pilot"
                else:
                    control_reason = "disarmed"
                self._set_control_status(
                    {
                        "updated_ts": now,
                        "armed": bool(self.state.is_armed()),
                        "sink_armed": bool(self._sink_armed),
                        "dry_run": bool(self.dry_run),
                        "mix_mode": str(self._mix_mode),
                        "reason": control_reason,
                        "pilot": _pilot_summary(pilot, fresh_age, fresh_pilot is not None),
                        "arming_event": arming_event,
                        "lights_event": lights_event,
                        "cmd_manual": {},
                        "cmd_final": {},
                        "thrusters_raw": {},
                        "thrusters_limited": {},
                        "thrusters_final": {},
                        "payload": _float_map(payload),
                        "gain": {
                            "base_power_scale": float(self._base_power_scale),
                            "pilot_max_gain": float(self._last_pilot_max_gain),
                            "effective_power_scale": float(self.gains.power_scale),
                            "back_gripper_gain": float(self._last_back_gripper_gain),
                            "arm_gain": float(self._last_arm_gain),
                        },
                    }
                )

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
                thruster_max_abs = self._live_thruster_max_abs(fresh_pilot)

                if self._mix_mode == 'simple_groups':
                    cmd2 = build_2axis(fresh_pilot, self.gains)
                    cmd_manual: Dict[str, float] = dict(cmd2)
                    cmd_final: Dict[str, float] = dict(cmd2)
                    raw_thr = self.mixer.mix(cmd2['surge'], cmd2['heave'])
                    mixer_diag_raw: Dict[str, Any] = {}
                    thr = global_limit(raw_thr, max_abs=thruster_max_abs)
                    thr_limited = dict(thr)
                    mixer_diag_limited: Dict[str, Any] = {}
                    thr = self._channels_to_named(thr)
                else:
                    cmd6 = build_6dof(fresh_pilot, self.gains)
                    cmd_manual = dict(cmd6)

                    # Autopilot: modify owned DOFs in-place before mixing.
                    if self._autopilot is not None:
                        modes = fresh_pilot.modes or {}
                        depth_m = self._depth_tap.last_depth_m if self._depth_tap is not None else None
                        depth_age = self._depth_tap.age_s(now) if self._depth_tap is not None else None
                        attitude = self._autopilot_tap.last_attitude if self._autopilot_tap is not None else {}
                        attitude_age = self._autopilot_tap.attitude_age_s(now) if self._autopilot_tap is not None else None
                        try:
                            cmd6, st = self._autopilot.step(
                                modes=modes,
                                cmd=cmd6,
                                depth_m=depth_m,
                                depth_age_s=depth_age,
                                attitude=attitude,
                                attitude_age_s=attitude_age,
                                dt=self.period,
                            )
                            self._set_autopilot_status(st)
                        except Exception as e:
                            # Fail open to manual heave if anything goes wrong.
                            self._set_autopilot_status(
                                {
                                    "enabled_cmd": False,
                                    "active": False,
                                    "depth_hold": {"enabled_cmd": False, "active": False, "reason": f"err:{e}"},
                                    "attitude": {"enabled_cmd": False, "active": False, "reason": f"err:{e}"},
                                }
                            )

                    cmd_final = dict(cmd6)
                    raw_thr = self.mixer.mix(cmd6)
                    mixer_diag_raw = self._mixer_diagnostics(cmd6, raw_thr)
                    thr = global_limit(raw_thr, max_abs=thruster_max_abs)
                    thr_limited = dict(thr)
                    mixer_diag_limited = self._mixer_diagnostics(cmd6, thr_limited)

                # Optional feed-forward current budget (fuse protection). Default
                # OFF; fail-open. Scales all thrusters together if the predicted
                # summed current would exceed the configured budget. The pilot can
                # toggle the active limiting live via modes["current_budget"]
                # (absent key -> active by default when the config master is on).
                budget_modes = fresh_pilot.modes or {}
                budget_active = bool(budget_modes.get("current_budget", True))
                budget_max_override = budget_modes.get("current_budget_max_a")
                thr, current_budget_diag = self._apply_current_budget(
                    thr, active=budget_active, max_a_override=budget_max_override
                )

                # Per-thruster deadband at the mix output (extra protection against creep)
                base_db = float(getattr(cfg, "MIX_OUTPUT_DEADBAND", 0.05))
                dh_db = float(getattr(cfg, "DEPTH_HOLD_MIX_DEADBAND", 0.02))
                ap_db = float(getattr(cfg, "AUTOPILOT_MIX_DEADBAND", dh_db))
                autopilot_vertical_cmd = False
                autopilot_horizontal_cmd = False
                try:
                    modes = fresh_pilot.modes or {}
                    ap_modes = modes.get("autopilot") if isinstance(modes.get("autopilot"), dict) else {}
                    ap_modes = dict(ap_modes or {})
                    depth_cmd = bool(ap_modes.get("depth", modes.get("depth_hold", modes.get("depth_hold_enabled", False))))
                    rp_level = bool(ap_modes.get("roll_pitch_level", modes.get("roll_pitch_level", False)))
                    roll_mode = str(ap_modes.get("roll", modes.get("attitude_roll", ""))).strip().lower()
                    pitch_mode = str(ap_modes.get("pitch", modes.get("attitude_pitch", ""))).strip().lower()
                    yaw_mode = str(ap_modes.get("yaw", modes.get("attitude_yaw", modes.get("yaw_hold", "")))).strip().lower()
                    roll_cmd = rp_level or roll_mode not in ("", "off", "free", "manual", "none", "false", "0")
                    pitch_cmd = rp_level or pitch_mode not in ("", "off", "free", "manual", "none", "false", "0")
                    yaw_cmd = bool(modes.get("yaw_hold", False)) or yaw_mode not in ("", "off", "free", "manual", "none", "false", "0")
                    autopilot_vertical_cmd = bool(depth_cmd or roll_cmd or pitch_cmd)
                    autopilot_horizontal_cmd = bool(yaw_cmd)
                except Exception:
                    autopilot_vertical_cmd = False
                    autopilot_horizontal_cmd = False

                for k, v in list(thr.items()):
                    if isinstance(k, str) and k.strip().lower() == "lights":
                        continue

                    db = base_db
                    # When depth hold is enabled, allow smaller vertical
                    # corrections so the controller can make fine trim.
                    if autopilot_vertical_cmd and isinstance(k, str) and k.strip().upper().startswith("V_"):
                        db = ap_db
                    elif autopilot_horizontal_cmd and isinstance(k, str) and k.strip().upper().startswith("H_"):
                        db = ap_db

                    if abs(float(v)) < float(db):
                        thr[k] = 0.0
                thr_after_deadband = dict(thr)

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
                thr_final = dict(thr)
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
                self._set_control_status(
                    {
                        "updated_ts": now,
                        "armed": bool(self.state.is_armed()),
                        "sink_armed": bool(self._sink_armed),
                        "dry_run": bool(self.dry_run),
                        "mix_mode": str(self._mix_mode),
                        "reason": "armed_apply",
                        "pilot": _pilot_summary(pilot, fresh_age, True),
                        "arming_event": arming_event,
                        "lights_event": lights_event,
                        "cmd_manual": _float_map(cmd_manual),
                        "cmd_final": _float_map(cmd_final),
                        "thrusters_raw": _float_map(raw_thr),
                        "thrusters_limited": _float_map(thr_limited),
                        "thrusters_after_deadband": _float_map(thr_after_deadband),
                        "thrusters_final": _float_map(thr_final),
                        "payload": _float_map(payload),
                        "ramp": float(ramp),
                        "deadband": {
                            "base": float(base_db),
                            "autopilot": float(ap_db),
                            "autopilot_vertical_cmd": bool(autopilot_vertical_cmd),
                            "autopilot_horizontal_cmd": bool(autopilot_horizontal_cmd),
                        },
                        "mixer": {
                            "mode": str(self._mix_mode),
                            "raw": mixer_diag_raw,
                            "limited": mixer_diag_limited,
                        },
                        "current_budget": current_budget_diag,
                        "gain": {
                            "base_power_scale": float(self._base_power_scale),
                            "pilot_max_gain": float(self._last_pilot_max_gain),
                            "effective_power_scale": float(self.gains.power_scale),
                            "configured_thruster_max_abs": float(self._configured_thruster_max_abs()),
                            "effective_thruster_max_abs": float(thruster_max_abs),
                        },
                    }
                )

                if self.debug and (now - self._last_log) > self.log_every_s:
                    if self._mix_mode == "simple_groups":
                        msg = f"[rov/control] APPLY seq={fresh_pilot.seq} age={fresh_age:.3f}s cmd2={cmd2} thr={thr} | gain={self._last_pilot_max_gain*100:.0f}% cap={thruster_max_abs:.2f} (k={self.gains.power_scale:.2f})"
                    else:
                        msg = f"[rov/control] APPLY seq={fresh_pilot.seq} age={fresh_age:.3f}s cmd6={cmd6} thr={thr} | gain={self._last_pilot_max_gain*100:.0f}% cap={thruster_max_abs:.2f} (k={self.gains.power_scale:.2f})"
                        try:
                            ap = self._last_autopilot_status or {}
                            st = dict(ap.get("depth_hold") or self._last_depth_status or {})
                            att = dict(ap.get("attitude") or {})
                            if bool(st.get("enabled_cmd", False)):
                                msg += f" | depth_hold={'ON' if st.get('active') else 'OFF'}"
                                if st.get("active") and ("target_m" in st):
                                    msg += f" z={float(st.get('depth_f_m', 0.0)):.2f}m->t={float(st.get('target_m', 0.0)):.2f}m"
                            if bool(att.get("enabled_cmd", False)):
                                axes = dict(att.get("axes") or {})
                                active_axes = [name for name, axis_st in axes.items() if bool((axis_st or {}).get("active"))]
                                msg += f" | attitude={'ON' if active_axes else 'WAIT'}"
                                if active_axes:
                                    msg += f" axes={','.join(active_axes)}"
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

    def _pilot_mode_gain(
        self,
        pilot: Optional[PilotFrame],
        keys: Tuple[str, ...],
        *,
        default: float = 1.0,
        min_value: float = 0.0,
    ) -> float:
        """Return a pilot-requested mode gain, clamped to [min_value..1.0]."""
        if pilot is None:
            return float(default)
        try:
            modes = pilot.modes or {}
        except Exception:
            return float(default)
        raw = None
        for k in keys:
            if k in modes:
                raw = modes.get(k)
                break
        if raw is None:
            return float(default)
        try:
            v = float(raw)
        except Exception:
            return float(default)
        if v < float(min_value):
            v = float(min_value)
        if v > 1.0:
            v = 1.0
        return v

    def _pilot_gain_multiplier(self, pilot: Optional[PilotFrame]) -> float:
        """Return pilot-requested max gain multiplier in [0.05..1.0] (default 1.0)."""
        # Preferred key from TritonPilot. Accept a few aliases for compatibility.
        return self._pilot_mode_gain(
            pilot,
            ("max_gain", "pilot_max_gain", "power_scale_max"),
            default=1.0,
            min_value=0.05,
        )

    def _apply_pilot_gain(self, pilot: Optional[PilotFrame]) -> None:
        """Update effective power scale using the pilot's runtime max gain cap."""
        mult = self._pilot_gain_multiplier(pilot)
        self._last_pilot_max_gain = float(mult)
        self.gains.power_scale = float(self._base_power_scale) * float(mult)

    @staticmethod
    def _configured_thruster_max_abs() -> float:
        """Return the configured final normalized thruster cap in [0..1]."""
        try:
            value = float(getattr(cfg, "THRUSTER_MAX_ABS", 1.0))
        except Exception:
            value = 1.0
        return max(0.0, min(1.0, value))

    def _live_thruster_max_abs(self, pilot: Optional[PilotFrame]) -> float:
        """Cap final mixed thruster outputs by both config and pilot max gain."""
        return min(self._configured_thruster_max_abs(), self._pilot_gain_multiplier(pilot))

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

    def _apply_current_budget(self, thr: Dict[Any, float], *, active: bool = True, max_a_override: Any = None) -> Tuple[Dict[Any, float], Dict[str, Any]]:
        """Scale thrusters so predicted total current stays under budget.

        Feed-forward (no current sensor) and fail-open: on any problem it returns
        ``thr`` unchanged. Only thruster keys (H_*/V_*) are considered/scaled;
        other keys (lights, gripper, ...) pass through untouched.

        ``active`` is the live pilot toggle (``modes["current_budget"]``). When
        the model is loaded the predicted draw is *always* computed (so topside
        can show it), but scaling is only applied when ``active`` is True. This
        keeps a clean separation between "measure" and "limit": the config master
        switch ``CURRENT_BUDGET_ENABLE`` loads the model; the pilot toggle decides
        whether it actually intervenes.
        """
        if not self._current_budget_enabled or self._current_model is None:
            return thr, {"enabled": False}
        try:
            from control.current_model import current_budget_scale

            thruster_keys = [
                k
                for k in thr.keys()
                if isinstance(k, str) and (k.upper().startswith("H_") or k.upper().startswith("V_"))
            ]
            if not thruster_keys:
                return thr, {"enabled": True, "active": bool(active), "applied": False, "reason": "no_thruster_keys"}

            norms = {k: float(thr[k]) for k in thruster_keys}
            # The pilot can override the configured cap live (modes["current_budget_max_a"]).
            max_a = self._current_budget_max_a
            if max_a_override is not None:
                try:
                    candidate = float(max_a_override)
                    if math.isfinite(candidate) and candidate > 0.0:
                        max_a = candidate
                except (TypeError, ValueError):
                    pass
            budget = max(0.0, max_a - self._current_budget_reserve_a)
            scale, pred_before, pred_after = current_budget_scale(
                norms,
                self._current_model,
                voltage=self._current_budget_voltage_v,
                budget_a=budget,
                min_scale=self._current_budget_min_scale,
            )
            applied = bool(active and scale < 1.0)
            if applied:
                out = dict(thr)
                for k in thruster_keys:
                    out[k] = float(thr[k]) * float(scale)
                thr = out
            return thr, {
                "enabled": True,
                "active": bool(active),
                "applied": applied,
                "scale": float(scale if active else 1.0),
                "budget_a": float(budget),
                "max_a": float(max_a),
                "voltage_v": float(self._current_budget_voltage_v),
                "predicted_before_a": float(pred_before),
                "predicted_after_a": float(pred_after if active else pred_before),
            }
        except Exception as e:
            # Never let the budget limiter break the control loop.
            if not self._current_budget_warned:
                self._current_budget_warned = True
                print(f"[rov/control] current budget error (failing open, passthrough): {e}")
            return thr, {"enabled": True, "active": bool(active), "applied": False, "error": str(e)}

    def _mixer_diagnostics(self, cmd: Mapping[str, float], thr: Mapping[str, float]) -> Dict[str, Any]:
        diag_fn = getattr(self.mixer, "diagnostics", None)
        if not callable(diag_fn):
            return {}
        try:
            return dict(diag_fn(cmd, thr))
        except Exception as exc:
            return {"error": str(exc)}

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
        gain = self._pilot_mode_gain(
            pilot,
            ("back_gripper_gain", "t200_wrist_gain", "wrist_rotate_gain"),
            default=1.0,
            min_value=0.0,
        )
        self._last_back_gripper_gain = float(gain)

        # Net command lets pilots feather both triggers; equal pressure cancels.
        cmd = (rt_mag - lt_mag) * float(self._wrist_rotate_speed) * float(gain)
        return max(-1.0, min(1.0, cmd))


    def _compute_gripper_diff(self, pilot: Optional[PilotFrame]) -> Tuple[float, float]:
        if not self._gripper_enabled:
            return 0.0, 0.0
        if pilot is None:
            return self._gripper_last_left, self._gripper_last_right

        aux = getattr(pilot, "aux", {}) or {}

        # arm_gain now scales motion *speed* on the pilot side; it no longer caps the
        # reachable range here. Keep reading it so telemetry/status stays accurate.
        self._last_arm_gain = self._pilot_mode_gain(
            pilot,
            ("arm_gain", "gripper_gain", "servo_wrist_gain"),
            default=1.0,
            min_value=0.0,
        )

        # Live tuning overrides streamed from the topside in modes["arm_tune"].
        # Any present key overrides the corresponding rov_config default for this
        # frame, so inverts / neutral / range can be dialed in without a restart.
        try:
            tune = dict((getattr(pilot, "modes", None) or {}).get("arm_tune") or {})
        except Exception:
            tune = {}

        def _tune(key: str, default: float) -> float:
            v = tune.get(key)
            try:
                return float(v) if v is not None else float(default)
            except Exception:
                return float(default)

        pitch_invert = _tune("pitch_invert", self._gripper_pitch_invert)
        yaw_invert = _tune("yaw_invert", self._gripper_yaw_invert)
        pitch_min, pitch_max = self._ordered_norm_pair(
            _tune("pitch_min", self._gripper_pitch_min),
            _tune("pitch_max", self._gripper_pitch_max),
        )
        yaw_min, yaw_max = self._ordered_norm_pair(
            _tune("yaw_min", self._gripper_yaw_min),
            _tune("yaw_max", self._gripper_yaw_max),
        )

        # gripper_pitch / gripper_yaw are absolute POSITION commands in [-1..1].
        # A *present* key (even 0.0) is applied directly so the pilot can hold any
        # pose, including centered. An *absent* axis holds its last commanded value
        # (safety net for a stale/older topside that omits the arm keys).
        pitch_present = self._gripper_pitch_key in aux
        yaw_present = self._gripper_yaw_key in aux
        if (not pitch_present) and (not yaw_present) and self._gripper_hold_last:
            return self._gripper_last_left, self._gripper_last_right

        last_pitch, last_yaw = self._last_gripper_axes()
        if pitch_present:
            try:
                pitch = max(-1.0, min(1.0, float(aux.get(self._gripper_pitch_key) or 0.0) * pitch_invert))
            except Exception:
                pitch = last_pitch
        else:
            pitch = last_pitch if self._gripper_hold_last else 0.0
        if yaw_present:
            try:
                yaw = max(-1.0, min(1.0, float(aux.get(self._gripper_yaw_key) or 0.0) * yaw_invert))
            except Exception:
                yaw = last_yaw
        else:
            yaw = last_yaw if self._gripper_hold_last else 0.0
        pitch = max(pitch_min, min(pitch_max, pitch))
        yaw = max(yaw_min, min(yaw_max, yaw))

        left, right = self._diff_mix_norm(pitch, yaw, overrides=tune)

        self._gripper_last_pitch = pitch
        self._gripper_last_yaw = yaw
        self._gripper_last_left = left
        self._gripper_last_right = right
        return left, right

    @staticmethod
    def _float_or_default(value, default: float) -> float:
        try:
            return float(value)
        except Exception:
            return float(default)

    @classmethod
    def _ordered_norm_pair(cls, minimum, maximum) -> Tuple[float, float]:
        lo = max(-1.0, min(1.0, cls._float_or_default(minimum, -1.0)))
        hi = max(-1.0, min(1.0, cls._float_or_default(maximum, 1.0)))
        if lo > hi:
            lo, hi = hi, lo
        return lo, hi

    @staticmethod
    def _diff_mix_norm_deg(
        pitch_norm: float,
        yaw_norm: float,
        *,
        servo_range_deg: float,
        pitch_span_deg: float,
        wrist_span_deg: float,
        pitch_neutral_deg: float,
        wrist_neutral_deg: float,
        left_invert: float = 1.0,
        right_invert: float = 1.0,
    ) -> Tuple[float, float]:
        """Map normalized (pitch, wrist) position commands to normalized servo outputs.

        Inputs are absolute positions in [-1..1]:
          ``pitch_norm`` -1..+1  ->  pitch 0..``pitch_span_deg`` (flat -> straight down)
          ``yaw_norm``   -1..+1  ->  wrist 0..``wrist_span_deg``

        A 1:1 differential gives servo angles ``s_L = dPitch + dWrist`` and
        ``s_R = dPitch - dWrist`` as deviations from the servo-center pose. A
        PITCH-PRIORITY clip keeps the requested pitch and tapers wrist so neither
        servo exceeds ``+/-servo_range_deg``. Returns ``(left_norm, right_norm)``
        where ``+/-1`` corresponds to ``+/-servo_range_deg``.
        """
        rng = max(1.0, float(servo_range_deg))
        p = max(-1.0, min(1.0, float(pitch_norm)))
        w = max(-1.0, min(1.0, float(yaw_norm)))
        pitch_deg = (p + 1.0) * 0.5 * float(pitch_span_deg)
        wrist_deg = (w + 1.0) * 0.5 * float(wrist_span_deg)
        d_pitch = pitch_deg - float(pitch_neutral_deg)
        d_wrist = wrist_deg - float(wrist_neutral_deg)
        # Pitch priority: clip pitch to range first, then give wrist the leftover budget.
        d_pitch = max(-rng, min(rng, d_pitch))
        room = max(0.0, rng - abs(d_pitch))
        d_wrist = max(-room, min(room, d_wrist))
        # Per-servo inversion handles the facing-servo bevel differential, where one
        # servo is physically mirrored. Inverting one un-swaps pitch and roll.
        left = float(left_invert) * (d_pitch + d_wrist) / rng
        right = float(right_invert) * (d_pitch - d_wrist) / rng
        return (max(-1.0, min(1.0, left)), max(-1.0, min(1.0, right)))

    def _diff_mix_norm(
        self, pitch_norm: float, yaw_norm: float, overrides: Optional[Dict[str, Any]] = None
    ) -> Tuple[float, float]:
        o = overrides or {}

        def g(key: str, attr: str) -> float:
            v = o.get(key)
            try:
                return float(v) if v is not None else float(getattr(self, attr))
            except Exception:
                return float(getattr(self, attr))

        return self._diff_mix_norm_deg(
            pitch_norm,
            yaw_norm,
            servo_range_deg=g("servo_range_deg", "_gripper_servo_range_deg"),
            pitch_span_deg=g("pitch_span_deg", "_gripper_pitch_span_deg"),
            wrist_span_deg=g("wrist_span_deg", "_gripper_wrist_span_deg"),
            pitch_neutral_deg=g("pitch_neutral_deg", "_gripper_pitch_neutral_deg"),
            wrist_neutral_deg=g("wrist_neutral_deg", "_gripper_wrist_neutral_deg"),
            left_invert=g("left_invert", "_gripper_left_invert"),
            right_invert=g("right_invert", "_gripper_right_invert"),
        )

    def _last_gripper_axes(self) -> Tuple[float, float]:
        try:
            return float(self._gripper_last_pitch), float(self._gripper_last_yaw)
        except Exception:
            left = float(getattr(self, "_gripper_last_left", 0.0))
            right = float(getattr(self, "_gripper_last_right", 0.0))
            return (left + right) * 0.5, (left - right) * 0.5

    def _set_gripper_park_pose(self) -> None:
        self._gripper_last_pitch = float(self._gripper_park_pitch)
        self._gripper_last_yaw = float(self._gripper_park_yaw)
        self._gripper_last_left = float(self._gripper_park_left)
        self._gripper_last_right = float(self._gripper_park_right)

    def _gripper_park_payload(self) -> Dict[str, float]:
        if not self._gripper_enabled:
            return {}
        self._set_gripper_park_pose()
        return {
            self._gripper_left_key: float(self._gripper_last_left),
            self._gripper_right_key: float(self._gripper_last_right),
        }

    def _gripper_payload_for_outputs(self, left: float, right: float) -> Dict[str, float]:
        if not self._gripper_enabled:
            return {}
        return {
            self._gripper_left_key: float(left),
            self._gripper_right_key: float(right),
        }

    def _send_gripper_park_pose(self, *, settle_s: float = 0.0) -> None:
        if not self._gripper_park_on_arm_disarm:
            return
        if not self._gripper_enabled:
            return

        start_left = float(getattr(self, "_gripper_last_left", self._gripper_park_left))
        start_right = float(getattr(self, "_gripper_last_right", self._gripper_park_right))
        target_left = float(self._gripper_park_left)
        target_right = float(self._gripper_park_right)
        rate = float(getattr(self, "_gripper_park_slew_norm_per_s", 0.0) or 0.0)
        settle = max(0.0, float(settle_s))

        can_sleep = (not self.dry_run) and self._hw_sink is not None
        max_dist = max(abs(target_left - start_left), abs(target_right - start_right))
        if rate > 0.0 and max_dist > 1e-6 and settle > 0.0 and can_sleep:
            duration = max(settle, max_dist / max(rate, 1e-6))
            step_s = max(0.02, min(0.05, float(getattr(self, "period", 0.02) or 0.02)))
            steps = max(1, int(math.ceil(duration / step_s)))
            sleep_s = duration / float(steps)
            for idx in range(1, steps + 1):
                frac = float(idx) / float(steps)
                left = start_left + (target_left - start_left) * frac
                right = start_right + (target_right - start_right) * frac
                payload = self._gripper_payload_for_outputs(left, right)
                if payload:
                    self._send_to_hw(payload)
                if idx < steps:
                    time.sleep(sleep_s)
            self._set_gripper_park_pose()
            return

        payload = self._gripper_park_payload()
        if payload:
            self._send_to_hw(payload)
        if settle > 0.0 and can_sleep and max_dist > 1e-6:
            time.sleep(settle)

    def _arm_with_gripper_park(self) -> None:
        if self._autopilot is not None:
            self._autopilot.reset()
        self.state.set_armed(True)
        hold_s = float(getattr(cfg, "ARM_HW_INIT_HOLD_S", 0.0) or 0.0)
        self._armed_since = time.time() + hold_s
        self._sync_sink_armed(force=True)
        self._send_gripper_park_pose(settle_s=self._gripper_park_settle_s)

    def _disarm_with_gripper_park(self) -> None:
        if self._autopilot is not None:
            self._autopilot.reset()
        # Send the park command while the PWM sink is still armed, then give the
        # servos a short window to move before PWM is disabled.
        self._send_gripper_park_pose(settle_s=self._gripper_park_settle_s)
        self.state.set_armed(False)
        self._armed_since = None
        self._set_gripper_park_pose()
        self._sync_sink_armed(force=True)

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
