"""motion/pwm.py

Navigator PWM output + thruster adapter.

This module intentionally uses the official BlueRobotics Navigator Python
bindings (``bluerobotics_navigator``) rather than talking to the PCA9685
directly.

Design goals:
  * PWM outputs can be kept disabled until ARM for safety (configurable).
  * On ARM/DISARM we can physically toggle Navigator PWM enable (OE) so ESCs
    re-acquire signal and produce obvious tones (configurable).
  * When DISARMED we drive neutral and can optionally disable PWM outputs entirely.
  * Batch updates are used when possible (lower I2C overhead).
  * Robust to channel numbering differences (0-based vs 1-based).

The control loop provides normalized thrust values in [-1.0, +1.0]. We map
that into microsecond pulses around neutral and convert to PCA9685 "OFF" count
values using the formula from the Navigator documentation:
    value = 4095 * pulse_duration / cycle_period
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple, Union

from utils.navigator_import import import_navigator_module, navigator_api_summary


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def us_to_count(pulse_us: float, freq_hz: float) -> int:
    """Convert pulse width in microseconds to PCA9685 OFF-count [0..4095]."""
    period_us = 1_000_000.0 / float(freq_hz)
    value = round(4095.0 * (float(pulse_us) / period_us))
    if value < 0:
        return 0
    if value > 4095:
        return 4095
    return int(value)


def thrust_to_us(
    thrust: float,
    neutral_us: int,
    span_us: int,
    min_us: int,
    max_us: int,
    deadband_norm: float,
) -> int:
    """Map normalized thrust [-1..+1] to PWM pulse in microseconds."""
    t = float(thrust)
    if abs(t) < float(deadband_norm):
        t = 0.0
    t = _clamp(t, -1.0, 1.0)
    pulse = float(neutral_us) + float(span_us) * t
    pulse = _clamp(pulse, float(min_us), float(max_us))
    return int(round(pulse))


# ---- Channel numbering helpers ----------------------------------------------

ChannelSpec = Any  # int or PwmChannel enum value
#HI

def _parse_base_setting(base: Any) -> Optional[int]:
    if base is None:
        return None
    if isinstance(base, int):
        return base if base in (0, 1) else None
    if isinstance(base, str):
        s = base.strip().lower()
        if s in ("0", "zero", "zero_based", "0-based", "zerobased"):
            return 0
        if s in ("1", "one", "one_based", "1-based", "onebased"):
            return 1
        if s in ("auto", "detect", "default"):
            return None
    return None


def _infer_user_base(ch_values: List[int], forced_base: Any = None) -> int:
    """
    Determine whether the *config* is using 0-based (0..15) or 1-based (1..16)
    PWM channel numbering.

    AUTO heuristic:
      - If any channel is 0 -> 0-based
      - Else if everything fits in 1..16 -> 1-based
      - Else -> 0-based
    """
    forced = _parse_base_setting(forced_base)
    if forced in (0, 1):
        return forced

    if any(v == 0 for v in ch_values):
        return 0
    mn, mx = min(ch_values), max(ch_values)
    if mn >= 1 and mx <= 16:
        return 1
    return 0


def _validate_user_channels(ch_values: List[int], user_base: int) -> None:
    if user_base == 0:
        if min(ch_values) < 0 or max(ch_values) > 15:
            raise ValueError(
                f"Navigator PWM channels out of range for 0-based numbering (expected 0..15, got {sorted(set(ch_values))})."
            )
    else:
        if min(ch_values) < 1 or max(ch_values) > 16:
            raise ValueError(
                f"Navigator PWM channels out of range for 1-based numbering (expected 1..16, got {sorted(set(ch_values))})."
            )


def _get_pwm_channel_enum_and_base(nav: Any) -> Tuple[Optional[Any], int]:
    """Return (PwmChannel enum, lib_base).

    This project has seen *multiple* versions of the Navigator Python bindings:
      - some expose a ``PwmChannel`` enum (Ch1..Ch16)
      - some accept only raw integer channel indices

    Unfortunately, the integer indexing convention is not consistent across
    those versions (some are 0-based, some are 1-based). If we guess wrong,
    everything shifts by one channel — exactly the symptom you observed
    (thrusters drive the next channel and can even hit the lights output).

    Detection strategy (safe on the vehicle):
      1) If ``PwmChannel.Ch1`` exists and its underlying int is 0 or 1, trust it.
      2) Otherwise, *probe* accepted integer range while PWM is disabled:
         - if channel 0 is accepted => 0-based
         - else if channel 1 is accepted => 1-based
         - else fall back to 0-based (matches most field installs)
    """
    PwmChannel = getattr(nav, "PwmChannel", None)

    # 1) Enum-based detection
    try:
        if PwmChannel is not None and hasattr(PwmChannel, "Ch1"):
            v = int(getattr(PwmChannel, "Ch1"))
            if v in (0, 1):
                return PwmChannel, v
    except Exception:
        # Fall through to integer probing.
        pass

    # 2) Integer probing (with PWM disabled so this is safe)
    try:
        # Best-effort disable; some installs raise if not initialized yet.
        if hasattr(nav, "set_pwm_enable"):
            try:
                nav.set_pwm_enable(False)
            except Exception:
                pass

        # If 0 is rejected, the binding is likely 1-based.
        try:
            nav.set_pwm_channel_value(0, 0)
            return PwmChannel, 0
        except Exception:
            pass

        try:
            nav.set_pwm_channel_value(1, 0)
            return PwmChannel, 1
        except Exception:
            pass
    except Exception:
        pass

    # Conservative fallback for field deployments (most common integer API)
    return PwmChannel, 0


def _to_lib_channel_index(ch_user: int, user_base: int, lib_base: int) -> int:
    # Convert between bases: channel_index = ch_user - user_base + lib_base
    return int(ch_user) - int(user_base) + int(lib_base)


def _lib_channel_obj(ch_lib: int, lib_base: int, PwmChannel: Optional[Any]) -> ChannelSpec:
    """
    Return whatever the Navigator binding accepts for a channel.

    If a PwmChannel enum exists, prefer using it (less ambiguity across versions).
    Otherwise fall back to raw integer channel indices.
    """
    if PwmChannel is not None:
        # Enum names are Ch1..Ch16 even if underlying values are 0-based.
        name_number = ch_lib if lib_base == 1 else (ch_lib + 1)
        attr = f"Ch{name_number}"
        if hasattr(PwmChannel, attr):
            return getattr(PwmChannel, attr)
    return int(ch_lib)


# ---- Navigator PWM wrapper ---------------------------------------------------


class NavigatorPWM:
    """Thin wrapper around the official ``bluerobotics_navigator`` bindings."""

    def __init__(self, freq_hz: float = 50.0, debug: bool = False):
        self.freq_hz = float(freq_hz)
        self.debug = bool(debug)
        self._enabled = False

        nav = import_navigator_module()

        self._nav = nav
        self._PwmChannel, self._lib_base = _get_pwm_channel_enum_and_base(nav)

        # Optional: explicit Pi version improves reliability on some setups.
        try:
            if hasattr(nav, "Raspberry") and hasattr(nav, "set_raspberry_pi_version"):
                nav.set_raspberry_pi_version(nav.Raspberry.Pi4)
        except Exception:
            pass

        # Navigator docs: init() is "not necessary" but safe and makes intent clear.
        if hasattr(nav, "init"):
            try:
                nav.init()
            except Exception:
                # don't hard fail; some installs expose init differently
                pass

        # Set frequency once; all channels share the same frequency.
        if not hasattr(nav, "set_pwm_freq_hz"):
            raise RuntimeError(
                "Navigator PWM: installed bluerobotics_navigator does not expose "
                f"set_pwm_freq_hz(); {navigator_api_summary(nav)}"
            )
        try:
            nav.set_pwm_freq_hz(self.freq_hz)
        except Exception as e:
            raise RuntimeError(
                f"Navigator PWM: failed to set frequency to {self.freq_hz} Hz: {e}; "
                f"{navigator_api_summary(nav)}"
            )

    @property
    def lib_base(self) -> int:
        return int(self._lib_base)

    @property
    def pwm_enum(self) -> Optional[Any]:
        return self._PwmChannel

    def enable(self, state: bool) -> None:
        state = bool(state)
        try:
            self._nav.set_pwm_enable(state)
        except Exception as e:
            raise RuntimeError(f"Navigator PWM: set_pwm_enable({state}) failed: {e}")
        self._enabled = state

    # Backwards-compat helpers (some of our tools used arm/disarm naming).
    def arm(self) -> None:
        self.enable(True)

    def disarm(self) -> None:
        self.enable(False)

    @property
    def enabled(self) -> bool:
        return bool(self._enabled)

    def set_counts(self, channels: List[ChannelSpec], counts: List[int]) -> None:
        """Batch set channels (preferred) with graceful fallback."""
        if not channels:
            return
        if len(channels) != len(counts):
            raise ValueError("channels and counts must have same length")

        for ch, v in zip(channels, counts):
            self._nav.set_pwm_channel_value(ch, int(v))

    def set_servo_us(self, channel: ChannelSpec, pulse_us: float) -> int:
        count = us_to_count(pulse_us, self.freq_hz)
        self._nav.set_pwm_channel_value(channel, int(count))
        return count


TrimSpec = Union[int, Mapping[str, int]]


@dataclass(frozen=True)
class ThrustConfig:
    freq_hz: float = 50.0
    neutral_us: int = 1500
    span_us: int = 400
    min_us: int = 1100
    max_us: int = 1900
    deadband_norm: float = 0.07
    # Additional microsecond deadband around neutral.
    deadband_us: int = 25
    trim_us: TrimSpec = 0
    esc_init_hold_s: float = 3.0
    # If True, ThrustWriter.arm()/disarm() will physically toggle PWM enable so ESCs
    # re-arm each time (audible tones on many ESCs).
    hardware_arm_disarm: bool = True
    # Seconds to keep PWM disabled before re-enabling on arm (helps some ESCs notice signal loss).
    pwm_rearm_off_s: float = 0.35
    # Hold neutral for a short period before disabling PWM on disarm.
    pwm_disarm_hold_s: float = 0.25
    # If True and hardware_arm_disarm is enabled, fully disable PWM outputs on disarm.
    disable_pwm_on_disarm: bool = True
    # If True, ThrustWriter.disarm() keeps PWM enabled and only drives neutral.
    keep_pwm_enabled_on_disarm: bool = True
    # PWM channel numbering in your config: "auto" (default), 0, or 1.
    channel_base: Union[str, int] = "auto"
    # Slew limiting on normalized thrust commands (units: normalized thrust per second).
    # Set <= 0 to disable. Example: 3.0 means a channel can change by ~0.06 per 50 Hz tick.
    slew_rate_norm_per_s: float = 0.0
    # Optional slower slew rate when the command reverses sign (reduces current spikes).
    # If None or <= 0, uses slew_rate_norm_per_s.
    slew_reverse_rate_norm_per_s: Optional[float] = None
    # Clamp dt used for slew limiting so a stalled loop does not instantly "catch up".
    slew_dt_max_s: float = 0.10


# ---- Auxiliary (non-thruster) PWM outputs -----------------------------------

ReverseKey = Union[str, int]
ReverseMap = Mapping[ReverseKey, bool]


@dataclass(frozen=True)
class AuxOutputConfig:
    """Configuration for an auxiliary PWM output such as lights or servos.

    The control loop usually supplies a normalized value in [0.0, 1.0].
    We map that into microseconds and then into PCA9685 counts.

    Defaults are compatible with many PWM-dimmable LED drivers (1100us off, 1900us full).
    """
    min_us: int = 1100
    max_us: int = 1900
    off_us: int = 1100
    deadband_norm: float = 0.02
    trim_us: int = 0

    # Input mode:
    #   - "norm01": map [0..1] into [min_us..max_us] and use off_us for zero.
    #   - "signed": map [-1..1] around center_us (for bidirectional servos).
    input_mode: str = "norm01"
    center_us: int = 1500

    # If True, aux outputs may still be updated when thrusters are disarmed.
    allow_when_disarmed: bool = True

    # If True, force aux to off_us when ThrustWriter.disarm() is called.
    force_off_on_disarm: bool = False

    # If True, center signed aux outputs when ThrustWriter.disarm() is called.
    # This is useful for bidirectional differential servos where a centered pulse
    # is safer/more stable than sending the off_us value or disabling updates.
    center_on_disarm: bool = False

    # Optional normalized target to command on disarm. For "signed" outputs this
    # should be in [-1..1], which lets a differential servo park fully to one side
    # instead of always returning to center.
    disarm_norm: Optional[float] = None

    # If True, keep PWM enabled on disarm so this output can continue holding its
    # parked position even while thrusters are forced to neutral.
    hold_pwm_on_disarm: bool = False

class ThrustWriter:
    """Map mixer outputs to Navigator PWM channels.

    Input format
    -----------
    The control loop typically provides normalized thrust values in [-1.0, +1.0].

    This writer accepts *either* of these command styles:

      1) Thruster-name keyed dict (recommended):
         {"H_FL": 0.2, "V_RR": -0.1, ...}

      2) PWM-channel keyed dict (useful for bring-up / simple group mixing):
         {6: 0.2, 1: 0.2, 2: -0.1, ...}

    You may also include auxiliary outputs (e.g. lights) by name:
         {"lights": 0.7}

    Channel values are interpreted in the same numbering scheme as your config
    (0-based or 1-based) and are converted to whatever the Navigator binding uses.
    """

    def __init__(
        self,
        thruster_channels: Optional[Mapping[str, int]] = None,
        *,
        cfg: Optional[ThrustConfig] = None,
        reversed_map: Optional[ReverseMap] = None,
        aux_channels: Optional[Mapping[str, int]] = None,
        aux_cfg: Optional[Mapping[str, AuxOutputConfig]] = None,
        debug: bool = False,
        auto_enable: bool = True,
    ):
        self.debug = bool(debug)
        self.cfg = cfg or ThrustConfig()

        # Reversal mapping may contain either thruster names or channel numbers.
        self.reversed_map: Dict[ReverseKey, bool] = dict(reversed_map or {})

        # Default mapping matches the eight-thruster mixer naming.
        self.thruster_channels: Dict[str, int] = dict(
            thruster_channels
            or {
                "H_FL": 1,
                "H_FR": 2,
                "H_RL": 3,
                "H_RR": 4,
                "V_FL": 5,
                "V_FR": 6,
                "V_RL": 7,
                "V_RR": 8,
            }
        )

        # Aux outputs (e.g. lights). Values are normalized [0..1].
        self.aux_channels: Dict[str, int] = dict(aux_channels or {})
        self.aux_cfg: Dict[str, AuxOutputConfig] = {}
        for name, ch in self.aux_channels.items():
            if aux_cfg and name in aux_cfg:
                self.aux_cfg[name] = aux_cfg[name]
            else:
                self.aux_cfg[name] = AuxOutputConfig()

        self._pwm = NavigatorPWM(freq_hz=self.cfg.freq_hz, debug=self.debug)
        self._lock = threading.Lock()
        self._armed = False
        # While time.time() < _arming_until we force neutral thrusters (ESC init/arm window).
        self._arming_until: float = 0.0
        # Slew limiter state (normalized thrust after reversal/deadband, per thruster).
        self._last_thr_cmd_norm: List[float] = []
        self._last_thr_write_t: Optional[float] = None

        # --- Channel mapping / normalization ----------------------------------
        # The binding and the config may disagree on base indexing. We detect both
        # and convert so the correct physical outputs are driven.
        user_ch_values = [int(v) for v in self.thruster_channels.values()]
        if self.aux_channels:
            user_ch_values.extend(int(v) for v in self.aux_channels.values())

        user_base = _infer_user_base(user_ch_values, forced_base=self.cfg.channel_base)
        _validate_user_channels(user_ch_values, user_base)


        pwm_enum = self._pwm.pwm_enum
        lib_base = int(self._pwm.lib_base)

        # If the binding exposes a PwmChannel enum (Ch1..Ch16), drive channels using
        # those enum members directly. This removes ambiguity about whether the integer
        # API is 0-based or 1-based and prevents off-by-one bugs (e.g. accidentally
        # driving the lights channel when you meant to drive a thruster).
        use_enum = pwm_enum is not None and hasattr(pwm_enum, "Ch1")

        def _physical_ch(ch_user: int) -> int:
            # Convert config "user channel" into *physical* channel number 1..16.
            return int(ch_user) if user_base == 1 else int(ch_user) + 1

        # Stable thruster order for vectorized updates.
        self._thruster_order: List[str] = sorted(self.thruster_channels.keys())
        self._thruster_user_channels: List[int] = [int(self.thruster_channels[n]) for n in self._thruster_order]
        self._channels: List[ChannelSpec] = []
        self._channels_phys: List[int] = []
        self._channels_lib: List[int] = []  # kept for backward-compat/debug

        for name, ch_user in zip(self._thruster_order, self._thruster_user_channels):
            phys = _physical_ch(ch_user)
            if not (1 <= phys <= 16):
                raise ValueError(f"thruster channel for {name} maps to invalid physical channel {phys} (expected 1..16)")

            if use_enum:
                ch_obj = getattr(pwm_enum, f"Ch{phys}")
                self._channels.append(ch_obj)
                self._channels_phys.append(phys)
                try:
                    self._channels_lib.append(int(ch_obj))
                except Exception:
                    self._channels_lib.append(-1)
            else:
                # Fallback: integer channel API. Preserve the old base-conversion logic.
                ch_lib = _to_lib_channel_index(ch_user, user_base, lib_base)

                if lib_base == 0 and not (0 <= ch_lib <= 15):
                    raise ValueError(
                        f"thruster channel for {name} maps to invalid lib channel {ch_lib} (0-based lib expects 0..15)"
                    )
                if lib_base == 1 and not (1 <= ch_lib <= 16):
                    raise ValueError(
                        f"thruster channel for {name} maps to invalid lib channel {ch_lib} (1-based lib expects 1..16)"
                    )

                self._channels_lib.append(ch_lib)
                self._channels.append(_lib_channel_obj(ch_lib, lib_base, pwm_enum))
                self._channels_phys.append(phys)

        # Aux channel objects
        self._aux_order: List[str] = sorted(self.aux_channels.keys())
        self._aux_user_channels: List[int] = [int(self.aux_channels[n]) for n in self._aux_order]
        self._aux_channels_objs: List[ChannelSpec] = []
        self._aux_channels_phys: List[int] = []
        self._aux_channels_lib: List[int] = []  # debug / backward-compat

        for name, ch_user in zip(self._aux_order, self._aux_user_channels):
            phys = _physical_ch(ch_user)
            if not (1 <= phys <= 16):
                raise ValueError(f"aux channel for {name} maps to invalid physical channel {phys} (expected 1..16)")

            if use_enum:
                ch_obj = getattr(pwm_enum, f"Ch{phys}")
                self._aux_channels_objs.append(ch_obj)
                self._aux_channels_phys.append(phys)
                try:
                    self._aux_channels_lib.append(int(ch_obj))
                except Exception:
                    self._aux_channels_lib.append(-1)
            else:
                ch_lib = _to_lib_channel_index(ch_user, user_base, lib_base)
                self._aux_channels_lib.append(ch_lib)
                self._aux_channels_objs.append(_lib_channel_obj(ch_lib, lib_base, pwm_enum))
                self._aux_channels_phys.append(phys)

        # Initialize slew state to neutral for each thruster channel.
        self._last_thr_cmd_norm = [0.0 for _ in self._channels]
        self._last_thr_write_t = None

        # Track last aux counts so disarm can keep lights etc. stable if desired.
        self._last_aux_counts: List[int] = self._aux_default_counts()

        if self.debug:
            mode = "enum" if use_enum else "int"
            print(f"[motion/pwm] PWM mapping mode={mode} config_user_base={user_base} binding_lib_base={lib_base}")
            for name, ch_u, phys, ch_l in zip(self._thruster_order, self._thruster_user_channels, self._channels_phys, self._channels_lib):
                print(f"[motion/pwm] thruster {name}: user_ch={ch_u} -> phys_ch={phys} (enum_int={ch_l})")
            for name, ch_u, phys, ch_l in zip(self._aux_order, self._aux_user_channels, self._aux_channels_phys, self._aux_channels_lib):
                print(f"[motion/pwm] aux {name}: user_ch={ch_u} -> phys_ch={phys} (enum_int={ch_l})")
        # Preload a safe neutral output. Whether PWM is actually enabled at boot is controlled
        # by the caller (main_rov uses PWM_AUTO_ENABLE). If PWM is kept disabled until ARM,
        # this prevents an unsafe first pulse when outputs are later enabled.
        try:
            self._apply_outputs(self._neutral_thruster_counts(), self._aux_default_counts())
        except Exception:
            pass

        # Optionally enable PWM output immediately and hold neutral so ESCs can initialize.
        if auto_enable:
            self._ensure_pwm_enabled()
            self._drive_neutral(hold_s=self.cfg.esc_init_hold_s)

    # --- lifecycle -------------------------------------------------
    def _ensure_pwm_enabled(self) -> None:
        if not self._pwm.enabled:
            self._pwm.enable(True)


    def arm(self) -> None:
        """Physically enable PWM outputs and allow non-neutral thruster commands.

        If hardware_arm_disarm is enabled, we toggle Navigator PWM enable (OE) so
        ESCs re-acquire the signal and typically emit their arming tones.
        """
        with self._lock:
            self._armed = True
            self._arming_until = 0.0

            if bool(getattr(self.cfg, "hardware_arm_disarm", False)):
                # Force a clean enable edge even if we were already enabled.
                try:
                    self._pwm.enable(False)
                except Exception:
                    pass
                off_s = float(getattr(self.cfg, "pwm_rearm_off_s", 0.0) or 0.0)
                if off_s > 0.0:
                    import time as _t
                    _t.sleep(off_s)

            # Enable outputs and drive neutral so ESCs can initialize/arm.
            self._ensure_pwm_enabled()

            hold_s = float(getattr(self.cfg, "esc_init_hold_s", 0.0) or 0.0)
            if hold_s > 0.0:
                import time as _t
                self._arming_until = _t.time() + hold_s

            # Hold neutral thrusters (keep current aux outputs stable).
            try:
                self._reset_slew_state()
                self._apply_outputs(self._neutral_thruster_counts(), list(self._last_aux_counts))
            except Exception:
                pass

    def disarm(self) -> None:
        """Drive thrusters to neutral and (optionally) physically disable PWM outputs."""
        with self._lock:
            self._armed = False
            self._arming_until = 0.0

            # Thrusters neutral; keep last aux outputs unless forced off.
            aux_counts = list(self._last_aux_counts)
            for i, name in enumerate(self._aux_order):
                aux_cfg = self.aux_cfg.get(name, AuxOutputConfig())
                if aux_cfg.force_off_on_disarm:
                    aux_counts[i] = self._aux_norm_to_count(name, 0.0)
                elif aux_cfg.center_on_disarm or (aux_cfg.disarm_norm is not None):
                    aux_counts[i] = self._aux_disarm_count(name)

            # Remember the parked aux state so a subsequent arm() starts from the
            # same pose instead of reviving a stale pre-disarm command.
            self._last_aux_counts = list(aux_counts)

            try:
                self._reset_slew_state()
                self._apply_outputs(self._neutral_thruster_counts(), aux_counts)
            except Exception:
                pass

            # Decide whether to physically disable outputs.
            if bool(getattr(self.cfg, "hardware_arm_disarm", False)):
                disable = bool(getattr(self.cfg, "disable_pwm_on_disarm", True))
            else:
                disable = (not bool(getattr(self.cfg, "keep_pwm_enabled_on_disarm", True)))

            if disable and any(bool(getattr(aux_cfg, "hold_pwm_on_disarm", False)) for aux_cfg in self.aux_cfg.values()):
                disable = False

            if disable:
                hold_s = float(getattr(self.cfg, "pwm_disarm_hold_s", 0.0) or 0.0)
                if hold_s > 0.0:
                    import time as _t
                    _t.sleep(hold_s)
                try:
                    self._pwm.enable(False)
                except Exception:
                    pass

    def shutdown(self) -> None:
        """Best-effort shutdown: neutral then disable PWM."""
        with self._lock:
            self._armed = False
            # Neutral thrusters and force aux off
            aux_counts = [self._aux_shutdown_count(name) for name in self._aux_order]
            self._reset_slew_state()
            self._apply_outputs(self._neutral_thruster_counts(), aux_counts)
            time.sleep(0.25)
            try:
                self._pwm.enable(False)
            except Exception:
                pass

    # --- helpers -------------------------------------------------
    def _trim_for(self, name: str) -> int:
        t = self.cfg.trim_us
        if isinstance(t, dict):
            return int(t.get(name, 0))
        return int(t)

    def _neutral_thruster_counts(self) -> List[int]:
        neutral = int(self.cfg.neutral_us)
        count = us_to_count(neutral, self.cfg.freq_hz)
        return [count for _ in self._channels]

    def _apply_outputs(self, thruster_counts: List[int], aux_counts: Optional[List[int]] = None) -> None:
        channels: List[ChannelSpec] = list(self._channels)
        counts: List[int] = list(thruster_counts)

        if self._aux_channels_objs:
            a = list(aux_counts) if aux_counts is not None else list(self._last_aux_counts)
            channels.extend(self._aux_channels_objs)
            counts.extend(a)

        self._pwm.set_counts(channels, counts)

    def _drive_neutral(self, hold_s: float = 0.0) -> None:
        """Neutral thrusters + aux defaults (typically lights off)."""
        thr_counts = self._neutral_thruster_counts()
        aux_counts = self._aux_default_counts()
        self._last_aux_counts = list(aux_counts)
        self._reset_slew_state()
        self._apply_outputs(thr_counts, aux_counts)
        if hold_s and hold_s > 0:
            time.sleep(float(hold_s))

    def _is_reversed(self, name: str, ch_user: int) -> bool:
        rm = self.reversed_map
        return bool(
            rm.get(name, False)
            or rm.get(ch_user, False)
            or rm.get(str(ch_user), False)
        )

    def _get_cmd_value(self, cmd: Mapping[Any, float], name: str, ch_user: int) -> float:
        # Prefer name-based command, then channel-based.
        if name in cmd:
            return float(cmd[name])
        if ch_user in cmd:
            return float(cmd[ch_user])
        s = str(ch_user)
        if s in cmd:
            return float(cmd[s])
        return 0.0

    def _reset_slew_state(self, now: Optional[float] = None) -> None:
        """Reset slew limiter to neutral (used on disarm / forced-neutral windows)."""
        self._last_thr_cmd_norm = [0.0 for _ in self._channels]
        self._last_thr_write_t = time.time() if now is None else float(now)

    def _apply_slew_limit(self, targets: List[float], now: float) -> List[float]:
        """Rate-limit normalized per-thruster commands to reduce current spikes."""
        base_rate = float(getattr(self.cfg, "slew_rate_norm_per_s", 0.0) or 0.0)
        if base_rate <= 0.0:
            # Keep state aligned even when disabled (useful if enabling later via config reload).
            out = [_clamp(float(t), -1.0, 1.0) for t in targets]
            self._last_thr_cmd_norm = list(out)
            self._last_thr_write_t = float(now)
            return out

        reverse_rate_cfg = getattr(self.cfg, "slew_reverse_rate_norm_per_s", None)
        reverse_rate = float(reverse_rate_cfg) if reverse_rate_cfg not in (None, False) else base_rate
        if reverse_rate <= 0.0:
            reverse_rate = base_rate

        dt = None if self._last_thr_write_t is None else (float(now) - float(self._last_thr_write_t))
        if dt is None:
            dt = 0.0
        dt = max(0.0, float(dt))
        dt_cap = float(getattr(self.cfg, "slew_dt_max_s", 0.10) or 0.10)
        if dt_cap > 0.0:
            dt = min(dt, dt_cap)

        # Ensure state length always matches channel count.
        if len(self._last_thr_cmd_norm) != len(targets):
            self._last_thr_cmd_norm = [0.0 for _ in targets]

        out: List[float] = []
        for prev, tgt in zip(self._last_thr_cmd_norm, targets):
            p = _clamp(float(prev), -1.0, 1.0)
            t = _clamp(float(tgt), -1.0, 1.0)

            rate = base_rate
            # If target asks to reverse direction, optionally use a slower rate.
            if (p > 0.0 and t < 0.0) or (p < 0.0 and t > 0.0):
                rate = min(base_rate, reverse_rate)

            max_delta = float(rate) * float(dt)
            delta = t - p
            if delta > max_delta:
                v = p + max_delta
            elif delta < -max_delta:
                v = p - max_delta
            else:
                v = t

            out.append(_clamp(v, -1.0, 1.0))

        self._last_thr_cmd_norm = list(out)
        self._last_thr_write_t = float(now)
        return out

    # --- aux mapping -------------------------------------------------
    def _aux_default_counts(self) -> List[int]:
        return [self._aux_norm_to_count(name, 0.0) for name in self._aux_order]

    def _aux_disarm_count(self, name: str) -> int:
        cfg = self.aux_cfg.get(name, AuxOutputConfig())
        if cfg.disarm_norm is not None:
            return self._aux_norm_to_count(name, float(cfg.disarm_norm))
        mode = str(getattr(cfg, "input_mode", "norm01") or "norm01").strip().lower()
        if mode == "signed":
            pulse = float(getattr(cfg, "center_us", 1500)) + float(getattr(cfg, "trim_us", 0))
            lo = min(float(cfg.min_us), float(cfg.max_us))
            hi = max(float(cfg.min_us), float(cfg.max_us))
            pulse = _clamp(pulse, lo, hi)
            return us_to_count(pulse, self.cfg.freq_hz)
        return self._aux_norm_to_count(name, 0.0)

    def _aux_shutdown_count(self, name: str) -> int:
        cfg = self.aux_cfg.get(name, AuxOutputConfig())
        if cfg.force_off_on_disarm:
            return self._aux_norm_to_count(name, 0.0)
        if cfg.center_on_disarm or (cfg.disarm_norm is not None):
            return self._aux_disarm_count(name)
        return self._aux_norm_to_count(name, 0.0)

    def _aux_norm_to_count(self, name: str, v_norm: float) -> int:
        cfg = self.aux_cfg.get(name, AuxOutputConfig())
        mode = str(getattr(cfg, "input_mode", "norm01") or "norm01").strip().lower()
        v = float(v_norm)

        lo = min(float(cfg.min_us), float(cfg.max_us))
        hi = max(float(cfg.min_us), float(cfg.max_us))

        if mode == "signed":
            if abs(v) < float(cfg.deadband_norm):
                v = 0.0
            v = _clamp(v, -1.0, 1.0)
            center = float(getattr(cfg, "center_us", 1500))
            if v >= 0.0:
                pulse = center + (float(cfg.max_us) - center) * v
            else:
                pulse = center + (center - float(cfg.min_us)) * v
        else:
            if v < float(cfg.deadband_norm):
                v = 0.0
            v = _clamp(v, 0.0, 1.0)
            if v <= 0.0:
                pulse = float(cfg.off_us)
            else:
                # Linear map [0..1] -> [min_us..max_us]
                pulse = float(cfg.min_us) + (float(cfg.max_us) - float(cfg.min_us)) * v

        pulse += float(cfg.trim_us)
        pulse = _clamp(pulse, lo, hi)
        return us_to_count(pulse, self.cfg.freq_hz)

    def _extract_aux_counts(self, cmd: Mapping[Any, float]) -> List[int]:
        counts: List[int] = []
        for i, name in enumerate(self._aux_order):
            v = None
            # Accept a few common spellings for convenience
            for k in (name, name.lower(), name.upper()):
                if k in cmd:
                    v = float(cmd[k])
                    break

            if v is None:
                # If not present, keep the last value
                counts.append(self._last_aux_counts[i])
                continue

            counts.append(self._aux_norm_to_count(name, v))
        return counts

    # --- actuation -------------------------------------------------
    def write(self, thr: Mapping[Any, float]) -> None:
        """Write thruster (and optional aux) commands.

        If disarmed, thrusters are actively driven to neutral. When hardware_arm_disarm
        is enabled and disable_pwm_on_disarm is True, we avoid re-enabling PWM while
        disarmed (so arming produces an obvious ESC re-arm tone).
        """
        with self._lock:
            cmd = dict(thr or {})
            now = time.time()

            # Aux outputs: compute first
            aux_counts = list(self._last_aux_counts)
            if self._aux_order:
                new_aux_counts = self._extract_aux_counts(cmd)

                # If disarmed and aux disallowed, keep last (or defaults).
                if not self._armed:
                    for i, name in enumerate(self._aux_order):
                        if not self.aux_cfg.get(name, AuxOutputConfig()).allow_when_disarmed:
                            new_aux_counts[i] = aux_counts[i]

                aux_counts = new_aux_counts
                self._last_aux_counts = list(aux_counts)

            # Determine whether we should keep PWM physically disabled while disarmed.
            if bool(getattr(self.cfg, "hardware_arm_disarm", False)):
                disable_on_disarm = bool(getattr(self.cfg, "disable_pwm_on_disarm", True))
            else:
                disable_on_disarm = (not bool(getattr(self.cfg, "keep_pwm_enabled_on_disarm", True)))

            # If we're disarmed and PWM is disabled, don't re-enable it just to drive neutral.
            if (not self._armed) and disable_on_disarm and (not self._pwm.enabled):
                return

            # Otherwise ensure outputs are live.
            self._ensure_pwm_enabled()

            # Disarmed or still in the ESC init/arming hold window => neutral thrusters.
            if (not self._armed) or (self._arming_until and now < float(self._arming_until)):
                self._reset_slew_state(now=now)
                self._apply_outputs(self._neutral_thruster_counts(), aux_counts)
                return

            targets_norm: List[float] = []
            for name, ch_user in zip(self._thruster_order, self._thruster_user_channels):
                t = float(self._get_cmd_value(cmd, name, ch_user))

                if self._is_reversed(name, ch_user):
                    t = -t

                # Apply the same normalized deadband before slew limiting so tiny stick jitter
                # does not accumulate in the slew state while the output remains neutral.
                if abs(t) < float(self.cfg.deadband_norm):
                    t = 0.0

                targets_norm.append(_clamp(t, -1.0, 1.0))

            slewed_norm = self._apply_slew_limit(targets_norm, now=now)

            thruster_counts: List[int] = []
            for name, t in zip(self._thruster_order, slewed_norm):
                pulse = thrust_to_us(
                    t,
                    neutral_us=self.cfg.neutral_us,
                    span_us=self.cfg.span_us,
                    min_us=self.cfg.min_us,
                    max_us=self.cfg.max_us,
                    deadband_norm=self.cfg.deadband_norm,
                )
                pulse += self._trim_for(name)

                # Microsecond deadband around neutral.
                if abs(pulse - int(self.cfg.neutral_us)) < int(self.cfg.deadband_us):
                    pulse = int(self.cfg.neutral_us)

                thruster_counts.append(us_to_count(pulse, self.cfg.freq_hz))

            self._apply_outputs(thruster_counts, aux_counts)


def write_thrust(thr: Mapping[str, float]) -> None:
    """Convenience global writer (lazy singleton).

    Prefer creating a ThrustWriter instance and passing it to ControlService.
    """
    global _GLOBAL_THRUST_WRITER
    if _GLOBAL_THRUST_WRITER is None:
        _GLOBAL_THRUST_WRITER = ThrustWriter()
        _GLOBAL_THRUST_WRITER.arm()
    _GLOBAL_THRUST_WRITER.write(thr)

_GLOBAL_THRUST_WRITER: Optional[ThrustWriter] = None
