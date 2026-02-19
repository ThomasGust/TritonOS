"""Power Sense Module (Blue Robotics PSM).

Publishes converted voltage/current telemetry as a *separate* sensor message.

The Navigator exposes an ADS1115 ADC (read via NavigatorBoard.read_adc()), and
the PSM provides two analog outputs:
  - VOLTAGE:   V_batt = V_adc * VOLT_MULT
  - CURRENT:   I_amps = (V_adc - AMPS_OFFSET_V) * AMPS_PER_VOLT

This sensor supports either fixed channel selection (0..3) or best-effort
auto-detection when channel ordering/wiring isn't stable.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

from sensors.base import BaseSensor


def _is_num(x: object) -> bool:
    return isinstance(x, (int, float))


def _median(xs: List[float]) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    n = len(s)
    mid = n // 2
    if n % 2:
        return float(s[mid])
    return float((s[mid - 1] + s[mid]) / 2.0)


def _normalize_adc(raw: object) -> List[float]:
    """Normalize ADC readings to a stable list.

    Handles both the pure-python path (already a list) and common bindings
    return types (dicts, list of (ch,val), or struct with .channel/.volts).
    """
    if raw is None:
        return []

    # Attribute variants
    if hasattr(raw, "volts"):
        raw = getattr(raw, "volts")
    elif hasattr(raw, "channel"):
        raw = getattr(raw, "channel")

    # Dict-like
    if isinstance(raw, dict):
        items = list(raw.items())

        def _k(k):
            try:
                return int(k)
            except Exception:
                return str(k)

        items.sort(key=lambda kv: _k(kv[0]))
        return [float(v) for _, v in items]

    # Sequence
    if isinstance(raw, (list, tuple)):
        if len(raw) == 0:
            return []
        if all(_is_num(x) for x in raw):
            return [float(x) for x in raw]
        if all(isinstance(x, (list, tuple)) and len(x) == 2 for x in raw):
            d: Dict[int, float] = {}
            for ch, v in raw:  # type: ignore[misc]
                try:
                    d[int(ch)] = float(v)
                except Exception:
                    continue
            return [float(v) for _, v in sorted(d.items(), key=lambda kv: kv[0])]

    # Fallback: try iter()
    try:
        return [float(x) for x in list(raw)]  # type: ignore[arg-type]
    except Exception:
        return []


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class PowerSenseSensor(BaseSensor):
    def __init__(
        self,
        board,
        rate_hz: float = 2.0,
        *,
        name: str = "power",
        volt_mult: float = 11.0,
        amps_per_volt: float = 37.8788,
        amps_offset_v: float = 0.330,
        volt_ch: Optional[int] = None,
        curr_ch: Optional[int] = None,
        # Ranges for plausibility checks (in converted units).
        v_batt_min: float = 5.0,
        v_batt_max: float = 30.0,
        i_min: float = -5.0,
        i_max: float = 150.0,
        # Sampling + filtering
        samples_per_read: int = 5,
        ema_alpha: float = 0.3,
        voltage_step_max_v: float = 3.0,
        current_step_max_a: float = 25.0,
        negative_current_clamp_a: float = 0.75,
        hold_last_good: bool = True,
        # Channel selection behavior
        track_channels: bool = True,
        switch_penalty: float = 80.0,
        reselect_after_bad: int = 0,
    ):
        super().__init__(name, rate_hz)
        self.board = board

        self.volt_mult = float(volt_mult)
        self.amps_per_volt = float(amps_per_volt)
        self.amps_offset_v = float(amps_offset_v)

        self.v_batt_min = float(v_batt_min)
        self.v_batt_max = float(v_batt_max)
        self.i_min = float(i_min)
        self.i_max = float(i_max)

        self.samples_per_read = int(max(1, samples_per_read))
        self.ema_alpha = float(_clamp(float(ema_alpha), 0.0, 1.0))
        self.voltage_step_max_v = float(max(0.0, voltage_step_max_v))
        self.current_step_max_a = float(max(0.0, current_step_max_a))
        self.negative_current_clamp_a = float(max(0.0, negative_current_clamp_a))
        self.hold_last_good = bool(hold_last_good)

        self._v_ch: Optional[int] = int(volt_ch) if volt_ch is not None else None
        self._i_ch: Optional[int] = int(curr_ch) if curr_ch is not None else None
        self._fixed = (volt_ch is not None and curr_ch is not None)

        # If channels are fixed, tracking is unnecessary (and could mask wiring issues).
        self.track_channels = bool(track_channels) and (not self._fixed)
        self.switch_penalty = float(max(0.0, switch_penalty))

        self._bad_count = 0
        self._reselect_after_bad = int(max(0, reselect_after_bad))

        # For continuity + filtering
        self._last_v_batt: Optional[float] = None
        self._last_i_amps: Optional[float] = None
        self._ema_v: Optional[float] = None
        self._ema_i: Optional[float] = None

    # ---- math -----------------------------------------------------------
    def _convert(self, adc_v: float, adc_i: float) -> Tuple[float, float]:
        v_batt = float(adc_v) * self.volt_mult
        i_amps = (float(adc_i) - self.amps_offset_v) * self.amps_per_volt
        return v_batt, i_amps

    def _plausible(self, v_batt: float, i_amps: float) -> bool:
        return (self.v_batt_min <= v_batt <= self.v_batt_max) and (self.i_min <= i_amps <= self.i_max)

    # ---- channel selection ---------------------------------------------
    def _score_pair(
        self,
        chans: List[float],
        v_ch: int,
        i_ch: int,
        *,
        prefer_v_ch: Optional[int] = None,
        prefer_i_ch: Optional[int] = None,
    ) -> float:
        adc_v = float(chans[v_ch])
        adc_i = float(chans[i_ch])
        v_batt, i_amps = self._convert(adc_v, adc_i)

        score = 0.0

        # Hard penalties for implausible ranges.
        if not (self.v_batt_min <= v_batt <= self.v_batt_max):
            # distance outside range
            dv = 0.0
            if v_batt < self.v_batt_min:
                dv = self.v_batt_min - v_batt
            elif v_batt > self.v_batt_max:
                dv = v_batt - self.v_batt_max
            score += 1000.0 + dv * 10.0

        if not (self.i_min <= i_amps <= self.i_max):
            di = 0.0
            if i_amps < self.i_min:
                di = self.i_min - i_amps
            elif i_amps > self.i_max:
                di = i_amps - self.i_max
            score += 1000.0 + di * 5.0

        # Prefer continuity with previous readings (reduces channel flapping).
        if self._last_v_batt is not None:
            score += abs(v_batt - self._last_v_batt) * 0.5
        else:
            # Prefer sane mid-range (typical 3S-6S batteries)
            score += abs(v_batt - 16.0) * 0.05

        if self._last_i_amps is not None:
            score += abs(i_amps - self._last_i_amps) * 0.05
        else:
            score += abs(i_amps - 0.0) * 0.01

        # Gentle preference: current sense output is offset at ~0.33 V at 0 A.
        score += abs(adc_i - self.amps_offset_v) * 0.2

        # Gentle preference: voltage sense tends to be > ~0.6 V for any real battery.
        if adc_v < 0.4:
            score += 10.0

        # Strong hysteresis: prefer staying on the same channels unless clearly wrong.
        if prefer_v_ch is not None and v_ch != int(prefer_v_ch):
            score += self.switch_penalty
        if prefer_i_ch is not None and i_ch != int(prefer_i_ch):
            score += self.switch_penalty

        return score

    def _autodetect(
        self,
        chans: List[float],
        *,
        prefer_v_ch: Optional[int] = None,
        prefer_i_ch: Optional[int] = None,
    ) -> Optional[Tuple[int, int, float]]:
        if not chans:
            return None
        n = len(chans)
        best: Tuple[int, int, float] | None = None
        for v_ch in range(n):
            for i_ch in range(n):
                if i_ch == v_ch:
                    continue
                s = self._score_pair(chans, v_ch, i_ch, prefer_v_ch=prefer_v_ch, prefer_i_ch=prefer_i_ch)
                if best is None or s < best[2]:
                    best = (v_ch, i_ch, s)
        return best

    # ---- public API -----------------------------------------------------
    def read(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": self.name,
            "type": "power",
        }

        try:
            # Take multiple ADC samples and use a per-channel median to reject
            # sporadic spikes / ordering quirks.
            samples: List[List[float]] = []
            for _ in range(self.samples_per_read):
                raw = self.board.read_adc()
                c = _normalize_adc(raw)
                if c:
                    samples.append([float(x) for x in c])

            if not samples:
                out["error"] = "no ADC samples"
                return out

            n = min(len(s) for s in samples)
            samples = [s[:n] for s in samples]
            chans = [_median([s[i] for s in samples]) for i in range(n)]
            out["samples"] = int(len(samples))
        except Exception as e:
            out["error"] = str(e)
            return out

        out["raw_channels_v"] = chans

        # Decide / track mapping.
        if self.track_channels or (self._v_ch is None or self._i_ch is None):
            det = self._autodetect(chans, prefer_v_ch=self._v_ch, prefer_i_ch=self._i_ch)
            if det is not None:
                self._v_ch, self._i_ch, best_score = det
                out["autodetect_score"] = float(best_score)
                out["tracking"] = bool(self.track_channels)

        # If fixed mapping was provided, just use it.
        if self._v_ch is None or self._i_ch is None:
            out["error"] = "power sense channels not available"
            return out

        v_ch = int(self._v_ch)
        i_ch = int(self._i_ch)
        if v_ch < 0 or v_ch >= len(chans) or i_ch < 0 or i_ch >= len(chans) or v_ch == i_ch:
            out["error"] = f"invalid power sense channel mapping v_ch={v_ch} i_ch={i_ch}"
            # Force a re-detect next read if not fixed.
            if not self._fixed:
                self._v_ch = None
                self._i_ch = None
            return out

        adc_v = float(chans[v_ch])
        adc_i = float(chans[i_ch])
        raw_v_batt, raw_i_amps = self._convert(adc_v, adc_i)

        # Clamp tiny negative currents (offset/noise) but treat large negatives as invalid.
        if raw_i_amps < 0 and abs(raw_i_amps) <= self.negative_current_clamp_a:
            raw_i_amps = 0.0

        # Basic plausibility + step/outlier check.
        ok_raw = self._plausible(raw_v_batt, raw_i_amps)
        step_ok = True
        if self._last_v_batt is not None and self.voltage_step_max_v > 0:
            if abs(raw_v_batt - float(self._last_v_batt)) > self.voltage_step_max_v:
                step_ok = False
        if self._last_i_amps is not None and self.current_step_max_a > 0:
            if abs(raw_i_amps - float(self._last_i_amps)) > self.current_step_max_a:
                step_ok = False

        ok = bool(ok_raw and step_ok)
        out["ok_raw"] = bool(ok_raw)
        out["ok"] = bool(ok)

        # Filtering / hold-last-good behavior.
        held = False
        v_batt = float(raw_v_batt)
        i_amps = float(raw_i_amps)

        if not ok:
            self._bad_count += 1
            out["bad_count"] = int(self._bad_count)
            out["raw_voltage_v"] = float(raw_v_batt)
            out["raw_current_a"] = float(raw_i_amps)
            out["raw_power_w"] = float(raw_v_batt * raw_i_amps)

            if self.hold_last_good and (self._ema_v is not None or self._last_v_batt is not None):
                held = True
                v_batt = float(self._ema_v if self._ema_v is not None else self._last_v_batt)
                i_amps = float(self._ema_i if self._ema_i is not None else self._last_i_amps)

            # Optional: if things look broken for a long time, allow a full reselect.
            if (not self._fixed) and self._reselect_after_bad > 0 and self._bad_count >= self._reselect_after_bad:
                self._v_ch = None
                self._i_ch = None
                self._bad_count = 0
        else:
            self._bad_count = 0

            # EMA smoothing
            if self.ema_alpha > 0.0:
                if self._ema_v is None:
                    self._ema_v = float(v_batt)
                    self._ema_i = float(i_amps)
                else:
                    a = float(self.ema_alpha)
                    self._ema_v = float(a * v_batt + (1.0 - a) * float(self._ema_v))
                    self._ema_i = float(a * i_amps + (1.0 - a) * float(self._ema_i))
                v_batt = float(self._ema_v)
                i_amps = float(self._ema_i)

            self._last_v_batt = float(v_batt)
            self._last_i_amps = float(i_amps)
        print("volts", v_batt, adc_v, v_ch)
        print("amps", i_amps, adc_i, i_ch)
        out.update(
            {
                "voltage_v": float(v_batt),
                "current_a": float(i_amps),
                "power_w": float(v_batt * i_amps),
                "voltage_sense_v": float(adc_v),
                "current_sense_v": float(adc_i),
                "voltage_ch": int(v_ch),
                "current_ch": int(i_ch),
                "held": bool(held),
                "volt_mult": float(self.volt_mult),
                "amps_per_volt": float(self.amps_per_volt),
                "amps_offset_v": float(self.amps_offset_v),
            }
        )

        return out
