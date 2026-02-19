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
        v_batt_min: float = 5.0,
        v_batt_max: float = 30.0,
        i_min: float = -5.0,
        i_max: float = 150.0,
        reselect_after_bad: int = 5,
        always_autodetect: bool = False,
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

        self._v_ch: Optional[int] = int(volt_ch) if volt_ch is not None else None
        self._i_ch: Optional[int] = int(curr_ch) if curr_ch is not None else None
        self._fixed = (volt_ch is not None and curr_ch is not None)
        self._always_autodetect = bool(always_autodetect)

        self._bad_count = 0
        self._reselect_after_bad = int(max(1, reselect_after_bad))

        # For scoring during auto-detection
        self._last_v_batt: Optional[float] = None
        self._last_i_amps: Optional[float] = None

    # ---- math -----------------------------------------------------------
    def _convert(self, adc_v: float, adc_i: float) -> Tuple[float, float]:
        v_batt = float(adc_v) * self.volt_mult
        i_amps = (float(adc_i) - self.amps_offset_v) * self.amps_per_volt
        return v_batt, i_amps

    def _plausible(self, v_batt: float, i_amps: float) -> bool:
        return (self.v_batt_min <= v_batt <= self.v_batt_max) and (self.i_min <= i_amps <= self.i_max)

    # ---- channel selection ---------------------------------------------
    def _score_pair(self, chans: List[float], v_ch: int, i_ch: int) -> float:
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

        return score

    def _autodetect(self, chans: List[float]) -> Optional[Tuple[int, int, float]]:
        if not chans:
            return None
        n = len(chans)
        best: Tuple[int, int, float] | None = None
        for v_ch in range(n):
            for i_ch in range(n):
                if i_ch == v_ch:
                    continue
                s = self._score_pair(chans, v_ch, i_ch)
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
            raw = self.board.read_adc()
            chans = list(raw.channel) if hasattr(raw, "channel") else list(raw)
            chans = [float(x) for x in chans]
        except Exception as e:
            out["error"] = str(e)
            return out

        out["raw_channels_v"] = chans

        # Decide mapping.
        if self._always_autodetect or (self._v_ch is None or self._i_ch is None):
            det = self._autodetect(chans)
            if det is not None:
                self._v_ch, self._i_ch, best_score = det
                out["autodetect_score"] = float(best_score)

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
        v_batt, i_amps = self._convert(adc_v, adc_i)

        out.update(
            {
                "voltage_v": float(v_batt),
                "current_a": float(i_amps),
                "power_w": float(v_batt * i_amps),
                "voltage_sense_v": float(adc_v),
                "current_sense_v": float(adc_i),
                "voltage_ch": int(v_ch),
                "current_ch": int(i_ch),
                "volt_mult": float(self.volt_mult),
                "amps_per_volt": float(self.amps_per_volt),
                "amps_offset_v": float(self.amps_offset_v),
            }
        )

        # Validate; if mapping becomes implausible, trigger re-detection.
        ok = self._plausible(v_batt, i_amps)
        out["ok"] = bool(ok)

        if ok:
            self._bad_count = 0
            self._last_v_batt = float(v_batt)
            self._last_i_amps = float(i_amps)
        else:
            self._bad_count += 1
            out["bad_count"] = int(self._bad_count)
            if (not self._fixed) and self._bad_count >= self._reselect_after_bad:
                # Forget mapping and try again next cycle.
                self._v_ch = None
                self._i_ch = None
                self._bad_count = 0

        return out
