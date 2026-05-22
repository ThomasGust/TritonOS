"""Minimal ADS1115 driver (I2C).

Navigator exposes an ADS1115 ADC at 0x48. This is used for battery voltage,
current sense, and/or auxiliary analog inputs depending on wiring.

This driver supports single-shot reads of AIN0..AIN3 (single-ended) and
returns values in volts.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from smbus2 import SMBus


ADS1115_ADDR_DEFAULT = 0x48


def _s16(v: int) -> int:
    return v - 65536 if v & 0x8000 else v


@dataclass
class ADS1115Reading:
    """Snapshot of all ADS1115 single-ended channels in volts."""

    ts: float
    volts: list[float]


class ADS1115:
    """Minimal ADS1115 single-shot reader for Navigator analog channels."""

    # Registers
    REG_CONVERSION = 0x00
    REG_CONFIG = 0x01

    def __init__(self, bus: int, addr: int = ADS1115_ADDR_DEFAULT, pga_v: float = 4.096, dr_sps: int = 128):
        self.bus_no = int(bus)
        self.addr = int(addr)
        self.pga_v = float(pga_v)
        self.dr_sps = int(dr_sps)

        # Data rate bits (DR[2:0]) mapping for ADS1115
        self._dr_bits = {
            8: 0b000,
            16: 0b001,
            32: 0b010,
            64: 0b011,
            128: 0b100,
            250: 0b101,
            475: 0b110,
            860: 0b111,
        }.get(self.dr_sps, 0b100)

        # PGA bits for +-FS
        self._pga_bits = {
            6.144: 0b000,
            4.096: 0b001,
            2.048: 0b010,
            1.024: 0b011,
            0.512: 0b100,
            0.256: 0b101,
        }.get(self.pga_v, 0b001)

    def _read_channel(self, ch: int) -> float:
        ch = int(ch)
        if ch not in (0, 1, 2, 3):
            raise ValueError("ADS1115 channel must be 0..3")

        # MUX bits for single-ended AINx vs GND
        mux_bits = {0: 0b100, 1: 0b101, 2: 0b110, 3: 0b111}[ch]

        # CONFIG layout:
        # [15] OS=1 (start single conversion)
        # [14:12] MUX
        # [11:9] PGA
        # [8] MODE=1 (single-shot)
        # [7:5] DR
        # [4] COMP_MODE=0
        # [3] COMP_POL=0
        # [2] COMP_LAT=0
        # [1:0] COMP_QUE=11 (disable comparator)
        cfg = 0
        cfg |= (1 << 15)
        cfg |= (mux_bits & 0x7) << 12
        cfg |= (self._pga_bits & 0x7) << 9
        cfg |= (1 << 8)
        cfg |= (self._dr_bits & 0x7) << 5
        cfg |= 0b11

        with SMBus(self.bus_no) as b:
            b.write_i2c_block_data(self.addr, self.REG_CONFIG, [(cfg >> 8) & 0xFF, cfg & 0xFF])

        # Wait for conversion. At 128 SPS, one conversion ~7.8ms.
        time.sleep(max(0.002, 1.0 / max(8, self.dr_sps)))

        with SMBus(self.bus_no) as b:
            raw = b.read_i2c_block_data(self.addr, self.REG_CONVERSION, 2)
        code = _s16((raw[0] << 8) | raw[1])
        # volts = code / 32768 * FS
        return float(code) * (self.pga_v / 32768.0)

    def read_all(self) -> ADS1115Reading:
        """Read all four single-ended channels and return volts."""

        volts = [self._read_channel(i) for i in range(4)]
        return ADS1115Reading(ts=time.time(), volts=volts)
