"""Minimal I2C driver for AK09915 3-axis magnetometer.

The Navigator includes an AK09915 and (often) an MMC5983. We use this driver
to read the AK09915 without pulling in the Navigator Rust/PyO3 bindings, which
can conflict with PWM_OE usage.

Reference: AK09915C datasheet (register map / fixed WIA values).
"""

from __future__ import annotations

import time
from dataclasses import dataclass


try:
    from smbus2 import SMBus
except Exception:  # pragma: no cover
    from smbus import SMBus  # type: ignore


@dataclass
class Vec3:
    x: float
    y: float
    z: float


def _twos16(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


class AK09915:
    # Address depends on CAD pins; try a small set.
    CANDIDATE_ADDRS = (0x0C, 0x0D, 0x0E, 0x0F)

    # Key registers
    REG_WIA1 = 0x00
    REG_WIA2 = 0x01
    REG_ST1 = 0x10
    REG_HXL = 0x11
    REG_ST2 = 0x18
    REG_CNTL2 = 0x31
    REG_CNTL3 = 0x32

    WIA1_EXPECTED = 0x48
    WIA2_EXPECTED = 0x10

    # CNTL2 modes (datasheet Table 9.2)
    MODE_POWER_DOWN = 0x00
    MODE_SINGLE = 0x01
    MODE_CONT_10HZ = 0x02
    MODE_CONT_20HZ = 0x04
    MODE_CONT_50HZ = 0x06
    MODE_CONT_100HZ = 0x08

    # Sensitivity: 0.15 uT/LSB typical (datasheet feature list)
    UT_PER_LSB = 0.15

    def __init__(self, bus: int = 1, addr: int | None = None, mode: int = MODE_CONT_100HZ):
        self.bus_no = int(bus)
        self.bus = SMBus(self.bus_no)
        self.addr = addr if addr is not None else self._auto_detect_addr()

        # Reset then enter continuous mode
        self.reset()
        self.set_mode(mode)

    def close(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass

    def _auto_detect_addr(self) -> int:
        for a in self.CANDIDATE_ADDRS:
            try:
                w1 = int(self.bus.read_byte_data(a, self.REG_WIA1))
                w2 = int(self.bus.read_byte_data(a, self.REG_WIA2))
                if w1 == self.WIA1_EXPECTED and w2 == self.WIA2_EXPECTED:
                    return a
            except Exception:
                continue
        raise RuntimeError(f"AK09915 not found on I2C bus {self.bus_no} at {self.CANDIDATE_ADDRS}")

    def reset(self) -> None:
        # Software reset: CNTL3.SRST = 1
        self.bus.write_byte_data(self.addr, self.REG_CNTL3, 0x01)
        time.sleep(0.01)

    def set_mode(self, mode: int) -> None:
        # Power-down first, then requested mode
        self.bus.write_byte_data(self.addr, self.REG_CNTL2, self.MODE_POWER_DOWN)
        time.sleep(0.002)
        self.bus.write_byte_data(self.addr, self.REG_CNTL2, int(mode) & 0x1F)
        time.sleep(0.002)

    def read_uT(self) -> Vec3:
        """Read X/Y/Z in microtesla.

        If data isn't ready, returns the last latched sample (best-effort).
        """
        # If DRDY isn't set, we still attempt a read (some boards behave oddly).
        try:
            st1 = int(self.bus.read_byte_data(self.addr, self.REG_ST1))
            _ = st1
        except Exception:
            pass

        data = list(self.bus.read_i2c_block_data(self.addr, self.REG_HXL, 6))
        # Little endian, two's complement
        x = _twos16((data[1] << 8) | data[0])
        y = _twos16((data[3] << 8) | data[2])
        z = _twos16((data[5] << 8) | data[4])

        # Must read ST2 to complete a read cycle (datasheet 11.3.7)
        try:
            _ = self.bus.read_byte_data(self.addr, self.REG_ST2)
        except Exception:
            pass

        return Vec3(x=x * self.UT_PER_LSB, y=y * self.UT_PER_LSB, z=z * self.UT_PER_LSB)
