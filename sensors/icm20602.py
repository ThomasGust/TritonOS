"""Minimal I2C driver for the Navigator's IMU (ICM-20602).

We intentionally avoid the `bluerobotics_navigator` bindings here because they
can grab hardware resources (notably the PWM_OE line) that conflict with our
custom PCA9685 thruster control.

This is a *minimal* driver intended for pool tests:
  - accel (m/s^2)
  - gyro (rad/s)
  - temperature (°C) (optional)
"""

from __future__ import annotations

import math
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


def _twos(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


class ICM20602:
    # Common I2C address for ICM-20602 on many designs
    DEFAULT_ADDRS = (0x68, 0x69)

    # Register map (matches MPU/ICM family)
    REG_WHOAMI = 0x75
    REG_PWR_MGMT_1 = 0x6B
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_CONFIG2 = 0x1D
    REG_ACCEL_XOUT_H = 0x3B

    WHOAMI_EXPECTED = {0x12, 0x11, 0x68}  # tolerate variants

    def __init__(self, bus: int = 1, addr: int | None = None):
        self.bus_no = int(bus)
        self.bus = SMBus(self.bus_no)

        self.addr = addr
        if self.addr is None:
            self.addr = self._auto_detect_addr()

        # Wake up
        self.bus.write_byte_data(self.addr, self.REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # Set sample rate and basic filtering.
        # Keep it conservative; the AHRS runs its own filtering.
        self.bus.write_byte_data(self.addr, self.REG_SMPLRT_DIV, 0x04)  # ~200 Hz base
        self.bus.write_byte_data(self.addr, self.REG_CONFIG, 0x03)      # DLPF

        # ±250 dps (FS_SEL=0), ±2g (AFS_SEL=0)
        self.bus.write_byte_data(self.addr, self.REG_GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.addr, self.REG_ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.addr, self.REG_ACCEL_CONFIG2, 0x03)

        # scales
        self._accel_lsb_per_g = 16384.0
        self._gyro_lsb_per_dps = 131.0

    def close(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass

    def _auto_detect_addr(self) -> int:
        for a in self.DEFAULT_ADDRS:
            try:
                who = int(self.bus.read_byte_data(a, self.REG_WHOAMI))
                if who in self.WHOAMI_EXPECTED:
                    return a
            except Exception:
                continue
        raise RuntimeError(f"ICM-20602 not found on I2C bus {self.bus_no} at {self.DEFAULT_ADDRS}")

    def _read14(self) -> list[int]:
        # accel(6) + temp(2) + gyro(6)
        return list(self.bus.read_i2c_block_data(self.addr, self.REG_ACCEL_XOUT_H, 14))

    def read_accel(self) -> Vec3:
        b = self._read14()
        ax = _twos((b[0] << 8) | b[1])
        ay = _twos((b[2] << 8) | b[3])
        az = _twos((b[4] << 8) | b[5])
        g = 9.80665
        return Vec3(
            x=(ax / self._accel_lsb_per_g) * g,
            y=(ay / self._accel_lsb_per_g) * g,
            z=(az / self._accel_lsb_per_g) * g,
        )

    def read_gyro(self) -> Vec3:
        b = self._read14()
        gx = _twos((b[8] << 8) | b[9])
        gy = _twos((b[10] << 8) | b[11])
        gz = _twos((b[12] << 8) | b[13])
        dps_to_rad = math.pi / 180.0
        return Vec3(
            x=(gx / self._gyro_lsb_per_dps) * dps_to_rad,
            y=(gy / self._gyro_lsb_per_dps) * dps_to_rad,
            z=(gz / self._gyro_lsb_per_dps) * dps_to_rad,
        )

    def read_temp_c(self) -> float:
        b = self._read14()
        tr = _twos((b[6] << 8) | b[7])
        # MPU/ICM family temp scaling. Good enough for telemetry.
        return (tr / 326.8) + 25.0
