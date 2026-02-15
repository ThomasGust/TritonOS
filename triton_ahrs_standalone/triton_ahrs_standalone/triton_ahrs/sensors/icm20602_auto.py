"""ICM-20602 IMU driver with auto-detection.

On a stock Blue Robotics Navigator the ICM-20602 is wired over SPI.
Some custom setups may expose it over I2C (0x68/0x69).

We support both and auto-detect by:
  1) trying I2C buses/address candidates
  2) trying available /dev/spidev* devices

The returned accel is in m/s^2 and gyro in rad/s.
"""

from __future__ import annotations

import glob
import math
import os
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

try:
    from smbus2 import SMBus
except Exception:  # pragma: no cover
    from smbus import SMBus  # type: ignore

try:
    import spidev  # type: ignore
except Exception:  # pragma: no cover
    spidev = None


@dataclass
class Vec3:
    x: float
    y: float
    z: float


def _twos(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


class ICM20602:
    # Register map (MPU/ICM family)
    REG_WHOAMI = 0x75
    REG_PWR_MGMT_1 = 0x6B
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_CONFIG2 = 0x1D
    REG_ACCEL_XOUT_H = 0x3B

    WHOAMI_EXPECTED = {0x12, 0x11, 0x68}
    I2C_ADDRS = (0x68, 0x69)

    def __init__(self, *, i2c_bus: Optional[int] = None, i2c_addr: Optional[int] = None,
                 spi_bus: Optional[int] = None, spi_dev: Optional[int] = None,
                 spi_mode: int = 0, spi_hz: int = 1_000_000):
        self._i2c: Optional[SMBus] = None
        self._i2c_bus_no: Optional[int] = None
        self._i2c_addr: Optional[int] = None

        self._spi = None
        self._spi_bus: Optional[int] = None
        self._spi_dev: Optional[int] = None
        self._spi_mode = int(spi_mode)
        self._spi_hz = int(spi_hz)

        if i2c_bus is not None:
            self._i2c_bus_no = int(i2c_bus)
            self._i2c = SMBus(self._i2c_bus_no)
            self._i2c_addr = int(i2c_addr) if i2c_addr is not None else self._auto_detect_i2c_addr()
            self._init_i2c()
        elif spi_bus is not None and spi_dev is not None:
            self._spi_bus = int(spi_bus)
            self._spi_dev = int(spi_dev)
            if spidev is None:
                raise RuntimeError("spidev not installed; cannot use SPI IMU")
            self._spi = spidev.SpiDev()
            self._spi.open(self._spi_bus, self._spi_dev)
            self._spi.max_speed_hz = self._spi_hz
            self._spi.mode = self._spi_mode
            self._init_spi()
        else:
            raise ValueError("Provide either i2c_bus or spi_bus/spi_dev")

        # scales
        self._accel_lsb_per_g = 16384.0
        self._gyro_lsb_per_dps = 131.0

    def close(self) -> None:
        try:
            if self._i2c is not None:
                self._i2c.close()
        except Exception:
            pass
        try:
            if self._spi is not None:
                self._spi.close()
        except Exception:
            pass

    # ---------- I2C ----------
    def _auto_detect_i2c_addr(self) -> int:
        assert self._i2c is not None
        for a in self.I2C_ADDRS:
            try:
                who = int(self._i2c.read_byte_data(a, self.REG_WHOAMI))
                if who in self.WHOAMI_EXPECTED:
                    return a
            except Exception:
                continue
        raise RuntimeError(f"ICM-20602 not found on I2C bus {self._i2c_bus_no} at {self.I2C_ADDRS}")

    def _init_i2c(self) -> None:
        assert self._i2c is not None and self._i2c_addr is not None
        # Wake up
        self._i2c.write_byte_data(self._i2c_addr, self.REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        # Conservative filtering
        self._i2c.write_byte_data(self._i2c_addr, self.REG_SMPLRT_DIV, 0x04)
        self._i2c.write_byte_data(self._i2c_addr, self.REG_CONFIG, 0x03)
        self._i2c.write_byte_data(self._i2c_addr, self.REG_GYRO_CONFIG, 0x00)
        self._i2c.write_byte_data(self._i2c_addr, self.REG_ACCEL_CONFIG, 0x00)
        self._i2c.write_byte_data(self._i2c_addr, self.REG_ACCEL_CONFIG2, 0x03)

    # ---------- SPI ----------
    def _spi_xfer(self, tx: list[int]) -> list[int]:
        assert self._spi is not None
        return list(self._spi.xfer2(tx))

    def _spi_read(self, reg: int, n: int = 1) -> list[int]:
        # For MPU/ICM SPI, MSB=1 indicates read.
        resp = self._spi_xfer([reg | 0x80] + [0x00] * n)
        return resp[1:]

    def _spi_write(self, reg: int, val: int) -> None:
        self._spi_xfer([reg & 0x7F, val & 0xFF])

    def _init_spi(self) -> None:
        # Reset then wake. Many boards need a small delay.
        self._spi_write(self.REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self._spi_write(self.REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)
        # Filter + scale
        self._spi_write(self.REG_SMPLRT_DIV, 0x04)
        self._spi_write(self.REG_CONFIG, 0x03)
        self._spi_write(self.REG_GYRO_CONFIG, 0x00)
        self._spi_write(self.REG_ACCEL_CONFIG, 0x00)
        self._spi_write(self.REG_ACCEL_CONFIG2, 0x03)

    # ---------- reads ----------
    def _read14(self) -> list[int]:
        if self._i2c is not None:
            assert self._i2c_addr is not None
            return list(self._i2c.read_i2c_block_data(self._i2c_addr, self.REG_ACCEL_XOUT_H, 14))
        return self._spi_read(self.REG_ACCEL_XOUT_H, 14)

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
        return (tr / 326.8) + 25.0

    # ---------- auto-detect helpers ----------
    @classmethod
    def auto_detect(
        cls,
        *,
        i2c_buses: Iterable[int] = (1,),
        spi_devices: Optional[Iterable[Tuple[int, int]]] = None,
        prefer_spi: bool = True,
    ) -> "ICM20602":
        """Create an IMU by trying SPI then I2C (or vice-versa).

        If spi_devices is None we enumerate /dev/spidev*.
        """
        spi_list: list[Tuple[int, int]] = []
        if spi_devices is not None:
            spi_list = list(spi_devices)
        else:
            for p in sorted(glob.glob("/dev/spidev*")):
                base = os.path.basename(p)  # spidevB.D
                try:
                    b_s, d_s = base.replace("spidev", "").split(".")
                    spi_list.append((int(b_s), int(d_s)))
                except Exception:
                    continue

        def try_spi() -> Optional["ICM20602"]:
            if spidev is None:
                return None
            for (b, d) in spi_list:
                # Try common SPI modes 0 then 3
                for mode in (0, 3):
                    try:
                        imu = cls(spi_bus=b, spi_dev=d, spi_mode=mode)
                        who = imu._spi_read(cls.REG_WHOAMI, 1)[0]
                        if int(who) in cls.WHOAMI_EXPECTED:
                            return imu
                        imu.close()
                    except Exception:
                        continue
            return None

        def try_i2c() -> Optional["ICM20602"]:
            for bus in i2c_buses:
                try:
                    return cls(i2c_bus=int(bus))
                except Exception:
                    continue
            return None

        if prefer_spi:
            imu = try_spi() or try_i2c()
        else:
            imu = try_i2c() or try_spi()
        if imu is None:
            raise RuntimeError("ICM-20602 not found on SPI or I2C")
        return imu
