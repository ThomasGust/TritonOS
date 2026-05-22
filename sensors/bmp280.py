"""Minimal BMP280 driver (I2C).

The Blue Robotics Navigator includes a BMP280 barometer/temperature sensor.
For Rev1 we just need:
  - compensated temperature (°C)
  - compensated pressure (Pa)

Implementation follows the Bosch BMP280 datasheet compensation algorithm.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from smbus2 import SMBus


BMP280_ADDR_DEFAULT = 0x76


def _u16(lo: int, hi: int) -> int:
    return (hi << 8) | lo


def _s16(lo: int, hi: int) -> int:
    v = _u16(lo, hi)
    return v - 65536 if v & 0x8000 else v


@dataclass
class BMP280Reading:
    """Compensated BMP280 temperature and pressure sample."""

    ts: float
    temperature_c: float
    pressure_pa: float


class BMP280:
    """Minimal BMP280 reader with Bosch compensation math."""

    def __init__(self, bus: int, addr: int = BMP280_ADDR_DEFAULT):
        self.bus_no = int(bus)
        self.addr = int(addr)
        self._t_fine = 0.0

        # Read calibration data once.
        with SMBus(self.bus_no) as b:
            calib = b.read_i2c_block_data(self.addr, 0x88, 24)

        self.dig_T1 = _u16(calib[0], calib[1])
        self.dig_T2 = _s16(calib[2], calib[3])
        self.dig_T3 = _s16(calib[4], calib[5])

        self.dig_P1 = _u16(calib[6], calib[7])
        self.dig_P2 = _s16(calib[8], calib[9])
        self.dig_P3 = _s16(calib[10], calib[11])
        self.dig_P4 = _s16(calib[12], calib[13])
        self.dig_P5 = _s16(calib[14], calib[15])
        self.dig_P6 = _s16(calib[16], calib[17])
        self.dig_P7 = _s16(calib[18], calib[19])
        self.dig_P8 = _s16(calib[20], calib[21])
        self.dig_P9 = _s16(calib[22], calib[23])

        # Configure: oversampling x2 temp, x4 pressure, normal mode, IIR filter.
        # ctrl_meas (0xF4): osrs_t=010, osrs_p=011, mode=11
        # config    (0xF5): t_sb=000, filter=100, spi3w_en=0
        with SMBus(self.bus_no) as b:
            b.write_byte_data(self.addr, 0xF5, 0b00010000)
            b.write_byte_data(self.addr, 0xF4, 0b01001111)
        time.sleep(0.05)

    def read(self) -> BMP280Reading:
        """Read and compensate the latest temperature/pressure sample."""

        # Raw data: pressure [19:0] then temp [19:0]
        with SMBus(self.bus_no) as b:
            data = b.read_i2c_block_data(self.addr, 0xF7, 6)

        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        # Temperature compensation
        var1 = (adc_t / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_t / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        self._t_fine = var1 + var2
        temp_c = self._t_fine / 5120.0

        # Pressure compensation
        var1p = self._t_fine / 2.0 - 64000.0
        var2p = var1p * var1p * self.dig_P6 / 32768.0
        var2p = var2p + var1p * self.dig_P5 * 2.0
        var2p = var2p / 4.0 + self.dig_P4 * 65536.0
        var1p = (self.dig_P3 * var1p * var1p / 524288.0 + self.dig_P2 * var1p) / 524288.0
        var1p = (1.0 + var1p / 32768.0) * self.dig_P1

        if var1p == 0:
            pressure_pa = 0.0
        else:
            p = 1048576.0 - adc_p
            p = (p - var2p / 4096.0) * 6250.0 / var1p
            var1p = self.dig_P9 * p * p / 2147483648.0
            var2p = p * self.dig_P8 / 32768.0
            p = p + (var1p + var2p + self.dig_P7) / 16.0
            pressure_pa = float(p)

        return BMP280Reading(ts=time.time(), temperature_c=float(temp_c), pressure_pa=pressure_pa)
