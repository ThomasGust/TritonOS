# sensors/mmc5983.py
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# Optional backends (both common on Raspberry Pi)
try:
    import smbus2  # type: ignore
except Exception:  # pragma: no cover
    smbus2 = None

try:
    import spidev  # type: ignore
except Exception:  # pragma: no cover
    spidev = None


MMC5983_I2C_ADDR = 0x30  # 7-bit address (datasheet: 0b0110000)
REG_XOUT0 = 0x00
REG_STATUS = 0x08
REG_CTRL0 = 0x09
REG_PROD_ID = 0x2F

# Control bits (datasheet examples)
CTRL0_TM_M = 0x01   # initiate measurement
CTRL0_SET = 0x08
CTRL0_RESET = 0x10

# In 18-bit mode, sensitivity is 16384 counts/G and "null field" output is 131072 counts
# (from MMC5983MA datasheet spec table)
COUNTS_PER_G_18B = 16384.0
NULL_FIELD_COUNTS_18B = 131072.0

G_TO_uT = 100.0  # 1 gauss = 100 microtesla


@dataclass
class MagReading:
    x_uT: float
    y_uT: float
    z_uT: float
    ts: float

    def as_dict(self) -> dict:
        return {"x": self.x_uT, "y": self.y_uT, "z": self.z_uT, "ts": self.ts}


class _I2CBackend:
    def __init__(self, bus: int, addr: int = MMC5983_I2C_ADDR):
        if smbus2 is None:
            raise RuntimeError("smbus2 is not available")
        self.bus_id = int(bus)
        self.addr = int(addr)
        self.bus = smbus2.SMBus(self.bus_id)

    def read_reg(self, reg: int) -> int:
        return int(self.bus.read_byte_data(self.addr, reg) & 0xFF)

    def write_reg(self, reg: int, val: int) -> None:
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read_block(self, start_reg: int, n: int) -> bytes:
        data = self.bus.read_i2c_block_data(self.addr, start_reg, n)
        return bytes(int(x) & 0xFF for x in data)


class _SPIBackend:
    def __init__(self, bus: int, cs: int, max_speed_hz: int = 10_000_000, mode: int = 0):
        if spidev is None:
            raise RuntimeError("spidev is not available")
        self.bus_id = int(bus)
        self.cs = int(cs)
        self.spi = spidev.SpiDev()
        self.spi.open(self.bus_id, self.cs)
        self.spi.max_speed_hz = int(max_speed_hz)
        # Navigator's MMC5983 is typically accessed with "MSB=1 => read" register
        # transactions (matches BlueRobotics' reference driver).
        self.spi.mode = int(mode)

    def read_reg(self, reg: int) -> int:
        resp = self.spi.xfer2([int(reg) | 0x80, 0x00])
        return int(resp[1] & 0xFF)

    def write_reg(self, reg: int, val: int) -> None:
        self.spi.xfer2([int(reg) & 0x7F, int(val) & 0xFF])

    def read_block(self, start_reg: int, n: int) -> bytes:
        resp = self.spi.xfer2([int(start_reg) | 0x80] + [0x00] * int(n))
        return bytes(int(b) & 0xFF for b in resp[1:])  # drop cmd echo


class MMC5983:
    """
    Minimal MMC5983MA magnetometer interface (I2C or SPI).

    This is intentionally small: it supports:
      - device detection via product ID register
      - single-shot measurement
      - optional SET/RESET pair to cancel bridge offset
    """

    def __init__(self, backend: object, *, use_set_reset: bool = True, status_timeout_s: float = 0.02):
        self._b = backend
        self.use_set_reset = bool(use_set_reset)
        self.status_timeout_s = float(status_timeout_s)

        pid = self._b.read_reg(REG_PROD_ID)
        if pid != 0x30:
            raise RuntimeError(f"MMC5983 not detected (product id 0x{pid:02X})")

    @classmethod
    def auto_detect(
        cls,
        *,
        i2c_buses: Sequence[int] = (6, 1),
        spi_devices: Sequence[Tuple[int, int]] = ((0, 0), (0, 1), (1, 0), (1, 1)),
        use_set_reset: bool = True,
    ) -> Optional["MMC5983"]:
        # Try I2C first (cheap), then SPI.
        for bus in i2c_buses:
            try:
                b = _I2CBackend(int(bus))
                pid = b.read_reg(REG_PROD_ID)
                if pid == 0x30:
                    return cls(b, use_set_reset=use_set_reset)
            except Exception:
                continue

        for bus, cs in spi_devices:
            try:
                b = _SPIBackend(int(bus), int(cs))
                pid = b.read_reg(REG_PROD_ID)
                if pid == 0x30:
                    return cls(b, use_set_reset=use_set_reset)
            except Exception:
                continue

        return None

    def _wait_meas_done(self) -> None:
        t0 = time.monotonic()
        while True:
            st = self._b.read_reg(REG_STATUS)
            if st & 0x01:
                return
            if (time.monotonic() - t0) > self.status_timeout_s:
                raise TimeoutError("MMC5983 measurement timeout")
            time.sleep(0.0005)

    def _trigger_measurement(self) -> None:
        self._b.write_reg(REG_CTRL0, CTRL0_TM_M)

    def _read_raw_18b(self) -> Tuple[int, int, int]:
        # Start measurement and wait for completion
        self._trigger_measurement()
        self._wait_meas_done()

        # Read 7 bytes: X[17:10], X[9:2], Y[17:10], Y[9:2], Z[17:10], Z[9:2], {X[1:0],Y[1:0],Z[1:0],..}
        b = self._b.read_block(REG_XOUT0, 7)
        b0, b1, b2, b3, b4, b5, b6 = (int(x) for x in b)

        x = (b0 << 10) | (b1 << 2) | ((b6 >> 6) & 0x03)
        y = (b2 << 10) | (b3 << 2) | ((b6 >> 4) & 0x03)
        z = (b4 << 10) | (b5 << 2) | ((b6 >> 2) & 0x03)
        return x, y, z

    @staticmethod
    def _counts_to_uT(x: int, y: int, z: int) -> Tuple[float, float, float]:
        # Convert to signed gauss (approximately), then uT.
        gx = (float(x) - NULL_FIELD_COUNTS_18B) / COUNTS_PER_G_18B
        gy = (float(y) - NULL_FIELD_COUNTS_18B) / COUNTS_PER_G_18B
        gz = (float(z) - NULL_FIELD_COUNTS_18B) / COUNTS_PER_G_18B
        return gx * G_TO_uT, gy * G_TO_uT, gz * G_TO_uT

    def read_uT(self) -> MagReading:
        now = time.time()
        if not self.use_set_reset:
            x, y, z = self._read_raw_18b()
            xu, yu, zu = self._counts_to_uT(x, y, z)
            return MagReading(x_uT=xu, y_uT=yu, z_uT=zu, ts=now)

        # SET -> measure
        self._b.write_reg(REG_CTRL0, CTRL0_SET)
        time.sleep(0.001)
        x1, y1, z1 = self._read_raw_18b()

        # RESET -> measure
        self._b.write_reg(REG_CTRL0, CTRL0_RESET)
        time.sleep(0.001)
        x2, y2, z2 = self._read_raw_18b()

        # The datasheet's SET/RESET protocol can be used to cancel offset:
        # (Output_set - Output_reset) / 2 ~= H
        x = int(round((x1 - x2) / 2.0 + NULL_FIELD_COUNTS_18B))
        y = int(round((y1 - y2) / 2.0 + NULL_FIELD_COUNTS_18B))
        z = int(round((z1 - z2) / 2.0 + NULL_FIELD_COUNTS_18B))

        xu, yu, zu = self._counts_to_uT(x, y, z)
        return MagReading(x_uT=xu, y_uT=yu, z_uT=zu, ts=now)
