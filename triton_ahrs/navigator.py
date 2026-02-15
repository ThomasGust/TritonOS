from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

from .sensors.icm20602_auto import ICM20602
from .sensors.ak09915 import AK09915

try:
    from .sensors.mmc5983 import MMC5983
except Exception:
    MMC5983 = None  # type: ignore


@dataclass
class Vec3:
    x: float
    y: float
    z: float


class NavigatorIMU:
    """Minimal sensor access for Blue Robotics Navigator (or compatible stack).

    - ICM20602: auto-detect SPI (/dev/spidev*) first, then I2C.
    - AK09915: I2C magnetometer (default bus 1).
    - Optional MMC5983: auto-detect I2C/SPI (if present and enabled).

    Units:
      - accel: m/s^2
      - gyro: rad/s
      - mag: microtesla
    """

    def __init__(
        self,
        *,
        imu_i2c_bus: int = 1,
        imu_spi_devices: Optional[Tuple[Tuple[int, int], ...]] = None,
        prefer_spi: bool = True,
        ak_i2c_bus: int = 1,
        enable_mmc5983: bool = True,
        mmc_i2c_buses: Tuple[int, ...] = (6, 1),
        mmc_spi_devices: Tuple[Tuple[int, int], ...] = ((0, 0), (0, 1), (1, 0), (1, 1)),
        mmc_use_set_reset: bool = True,
    ):
        self.imu = ICM20602.auto_detect(
            i2c_buses=(int(imu_i2c_bus),),
            spi_devices=list(imu_spi_devices) if imu_spi_devices else None,
            prefer_spi=bool(prefer_spi),
        )
        self.ak = AK09915(bus=int(ak_i2c_bus))

        self.mmc = None
        if enable_mmc5983 and MMC5983 is not None:
            try:
                self.mmc = MMC5983.auto_detect(
                    i2c_buses=mmc_i2c_buses,
                    spi_devices=mmc_spi_devices,
                    use_set_reset=bool(mmc_use_set_reset),
                )
            except Exception:
                self.mmc = None

    def close(self) -> None:
        try:
            self.imu.close()
        except Exception:
            pass
        try:
            self.ak.close()
        except Exception:
            pass
        try:
            if self.mmc is not None:
                self.mmc.close()
        except Exception:
            pass

    def read_accel(self) -> Vec3:
        a = self.imu.read_accel()
        return Vec3(a.x, a.y, a.z)

    def read_gyro(self) -> Vec3:
        g = self.imu.read_gyro()
        return Vec3(g.x, g.y, g.z)

    def read_mag(self) -> tuple[str, Vec3]:
        """Return (source, mag_uT). Prefers MMC5983 if detected."""
        if self.mmc is not None:
            r = self.mmc.read_uT()
            return ("mmc5983", Vec3(r.x_uT, r.y_uT, r.z_uT))
        m = self.ak.read_uT()
        return ("ak09915", Vec3(m.x, m.y, m.z))

    def read_all(self) -> dict:
        src, m = self.read_mag()
        a = self.read_accel()
        g = self.read_gyro()
        return {
            "ts": time.time(),
            "accel": {"x": a.x, "y": a.y, "z": a.z},
            "gyro": {"x": g.x, "y": g.y, "z": g.z},
            "mag": {"x": m.x, "y": m.y, "z": m.z, "source": src},
        }
