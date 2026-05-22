"""Navigator sensor access.

Why this exists:
  The official `bluerobotics_navigator` python bindings are convenient, but on
  some systems they can claim the Navigator's PWM_OE line / PCA9685 resources.
  If you control thrusters via your own PCA9685 + gpiod code, that conflict
  manifests as EBUSY / panics and/or "arming does nothing".

So this module defaults to *direct I2C* access for the IMU and AK09915
magnetometer, plus an optional MMC5983 driver (also direct).

If you *really* want to use the official bindings, set:
    TRITON_USE_NAV_BINDINGS=1
in the environment.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

from sensors.base import BaseSensor
import sensors.ms5837 as ms5837  # for Bar30
from utils.navigator_import import import_navigator_module

# Optional: second magnetometer on Navigator
try:
    from sensors.mmc5983 import MMC5983
except Exception:  # pragma: no cover
    MMC5983 = None


@dataclass
class Vec3:
    """Simple 3-axis vector used to normalize different hardware APIs."""

    x: float
    y: float
    z: float


def _as_vec3(v) -> Vec3:
    """Best-effort normalize different driver return types into Vec3."""
    if isinstance(v, Vec3):
        return v
    if hasattr(v, "x") and hasattr(v, "y") and hasattr(v, "z"):
        return Vec3(float(v.x), float(v.y), float(v.z))
    if isinstance(v, dict):
        return Vec3(float(v["x"]), float(v["y"]), float(v["z"]))
    raise TypeError(f"Can't convert {type(v)} to Vec3")


class NavigatorBoard:
    """Unified access to Navigator sensors."""

    def __init__(self):
        self.started = time.time()

        # ---- config (optional) ----
        imu_bus = 1
        mag_bus = 1
        try:
            import rov_config as cfg  # type: ignore
            imu_bus = int(getattr(cfg, "NAV_IMU_I2C_BUS", imu_bus))
            mag_bus = int(getattr(cfg, "NAV_MAG_I2C_BUS", mag_bus))
        except Exception:
            pass

        # ---- decide backend ----
        # Default: direct access. The official bindings are opt-in only.
        self._use_bindings = os.getenv("TRITON_USE_NAV_BINDINGS", "0").strip() == "1"
        self._mmc5983 = None

        # Extra Navigator peripherals (direct access)
        self._baro = None
        self._adc = None
        self._leak = None

        if self._use_bindings:
            # Use official bindings (may conflict with PWM_OE; opt-in only)
            navigator = import_navigator_module()

            # Some versions don't expose init(); be permissive.
            if hasattr(navigator, "init"):
                navigator.init()  # type: ignore[attr-defined]
            self._nav = navigator
            self._imu = None
            self._ak = None
        else:
            # Direct access (recommended). On a stock Navigator the IMU is SPI.
            self._nav = None
            from sensors.icm20602_auto import ICM20602
            from sensors.ak09915 import AK09915
            from sensors.bmp280 import BMP280
            from sensors.ads1115 import ADS1115

            # IMU: try SPI first, then I2C as fallback.
            spi_devs = None
            i2c_buses = (imu_bus,)
            env_bus = 1
            adc_bus = 1
            leak_chip = "/dev/gpiochip0"
            leak_line = None
            leak_invert = False
            try:
                import rov_config as cfg  # type: ignore
                spi_devs = getattr(cfg, "NAV_IMU_SPI_DEVICES", None)
                env_bus = int(getattr(cfg, "NAV_ENV_I2C_BUS", env_bus))
                adc_bus = int(getattr(cfg, "NAV_ADC_I2C_BUS", adc_bus))
                leak_chip = str(getattr(cfg, "LEAK_GPIO_CHIP", leak_chip))
                leak_line = getattr(cfg, "LEAK_GPIO_LINE", leak_line)
                leak_invert = bool(getattr(cfg, "LEAK_GPIO_INVERT", leak_invert))
            except Exception:
                pass

            self._imu = ICM20602.auto_detect(i2c_buses=i2c_buses, spi_devices=spi_devs, prefer_spi=True)
            self._ak = AK09915(bus=mag_bus)

            # Barometer + ADC are optional but expected on Navigator.
            try:
                self._baro = BMP280(bus=env_bus)
            except Exception as e:
                print("[navigator] BMP280 init failed:", e)

            try:
                self._adc = ADS1115(bus=adc_bus)
            except Exception as e:
                print("[navigator] ADS1115 init failed:", e)

            # Leak input is hardware-dependent. If not configured, treat as unavailable.
            if leak_line is not None:
                try:
                    from sensors.leak_gpio import LeakGPIO

                    self._leak = LeakGPIO(chip=leak_chip, line=int(leak_line), invert=leak_invert)
                except Exception as e:
                    print("[navigator] LeakGPIO init failed:", e)

        # ---- optional MMC5983 ----
        enable = True
        use_set_reset = True
        i2c_buses = (6, 1)
        spi_devices = ((0, 0), (0, 1), (1, 0), (1, 1))
        try:
            import rov_config as cfg  # type: ignore
            enable = bool(getattr(cfg, "MMC5983_ENABLE", enable))
            use_set_reset = bool(getattr(cfg, "MMC5983_USE_SET_RESET", use_set_reset))
            i2c_buses = tuple(getattr(cfg, "MMC5983_I2C_BUSES", i2c_buses))
            spi_devices = tuple(getattr(cfg, "MMC5983_SPI_DEVICES", spi_devices))
        except Exception:
            pass

        if enable and MMC5983 is not None:
            try:
                self._mmc5983 = MMC5983.auto_detect(
                    i2c_buses=i2c_buses,
                    spi_devices=spi_devices,
                    use_set_reset=use_set_reset,
                )
                if self._mmc5983 is not None:
                    print("[navigator] MMC5983 magnetometer detected")
                else:
                    print("[navigator] MMC5983 magnetometer not detected (continuing with AK09915 only)")
            except Exception as e:
                print("[navigator] MMC5983 init failed (continuing with AK09915 only):", e)

    # ---- IMU ----
    def read_accel(self) -> Vec3:
        """Read accelerometer values from the active backend."""

        if self._use_bindings:
            a = self._nav.read_accel()  # type: ignore[union-attr]
            return _as_vec3(a)
        return _as_vec3(self._imu.read_accel())  # type: ignore[union-attr]

    def read_gyro(self) -> Vec3:
        """Read gyroscope values from the active backend."""

        if self._use_bindings:
            g = self._nav.read_gyro()  # type: ignore[union-attr]
            return _as_vec3(g)
        return _as_vec3(self._imu.read_gyro())  # type: ignore[union-attr]

    def read_imu(self):
        """Read accel + gyro from a single burst (same sample instant).

        Returns (Vec3_accel, Vec3_gyro).  Falls back to two separate reads
        when the bindings backend does not support a combined read.
        """
        if self._use_bindings:
            return (self.read_accel(), self.read_gyro())
        a, g = self._imu.read_all()  # type: ignore[union-attr]
        return (_as_vec3(a), _as_vec3(g))

    def read_temp(self) -> float:
        """Read IMU temperature from the active backend."""

        if self._use_bindings:
            return float(self._nav.read_temp())  # type: ignore[union-attr]
        return float(self._imu.read_temp_c())  # type: ignore[union-attr]

    # ---- magnetometers ----
    def read_mag_ak09915(self) -> Vec3:
        """Read the primary AK09915 magnetometer."""

        if self._use_bindings:
            m = self._nav.read_mag()  # type: ignore[union-attr]
            return _as_vec3(m)
        return _as_vec3(self._ak.read_uT())  # type: ignore[union-attr]

    def read_mag_mmc5983(self):
        """Read optional MMC5983 magnetometer data, if present."""

        if self._mmc5983 is None:
            return None
        r = self._mmc5983.read_uT()
        return {"x": r.x_uT, "y": r.y_uT, "z": r.z_uT, "ts": r.ts}

    def read_mags(self) -> Dict[str, Any]:
        """Return all available magnetometer sources in a stable dictionary."""

        ak = self.read_mag_ak09915()
        return {
            "ak09915": {"x": ak.x, "y": ak.y, "z": ak.z, "ts": time.time()},
            "mmc5983": self.read_mag_mmc5983(),
        }

    # Back-compat: existing code expects read_mag() -> AK09915
    def read_mag(self) -> Vec3:
        """Back-compat alias returning the AK09915 magnetometer vector."""

        return self.read_mag_ak09915()

    # ---- other Navigator features ----
    def read_pressure(self) -> float:
        """Return barometric pressure in kPa (for UI compatibility)."""
        if self._use_bindings:
            return float(self._nav.read_pressure())  # type: ignore[union-attr]
        if self._baro is None:
            raise RuntimeError("BMP280 not available")
        r = self._baro.read()
        return float(r.pressure_pa) / 1000.0

    def read_adc(self):
        """Return ADC readings as volts for channels 0..3."""
        if self._use_bindings:
            return self._nav.read_adc()  # type: ignore[union-attr]
        if self._adc is None:
            raise RuntimeError("ADS1115 not available")
        r = self._adc.read_all()
        return list(r.volts)

    def read_leak(self) -> bool:
        """Leak status.

        Navigator leak input routing varies by firmware/wiring. If not
        configured, we default to False (no leak) instead of raising.
        """
        if self._use_bindings:
            return bool(self._nav.read_leak())  # type: ignore[union-attr]
        if self._leak is None:
            return False
        return bool(self._leak.read())


# -----------------------------------------------------------------------------
# Sensors that can be polled by SensorPublisherService
# -----------------------------------------------------------------------------


class IMUSensor(BaseSensor):
    """Polled sensor wrapper that emits combined accelerometer/gyro telemetry."""

    def __init__(self, board: NavigatorBoard, rate_hz: float = 20.0, include_mag: bool = False):
        super().__init__("imu", rate_hz)
        self.board = board
        self.include_mag = bool(include_mag)

    def read(self) -> Dict[str, Any]:
        """Return one IMU telemetry message."""

        a, g = self.board.read_imu()
        out: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": self.name,
            "type": "imu",
            "accel": {"x": a.x, "y": a.y, "z": a.z},
            "gyro": {"x": g.x, "y": g.y, "z": g.z},
        }
        if self.include_mag:
            mags = self.board.read_mags()
            ak = mags.get("ak09915") or {"x": 0.0, "y": 0.0, "z": 0.0}
            out.update(
                {
                    "mag": {"x": float(ak["x"]), "y": float(ak["y"]), "z": float(ak["z"])},
                    "mag_source": "ak09915",
                    "mag_sources": mags,
                }
            )
        return out


class MagSensor(BaseSensor):
    """Polled sensor wrapper that emits normalized magnetometer telemetry."""

    def __init__(self, board: NavigatorBoard, rate_hz: float = 5.0):
        super().__init__("mag", rate_hz)
        self.board = board

    def read(self) -> Dict[str, Any]:
        """Return one magnetometer telemetry message."""

        mags = self.board.read_mags()
        ak = mags.get("ak09915") or {"x": 0.0, "y": 0.0, "z": 0.0}
        return {
            "ts": time.time(),
            "sensor": self.name,
            "type": "mag",
            "mag": {"x": float(ak["x"]), "y": float(ak["y"]), "z": float(ak["z"])},
            "mag_source": "ak09915",
            "mag_sources": mags,
        }


class EnvSensor(BaseSensor):
    """Polled sensor wrapper for Navigator temperature and barometric pressure."""

    def __init__(self, board: NavigatorBoard, rate_hz: float = 2.0):
        super().__init__("env", rate_hz)
        self.board = board

    def read(self) -> Dict[str, Any]:
        """Return one environment telemetry message."""

        out: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": self.name,
            "type": "env",
        }
        try:
            out["temperature_c"] = float(self.board.read_temp())
        except Exception as e:
            out["temperature_error"] = str(e)
        try:
            out["pressure_kpa"] = float(self.board.read_pressure())
        except Exception as e:
            out["pressure_error"] = str(e)
        return out


class LeakSensor(BaseSensor):
    """Polled sensor wrapper for the configured leak detector input."""

    def __init__(self, board: NavigatorBoard, rate_hz: float = 2.0):
        super().__init__("leak", rate_hz)
        self.board = board

    def read(self) -> Dict[str, Any]:
        """Return one leak-state telemetry message."""

        out: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": self.name,
            "type": "leak",
        }
        try:
            out["leak"] = bool(self.board.read_leak())
        except Exception as e:
            out["error"] = str(e)
        return out


class ADCSensor(BaseSensor):
    """Polled sensor wrapper for raw Navigator ADC channel voltages."""

    def __init__(self, board: NavigatorBoard, rate_hz: float = 5.0):
        super().__init__("adc", rate_hz)
        self.board = board

    def read(self) -> Dict[str, Any]:
        """Return one raw ADC telemetry message."""

        out: Dict[str, Any] = {
            "ts": time.time(),
            "sensor": self.name,
            "type": "adc",
        }
        try:
            raw = self.board.read_adc()
            # Navigator bindings return a struct-like object; normalize to list.
            if hasattr(raw, "channel"):
                out["channels"] = list(raw.channel)
            elif isinstance(raw, (list, tuple)):
                out["channels"] = list(raw)
            else:
                out["channels"] = raw
        except Exception as e:
            out["error"] = str(e)
        return out



# -----------------------------------------------------------------------------
# External depth / pressure sensor (BlueRobotics MS5837)
# -----------------------------------------------------------------------------

def _ms5837_make(bus: int, model: str | None):
    """Create an MS5837 instance.

    model:
      - "auto" (default): read PROM and auto-detect 30BA vs 02BA
      - "30BA" / "bar30"
      - "02BA" / "bar02"
    """
    m = (model or "auto").strip().lower()
    if m in ("auto", "detect", "unknown", ""):
        return ms5837.MS5837(model=ms5837.MODEL_UNKNOWN, bus=bus)
    if m in ("30ba", "bar30", "30"):
        return ms5837.MS5837_30BA(bus=bus)
    if m in ("02ba", "bar02", "02", "2"):
        return ms5837.MS5837_02BA(bus=bus)
    raise ValueError(f"Unsupported MS5837 model {model!r} (use 'auto', '30BA', or '02BA')")


class MS5837Sensor(BaseSensor):
    """External MS5837 pressure/depth sensor (Blue Robotics Bar30 / Bar02).

    Publishes telemetry under type='external_depth' with:
      - depth_m: depth relative to a surface reference measured at startup
      - pressure_mbar: absolute pressure in mbar
      - temperature_c: sensor temperature in °C
      - surface_pressure_mbar: reference surface pressure used for depth
      - model: detected or configured model string
      - i2c_bus: which I2C bus the sensor was found on
    """

    def __init__(
        self,
        rate_hz: float = 5.0,
        bus: int | Sequence[int] = 6,
        model: str | None = None,
        fluid_density: float = ms5837.DENSITY_SALTWATER,
        osr: int = ms5837.OSR_8192,
        surface_cal_samples: int = 15,
        surface_cal_delay_s: float = 0.02,
        surface_pressure_mbar: float | None = None,
        depth_offset_m: float = 0.0,
        name: str | None = "auto",
    ):
        # BaseSensor name is used as the row key topside. When name == "auto",
        # we will pick bar02/bar30 based on detected model.
        super().__init__(str(name or "ms5837"), rate_hz)

        # Normalize bus(es)
        if isinstance(bus, (list, tuple)):
            self._buses = [int(b) for b in bus]
        else:
            self._buses = [int(bus)]

        self._model_cfg = (model or "auto")
        self._fluid_density = float(fluid_density)
        self._osr = int(osr)
        self._surface_cal_samples = int(max(0, surface_cal_samples))
        self._surface_cal_delay_s = float(max(0.0, surface_cal_delay_s))
        self._depth_offset_m = float(depth_offset_m)

        self._bus_used: int | None = None
        self.sensor = None  # type: ignore[assignment]
        self._read_lock = threading.Lock()

        last_err: str | None = None
        for b in self._buses:
            try:
                s = _ms5837_make(bus=b, model=self._model_cfg)
                ok = bool(s.init())
            except Exception as e:
                ok = False
                last_err = str(e)
                s = None  # type: ignore
            if ok and s is not None:
                self.sensor = s
                self._bus_used = int(b)
                break

        if self.sensor is None or self._bus_used is None:
            raise RuntimeError(
                f"MS5837 could not be initialized on I2C bus(es) {self._buses}. "
                f"{'Last error: ' + last_err if last_err else ''}"
            )

        # Set density first (used for depth conversion).
        try:
            self.sensor.setFluidDensity(self._fluid_density)
        except Exception:
            pass

        # Determine detected model (when using auto / unknown)
        self._model_detected = None
        try:
            if getattr(self.sensor, "_model", None) == ms5837.MODEL_30BA:
                self._model_detected = "30BA"
            elif getattr(self.sensor, "_model", None) == ms5837.MODEL_02BA:
                self._model_detected = "02BA"
        except Exception:
            self._model_detected = None

        # If name is "auto"/None, pick a human-friendly name based on model.
        if name is None or (isinstance(name, str) and name.strip().lower() in ("auto", "detect", "ms5837", "")):
            if self._model_detected == "02BA":
                self.name = "bar02"
            elif self._model_detected == "30BA":
                self.name = "bar30"
            else:
                self.name = "ms5837"

        # Prefer a persisted/reference surface pressure if one is configured.
        self._p0_mbar = None
        if surface_pressure_mbar is not None:
            try:
                p0 = float(surface_pressure_mbar)
            except Exception:
                p0 = None  # type: ignore[assignment]
            if p0 is not None and p0 > 0.0:
                self._p0_mbar = float(p0)

        # Otherwise calibrate surface reference pressure at startup.
        if self._p0_mbar is None:
            self._calibrate_surface_pressure()

    def _calibrate_surface_pressure(self):
        if self._surface_cal_samples <= 0:
            return

        ps = []
        for _ in range(self._surface_cal_samples):
            try:
                if self.sensor.read(self._osr):  # type: ignore[union-attr]
                    ps.append(float(self.sensor.pressure()))  # type: ignore[union-attr]
            except Exception:
                pass
            if self._surface_cal_delay_s > 0:
                time.sleep(self._surface_cal_delay_s)

        if ps:
            self._p0_mbar = float(sum(ps) / len(ps))

    def _depth_from_pressure(self, p_mbar: float) -> float:
        # depth = (p - p0) / (rho * g)
        # pressure: mbar -> Pa
        p0 = float(self._p0_mbar) if self._p0_mbar is not None else 1013.0
        dp_pa = (float(p_mbar) - p0) * 100.0
        return dp_pa / (self._fluid_density * 9.80665)

    def read(self) -> Dict[str, Any]:
        """Read the pressure sensor and publish normalized depth telemetry."""

        with self._read_lock:
            try:
                ok = bool(self.sensor.read(self._osr))  # type: ignore[union-attr]
            except Exception as e:
                ok = False
                err = str(e)
            else:
                err = None

            if not ok:
                out = {
                    "ts": time.time(),
                    "sensor": self.name,
                    "type": "external_depth",
                    "error": err or "ms5837 read failed",
                }
                if self._p0_mbar is not None:
                    out["surface_pressure_mbar"] = float(self._p0_mbar)
                if self._bus_used is not None:
                    out["i2c_bus"] = int(self._bus_used)
                if self._model_detected is not None:
                    out["model"] = self._model_detected
                else:
                    out["model"] = str(self._model_cfg)
                return out

            p_mbar = float(self.sensor.pressure())  # type: ignore[union-attr]
            t_c = float(self.sensor.temperature())  # type: ignore[union-attr]
            depth_sensor_m = float(self._depth_from_pressure(p_mbar))
            depth_m = float(depth_sensor_m - float(self._depth_offset_m))

            out = {
                "ts": time.time(),
                "sensor": self.name,
                "type": "external_depth",
                "depth_m": depth_m,
                "depth_sensor_m": depth_sensor_m,
                "depth_offset_m": float(self._depth_offset_m),
                "temperature_c": t_c,
                "pressure_mbar": p_mbar,
                "fluid_density": float(self._fluid_density),
                "model": self._model_detected or str(self._model_cfg),
            }
            if self._p0_mbar is not None:
                out["surface_pressure_mbar"] = float(self._p0_mbar)
            if self._bus_used is not None:
                out["i2c_bus"] = int(self._bus_used)
            return out


class ExternalDepthSensor(MS5837Sensor):
    """Preferred name: publishes as bar02/bar30 automatically when model is detected."""
    def __init__(self, *args, **kwargs):
        # Force auto naming unless explicitly overridden
        if "name" not in kwargs:
            kwargs["name"] = "auto"
        super().__init__(*args, **kwargs)


class Bar02Sensor(MS5837Sensor):
    """Back-compat alias: publishes under sensor name 'bar02' and defaults to 02BA."""
    def __init__(self, rate_hz: float = 5.0, bus: int | Sequence[int] = 6, **kwargs):
        if "model" not in kwargs:
            kwargs["model"] = "02BA"
        super().__init__(rate_hz=rate_hz, bus=bus, name="bar02", **kwargs)

class Bar30Sensor(MS5837Sensor):
    """Back-compat alias: publishes under sensor name 'bar30'."""

    def __init__(self, rate_hz: float = 5.0, bus: int | Sequence[int] = 6, **kwargs):
        super().__init__(rate_hz=rate_hz, bus=bus, name="bar30", **kwargs)
