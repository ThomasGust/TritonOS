#!/usr/bin/env python3
"""
navigator_pca9685_basic_esc_test.py

Direct PCA9685 PWM test for Blue Robotics Navigator:
- Finds PCA9685 on an I2C bus (defaults to /dev/i2c-4, addr 0x40, with scan fallback)
- Sets PWM frequency (default 50 Hz)
- Drives all selected channels to 1500us (neutral) for arming/init
- Enables outputs via OE GPIO
- Cycles through a pulse sequence to verify the control chain
- Cleans up safely on Ctrl+C / exit (neutral + outputs disabled)

Dependencies:
  pip install smbus2
Optional (for OE):
  pip install RPi.GPIO
  (If RPi.GPIO isn't available, the script will try python gpiod)

Run:
  sudo python3 navigator_pca9685_basic_esc_test.py --channels 1,2,3 --sequence 1500,1525,1500,1475,1500
"""

import argparse
import glob
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from smbus2 import SMBus

# PCA9685 Registers
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA
ALL_LED_OFF_L = 0xFC

# MODE1 bits
RESTART = 0x80
SLEEP = 0x10
AI = 0x20  # auto-increment

# MODE2 bits
OUTDRV = 0x04  # totem pole


def _list_i2c_bus_numbers() -> List[int]:
    buses = []
    for dev in glob.glob("/dev/i2c-*"):
        try:
            buses.append(int(dev.split("-")[-1]))
        except ValueError:
            pass
    return sorted(set(buses))


def _i2c_probe_read(bus_num: int, addr: int, reg: int) -> Optional[int]:
    try:
        with SMBus(bus_num) as bus:
            return bus.read_byte_data(addr, reg)
    except Exception:
        return None


def find_pca9685(preferred_bus: Optional[int], preferred_addr: Optional[int]) -> Tuple[int, int]:
    """
    Try preferred bus/addr first, then scan.
    We scan common PCA9685 address range (0x40-0x7F) on available i2c busses.
    """
    bus_candidates = []
    if preferred_bus is not None:
        bus_candidates.append(preferred_bus)
    bus_candidates.extend([b for b in _list_i2c_bus_numbers() if b not in bus_candidates])

    addr_candidates = []
    if preferred_addr is not None:
        addr_candidates.append(preferred_addr)
    addr_candidates.extend([a for a in range(0x40, 0x80) if a not in addr_candidates])

    last_err = None
    for b in bus_candidates:
        if not os.path.exists(f"/dev/i2c-{b}"):
            continue
        for a in addr_candidates:
            try:
                v = _i2c_probe_read(b, a, MODE1)
                if v is None:
                    continue
                # Very light sanity check: MODE1 is an 8-bit register; any value is "possible".
                # Read MODE2 as well to reduce false positives.
                v2 = _i2c_probe_read(b, a, MODE2)
                if v2 is None:
                    continue
                return b, a
            except Exception as e:
                last_err = e
                continue

    raise RuntimeError(
        "Could not find a PCA9685 on any /dev/i2c-* bus. "
        "Check that I2C is enabled and the Navigator overlay is loaded."
        + (f" Last error: {last_err}" if last_err else "")
    )


class OEController:
    """
    Output Enable (OE) GPIO controller.

    PCA9685 OE is active-low (LOW enables outputs) by chip design.
    Depending on board wiring, you may need to invert; that's why we support active_low flag.
    """

    def __init__(self, bcm_gpio: int, active_low: bool = True):
        self.bcm_gpio = bcm_gpio
        self.active_low = active_low
        self._backend = None

        # Try RPi.GPIO first (common on Pi images)
        try:
            import RPi.GPIO as GPIO  # type: ignore
            self._backend = ("rpi_gpio", GPIO)
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.bcm_gpio, GPIO.OUT, initial=self._drive_level(False))
            return
        except Exception:
            pass

        # Fall back to gpiod (works on newer distros that drop sysfs/RPi.GPIO)
        try:
            import gpiod  # type: ignore
            self._backend = ("gpiod", gpiod)
            self._chip = gpiod.Chip("gpiochip0")
            self._line = self._chip.get_line(self.bcm_gpio)
            self._line.request(
                consumer="pca9685-oe",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[self._drive_level(False)],
            )
            return
        except Exception:
            self._backend = None

        raise RuntimeError(
            "Could not initialize OE GPIO control. Install one of:\n"
            "  - RPi.GPIO (pip install RPi.GPIO)\n"
            "  - gpiod python bindings (pip install gpiod) / apt install python3-libgpiod\n"
            "Or run with --no-oe if you want to skip OE control."
        )

    def _drive_level(self, enabled: bool) -> int:
        # enabled=True means "PWM outputs enabled"
        # If active_low: enable -> drive 0, disable -> drive 1
        if self.active_low:
            return 0 if enabled else 1
        else:
            return 1 if enabled else 0

    def set_enabled(self, enabled: bool) -> None:
        """Drive the OE line to enable or disable PWM outputs."""

        if self._backend is None:
            return
        kind, lib = self._backend
        level = self._drive_level(enabled)

        if kind == "rpi_gpio":
            lib.output(self.bcm_gpio, level)
        elif kind == "gpiod":
            self._line.set_value(level)

    def close(self) -> None:
        """Disable outputs and release GPIO resources."""

        if self._backend is None:
            return
        kind, lib = self._backend
        try:
            self.set_enabled(False)
        except Exception:
            pass
        if kind == "rpi_gpio":
            try:
                lib.cleanup(self.bcm_gpio)
            except Exception:
                pass
        elif kind == "gpiod":
            try:
                self._line.release()
            except Exception:
                pass
            try:
                self._chip.close()
            except Exception:
                pass


@dataclass
class PCA9685:
    """Direct-I2C PCA9685 helper for simple ESC pulse tests."""

    bus_num: int
    address: int
    freq_hz: float = 50.0
    osc_hz: float = 24_576_000.0  # typical PCA9685 oscillator

    def __post_init__(self):
        self.bus = SMBus(self.bus_num)

    def close(self):
        """Close the I2C bus handle."""

        try:
            self.bus.close()
        except Exception:
            pass

    def read8(self, reg: int) -> int:
        """Read one PCA9685 register."""

        return self.bus.read_byte_data(self.address, reg)

    def write8(self, reg: int, val: int) -> None:
        """Write one PCA9685 register."""

        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def set_pwm_freq(self, freq_hz: float) -> None:
        """Configure PCA9685 PWM frequency using the prescale register."""

        # prescale = round(osc / (4096*freq) - 1)
        prescaleval = (self.osc_hz / (4096.0 * float(freq_hz))) - 1.0
        prescale = int(round(prescaleval))
        if prescale < 3 or prescale > 255:
            raise ValueError(f"Computed prescale {prescale} out of range for freq {freq_hz} Hz")

        oldmode = self.read8(MODE1)
        sleepmode = (oldmode & 0x7F) | SLEEP  # sleep, clear restart
        self.write8(MODE1, sleepmode)
        time.sleep(0.005)

        self.write8(PRESCALE, prescale)

        # Wake up, enable auto-increment, then restart
        self.write8(MODE1, (oldmode & 0x7F) | AI)
        time.sleep(0.005)
        self.write8(MODE1, ((oldmode & 0x7F) | AI) | RESTART)

        self.freq_hz = float(freq_hz)

    def init(self) -> None:
        """Initialize output mode and PWM frequency."""

        # Basic init: OUTDRV totem pole, auto-increment enabled via set_pwm_freq
        self.write8(MODE2, OUTDRV)
        # MODE1 reset-ish (but don't hard reset all bits; set_pwm_freq will manage)
        self.write8(MODE1, AI)
        time.sleep(0.005)
        self.set_pwm_freq(self.freq_hz)

    def set_pwm_counts(self, channel: int, on: int, off: int) -> None:
        """Write raw ON/OFF counts for one zero-based channel."""

        if not (0 <= channel <= 15):
            raise ValueError("Channel must be 0..15")
        reg = LED0_ON_L + 4 * channel
        data = [on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF]
        # write block
        self.bus.write_i2c_block_data(self.address, reg, data)

    def set_all_pwm_counts(self, on: int, off: int) -> None:
        """Write raw ON/OFF counts to every PCA9685 channel."""

        data = [on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF]
        self.bus.write_i2c_block_data(self.address, ALL_LED_ON_L, data)

    def us_to_counts(self, pulse_us: float) -> int:
        """Convert pulse width in microseconds to a PCA9685 count."""

        period_us = 1_000_000.0 / self.freq_hz
        counts = int(round((pulse_us / period_us) * 4096.0))
        # clamp to valid 12-bit
        return max(0, min(4095, counts))

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        """Set a zero-based PCA9685 channel by pulse width."""

        off = self.us_to_counts(pulse_us)
        self.set_pwm_counts(channel, 0, off)

    def set_all_pulse_us(self, pulse_us: float) -> None:
        """Set every PCA9685 channel to the same pulse width."""

        off = self.us_to_counts(pulse_us)
        self.set_all_pwm_counts(0, off)


def parse_int_list(s: str) -> List[int]:
    """Parse comma-separated integer channel values."""

    out = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part))
    return out


def parse_float_list(s: str) -> List[float]:
    """Parse comma-separated pulse widths in microseconds."""

    out = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(float(part))
    return out


def main() -> int:
    """Run the basic PCA9685/ESC sequence with safe shutdown handling."""

    ap = argparse.ArgumentParser(description="Direct PCA9685 PWM test for Navigator + Basic ESCs")
    ap.add_argument("--bus", type=int, default=4, help="Preferred I2C bus number (default: 4)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x40, help="Preferred I2C address (default: 0x40)")
    ap.add_argument("--freq", type=float, default=50.0, help="PWM frequency in Hz (default: 50)")
    ap.add_argument("--osc-hz", type=float, default=25_000_000.0, help="PCA9685 oscillator Hz (default: 25e6)")

    ap.add_argument("--channels", type=str, default="1",
                    help="Navigator output channels 1-16 (comma-separated). Example: 1,2,3 (default: 1)")
    ap.add_argument("--sequence", type=str, default="1500,1525,1500,1475,1500",
                    help="Pulse sequence in microseconds (comma-separated). Default is gentle nudge around neutral.")
    ap.add_argument("--hold", type=float, default=1.0, help="Seconds to hold each sequence step (default: 1.0)")
    ap.add_argument("--arm-seconds", type=float, default=3.0,
                    help="Seconds to hold neutral (1500us) for ESC init/arm (default: 3.0)")
    ap.add_argument("--loops", type=int, default=2, help="How many times to repeat the sequence (default: 2)")

    ap.add_argument("--oe-gpio", type=int, default=26,
                    help="BCM GPIO used for PCA9685 OE control (default: 26). Override if needed.")
    ap.add_argument("--oe-active-low", action="store_true", default=True,
                    help="Treat OE as active-low (LOW enables outputs). Default: on.")
    ap.add_argument("--oe-active-high", action="store_true",
                    help="Treat OE as active-high (HIGH enables outputs).")
    ap.add_argument("--no-oe", action="store_true",
                    help="Skip OE control entirely (not recommended; outputs may be enabled immediately).")

    ap.add_argument("--scan-only", action="store_true", help="Only scan and print detected PCA9685 then exit.")
    args = ap.parse_args()

    if args.oe_active_high:
        oe_active_low = False
    else:
        oe_active_low = True  # default

    # Convert Navigator channel numbers 1-16 -> PCA9685 channels 0-15
    nav_channels = parse_int_list(args.channels)
    pca_channels = []
    for ch in nav_channels:
        if not (1 <= ch <= 16):
            raise SystemExit("Channels must be 1..16 (Navigator labeling).")
        pca_channels.append(ch - 1)

    seq = parse_float_list(args.sequence)
    if len(seq) == 0:
        raise SystemExit("Empty --sequence")

    # Find PCA9685 (try preferred, then scan)
    bus_num, addr = find_pca9685(args.bus, args.addr)
    print(f"[OK] Found PCA9685 at bus /dev/i2c-{bus_num}, address 0x{addr:02X}")

    if args.scan_only:
        return 0

    oe = None
    if not args.no_oe:
        oe = OEController(args.oe_gpio, active_low=oe_active_low)
        # Disable outputs first to avoid glitches during setup
        oe.set_enabled(False)
        print(f"[OK] OE GPIO BCM{args.oe_gpio} set to DISABLE outputs (polarity active_low={oe_active_low})")
    else:
        print("[WARN] --no-oe specified: skipping OE control.")

    pca = PCA9685(bus_num=bus_num, address=addr, freq_hz=args.freq, osc_hz=args.osc_hz)

    def safe_shutdown(*_):
        print("\n[SAFE] Shutting down: neutral + outputs disabled")
        try:
            # Drive neutral first
            for ch in pca_channels:
                pca.set_pulse_us(ch, 1500.0)
            time.sleep(0.2)
        except Exception:
            pass
        try:
            if oe:
                oe.set_enabled(False)
        except Exception:
            pass
        try:
            pca.close()
        except Exception:
            pass
        try:
            if oe:
                oe.close()
        except Exception:
            pass

    signal.signal(signal.SIGINT, safe_shutdown)
    signal.signal(signal.SIGTERM, safe_shutdown)

    try:
        pca.init()
        print(f"[OK] PCA9685 initialized at {args.freq:.2f} Hz")

        # Set neutral (1500us) before enabling OE
        for ch in pca_channels:
            pca.set_pulse_us(ch, 1500.0)
        print(f"[OK] Set channels {nav_channels} to 1500us (neutral)")

        # Enable outputs and hold neutral to initialize/arm ESCs
        if oe:
            oe.set_enabled(True)
            print("[OK] Outputs ENABLED via OE")
        print(f"[ARM] Holding 1500us for {args.arm_seconds:.1f}s to initialize/arm ESCs...")
        time.sleep(args.arm_seconds)

        # Cycle sequence
        print(f"[TEST] Running sequence {seq} us, hold={args.hold:.2f}s, loops={args.loops}")
        for i in range(args.loops):
            print(f"[TEST] Loop {i+1}/{args.loops}")
            for pulse in seq:
                for ch in pca_channels:
                    pca.set_pulse_us(ch, pulse)
                print(f"  -> {pulse:.1f} us")
                time.sleep(args.hold)

        # Return to neutral
        for ch in pca_channels:
            pca.set_pulse_us(ch, 1500.0)
        print("[DONE] Returned to 1500us neutral")
        time.sleep(0.5)

        # Disable outputs at end
        if oe:
            oe.set_enabled(False)
            print("[DONE] Outputs DISABLED via OE")

    finally:
        safe_shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
