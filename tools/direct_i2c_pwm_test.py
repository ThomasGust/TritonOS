#!/usr/bin/env python3
"""
Direct I2C PCA9685 PWM test for TritonOS.

This script bypasses bluerobotics_navigator completely and talks straight to the
PCA9685 over I2C using smbus2. It is intended for bottom-side troubleshooting
when Navigator Python bindings are broken or unavailable.

Features:
- Detect / connect to PCA9685 directly on I2C (default bus 4, addr 0x40)
- Optional OE GPIO control (default BCM26)
- Hold 1500us neutral to arm ESCs
- Drive one or more channels directly by pulse width
- Optional pulse sequence / sweep modes
- Safe shutdown to neutral + outputs disabled

Examples:
  # Scan only
  sudo .venv/bin/python direct_i2c_pwm_test.py --scan-only

  # Arm channel 11, then hold neutral
  sudo .venv/bin/python direct_i2c_pwm_test.py --channels 11 --arm-seconds 3

  # Gentle nudge on channel 11
  sudo .venv/bin/python direct_i2c_pwm_test.py --channels 11 --sequence 1500,1525,1500,1475,1500

  # Test channels 11 and 12 together
  sudo .venv/bin/python direct_i2c_pwm_test.py --channels 11,12 --sequence 1500,1600,1500,1400,1500

  # Manual pulse
  sudo .venv/bin/python direct_i2c_pwm_test.py --channels 11 --pulse-us 1600 --hold 2

Dependencies:
  pip install smbus2
Optional for OE:
  pip install RPi.GPIO
  # or use gpiod python bindings
"""

from __future__ import annotations

import argparse
import glob
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from smbus2 import SMBus

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA
AI = 0x20
SLEEP = 0x10
RESTART = 0x80
OUTDRV = 0x04


def list_i2c_buses() -> List[int]:
    """Return visible Linux I2C bus numbers from ``/dev/i2c-*``."""

    buses: List[int] = []
    for dev in glob.glob('/dev/i2c-*'):
        try:
            buses.append(int(dev.rsplit('-', 1)[-1]))
        except ValueError:
            pass
    return sorted(set(buses))


def probe_reg(bus_num: int, addr: int, reg: int) -> Optional[int]:
    """Read one register from an I2C device, returning None on failure."""

    try:
        with SMBus(bus_num) as bus:
            return bus.read_byte_data(addr, reg)
    except Exception:
        return None


def find_pca9685(preferred_bus: Optional[int], preferred_addr: Optional[int]) -> Tuple[int, int]:
    """Locate a PCA9685 by trying preferred values, then scanning candidates."""

    bus_candidates: List[int] = []
    if preferred_bus is not None:
        bus_candidates.append(preferred_bus)
    bus_candidates.extend([b for b in list_i2c_buses() if b not in bus_candidates])

    addr_candidates: List[int] = []
    if preferred_addr is not None:
        addr_candidates.append(preferred_addr)
    addr_candidates.extend([a for a in range(0x40, 0x80) if a not in addr_candidates])

    for bus_num in bus_candidates:
        if not os.path.exists(f'/dev/i2c-{bus_num}'):
            continue
        for addr in addr_candidates:
            v1 = probe_reg(bus_num, addr, MODE1)
            if v1 is None:
                continue
            v2 = probe_reg(bus_num, addr, MODE2)
            if v2 is None:
                continue
            return bus_num, addr
    raise RuntimeError('No PCA9685 found on any visible I2C bus.')


class OEController:
    """Controls PCA9685 OE pin. OE is active-low on the chip."""

    def __init__(self, bcm_gpio: int, active_low: bool = True):
        self.bcm_gpio = int(bcm_gpio)
        self.active_low = bool(active_low)
        self._backend = None

        try:
            import RPi.GPIO as GPIO  # type: ignore
            self._backend = ('rpi_gpio', GPIO)
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.bcm_gpio, GPIO.OUT, initial=self._drive_level(False))
            return
        except Exception:
            pass

        try:
            import gpiod  # type: ignore
            self._chip = gpiod.Chip('gpiochip0')
            self._line = self._chip.get_line(self.bcm_gpio)
            self._line.request(
                consumer='direct-i2c-pwm-test',
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[self._drive_level(False)],
            )
            self._backend = ('gpiod', gpiod)
            return
        except Exception:
            pass

        raise RuntimeError(
            'Could not initialize OE GPIO. Install RPi.GPIO or gpiod, or use --no-oe.'
        )

    def _drive_level(self, enabled: bool) -> int:
        if self.active_low:
            return 0 if enabled else 1
        return 1 if enabled else 0

    def set_enabled(self, enabled: bool) -> None:
        """Drive the OE line to enable or disable PWM outputs."""

        if self._backend is None:
            return
        kind, lib = self._backend
        level = self._drive_level(bool(enabled))
        if kind == 'rpi_gpio':
            lib.output(self.bcm_gpio, level)
        elif kind == 'gpiod':
            self._line.set_value(level)

    def close(self) -> None:
        """Disable outputs and release GPIO resources."""

        try:
            self.set_enabled(False)
        except Exception:
            pass
        if self._backend is None:
            return
        kind, lib = self._backend
        if kind == 'rpi_gpio':
            try:
                lib.cleanup(self.bcm_gpio)
            except Exception:
                pass
        elif kind == 'gpiod':
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
    """Small direct-I2C PCA9685 driver used by hardware diagnostic scripts."""

    bus_num: int
    address: int
    freq_hz: float = 50.0
    osc_hz: float = 25_000_000.0

    def __post_init__(self) -> None:
        self.bus = SMBus(self.bus_num)

    def close(self) -> None:
        """Close the I2C bus handle."""

        try:
            self.bus.close()
        except Exception:
            pass

    def read8(self, reg: int) -> int:
        """Read one PCA9685 register."""

        return self.bus.read_byte_data(self.address, reg)

    def write8(self, reg: int, value: int) -> None:
        """Write one PCA9685 register."""

        self.bus.write_byte_data(self.address, reg, int(value) & 0xFF)

    def set_pwm_freq(self, freq_hz: float) -> None:
        """Configure PCA9685 PWM frequency using the prescale register."""

        prescaleval = (self.osc_hz / (4096.0 * float(freq_hz))) - 1.0
        prescale = int(round(prescaleval))
        if not (3 <= prescale <= 255):
            raise ValueError(f'Computed prescale {prescale} out of range for {freq_hz} Hz')

        oldmode = self.read8(MODE1)
        self.write8(MODE1, (oldmode & 0x7F) | SLEEP)
        time.sleep(0.005)
        self.write8(PRESCALE, prescale)
        self.write8(MODE1, (oldmode & 0x7F) | AI)
        time.sleep(0.005)
        self.write8(MODE1, ((oldmode & 0x7F) | AI) | RESTART)
        self.freq_hz = float(freq_hz)

    def init(self) -> None:
        """Initialize output mode and PWM frequency."""

        self.write8(MODE2, OUTDRV)
        self.write8(MODE1, AI)
        time.sleep(0.005)
        self.set_pwm_freq(self.freq_hz)

    def set_pwm_counts(self, channel_zero_based: int, on: int, off: int) -> None:
        """Write raw ON/OFF counts for one zero-based PCA9685 channel."""

        if not (0 <= channel_zero_based <= 15):
            raise ValueError('PCA9685 channel must be 0..15')
        reg = LED0_ON_L + 4 * int(channel_zero_based)
        data = [on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF]
        self.bus.write_i2c_block_data(self.address, reg, data)

    def us_to_counts(self, pulse_us: float) -> int:
        """Convert pulse width in microseconds to a PCA9685 count."""

        period_us = 1_000_000.0 / float(self.freq_hz)
        counts = int(round((float(pulse_us) / period_us) * 4096.0))
        return max(0, min(4095, counts))

    def set_pulse_us_nav_channel(self, nav_channel_one_based: int, pulse_us: float) -> None:
        """Set a Navigator-labeled 1..16 channel by pulse width."""

        if not (1 <= nav_channel_one_based <= 16):
            raise ValueError('Navigator/PCA9685 channel must be 1..16')
        self.set_pwm_counts(nav_channel_one_based - 1, 0, self.us_to_counts(pulse_us))


def parse_channel_list(s: str) -> List[int]:
    """Parse comma-separated Navigator channel numbers."""

    out: List[int] = []
    for part in s.split(','):
        part = part.strip()
        if not part:
            continue
        out.append(int(part))
    return out


def parse_float_list(s: str) -> List[float]:
    """Parse comma-separated floating-point pulse widths."""

    out: List[float] = []
    for part in s.split(','):
        part = part.strip()
        if not part:
            continue
        out.append(float(part))
    return out


def set_channels(pca: PCA9685, channels: List[int], pulse_us: float) -> None:
    """Apply the same pulse width to several Navigator-labeled channels."""

    for ch in channels:
        pca.set_pulse_us_nav_channel(ch, pulse_us)


def main() -> int:
    """Run the direct-I2C PWM diagnostic and always return channels neutral."""

    ap = argparse.ArgumentParser(description='Direct I2C PCA9685 PWM test (no Navigator Python module).')
    ap.add_argument('--bus', type=int, default=4, help='Preferred I2C bus number. Default: 4')
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x40, help='Preferred I2C address. Default: 0x40')
    ap.add_argument('--freq', type=float, default=50.0, help='PWM frequency in Hz. Default: 50')
    ap.add_argument('--osc-hz', type=float, default=25_000_000.0, help='PCA9685 oscillator Hz. Default: 25e6')
    ap.add_argument('--channels', type=str, default='11', help='Navigator-labeled channels 1..16, comma-separated. Default: 11')
    ap.add_argument('--arm-seconds', type=float, default=3.0, help='How long to hold 1500us neutral before test. Default: 3.0')
    ap.add_argument('--hold', type=float, default=1.0, help='How long to hold each pulse in sequence/manual mode. Default: 1.0')
    ap.add_argument('--sequence', type=str, default='1500,1525,1500,1475,1500', help='Pulse sequence in us. Default: gentle nudge sequence')
    ap.add_argument('--loops', type=int, default=1, help='Repeat sequence this many times. Default: 1')
    ap.add_argument('--pulse-us', type=float, default=None, help='Single pulse width to apply after arming')
    ap.add_argument('--scan-only', action='store_true', help='Only locate the PCA9685 and exit')
    ap.add_argument('--oe-gpio', type=int, default=26, help='BCM GPIO used for OE. Default: 26')
    ap.add_argument('--oe-active-high', action='store_true', help='Treat OE as active-high instead of active-low')
    ap.add_argument('--no-oe', action='store_true', help='Skip OE control entirely')
    args = ap.parse_args()

    channels = parse_channel_list(args.channels)
    if not channels:
        raise SystemExit('No channels specified')
    for ch in channels:
        if not (1 <= ch <= 16):
            raise SystemExit(f'Channel {ch} out of range; expected 1..16')

    bus_num, addr = find_pca9685(args.bus, args.addr)
    print(f'[OK] Found PCA9685 at /dev/i2c-{bus_num} addr 0x{addr:02X}')
    if args.scan_only:
        return 0

    oe: Optional[OEController] = None
    if not args.no_oe:
        oe = OEController(args.oe_gpio, active_low=not args.oe_active_high)
        oe.set_enabled(False)
        print(f'[OK] OE prepared on BCM{args.oe_gpio} (enabled=False)')
    else:
        print('[WARN] --no-oe: skipping OE control')

    pca = PCA9685(bus_num=bus_num, address=addr, freq_hz=args.freq, osc_hz=args.osc_hz)

    def safe_shutdown(*_args) -> None:
        print('\n[SAFE] Returning channels to 1500us and disabling outputs')
        try:
            set_channels(pca, channels, 1500.0)
            time.sleep(0.2)
        except Exception:
            pass
        try:
            if oe is not None:
                oe.set_enabled(False)
        except Exception:
            pass
        try:
            pca.close()
        except Exception:
            pass
        try:
            if oe is not None:
                oe.close()
        except Exception:
            pass

    signal.signal(signal.SIGINT, safe_shutdown)
    signal.signal(signal.SIGTERM, safe_shutdown)

    try:
        pca.init()
        print(f'[OK] PCA9685 initialized at {args.freq:.2f} Hz')

        set_channels(pca, channels, 1500.0)
        print(f'[OK] Set channels {channels} to 1500us neutral')

        if oe is not None:
            oe.set_enabled(True)
            print('[OK] Outputs enabled via OE')

        print(f'[ARM] Holding neutral for {args.arm_seconds:.1f}s')
        time.sleep(args.arm_seconds)

        if args.pulse_us is not None:
            pulse = float(args.pulse_us)
            print(f'[TEST] Applying {pulse:.1f}us on channels {channels} for {args.hold:.1f}s')
            set_channels(pca, channels, pulse)
            time.sleep(args.hold)
        else:
            seq = parse_float_list(args.sequence)
            print(f'[TEST] Running sequence {seq} on channels {channels}, hold={args.hold:.2f}s, loops={args.loops}')
            for i in range(max(1, int(args.loops))):
                print(f'  loop {i+1}/{max(1, int(args.loops))}')
                for pulse in seq:
                    set_channels(pca, channels, pulse)
                    print(f'    -> {pulse:.1f}us')
                    time.sleep(args.hold)

        set_channels(pca, channels, 1500.0)
        print('[DONE] Returned to 1500us neutral')
        time.sleep(0.3)
        if oe is not None:
            oe.set_enabled(False)
            print('[DONE] Outputs disabled via OE')
        return 0
    finally:
        safe_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
