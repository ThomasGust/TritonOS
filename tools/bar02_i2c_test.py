#!/usr/bin/env python3
"""
Bar02 (MS5837-02BA) I2C test script for Raspberry Pi / Navigator setups.

- Scans an I2C bus and reports devices found
- Verifies an MS5837 (Bar02/Bar30) is present by initializing the sensor
- Prints live pressure + temperature (+ depth)

Refs:
- Blue Robotics ms5837-python usage/API: https://github.com/bluerobotics/ms5837-python
"""

import argparse
import datetime as dt
import sys
import time
import subprocess

from smbus2 import SMBus

BAR_SENSOR_ADDR = 0x76  # fixed for Bar02/Bar30 on the bus

def list_i2c_devices(bus_num: int) -> list[int]:
    """Return list of responding 7-bit addresses on the bus."""
    found = []
    with SMBus(bus_num) as bus:
        for addr in range(0x03, 0x78):
            try:
                # write_quick is a common low-impact probe; NACK -> OSError
                bus.write_quick(addr)
                found.append(addr)
            except OSError:
                pass
    return found

def run_i2cdetect(bus_num: int) -> str | None:
    """Return i2cdetect output if available, else None."""
    try:
        return subprocess.check_output(["i2cdetect", "-y", str(bus_num)], text=True)
    except FileNotFoundError:
        return None
    except subprocess.CalledProcessError as e:
        return e.output or str(e)

def main() -> int:
    ap = argparse.ArgumentParser(description="Bar02 (MS5837-02BA) I2C test")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    ap.add_argument("--interval", type=float, default=0.5, help="Seconds between prints (default: 0.5)")
    ap.add_argument("--once", action="store_true", help="Read once and exit")
    ap.add_argument("--density", type=float, default=997.0, help="Fluid density kg/m^3 (default freshwater ~997)")
    ap.add_argument(
        "--osr",
        default="OSR_8192",
        choices=["OSR_256", "OSR_512", "OSR_1024", "OSR_2048", "OSR_4096", "OSR_8192"],
        help="Oversampling (default: OSR_8192)",
    )
    ap.add_argument("--show-i2cdetect", action="store_true", help="Also print `i2cdetect` output")
    args = ap.parse_args()

    print(f"Scanning I2C bus /dev/i2c-{args.bus} ...")
    try:
        addrs = list_i2c_devices(args.bus)
    except PermissionError:
        print("❌ Permission denied opening I2C bus. Try: sudo usermod -aG i2c $USER && logout/login, or run with sudo.")
        return 1
    except FileNotFoundError:
        print(f"❌ /dev/i2c-{args.bus} not found. Check I2C is enabled and the bus number is correct.")
        return 1

    if addrs:
        print("Found device(s): " + ", ".join(f"0x{a:02X}" for a in addrs))
    else:
        print("Found device(s): (none)")

    if args.show_i2cdetect:
        out = run_i2cdetect(args.bus)
        if out is None:
            print("(i2cdetect not installed)")
        else:
            print("\n--- i2cdetect output ---")
            print(out.rstrip())
            print("--- end ---\n")

    if BAR_SENSOR_ADDR not in addrs:
        print(f"❌ No device responded at 0x{BAR_SENSOR_ADDR:02X}.")
        print("   - Bar02/Bar30 are fixed at 0x76 (cannot change).")
        print("   - Check wiring (SDA/SCL/3V3/GND), bus selection, and pull-ups.")
        return 2

    print(f"✅ Something responded at 0x{BAR_SENSOR_ADDR:02X}. Now verifying it is an MS5837 (Bar02/Bar30)...")

    try:
        import ms5837
    except ImportError:
        print("❌ Python module `ms5837` not found.")
        print("   Install with:")
        print("   python3 -m pip install git+https://github.com/bluerobotics/ms5837-python.git")
        return 3

    # Bar02 is MS5837-02BA
    sensor = ms5837.MS5837_02BA(args.bus)

    if not sensor.init():
        print("❌ MS5837 init() failed. A device at 0x76 exists, but it does not look like an MS5837.")
        print("   (Common cause: address conflict on that bus, or a different sensor at 0x76.)")
        return 4

    sensor.setFluidDensity(args.density)
    osr = getattr(ms5837, args.osr)

    print("✅ MS5837 initialized successfully (Bar sensor detected).")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            ok = sensor.read(osr)
            ts = dt.datetime.now().isoformat(timespec="seconds")

            if not ok:
                print(f"{ts}  ❌ read() failed")
            else:
                p_mbar = sensor.pressure()              # default mbar
                t_c = sensor.temperature()              # default °C
                depth_m = sensor.depth()                # meters (uses density)
                alt_m = sensor.altitude()               # meters (air model)

                print(
                    f"{ts}  P={p_mbar:8.2f} mbar  "
                    f"T={t_c:6.2f} °C  "
                    f"depth={depth_m:7.3f} m  "
                    f"alt={alt_m:7.2f} m"
                )

            if args.once:
                break
            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
