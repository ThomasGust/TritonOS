#!/usr/bin/env python3
"""Small PCA9685 / PWM_OE diagnostic for the Navigator.

This does NOT spin thrusters by default; it just:
- opens the PCA9685
- prints register probe info
- enables outputs (OE low) for a moment, then disables (OE high)

Use together with `bin/thruster_test.py --scan` to map wiring.

Safety:
- If your ESCs/thrusters are connected, enabling outputs will present *neutral* PWM.
"""

from __future__ import annotations
import time
from motion.pwm import ThrustWriter

def main():
    try:
        import rov_config as cfg  # type: ignore
        tw = ThrustWriter(
            i2c_bus=int(getattr(cfg, "PWM_I2C_BUS", 4)),
            i2c_addr=int(getattr(cfg, "PWM_I2C_ADDR", 0x40)),
            oe_chip=getattr(cfg, "PWM_OE_CHIP", "/dev/gpiochip0"),
            oe_line=int(getattr(cfg, "PWM_OE_LINE", 26)),
            freq_hz=float(getattr(cfg, "PWM_FREQ_HZ", 50.0)),
            osc_hz=float(getattr(cfg, "PWM_OSC_HZ", 24_576_000.0)),
            debug=True,
        )
    except Exception:
        tw = ThrustWriter(debug=True)

    try:
        info = tw.pwm.probe() if hasattr(tw.pwm, "probe") else {}
        print(f"[pwm_diag] probe: {info}")
        print("[pwm_diag] writing NEUTRAL on all mapped channels ...")
        tw.neutral()
        print("[pwm_diag] enabling outputs for 2s ...")
        tw.enable_outputs()
        time.sleep(2.0)
        print("[pwm_diag] disabling outputs for 2s ...")
        tw.disable_outputs()
        time.sleep(2.0)
        print("[pwm_diag] done.")
    finally:
        try:
            tw.neutral()
        except Exception:
            pass
        tw.close()

if __name__ == "__main__":
    main()
