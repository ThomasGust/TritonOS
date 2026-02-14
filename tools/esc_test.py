"""Legacy ESC test (single channel).

Prefer tools.native_motor_test or tools.thruster_test.
"""

import time

from motion import NavigatorPWM


def main() -> None:
    pwm = NavigatorPWM(freq_hz=50.0)
    channel = 1

    # Proper ESC init: set neutral, enable PWM, hold neutral.
    pwm.set_servo_us(channel, 1500)
    pwm.arm()
    print("[esc_test] PWM enabled; holding neutral")
    time.sleep(3.0)

    # Small steps.
    pwm.set_servo_us(channel, 1490)
    time.sleep(2)
    pwm.set_servo_us(channel, 1700)
    time.sleep(2)
    pwm.set_servo_us(channel, 1500)
    time.sleep(2)
    pwm.set_servo_us(channel, 1200)
    time.sleep(2)
    pwm.set_servo_us(channel, 1500)
    time.sleep(2)

    pwm.disarm()


if __name__ == "__main__":
    main()