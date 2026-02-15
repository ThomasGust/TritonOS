"""Standalone AHRS for Raspberry Pi + Blue Robotics Navigator (ICM20602 + AK09915).

This project is intentionally self-contained: sensor drivers + calibration + AHRS.

Run:
  python3 -m triton_ahrs.run_ahrs

Calibrate:
  python3 -m triton_ahrs.calibrate_gyro
  python3 -m triton_ahrs.calibrate_mag
"""
