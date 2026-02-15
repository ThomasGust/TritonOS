# Triton Standalone AHRS (RPi + Navigator)

This is a standalone attitude estimator (AHRS) designed to run on a Raspberry Pi + Blue Robotics Navigator-style sensor stack.

Features:
- Reads **ICM20602 accel/gyro** (SPI preferred, I2C fallback)
- Reads **AK09915 mag** (I2C), optional **MMC5983** if present
- **Madgwick quaternion AHRS** (IMU-only roll/pitch) + **robust magnetometer yaw correction** (default), with classic 9DOF Madgwick available
- **Startup gyro bias calibration**
- **Startup attitude seeding** (fast lock, no long settle)
- **Gain scheduling** (fast warmup, stable steady-state)
- **Accel/mag health gating** + mag hysteresis
- Optional **stationary gyro-bias refinement**
- Optional **LPF** on accel/mag
- Output as human text or JSON lines

## Install

Enable SPI + I2C with `raspi-config`, then:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

## Run

You can run either module path:

```bash
python -m triton_ahrs.run_ahrs --auto-gyro-cal --yaw-zero --prefer-spi --zero-attitude
# OR (compat wrapper)
python -m triton_ahrs_standalone.triton_ahrs.run_ahrs --auto-gyro-cal --yaw-zero --prefer-spi --zero-attitude
```

## Calibration (recommended)

Gyro bias:
```bash
python -m triton_ahrs.calibrate_gyro --seconds 8 --out gyro_cal.json --prefer-spi
```

Mag calibration:
```bash
python -m triton_ahrs.calibrate_mag --seconds 90 --out mag_cal.json --prefer-spi
```

Then:
```bash
python -m triton_ahrs.run_ahrs --gyro-cal gyro_cal.json --mag-cal mag_cal.json --yaw-zero --prefer-spi --zero-attitude
```

## Notes

- If you still see a 180° flip when level, try:
  - `--accel-sign invert` (forces accel sign), or
  - provide a proper mount matrix (`--mount mount.json`).

- For operator-friendly behavior, `--zero-attitude` is recommended during bringup.
