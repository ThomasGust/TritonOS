# Triton Standalone AHRS (RPi 4 + Navigator)

This is a **standalone** AHRS you can run on a Raspberry Pi 4 with a Blue Robotics **Navigator** (or compatible) sensor stack.

It includes:
- Sensor access (ICM20602 accel/gyro over SPI or I2C, auto-detected)
- Magnetometer access (AK09915 over I2C, with optional MMC5983 auto-detect)
- Robust gyro bias calibration
- Robust mag calibration (hard + soft iron) and mag health gating
- Madgwick quaternion AHRS with automatic 9DOF → 6DOF fallback when mag is unhealthy

## 1) Enable SPI + I2C on the Pi

Use `raspi-config`:
- Interface Options → **SPI** → enable
- Interface Options → **I2C** → enable

Reboot.

## 2) Install dependencies

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev i2c-tools
pip3 install -r requirements.txt
```

If `spidev` install fails, try:
```bash
sudo apt-get install -y python3-spidev
```

## 3) Run AHRS (quick start)

```bash
python3 -m triton_ahrs.run_ahrs --auto-gyro-cal --yaw-zero --zero-attitude
```

You should see roll/pitch/yaw printing at ~20 Hz.

### Helpful options
- Faster lock + less jitter (recommended defaults already):
  - `--init-seconds` seeds initial attitude from averaged accel(+mag) so you don't wait ~30s to settle.
  - `--warmup-seconds` / `--beta-init` converges quickly at startup then drops to steady beta.
  - `--bias-adapt-tau` refines gyro bias only when stationary.


- Log to CSV:
  ```bash
  python3 -m triton_ahrs.run_ahrs --auto-gyro-cal --yaw-zero --log-csv ahrs_log.csv
  ```

- Emit JSON-lines (easy to pipe into other tools):
  ```bash
  python3 -m triton_ahrs.run_ahrs --auto-gyro-cal --yaw-zero --json
  ```

## 4) Calibrate sensors (recommended)

### Gyro bias (stationary)

```bash
python3 -m triton_ahrs.calibrate_gyro --seconds 8 --out gyro_cal.json
```

### Magnetometer (rotate through many orientations)

```bash
python3 -m triton_ahrs.calibrate_mag --seconds 90 --out mag_cal.json
```

Then run with:

```bash
python3 -m triton_ahrs.run_ahrs --gyro-cal gyro_cal.json --mag-cal mag_cal.json --yaw-zero
```

## 5) Axis mapping / mount matrix (if needed)

If roll/pitch signs or axes look wrong, create a mount JSON:

```json
{
  "R": [[1,0,0],[0,1,0],[0,0,1]]
}
```

This maps **sensor axes → body axes**: `v_body = R @ v_sensor`.

Run with:
```bash
python3 -m triton_ahrs.run_ahrs --mount mount.json ...
```

## Notes

- If you run thrusters near the magnetometer, yaw can be corrupted. This AHRS detects that and automatically drops to 6DOF (gyro+accel) until the field is healthy again.
- For operator-friendly behavior, `--yaw-zero` is enabled in most examples (yaw starts at 0 and stays smooth).