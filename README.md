# TritonOS

TritonOS is the onboard runtime for the ROV. It starts the pilot/control loop,
hardware output sink, sensor publisher, video RPC service, and management RPC
service used by the topside TritonPilot app.

Current stability software is intentionally limited to depth hold. Raw IMU
telemetry still publishes accelerometer, gyroscope, AK09915 magnetometer, and
optional MMC5983 magnetometer samples so the next estimator pipeline can be
built from clean primitives.

## Raw Sensor Stream

The ROV publishes sensor telemetry on `rov_config.SENSOR_PUB_ENDPOINT`, usually
`tcp://0.0.0.0:6001`. IMU messages include raw accel, gyro, the primary AK09915
mag vector, and `mag_sources` for all detected raw magnetometers.

For isolated stream testing on the ROV:

```bash
python tools/sensor_stream_pub_test.py --fake
```

Then connect from the TritonPilot checkout with:

```bash
python tools/sensor_stream_sub_test.py --endpoint tcp://<rov-ip>:6001
```

## Test

```bash
pytest
```

The pytest configuration uses a repo-local `.pytest-tmp` directory and skips
tests marked `hardware` by default so the suite can run on a development
machine without touching physical ROV devices.
