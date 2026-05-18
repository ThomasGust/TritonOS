# TritonOS

TritonOS is the onboard runtime for the ROV. It starts the pilot/control loop,
hardware output sink, sensor publisher, video RPC service, and management RPC
service used by the topside TritonPilot app.

Current stability software is intentionally limited to depth hold. Raw IMU
telemetry still publishes accelerometer, gyroscope, AK09915 magnetometer, and
optional MMC5983 magnetometer samples so the next estimator pipeline can be
built from clean primitives.

## Test

```bash
pytest
```

The pytest configuration uses a repo-local `.pytest-tmp` directory and skips
tests marked `hardware` by default so the suite can run on a development
machine without touching physical ROV devices.
