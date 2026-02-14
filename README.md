# TritonOS
Operating software for Triton Robotics ROVs.

## Network transparency

TritonOS publishes live network telemetry (interface, IP, link state/speed,
RX/TX throughput, errors/drops) onto the existing sensor stream
(``rov_config.SENSOR_PUB_ENDPOINT``) as a ``type="net"`` message.

An optional lightweight netdiag server (UDP echo + TCP throughput) can be
started automatically by ``main_rov.py`` (see ``rov_config.NETDIAG_*``).
