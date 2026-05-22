# Network Guide

This guide explains how TritonOS communicates with the pilot computer and how
to configure the tether path used during operation and code updates.

## Default Topology

The normal competition layout is:

```text
Pilot computer Ethernet: 192.168.1.1/24
ROV Ethernet:            192.168.1.4/24
```

The ROV services bind on `0.0.0.0`, which means they listen on all local
interfaces. TritonPilot should connect to the ROV using the tether IP unless a
different network is intentionally being used.

## Ports And Protocols

| Purpose | Direction | Default |
| --- | --- | --- |
| Pilot control frames | TritonPilot -> TritonOS | ROV TCP `6000` |
| Sensor telemetry | TritonOS -> TritonPilot | ROV TCP `6001` |
| Video control RPC | TritonPilot -> TritonOS | ROV TCP `5555` |
| Management RPC | tools/TritonPilot -> TritonOS | ROV TCP `5556` |
| Camera RTP/UDP payloads | TritonOS -> TritonPilot | pilot UDP stream ports, commonly `5000+` |
| Network diagnostics | pilot tools -> TritonOS | ROV TCP/UDP `7700` |

The exact values live in `rov_config.py`.

## Control And Telemetry Endpoints

TritonOS uses ZeroMQ for control and telemetry:

- `PILOT_SUB_ENDPOINT = "tcp://0.0.0.0:6000"`
- `SENSOR_PUB_ENDPOINT = "tcp://0.0.0.0:6001"`
- `MANAGEMENT_RPC_ENDPOINT = "tcp://0.0.0.0:5556"`

The names describe TritonOS behavior. For example, `PILOT_SUB_ENDPOINT` is a
SUB socket from TritonOS's perspective. TritonPilot publishes pilot frames to
that address.

## Video Networking

Video control and video payloads are separate:

- Video RPC runs on `VIDEO_RPC_ENDPOINT`, normally `tcp://0.0.0.0:5555`.
- Camera payloads are GStreamer RTP streams sent from the ROV to the pilot
  computer using UDP or TCP depending on stream config.

For tether-first video routing, use:

```python
VIDEO_ENFORCE_TETHER = True
VIDEO_TETHER_IFACE = "eth0"
VIDEO_TETHER_SRC_IP = "192.168.1.4"
VIDEO_ENFORCE_HOST_ROUTE = True
```

Only enable those settings when the tether interface and addresses are known.
The helper code in `video/tether.py` is best-effort and should not be treated as
a replacement for checking the OS route table.

## Windows Pilot Computer Setup

Configure the pilot computer Ethernet adapter to use:

```text
IP address: 192.168.1.1
Subnet:     255.255.255.0
```

If the ROV needs internet through the pilot computer, configure internet
sharing/NAT from the TritonPilot repository:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\setup_tether_nat.ps1 -TuneAdapter -ResetAdapter
```

Run that command on the pilot computer, not on the ROV.

## Pi Tether Gateway Setup

Run these commands on the ROV:

```bash
cd /home/TritonOS
sudo bash bin/configure_tether_gateway.sh --probe
```

If the probe succeeds, install a persistent route:

```bash
sudo bash bin/configure_tether_gateway.sh --persistent
```

The script refuses to install the route unless the gateway responds first. That
guard matters because an Ethernet link can report `UP` even when frames are not
actually reaching the pilot computer.

For a temporary route that does not change NetworkManager profiles:

```bash
sudo bash bin/configure_tether_gateway.sh --temporary
```

## Custom Tether Values

The gateway script accepts flags:

```bash
sudo bash bin/configure_tether_gateway.sh \
  --iface eth0 \
  --addr 192.168.1.4/24 \
  --gateway 192.168.1.1 \
  --dns "8.8.8.8 1.1.1.1" \
  --persistent
```

The same values can be supplied as environment variables:

```bash
export TRITON_TETHER_IFACE=eth0
export TRITON_TETHER_ADDR=192.168.1.4/24
export TRITON_TETHER_GATEWAY=192.168.1.1
sudo -E bash bin/configure_tether_gateway.sh --probe
```

## Basic Connectivity Checks

Run on the pilot computer:

```powershell
ping 192.168.1.4
ssh triton@192.168.1.4
```

Run on the ROV:

```bash
ip -br addr
ip route
ping -c 2 192.168.1.1
```

If the pilot computer is sharing internet:

```bash
curl -4 -I --connect-timeout 5 --max-time 8 https://github.com
```

## TritonOS Network Diagnostics

When `NETDIAG_ENABLE = True`, TritonOS starts the lightweight diagnostics
server from `tools/netdiag_server.py` on port `7700`.

From a pilot-side tool, use that server to test:

- UDP echo latency/loss.
- TCP receive throughput.
- TCP transmit throughput.

If the diagnostics server is not running, check:

```bash
sudo journalctl -u tritonos-rov.service -n 100
```

## Common Failure Patterns

### Pilot Cannot Connect To ROV

Check in this order:

1. Ethernet link lights are on.
2. Pilot Ethernet adapter has `192.168.1.1/24`.
3. ROV has `192.168.1.4/24` on the tether interface.
4. `ping 192.168.1.4` works from pilot.
5. `sudo systemctl status tritonos-rov.service` is active.
6. Firewall is not blocking the TritonOS ports.

### Sensor Stream Is Missing

Check whether TritonOS is publishing:

```bash
sudo journalctl -u tritonos-rov.service -f
```

Then run a fake stream in isolation:

```bash
cd /home/TritonOS
python -m tools.sensor_stream_pub_test --fake
```

Connect from TritonPilot's sensor subscriber test to the ROV address.

### Video RPC Works But No Video Appears

Separate the control path from payload path:

1. Confirm `VIDEO_RPC_ENDPOINT` is reachable on port `5555`.
2. Run `python -m tools.rov_preflight` on the ROV to list cameras and formats.
3. Confirm the stream host points at the pilot computer's tether IP.
4. Confirm the pilot firewall allows incoming UDP video payloads.
5. Check GStreamer errors in `journalctl`.

### Pi Can SSH But Cannot Pull Updates

SSH only proves the pilot can reach the Pi. It does not prove the Pi can reach
the internet. On the ROV:

```bash
ip route
ping -c 2 192.168.1.1
curl -4 -I --connect-timeout 5 --max-time 8 https://github.com
```

If GitHub is unreachable, configure Windows NAT and rerun:

```bash
sudo bash bin/configure_tether_gateway.sh --probe
sudo bash bin/configure_tether_gateway.sh --persistent
```
