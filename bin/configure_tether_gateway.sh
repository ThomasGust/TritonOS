#!/usr/bin/env bash
set -euo pipefail

# Configure the Pi to use the pilot computer as the tether internet gateway.
#
# Safe default: only probe the gateway. Use --temporary or --persistent to make
# changes, and the script still refuses to switch routes unless the tether
# gateway responds first.

IFACE="${TRITON_TETHER_IFACE:-eth0}"
ADDR="${TRITON_TETHER_ADDR:-192.168.1.4/24}"
GATEWAY="${TRITON_TETHER_GATEWAY:-192.168.1.1}"
DNS="${TRITON_TETHER_DNS:-8.8.8.8 1.1.1.1}"
METRIC="${TRITON_TETHER_METRIC:-50}"
WIFI_METRIC="${TRITON_WIFI_BACKUP_METRIC:-600}"
CONNECTION="${TRITON_TETHER_CONNECTION:-}"
MODE="probe"
FORCE=0

log() {
  printf '[tether-gateway] %s\n' "$*"
}

die() {
  printf '[tether-gateway] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Configure or test routing from the Pi through the pilot computer.

Usage:
  sudo bash bin/configure_tether_gateway.sh [options]

Options:
  --probe             test only (default)
  --temporary         add a temporary default route via the tether
  --persistent        update the NetworkManager wired profile
  --force             apply even if the tether gateway probe fails
  --iface <name>      tether interface (default: eth0)
  --addr <cidr>       tether address (default: 192.168.1.4/24)
  --gateway <ip>      pilot tether IP (default: 192.168.1.1)
  --dns <servers>     quoted DNS server list (default: "8.8.8.8 1.1.1.1")
  --metric <number>   tether default route metric (default: 50)
  --wifi-metric <n>   Wi-Fi backup route metric for --persistent (default: 600)
  --connection <name> NetworkManager wired connection name
  -h, --help          show this help

The script refuses to install a default route unless the pilot computer is
reachable at --gateway first, because a link can report UP even when Ethernet
frames are not actually crossing the tether.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --probe)
      MODE="probe"
      shift
      ;;
    --temporary)
      MODE="temporary"
      shift
      ;;
    --persistent)
      MODE="persistent"
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --iface)
      [[ $# -ge 2 ]] || die "--iface requires a value"
      IFACE="$2"
      shift 2
      ;;
    --addr)
      [[ $# -ge 2 ]] || die "--addr requires a value"
      ADDR="$2"
      shift 2
      ;;
    --gateway)
      [[ $# -ge 2 ]] || die "--gateway requires a value"
      GATEWAY="$2"
      shift 2
      ;;
    --dns)
      [[ $# -ge 2 ]] || die "--dns requires a quoted server list"
      DNS="$2"
      shift 2
      ;;
    --metric)
      [[ $# -ge 2 ]] || die "--metric requires a value"
      METRIC="$2"
      shift 2
      ;;
    --wifi-metric)
      [[ $# -ge 2 ]] || die "--wifi-metric requires a value"
      WIFI_METRIC="$2"
      shift 2
      ;;
    --connection)
      [[ $# -ge 2 ]] || die "--connection requires a value"
      CONNECTION="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  die "run with sudo so the script can test ARP and change routes safely"
fi

command -v ip >/dev/null 2>&1 || die "missing ip command"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  die "interface not found: $IFACE"
fi

detect_connection() {
  if [[ -n "$CONNECTION" ]]; then
    printf '%s\n' "$CONNECTION"
    return 0
  fi
  if command -v nmcli >/dev/null 2>&1; then
    nmcli -t -f NAME,DEVICE con show --active | awk -F: -v iface="$IFACE" '$2 == iface { print $1; exit }'
  fi
}

gateway_reachable() {
  if command -v arping >/dev/null 2>&1; then
    arping -c 2 -w 3 -I "$IFACE" "$GATEWAY" >/dev/null 2>&1
    return $?
  fi
  ping -c 2 -W 1 -I "$IFACE" "$GATEWAY" >/dev/null 2>&1
}

show_state() {
  log "Interface state:"
  ip -br addr show "$IFACE" || true
  ip -s link show "$IFACE" || true
  log "Routes:"
  ip route || true
  log "Neighbor table:"
  ip neigh show dev "$IFACE" || true
}

show_state

if gateway_reachable; then
  log "Gateway $GATEWAY is reachable on $IFACE."
else
  log "Gateway $GATEWAY is NOT reachable on $IFACE."
  if [[ "$FORCE" -ne 1 ]]; then
    if [[ "$MODE" == "probe" ]]; then
      exit 2
    fi
    die "not changing routes; fix tether L2 first or rerun with --force"
  fi
fi

if [[ "$MODE" == "probe" ]]; then
  exit 0
fi

if ! ip addr show dev "$IFACE" | grep -q "${ADDR%/*}/"; then
  log "Adding tether address $ADDR to $IFACE."
  ip addr replace "$ADDR" dev "$IFACE"
fi

if [[ "$MODE" == "temporary" ]]; then
  log "Installing temporary default route via $GATEWAY on $IFACE."
  ip route replace default via "$GATEWAY" dev "$IFACE" metric "$METRIC"
  if command -v resolvectl >/dev/null 2>&1; then
    # shellcheck disable=SC2086
    resolvectl dns "$IFACE" $DNS || true
  fi
elif [[ "$MODE" == "persistent" ]]; then
  command -v nmcli >/dev/null 2>&1 || die "NetworkManager/nmcli is required for --persistent"
  CONNECTION="$(detect_connection)"
  [[ -n "$CONNECTION" ]] || die "could not detect active NetworkManager connection for $IFACE"
  log "Updating NetworkManager connection '$CONNECTION'."
  nmcli con mod "$CONNECTION" \
    ipv4.method manual \
    ipv4.addresses "$ADDR" \
    ipv4.gateway "$GATEWAY" \
    ipv4.dns "$DNS" \
    ipv4.route-metric "$METRIC" \
    ipv4.never-default no \
    connection.autoconnect yes

  wifi_connection="$(nmcli -t -f NAME,TYPE,DEVICE con show --active | awk -F: '$2 == "wifi" { print $1; exit }')"
  if [[ -n "$wifi_connection" ]]; then
    log "Keeping Wi-Fi as backup with route metric $WIFI_METRIC on '$wifi_connection'."
    nmcli con mod "$wifi_connection" ipv4.route-metric "$WIFI_METRIC" || true
  fi
  nmcli con up "$CONNECTION"
else
  die "internal error: unknown mode $MODE"
fi

show_state
log "Internet route probe:"
ip route get 1.1.1.1 || true
if command -v curl >/dev/null 2>&1; then
  curl -4 -I --connect-timeout 5 --max-time 8 https://github.com >/dev/null && \
    log "GitHub HTTPS probe passed." || \
    log "GitHub HTTPS probe failed."
fi
