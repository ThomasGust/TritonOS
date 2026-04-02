#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/TritonOS}"
SERVICE_NAME="${SERVICE_NAME:-tritonos-rov.service}"
MAIN_SCRIPT="${PROJECT_DIR}/main_rov.py"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
RUN_AS_USER="${SUDO_USER:-${USER}}"

usage() {
  cat <<EOF
Usage: sudo bash bin/rov_debug.sh <command>

Commands:
  stop    Stop the systemd service and force-kill any leftover main_rov.py process
  run     Do 'stop', then launch main_rov.py in the foreground for live debugging
  status  Show service state and any matching main_rov.py processes
  logs    Show recent service logs
EOF
}

require_root() {
  if [[ ${EUID} -ne 0 ]]; then
    echo "Please run with sudo so the script can control systemd and kill stale ROV processes." >&2
    exit 1
  fi
}

kill_matching_main() {
  local pids
  pids="$(pgrep -f "$MAIN_SCRIPT" || true)"
  if [[ -z "$pids" ]]; then
    echo "[rov_debug] No lingering main_rov.py process found."
    return
  fi

  echo "[rov_debug] Sending SIGTERM to lingering main_rov.py process(es): $pids"
  kill $pids || true
  sleep 1

  pids="$(pgrep -f "$MAIN_SCRIPT" || true)"
  if [[ -n "$pids" ]]; then
    echo "[rov_debug] Escalating to SIGKILL for process(es): $pids"
    kill -9 $pids || true
  fi
}

stop_service_and_processes() {
  echo "[rov_debug] Stopping ${SERVICE_NAME} if it is running..."
  systemctl stop "$SERVICE_NAME" 2>/dev/null || true
  systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true
  kill_matching_main
}

show_status() {
  echo "[rov_debug] systemd status for ${SERVICE_NAME}:"
  systemctl status "$SERVICE_NAME" --no-pager -l || true
  echo
  echo "[rov_debug] Matching main_rov.py processes:"
  pgrep -af "$MAIN_SCRIPT" || echo "[rov_debug] No matching processes."
}

show_logs() {
  journalctl -u "$SERVICE_NAME" -n 100 --no-pager || true
}

run_foreground() {
  stop_service_and_processes

  if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "[rov_debug] Python venv not found at $PYTHON_BIN" >&2
    exit 1
  fi

  echo "[rov_debug] Starting main_rov.py in the foreground as ${RUN_AS_USER}"
  cd "$PROJECT_DIR"
  exec sudo -u "$RUN_AS_USER" "$PYTHON_BIN" "$MAIN_SCRIPT"
}

main() {
  local cmd="${1:-}"
  if [[ -z "$cmd" ]]; then
    usage
    exit 1
  fi

  require_root

  case "$cmd" in
    stop)
      stop_service_and_processes
      ;;
    run)
      run_foreground
      ;;
    status)
      show_status
      ;;
    logs)
      show_logs
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
