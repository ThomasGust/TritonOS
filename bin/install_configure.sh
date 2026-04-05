#!/usr/bin/env bash
#This is bad practice, the pat in question though is read only to one repository so I don't really care
set -euo pipefail

REPO_URL="https://github.com/ThomasGust/TritonOS.git"
DEST_DIR="/home/TritonOS"

# HARD-CODED CREDS (yes, bad practice)
GIT_USER="ThomasGust"
GIT_TOKEN="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"

sudo apt-get update -y
sudo apt-get install -y --no-install-recommends git ca-certificates curl

# Store credentials (plaintext on disk).
# Git's "store" helper writes to ~/.git-credentials. 
git config --global credential.helper store
printf "protocol=https\nhost=github.com\nusername=%s\npassword=%s\n\n" \
  "$GIT_USER" "$GIT_TOKEN" | git credential approve >/dev/null

if [ -d "$DEST_DIR/.git" ]; then
  git -C "$DEST_DIR" pull --ff-only
else
  git clone "$REPO_URL" "$DEST_DIR"
fi
# install_tritonos_pi.sh
#
# Usage (from your TritonOS repo root):
#   chmod +x ./install_tritonos_pi.sh
#   sudo ./install_tritonos_pi.sh
#
# Optional flags:
#   --project-dir <path>        (default: directory containing this script)
#   --no-navigator-overlay      (skip BlueRobotics board overlay script)
#   --no-legacy-camera          (skip enabling legacy camera + bcm2835-v4l2)
#   --recreate-venv             (delete and rebuild .venv before reinstalling deps)
#   --reboot                    (reboot automatically at the end)

set -euo pipefail

# ----------------------------
# Args
# ----------------------------
PROJECT_DIR="/home/TritonOS"
DO_NAV_OVERLAY=1
DO_LEGACY_CAMERA=1
DO_REBOOT=0
DO_RECREATE_VENV=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --project-dir)
      PROJECT_DIR="$2"
      shift 2
      ;;
    --no-navigator-overlay)
      DO_NAV_OVERLAY=0
      shift
      ;;
    --no-legacy-camera)
      DO_LEGACY_CAMERA=0
      shift
      ;;
    --reboot)
      DO_REBOOT=1
      shift
      ;;
    --recreate-venv)
      DO_RECREATE_VENV=1
      shift
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (use sudo)." >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-${USER}}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
if [[ -z "$TARGET_HOME" ]]; then
  echo "Could not resolve home directory for user: $TARGET_USER" >&2
  exit 1
fi

BOOT_CONFIG=""
if [[ -f /boot/firmware/config.txt ]]; then
  BOOT_CONFIG="/boot/firmware/config.txt"
elif [[ -f /boot/config.txt ]]; then
  BOOT_CONFIG="/boot/config.txt"
else
  echo "Could not find boot config.txt at /boot/firmware/config.txt or /boot/config.txt" >&2
  exit 1
fi

echo "[TritonOS] Project dir:  $PROJECT_DIR"
echo "[TritonOS] Target user:  $TARGET_USER"
echo "[TritonOS] Boot config:  $BOOT_CONFIG"

# ----------------------------
# Helpers
# ----------------------------
apt_install() {
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
}

apt_install_optional() {
  local pkg="$1"
  if apt-cache show "$pkg" >/dev/null 2>&1; then
    apt_install "$pkg"
  else
    echo "[TritonOS] Optional package not found in apt repo: $pkg (skipping)"
  fi
}

ensure_line_in_file() {
  local file="$1"
  local line="$2"
  grep -qxF "$line" "$file" 2>/dev/null || echo "$line" >>"$file"
}

set_kv_in_file() {
  # ensures a "key=value" line exists (replaces existing key=... if present, else appends)
  local file="$1"
  local key="$2"
  local value="$3"
  if grep -qE "^[#]*\s*${key}=" "$file"; then
    sed -i -E "s|^[#]*\s*(${key})=.*|\1=${value}|g" "$file"
  else
    echo "${key}=${value}" >>"$file"
  fi
}

install_rov_service() {
  local service_name="tritonos-rov.service"
  local service_path="/etc/systemd/system/${service_name}"

  echo "[TritonOS] Installing systemd service: ${service_name}"
  cat >"$service_path" <<EOF
[Unit]
Description=TritonOS ROV main service
After=local-fs.target systemd-modules-load.service
Wants=systemd-modules-load.service

[Service]
Type=simple
User=${TARGET_USER}
SupplementaryGroups=i2c gpio video
WorkingDirectory=${PROJECT_DIR}
Environment=PYTHONUNBUFFERED=1
ExecStart=${PROJECT_DIR}/.venv/bin/python ${PROJECT_DIR}/main_rov.py
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl daemon-reload
  systemctl unmask "$service_name" || true
  systemctl enable "$service_name"
  systemctl restart "$service_name"
}

run_as_target_user() {
  sudo -H -u "$TARGET_USER" env HOME="$TARGET_HOME" PYTHONNOUSERSITE=1 "$@"
}

venv_python() {
  run_as_target_user "$PROJECT_DIR/.venv/bin/python" "$@"
}

recreate_venv() {
  local reason="${1:-requested}"
  echo "[TritonOS] Recreating Python venv (${reason})..."
  rm -rf "$PROJECT_DIR/.venv"
  run_as_target_user python3 -m venv --system-site-packages "$PROJECT_DIR/.venv"
}

ensure_venv() {
  local reason=""

  if [[ "$DO_RECREATE_VENV" -eq 1 ]]; then
    reason="requested via --recreate-venv"
  elif [[ ! -x "$PROJECT_DIR/.venv/bin/python" ]]; then
    reason="missing .venv/bin/python"
  elif [[ ! -f "$PROJECT_DIR/.venv/pyvenv.cfg" ]]; then
    reason="missing pyvenv.cfg"
  elif ! grep -Eq '^include-system-site-packages *= *true$' "$PROJECT_DIR/.venv/pyvenv.cfg"; then
    reason="venv was not created with --system-site-packages"
  elif ! venv_python -c "import sys; print(sys.executable)" >/dev/null 2>&1; then
    reason="venv python is not runnable"
  fi

  if [[ -n "$reason" ]]; then
    recreate_venv "$reason"
  fi
}

install_python_deps() {
  echo "[TritonOS] Installing Python deps..."
  if [[ -f "$PROJECT_DIR/requirements.txt" ]]; then
    # This venv uses --system-site-packages, so many hardware deps already come
    # from apt. Do not use --upgrade here: it can force pip to replace working
    # distro packages (for example python3-spidev) with a source build that then
    # fails or becomes less stable.
    venv_python -m pip install --prefer-binary -r "$PROJECT_DIR/requirements.txt"
  fi
}

clean_navigator_install() {
  echo "[TritonOS] Removing stale Navigator package state..."
  venv_python -m pip uninstall -y bluerobotics-navigator bluerobotics_navigator >/dev/null 2>&1 || true
  find "$PROJECT_DIR/.venv" \
    \( \
      -path '*/site-packages/bluerobotics_navigator*' -o \
      -path '*/site-packages/bluerobotics-navigator-*.dist-info' -o \
      -path '*/site-packages/bluerobotics_navigator-*.dist-info' \
    \) \
    -exec rm -rf {} + 2>/dev/null || true
}

install_navigator_bindings() {
  echo "[TritonOS] Reinstalling Navigator Python bindings cleanly..."
  clean_navigator_install
  # The import name is ``bluerobotics_navigator`` but the canonical PyPI
  # project name is ``bluerobotics-navigator``.
  venv_python -m pip install --no-cache-dir --force-reinstall --upgrade --prefer-binary "bluerobotics-navigator"
}

verify_navigator_bindings() {
  local pass_label="${1:-}"
  if [[ -n "$pass_label" ]]; then
    echo "[TritonOS] Verifying Navigator Python bindings (${pass_label})..."
  else
    echo "[TritonOS] Verifying Navigator Python bindings..."
  fi
  venv_python - <<'PY'
from pathlib import Path
import importlib
import importlib.metadata as md

from utils.navigator_import import import_navigator_module, navigator_api_info

required = ("set_pwm_freq_hz", "set_pwm_enable", "set_pwm_channel_value")

try:
    version = md.version("bluerobotics-navigator")
except Exception as exc:
    raise SystemExit(f"Navigator wheel metadata missing for bluerobotics-navigator: {exc}")

try:
    dist = md.distribution("bluerobotics-navigator")
except Exception as exc:
    raise SystemExit(f"Navigator distribution lookup failed: {exc}")

dist_root = Path(dist.locate_file(""))
dist_info_dirs = sorted(p.name for p in dist_root.glob("bluerobotics_navigator-*.dist-info"))
if not dist_info_dirs:
    raise SystemExit(
        f"Navigator dist-info directory missing under {dist_root}"
    )

pkg = importlib.import_module("bluerobotics_navigator")
pkg_file = Path(getattr(pkg, "__file__", ""))
if not pkg_file.exists():
    raise SystemExit(f"Navigator package __file__ missing on disk: {pkg_file}")

pkg_dir = pkg_file.parent
ext_candidates = sorted(p.name for p in pkg_dir.glob("bluerobotics_navigator*.so"))
if not ext_candidates:
    raise SystemExit(
        f"Navigator compiled extension missing under {pkg_dir}"
    )

nav = import_navigator_module()
info = navigator_api_info(nav)
print("Navigator API info:", info)
print("Navigator dist-info:", dist_info_dirs)
print("Navigator extension candidates:", ext_candidates)
print("Navigator version:", version)

missing = [name for name in required if not info.get(f"has_{name}", False)]
if missing:
    raise SystemExit(
        "Navigator Python bindings installed incorrectly. Missing required API: "
        + ", ".join(missing)
    )
PY
}

# ----------------------------
# OS packages
# ----------------------------
echo "[TritonOS] Installing OS dependencies…"
apt-get update -y

apt_install \
  ca-certificates curl git \
  build-essential pkg-config \
  python3 python3-dev python3-pip python3-venv \
  python3-numpy python3-zmq python3-smbus2 python3-libgpiod python3-spidev \
  i2c-tools v4l-utils \
  python3-gi python3-gi-cairo \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gir1.2-gstreamer-1.0

# Helpful camera-related packages (availability varies by Pi OS release)
apt_install_optional libcamera-apps
apt_install_optional libcamera0.7 || apt_install_optional libcamera0.6
apt_install_optional gstreamer1.0-libcamera
apt_install_optional ffmpeg

# raspi-config isn't always installed on minimal images
apt_install_optional raspi-config

# ----------------------------
# Permissions/groups for hardware access
# ----------------------------
echo "[TritonOS] Adding user '$TARGET_USER' to groups: i2c, gpio, video…"
usermod -aG i2c,gpio,video "$TARGET_USER" || true

# ----------------------------
# Enable interfaces
# ----------------------------
echo "[TritonOS] Enabling I2C…"
if command -v raspi-config >/dev/null 2>&1; then
  # Official docs: raspi-config nonint do_i2c 0 enables I2C
  raspi-config nonint do_i2c 0 || true
else
  # Fallback: ensure dtparam
  set_kv_in_file "$BOOT_CONFIG" "dtparam" "i2c_arm=on"
fi

# ----------------------------
# Navigator board overlay setup (recommended by Blue Robotics docs)
# ----------------------------
if [[ "$DO_NAV_OVERLAY" -eq 1 ]]; then
  echo "[TritonOS] Running Blue Robotics 'configure_board.sh' (Navigator overlays, I2C/SPI/GPIO, etc)…"
  # Source referenced by the official Navigator package docs.
  # This will edit boot config/overlays; a reboot is required afterwards.
  curl -fsSL https://raw.githubusercontent.com/bluerobotics/blueos-docker/master/install/boards/configure_board.sh | bash
else
  echo "[TritonOS] Skipping Navigator overlay setup (--no-navigator-overlay)."
fi

# ----------------------------
# Legacy camera (/dev/video0) for v4l2src pipelines
# ----------------------------
if [[ "$DO_LEGACY_CAMERA" -eq 1 ]]; then
  echo "[TritonOS] Enabling legacy camera support (for /dev/video0 via bcm2835-v4l2) …"
  if command -v raspi-config >/dev/null 2>&1; then
    # For legacy camera support, raspi-config uses do_legacy on newer raspi-config versions.
    # If it fails (older builds), we fall back to config.txt edits below.
    raspi-config nonint do_legacy 0 || true
  fi

  # Ensure legacy stack settings in config.txt (safe/idempotent)
  set_kv_in_file "$BOOT_CONFIG" "start_x" "1"
  set_kv_in_file "$BOOT_CONFIG" "gpu_mem" "128"
  # Optional: turn off camera LED (comment out if you want LED)
  # set_kv_in_file "$BOOT_CONFIG" "disable_camera_led" "1"

  # Load V4L2 driver at boot
  cat >/etc/modules-load.d/tritonos.conf <<'EOF'
# TritonOS: camera V4L2 driver for legacy camera stack
bcm2835-v4l2
EOF

  # Load now (won't hurt if it fails until reboot)
  modprobe bcm2835-v4l2 || true
else
  echo "[TritonOS] Skipping legacy camera setup (--no-legacy-camera)."
fi

# ----------------------------
# Python environment (venv)
# ----------------------------
sudo chown -R "$TARGET_USER":"$TARGET_USER" "$PROJECT_DIR"
sudo chmod -R u+rwX "$PROJECT_DIR"
for script in "$PROJECT_DIR"/bin/*.sh; do
  [[ -e "$script" ]] || continue
  chmod +x "$script"
done
echo "[TritonOS] Creating/Updating Python venv in project…"
cd "$PROJECT_DIR"
ensure_venv

venv_python -m pip install --upgrade pip setuptools wheel

echo "[TritonOS] Installing Python deps…"
if [[ -f "requirements.txt" ]]; then
  # This venv uses --system-site-packages, so many hardware deps already come
  # from apt. Do not use --upgrade here: it can force pip to replace working
  # distro packages (for example python3-spidev) with a source build that then
  # fails or becomes less stable.
  venv_python -m pip install --prefer-binary -r "$PROJECT_DIR/requirements.txt"
fi

echo "[TritonOS] Reinstalling Navigator Python bindings cleanly…"
clean_navigator_install

# Your code imports `bluerobotics_navigator` (underscore).
# The PyPI project is "bluerobotics-navigator".
venv_python -m pip install --no-cache-dir --force-reinstall --upgrade --prefer-binary "bluerobotics-navigator"

echo "[TritonOS] Verifying Navigator Python bindings…"
if ! verify_navigator_bindings "pass 1" || ! verify_navigator_bindings "pass 2"; then
  echo "[TritonOS] Navigator install/import failed; rebuilding venv and retrying once..."
  recreate_venv "Navigator install/import verification failure"
  venv_python -m pip install --upgrade pip setuptools wheel
  install_python_deps
  install_navigator_bindings
  verify_navigator_bindings "retry pass 1"
  verify_navigator_bindings "retry pass 2"
fi

# ----------------------------
# Boot-time ROV startup
# ----------------------------
install_rov_service

# ----------------------------
# Quick sanity checks (non-fatal)
# ----------------------------
echo "[TritonOS] Sanity checks (non-fatal):"
venv_python - <<'PY' || true
import sys
print("Python:", sys.version)
try:
    import zmq
    print("pyzmq OK:", zmq.__version__)
except Exception as e:
    print("pyzmq NOT OK:", e)
try:
    from utils.navigator_import import import_navigator_module, navigator_api_info
    nav = import_navigator_module()
    print("bluerobotics_navigator API:", navigator_api_info(nav))
except Exception as e:
    print("bluerobotics_navigator import NOT OK:", e)
try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    print("GStreamer GI OK")
except Exception as e:
    print("GStreamer GI NOT OK:", e)
PY

echo
echo "[TritonOS] Done."
echo "Next run (from repo root):"
echo "  sudo -u $TARGET_USER $PROJECT_DIR/.venv/bin/python $PROJECT_DIR/main_rov.py"
echo "Boot service:"
echo "  sudo systemctl status tritonos-rov.service"
echo "Logs:"
echo "  sudo journalctl -u tritonos-rov.service -f"
echo "Debug helper:"
echo "  sudo bash $PROJECT_DIR/bin/rov_debug.sh run"
echo
echo "NOTE: Interface/camera/overlay changes usually require a reboot to take full effect."

if [[ "$DO_REBOOT" -eq 1 ]]; then
  echo "[TritonOS] Rebooting now…"
  reboot
fi
