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
#   --reboot                    (reboot automatically at the end)

set -euo pipefail

# ----------------------------
# Args
# ----------------------------
PROJECT_DIR="/home/TritonOS"
DO_NAV_OVERLAY=1
DO_LEGACY_CAMERA=1
DO_REBOOT=0

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

# ----------------------------
# OS packages
# ----------------------------
echo "[TritonOS] Installing OS dependencies…"
apt-get update -y

apt_install \
  ca-certificates curl git \
  python3 python3-pip python3-venv \
  python3-numpy python3-zmq python3-smbus2 python3-libgpiod \
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
if [[ ! -d ".venv" ]]; then
  # We need system-site-packages so python3-gi / Gst introspection works inside venv.
  sudo -u "$TARGET_USER" python3 -m venv --system-site-packages .venv
fi

sudo -u "$TARGET_USER" .venv/bin/python -m pip install --upgrade pip setuptools wheel

echo "[TritonOS] Installing Python deps…"
# Your code imports `bluerobotics_navigator` (underscore).
# The PyPI project is "bluerobotics-navigator" but pip usually accepts both spellings.
sudo -u "$TARGET_USER" .venv/bin/pip install --upgrade "bluerobotics-navigator" || \
sudo -u "$TARGET_USER" .venv/bin/pip install --upgrade "bluerobotics_navigator"

# ----------------------------
# Boot-time ROV startup
# ----------------------------
install_rov_service

# ----------------------------
# Quick sanity checks (non-fatal)
# ----------------------------
echo "[TritonOS] Sanity checks (non-fatal):"
sudo -u "$TARGET_USER" .venv/bin/python - <<'PY' || true
import sys
print("Python:", sys.version)
try:
    import zmq
    print("pyzmq OK:", zmq.__version__)
except Exception as e:
    print("pyzmq NOT OK:", e)
try:
    import bluerobotics_navigator as nav
    print("bluerobotics_navigator import OK")
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
