#!/usr/bin/env bash
set -euo pipefail

# Fast TritonOS code updater for the ROV.
#
# Default behavior is intentionally code-first:
#   - do not run apt-get update unless a required tool is missing
#   - do not run git fsck unless requested
#   - bound apt network timeouts so a Pi with partial internet does not hang
#
# Useful flags:
#   --with-apt       refresh/install base packages even if tools exist
#   --fsck           run git fsck --connectivity-only before pulling
#   --dir <path>     repository path (default: current repo, else /home/TritonOS)
#   --branch <name>  branch to pull (default: current branch, else main)

REPO_URL="${TRITONOS_REPO_URL:-https://github.com/ThomasGust/TritonOS.git}"
DEST_DIR="${TRITONOS_DIR:-}"
BRANCH="${TRITONOS_BRANCH:-}"
DO_APT=0
DO_FSCK=0
GIT_TIMEOUT_S="${TRITON_GIT_TIMEOUT_S:-30}"

log() {
  printf '[TritonOS update] %s\n' "$*"
}

warn() {
  printf '[TritonOS update] WARNING: %s\n' "$*" >&2
}

die() {
  printf '[TritonOS update] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Fast TritonOS code updater for the ROV.

Usage:
  sudo bash bin/update_code.sh [options]

Options:
  --with-apt       refresh/install base packages even if tools exist
  --fsck           run git fsck --connectivity-only before pulling
  --dir <path>     repository path (default: current repo, else /home/TritonOS)
  --branch <name>  branch to pull (default: current branch, else main)
  -h, --help       show this help

Environment:
  TRITONOS_REPO_URL      repository URL
  TRITONOS_DIR           repository path
  TRITONOS_BRANCH        branch name
  TRITON_GITHUB_TOKEN    optional GitHub token to store for private repos
  TRITON_GITHUB_USER     GitHub username for that token
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-apt)
      DO_APT=1
      shift
      ;;
    --fsck)
      DO_FSCK=1
      shift
      ;;
    --dir)
      [[ $# -ge 2 ]] || die "--dir requires a path"
      DEST_DIR="$2"
      shift 2
      ;;
    --branch)
      [[ $# -ge 2 ]] || die "--branch requires a branch name"
      BRANCH="$2"
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

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SCRIPT_REPO="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
if [[ -z "$DEST_DIR" ]]; then
  if [[ -d "${SCRIPT_REPO}/.git" ]]; then
    DEST_DIR="$SCRIPT_REPO"
  else
    DEST_DIR="/home/TritonOS"
  fi
fi

if [[ -z "$BRANCH" && -d "${DEST_DIR}/.git" ]]; then
  BRANCH="$(git -C "$DEST_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
  [[ "$BRANCH" == "HEAD" ]] && BRANCH=""
fi
BRANCH="${BRANCH:-main}"

TARGET_USER="${SUDO_USER:-${USER:-triton}}"
if [[ "$TARGET_USER" == "root" && -d "$DEST_DIR/.git" ]]; then
  owner_uid="$(stat -c '%u' "$DEST_DIR" 2>/dev/null || true)"
  if [[ -n "$owner_uid" && "$owner_uid" != "0" ]]; then
    TARGET_USER="$(getent passwd "$owner_uid" | cut -d: -f1)"
  fi
fi
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
[[ -n "$TARGET_HOME" ]] || die "could not resolve home directory for $TARGET_USER"

run_as_target() {
  if [[ "${EUID:-$(id -u)}" -eq 0 && "$TARGET_USER" != "root" ]]; then
    sudo -H -u "$TARGET_USER" env HOME="$TARGET_HOME" "$@"
  else
    "$@"
  fi
}

APT_OPTS=(
  -o Acquire::ForceIPv4=true
  -o Acquire::Retries=1
  -o Acquire::http::Timeout=8
  -o Acquire::https::Timeout=8
  -o DPkg::Lock::Timeout=20
)

apt_update_quick() {
  [[ "${EUID:-$(id -u)}" -eq 0 ]] || die "apt work needs root; rerun with sudo or omit --with-apt"
  log "Refreshing apt indexes with short IPv4-only timeouts..."
  if ! DEBIAN_FRONTEND=noninteractive apt-get "${APT_OPTS[@]}" update; then
    warn "apt update failed. Continuing only if required tools are already installed."
    return 1
  fi
}

apt_install_base() {
  [[ "${EUID:-$(id -u)}" -eq 0 ]] || die "apt install needs root; rerun with sudo"
  DEBIAN_FRONTEND=noninteractive apt-get "${APT_OPTS[@]}" install -y --no-install-recommends \
    git ca-certificates curl
}

missing_base_tools=0
for cmd in git curl; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    missing_base_tools=1
  fi
done

if [[ "$DO_APT" -eq 1 || "$missing_base_tools" -eq 1 ]]; then
  if apt_update_quick && apt_install_base; then
    log "Base OS tools are installed."
  elif [[ "$missing_base_tools" -eq 1 ]]; then
    die "missing git/curl and apt could not install them; check internet or package sources"
  else
    warn "Skipping apt install because apt refresh failed and required tools already exist."
  fi
fi

TOKEN="${TRITON_GITHUB_TOKEN:-${GITHUB_TOKEN:-}}"
if [[ -n "$TOKEN" ]]; then
  log "Storing GitHub credential from environment for $TARGET_USER."
  run_as_target git config --global credential.helper store
  printf "protocol=https\nhost=github.com\nusername=%s\npassword=%s\n\n" \
    "${TRITON_GITHUB_USER:-ThomasGust}" "$TOKEN" | run_as_target git credential approve >/dev/null
fi

TARGET_GROUP="$(id -gn "$TARGET_USER" 2>/dev/null || printf '%s' "$TARGET_USER")"
TARGET_CRED_FILE="${TARGET_HOME}/.git-credentials"
ROOT_CRED_FILE="/root/.git-credentials"

target_has_github_credential() {
  [[ -r "$TARGET_CRED_FILE" ]] && grep -q 'github\.com' "$TARGET_CRED_FILE"
}

migrate_root_github_credential_if_needed() {
  [[ "${EUID:-$(id -u)}" -eq 0 ]] || return 0
  [[ "$TARGET_USER" != "root" ]] || return 0
  [[ -r "$ROOT_CRED_FILE" ]] || return 0
  grep -q 'github\.com' "$ROOT_CRED_FILE" || return 0
  if target_has_github_credential; then
    return 0
  fi

  log "Found an existing root GitHub credential; copying it to $TARGET_USER for future non-root git pulls."
  install -m 0600 -o "$TARGET_USER" -g "$TARGET_GROUP" /dev/null "$TARGET_CRED_FILE"
  grep 'github\.com' "$ROOT_CRED_FILE" >>"$TARGET_CRED_FILE"
  chown "$TARGET_USER:$TARGET_GROUP" "$TARGET_CRED_FILE"
  chmod 0600 "$TARGET_CRED_FILE"
  run_as_target git config --global credential.helper store
}

migrate_root_github_credential_if_needed

github_network_ready() {
  if [[ "$REPO_URL" != *github.com* ]]; then
    return 0
  fi
  if timeout 5 getent hosts github.com >/dev/null 2>&1; then
    return 0
  fi
  if timeout 8 curl -4 -I --connect-timeout 4 --max-time 7 https://github.com >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

diagnose_git_failure() {
  cat >&2 <<EOF

[TritonOS update] Git fetch/pull failed.

Most common causes:
  1. The Pi cannot reach github.com on the current network.
  2. The repository is private and $TARGET_USER has no stored GitHub credential.

Quick checks on the Pi:
  ping -c 2 github.com
  git -C "$DEST_DIR" remote -v

If this is a credential issue, set a fresh read-only token once and rerun:
  export TRITON_GITHUB_TOKEN='...'
  sudo -E bash bin/update_code.sh

EOF
}

git_as_target() {
  local rc=0
  if [[ "${EUID:-$(id -u)}" -eq 0 && "$TARGET_USER" != "root" ]]; then
    timeout "$GIT_TIMEOUT_S" sudo -H -u "$TARGET_USER" env HOME="$TARGET_HOME" git "$@" || rc=$?
  else
    timeout "$GIT_TIMEOUT_S" git "$@" || rc=$?
  fi
  if [[ "$rc" -ne 0 ]]; then
    diagnose_git_failure
    return "$rc"
  fi
}

if ! github_network_ready; then
  cat >&2 <<EOF
[TritonOS update] Cannot reach github.com from this Pi right now.

SSH from the laptop works, but the Pi does not currently have working internet
DNS/routing for GitHub. Connect the Pi to an internet-capable network, or copy
files over SSH/SCP from the laptop for bench testing.

EOF
  exit 2
fi

if [[ -d "$DEST_DIR/.git" ]]; then
  log "Updating existing checkout: $DEST_DIR"
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    chown -R "$TARGET_USER:$TARGET_USER" "$DEST_DIR"
  fi

  if [[ "$DO_FSCK" -eq 1 ]]; then
    log "Checking git object connectivity..."
    git_as_target -C "$DEST_DIR" fsck --connectivity-only
  fi

  git_as_target -C "$DEST_DIR" remote set-url origin "$REPO_URL"
  git_as_target -C "$DEST_DIR" fetch --prune origin "$BRANCH"
  git_as_target -C "$DEST_DIR" checkout "$BRANCH"
  git_as_target -C "$DEST_DIR" pull --ff-only origin "$BRANCH"
else
  log "Cloning $REPO_URL into $DEST_DIR"
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    mkdir -p "$DEST_DIR"
    chown "$TARGET_USER:$TARGET_USER" "$DEST_DIR"
  fi
  git_as_target clone --branch "$BRANCH" "$REPO_URL" "$DEST_DIR"
fi

if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  chown -R "$TARGET_USER:$TARGET_USER" "$DEST_DIR"
fi

log "Done. Current revision:"
run_as_target git -C "$DEST_DIR" --no-pager log -1 --oneline
