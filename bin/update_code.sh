#!/usr/bin/env bash
#This is bad practice, the pat in question though is read only to one repository so I don't really care
set -euo pipefail

REPO_URL="https://github.com/ThomasGust/TritonOS.git"
DEST_DIR="/home/TritonOS"

# HARD-CODED CREDS (yes, bad practice)
GIT_USER="ThomasGust"
GIT_TOKEN="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"

git fsck --full
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
