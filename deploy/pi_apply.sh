#!/usr/bin/env bash
# Runs ON THE PI, invoked by deploy_to_pi.ps1. Backs up the repo, then syncs it
# to the bundled HEAD pushed from the laptop. Args (positional):
#   $1 REPO    path to the TritonOS checkout on the Pi
#   $2 BUNDLE  path to the git bundle that was scp'd over
#   $3 BR      branch name inside the bundle (e.g. main)
#   $4 TS      timestamp tag for backup filenames
set -eu

REPO="$1"; BUNDLE="$2"; BR="$3"; TS="$4"
BAK="$HOME/triton-deploy-backups"
mkdir -p "$BAK"

OLD=$(git -C "$REPO" rev-parse --short HEAD)

# Full, cheap, recoverable backup BEFORE we touch anything:
#  - bundle of all refs (every commit the Pi has, incl. any local divergence)
#  - patch of uncommitted tracked changes
#  - the porcelain status (records untracked files too)
git -C "$REPO" bundle create "$BAK/$TS-pre.bundle" --all >/dev/null 2>&1 || true
git -C "$REPO" diff HEAD > "$BAK/$TS-pre-uncommitted.patch" 2>/dev/null || true
git -C "$REPO" status --porcelain > "$BAK/$TS-pre-status.txt" 2>/dev/null || true

git -C "$REPO" fetch -q "$BUNDLE" "$BR"
NEW=$(git -C "$REPO" rev-parse --short FETCH_HEAD)

if git -C "$REPO" merge-base --is-ancestor HEAD FETCH_HEAD; then
  MODE=fast-forward
else
  MODE=reset-over-divergence   # backup above preserves the diverged state
fi

# "Sync everything": make the working tree exactly match the laptop HEAD.
git -C "$REPO" reset --hard FETCH_HEAD >/dev/null

rm -f "$BUNDLE"
echo "RESULT old=$OLD new=$NEW mode=$MODE backup=$BAK/$TS-pre.bundle"
