#requires -Version 5.1
<#
.SYNOPSIS
  Push the laptop's committed TritonOS to the tethered Raspberry Pi.

.DESCRIPTION
  Transport is the 'tritonpi' SSH alias configured in ~/.ssh/config (key auth +
  a KEX pinned for the Pi's OpenSSH 10 + pinned host key), so this runs
  non-interactively wherever the Pi is tethered. Flow:
    1. verify the Pi is reachable over the tether
    2. (optional) pull latest on the laptop from origin
    3. bundle the laptop's current branch
    4. copy the bundle + apply-script to the Pi
    5. the Pi backs itself up (all refs + uncommitted patch) under
       ~/triton-deploy-backups, then resets to the laptop HEAD
    6. verify the Pi HEAD now matches; optionally restart the ROV service

  "Sync everything": the Pi is made to match the laptop's committed HEAD. Any
  Pi-side divergence is captured in the backup BEFORE the reset, so nothing is
  lost irrecoverably.

.PARAMETER NoPull   Skip the origin pull; deploy the laptop's current HEAD as-is (works offline).
.PARAMETER Restart  Restart tritonos-rov.service on the Pi after a successful sync.
.PARAMETER SshAlias ssh_config Host alias for the Pi (default: tritonpi).
.PARAMETER PiRepo   TritonOS checkout path on the Pi (default: /home/TritonOS).

.EXAMPLE  .\deploy\deploy_to_pi.ps1
.EXAMPLE  .\deploy\deploy_to_pi.ps1 -NoPull -Restart
#>
[CmdletBinding()]
param(
  [switch]$NoPull,
  [switch]$Restart,
  [string]$SshAlias = 'tritonpi',
  [string]$PiRepo   = '/home/TritonOS'
)

$ErrorActionPreference = 'Stop'
function Info($m) { Write-Host $m -ForegroundColor Cyan }
function Warn($m) { Write-Host $m -ForegroundColor Yellow }
function Die ($m) { Write-Host "DEPLOY FAILED: $m" -ForegroundColor Red; exit 1 }

# --- locate the laptop repo (this script lives in <repo>\deploy) -----------
$RepoRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Join-Path $RepoRoot '.git'))) { Die "no git repo at $RepoRoot" }
$applyLocal = Join-Path $PSScriptRoot 'pi_apply.sh'
if (-not (Test-Path $applyLocal)) { Die "missing $applyLocal" }

# --- 1) reachability -------------------------------------------------------
Info "Checking tether to '$SshAlias'..."
& ssh -o BatchMode=yes $SshAlias 'true'
if ($LASTEXITCODE -ne 0) { Die "Pi '$SshAlias' not reachable -- is it plugged in and booted?" }

# --- 2) optional pull on the laptop ---------------------------------------
if (-not $NoPull) {
  Info "Pulling latest on laptop (origin, fast-forward only)..."
  git -C $RepoRoot pull --ff-only --quiet
  if ($LASTEXITCODE -ne 0) { Warn "  pull skipped/failed (offline or non-ff) -- deploying current local HEAD" }
}

# --- 3) describe what we will deploy --------------------------------------
$branch    = (git -C $RepoRoot rev-parse --abbrev-ref HEAD).Trim()
$head      = (git -C $RepoRoot rev-parse HEAD).Trim()
$headShort = (git -C $RepoRoot rev-parse --short HEAD).Trim()
if (git -C $RepoRoot status --porcelain) {
  Warn "  laptop tree is dirty; only the committed HEAD ($headShort) will deploy"
}
Info "Deploying $branch @ $headShort  ->  ${SshAlias}:$PiRepo"

# --- 4) bundle the branch --------------------------------------------------
$ts     = Get-Date -Format 'yyyyMMdd-HHmmss'
$bundle = Join-Path ([System.IO.Path]::GetTempPath()) "triton-$ts.bundle"
git -C $RepoRoot bundle create $bundle $branch | Out-Null
if (-not (Test-Path $bundle)) { Die "git bundle creation failed" }

# --- 5) copy bundle + apply-script to the Pi -------------------------------
# Push-Location so scp sees a colon-free relative filename (a Windows path like
# C:\... would be misread as host:path).
$remoteBundle = "/tmp/triton-$ts.bundle"
$remoteApply  = "/tmp/triton-apply-$ts.sh"
Push-Location (Split-Path $bundle)
try {
  & scp -q (Split-Path $bundle -Leaf) "${SshAlias}:$remoteBundle"
  if ($LASTEXITCODE -ne 0) { throw "scp of bundle failed" }
} finally { Pop-Location }
Push-Location $PSScriptRoot
try {
  & scp -q 'pi_apply.sh' "${SshAlias}:$remoteApply"
  if ($LASTEXITCODE -ne 0) { throw "scp of apply-script failed" }
} finally { Pop-Location }

# --- 6) back up + sync on the Pi (tr strips any CRLF before bash) ----------
Info "Backing up Pi and syncing..."
$result = & ssh $SshAlias "tr -d '\r' < '$remoteApply' | bash -s -- '$PiRepo' '$remoteBundle' '$branch' '$ts'; rm -f '$remoteApply'"
if ($LASTEXITCODE -ne 0) { Die "remote sync failed" }
Write-Host $result -ForegroundColor Green

# --- 7) verify -------------------------------------------------------------
$piHead = (& ssh $SshAlias "git -C '$PiRepo' rev-parse HEAD").Trim()
if ($piHead -ne $head) { Die "Pi HEAD $piHead != laptop $head (sync did not land)" }
Write-Host "OK: Pi now at $headShort (matches laptop)." -ForegroundColor Green

# --- 8) optional restart ---------------------------------------------------
if ($Restart) {
  Info "Restarting tritonos-rov.service..."
  & ssh $SshAlias 'sudo systemctl restart tritonos-rov.service; sleep 1; systemctl is-active tritonos-rov.service'
}

Remove-Item $bundle -ErrorAction SilentlyContinue
