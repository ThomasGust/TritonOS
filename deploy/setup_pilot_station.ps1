#requires -Version 5.1
<#
.SYNOPSIS
  One-time bootstrap of a Windows laptop as a TritonOS pilot / deploy station.

.DESCRIPTION
  Reproduces, automatically, the setup that lets a laptop both push code to the
  ROV Pi and share its Wi-Fi to the Pi over the wired tether:

    1. TETHER  - give the wired USB-Ethernet adapter the static IP the Pi expects
                 as its gateway (default 192.168.1.1/24; Pi is static 192.168.1.4).
    2. SSH     - generate a passphrase-less key, install it in the Pi's
                 authorized_keys, and write an ssh_config 'tritonpi' alias with a
                 KEX pinned for the Pi's OpenSSH 10 + accept-new host-key trust.
    3. ICS     - enable Internet Connection Sharing (Wi-Fi -> tether) with the
                 scope pinned so the tether keeps its 192.168.1.x address, and set
                 the SharedAccess service to start automatically.

  Adapters are detected by role (the default-route adapter is "internet"; the one
  connected wired adapter that isn't it is the "tether"), so this works regardless
  of how the adapters are named on a given laptop. Idempotent and safe to re-run.

  Requires an elevated (Administrator) PowerShell, the Windows OpenSSH client
  (ssh/scp/ssh-keygen), and the Pi powered on and tethered.

.PARAMETER DetectOnly  Report what would happen (adapters, current state); make NO changes.
.PARAMETER SkipTether  Don't touch the tether adapter IP.
.PARAMETER SkipSsh     Don't set up the SSH key / alias.
.PARAMETER SkipIcs     Don't enable ICS.
.PARAMETER DisableIcs  Tear ICS down (disable sharing on all connections) and exit.
.PARAMETER Force       Don't prompt before network-changing steps.

.EXAMPLE  .\deploy\setup_pilot_station.ps1 -DetectOnly
.EXAMPLE  .\deploy\setup_pilot_station.ps1
.EXAMPLE  .\deploy\setup_pilot_station.ps1 -SkipIcs        # deploy-only station
.EXAMPLE  .\deploy\setup_pilot_station.ps1 -DisableIcs     # undo ICS
#>
[CmdletBinding()]
param(
  [string]$PiUser        = 'triton',
  [string]$PiHost        = '192.168.1.4',
  [string]$TetherIp      = '192.168.1.1',
  [int]   $TetherPrefix  = 24,
  [string]$TetherAdapter = '',
  [string]$PublicAdapter = '',
  [string]$SshAlias      = 'tritonpi',
  [string]$KeyName       = 'id_ed25519_tritonpi',
  [switch]$SkipTether,
  [switch]$SkipSsh,
  [switch]$SkipIcs,
  [switch]$DisableIcs,
  [switch]$DetectOnly,
  [switch]$Force
)
$ErrorActionPreference = 'Stop'
function Info($m){ Write-Host "[*] $m"  -ForegroundColor Cyan }
function Good($m){ Write-Host "[ok] $m" -ForegroundColor Green }
function Warn($m){ Write-Host "[!] $m"  -ForegroundColor Yellow }
function Die ($m){ Write-Host "[x] $m"  -ForegroundColor Red; exit 1 }
function Ask ($m){ if($Force){return $true}; return ((Read-Host "$m [y/N]") -match '^(y|yes)$') }
function Is-Admin { ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltinRole]::Administrator) }

if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
  Die "OpenSSH client not found. Install it:  Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0"
}
if (-not $DetectOnly -and -not (Is-Admin)) {
  Die "Run this in an ELEVATED PowerShell (Administrator) -- it changes the adapter IP and ICS."
}

# ---------------------------------------------------------------- detection --
function Resolve-InternetAdapter {
  if ($PublicAdapter) { return (Get-NetAdapter -Name $PublicAdapter -ErrorAction Stop) }
  $r = Get-NetRoute -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue |
       Sort-Object RouteMetric | Select-Object -First 1
  if (-not $r) { Die "No default route found -- is this laptop online (Wi-Fi)?" }
  return (Get-NetAdapter -InterfaceIndex $r.ifIndex)
}
function Resolve-TetherAdapter($internetName) {
  if ($TetherAdapter) { return (Get-NetAdapter -Name $TetherAdapter -ErrorAction Stop) }
  $cands = Get-NetAdapter -Physical | Where-Object {
    $_.Status -eq 'Up' -and $_.Name -ne $internetName -and $_.MediaType -ne 'Native 802.11'
  }
  if (@($cands).Count -eq 1) { return $cands[0] }
  if (@($cands).Count -eq 0) { Die "No connected wired tether adapter found. Plug in the Pi, or pass -TetherAdapter." }
  Die ("Multiple wired adapters are up; pass -TetherAdapter <name>. Candidates: " + (($cands.Name) -join ', '))
}

$internet = Resolve-InternetAdapter
$tether   = Resolve-TetherAdapter $internet.Name
Info "Internet (public) adapter : $($internet.Name)  [$($internet.InterfaceDescription)]"
Info "Tether   (private) adapter: $($tether.Name)  [$($tether.InterfaceDescription)]"

$sshDir  = Join-Path $env:USERPROFILE '.ssh'
$keyPath = Join-Path $sshDir $KeyName
$cfgPath = Join-Path $sshDir 'config'

# ---------------------------------------------------------------- ICS teardown
function Disable-Ics {
  $m = New-Object -ComObject HNetCfg.HNetShare
  foreach($c in @($m.EnumEveryConnection)){
    $cfg=$m.INetSharingConfigurationForINetConnection.Invoke($c)
    if($cfg.SharingEnabled){ $cfg.DisableSharing() }
  }
  Good "ICS sharing disabled on all connections."
}
if ($DisableIcs) {
  if ($DetectOnly) { Info "(DetectOnly) would disable ICS sharing"; exit 0 }
  if (-not (Is-Admin)) { Die "elevation required to disable ICS" }
  Disable-Ics; exit 0
}

# ----------------------------------------------------------- DetectOnly report
if ($DetectOnly) {
  $tip = (Get-NetIPAddress -InterfaceIndex $tether.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue).IPAddress
  Info "Tether current IPv4   : $($tip -join ', ')   (want $TetherIp/$TetherPrefix)"
  Info "SSH key present       : $([bool](Test-Path $keyPath))   ($keyPath)"
  $hasAlias = (Test-Path $cfgPath) -and ((Get-Content $cfgPath -Raw) -match "(?im)^[ \t]*Host[ \t]+$([regex]::Escape($SshAlias))\b")
  Info "ssh_config has '$SshAlias': $hasAlias"
  $m = New-Object -ComObject HNetCfg.HNetShare
  $shared = foreach($c in @($m.EnumEveryConnection)){ $p=$m.NetConnectionProps.Invoke($c); $cfg=$m.INetSharingConfigurationForINetConnection.Invoke($c); if($cfg.SharingEnabled){ "{0}(type {1})" -f $p.Name,$cfg.SharingConnectionType } }
  Info "ICS currently sharing : $(@($shared) -join ', ')"
  Write-Host "`nDetectOnly: no changes made." -ForegroundColor Green
  exit 0
}

# ------------------------------------------------------------------- 1) tether
if (-not $SkipTether) {
  $existing = Get-NetIPAddress -InterfaceIndex $tether.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue
  if ($existing.IPAddress -contains $TetherIp) {
    Good "Tether '$($tether.Name)' already has $TetherIp/$TetherPrefix"
  } elseif (Ask "Set '$($tether.Name)' to static $TetherIp/$TetherPrefix (replaces its current IPv4)?") {
    Set-NetIPInterface -InterfaceIndex $tether.ifIndex -Dhcp Disabled -ErrorAction SilentlyContinue
    foreach($a in $existing){ Remove-NetIPAddress -InputObject $a -Confirm:$false -ErrorAction SilentlyContinue }
    Get-NetRoute -InterfaceIndex $tether.ifIndex -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue |
      Remove-NetRoute -Confirm:$false -ErrorAction SilentlyContinue
    New-NetIPAddress -InterfaceIndex $tether.ifIndex -IPAddress $TetherIp -PrefixLength $TetherPrefix -ErrorAction Stop | Out-Null
    Good "Tether set to $TetherIp/$TetherPrefix (no gateway -- internet comes from $($internet.Name))"
  } else { Warn "skipped tether IP" }
}

# ---------------------------------------------------------------------- 2) ssh
if (-not $SkipSsh) {
  if (-not (Test-Path $sshDir)) { New-Item -ItemType Directory -Path $sshDir | Out-Null }
  if (-not (Test-Path $keyPath)) {
    Info "Generating key $keyPath"
    ssh-keygen -t ed25519 -f $keyPath -N '""' -C 'tritonpi-deploy' -q
  } else { Good "key exists: $keyPath" }

  Info "Caching Pi host key (the 'Permission denied' below is EXPECTED -- we only fetch the key)"
  ssh -o KexAlgorithms=curve25519-sha256 -o HostKeyAlgorithms=ssh-ed25519 `
      -o StrictHostKeyChecking=accept-new -o BatchMode=yes -o ConnectTimeout=8 `
      "$PiUser@$PiHost" exit

  $pub = (Get-Content "$keyPath.pub" -Raw).Trim()
  if (-not $pub) { Die "could not read $keyPath.pub" }
  $inst = "umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; " +
          "grep -qxF '$pub' ~/.ssh/authorized_keys || echo '$pub' >> ~/.ssh/authorized_keys; " +
          "chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys; echo INSTALLED_OK"
  Info "Installing public key -- enter the Pi password for '$PiUser' when prompted:"
  ssh -o KexAlgorithms=curve25519-sha256 -o StrictHostKeyChecking=accept-new "$PiUser@$PiHost" $inst

  $block = @"
Host $SshAlias
    HostName $PiHost
    User $PiUser
    IdentityFile ~/.ssh/$KeyName
    IdentitiesOnly yes
    KexAlgorithms curve25519-sha256,curve25519-sha256@libssh.org
    StrictHostKeyChecking accept-new
    ConnectTimeout 8
    ServerAliveInterval 5
    ServerAliveCountMax 3
"@
  if (Test-Path $cfgPath) {
    if ((Get-Content $cfgPath -Raw) -match "(?im)^[ \t]*Host[ \t]+$([regex]::Escape($SshAlias))\b") {
      Good "ssh_config already has Host $SshAlias"
    } else { Add-Content -Path $cfgPath -Value "`n$block"; Good "appended Host $SshAlias to ssh_config" }
  } else { Set-Content -Path $cfgPath -Value $block -Encoding ascii; Good "created ssh_config with Host $SshAlias" }

  Info "Verifying passwordless key auth..."
  $r = ssh -o BatchMode=yes $SshAlias "echo KEYAUTH_OK; hostname"
  if ($LASTEXITCODE -eq 0 -and ($r -join "`n") -match 'KEYAUTH_OK') { Good "key auth works ($($r -join ' '))" }
  else { Die "key auth verification failed -- check the password step above and that the key is passphrase-less" }
}

# ---------------------------------------------------------------------- 3) ICS
if (-not $SkipIcs) {
  if (Ask "Enable Internet Connection Sharing ($($internet.Name) -> $($tether.Name)), scope pinned to $TetherIp?") {
    $rk='HKLM:\SYSTEM\CurrentControlSet\Services\SharedAccess\Parameters'
    New-ItemProperty -Path $rk -Name ScopeAddress       -Value $TetherIp -PropertyType String -Force | Out-Null
    New-ItemProperty -Path $rk -Name ScopeAddressBackup -Value $TetherIp -PropertyType String -Force | Out-Null

    $m = New-Object -ComObject HNetCfg.HNetShare
    $byGuid=@{}
    foreach($c in @($m.EnumEveryConnection)){ $p=$m.NetConnectionProps.Invoke($c); if($p){ $byGuid[$p.Guid.ToUpper()]=$c } }
    foreach($c in @($m.EnumEveryConnection)){ $cfg=$m.INetSharingConfigurationForINetConnection.Invoke($c); if($cfg.SharingEnabled){ $cfg.DisableSharing() } }
    $pubGuid  = $internet.InterfaceGuid.ToString().ToUpper()
    $privGuid = $tether.InterfaceGuid.ToString().ToUpper()
    if (-not $byGuid.ContainsKey($pubGuid))  { Die "internet adapter not found in ICS connection list" }
    if (-not $byGuid.ContainsKey($privGuid)) { Die "tether adapter not found in ICS connection list" }
    $m.INetSharingConfigurationForINetConnection.Invoke($byGuid[$pubGuid]).EnableSharing(0)   # 0 = public
    $m.INetSharingConfigurationForINetConnection.Invoke($byGuid[$privGuid]).EnableSharing(1)  # 1 = private
    Set-Service SharedAccess -StartupType Automatic
    Good "ICS enabled; SharedAccess set to Automatic"

    Start-Sleep -Seconds 3
    $tip = (Get-NetIPAddress -InterfaceIndex $tether.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue).IPAddress
    if ($tip -contains $TetherIp) { Good "tether still $TetherIp after ICS (scope pin held)" }
    else { Warn "tether IP is now '$($tip -join ', ')' (wanted $TetherIp); ICS may have renumbered it -- re-run or set ScopeAddress and retry" }
  } else { Warn "skipped ICS" }
}

# ------------------------------------------------------------- final smoke test
if (-not $SkipSsh) {
  Info "Pi internet check..."
  $net = ssh -o BatchMode=yes $SshAlias "ping -c2 -W3 8.8.8.8 >/dev/null 2>&1 && echo IP_OK; ping -c2 -W3 github.com >/dev/null 2>&1 && echo DNS_OK" 2>$null
  Write-Host ("  " + (($net -join ' ') -replace '\s+',' '))
  if (($net -join ' ') -match 'IP_OK' -and ($net -join ' ') -match 'DNS_OK') { Good "Pi reaches the internet through this laptop" }
  elseif (-not $SkipIcs) { Warn "Pi not online yet -- check ICS, the Pi's gateway ($TetherIp), and Wi-Fi" }
}

Write-Host "`nDone. Deploy code with:  .\deploy\deploy_to_pi.ps1" -ForegroundColor Green
