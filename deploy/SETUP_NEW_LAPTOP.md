# Setting up a new pilot / deploy station

How to turn a fresh Windows laptop into a station that can (a) **push code to the
ROV Pi** over the tether and (b) **share its Wi-Fi to the Pi** so the Pi can reach
the internet. Most of this is automated by
[`setup_pilot_station.ps1`](setup_pilot_station.ps1); the manual equivalents and
the reasoning are below so you can do it by hand or debug it.

---

## Architecture

```
         Wi-Fi (internet, DHCP)                 wired USB-Ethernet tether
  Internet ── router ──   [ LAPTOP ]  ===========================  [ Raspberry Pi ]
                          Wi-Fi: dynamic         192.168.1.1/24      eth0 192.168.1.4/24
                          ICS NAT  ◄─────────────  shares ─────►      gateway 192.168.1.1
                                                                      DNS 8.8.8.8 / 1.1.1.1
```

- **Tether** — a USB-Ethernet adapter on the laptop, static **192.168.1.1/24**,
  cabled to the Pi. The Pi is static **192.168.1.4** with the laptop as its gateway.
- **Deploy** — `deploy_to_pi.ps1` git-bundles the laptop's committed `main` and
  fast-forwards the Pi's `/home/TritonOS` checkout to it (backing the Pi up first).
- **Internet for the Pi** — Windows **ICS** NATs the Pi's traffic out the Wi-Fi.

The laptop needs internet (Wi-Fi); the Pi never does for deploys — only the laptop
must be online to `git pull`, then it transfers over the wire.

---

## Prerequisites

- Windows 10/11, an **Administrator** PowerShell.
- **OpenSSH client** (ships with Windows 10 1809+). If `ssh` is missing:
  `Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0`
- **Git** for Windows.
- The USB-Ethernet tether adapter + cable; the Pi powered and tethered.
- A Pi already configured as a static tether endpoint (see
  [Pi-side config](#pi-side-config-reference)). The current ROV Pi already is.

> PuTTY is **not** required — we use OpenSSH end to end. (plink hangs on its
> host-key prompt when run non-interactively, which is why the old approach was
> brittle.)

---

## Quick start (automated)

```powershell
# 1) get the code (needs laptop internet)
git clone https://github.com/ThomasGust/TritonOS.git
cd TritonOS

# 2) preview — detects adapters, shows current state, changes nothing
.\deploy\setup_pilot_station.ps1 -DetectOnly

# 3) apply (elevated). Prompts once for the Pi password to install the key,
#    and confirms before changing the adapter IP / enabling ICS.
.\deploy\setup_pilot_station.ps1

# 4) push code
.\deploy\deploy_to_pi.ps1
```

Useful switches: `-SkipIcs` (deploy-only station), `-DisableIcs` (undo ICS),
`-Force` (no prompts), `-TetherAdapter "<name>"` / `-PublicAdapter "<name>"` if
auto-detection is ambiguous, `-PiHost` / `-TetherIp` to change the addressing.

Adapters are chosen by **role**, not name: the adapter holding the default route
is "internet"; the single connected wired adapter that isn't it is the "tether".

---

## What it does, and the manual equivalent

### 1. Tether static IP
The Pi expects its gateway at `192.168.1.1`, so the laptop's tether adapter must
own that address. Manual:

```powershell
$ifx = (Get-NetAdapter -Name '<tether adapter>').ifIndex
Set-NetIPInterface -InterfaceIndex $ifx -Dhcp Disabled
New-NetIPAddress -InterfaceIndex $ifx -IPAddress 192.168.1.1 -PrefixLength 24
# no default gateway on the tether -- the internet comes from Wi-Fi
```

### 2. Passwordless SSH (key + alias + pinned KEX)
The Pi runs **OpenSSH 10**, whose default post-quantum key exchange the bundled
Windows OpenSSH client can't negotiate (`unsupported KEX method
sntrup761x25519-sha512`). We pin `curve25519-sha256` to sidestep it. Manual:

```powershell
# key (passphrase-less)
ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519_tritonpi -N '""' -C tritonpi-deploy
# trust the host key (forced KEX; this attempt fails auth -- that's fine, it caches the key)
ssh -o KexAlgorithms=curve25519-sha256 -o StrictHostKeyChecking=accept-new -o BatchMode=yes triton@192.168.1.4 exit
# install the public key (enter the Pi password once)
type $env:USERPROFILE\.ssh\id_ed25519_tritonpi.pub | ssh -o KexAlgorithms=curve25519-sha256 triton@192.168.1.4 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

Then add this block to `~/.ssh/config` (`%USERPROFILE%\.ssh\config`):

```
Host tritonpi
    HostName 192.168.1.4
    User triton
    IdentityFile ~/.ssh/id_ed25519_tritonpi
    IdentitiesOnly yes
    KexAlgorithms curve25519-sha256,curve25519-sha256@libssh.org
    StrictHostKeyChecking accept-new
```

Verify: `ssh -o BatchMode=yes tritonpi "echo ok; hostname"` (must not prompt).

### 3. Internet Connection Sharing (ICS)
We use ICS, **not** `New-NetNat`/WinNAT — on some Windows images the `MSFT_NetNat`
WMI class is unregistered and `New-NetNat` throws *"Invalid class"* (that was the
case on the analysis laptop). ICS runs on the separate SharedAccess service and
works. The scope is pinned so it keeps the `192.168.1.x` tether instead of the
default `192.168.137.x`. Manual:

```powershell
$rk='HKLM:\SYSTEM\CurrentControlSet\Services\SharedAccess\Parameters'
New-ItemProperty $rk -Name ScopeAddress       -Value 192.168.1.1 -PropertyType String -Force
New-ItemProperty $rk -Name ScopeAddressBackup -Value 192.168.1.1 -PropertyType String -Force
# enable via COM: share Wi-Fi (public, type 0) -> tether (private, type 1)
$m = New-Object -ComObject HNetCfg.HNetShare
# ...match Wi-Fi and the tether by GUID, then EnableSharing(0)/EnableSharing(1)...  (the script does this)
Set-Service SharedAccess -StartupType Automatic   # survive reboot
```

---

## Verify

```powershell
ssh tritonpi "hostname; ip -4 -br addr"                 # tether + key auth
ssh tritonpi "ping -c2 8.8.8.8 && ping -c2 github.com"  # Pi internet via ICS
.\deploy\deploy_to_pi.ps1 -NoPull                       # push current HEAD
```

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `unsupported KEX method sntrup761x25519-sha512` | Old Windows OpenSSH vs Pi's OpenSSH 10. The `KexAlgorithms curve25519-sha256` pin fixes it; or update OpenSSH. |
| plink hangs / "Host does not exist" | Use OpenSSH (`ssh tritonpi`), not plink. mDNS `tritonpi.local` can drop — use the IP `192.168.1.4`. |
| `New-NetNat : Invalid class` | `MSFT_NetNat` WMI class missing on this image. Don't fight it — use ICS (this setup does). |
| ICS renumbered tether to `192.168.137.1` | The `ScopeAddress` pin didn't apply. Set both `ScopeAddress`/`ScopeAddressBackup` = `192.168.1.1`, then re-run (disable + re-enable ICS). |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` (Pi reimaged) | `ssh-keygen -R 192.168.1.4` then reconnect to re-trust. |
| Pi unreachable | Tether adapter `Up`? `arp -a` shows `192.168.1.4`? Tether IP `192.168.1.1`? Pi powered? |
| "Multiple wired adapters are up" | Pass `-TetherAdapter "<name>"` (see `Get-NetAdapter`). |
| key auth still asks for a password | The key has a passphrase. Regenerate passphrase-less, or strip it: `ssh-keygen -p -f <key>`. |

---

## Pi-side config (reference)

The Pi needs no per-laptop change — its NetworkManager profile is static:

```bash
nmcli -g ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.dns connection show 'Wired connection 1'
# manual | 192.168.1.4/24 | 192.168.1.1 | 8.8.8.8,1.1.1.1
```

To configure a **fresh** Pi the same way:

```bash
sudo nmcli con mod 'Wired connection 1' \
  ipv4.method manual ipv4.addresses 192.168.1.4/24 \
  ipv4.gateway 192.168.1.1 ipv4.dns "8.8.8.8 1.1.1.1"
sudo nmcli con up 'Wired connection 1'
```

---

## Undo / move to another laptop

- **Disable ICS:** `.\deploy\setup_pilot_station.ps1 -DisableIcs`
- **Tether back to DHCP:** `Set-NetIPInterface -InterfaceIndex <ifx> -Dhcp Enabled`
  then `Remove-NetIPAddress -InterfaceIndex <ifx> -IPAddress 192.168.1.1`
- **SSH:** delete the `Host tritonpi` block from `~/.ssh/config` and remove the key
  pair if unwanted; on the Pi, drop the matching line from `~/.ssh/authorized_keys`.

Only **one** laptop should share to the Pi at a time (the Pi has a single static
gateway). Run the setup on whichever laptop is the active station.

---

## Files in `deploy/`

| File | Role |
|---|---|
| `setup_pilot_station.ps1` | One-time station bootstrap (tether IP, SSH, ICS). Idempotent; `-DetectOnly` to preview. |
| `deploy_to_pi.ps1` | Push the laptop's committed `main` to the Pi (`-NoPull`, `-Restart`). |
| `pi_apply.sh` | Runs on the Pi: back up, then fast-forward to the pushed HEAD. |
| `README.md` | Day-to-day deploy usage + rollback. |
| `SETUP_NEW_LAPTOP.md` | This document. |
