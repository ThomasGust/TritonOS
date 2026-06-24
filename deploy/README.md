# Deploying TritonOS to the Pi over the tether

Reliable, non-interactive code push from the laptop to the ROV Pi, usable
wherever the Pi is tethered (no internet needed on the Pi).

## Usage

From the repo root on the laptop:

```powershell
.\deploy\deploy_to_pi.ps1            # pull latest on laptop, then sync to the Pi
.\deploy\deploy_to_pi.ps1 -NoPull    # sync the laptop's current HEAD as-is (offline ok)
.\deploy\deploy_to_pi.ps1 -Restart   # also restart tritonos-rov after syncing
```

What it does: bundles the laptop's committed `main`, copies it to the Pi, the Pi
**backs itself up** to `~/triton-deploy-backups/` (a bundle of all its refs plus a
patch of any uncommitted changes), then `git reset --hard`s to the laptop HEAD.
The ROV service is **not** restarted unless you pass `-Restart`.

To roll back a deploy on the Pi:

```bash
cd /home/TritonOS
git fetch ~/triton-deploy-backups/<TS>-pre.bundle '*:*'   # restore old refs
git reset --hard <old-short-sha>                          # from the RESULT line
git apply ~/triton-deploy-backups/<TS>-pre-uncommitted.patch   # if needed
```

## One-time setup (already done on this laptop)

The transport is the `tritonpi` host alias in `~/.ssh/config`:

```
Host tritonpi
    HostName 192.168.1.4
    User triton
    IdentityFile ~/.ssh/id_ed25519_tritonpi
    IdentitiesOnly yes
    KexAlgorithms curve25519-sha256,curve25519-sha256@libssh.org
    StrictHostKeyChecking accept-new
```

- **Why the explicit `KexAlgorithms`:** the Pi runs OpenSSH 10, whose default
  post-quantum key exchange the bundled Windows OpenSSH client can't negotiate
  (it errors with `unsupported KEX method sntrup761x25519-sha512`). Pinning
  curve25519 sidesteps that. If you update the laptop's OpenSSH to a current
  build, this line becomes unnecessary.
- **Key auth:** `~/.ssh/id_ed25519_tritonpi` (passphrase-less) is installed in the
  Pi's `~/.ssh/authorized_keys`. No password lives in any script.

To set this up on a **new laptop** (e.g. the pilot station), don't do it by hand —
run the bootstrap, which also configures the tether IP and Pi internet sharing:

```powershell
.\deploy\setup_pilot_station.ps1 -DetectOnly   # preview
.\deploy\setup_pilot_station.ps1               # apply (elevated)
```

See [SETUP_NEW_LAPTOP.md](SETUP_NEW_LAPTOP.md) for the full walkthrough, the manual
equivalents, and troubleshooting.
