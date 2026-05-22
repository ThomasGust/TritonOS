# Reinstall Notes

Use the maintained recovery instructions in [docs/SETUP.md](docs/SETUP.md).

Common repair paths:

```bash
cd /home/TritonOS
sudo bash bin/install_configure.sh --project-dir /home/TritonOS --recreate-venv
```

If the checkout is unusable, preserve it before recloning so calibration files
can be recovered:

```bash
cd /home
sudo mv TritonOS TritonOS.broken.$(date +%Y%m%d-%H%M%S)
sudo git clone https://github.com/ThomasGust/TritonOS.git /home/TritonOS
cd /home/TritonOS
sudo bash bin/install_configure.sh --project-dir /home/TritonOS
```
