Run this stuff:
ssh-keygen -R tritonpi.local

scp bin/install_configure.sh triton@tritonpi.local:/tmp/install_configure.sh

ssh triton@tritonpi.local "chmod +x /tmp/install_configure.sh && sudo /tmp/install_configure.sh"