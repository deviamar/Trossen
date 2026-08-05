#!/usr/bin/env bash
# =============================================================================
# Per-machine host setup for the middle-arm container.
#
# Everything else about this container is portable -- this script is the
# irreducible remainder that Docker cannot carry between computers:
#   1. udev rules      (kernel-level device naming; containers only see nodes)
#   2. NVIDIA toolkit  (wires the host GPU driver into the container runtime)
#   3. X11 permission  (per login session, not persistent)
#
# Idempotent: safe to re-run.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

info()  { printf '\033[0;32m==>\033[0m %s\n' "$1"; }
warn()  { printf '\033[0;33m[!]\033[0m %s\n' "$1"; }

# ---- 1. Interbotix udev rules -----------------------------------------------
info "Installing Interbotix udev rules"
sudo cp "${SCRIPT_DIR}/99-interbotix-udev.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# `trigger` only queues the events -- without `settle` the check below races udev
# and reports "not present" for a symlink that appears a moment later.
sudo udevadm settle

if [ -e /dev/ttyDXL ]; then
  info "/dev/ttyDXL present: $(readlink -f /dev/ttyDXL)"
else
  warn "/dev/ttyDXL not present. Expected if the U2D2 is unplugged or the arm is"
  warn "powered off -- plug it in and re-check with: ls -l /dev/ttyDXL"
fi

# ---- 2. NVIDIA Container Toolkit --------------------------------------------
# Needed for the GPU reservation in docker-compose.yml. The ZED SDK is CUDA-only;
# without a GPU in the container the wrapper will not start at all.
if docker info 2>/dev/null | grep -q nvidia; then
  info "NVIDIA container runtime already registered with Docker"
else
  info "Installing NVIDIA Container Toolkit"
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
  # Scoped to the NVIDIA list alone, deliberately. A plain `apt-get update` exits
  # non-zero if ANY configured mirror is unhealthy -- and under `set -e` that
  # aborts this script over a repo we do not even need. That is not theoretical:
  # it happened here, on br.archive.ubuntu.com serving a half-synced dep11
  # (appstream metadata) index, which has nothing to do with the toolkit.
  # Restricting the refresh keeps unrelated mirror breakage out of the way; the
  # install below still resolves deps from the other repos' cached lists.
  sudo apt-get update \
    -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/nvidia-container-toolkit.list \
    -o Dir::Etc::sourceparts=/dev/null \
    -o APT::Get::List-Cleanup=0
  sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  info "NVIDIA Container Toolkit installed"
fi

info "Verifying GPU passthrough"
if docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L; then
  info "GPU visible inside containers"
else
  warn "GPU passthrough check failed -- resolve before building"
fi

# ---- 3. X11 ------------------------------------------------------------------
# Not persistent: this must be re-run after every logout, so it is also worth
# putting in ~/.bashrc if you use RViz2 regularly.
info "Granting local containers access to the X display"
xhost +local:docker || warn "xhost failed (no X session?)"

info "Host setup complete. Next: cd .. && export UID GID && docker compose build"
