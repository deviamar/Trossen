#!/usr/bin/env bash
# =============================================================================
# Per-machine host setup for the slate-base container.
#
# Everything else about this container is portable -- this script is the
# irreducible remainder that Docker cannot carry between computers:
#   1. udev rules   (kernel-level device naming; containers only see nodes)
#   2. brltty       (a host package that steals the base's USB serial port)
#   3. dialout      (host group membership, for driving the base outside Docker)
#
# No NVIDIA step here, unlike ../../middle-arm/host-setup/setup-host.sh -- the
# base has no camera and needs no GPU.
#
# Idempotent: safe to re-run.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

info() { printf '\033[0;32m==>\033[0m %s\n' "$1"; }
warn() { printf '\033[0;33m[!]\033[0m %s\n' "$1"; }

# ---- 1. brltty ---------------------------------------------------------------
# Do this BEFORE installing the rules and triggering udev. brltty's own rule
# claims the CH340 (1a86:7523) as a braille display and takes the port away
# within a second or so of it enumerating -- so with brltty still installed, the
# /dev/ttySLATE check below would race it and report a symlink that is about to
# disappear.
if dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q "^install ok installed$"; then
  warn "brltty is installed. It claims the base's USB-serial chip (1a86:7523)"
  warn "as a braille display and will take the port away seconds after plug-in."
  warn "Trossen's setup instructions say to remove it."
  read -r -p "    Remove brltty now? [y/N] " reply
  if [[ "${reply}" =~ ^[Yy]$ ]]; then
    sudo apt-get remove -y brltty
    info "brltty removed"
  else
    warn "Left installed -- expect /dev/ttyUSB* to appear and then vanish."
  fi
else
  info "brltty not installed (good)"
fi

# ---- 2. udev rules -----------------------------------------------------------
info "Installing SLATE udev rules"
sudo cp "${SCRIPT_DIR}/99-trossen-slate.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# `trigger` only queues the events -- without `settle` the check below races udev
# and reports "not present" for a symlink that appears a moment later.
sudo udevadm settle

if [ -e /dev/ttySLATE ]; then
  info "/dev/ttySLATE present: $(readlink -f /dev/ttySLATE)"
else
  warn "/dev/ttySLATE not present. Expected if the base is unplugged or powered"
  warn "off -- connect it and re-check with: ls -l /dev/ttySLATE"
  if lsusb 2>/dev/null | grep -qi '1a86:7523'; then
    warn "...though lsusb DOES show 1a86:7523, so the device is connected and"
    warn "the rule did not take. Check: udevadm test \$(udevadm info -q path -n /dev/ttyUSB0)"
  fi
fi

# ---- 3. dialout --------------------------------------------------------------
# Not needed for the container (the rule above sets the node 0666, and the image
# puts its user in dialout anyway). It is needed to run the driver or any serial
# tool directly on the host, which is a normal thing to do while debugging.
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  info "$USER is already in the dialout group"
else
  info "Adding $USER to the dialout group"
  sudo usermod -aG dialout "$USER"
  warn "Group membership only applies to NEW logins. Log out and back in (or"
  warn "reboot, as Trossen's instructions say) before relying on it."
fi

info "Host setup complete. Next: cd .. && ../setup.sh && docker compose build"
