#!/usr/bin/env bash
# =============================================================================
# Stop USB autosuspend from dropping the arms' Ethernet adapter and the SLATE.
#
#   sudo ./tools/usb-power-fix.sh            # apply now (until reboot)
#   sudo ./tools/usb-power-fix.sh --persist  # also install the udev rule
#
# WHAT THIS IS FOR
# ----------------
# The Realtek USB-Ethernet adapter that carries BOTH arms has dropped off the
# bus repeatedly. The kernel log for the last one:
#
#   r8152 4-1.2:1.0 enx00e04c68187c: NETDEV WATCHDOG: transmit queue 0 timed
#                                    out 5447 ms
#   r8152 ...: Tx timeout
#   usb 4-1: USB disconnect, device number 2      <- the hub went too
#   r8152-cfgselector 4-1.2: USB disconnect
#
# The transmit queue hung for five and a half seconds WHILE the arms were
# streaming, then the adapter and the hub above it both left the bus. Both arms
# lost their controller in the same instant, which is what "the arms stopped
# responding" actually was -- one adapter, two arms.
#
# The r8152 driver has a long history of exactly this under autosuspend, and on
# this machine every hub is eligible for it:
#
#   1-1      USB2.1 Hub   control=auto   autosuspend_delay_ms=0
#   1-1.1    USB Hub      control=auto   autosuspend_delay_ms=0
#
# A zero delay means "suspend as soon as idle". A suspended hub can take its
# whole subtree with it. The SLATE's serial adapter was pinned to control=on
# earlier and has survived; nothing pinned the hubs, and the hub chains have
# re-enumerated repeatedly today.
#
# THIS IS A STRONG SUSPECT, NOT A PROVEN CAUSE. The adapter is not on the bus
# now, so its own power settings cannot be read back. Marginal power delivery
# and a failing dongle produce the same log. What makes autosuspend worth ruling
# out first is that it is free to rule out.
# =============================================================================
set -uo pipefail

PERSIST=false
[ "${1:-}" = "--persist" ] && PERSIST=true

if [ "$(id -u)" -ne 0 ]; then
  echo "needs root:  sudo $0 ${1:-}" >&2
  exit 1
fi

echo "  pinning every USB hub and network device to control=on ..."
changed=0
for d in /sys/bus/usb/devices/*/; do
  [ -f "$d/power/control" ] || continue
  cls=$(cat "$d/bDeviceClass" 2>/dev/null || echo "")
  prod=$(cat "$d/product" 2>/dev/null || echo "?")
  # 09 is the USB hub class. Network adapters advertise their class per
  # interface rather than per device, so those are matched by driver below.
  is_net=false
  for i in "$d"*:*/driver; do
    case "$(basename "$(readlink -f "$i" 2>/dev/null)" 2>/dev/null)" in
      r8152|r8153|cdc_ncm|cdc_ether|ax88179_178a|asix) is_net=true ;;
    esac
  done
  if [ "$cls" = "09" ] || [ "$is_net" = true ]; then
    cur=$(cat "$d/power/control" 2>/dev/null)
    if [ "$cur" != "on" ]; then
      echo on > "$d/power/control" 2>/dev/null && {
        echo "    $(basename "$d")  ${prod}  auto -> on"
        changed=$((changed + 1))
      }
    fi
  fi
done
echo "  changed ${changed} device(s)."

if [ "${PERSIST}" = true ]; then
  rule=/etc/udev/rules.d/99-rig-usb-power.rules
  cat > "$rule" <<'RULE'
# Keep the rig's USB devices out of autosuspend.
#
# The arms' Ethernet adapter drops off the bus under load with an r8152 Tx
# timeout, taking both arms with it. Autosuspend is the first thing to rule out,
# and these rules make the setting survive a reboot and a re-plug -- which
# matters, because a device that has just re-enumerated comes back with the
# default power settings, i.e. exactly the state it died in.

# Every USB hub.
ACTION=="add", SUBSYSTEM=="usb", ATTR{bDeviceClass}=="09", TEST=="power/control", ATTR{power/control}="on"

# Realtek USB Ethernet (r8152/r8153 family) -- the arms' link.
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", TEST=="power/control", ATTR{power/control}="on"

# QinHeng CH340 -- the SLATE base's serial adapter.
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", TEST=="power/control", ATTR{power/control}="on"
RULE
  udevadm control --reload-rules && udevadm trigger --subsystem-match=usb
  echo "  installed ${rule} and reloaded udev."
  echo "  For a belt-and-braces fix, disable autosuspend kernel-wide by adding"
  echo "      usbcore.autosuspend=-1"
  echo "  to GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub, then update-grub."
fi
