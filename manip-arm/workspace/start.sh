#!/usr/bin/env bash
# =============================================================================
# This arm's ROS agent, autostarted. The container's `command:`.
#
# After `docker compose up -d` the arm's contract topics are live:
#   subscribes  <ARM_NS>/cmd_pose, /cmd_gripper, /enable
#   publishes   <ARM_NS>/ee_pose, /joint_states, /active
#
# THIS HOLDS THE ARM'S ONLY CONNECTION. The WXAI controller accepts one driver
# at a time, so while this runs pose.py, read_joints.py, teach.py and gripper.py
# CANNOT reach this arm. That is the hardware's rule and there is no way around
# it -- so autostart is a real tradeoff, not a free convenience.
#
# To use the CLIs instead, either set AUTOSTART=false in docker-compose.yml, or:
#     docker compose stop left-arm && docker compose start left-arm   # cycles it
# or just run the container with AUTOSTART=false for a bare shell.
# =============================================================================
set -euo pipefail

# `set -u` OFF while sourcing ROS, and back on afterwards.
#
# /opt/ros/humble/setup.bash reads AMENT_TRACE_SETUP_FILES and several other
# variables without defaulting them, so under `set -u` it aborts on line 8 and
# takes this script -- and therefore the container -- with it. The failure is
# one line of output and an exited container, which reads as "the arm is
# unreachable" rather than "the shell options are wrong".
#
# ../../slate-base and ../../middle-arm do not hit this only by luck: their
# images set $BASH_ENV, so ros-env.sh has already run by the time start.sh
# begins and its idempotence guard makes the second source a no-op that never
# reaches setup.bash. This image has no ros-env.sh, so it sources for real.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "${DESC_WS:-$HOME/ros2_ws}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${DESC_WS:-$HOME/ros2_ws}/install/setup.bash"
fi
set -u

echo "=== ${ARM_NAME:-arm} starting ==="
echo "  ip ${ARM_IP:-unset}  ns ${ARM_NS:-unset}"
echo "  NOTE: this holds the arm's only connection; the CLIs cannot run"
echo "        against this arm until it exits."
echo

# The arm may be powered off, or the host NIC may have no address on the arm
# subnet. Either way arm_agent exits, and a container that exits takes the whole
# `make` down with it and then restart-loops so the reason scrolls past. Check
# first, and idle with an explanation instead.
if ! ping -c1 -W2 "${ARM_IP}" >/dev/null 2>&1; then
  echo "=================================================================="
  echo "  ${ARM_NAME:-arm}: no reply from ${ARM_IP}."
  echo
  echo "  Either the arm is powered off, or the host NIC on the arm subnet"
  echo "  is down. That second one is the common case and is a HOST setting"
  echo "  -- Docker inherits it and cannot fix it:"
  echo "      ip -br addr             # find the wired NIC, note its name"
  echo "      sudo nmcli con add type ethernet ifname <NIC> \\"
  echo "        con-name trossen-arm ipv4.method manual \\"
  echo "        ipv4.addresses 192.168.1.1/24"
  echo "      sudo nmcli con up trossen-arm"
  echo
  echo "  This container is UP but idle -- it publishes nothing."
  echo "  Then:  make restart SVC=$(echo "${ARM_NAME:-arm}" | tr _ -)"
  echo "=================================================================="
  exec sleep infinity
fi

# --clear-error on every start. The controller latches a fault and refuses
# commands until it is cleared, and clearing happens at connect time -- so
# without this a restart (including one triggered by <ns>/reset) reconnects
# straight back into the same latched error and looks like the reset did
# nothing. The error is still printed before it is cleared.
exec ./arm_agent.py --clear-error "$@"
