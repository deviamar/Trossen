#!/usr/bin/env bash
# =============================================================================
# Bring up the SLATE mobile base driver.
#
# Wraps `ros2 run interbotix_slate_driver slate_base_node` with the namespace
# and parameters this rig needs. There is no launch file upstream -- the package
# installs one executable and nothing else -- so this script is the launch file.
#
# Usage, from inside the container:
#   ./launch-base.sh                 # driver only, no TF
#   ./launch-base.sh --tf            # also broadcast odom -> base_link
#   ./launch-base.sh --rate 50       # faster control loop (default 20 Hz)
#   ./launch-base.sh --ns /slate2    # a second base on the same graph
#
# NO SIMULATION MODE. ../../middle-arm/workspace/launch-arm.sh has --sim, which
# makes it possible to prove the stack launches with no hardware attached. There
# is no equivalent here: the driver calls init_base() in its constructor, and
# that goes straight to a real serial port. Without a base connected the node
# starts, logs FATAL "Failed to initialize base port", and then keeps running
# with a timer that reads nothing -- it does NOT exit. Treat a node that is up
# but publishing no /odom as exactly that failure.
#
# THE BASE HOLDS NOTHING ON EXIT. Killing this is not like killing the arm
# driver: there is no torque to drop and nothing to fall. The base stops,
# because /cmd_vel stops arriving and the 300 ms deadline expires. If it is
# moving when you Ctrl-C, it coasts for up to 300 ms first -- so stop it before
# stopping the driver, not the other way round.
# =============================================================================
set -euo pipefail

NS="${SLATE_NS:-/slate}"
PUBLISH_TF=false
RATE=20
ODOM_FRAME="odom"
BASE_FRAME="base_link"

while [ $# -gt 0 ]; do
  case "$1" in
    --tf)          PUBLISH_TF=true ;;
    --rate)        RATE="$2"; shift ;;
    --ns)          NS="$2"; shift ;;
    --odom-frame)  ODOM_FRAME="$2"; shift ;;
    --base-frame)  BASE_FRAME="$2"; shift ;;
    -h|--help)     sed -n '2,32p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
  esac
  shift
done

# The driver picks its own port -- it enumerates USB serial devices with libudev
# and matches the CH340's 1a86:7523 itself, so this symlink is not what it opens.
# Checking for it anyway converts the most common failure ("base unplugged or
# powered off") from a FATAL log line inside a node that then sits there doing
# nothing into one clear message before anything starts.
# A SECOND DRIVER IS THE WORST FAILURE MODE THIS CONTAINER HAS, because it does
# not look like a failure. Both nodes are called slate_base and both offer the
# same services, so requests round-robin between them -- and only one of them
# holds the serial port. The other keeps running with a dead file descriptor
# (init_base() fails and the node does NOT exit; see the header above), answers
# roughly half your service calls, and fails every one it answers.
#
# Observed on this rig, and it cost an hour: a driver left over from a closed
# terminal held /dev/ttyUSB0 while the base re-enumerated to ttyUSB1. Torque
# commands "failed" about four times in five, the driver log showed nothing but
# successes, and every symptom pointed at flaky Modbus writes. Nothing was flaky.
# There were two drivers.
#
# `ros2 node list` does warn about the duplicate name, but only if you think to
# look -- so refuse to start instead.
if pgrep -f 'interbotix_slate_driver/slate_base_node' >/dev/null 2>&1; then
  echo "ERROR: a slate_base_node is already running in this container:" >&2
  pgrep -af 'interbotix_slate_driver/slate_base_node' | sed 's/^/       /' >&2
  echo "" >&2
  echo "       Starting a second one gives you two nodes named slate_base" >&2
  echo "       offering the same services, with only one holding the serial" >&2
  echo "       port. Service calls then round-robin and fail about half the" >&2
  echo "       time, for no visible reason." >&2
  echo "" >&2
  echo "       Stop the old one first:" >&2
  echo "         pkill -f interbotix_slate_driver/slate_base_node" >&2
  echo "" >&2
  echo "       A driver started with 'docker compose exec' does NOT die when" >&2
  echo "       you close its terminal -- that is the usual way to end up here." >&2
  exit 1
fi

if [ ! -e /dev/ttySLATE ]; then
  echo "ERROR: /dev/ttySLATE is missing. Either the base is unplugged or" >&2
  echo "       powered off, or the host udev rule was never installed." >&2
  echo "       Host-side check:  ls -l /dev/ttySLATE" >&2
  echo "                         lsusb | grep 1a86:7523" >&2
  echo "       Install rules:    ./host-setup/setup-host.sh   (on the host)" >&2
  echo "" >&2
  echo "       If lsusb DOES show 1a86:7523 but the node keeps vanishing," >&2
  echo "       suspect brltty on the host -- see host-setup/99-trossen-slate.rules." >&2
  exit 1
fi

echo "Starting slate_base in namespace ${NS} at ${RATE} Hz (publish_tf=${PUBLISH_TF})."
echo "Topics will be ${NS}/odom, ${NS}/battery_state, ${NS}/cmd_vel."
echo

# --ros-args -r __ns:= is how the namespace gets applied: the node hardcodes its
# own name ("slate_base") and creates every topic relative to its namespace, and
# it takes no namespace argument of its own.
exec ros2 run interbotix_slate_driver slate_base_node --ros-args \
  -r __ns:="${NS}" \
  -p update_frequency:="${RATE}" \
  -p publish_tf:="${PUBLISH_TF}" \
  -p odom_frame_name:="${ODOM_FRAME}" \
  -p base_frame_name:="${BASE_FRAME}"
