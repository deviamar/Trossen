#!/usr/bin/env bash
# =============================================================================
# Bring up the Quest teleop pair: driver + mapping, one process each.
#
#   ./launch-quest.sh                      # udp backend, everything live
#   ./launch-quest.sh --backend sim        # no headset, fake motion
#   ./launch-quest.sh --dry-run            # publish /quest/*, no robot commands
#   ./launch-quest.sh --no-base            # arms only
#   ./launch-quest.sh --scale 0.5          # arms follow at half distance
#   ./launch-quest.sh --driver-only        # raw topics only, no mapping
#
# Two nodes rather than one so the device half can run without the robot half.
# `--driver-only` plus `ros2 topic echo /quest/right/joy` is how you find out
# which button is which, with every robot powered down.
#
# START HERE, IN THIS ORDER, THE FIRST TIME:
#   1. ./launch-quest.sh --backend sim --dry-run   -- nothing can move
#   2. ./launch-quest.sh --backend sim             -- robots move, no headset
#   3. ./launch-quest.sh --backend udp             -- for real
# =============================================================================
set -e

BACKEND=udp
DRIVER_ONLY=false
TELEOP_ARGS=()
DRIVER_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --backend)      BACKEND="$2"; shift 2 ;;
    --port)         DRIVER_ARGS+=(--port "$2"); shift 2 ;;
    --press)        DRIVER_ARGS+=(--press "$2"); shift 2 ;;
    --driver-only)  DRIVER_ONLY=true; shift ;;
    --dry-run)      TELEOP_ARGS+=(--dry-run); shift ;;
    --no-base)      TELEOP_ARGS+=(--no-base); shift ;;
    --scale)        TELEOP_ARGS+=(--scale "$2"); shift 2 ;;
    -h|--help)      sed -n '3,22p' "$0"; exit 0 ;;
    *)              echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# shellcheck disable=SC1091
source /usr/local/bin/ros-env.sh

cleanup() {
  # Kill the mapping node FIRST. It publishes the robot commands, and its own
  # shutdown releases the arms and zeroes the base; killing the driver first
  # would leave it publishing from a headset that has stopped updating.
  [ -n "${TELEOP_PID:-}" ] && kill "${TELEOP_PID}" 2>/dev/null || true
  [ -n "${TELEOP_PID:-}" ] && wait "${TELEOP_PID}" 2>/dev/null || true
  [ -n "${DRIVER_PID:-}" ] && kill "${DRIVER_PID}" 2>/dev/null || true
  echo "  stopped."
}
trap cleanup EXIT INT TERM

./quest_driver.py --backend "${BACKEND}" "${DRIVER_ARGS[@]}" &
DRIVER_PID=$!

if [ "${DRIVER_ONLY}" = true ]; then
  echo "  driver only -- no robot commands will be published."
  wait "${DRIVER_PID}"
  exit 0
fi

# Let the driver bind its socket and publish /quest/connected before the mapping
# node starts looking for it, so the first log line is not a spurious warning.
sleep 1
./quest_teleop.py "${TELEOP_ARGS[@]}" &
TELEOP_PID=$!

wait "${TELEOP_PID}"
