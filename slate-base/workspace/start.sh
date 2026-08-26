#!/usr/bin/env bash
# =============================================================================
# Everything this container provides, in one process tree.
#
# This is the container's `command:` in docker-compose.yml, so after
# `docker compose up -d` the base's topics are live with nothing else to run:
#
#   driver        -> /slate/odom, /slate/battery_state, subscribes /slate/cmd_vel
#   governor      -> owns /slate/cmd_vel; subscribes cmd_vel_teleop, cmd_vel_nav
#   lift_agent    -> /slate/lift/*    (SIMULATED -- no lift hardware)
#
# Set AUTOSTART=false in the environment to get a bare shell instead, which is
# what you want when a script needs the serial port for itself.
#
# SLATE_SIM=true swaps the real driver for sim_base.py: no serial port, no
# hardware, odometry integrated from whatever you command. Everything above the
# driver -- governor, lift, the CLIs, the monitor -- is unchanged and does not
# know the difference. That is the whole point: prove the velocity chain at a
# desk, then flip one variable.
#
# ORDER MATTERS ONCE. The governor publishes to /slate/cmd_vel, which only means
# anything if the driver is subscribed, so the driver goes first and we wait for
# it. Nothing else here has an ordering constraint.
#
# ONE PROCESS GROUP. A SIGTERM from `docker compose down` reaches this script,
# which stops the children in reverse order -- governor before driver, so the
# base is commanded to zero while something is still listening.
# =============================================================================
set -euo pipefail

# shellcheck disable=SC1091
source /usr/local/bin/ros-env.sh

LOG_DIR="${ROS_WS:-$HOME/workspace}"
PIDS=()

shutdown() {
  echo "  stopping ..."
  # Reverse order: the governor publishes a zero Twist on its way out, and that
  # is only useful while the driver is still there to receive it.
  for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
    kill "${PIDS[i]}" 2>/dev/null || true
    wait "${PIDS[i]}" 2>/dev/null || true
  done
  kill "${DRIVER_PID:-}" 2>/dev/null || true
  echo "  stopped."
}
trap shutdown EXIT INT TERM

echo "=== slate-base starting ==="

SLATE_DEV="${SLATE_DEV:-/dev/ttySLATE}"

resolve_dev() {
  # `readlink -f` on a symlink that has momentarily vanished returns the path
  # itself, which looks like a move to a device called /dev/ttySLATE and makes
  # the watchdog restart the driver for nothing. Report `none` unless the link
  # exists AND its target does.
  if [ -e "${SLATE_DEV}" ]; then
    readlink -f "${SLATE_DEV}" 2>/dev/null || echo none
  else
    echo none
  fi
}

start_driver() {
  if [ "${SLATE_SIM:-false}" = true ]; then
    echo "  *** SIMULATED BASE -- no hardware, no serial port ***"
    # Torqued at start, unlike the real base. In sim the torque gate has already
    # been demonstrated once and then it is just an obstacle between you and the
    # thing you are testing; on hardware it stays a deliberate step.
    ./sim_base.py --torque-on > "${LOG_DIR}/driver.log" 2>&1 &
  else
    ./launch-base.sh > "${LOG_DIR}/driver.log" 2>&1 &
  fi
  DRIVER_PID=$!
  DEV_TARGET="$(resolve_dev)"
  echo "  driver started (pid ${DRIVER_PID}, device ${DEV_TARGET}),"
  echo "  logging to ${LOG_DIR}/driver.log"
}

stop_driver() {
  # Killing ${DRIVER_PID} alone is NOT enough, and getting this wrong is how the
  # restart silently fails.
  #
  # launch-base.sh ends in `exec ros2 run ...`, so DRIVER_PID is the `ros2 run`
  # process -- but `ros2 run` SPAWNS the node executable as a child rather than
  # exec'ing it. Kill the parent and the node is orphaned, still running, still
  # holding the serial port. The next driver then starts, finds the port busy,
  # and exits with the "another driver is already running" message, so the
  # watchdog appears to work while the base stays dead.
  #
  # So: kill the wrapper, then the node by name. The driver's own error text
  # recommends exactly this pkill.
  kill "${DRIVER_PID:-}" 2>/dev/null || true
  wait "${DRIVER_PID:-}" 2>/dev/null || true
  pkill -f "interbotix_slate_driver/slate_base_node" 2>/dev/null || true
  pkill -f "sim_base.py" 2>/dev/null || true
  # Give the kernel a moment to release the port before reopening it.
  for _ in $(seq 1 20); do
    pgrep -f "interbotix_slate_driver/slate_base_node" >/dev/null 2>&1 || break
    sleep 0.25
  done
}

# WATCHDOG: restart the driver when the serial device moves underneath it.
#
# The base's USB-serial adapter re-enumerates on this rig -- observed three
# times in an afternoon with nothing physically touched, coming back as
# ttyUSB0, then ttyUSB1, then ttyUSB0 again. The udev symlink follows it, but
# the DRIVER DOES NOT: it holds the file descriptor it opened at startup, which
# is now a dead node. The failure is silent and nasty -- every topic still
# exists, `ros2 topic list` is complete, publishing simply stops and service
# calls return "Failed to enable motor torque" with no reason given.
#
# So: watch what the symlink resolves to, and restart the driver when it
# changes or disappears. Compares the resolved target rather than polling the
# topic, because that distinguishes "the device moved" from "the base is idle",
# and only the first one is fixable from here.
#
# The real fix is upstream of this container: both USB hubs have
# power/control=auto with autosuspend_delay=0, so they suspend the moment they
# go idle and drop what is below them. See README.md.
device_watchdog() {
  [ "${SLATE_SIM:-false}" = true ] && return 0
  while true; do
    sleep 5
    now="$(resolve_dev)"
    if [ "${now}" != "${DEV_TARGET}" ]; then
      echo
      echo "  !! ${SLATE_DEV} moved: ${DEV_TARGET} -> ${now}"
      if [ "${now}" = none ]; then
        echo "  !! device is gone. Waiting for it to come back ..."
        while [ ! -e "${SLATE_DEV}" ]; do sleep 2; done
        echo "  !! device is back."
      fi
      stop_driver
      sleep 1
      start_driver
      echo "  !! driver restarted on the new device."
    fi
  done
}

start_driver
# DRIVER_PID is deliberately NOT in PIDS: the watchdog owns its lifecycle and
# restarts it in place. shutdown() kills it separately.

# Wait for the driver to actually subscribe rather than sleeping a guess. A
# governor that starts first publishes into nothing and the base never moves,
# which looks identical to a dead governor.
echo -n "  waiting for the driver to subscribe to /slate/cmd_vel "
for _ in $(seq 1 40); do
  if ros2 topic info /slate/cmd_vel 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    echo "-- up."
    break
  fi
  echo -n "."
  sleep 0.5
done

if ! ros2 topic info /slate/cmd_vel 2>/dev/null | grep -q "Subscription count: [1-9]"; then
  echo
  echo "  WARNING: the driver never subscribed. It is probably running but not" >&2
  echo "  talking to hardware -- it logs FATAL and keeps going. Check:" >&2
  echo "      cat ${LOG_DIR}/driver.log" >&2
  echo "  Continuing anyway; the rest of the graph will be live." >&2
fi

./governor.py &
PIDS+=($!)
echo "  governor started (owns /slate/cmd_vel)"

./lift_agent.py &
PIDS+=($!)
echo "  lift agent started (SIMULATED)"

echo "=== slate-base ready ==="
if [ "${SLATE_SIM:-false}" = true ]; then
  echo "  SIMULATED. /slate/odom is computed, not measured."
  echo "  Try:  docker compose exec monitor ./drive_test.py --execute"
else
  echo "  the base ignores velocity commands until its motors are torqued:"
  echo "      ./base_ctl.py torque on"
fi
echo

# The watchdog runs in the FOREGROUND and never returns. That is the point: it
# restarts the driver in place, and if this script were sitting on `wait -n`
# instead, the deliberate kill would look like "a child died", the script would
# fall through, and the EXIT trap would tear the container down -- which is
# exactly what happened the first time this was written.
device_watchdog
