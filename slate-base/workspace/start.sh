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
  echo "  stopped."
}
trap shutdown EXIT INT TERM

echo "=== slate-base starting ==="

if [ "${SLATE_SIM:-false}" = true ]; then
  echo "  *** SIMULATED BASE -- no hardware, no serial port ***"
  # Torqued at start, unlike the real base. In sim the torque gate has already
  # been demonstrated once and then it is just an obstacle between you and the
  # thing you are testing; on hardware it stays a deliberate step.
  ./sim_base.py --torque-on > "${LOG_DIR}/driver.log" 2>&1 &
else
  ./launch-base.sh > "${LOG_DIR}/driver.log" 2>&1 &
fi
PIDS+=($!)
echo "  driver started, logging to ${LOG_DIR}/driver.log"

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

wait -n
