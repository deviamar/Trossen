#!/usr/bin/env bash
# =============================================================================
# The active-vision arm: driver plus IK agent. The container's `command:`.
#
# After `docker compose up -d`:
#   xs_sdk        -> /middle/joint_states, /middle/commands/joint_group, ...
#   head_agent    -> /middle/cmd_pose, /middle/enable, /middle/ee_pose
#
# TORQUE COMES ON. Unlike `./launch-arm.sh --limp`, this starts the driver in
# its normal torqued mode, because an autostarted arm that cannot be commanded
# is not much use. Set AUTOSTART=false for a bare shell if you want --limp first,
# and do that on any arm you have not brought up before.
#
# THE URDF IS GENERATED HERE, from the same xacro and the same arguments the
# driver is launched with. head_agent's solver describes whatever URDF it is
# given, so generating it beside the driver is the only way to be sure the model
# and the machine agree.
# =============================================================================
set -euo pipefail

# shellcheck disable=SC1091
source /usr/local/bin/ros-env.sh

URDF="${MIDDLE_URDF:-/tmp/middle_arm.urdf}"
PIDS=()

shutdown() {
  echo "  stopping ..."
  for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
    kill "${PIDS[i]}" 2>/dev/null || true
    wait "${PIDS[i]}" 2>/dev/null || true
  done
  echo "  stopped."
}
trap shutdown EXIT INT TERM

# Hardware may simply not be plugged in yet. A container that exits on that is
# the wrong behaviour for `make`: the whole rig fails to come up because one
# subsystem is powered off, and compose then restarts it in a loop that buries
# the reason. Stay up, say exactly what is missing, and be ready to be
# restarted once it is connected.
if [ ! -e "${DXL_PORT:-/dev/ttyDXL}" ]; then
  echo "=================================================================="
  echo "  middle-arm: ${DXL_PORT:-/dev/ttyDXL} is missing."
  echo "  The arm is powered off, the U2D2 is unplugged, or the host udev"
  echo "  rule was never installed."
  echo
  echo "  This container is UP but idle -- it publishes nothing."
  echo "  On the host:   ls -l /dev/ttyDXL"
  echo "                 ./middle-arm/host-setup/setup-host.sh"
  echo "  Then:          make restart SVC=middle-arm"
  echo "=================================================================="
  # Idle rather than exit. `exec sleep infinity` replaces this shell so the
  # container's PID 1 is something that responds to SIGTERM immediately.
  exec sleep infinity
fi

# PREFLIGHT: prove the solver can import BEFORE torquing anything. The failure
# this prevents was real: the driver launched and TORQUED the arm, head_agent
# then died on an import error (stale image), the container exited with it and
# killed the driver -- leaving the motors locked, no /middle topics, and a U2D2
# whose data LED never blinks. Now a broken solver stops the show while the arm
# is still untouched.
if ! err=$(python3 -c "import middle_ik" 2>&1); then
  echo "=================================================================="
  echo "  middle-arm: head_agent's IK stack cannot import -- NOT starting"
  echo "  the driver (it would torque the arm and then die with the agent)."
  echo
  echo "${err}" | tail -4
  echo
  echo "  This is the stale-image failure: rebuild picks up the pinned"
  echo "  pyroki/jaxls fix in the Dockerfile:"
  echo "      docker compose up -d --build middle-arm    (~25 min)"
  echo "=================================================================="
  exec sleep infinity
fi

echo "=== middle-arm starting ==="

echo "  generating URDF -> ${URDF}"
# prep_urdf.py strips the phantom `gripper` joint (the servo is not fitted --
# leaving it in gives pyroki 7 actuated joints against xs_sdk's 6) and grafts
# camera_link onto the flange with the MIDDLE_ZED_* offsets, so head_agent's
# targets mean the camera rather than the flange.
./launch-arm.sh --dump-urdf | ./prep_urdf.py > "${URDF}"

./launch-arm.sh > "${ROS_WS:-$HOME/workspace}/driver.log" 2>&1 &
PIDS+=($!)
echo "  driver started (TORQUED), logging to driver.log"

# Wait on actual DATA, not graph metadata, and bypass the ros2 CLI daemon.
# Every container shares the host network, so `ros2 topic info` talks to
# whichever container's daemon grabbed this domain's port first -- and a stale
# or wedged daemon then reports nothing for a topic that is demonstrably
# publishing (observed: 30 s of dots while driver.log said "Driver is up!").
# `echo --once` subscribes directly: if a message arrives, the driver is up,
# whatever any daemon thinks.
echo -n "  waiting for /middle/joint_states "
for _ in $(seq 1 12); do
  if timeout 5 ros2 topic echo --once --no-daemon /middle/joint_states >/dev/null 2>&1; then
    echo "-- up."
    break
  fi
  echo -n "."
done

# The first IK solve traces and compiles under jax, which takes seconds. Doing
# it here means the stall happens now, with the arm still, rather than in the
# middle of a live session.
echo "  starting head agent (compiling the IK solver, this takes a few seconds)"
./head_agent.py --urdf "${URDF}" &
PIDS+=($!)

echo "=== middle-arm ready ==="
wait -n
