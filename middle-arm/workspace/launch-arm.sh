#!/usr/bin/env bash
# =============================================================================
# Bring up the middle arm (wx250s, gripper removed -- camera in its place).
#
# Wraps xsarm_control.launch.py with the six arguments this specific arm needs.
# They are easy to get wrong by hand, and getting them wrong fails in two
# distinct ways:
#   - forgetting motor_configs  -> xs_sdk pings DYNAMIXEL ID 9, gets no answer,
#                                  and aborts ("9th motor not found")
#   - forgetting use_gripper    -> driver is fine, but robot_description still
#                                  carries the gripper links, so RViz and TF
#                                  show a gripper that is not on the robot
#
# Usage, from inside the container:
#   ./launch-arm.sh              # real hardware, no RViz
#   ./launch-arm.sh --rviz       # real hardware + RViz
#   ./launch-arm.sh --sim        # DYNAMIXEL simulator, no hardware needed
#   ./launch-arm.sh --limp       # real hardware, TORQUE OFF (read-only test)
#   ./launch-arm.sh --dump-urdf  # print the URDF and exit (for head_agent.py)
#   ./launch-arm.sh --sim --rviz
#
# On the real arm this holds station: position mode + torque_enable seeds each
# goal from the present encoder reading, so the arm locks where it already is
# and no motion is commanded. See modes_nogripper.yaml.
#
# --limp is the safe first test against real hardware: it enumerates the whole
# DYNAMIXEL chain and publishes live encoder values without energizing anything,
# so a mis-configured bus surfaces as a clean startup error rather than a
# powered arm doing something unexpected. Support the arm first -- torque off
# means it is held only by friction.
# =============================================================================
set -euo pipefail

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/config"

ROBOT_MODEL="${ROBOT_MODEL:-wx250s}"
ROBOT_NAME="${ROBOT_NAME:-middle}"
USE_SIM=false
USE_RVIZ=false
MODE_FILE="modes_nogripper.yaml"

# Registers are stored in each motor's EEPROM and survive a power cycle, so they
# only need writing once. Leave this true for the first successful run against
# new hardware, then export LOAD_CONFIGS=false -- it shaves a few seconds off
# startup and spares the EEPROM's finite write cycles.
LOAD_CONFIGS="${LOAD_CONFIGS:-true}"

# --dump-urdf writes this arm's URDF to stdout and exits, launching nothing.
# head_agent.py needs a URDF to build its pyroki model, and the only URDF that
# is definitely right is the one generated from the same xacro and the same
# arguments the driver uses -- writing one by hand is how the solver ends up
# describing a slightly different robot than the one on the bench.
DUMP_URDF=false

for arg in "$@"; do
  case "$arg" in
    --sim)   USE_SIM=true  ;;
    --rviz)  USE_RVIZ=true ;;
    --limp)  MODE_FILE="modes_nogripper_limp.yaml" ;;
    --dump-urdf) DUMP_URDF=true ;;
    -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
  esac
done

if [ "${DUMP_URDF}" = true ]; then
  # Same arguments as the launch below, so the model matches the running arm.
  # Note this does NOT include the ZED: the vendor xacro has no camera on the
  # wrist (see README), so the URDF ends at the flange.
  exec xacro \
    "$(ros2 pkg prefix interbotix_xsarm_descriptions)/share/interbotix_xsarm_descriptions/urdf/${ROBOT_MODEL}.urdf.xacro" \
    robot_name:="${ROBOT_NAME}" \
    base_link_frame:=base_link \
    show_ar_tag:=false \
    show_gripper_bar:=false \
    show_gripper_fingers:=false \
    use_world_frame:=false
fi

if [ "${USE_SIM}" = "false" ] && [ ! -e /dev/ttyDXL ]; then
  echo "ERROR: /dev/ttyDXL is missing -- the U2D2 is unplugged, the arm is" >&2
  echo "       powered off, or the host udev rule was never installed." >&2
  echo "       Host-side check:  ls -l /dev/ttyDXL" >&2
  echo "       Install rules:    ./host-setup/setup-host.sh" >&2
  echo "       Or dry-run without hardware:  $0 --sim" >&2
  exit 1
fi

exec ros2 launch interbotix_xsarm_control xsarm_control.launch.py \
  robot_model:="${ROBOT_MODEL}" \
  robot_name:="${ROBOT_NAME}" \
  motor_configs:="${CONFIG_DIR}/wx250s_nogripper.yaml" \
  mode_configs:="${CONFIG_DIR}/${MODE_FILE}" \
  load_configs:="${LOAD_CONFIGS}" \
  use_gripper:=false \
  show_gripper_bar:=false \
  show_gripper_fingers:=false \
  use_sim:="${USE_SIM}" \
  use_rviz:="${USE_RVIZ}"
