#!/usr/bin/env bash
# RViz showing the wxai model, driven by whatever publishes /joint_states.
#
#   ./launch-rviz.sh              # empty model, waiting for preview.py
#   ./launch-rviz.sh --sliders    # joint_state_publisher_gui instead: drag joints by hand
#   ./launch-rviz.sh --variant follower
#
# Then, in a SECOND shell into the same container:
#   ./preview.py pose <name>
#
# This starts robot_state_publisher + rviz2 from trossen_arm_description and
# nothing else. It opens NO connection to the arm -- it is inert, so it is safe
# to leave running while pose.py or move_joint.py commands the real hardware,
# and `./preview.py live` will mirror the arm into it.
#
# The default flips upstream's use_joint_pub_gui to false on purpose: the
# slider GUI publishes /joint_states itself, and two publishers on that topic
# make the model jitter between them. Pass --sliders when the sliders ARE what
# you want, and then do not run preview.py at the same time.
set -e

VARIANT="${ARM_EE_VARIANT:-base}"   # base | leader | follower
SLIDERS=false

while [ $# -gt 0 ]; do
  case "$1" in
    --sliders)  SLIDERS=true; shift ;;
    --variant)  VARIANT="$2"; shift 2 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *)          echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is unset -- RViz has nowhere to draw." >&2
  echo "On the host, once per login session: xhost +local:docker" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# trossen_arm_description is built INTO THE IMAGE, not into the bind-mounted
# ./workspace. So this script arriving in a running container (it is bind-mounted,
# it appears the moment it is written on the host) does NOT mean the package it
# needs is there: that takes a rebuild. Checked explicitly because the bare
# `source` failure is an unhelpful "No such file or directory" on a path nobody
# asked for.
DESC_WS="${DESC_WS:-$HOME/ros2_ws}"
if [ ! -f "${DESC_WS}/install/setup.bash" ]; then
  echo "trossen_arm_description is not in this container: no ${DESC_WS}/install" >&2
  echo >&2
  echo "It is built into the image, so a running container from an older build" >&2
  echo "does not have it. On the HOST:" >&2
  echo >&2
  echo "    cd ~/Trossen/manip-arm" >&2
  echo "    docker compose build arm-1" >&2
  echo "    docker compose up -d --force-recreate arm-1" >&2
  echo >&2
  echo "then exec back in and re-run this. (up -d alone will not replace a" >&2
  echo "container that is already running -- it needs --force-recreate.)" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${DESC_WS}/install/setup.bash"

echo "  wxai (${VARIANT}) in RViz. Ctrl-C to stop."
if [ "${SLIDERS}" = true ]; then
  echo "  Sliders on: do NOT run preview.py at the same time."
else
  echo "  Waiting for /joint_states -- run ./preview.py in another shell."
  echo "  The model sits at all-zeros until something publishes."
fi

exec ros2 launch trossen_arm_description display.launch.py \
  arm_variant:="${VARIANT}" \
  use_rviz:=true \
  use_joint_pub:=false \
  use_joint_pub_gui:="${SLIDERS}"
