#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

ws="${ROS_WS:-$HOME/workspace}"
if [ -f "${ws}/install/setup.bash" ]; then
  source "${ws}/install/setup.bash"
fi

exec "$@"
