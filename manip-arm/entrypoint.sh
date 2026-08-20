#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

# The image-built overlay carrying trossen_arm_description (URDF + meshes for
# the RViz preview). This covers the container's main process; `docker compose
# exec` bypasses the entrypoint entirely, and picks the overlay up from .bashrc
# instead -- which is why `exec arm-1 bash` works but `exec arm-1 ./preview.py`
# (no shell, no .bashrc) would not find rclpy. Run it from inside a shell, or
# `exec arm-1 bash -lic './preview.py ...'`.
desc_ws="${DESC_WS:-$HOME/ros2_ws}"
if [ -f "${desc_ws}/install/setup.bash" ]; then
  source "${desc_ws}/install/setup.bash"
fi

ws="${ROS_WS:-$HOME/workspace}"
if [ -f "${ws}/install/setup.bash" ]; then
  source "${ws}/install/setup.bash"
fi

exec "$@"
