# Source this on the HOST to talk to the rig's ROS graph from an ordinary shell.
#
#   source ~/Trossen/env.sh
#   ros2 topic list
#
# Inside the containers you never need this -- each image sources ROS from its
# entrypoint, ~/.bashrc and $BASH_ENV, so `docker compose exec <svc> bash` and
# `exec <svc> bash -lc '...'` both land in a working environment. This file is
# only for the host, where `ros2: command not found` means nothing has been
# sourced yet.
#
# It works because every container runs `network_mode: host`: the containers and
# your host shell share one network stack, so a host-side `ros2 topic echo` sees
# container topics with no bridge and no extra configuration. The three settings
# below are what put you on the same graph -- they must match the containers or
# you will see an empty topic list and no error.
#
# To have it always: echo 'source ~/Trossen/env.sh' >> ~/.bashrc

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "env.sh: no ROS 2 Humble at /opt/ros/humble -- host-side ros2 tools" >&2
  echo "        will not work. Use the monitor container instead:" >&2
  echo "        docker compose exec monitor ./watch.py" >&2
fi

# MUST match every docker-compose.yml in this repo. Not 0: that is the ROS
# default, so an unrelated rig on the same wifi would land in the same graph.
export ROS_DOMAIN_ID=42

# Pinned everywhere for the same reason: a DDS implementation mismatch is a
# silent no-discovery failure that looks exactly like a network problem.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
