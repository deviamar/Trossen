#!/usr/bin/env bash
set -e
# ROS's setup.bash reads unset variables; `set -u` here would abort on
# AMENT_TRACE_SETUP_FILES before the container ever starts. Learned the hard way
# on the arm containers.
set +u
source /opt/ros/humble/setup.bash
set -u
exec "$@"
