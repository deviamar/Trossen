# Sourced, not executed. Layers the ROS 2 overlays in dependency order:
#   1. /opt/ros/$ROS_DISTRO   ROS 2 Humble itself
#   2. /opt/interbotix_ws     interbotix_slate_driver + trossen_slate  (in image)
#   3. $ROS_WS                your code                  (bind-mounted from host)
#
# Shared by entrypoint.sh, ~/.bashrc, and $BASH_ENV so an `exec bash` and a `run`
# command always land in the same environment. Same file, same job, as
# ../middle-arm/ros-env.sh -- with one fewer overlay, since there is no ZED here.
#
# Idempotence guard: $BASH_ENV makes bash source this for EVERY non-interactive
# shell, including nested ones inside scripts. Without the guard each nesting
# level re-appends to AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH and pays the
# sourcing cost again. The flag is exported, so once any ancestor process has
# set the environment up its children correctly skip the work.
if [ -n "${ROS_ENV_SOURCED:-}" ]; then
  return 0 2>/dev/null || true
fi
export ROS_ENV_SOURCED=1

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f "${INTERBOTIX_WS:-/opt/interbotix_ws}/install/setup.bash" ]; then
  source "${INTERBOTIX_WS:-/opt/interbotix_ws}/install/setup.bash"
fi

ws="${ROS_WS:-$HOME/workspace}"
if [ -f "${ws}/install/setup.bash" ]; then
  source "${ws}/install/setup.bash"
fi
unset ws
