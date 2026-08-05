# Sourced, not executed. Layers the four ROS 2 overlays in dependency order:
#   1. /opt/ros/$ROS_DISTRO   ROS 2 Humble itself
#   2. /opt/zed_ws            ZED ROS 2 wrapper           (baked into the image)
#   3. /opt/interbotix_ws     Interbotix X-Series stack   (baked into the image)
#   4. $ROS_WS                your code                   (bind-mounted from host)
#
# Shared by entrypoint.sh, ~/.bashrc, and $BASH_ENV so an `exec bash` and a `run`
# command always land in the same environment.
#
# Idempotence guard: $BASH_ENV makes bash source this file for EVERY
# non-interactive shell, including nested ones inside build scripts. Without the
# guard each nesting level re-appends to AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH
# and pays the sourcing cost again. The flag is exported, so once any ancestor
# process has set up the environment its children correctly skip the work.
if [ -n "${ROS_ENV_SOURCED:-}" ]; then
  return 0 2>/dev/null || true
fi
export ROS_ENV_SOURCED=1

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f "${ZED_WS:-/opt/zed_ws}/install/setup.bash" ]; then
  source "${ZED_WS:-/opt/zed_ws}/install/setup.bash"
fi

if [ -f "${INTERBOTIX_WS:-/opt/interbotix_ws}/install/setup.bash" ]; then
  source "${INTERBOTIX_WS:-/opt/interbotix_ws}/install/setup.bash"
fi

ws="${ROS_WS:-$HOME/workspace}"
if [ -f "${ws}/install/setup.bash" ]; then
  source "${ws}/install/setup.bash"
fi
unset ws
