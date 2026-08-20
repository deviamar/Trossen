# Sourced, not executed. Same shape as ../slate-base/ros-env.sh and
# ../middle-arm/ros-env.sh: one file, sourced from the entrypoint, ~/.bashrc and
# $BASH_ENV, so every way into the container lands in the same environment.
#
# The idempotence guard matters because $BASH_ENV makes bash source this for
# every non-interactive shell, nested ones included.
if [ -n "${ROS_ENV_SOURCED:-}" ]; then
  return 0 2>/dev/null || true
fi
export ROS_ENV_SOURCED=1

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# The workspace is bind-mounted Python, not a colcon build -- put it on the path
# so `import quest_config` and `import backends` work from any directory.
export PYTHONPATH="${ROS_WS:-$HOME/workspace}:${PYTHONPATH:-}"
