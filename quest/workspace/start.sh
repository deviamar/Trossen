#!/usr/bin/env bash
# =============================================================================
# The teleop source. The container's `command:`.
#
# Backend from QUEST_BACKEND (default sim, deliberately): after a fresh build
# the safe thing is a container whose topics are live and whose robot commands
# come from a fake headset moving in a slow circle. Set QUEST_BACKEND=webrtc in
# docker-compose.yml when the chain is proven and the secrets are mounted.
#
# QUEST_TELEOP_ARGS passes flags through, e.g. "--dry-run" or "--no-base".
#
# Set AUTOSTART=false for a bare shell -- which is what you want for
# keyboard_teleop.py, since two teleop sources publishing to the same command
# topics is a fight neither wins.
# =============================================================================
set -euo pipefail

# shellcheck disable=SC1091
source /usr/local/bin/ros-env.sh

BACKEND="${QUEST_BACKEND:-sim}"
echo "=== quest starting (backend: ${BACKEND}) ==="
if [ "${BACKEND}" = "sim" ]; then
  echo "  SIMULATED HEADSET -- robot commands come from a fake circle, not a person."
fi
echo

# shellcheck disable=SC2086
exec ./launch-quest.sh --backend "${BACKEND}" ${QUEST_TELEOP_ARGS:-}
