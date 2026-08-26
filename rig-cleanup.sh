#!/usr/bin/env bash
# =============================================================================
# Shut down everything the tmux rig session started INSIDE the containers.
#
#   ./rig-cleanup.sh            # kill container-side control processes
#   ./rig-cleanup.sh --dry-run  # list what would be killed, touch nothing
#   ./rig-cleanup.sh --force    # clean up even while the rig session is alive
#
# WHY THIS IS NEEDED AT ALL
# -------------------------
# `docker compose exec` does NOT kill the process inside the container when the
# client goes away. tmux kill-session tears down the panes, the exec clients die
# with them, and watch.py / rig_key.py keep running in the container -- forever,
# and invisibly, because nothing on the host shows them any more.
#
# They are not idle. Every orphaned rig_key.py keeps publishing to the SAME
# command topics as the live one at 20 Hz, and the newest message on a topic
# wins. An orphan whose dead-man has expired publishes ZEROES, so the base gets
# go/stop/go/stop and the motion turns to stutter. An orphan that still thinks
# an arm is enabled republishes an anchor pose captured before the arm moved,
# which the agent then refuses as a metre-scale jump every frame.
#
# Both of those were observed. Four watch.py and three rig_key.py had
# accumulated over a session, holding the monitor container at 120% CPU.
#
# THE GUARD. By default this refuses to run while a live `rig` session exists,
# so it is safe to wire to a GLOBAL tmux hook: whichever session closed, the
# cleanup only proceeds once the rig itself is actually gone.
# =============================================================================
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

SESSION="${TMUX_SESSION:-rig}"
DOCKER="${DOCKER:-docker}"
COMPOSE="${DOCKER} compose"
DRY=false
FORCE=false

while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=true; shift ;;
    --force)   FORCE=true; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [ "${FORCE}" = false ] && tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "  '${SESSION}' is still running -- nothing to clean up."
  echo "  (kill it first, or pass --force)"
  exit 0
fi

if ! ${COMPOSE} ps --services --filter status=running 2>/dev/null | grep -qx monitor; then
  echo "  monitor is not running -- no container-side processes to kill."
  exit 0
fi

# Matched by reading /proc/<pid>/cmdline rather than with `pkill -f`. pkill
# matches the FULL command line of every process, including the shell running
# this very script -- which contains these patterns as text, so pkill kills
# itself partway through. That is not hypothetical; it happened.
read -r -d '' FINDER <<'INNER'
for p in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
  [ -r "/proc/$p/cmdline" ] || continue
  # What the process IS, not what its command line mentions. Matching the
  # cmdline alone makes this script find ITSELF: the shell running this finder
  # has these very patterns in its argv, so it matched, and the cleanup then
  # tried to kill the process doing the cleaning. comm is the executable name,
  # so a bash wrapper is excluded no matter what text it carries.
  comm=$(cat "/proc/$p/comm" 2>/dev/null)
  case "$comm" in python*) ;; *) continue ;; esac
  c=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null)
  case "$c" in
    *rig_key.py*|*watch.py*|*teleop_keyboard.py*) echo "$p|$c" ;;
  esac
done
INNER

found=$(${COMPOSE} exec -T monitor bash -c "${FINDER}" 2>/dev/null)

if [ -z "${found}" ]; then
  echo "  no orphaned control processes in monitor."
else
  echo "  orphaned control processes in monitor:"
  echo "${found}" | while IFS='|' read -r pid cmd; do
    echo "    pid ${pid}  ${cmd}"
  done
  if [ "${DRY}" = true ]; then
    echo "  DRY RUN -- nothing killed."
  else
    pids=$(echo "${found}" | cut -d'|' -f1 | tr '\n' ' ')
    # SIGTERM first, and the wait is not politeness. rig_key.py releases every
    # arm in its `finally` block; SIGKILL skips that and leaves them ARMED,
    # which is the one outcome this script must never produce.
    ${COMPOSE} exec -T monitor bash -c "kill -TERM ${pids} 2>/dev/null; sleep 3; kill -KILL ${pids} 2>/dev/null" >/dev/null 2>&1
    echo "  terminated: ${pids}"
  fi
fi

# Backstop, because the above may have had to resort to SIGKILL: say plainly
# that nothing should be moving. Disable puts each arm in idle, which on a wxai
# is a braked hold, and a zero Twist stops the base. Both are safe to repeat and
# safe to send when the hardware is absent -- they just go nowhere.
if [ "${DRY}" = false ]; then
  ${COMPOSE} exec -T monitor bash -c '
    set +u; source /opt/ros/humble/setup.bash
    for ns in /left_arm /right_arm /middle; do
      timeout 3 ros2 topic pub --once "$ns/enable" std_msgs/Bool "{data: false}" >/dev/null 2>&1
    done
    timeout 3 ros2 topic pub --once /slate/cmd_vel_teleop geometry_msgs/Twist \
      "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1' >/dev/null 2>&1
  echo "  arms released, base commanded to zero."
fi
