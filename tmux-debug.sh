#!/usr/bin/env bash
# =============================================================================
# A SECOND tmux session, for debugging one arm at a time.
#
#   ./tmux-debug.sh        (or: make debug)
#
#   +---------------------------+---------------------------+
#   |  state                    |  rig_debug.py             |
#   |  watch.py --dash          |  TAB select  SPACE arm    |
#   |                           |  1-6/!@#$%^ joints        |
#   |                           |  q/a w/s e/d  orientation |
#   |                           |  u/j i/k o/l  position    |
#   |                           |  p/ENTER poses  h home    |

#
# SEPARATE FROM `rig`, AND MUTUALLY EXCLUSIVE WITH IT. rig_key.py and
# rig_debug.py publish to the same command topics, and the newest message on a
# topic wins -- run both and each silently overwrites the other's target every
# 50 ms. control_lock.py refuses to start the second one, so this script kills
# the operating session first rather than letting you discover it as juddering
# motion.
#
#   Ctrl-b then d       detach (everything keeps running)
#   tmux attach -t rig-debug
#   make kill SESSION=rig-debug   end it and clean up the containers
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

SESSION="${TMUX_DEBUG_SESSION:-rig-debug}"
DOCKER="${DOCKER:-docker}"
COMPOSE="${DOCKER} compose"

command -v tmux >/dev/null || { echo "tmux is not installed: sudo apt install tmux" >&2; exit 1; }

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "  session '${SESSION}' already exists -- attaching."
  exec tmux attach -t "${SESSION}"
fi

# The operating session has to go first: its rig_key.py would fight rig_debug.py
# for the command topics, and control_lock.py would simply refuse to start.
if tmux has-session -t rig 2>/dev/null; then
  echo "  stopping the 'rig' session -- only one control tool may run at a time"
  tmux kill-session -t rig 2>/dev/null || true
fi
./rig-cleanup.sh --force 2>/dev/null | sed 's/^/  /' || true

echo "  starting containers ..."
${COMPOSE} up -d >/dev/null 2>&1 || { echo "  docker compose up failed. Try: make" >&2; exit 1; }

if ${COMPOSE} ps --services --filter status=running 2>/dev/null | grep -qx quest; then
  echo "  stopping quest (it competes for the command topics)"
  ${COMPOSE} stop quest >/dev/null 2>&1 || true
fi
sleep 3

tmux new-session -d -s "${SESSION}" -n debug \
  "${COMPOSE} exec monitor ./watch.py --dash; echo; echo '[state pane exited -- enter]'; read"
tmux split-window -h -t "${SESSION}:debug" \
  "${COMPOSE} exec monitor ./rig_debug.py; echo; echo '[debug pane exited -- enter]'; read"
# Two panes only, same reasoning as the operating session: a third pane costs
# the other two half their height, and rig_debug's status line is long. A shell
# is Ctrl-b c away.

# Mouse mode is not a convenience. Without it tmux does not interpret the scroll
# wheel, so the terminal turns a scroll into ARROW KEYS and delivers them to the
# focused pane -- which here is a tool that moves a robot arm.
tmux set-option -t "${SESSION}" mouse on
tmux set-option -t "${SESSION}" history-limit 20000
tmux select-layout -t "${SESSION}:debug" even-horizontal
tmux select-pane -t "${SESSION}:debug.1"

# Same watcher as the operating session: killing the LAST tmux session makes the
# server exit before any hook can run, so cleanup cannot depend on tmux at all.
setsid nohup bash -c "
  while tmux has-session -t '${SESSION}' 2>/dev/null; do sleep 2; done
  sleep 4
  '${PWD}/rig-cleanup.sh' --force
" >/dev/null 2>&1 &
disown 2>/dev/null || true

echo
echo "  TAB select arm | SPACE arm/disarm | 1-6 !@#\$%^ joints"
echo "  q/a w/s e/d orientation | u/j i/k o/l position | p ENTER poses | h home"
echo
if [ -t 0 ] && [ -t 1 ]; then
  exec tmux attach -t "${SESSION}"
else
  echo "  not a terminal -- attach with:  tmux attach -t ${SESSION}"
fi
