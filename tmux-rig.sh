#!/usr/bin/env bash
# =============================================================================
# One tmux session, one pane per live device.
#
#   ./tmux-rig.sh          (or: make tmux)
#
# Panes are created ONLY for components that are actually up, so a rig with the
# middle arm unplugged gets a three-pane session rather than a four-pane one
# with a dead terminal in the corner. Re-run it after connecting hardware.
#
#   +---------------------------+---------------------------+
#   |  state                    |  control                  |
#   |  watch.py --dash          |  rig_key.py               |
#   |  one line per subsystem   |  qwe/asd rty/fgh uio/jkl  |
#   |                           |  arrows base, z/x n/m grip|
#   +---------------------------+---------------------------+
#   |  shell -- arm_ctl.py, drive_test.py, ros2 topic ...   |
#   +-------------------------------------------------------+
#
# WHY EACH TOOL RUNS WHERE IT DOES. The base pane execs into slate-base and uses
# that container's own teleop_keyboard.py rather than a copy living in monitor:
# the base's velocity limits, its 300 ms deadline and its clamp all live in
# base_config.py next to it, and a second implementation would drift from them.
# The arm pane runs in monitor because arm_key.py is a pure ROS client and the
# arm containers are each locked to one arm -- one jog tool that can switch
# between arms with a keypress has to sit outside both.
#
# EVERY PANE NEEDS A REAL TTY, which is why this uses `docker compose exec`
# without -T. Both keyboard tools put the terminal in cbreak mode to read single
# keys without Enter; through `exec -T` there is no tty, cbreak fails, and the
# tool exits or ignores every key.
#
#   Ctrl-b then arrow   move between panes
#   click a pane        focus it (mouse mode is on)
#   scroll wheel        scroll THAT pane's history -- q or Esc to leave
#   Ctrl-b then d       detach (everything keeps running)
#   tmux attach -t rig  come back
#   Ctrl-b then &       kill the session (cleans up the containers too)
#
# Mouse mode is not a convenience. Without it the wheel is translated into arrow
# keys and delivered to the focused pane, which in this session is a tool that
# jogs a robot arm with the arrow keys. See the note further down.
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

SESSION="${TMUX_SESSION:-rig}"
DOCKER="${DOCKER:-docker}"
COMPOSE="${DOCKER} compose"

command -v tmux >/dev/null || {
  echo "tmux is not installed:  sudo apt install tmux" >&2
  exit 1
}

up() { ${COMPOSE} ps --services --filter status=running 2>/dev/null | grep -qx "$1"; }

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "  session '${SESSION}' already exists -- attaching."
  echo "  (kill it first with: tmux kill-session -t ${SESSION})"
  exec tmux attach -t "${SESSION}"
fi

# Bring the rig up first. Previously this refused to start when the containers
# were down and told you to run `make` -- which is a pointless extra step when
# it can simply do it. `up -d` is a no-op for anything already running.
# Anything left over from a previous session competes for the command topics
# with what this one is about to start, so it goes first -- and --force,
# because the session being created now does not yet exist to guard against.
if [ -x ./rig-cleanup.sh ]; then
  ./rig-cleanup.sh --force 2>/dev/null | sed 's/^/  /' || true
fi

echo "  starting containers ..."
${COMPOSE} up -d >/dev/null 2>&1 || {
  echo "  docker compose up failed. Try:  make" >&2
  exit 1
}

# The teleop source publishes to the same command topics as the tools in this
# session, at 72 Hz, and the newest message wins -- so a running quest container
# makes the base and arm panes feel dead. Stop it here rather than leaving a
# trap set.
if up quest; then
  echo "  stopping quest (it competes for the command topics)"
  ${COMPOSE} stop quest >/dev/null 2>&1 || true
fi

# Give the agents a moment to connect and start publishing before deciding which
# panes are worth creating.
sleep 3

up monitor || {
  echo "the monitor container is not running even after `up -d`." >&2
  echo "Check:  docker compose logs monitor" >&2
  exit 1
}

echo "  building session '${SESSION}' from what is actually up ..."

# Pane 0: state. Always present -- if only one thing works, it should be the
# one that tells you what everything else is doing.
tmux new-session -d -s "${SESSION}" -n rig \
  "${COMPOSE} exec monitor ./watch.py --dash; echo; echo '[state pane exited -- press enter]'; read"
echo "    state   watch.py --dash   (full table: ./watch.py in the shell pane)"

# An arm container can be UP and idle -- start.sh keeps it alive with an
# explanation when the arm is unreachable. What decides whether a jog pane is
# worth having is whether an agent is publishing ee_pose.
# ONE control pane, ALWAYS created.
#
# There used to be two keyboard panes -- a base teleop and an arm jog -- with
# different meanings for the same keys, and the arm one vanished when the arms
# were unreachable. Both were confusing: whichever pane had focus decided what a
# keystroke did, and a missing pane is harder to understand than one that says
# why it is waiting.
#
# rig_key.py handles base, lift and all three arms with one key map, and reports
# what is missing rather than disappearing. The layout is now the same every
# time, whatever is plugged in.
tmux split-window -h -t "${SESSION}:rig" \
  "${COMPOSE} exec monitor ./rig_key.py; echo; echo '[control pane exited -- press enter]'; read"
echo "    control rig_key.py   (SPACE enable all | qwe/asd rty/fgh uio/jkl = +/- xyz"
echo "                          arrows = base, z/x n/m grippers)"

# A free shell last, so there is always somewhere to type arm_ctl.py or
# drive_test.py without stealing a pane that is doing something.
LAST_PANE=$(tmux list-panes -t "${SESSION}:rig" -F '#{pane_index}' | tail -1)
tmux split-window -v -t "${SESSION}:rig.${LAST_PANE}" "${COMPOSE} exec monitor bash"
echo "    shell   free terminal in monitor"

# MOUSE MODE ON. This fixes two things that are the same bug wearing different
# hats.
#
# Without it, tmux does not interpret the scroll wheel at all, so the terminal
# emulator turns a scroll into arrow-key escape sequences and sends them to the
# FOCUSED pane -- regardless of which pane the pointer is over. In this session
# the focused pane is usually the arm jog tool, whose up/down arrows are bound
# to +-z. Scrolling anywhere in the window therefore JOGGED THE ARM, which is
# how an arm moved with nobody pressing a key.
#
# With mouse on, the wheel enters tmux's copy mode and scrolls that pane's
# scrollback instead. Nothing reaches the application. The same change makes the
# state pane scrollable, which is the other thing that did not work.
#
# q or Escape leaves copy mode and returns to live output.
tmux set-option -t "${SESSION}" mouse on
tmux set-option -t "${SESSION}" history-limit 20000

# MAKE `tmux kill-session` ACTUALLY SHUT THINGS DOWN.
#
# Killing the session tears down the panes, and the `docker compose exec`
# clients die with them -- but the processes INSIDE the container do not.
# watch.py and rig_key.py keep running, invisible to the host, still publishing
# to the same command topics at 20 Hz. Orphaned control processes are not inert:
# an expired dead-man publishes zeroes into the base's velocity topic and an
# expired anchor publishes a stale arm pose, and the newest message wins. Four
# watch.py and three rig_key.py had piled up in one session this way.
#
# The hook is GLOBAL, not session-scoped, because a session-scoped hook has no
# session left to run in by the time the session has closed. rig-cleanup.sh
# guards itself -- it exits immediately if a live `rig` session still exists --
# so firing it whenever ANY session closes is harmless.
#
# `-b` runs it in the background: a hook that blocks holds up the server.
tmux set-hook -g session-closed[71] \
  "run-shell -b '${PWD}/rig-cleanup.sh >/dev/null 2>&1'"

# ...AND A WATCHER, because the hook alone is not enough.
#
# Killing the LAST session makes the tmux server exit, and it exits before
# running the hook -- so `tmux kill-session -t rig` on a machine with only the
# rig session open, which is the normal case, would clean up nothing. Verified,
# not assumed: with a second session present the hook fires; alone, it does not.
#
# So a small detached watcher polls for the session instead. It needs no tmux
# server to survive, and it covers every way the session can end -- kill-session,
# Ctrl-b &, the server dying, the terminal being closed.
#
# It calls rig-cleanup.sh WITHOUT --force on purpose. If you kill the session and
# immediately start a new one, the watcher wakes to find a live `rig` again, and
# the cleanup guard makes it a no-op rather than killing the new session's
# processes. The extra sleep gives that restart room to happen.
#
# setsid + nohup so it is not in this shell's process group: without it the
# watcher dies with the terminal that ran `make tmux`, which is exactly when it
# is most needed.
setsid nohup bash -c "
  while tmux has-session -t '${SESSION}' 2>/dev/null; do sleep 2; done
  sleep 4
  '${PWD}/rig-cleanup.sh'
" >/dev/null 2>&1 &
disown 2>/dev/null || true

tmux select-layout -t "${SESSION}:rig" tiled
tmux select-pane -t "${SESSION}:rig.0"

echo
echo "  Ctrl-b <arrow> move  |  click to focus  |  scroll to read history"
echo "  Ctrl-b d detach       |  Ctrl-b & kill session"
echo

# Attach only if there is a terminal to attach TO. Started from a script, a
# pipe, or a `make` recipe whose stdin was redirected, `tmux attach` fails with
# "open terminal failed: not a terminal" and the script exits non-zero having
# actually done its job -- the session is built and running. Leave it detached
# and say how to reach it.
if [ -t 0 ] && [ -t 1 ]; then
  exec tmux attach -t "${SESSION}"
else
  echo "  not a terminal -- session left running in the background."
  echo "  Attach with:   tmux attach -t ${SESSION}"
fi
