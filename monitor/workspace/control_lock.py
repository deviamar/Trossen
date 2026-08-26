#!/usr/bin/env python3
"""One rig control tool at a time.

TWO OF THESE FIGHT, and the failure is silent. rig_key.py and rig_debug.py
publish to the same command topics -- cmd_pose, cmd_joints, enable, cmd_vel --
and on a ROS topic the newest message wins. Run both and every 50 ms each one
overwrites the other's target with its own idea of where the arm should be.

That is not hypothetical. Three orphaned rig_key processes accumulated in one
session (docker compose exec does not kill the process inside the container when
the client dies) and the result was juddering base motion, arm targets rejected
as metre-scale jumps, and a monitor container at 120% CPU. None of it looked
like a duplicate process; it looked like broken hardware.

So the tools check for each other before starting.
"""
import os

# Every program that publishes to the rig's command topics. Adding one here is
# what makes it mutually exclusive with the rest.
CONTROL_TOOLS = ("rig_key.py", "rig_debug.py", "teleop_keyboard.py", "arm_key.py")


def other_instances(exclude=()):
    """Control tools running in this container, as (pid, cmdline).

    Read from /proc rather than shelling out to pgrep: `pgrep -f` matches the
    full command line of EVERY process including the one doing the matching, so
    the obvious implementation finds itself and nothing ever starts. Anchored on
    /proc/<pid>/comm -- what the process IS -- so a shell that merely mentions
    one of these names in its arguments is not mistaken for the tool itself.
    """
    me = os.getpid()
    names = [t for t in CONTROL_TOOLS if t not in exclude]
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(f"/proc/{entry}/comm") as f:
                comm = f.read().strip()
            if not comm.startswith("python"):
                continue
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="ignore").strip()
        except OSError:
            continue          # it exited while we were looking
        if any(n in cmd for n in names):
            found.append((entry, cmd))
    return found


def refuse_if_busy(self_name, stream):
    """Print why and return an exit code, or None if it is safe to start."""
    others = other_instances()
    if not others:
        return None
    print(f"another rig control tool is already running in this container:",
          file=stream)
    for pid, cmd in others:
        print(f"    pid {pid}  {cmd}", file=stream)
    print("\nTwo of these FIGHT: they publish to the same command topics and the\n"
          "newest message wins, so each overwrites the other's target. That shows\n"
          "up as juddering motion and as commands being ignored -- never as a\n"
          "duplicate process, which is why this refuses to start.\n\n"
          "    make kill        end the session and clean up\n"
          "    make orphans     list them without killing anything",
          file=stream)
    return 2
