#!/usr/bin/env python3
"""Arrow-key teleop for the middle arm (wx250s, gripperless).

    ./teleop_keyboard.py                 # 0.03 rad steps, 300 ms motion profile
    ./teleop_keyboard.py --step 0.05
    ./teleop_keyboard.py --profile-ms 500   # gentler/slower response

MUST be run on a terminal (an interactive `docker compose exec middle-arm bash`),
not through `exec -T` -- it puts the tty in cbreak mode to read single keys.

Controls
    up / down       shoulder   raise / lower the arm
    left / right    waist      swing left / right
    w / s           elbow
    a / d           wrist_angle
    r / f           wrist_rotate     <- camera mount, watch the cable
    t / g           forearm_roll     <- camera mount, watch the cable
    [ / ]           step size down / up
    space           resync target to where the arm actually is
    ESC or Ctrl-C   quit

Design notes:

Commands go out as JointSingleCommand, one joint at a time, rather than a group
command. That matters here: a group command carries all six joints, and this arm
often sits with `shoulder` slightly outside its URDF limit after being folded --
so the whole group message would be refused, or would jolt the shoulder on
startup as it got clamped into range. Per-joint commands touch only the joint you
press and leave the driver holding the rest exactly where they are.

Targets are CLAMPED to the joint limits here, not refused as in move_joint.py.
For one-shot commands, refusing is right -- a clamped move still moves, just not
where you asked. For continuous teleop, refusing means the arm stops responding
at the edge of travel with no indication why, so clamping (with a visible LIM
marker) is the better behavior.

Torque is deliberately left ON at exit. Dropping it would let the arm fall.
"""
import argparse
import math
import os
import re
import select
import shutil
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes

NS = "/middle"
GROUP = "arm"

# key -> (joint, direction). Arrows are the spatial ones; letters the rest.
KEYMAP = {
    "\x1b[A": ("shoulder", +1),   "\x1b[B": ("shoulder", -1),
    "\x1b[D": ("waist", +1),      "\x1b[C": ("waist", -1),
    "w": ("elbow", +1),           "s": ("elbow", -1),
    "a": ("wrist_angle", +1),     "d": ("wrist_angle", -1),
    "r": ("wrist_rotate", +1),    "f": ("wrist_rotate", -1),
    "t": ("forearm_roll", +1),    "g": ("forearm_roll", -1),
}


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.joints = None
        self.positions = None
        self.limits = {}
        self.create_subscription(JointState, f"{NS}/joint_states", self._js_cb, 1)
        self.create_subscription(
            String, f"{NS}/robot_description", self._urdf_cb,
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST),
        )
        self.pub = self.create_publisher(
            JointSingleCommand, f"{NS}/commands/joint_single", 1)
        self.modes = self.create_client(OperatingModes, f"{NS}/set_operating_modes")

    def _js_cb(self, msg):
        self.joints, self.positions = list(msg.name), list(msg.position)

    def _urdf_cb(self, msg):
        for m in re.finditer(r"<joint name=\"([a-z_]+)\"[^>]*>(.*?)</joint>",
                             msg.data, re.S):
            lim = re.search(r"lower=\"([-0-9.eE]+)\"\s+upper=\"([-0-9.eE]+)\"",
                            m.group(2))
            if lim:
                self.limits[m.group(1)] = (float(lim.group(1)), float(lim.group(2)))

    def ready(self):
        return self.joints is not None and bool(self.limits)

    def set_profile(self, ms):
        """Retune how long each commanded move takes. Returns True on success."""
        if not self.modes.wait_for_service(timeout_sec=2.0):
            return False
        req = OperatingModes.Request(
            cmd_type="group", name=GROUP, mode="position",
            profile_type="time", profile_velocity=int(ms),
            profile_acceleration=int(max(1, ms // 6)),
        )
        fut = self.modes.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.done()


def read_key(timeout=0.05):
    """Return a keypress, decoding arrow escape sequences. None if nothing.

    Uses os.read on the raw fd rather than sys.stdin.read(). Two reasons, both
    of which broke the earlier version:

      1. sys.stdin is a buffered TEXT stream, and .read(1) on it blocks trying
         to fill its internal buffer rather than returning the single byte that
         is already available.
      2. Even when it returns, it may pull the whole escape sequence into that
         Python-level buffer. The follow-up select() then polls the OS-level fd,
         sees nothing pending, and concludes the ESC was standalone -- so every
         arrow key decoded as "quit".

    os.read bypasses the buffering entirely, keeping select() and the reads
    talking about the same bytes.
    """
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout)[0]:
        return None
    data = os.read(fd, 1)
    if data != b"\x1b":
        return data.decode("utf-8", "replace")
    # Bare ESC or the start of a CSI sequence? Peek briefly so a lone ESC still
    # registers as quit instead of hanging here.
    if select.select([fd], [], [], 0.05)[0]:
        data += os.read(fd, 2)
    return data.decode("utf-8", "replace")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", type=float, default=0.03,
                    help="radians per keypress (default 0.03, ~1.7 deg)")
    ap.add_argument("--profile-ms", type=int, default=300,
                    help="ms per commanded move; lower is snappier (default 300)")
    args = ap.parse_args()

    if not os.isatty(sys.stdin.fileno()):
        print("stdin is not a tty -- run this from an interactive shell:\n"
              "  docker compose exec middle-arm bash\n"
              "  ./teleop_keyboard.py", file=sys.stderr)
        return 1

    rclpy.init()
    node = Teleop()
    step = args.step
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        deadline = node.get_clock().now().nanoseconds + int(5e9)
        while rclpy.ok() and not node.ready():
            if node.get_clock().now().nanoseconds > deadline:
                print(f"No data on {NS}/joint_states within 5s. Is the arm launched?",
                      file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        profile_ok = node.set_profile(args.profile_ms)
        target = dict(zip(node.joints, node.positions))

        print(__doc__.split("Controls")[1].split("Design notes")[0].rstrip())
        print(f"\n  step={step:.3f} rad   profile={args.profile_ms} ms"
              f"{'' if profile_ok else '  (profile NOT set -- service call failed)'}\n")

        tty.setcbreak(fd)
        lines = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()

            if key in ("\x1b", "\x03"):
                break
            if key == " ":
                target = dict(zip(node.joints, node.positions))
            elif key == "[":
                step = max(0.005, step - 0.005)
            elif key == "]":
                step = min(0.25, step + 0.005)
            elif key in KEYMAP:
                joint, sign = KEYMAP[key]
                if joint in target:
                    lo, hi = node.limits.get(joint, (-math.inf, math.inf))
                    target[joint] = min(hi, max(lo, target[joint] + sign * step))
                    node.pub.publish(
                        JointSingleCommand(name=joint, cmd=float(target[joint])))

            rows = [f"  step {step:.3f} rad   ESC=quit  space=resync  [/]=step",
                    f"  {'joint':<14}{'actual':>9}{'target':>9}  {'':>4}"]
            for j, actual in zip(node.joints, node.positions):
                tgt = target.get(j, actual)
                lo, hi = node.limits.get(j, (-math.inf, math.inf))
                mark = "LIM" if tgt <= lo + 1e-6 or tgt >= hi - 1e-6 else ""
                rows.append(f"  {j:<14}{actual:>9.4f}{tgt:>9.4f}  {mark:>4}")

            # Truncate every row to the terminal width. A line longer than the
            # window wraps onto a second PHYSICAL row, but the cursor-up below
            # counts LOGICAL rows -- so a single wrap desynchronizes them and
            # the display shreds itself a little more on every frame.
            width = shutil.get_terminal_size((80, 24)).columns
            rows = [r[: max(1, width - 1)] for r in rows]

            if lines:
                sys.stdout.write(f"\033[{lines}A")
            lines = len(rows)
            # \033[K clears to end of line, so a shorter row cannot leave tail
            # characters behind from the longer row it is overwriting.
            sys.stdout.write("".join(r + "\033[K\n" for r in rows))
            sys.stdout.flush()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        # Put the motion profile back so a later launch or script sees the
        # configured default rather than this session's teleop tuning.
        try:
            node.set_profile(2000)
        except Exception:
            pass
        print("\n  teleop ended. Torque left ON -- the arm is still holding position.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
