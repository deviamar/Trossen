#!/usr/bin/env python3
"""One arm at a time, every way of commanding it. For debugging, not driving.

    ./rig_debug.py                  # left arm selected, nothing armed
    ./rig_debug.py --arm right
    ./rig_debug.py --dry-run        # print, publish nothing

HOW THIS DIFFERS FROM rig_key.py, and why both exist. rig_key drives the whole
rig at once: every arm and the base live on one keyboard, no modes, so you can
move three things in the same second. That is the right shape for operating and
the wrong shape for debugging, because it has no room left for the controls you
only want occasionally -- joint angles, wrist attitude, named poses.

This is the other shape. ONE arm is selected at a time, and the entire keyboard
belongs to it. That frees the number row for joints and the letters for the
things rig_key cannot spare keys for.

    SELECT     TAB    cycle arm: left -> right -> middle
               SPACE  arm / disarm the selected arm

    JOINTS     1 2 3 4 5 6      + that joint
               ! @ # $ % ^      - that joint          [ ] step size
      Joint space has no IK, so it is the ONLY thing that still works at a
      singularity, and the way out of one.

    ORIENT     q/a roll   w/s pitch   e/d yaw         { } step size
      Rotates the end effector about the WORLD axes, holding position.

    POSITION   u/j  x     i/k  y      o/l  z          - = step size
      Same axis order as rig_key's left-arm cluster.

    POSES      p      cycle through this arm's saved poses
               ENTER  go to the shown pose            h  go straight to 'home'
      A whole-arm move over several seconds. Watch it.

    STATE      z  re-zero (current position reads 0,0,0)
               r  reset this arm's agent after a fault
               s  save the current pose (prompts for a name)

    QUIT       ESC or Ctrl-C -- arms are left ARMED and holding, deliberately.

MUTUALLY EXCLUSIVE WITH rig_key.py. Both publish to the same command topics and
the newest message wins, so running both means each silently overwrites the
other's target. control_lock.py refuses to start the second one.
"""
import argparse
import math
import os
import select
import sys
import termios
import time
import tty

import control_lock
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

ARMS = {
    "left": os.environ.get("RIG_LEFT_NS", "/left_arm"),
    "right": os.environ.get("RIG_RIGHT_NS", "/right_arm"),
    "middle": os.environ.get("RIG_MIDDLE_NS", "/middle"),
}
NUM_ARM_JOINTS = 6

JOINT_POS = "123456"
JOINT_NEG = "!@#$%^"
ROT_KEYS = {"q": ("x", +1), "a": ("x", -1),
            "w": ("y", +1), "s": ("y", -1),
            "e": ("z", +1), "d": ("z", -1)}
POS_KEYS = {"u": ("x", +1), "j": ("x", -1),
            "i": ("y", +1), "k": ("y", -1),
            "o": ("z", +1), "l": ("z", -1)}

DEFAULT_JOINT_STEP = math.radians(3.0)
DEFAULT_ROT_STEP = math.radians(5.0)
DEFAULT_POS_STEP = 0.01
REPUBLISH_HZ = 20.0
UP, DOWN, RIGHT, LEFT = "\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D"


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def quat_about(axis, angle):
    h = angle / 2.0
    q = [0.0, 0.0, 0.0, math.cos(h)]
    q["xyz".index(axis)] = math.sin(h)
    return q


def quat_norm(q):
    """Composing many small rotations lets rounding error grow, and a quaternion
    off the unit sphere is no longer a rotation -- it shows up as the wrist
    skewing rather than turning."""
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return [v / n for v in q]


class RigDebug(Node):
    def __init__(self, args):
        super().__init__("rig_debug")
        self.args = args
        self.sel = args.arm
        self.joint_step = DEFAULT_JOINT_STEP
        self.rot_step = DEFAULT_ROT_STEP
        self.pos_step = DEFAULT_POS_STEP

        self.measured, self.joints, self.names, self.active = {}, {}, {}, {}
        self.enabled, self.target = {}, {}
        self.pose_ix = 0

        self.pub = {}
        for name, ns in ARMS.items():
            self.enabled[name] = False
            self.create_subscription(PoseStamped, f"{ns}/ee_pose",
                                     lambda m, n=name: self._on_ee(n, m), 1)
            self.create_subscription(JointState, f"{ns}/joint_states",
                                     lambda m, n=name: self.joints.__setitem__(n, list(m.position)), 1)
            self.create_subscription(String, f"{ns}/pose_names",
                                     lambda m, n=name: self.names.__setitem__(
                                         n, [x.strip(' "[]') for x in m.data.split(",") if x.strip(' "[]')]), 1)
            self.create_subscription(Bool, f"{ns}/active",
                                     lambda m, n=name: self.active.__setitem__(n, m.data), 1)
            self.pub[name] = {
                "pose": self.create_publisher(PoseStamped, f"{ns}/cmd_pose", 1),
                "joints": self.create_publisher(JointState, f"{ns}/cmd_joints", 1),
                "name": self.create_publisher(String, f"{ns}/cmd_pose_name", 1),
                "enable": self.create_publisher(Bool, f"{ns}/enable", 1),
                "zero": self.create_publisher(Bool, f"{ns}/zero", 1),
                "reset": self.create_publisher(Bool, f"{ns}/reset", 1),
                "save": self.create_publisher(String, f"{ns}/save_pose", 1),
            }
        # An armed arm's target is dropped by its agent after 300 ms of silence,
        # so it has to be repeated even when nothing is being pressed.
        self.create_timer(1.0 / REPUBLISH_HZ, self._tick)

    def _on_ee(self, name, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.measured[name] = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    def _tick(self):
        if self.args.dry_run:
            return
        for name, on in self.enabled.items():
            if on and name in self.target:
                self._send(name)

    def _send(self, name):
        pos, quat = self.target[name]
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in pos]
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = [float(v) for v in quat]
        if not self.args.dry_run:
            self.pub[name]["pose"].publish(m)

    # ---- guards ----------------------------------------------------------
    def _ready(self):
        """The selected arm, if it is armed and the agent agrees. Else a reason."""
        name = self.sel
        if not self.enabled.get(name):
            return None, f"{name} not armed -- SPACE"
        if self.active.get(name) is False:
            self.enabled[name] = False
            self.target.pop(name, None)
            return None, (f"{name} was DISARMED by its agent (restart or fault) -- "
                          "commands were going nowhere. SPACE to re-arm.")
        return name, None

    # ---- actions ---------------------------------------------------------
    def cycle_arm(self):
        order = list(ARMS)
        self.sel = order[(order.index(self.sel) + 1) % len(order)]
        self.pose_ix = 0
        have = "" if self.sel in self.measured else "   (no ee_pose -- agent up?)"
        return f"selected {self.sel}{have}"

    def toggle(self):
        name = self.sel
        if self.enabled.get(name):
            self.enabled[name] = False
            self.target.pop(name, None)
            if not self.args.dry_run:
                self.pub[name]["enable"].publish(Bool(data=False))
            return f"{name} DISARMED"
        if name not in self.measured:
            return f"{name}: no ee_pose -- is its agent running?"
        pos, quat = self.measured[name]
        self.target[name] = (list(pos), list(quat))    # anchor: arming is not a move
        self.enabled[name] = True
        if not self.args.dry_run:
            self.pub[name]["enable"].publish(Bool(data=True))
        return f"{name} ARMED at ({pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f})"

    def move_joint(self, index, sign):
        name, why = self._ready()
        if why:
            return why
        cur = self.joints.get(name)
        if cur is None:
            return f"{name}: no joint_states yet"
        if index >= len(cur):
            return f"{name}: only {len(cur)} joints reported"
        self.target.pop(name, None)          # joint and Cartesian must not fight
        want = list(cur[:NUM_ARM_JOINTS])
        want[index] += sign * self.joint_step
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position = [float(v) for v in want]
        if not self.args.dry_run:
            self.pub[name]["joints"].publish(m)
        return f"{name} joint {index + 1} -> {math.degrees(want[index]):+.1f}deg"

    def rotate(self, axis, sign):
        name, why = self._ready()
        if why:
            return why
        pos, quat = self.target[name]
        # Pre-multiply: the delta is in the WORLD frame, so it goes on the left.
        # Post-multiplying would rotate about the tool's own axes instead.
        quat = quat_norm(quat_mul(quat_about(axis, sign * self.rot_step), quat))
        self.target[name] = (pos, quat)
        self._send(name)
        return f"{name} rot {axis}{'+' if sign > 0 else '-'} ({math.degrees(self.rot_step):.0f}deg)"

    def translate(self, axis, sign):
        name, why = self._ready()
        if why:
            return why
        pos, quat = self.target[name]
        pos = list(pos)
        pos["xyz".index(axis)] += sign * self.pos_step
        self.target[name] = (pos, quat)
        self._send(name)
        return f"{name} {axis}{'+' if sign > 0 else '-'} -> ({pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f})"

    def cycle_pose(self):
        names = self.names.get(self.sel) or []
        if not names:
            return f"{self.sel}: no poses reported yet"
        self.pose_ix = (self.pose_ix + 1) % len(names)
        return f"{self.sel} pose: {names[self.pose_ix]}   (ENTER to go)"

    def go_pose(self, explicit=None):
        name, why = self._ready()
        if why:
            return why
        names = self.names.get(name) or []
        pose = explicit or (names[self.pose_ix] if names else None)
        if not pose:
            return f"{name}: no poses to go to"
        if names and pose not in names:
            return f"{name} has no pose {pose!r}. Known: {', '.join(names)}"
        self.target.pop(name, None)
        if not self.args.dry_run:
            self.pub[name]["name"].publish(String(data=pose))
        return f"{name} -> {pose}  (joint-space, several seconds -- WATCH IT)"

    def zero(self):
        name, why = self._ready()
        if why:
            return why
        self.target.pop(name, None)
        self.enabled[name] = False
        if not self.args.dry_run:
            self.pub[name]["zero"].publish(Bool(data=True))
            self.pub[name]["enable"].publish(Bool(data=False))
        return f"{name} re-zeroed and disarmed -- the anchor moved, so SPACE to re-arm"

    def reset(self):
        name = self.sel
        self.enabled[name] = False
        self.target.pop(name, None)
        if not self.args.dry_run:
            self.pub[name]["reset"].publish(Bool(data=True))
        return f"{name} RESET -- its container restarts (~10 s). The arm holds."

    def save(self, pose_name):
        if not self.args.dry_run:
            self.pub[self.sel]["save"].publish(String(data=pose_name))
        return f"{self.sel}: saved current pose as {pose_name!r}"

    def status(self, width):
        name = self.sel
        live = self.active.get(name)
        if self.enabled.get(name) and live is False:
            mark = "!"                      # armed here, not there
        elif self.enabled.get(name):
            mark = "*" if live else "?"
        else:
            mark = "-"
        j = self.joints.get(name)
        jtxt = " ".join(f"{math.degrees(v):+.0f}" for v in j[:6]) if j else "--"
        return (f"[{name}{mark}] j:{jtxt}  step {math.degrees(self.joint_step):.0f}deg/"
                f"{math.degrees(self.rot_step):.0f}deg/{self.pos_step * 100:.1f}cm")[:width]


def read_keys(timeout_s):
    """os.read on the raw fd, NOT sys.stdin.read: sys.stdin is buffered, so
    reading one character pulls the whole chunk into Python's buffer and a
    following select() reports nothing -- an arrow key is then seen as a bare
    ESC and silently does nothing."""
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout_s)[0]:
        return []
    data = os.read(fd, 64)
    if not data:
        raise EOFError("stdin closed")
    text = data.decode(errors="ignore")
    tokens, i = [], 0
    while i < len(text):
        if text[i] == "\x1b":
            if text[i:i + 3] in (UP, DOWN, LEFT, RIGHT):
                tokens.append(text[i:i + 3]); i += 3
            else:
                tokens.append("ESC"); i += 1
        else:
            tokens.append(text[i]); i += 1
    return tokens


def handle(node, key):
    if key == "\t":
        return node.cycle_arm()
    if key == " ":
        return node.toggle()
    if key in JOINT_POS:
        return node.move_joint(JOINT_POS.index(key), +1)
    if key in JOINT_NEG:
        return node.move_joint(JOINT_NEG.index(key), -1)
    if key in ROT_KEYS:
        return node.rotate(*ROT_KEYS[key])
    if key in POS_KEYS:
        return node.translate(*POS_KEYS[key])
    if key == "p":
        return node.cycle_pose()
    if key in ("\r", "\n"):
        return node.go_pose()
    if key == "h":
        return node.go_pose("home")
    if key == "z":
        return node.zero()
    if key == "r":
        return node.reset()
    if key in "[]":
        f = 1 / 1.5 if key == "[" else 1.5
        node.joint_step = max(math.radians(0.5), min(math.radians(20), node.joint_step * f))
        return f"joint step {math.degrees(node.joint_step):.1f} deg"
    if key in "{}":
        f = 1 / 1.5 if key == "{" else 1.5
        node.rot_step = max(math.radians(0.5), min(math.radians(30), node.rot_step * f))
        return f"rotation step {math.degrees(node.rot_step):.1f} deg"
    if key in "-=":
        f = 1 / 1.5 if key == "-" else 1.5
        node.pos_step = max(0.001, min(0.05, node.pos_step * f))
        return f"position step {node.pos_step * 100:.2f} cm"
    return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", default="left", choices=list(ARMS))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    busy = control_lock.refuse_if_busy("rig_debug.py", sys.stderr)
    if busy:
        return busy
    if not sys.stdin.isatty():
        print("needs a real terminal -- `docker compose exec monitor bash`, "
              "not `exec -T`.", file=sys.stderr)
        return 2

    rclpy.init()
    node = RigDebug(args)
    print(__doc__.split("MUTUALLY EXCLUSIVE")[0].split("HOW THIS DIFFERS")[1]
          .split("\n", 6)[-1])
    print(f"  selected: {args.arm}"
          f"{'   [DRY RUN]' if args.dry_run else ''}\n")

    old = termios.tcgetattr(sys.stdin)
    status = "nothing armed -- TAB to select, SPACE to arm"
    try:
        tty.setcbreak(sys.stdin.fileno())
        # ISIG off so Ctrl-C arrives as a BYTE we read, not as a signal. rclpy's
        # SIGINT handler invalidates the context before `finally` runs, so a
        # signal-driven exit publishes into a dead context and does nothing.
        at = termios.tcgetattr(sys.stdin.fileno())
        at[3] &= ~termios.ISIG
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, at)

        end = time.monotonic() + 1.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        import shutil
        while rclpy.ok():
            for _ in range(8):            # drain callbacks, do not starve the timer
                rclpy.spin_once(node, timeout_sec=0.0)
            try:
                keys = read_keys(0.02)
            except EOFError:
                break
            stop = False
            for key in keys:
                if key in ("\x03", "ESC"):
                    stop = True
                    break
                line = handle(node, key)
                if line is not None:
                    status = line
            if stop:
                break
            if keys:
                w = shutil.get_terminal_size((80, 24)).columns
                sys.stdout.write("\r\033[K" + f"  {node.status(30)}  {status}"[:w - 1])
                sys.stdout.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("\n  exiting -- arms left ARMED and holding, on purpose.")
        print("  Disarm from here with SPACE, or:  make arm-stop")
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
