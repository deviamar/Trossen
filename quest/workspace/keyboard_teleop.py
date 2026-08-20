#!/usr/bin/env python3
"""Drive any part of the rig from the keyboard. The headset-free test path.

    ./keyboard_teleop.py                 # 1 cm / 0.1 rad steps
    ./keyboard_teleop.py --step 0.02
    ./keyboard_teleop.py --dry-run       # print what it would send

This publishes exactly the same topics quest_teleop.py does -- the contract in
docs/topic-contract.md -- so it exercises the whole ROS pipeline (arm_agent,
head_agent, governor) with no headset, no WebRTC, no credentials and no Unity
app. If something is broken, this tells you whether it is the robot side or the
headset side, which is the one question a failed teleop session cannot answer on
its own.

It is NOT a nicer version of move_joint.py. That talks to the arm's SDK
directly; this talks to the ROS contract. Use move_joint.py to check the arm,
this to check the pipeline.

MUST be run on a real terminal (an interactive `docker compose exec quest bash`),
not through `exec -T` -- it puts the tty in cbreak mode to read single keys.

Controls
    1 / 2 / 3       select left arm / right arm / camera arm
    0               select the base
    space           engage / release the selected arm  (engage anchors here)

  with an arm selected
    w / s           +x / -x     (forward / back in the arm's base frame)
    a / d           +y / -y     (left / right)
    q / e           +z / -z     (up / down)
    g / h           gripper open / close, one step
    r               resync the target to where the arm actually is

  with the base selected
    arrow keys      drive / turn, held
    space           stop now

    [ / ]           step size down / up
    ESC or Ctrl-C   release everything and quit

WHY IT SENDS ABSOLUTE POSES. Same reason quest_teleop does: cmd_pose says where
the end effector should be, not how far to move it. This node accumulates the
offsets and owns the target, so a dropped message costs one frame rather than
permanently shifting your frame of reference against the robot's.

THE BASE IS DEAD-MAN, THE ARMS ARE NOT. A held arrow key keeps the base moving
and it stops ~0.3 s after you let go, because a terminal's key-repeat is the
only "still pressed" signal available. An arm step is discrete: press once, the
target moves 1 cm, and it stays there. That asymmetry is deliberate -- a base
that keeps rolling because a key repeated is a collision, an arm that holds a
commanded pose is just an arm.
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32

import quest_config as cfg

UP, DOWN, RIGHT, LEFT = "\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D"

# How long a base command survives without a key repeat. Longer than a
# terminal's repeat interval, shorter than the driver's own 300 ms deadline.
BASE_HOLD_S = 0.3

TARGETS = {
    "1": ("left", cfg.ARM_NS_LEFT, True),
    "2": ("right", cfg.ARM_NS_RIGHT, True),
    "3": ("middle", cfg.MIDDLE_NS, False),
}


class KeyboardTeleop(Node):
    def __init__(self, args):
        super().__init__("keyboard_teleop")
        self.args = args
        self.step = args.step
        self.rot_step = args.rot_step
        self.selected = "1"
        self.base_mode = False

        self.measured = {}     # ns -> (pos, quat_xyzw)
        self.target = {}       # ns -> (pos, quat_xyzw), what we last commanded
        self.engaged = {}
        self.gripper = {}

        self.pub_cmd, self.pub_enable, self.pub_grip = {}, {}, {}
        for _key, (_name, ns, has_grip) in TARGETS.items():
            self.create_subscription(
                PoseStamped, f"{ns}/ee_pose", lambda m, n=ns: self._on_ee(n, m), 1)
            self.pub_cmd[ns] = self.create_publisher(PoseStamped, f"{ns}/cmd_pose", 1)
            self.pub_enable[ns] = self.create_publisher(Bool, f"{ns}/enable", 1)
            if has_grip:
                self.pub_grip[ns] = self.create_publisher(Float32, f"{ns}/cmd_gripper", 1)
            self.engaged[ns] = False
            self.gripper[ns] = cfg.GRIPPER_OPEN

        self.pub_base = self.create_publisher(
            Twist, f"{cfg.BASE_NS}/cmd_vel_teleop", 1)
        self.base_cmd = (0.0, 0.0)
        self.base_stamp = 0.0

        # The base needs a repeating publisher: /cmd_vel is a setpoint with a
        # 300 ms deadline, so a single message would buy 300 ms of motion.
        self.create_timer(1.0 / 20.0, self._base_tick)

    # ---- state -----------------------------------------------------------
    def _on_ee(self, ns, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.measured[ns] = (np.array([p.x, p.y, p.z]),
                             np.array([o.x, o.y, o.z, o.w]))

    def ns(self):
        return TARGETS[self.selected][1]

    def name(self):
        return TARGETS[self.selected][0]

    # ---- arms ------------------------------------------------------------
    def toggle_engage(self):
        ns = self.ns()
        if self.engaged[ns]:
            self.engaged[ns] = False
            self._publish_enable(ns, False)
            self.target.pop(ns, None)
            return f"{ns} released"

        if ns not in self.measured:
            # Anchoring on a pose we do not have would send the arm to
            # wherever the last target happened to be.
            return (f"cannot engage {ns}: no {ns}/ee_pose. "
                    "Is its agent running?")

        # Anchor: the first commanded pose IS the measured pose, so engaging
        # never moves the arm.
        self.target[ns] = tuple(v.copy() for v in self.measured[ns])
        self.engaged[ns] = True
        self._publish_enable(ns, True)
        self._publish_target(ns)
        return f"{ns} ENGAGED (anchored where it is)"

    def nudge(self, axis, sign):
        ns = self.ns()
        if not self.engaged.get(ns):
            return "not engaged -- press space first"
        pos, quat = self.target[ns]
        pos = pos.copy()
        pos["xyz".index(axis)] += sign * self.step
        self.target[ns] = (pos, quat)
        self._publish_target(ns)
        return f"{ns} -> ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"

    def resync(self):
        ns = self.ns()
        if ns not in self.measured:
            return f"no {ns}/ee_pose yet"
        self.target[ns] = tuple(v.copy() for v in self.measured[ns])
        p = self.target[ns][0]
        return f"{ns} resynced to ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"

    def gripper_step(self, opening_delta):
        ns = self.ns()
        if ns not in self.pub_grip:
            return f"{ns} has no gripper"
        v = self.gripper[ns] + opening_delta
        v = max(cfg.GRIPPER_CLOSED, min(cfg.GRIPPER_OPEN, v))
        self.gripper[ns] = v
        if not self.args.dry_run:
            m = Float32()
            m.data = float(v)
            self.pub_grip[ns].publish(m)
        return f"{ns} gripper {v * 1000:.1f} mm"

    def _publish_enable(self, ns, on):
        if self.args.dry_run:
            return
        m = Bool()
        m.data = bool(on)
        self.pub_enable[ns].publish(m)

    def _publish_target(self, ns):
        pos, quat = self.target[ns]
        if self.args.dry_run:
            return
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in pos]
        m.pose.orientation.x = float(quat[0])
        m.pose.orientation.y = float(quat[1])
        m.pose.orientation.z = float(quat[2])
        m.pose.orientation.w = float(quat[3])
        self.pub_cmd[ns].publish(m)

    def republish_engaged(self):
        """Arm agents drop their target after CMD_TIMEOUT_S of silence."""
        for ns, on in self.engaged.items():
            if on and ns in self.target:
                self._publish_target(ns)

    # ---- base ------------------------------------------------------------
    def drive(self, lin, ang):
        self.base_cmd = (lin, ang)
        self.base_stamp = time.monotonic()

    def _base_tick(self):
        lin, ang = self.base_cmd
        if time.monotonic() - self.base_stamp > BASE_HOLD_S:
            lin, ang = 0.0, 0.0
            self.base_cmd = (0.0, 0.0)
        if self.args.dry_run:
            return
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub_base.publish(m)
        # Keep engaged arms alive on the same tick, for the same reason.
        self.republish_engaged()

    def release_all(self):
        for ns, on in list(self.engaged.items()):
            if on:
                self.engaged[ns] = False
                self._publish_enable(ns, False)
        self.base_cmd = (0.0, 0.0)
        if not self.args.dry_run:
            self.pub_base.publish(Twist())


def read_keys(timeout_s):
    """Every key token readable within timeout_s.

    os.read on the raw fd, NOT sys.stdin.read. sys.stdin is a buffered
    TextIOWrapper: reading one character pulls a whole chunk out of the OS
    buffer into Python's own, so a following select() on the descriptor reports
    "nothing to read" while the rest of an escape sequence sits in userspace --
    the arrow key is then seen as a bare ESC and silently does nothing.
    select() and buffered IO cannot be mixed on one stream. See the longer note
    in ../../slate-base/workspace/teleop_keyboard.py, which hit it first.
    """
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
                tokens.append(text[i:i + 3])
                i += 3
            else:
                tokens.append("ESC")
                i += 1
        else:
            tokens.append(text[i])
            i += 1
    return tokens


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", type=float, default=0.01, help="metres per press")
    ap.add_argument("--rot-step", type=float, default=0.1, help="radians per press")
    ap.add_argument("--grip-step", type=float, default=0.005, help="metres per press")
    ap.add_argument("--dry-run", action="store_true", help="print, publish nothing")
    args = ap.parse_args()

    if not sys.stdin.isatty():
        print("needs a real terminal -- run from `docker compose exec quest bash`, "
              "not `exec -T`.", file=sys.stderr)
        return 2

    rclpy.init()
    node = KeyboardTeleop(args)
    print(__doc__.split("Controls")[1].split("WHY IT SENDS")[0])
    print(f"  step {args.step * 100:.1f} cm"
          f"{'   [DRY RUN]' if args.dry_run else ''}")
    print(f"  selected: {node.name()} ({node.ns()})\n")

    old = termios.tcgetattr(sys.stdin)
    status = ""
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            try:
                keys = read_keys(0.02)
            except EOFError:
                break
            if not keys:
                continue
            # One token per tick is enough here: every action is discrete, and
            # draining the rest next tick keeps a held key from running away.
            key = keys[0]

            if key in ("\x03", "ESC"):
                break
            elif key in TARGETS:
                node.selected, node.base_mode = key, False
                status = f"selected {node.name()} ({node.ns()})"
            elif key == "0":
                node.base_mode = True
                status = f"selected the base ({cfg.BASE_NS})"
            elif key == " ":
                if node.base_mode:
                    node.drive(0.0, 0.0)
                    status = "base stop"
                else:
                    status = node.toggle_engage()
            elif key in (UP, DOWN, LEFT, RIGHT):
                lin = cfg.BASE_MAX_VEL_X if key == UP else -cfg.BASE_MAX_VEL_X if key == DOWN else 0.0
                ang = cfg.BASE_MAX_VEL_Z if key == LEFT else -cfg.BASE_MAX_VEL_Z if key == RIGHT else 0.0
                node.drive(lin, ang)
                status = f"base {lin:+.2f} m/s {ang:+.2f} rad/s"
            elif key in "wsadqe":
                axis = {"w": "x", "s": "x", "a": "y", "d": "y", "q": "z", "e": "z"}[key]
                sign = 1 if key in "waq" else -1
                status = node.nudge(axis, sign)
            elif key == "g":
                status = node.gripper_step(+args.grip_step)
            elif key == "h":
                status = node.gripper_step(-args.grip_step)
            elif key == "r":
                status = node.resync()
            elif key == "[":
                node.step = max(0.001, node.step / 1.5)
                status = f"step {node.step * 100:.2f} cm"
            elif key == "]":
                node.step = min(0.10, node.step * 1.5)
                status = f"step {node.step * 100:.2f} cm"
            else:
                continue

            sys.stdout.write(f"\r\033[K  {status}")
            sys.stdout.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if rclpy.ok():
            node.release_all()
        print("\n  released everything.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
