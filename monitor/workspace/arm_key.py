#!/usr/bin/env python3
"""Jog an arm in Cartesian space from the keyboard.

    ./arm_key.py                       # starts on the left arm, 1 cm steps
    ./arm_key.py --arm /right_arm
    ./arm_key.py --step 0.005          # finer
    ./arm_key.py --dry-run             # print, publish nothing

Controls
    1 / 2 / 3       select left arm / right arm / camera arm
    space           ENGAGE / release the selected arm

  once engaged
    up / down       +z / -z        (up and down in the world)
    w / s           +x / -x        (away from / toward the base)
    a / d           +y / -y        (left / right)
    g / h           gripper open / close, one step
    r               resync the target to where the arm actually is
    [ / ]           step size down / up
    ESC or Ctrl-C   release everything and quit

WHICH WAY IS WHICH.

The frame is the arm's own base frame, REP-103:
    +x forward out of the base
    +y to its left
    +z up

That is what the SDK's get_cartesian_positions() returns and what the
controller solves against. "Forward" depends on which way the arm is
bolted down.

ENGAGING NEVER MOVES ANYTHING.

Space only changes the /enable state.

When engaging, this node records the arm's current measured pose as the
initial target, but DOES NOT publish a command pose. The arm therefore
cannot move merely because space was pressed.

The first /cmd_pose message is sent only after an actual direction key
is pressed.

DELTAS IN, ABSOLUTE OUT.

Each movement key modifies a target this node holds. The complete absolute
target is then published.

The arm agent drops its target after 300 ms of silence, so while engaged
this node republishes the current target at REPUBLISH_HZ.

MUST be run on a real terminal:

    docker compose exec monitor bash

Do NOT use:

    docker compose exec -T ...

because single-key input requires a real TTY.

ONE COMMANDER AT A TIME.

quest_teleop, keyboard_teleop and this node all publish the same command
topics and the newest message wins.

Stop the quest container first:

    docker compose stop quest
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32


# ANSI escape sequences for arrow keys.
UP = "\x1b[A"
DOWN = "\x1b[B"
RIGHT = "\x1b[C"
LEFT = "\x1b[D"


TARGETS = {
    "1": ("left arm", "/left_arm", True),
    "2": ("right arm", "/right_arm", True),
    "3": ("camera arm", "/middle", False),
}


GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.0


# arm_agent drops its target after 300 ms of silence.
# Republish faster than that while engaged.
REPUBLISH_HZ = 20.0


class ArmKey(Node):
    def __init__(self, args):
        super().__init__("arm_key")

        self.args = args
        self.step = args.step
        self.selected = args.arm

        # Latest measured EE pose:
        #   ns -> ([x, y, z], [qx, qy, qz, qw])
        self.measured = {}

        # Absolute command target:
        #   ns -> ([x, y, z], [qx, qy, qz, qw])
        self.target = {}

        # Whether this node has explicitly engaged the arm.
        self.engaged = {}

        # Current gripper command.
        self.gripper = {}

        self.pub_cmd = {}
        self.pub_en = {}
        self.pub_grip = {}

        for _key, (_label, ns, has_grip) in TARGETS.items():

            self.create_subscription(
                PoseStamped,
                f"{ns}/ee_pose",
                lambda msg, namespace=ns: self._on_ee(namespace, msg),
                1,
            )

            self.pub_cmd[ns] = self.create_publisher(
                PoseStamped,
                f"{ns}/cmd_pose",
                1,
            )

            self.pub_en[ns] = self.create_publisher(
                Bool,
                f"{ns}/enable",
                1,
            )

            if has_grip:
                self.pub_grip[ns] = self.create_publisher(
                    Float32,
                    f"{ns}/cmd_gripper",
                    1,
                )

            self.engaged[ns] = False
            self.gripper[ns] = GRIPPER_OPEN

        self.create_timer(
            1.0 / REPUBLISH_HZ,
            self._republish,
        )

    def _on_ee(self, ns, msg):
        """Store the latest measured end-effector pose."""

        p = msg.pose.position
        o = msg.pose.orientation

        self.measured[ns] = (
            [p.x, p.y, p.z],
            [o.x, o.y, o.z, o.w],
        )

    def label(self):
        """Return human-readable name of currently selected arm."""

        for _key, (label, ns, _has_gripper) in TARGETS.items():
            if ns == self.selected:
                return label

        return self.selected

    def toggle(self):
        """Engage or release the selected arm.

        IMPORTANT:
        Engaging ONLY changes the enable state.

        We initialize the internal target from the measured pose, but
        deliberately DO NOT publish /cmd_pose here.

        This guarantees that pressing space by itself cannot create a
        Cartesian motion command.
        """

        ns = self.selected

        # ------------------------------------------------------------
        # RELEASE
        # ------------------------------------------------------------

        if self.engaged[ns]:
            self.engaged[ns] = False

            self._enable(ns, False)

            # Remove our command target. It will be recreated from the
            # measured pose on the next engage.
            self.target.pop(ns, None)

            return f"{ns} RELEASED"

        # ------------------------------------------------------------
        # ENGAGE
        # ------------------------------------------------------------

        if ns not in self.measured:
            return (
                f"cannot engage {ns}: no {ns}/ee_pose. "
                "Is its container up and its agent running?"
            )

        measured_pos, measured_quat = self.measured[ns]

        # Anchor our internal target to the current measured pose.
        #
        # This is NOT sent to /cmd_pose yet.
        self.target[ns] = (
            list(measured_pos),
            list(measured_quat),
        )

        self.engaged[ns] = True

        # Space ONLY changes enable state.
        self._enable(ns, True)

        return (
            f"{ns} ENGAGED at "
            f"({measured_pos[0]:+.3f}, "
            f"{measured_pos[1]:+.3f}, "
            f"{measured_pos[2]:+.3f})"
        )

    def nudge(self, axis, sign):
        """Move the internal Cartesian target and publish it."""

        ns = self.selected

        if not self.engaged.get(ns, False):
            return "not engaged -- press space first"

        if ns not in self.target:
            return "no target -- press r or re-engage"

        pos, quat = self.target[ns]

        pos = list(pos)

        axis_index = {
            "x": 0,
            "y": 1,
            "z": 2,
        }[axis]

        pos[axis_index] += sign * self.step

        self.target[ns] = (
            pos,
            list(quat),
        )

        # THIS is the first point where a Cartesian command is published.
        self._send(ns)

        err = ""

        if ns in self.measured:
            measured_pos = self.measured[ns][0]
            distance = math.dist(pos, measured_pos)

            if distance > 0.02:
                err = (
                    f"   (arm is "
                    f"{distance * 1000:.0f} mm behind)"
                )

        return (
            f"{ns} -> "
            f"({pos[0]:+.3f}, "
            f"{pos[1]:+.3f}, "
            f"{pos[2]:+.3f})"
            f"{err}"
        )

    def resync(self):
        """Reset the command target to the measured pose.

        No command is published. This prevents r from itself causing
        motion.
        """

        ns = self.selected

        if ns not in self.measured:
            return f"no {ns}/ee_pose yet"

        measured_pos, measured_quat = self.measured[ns]

        self.target[ns] = (
            list(measured_pos),
            list(measured_quat),
        )

        return (
            f"{ns} resynced to "
            f"({measured_pos[0]:+.3f}, "
            f"{measured_pos[1]:+.3f}, "
            f"{measured_pos[2]:+.3f})"
        )

    def grip(self, delta):
        """Move the gripper by one step."""

        ns = self.selected

        if ns not in self.pub_grip:
            return f"{ns} has no gripper"

        value = max(
            GRIPPER_CLOSED,
            min(
                GRIPPER_OPEN,
                self.gripper[ns] + delta,
            ),
        )

        self.gripper[ns] = value

        if not self.args.dry_run:
            msg = Float32()
            msg.data = float(value)
            self.pub_grip[ns].publish(msg)

        return (
            f"{ns} gripper "
            f"{value * 1000:.1f} mm"
        )

    def _enable(self, ns, on):
        """Publish enable/disable state.

        Publish multiple times because a missed enable message could
        otherwise leave the arm in the wrong state.
        """

        if self.args.dry_run:
            return

        msg = Bool()
        msg.data = bool(on)

        for _ in range(3):
            self.pub_en[ns].publish(msg)

    def _send(self, ns):
        """Publish the current absolute Cartesian target."""

        if self.args.dry_run:
            return

        if ns not in self.target:
            return

        pos, quat = self.target[ns]

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        self.pub_cmd[ns].publish(msg)

    def _republish(self):
        """Keep engaged targets alive."""

        for ns, engaged in self.engaged.items():

            if not engaged:
                continue

            # Important:
            # We only republish after engagement has happened.
            #
            # The target was initialized to the measured pose when
            # space was pressed, so these republished messages should
            # not cause a jump by themselves.
            self._send(ns)

    def release_all(self):
        """Disable every arm controlled by this node."""

        for ns, engaged in list(self.engaged.items()):

            if engaged:
                self.engaged[ns] = False
                self._enable(ns, False)


# ----------------------------------------------------------------------
# Keyboard input
# ----------------------------------------------------------------------

class KeyReader:
    """Robust single-key reader for a real terminal.

    Escape sequences such as arrow keys may arrive from the terminal
    in multiple os.read() calls.

    Therefore we maintain a small byte buffer instead of assuming that
    ESC + '[' + 'A' arrives atomically.
    """

    ESCAPE_SEQUENCES = {
        b"\x1b[A": UP,
        b"\x1b[B": DOWN,
        b"\x1b[C": RIGHT,
        b"\x1b[D": LEFT,
    }

    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()

    def read(self, timeout_s):
        """Return every complete key token currently available."""

        readable, _, _ = select.select(
            [self.fd],
            [],
            [],
            timeout_s,
        )

        if readable:
            data = os.read(self.fd, 64)

            if not data:
                raise EOFError("stdin closed")

            self.buffer.extend(data)

        return self._parse()

    def _parse(self):
        tokens = []

        while self.buffer:

            # ----------------------------------------------------------
            # Arrow keys / ANSI escape sequences
            # ----------------------------------------------------------

            if self.buffer[0] == 0x1B:

                # If we have a complete known escape sequence, consume it.
                matched = False

                for raw, token in self.ESCAPE_SEQUENCES.items():

                    if self.buffer.startswith(raw):
                        tokens.append(token)
                        del self.buffer[:len(raw)]
                        matched = True
                        break

                if matched:
                    continue

                # We have an ESC but not enough bytes yet.
                #
                # Don't immediately interpret it as ESC if it could
                # still become an arrow key.
                if len(self.buffer) < 3:
                    break

                # Unknown escape sequence.
                del self.buffer[0]
                tokens.append("ESC")
                continue

            # ----------------------------------------------------------
            # Normal character
            # ----------------------------------------------------------

            byte = bytes([self.buffer[0]])
            del self.buffer[0]

            tokens.append(
                byte.decode(errors="ignore")
            )

        return tokens


def handle(node, key, args):
    """Map one keyboard token to one action."""

    # Arm selection.
    if key in TARGETS:
        node.selected = TARGETS[key][1]

        return (
            f"selected {node.label()} "
            f"({node.selected})"
        )

    # Engage / release.
    if key == " ":
        return node.toggle()

    # Cartesian movement.
    if key == UP:
        return node.nudge("z", +1)

    if key == DOWN:
        return node.nudge("z", -1)

    if key == "w":
        return node.nudge("x", +1)

    if key == "s":
        return node.nudge("x", -1)

    if key == "a":
        return node.nudge("y", +1)

    if key == "d":
        return node.nudge("y", -1)

    # Gripper.
    if key == "g":
        return node.grip(+args.grip_step)

    if key == "h":
        return node.grip(-args.grip_step)

    # Resync.
    if key == "r":
        return node.resync()

    # Step size.
    if key == "[":
        node.step = max(
            0.001,
            node.step / 1.5,
        )

        return (
            f"step "
            f"{node.step * 100:.2f} cm"
        )

    if key == "]":
        node.step = min(
            0.05,
            node.step * 1.5,
        )

        return (
            f"step "
            f"{node.step * 100:.2f} cm"
        )

    return None


def main():

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    ap.add_argument(
        "--arm",
        default="/left_arm",
        help="which arm to start on",
    )

    ap.add_argument(
        "--step",
        type=float,
        default=0.01,
        help="metres per press",
    )

    ap.add_argument(
        "--grip-step",
        type=float,
        default=0.005,
        help="metres per press",
    )

    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="print, publish nothing",
    )

    args = ap.parse_args()

    valid_arms = {
        TARGETS[key][1]
        for key in TARGETS
    }

    if args.arm not in valid_arms:
        print(
            f"invalid arm {args.arm!r}; "
            f"choose from {sorted(valid_arms)}",
            file=sys.stderr,
        )
        return 2

    # Keyboard control requires a real TTY.
    if not sys.stdin.isatty():
        print(
            "needs a real terminal -- use "
            "`docker compose exec monitor bash`, "
            "not `exec -T`.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()

    node = ArmKey(args)

    print(
        "\n".join(
            __doc__.split("Controls")[1]
            .split("WHICH WAY")[0]
            .strip()
            .splitlines()
        )
    )

    print(
        f"  step {args.step * 100:.1f} cm"
        f"{'   [DRY RUN]' if args.dry_run else ''}"
    )

    print(
        f"  selected: "
        f"{node.label()} "
        f"({node.selected})"
    )

    print("  waiting for ee_pose ...")

    old_terminal_settings = termios.tcgetattr(
        sys.stdin
    )

    status = ""

    try:

        # cbreak:
        # - characters become available immediately
        # - Ctrl-C remains available
        # - no need to press Enter
        tty.setcbreak(
            sys.stdin.fileno()
        )

        # Give the ROS subscriptions time to receive poses.
        end = time.monotonic() + 2.0

        while (
            rclpy.ok()
            and time.monotonic() < end
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

        found = (
            ", ".join(
                sorted(node.measured)
            )
            or "none yet"
        )

        print(
            f"  arms publishing ee_pose: "
            f"{found}\n"
        )

        key_reader = KeyReader(
            sys.stdin.fileno()
        )

        while rclpy.ok():

            # Process ROS callbacks.
            rclpy.spin_once(
                node,
                timeout_sec=0.0,
            )

            try:
                keys = key_reader.read(
                    0.02
                )

            except EOFError:
                break

            stop = False

            for key in keys:

                # Ctrl-C / ESC.
                if key in (
                    "\x03",
                    "ESC",
                ):
                    stop = True
                    break

                line = handle(
                    node,
                    key,
                    args,
                )

                if line is not None:
                    status = line

            if stop:
                break

            if keys and status:

                sys.stdout.write(
                    f"\r\033[K  {status}"
                )

                sys.stdout.flush()

    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):
        pass

    finally:

        # Always restore the user's terminal.
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )

        # Safety: disable every arm before quitting.
        # No rclpy.ok() guard: if the context somehow died the calls are
        # no-ops, but skipping them outright is how the arms stayed enabled.
        try:
            node.release_all()
        except Exception:
            pass

        print(
            "\n  released everything."
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())