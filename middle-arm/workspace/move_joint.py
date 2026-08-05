#!/usr/bin/env python3
"""Command the middle arm's joints, with limits enforced and a dry-run default.

    ./move_joint.py waist 0.2             # absolute radians
    ./move_joint.py --rel waist 0.1       # relative to current position
    ./move_joint.py --rel waist -0.1      # relative, backwards
    ./move_joint.py --deg wrist_rotate 15 # degrees instead of radians
    ./move_joint.py --group 0 -1.8 1.55 0 0.8 0
    ./move_joint.py --sleep               # fold to the configured sleep pose

Nothing is sent without --execute. The default prints the move it *would* make,
including the delta from the current pose, so you can see the size of a motion
before the arm makes it. That default is the point of this script: a mistyped
radian value is indistinguishable from a correct one until the arm moves.

Limits are read from the live robot_description, not hardcoded, so they track
whatever URDF the running driver actually loaded. Targets outside a joint's
range are refused rather than silently clamped -- a clamped move still moves,
just not where you asked, which is the more confusing failure.

Note that every move is profiled over profile_velocity ms (2000 by default, set
in modes_nogripper.yaml), so even a large commanded jump is executed as a ~2 s
sweep rather than a snap.
"""
import argparse
import math
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand

NS = "/middle"
GROUP = "arm"


class ArmCommander(Node):
    def __init__(self):
        super().__init__("move_joint")
        self.joints = None
        self.positions = None
        self.limits = {}

        self.create_subscription(JointState, f"{NS}/joint_states", self._js_cb, 1)
        # robot_description is published transient-local (latched): a late
        # subscriber still receives the last message, which is how we can read
        # the URDF without racing the publisher at startup.
        self.create_subscription(
            String, f"{NS}/robot_description", self._urdf_cb,
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST),
        )
        self.pub_single = self.create_publisher(
            JointSingleCommand, f"{NS}/commands/joint_single", 1)
        self.pub_group = self.create_publisher(
            JointGroupCommand, f"{NS}/commands/joint_group", 1)

    def _js_cb(self, msg):
        self.joints, self.positions = list(msg.name), list(msg.position)

    def _urdf_cb(self, msg):
        for m in re.finditer(
            r"<joint name=\"([a-z_]+)\"[^>]*>(.*?)</joint>", msg.data, re.S
        ):
            lim = re.search(r"lower=\"([-0-9.eE]+)\"\s+upper=\"([-0-9.eE]+)\"", m.group(2))
            if lim:
                self.limits[m.group(1)] = (float(lim.group(1)), float(lim.group(2)))

    def ready(self):
        return self.joints is not None and bool(self.limits)


def spin_until_ready(node, timeout_s=5.0):
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while rclpy.ok() and not node.ready():
        if node.get_clock().now().nanoseconds > deadline:
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return True


def check(node, name, target):
    """Return an error string, or None if the target is in range.

    The message reports how much travel is actually left, because "refusing"
    alone is actively unhelpful when most of the requested move was legal: at
    -1.399 with a -1.885 limit there is 0.486 rad available, and rejecting a
    -0.5 request outright reads as "this joint will not move" rather than
    "your request overshot by 0.015".
    """
    if name not in node.joints:
        return f"unknown joint {name!r}; have: {', '.join(node.joints)}"
    lo, hi = node.limits.get(name, (-math.inf, math.inf))
    if not (lo <= target <= hi):
        cur = node.positions[node.joints.index(name)]
        edge = lo if target < lo else hi
        room = edge - cur
        return (f"{name}: target {target:.4f} is outside limits "
                f"[{lo:.4f}, {hi:.4f}] rad -- refusing\n"
                f"  currently at {cur:.4f}; {abs(room):.4f} rad of travel left "
                f"in that direction (largest valid move: {room:+.4f})\n"
                f"  use --clamp to go as far as the limit allows")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("joint", nargs="?", help="joint name")
    ap.add_argument("value", nargs="?", help="target position (absolute unless --rel)")
    # An explicit flag rather than inferring from a leading +/-: "-1.9" is
    # genuinely ambiguous between a negative absolute target and a relative
    # decrease, and guessing wrong turns a hold-still command into a large move.
    ap.add_argument("--rel", action="store_true",
                    help="treat the value as a delta from the current position")
    ap.add_argument("--group", nargs="+", metavar="V",
                    help=f"command all joints of group '{GROUP}' at once")
    ap.add_argument("--sleep", action="store_true",
                    help="fold to the sleep pose from the motor config")
    ap.add_argument("--deg", action="store_true", help="values are degrees")
    ap.add_argument("--clamp", action="store_true",
                    help="clamp out-of-range targets to the limit instead of refusing")
    ap.add_argument("--execute", action="store_true",
                    help="actually send it (default is dry-run)")
    args = ap.parse_args()

    # sleep_positions from wx250s_nogripper.yaml, gripper already excluded.
    SLEEP = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    rclpy.init()
    node = ArmCommander()
    try:
        if not spin_until_ready(node):
            print(f"No data on {NS}/joint_states or {NS}/robot_description within 5s.\n"
                  "Is the arm launched?  ./launch-arm.sh", file=sys.stderr)
            return 1

        conv = math.radians if args.deg else (lambda v: v)

        if args.sleep or args.group:
            vals = SLEEP if args.sleep else [conv(float(v)) for v in args.group]
            if len(vals) != len(node.joints):
                print(f"--group needs {len(node.joints)} values "
                      f"({', '.join(node.joints)}), got {len(vals)}", file=sys.stderr)
                return 2
            errs = [e for j, v in zip(node.joints, vals) if (e := check(node, j, v))]
            if errs:
                print("\n".join(errs), file=sys.stderr)
                return 3
            print(f"  {'joint':<14} {'current':>9} {'target':>9} {'delta':>9}")
            print("  " + "-" * 44)
            for j, cur, tgt in zip(node.joints, node.positions, vals):
                print(f"  {j:<14} {cur:>9.4f} {tgt:>9.4f} {tgt - cur:>+9.4f}")
            msg = JointGroupCommand(name=GROUP, cmd=[float(v) for v in vals])
            pub, what = node.pub_group, f"group '{GROUP}'"
        else:
            if not args.joint or args.value is None:
                ap.error("give a joint and a value, or use --group / --sleep")
            name = args.joint
            if name not in node.joints:
                print(f"unknown joint {name!r}; have: {', '.join(node.joints)}",
                      file=sys.stderr)
                return 2
            cur = node.positions[node.joints.index(name)]
            target = cur + conv(float(args.value)) if args.rel else conv(float(args.value))
            if args.clamp:
                lo, hi = node.limits.get(name, (-math.inf, math.inf))
                clamped = min(hi, max(lo, target))
                if clamped != target:
                    print(f"  clamped {target:.4f} -> {clamped:.4f} rad (joint limit)")
                    target = clamped
            if (err := check(node, name, target)):
                print(err, file=sys.stderr)
                return 3
            print(f"  {name}: {cur:.4f} -> {target:.4f} rad "
                  f"({math.degrees(cur):.1f} -> {math.degrees(target):.1f} deg, "
                  f"delta {target - cur:+.4f} rad)")
            msg = JointSingleCommand(name=name, cmd=float(target))
            pub, what = node.pub_single, name

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute to move the arm.")
            return 0

        print(f"\n  sending to {what} ...")
        pub.publish(msg)
        # Give the publisher a moment to actually get the message out before the
        # process exits and tears the publisher down underneath it.
        end = node.get_clock().now().nanoseconds + int(1.0e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print("  sent. Motion is profiled over profile_velocity ms (~2 s by default).")
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
