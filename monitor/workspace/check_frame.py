#!/usr/bin/env python3
"""Find out which way an arm really moves, and print the mount frame that fixes it.

    ./check_frame.py --arm left              # DRY RUN -- explains, moves nothing
    ./check_frame.py --arm left --execute    # three 3 cm moves, one per axis

WHY THIS EXISTS. Every consumer in this rig commands one world-aligned frame --
x forward, y left, z up -- and each manipulator container converts that to its
own base frame with ARM_WORLD_X/Y/Z (see manip-arm/docker-compose.yml). Those
three strings describe how the arm is physically BOLTED ON, and nothing in
software can derive them: the arm knows where its end effector is in its own
frame and has no idea which way that frame points in the room. They were set by
hand and have never been checked against the hardware.

A wrong mount frame is not subtle to operate and is very subtle to diagnose: q
moves the arm sideways, w moves it down, and every explanation you reach for --
bad IK, a broken guard, a scaling bug -- is somewhere else entirely. So measure
it instead of reasoning about it.

WHAT IT DOES. For each world axis in turn: anchor on where the arm is, command
a small step along that axis, wait, ask you which way the arm ACTUALLY went,
then put it back. Three answers determine the mounting completely, and the tool
prints the exact lines to paste into docker-compose.yml.

WHAT IT ASSUMES. That you answer in the OPERATOR's frame, which is the one the
rig is commanded in:

    forward = the direction the base drives      = world +x
    left    = your left when facing that way     = world +y
    up      = away from the floor                = world +z

SAFETY. Nothing moves without --execute. Steps are 3 cm by default and the arm
is returned to where it started after each one. Orientation is never commanded.
Run it with the workspace clear and a hand near the power.
"""
import argparse
import math
import os
import sys
import time

import control_lock
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

ARMS = {
    "left": os.environ.get("RIG_LEFT_NS", "/left_arm"),
    "right": os.environ.get("RIG_RIGHT_NS", "/right_arm"),
    "middle": os.environ.get("RIG_MIDDLE_NS", "/middle"),
}

# What the operator can answer, and the world-frame unit vector it means.
ANSWERS = {
    "f": ("forward", (+1, "x")), "b": ("back", (-1, "x")),
    "l": ("left", (+1, "y")),    "r": ("right", (-1, "y")),
    "u": ("up", (+1, "z")),      "d": ("down", (-1, "z")),
}


class FrameCheck(Node):
    def __init__(self, ns, dry_run):
        super().__init__("check_frame")
        self.dry_run = dry_run
        self.measured = None
        self.active = None
        self.create_subscription(PoseStamped, f"{ns}/ee_pose", self._on_ee, 1)
        self.create_subscription(Bool, f"{ns}/active",
                                 lambda m: setattr(self, "active", m.data), 1)
        self.pub_cmd = self.create_publisher(PoseStamped, f"{ns}/cmd_pose", 1)
        self.pub_en = self.create_publisher(Bool, f"{ns}/enable", 1)

    def _on_ee(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.measured = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def enable(self, on):
        if self.dry_run:
            return
        for _ in range(3):
            self.pub_en.publish(Bool(data=bool(on)))
            self.spin(0.05)

    def hold(self, pos, quat, seconds):
        """Publish one target repeatedly -- the agent drops it after 300 ms."""
        if self.dry_run:
            return
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            (m.pose.position.x, m.pose.position.y,
             m.pose.position.z) = [float(v) for v in pos]
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = [float(v) for v in quat]
            self.pub_cmd.publish(m)
            self.spin(0.05)


def ask(question):
    """One letter from the operator. Loops until it is one we understand."""
    while True:
        sys.stdout.write(question)
        sys.stdout.flush()
        got = sys.stdin.readline().strip().lower()[:1]
        if got in ANSWERS:
            return ANSWERS[got]
        if got == "n":
            return ("did not move", None)
        print("   answer f/b (forward/back), l/r (left/right), u/d (up/down), "
              "or n if it did not move")


def solve(current, observed):
    """current: {'x': '+z', ...} as configured. observed: {'x': (sign, axis)}.

    Commanding world axis w currently produces physical direction sign*axis.
    So whatever arm axis w is mapped to is the arm axis that physically points
    along sign*axis -- which is exactly what the NEW mapping for world axis
    `axis` needs, with the sign folded in.
    """
    new = {}
    for w, obs in observed.items():
        if obs is None:
            return None, f"world {w} produced no motion -- cannot solve"
        sign, phys = obs
        spec = current[w]
        cur_sign = -1 if spec.startswith("-") else +1
        new[phys] = f"{'+' if cur_sign * sign > 0 else '-'}{spec.lstrip('+-')}"
    if len(new) != 3:
        return None, ("two world axes moved the arm the same way -- the answers "
                      "are inconsistent, re-run and watch more carefully")
    axes = {"x": 0, "y": 1, "z": 2}
    R = [[0.0] * 3 for _ in range(3)]
    for col, w in enumerate("xyz"):
        spec = new[w]
        R[axes[spec.lstrip("+-")]][col] = -1.0 if spec.startswith("-") else 1.0
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    if abs(det - 1.0) > 1e-6:
        return None, (f"the answers describe a mirror, not a rotation "
                      f"(det={det:+.0f}). One of the three is flipped -- an arm "
                      "cannot be mounted this way, so re-run and check each "
                      "direction against the floor.")
    return new, None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", default="left", choices=list(ARMS))
    ap.add_argument("--step", type=float, default=0.03, help="metres per test move")
    ap.add_argument("--settle", type=float, default=2.5, help="seconds per move")
    ap.add_argument("--current", default="+z,+y,-x",
                    help="ARM_WORLD_X,Y,Z as configured now (docker-compose.yml)")
    ap.add_argument("--execute", action="store_true", help="actually move the arm")
    args = ap.parse_args()

    current = dict(zip("xyz", [v.strip().lower() for v in args.current.split(",")]))
    if len(current) != 3 or any(c.lstrip("+-") not in "xyz" for c in current.values()):
        print(f"--current {args.current!r}: want three of +x/-x/+y/... comma separated",
              file=sys.stderr)
        return 2

    busy = control_lock.refuse_if_busy("check_frame.py", sys.stderr)
    if busy:
        return busy

    rclpy.init()
    node = FrameCheck(ARMS[args.arm], not args.execute)
    print(f"\n  frame check: {args.arm}  ({ARMS[args.arm]})")
    print(f"  configured now: ARM_WORLD_X={current['x']}  "
          f"ARM_WORLD_Y={current['y']}  ARM_WORLD_Z={current['z']}")
    print(f"  step {args.step * 100:.0f} cm per axis"
          f"{'' if args.execute else '   [DRY RUN -- nothing will move]'}\n")

    node.spin(2.0)
    if node.measured is None:
        print(f"  no {ARMS[args.arm]}/ee_pose -- is that arm's agent running?",
              file=sys.stderr)
        return 3

    if not args.execute:
        print("  It would, for each of world x, y and z:")
        print("    anchor where the arm is, move it "
              f"{args.step * 100:.0f} cm along that axis, ask which way it")
        print("    actually went, then put it back.\n")
        print("  Re-run with --execute. Clear the workspace first.")
        return 0

    node.enable(True)
    node.spin(0.3)
    if node.active is False:
        print("  the agent did not arm -- check its log", file=sys.stderr)
        return 4

    observed = {}
    try:
        for w in "xyz":
            anchor_pos, quat = node.measured[0][:], node.measured[1][:]
            target = list(anchor_pos)
            target["xyz".index(w)] += args.step
            print(f"  world +{w}: moving {args.step * 100:.0f} cm ...")
            node.hold(target, quat, args.settle)
            moved = [node.measured[0][i] - anchor_pos[i] for i in range(3)]
            print(f"    reported delta: "
                  f"({moved[0]:+.3f}, {moved[1]:+.3f}, {moved[2]:+.3f}) m"
                  f"   |{math.dist(moved, [0, 0, 0]):.3f}|")
            obs = ask("    which way did the ARM actually move? "
                      "[f]wd [b]ack [l]eft [r]ight [u]p [d]own [n]one: ")
            observed[w] = obs[1]
            print(f"    -> {obs[0]}")
            node.hold(anchor_pos, quat, args.settle)     # put it back
            print("    returned.\n")
    except KeyboardInterrupt:
        print("\n  interrupted -- releasing.")
        node.enable(False)
        return 130

    node.enable(False)

    new, why = solve(current, observed)
    print("  " + "-" * 60)
    if why:
        print(f"  {why}")
        return 5
    unchanged = all(new[w] == current[w] for w in "xyz")
    if unchanged:
        print("  The mount frame is already correct -- every axis moved the way\n"
              "  it was commanded. Whatever else is wrong, it is not this.")
        return 0
    print("  The mount frame is WRONG. Put these in manip-arm/docker-compose.yml")
    print(f"  (in x-arm-env, replacing the current values) for the {args.arm} arm:\n")
    for w in "xyz":
        mark = "   <- changed" if new[w] != current[w] else ""
        print(f"      ARM_WORLD_{w.upper()}: \"{new[w]}\"{mark}")
    print("\n  Then:  docker compose up -d left-arm right-arm")
    print("  If the two arms are mounted as mirror images, run this for the")
    print("  other one too -- they currently share one setting, and mirrored")
    print("  arms need their own.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
