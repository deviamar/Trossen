#!/usr/bin/env python3
"""Wait until the arm agents are actually up, and say plainly which are not.

    ./wait_arms.py                 # wait up to 40 s for all three
    ./wait_arms.py --timeout 60
    ./wait_arms.py --arms /left_arm /right_arm

Exists because a fixed `sleep` before a whole-rig pose move is a guess, and the
way it fails is quiet: the WXAI agents ping their controller, configure the SDK
and re-apply joint limits before they publish anything, which takes longer than
the middle arm's USB serial bring-up. Move the rig too early and arm_ctl moves
only the arms that happened to be ready -- it says so, in one line, in the
middle of a wall of startup output, and the arms that were missed simply stay
where they are.

So: block until every expected arm has published an ee_pose, and if some never
do, name them and exit non-zero. That turns "two arms silently did not move"
into "the left and right arms are not answering", which is the difference
between a puzzle and a power cable.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

DEFAULT_ARMS = ["/left_arm", "/right_arm", "/middle"]


class Waiter(Node):
    def __init__(self, arms):
        super().__init__("wait_arms")
        self.seen = {}
        for ns in arms:
            self.create_subscription(
                PoseStamped, f"{ns}/ee_pose",
                lambda m, n=ns: self.seen.setdefault(n, time.monotonic()), 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arms", nargs="*", default=DEFAULT_ARMS)
    ap.add_argument("--timeout", type=float, default=40.0)
    args = ap.parse_args()

    rclpy.init()
    node = Waiter(args.arms)
    deadline = time.monotonic() + args.timeout
    said = set()
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        for ns in sorted(node.seen):
            if ns not in said:
                said.add(ns)
                print(f"  up: {ns}")
        if len(node.seen) >= len(args.arms):
            break

    missing = [ns for ns in args.arms if ns not in node.seen]
    node.destroy_node()
    rclpy.shutdown()

    if missing:
        print(f"\n  NOT ANSWERING: {', '.join(missing)}")
        print("  These arms will NOT be moved and will not respond to teleop.")
        print("  Check each one's log for the reason:  docker compose logs <arm>")
        return 1
    print(f"  all {len(args.arms)} arms up")
    return 0


if __name__ == "__main__":
    sys.exit(main())
