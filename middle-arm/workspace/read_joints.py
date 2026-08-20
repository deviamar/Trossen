#!/usr/bin/env python3
"""Read joint positions from the middle arm. Read-only -- commands nothing.

    ./read_joints.py              # one snapshot, labelled, rad + deg
    ./read_joints.py --watch      # live-updating table until Ctrl-C
    ./read_joints.py --raw        # one line of bare radians, for piping

Deliberately a plain rclpy subscriber rather than InterbotixManipulatorXS. The
convenience API spins up a full robot object and several of its documented usage
patterns command motion during setup; a subscriber physically cannot. When you
move on to sending commands, that is the moment to switch -- not before.

Joint order comes from the incoming message, not a hardcoded list, so this stays
correct if joint_order in the motor config ever changes.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC = "/middle/joint_states"


class JointReader(Node):
    def __init__(self):
        super().__init__("joint_reader")
        self.latest = None
        # depth=1 keeps only the newest sample: at 100 Hz a deeper queue just
        # hands us stale readings after any pause in processing.
        self.create_subscription(JointState, TOPIC, self._cb, 1)

    def _cb(self, msg):
        self.latest = msg


def wait_for_sample(node, timeout_s=5.0):
    """Spin until a message lands. Returns None on timeout."""
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while rclpy.ok() and node.latest is None:
        if node.get_clock().now().nanoseconds > deadline:
            return None
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.latest


def render(msg):
    rows = ["  joint            rad        deg", "  " + "-" * 32]
    for name, pos in zip(msg.name, msg.position):
        rows.append(f"  {name:<14} {pos:>8.4f}  {math.degrees(pos):>8.2f}")
    return "\n".join(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--watch", action="store_true", help="live table until Ctrl-C")
    ap.add_argument("--raw", action="store_true", help="one line of radians")
    args = ap.parse_args()

    rclpy.init()
    node = JointReader()
    try:
        if wait_for_sample(node) is None:
            print(
                f"No message on {TOPIC} within 5s.\n"
                "Is the arm launched?  ./launch-arm.sh --limp\n"
                "Check the topic exists:  ros2 topic list | grep joint_states",
                file=sys.stderr,
            )
            return 1

        if args.raw:
            print(" ".join(f"{p:.6f}" for p in node.latest.position))
        elif args.watch:
            print("Reading (Ctrl-C to stop). Back-drive a joint to see it move.\n")
            lines = 0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
                block = render(node.latest)
                if lines:  # redraw in place rather than scrolling the terminal
                    sys.stdout.write(f"\033[{lines}A")
                lines = block.count("\n") + 1
                print(block)
        else:
            print(render(node.latest))
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
