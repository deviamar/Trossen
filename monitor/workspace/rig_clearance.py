#!/usr/bin/env python3
"""Live inter-arm clearance, from the arms' own joint states.

    ./rig_clearance.py             # table, refreshed 5x a second
    ./rig_clearance.py --once      # one reading and exit

Runs the SAME model the agents' collision gate runs (rig/rig_collision.py on
rig/capsules.yaml), fed from /left_arm, /right_arm and /middle joint_states
and /slate/lift/height -- so it shows what the gate WOULD do whether or not
RIG_GATE is on in the arm containers. The one use it was written for: with
the gate off, bring two arms close, read this number, and compare it with a
ruler. If they disagree by more than a couple of centimetres, the mount
positions in sim/rig_params.yaml are wrong and the gate must stay off until
they are fixed.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

RIG = os.environ.get("RIG_DIR", "/home/robot/rig")
sys.path.insert(0, RIG)
from rig_collision import RigGate  # noqa: E402

TOPICS = {"left_arm": "/left_arm/joint_states",
          "right_arm": "/right_arm/joint_states",
          "middle": "/middle/joint_states"}


class Clearance(Node):
    def __init__(self, gate):
        super().__init__("rig_clearance")
        self.gate = gate
        self.seen = {}
        for arm, topic in TOPICS.items():
            self.create_subscription(JointState, topic,
                                     lambda m, a=arm: self._on_js(a, m), 1)
        self.create_subscription(Float32, "/slate/lift/height",
                                 lambda m: self.gate.set_lift(m.data), 1)

    def _on_js(self, arm, msg):
        self.gate.update(arm, msg.position)
        self.seen[arm] = time.monotonic()

    def table(self):
        now = time.monotonic()
        rows = []
        for arm in self.gate.arms:
            age = now - self.seen[arm] if arm in self.seen else None
            v = self.gate.clearance(arm)
            if v.pair is None:
                state = "no peers" if age is not None else "NOT HEARD FROM"
                rows.append(f"  {arm:10s} {state}")
                continue
            me, other, link = v.pair
            flag = "  <-- INSIDE MARGIN" if v.slack_m < 0 else ""
            rows.append(
                f"  {arm:10s} {me:14s} <-> {other}:{link:16s} "
                f"{v.distance_m * 1e3:+7.1f} mm  (margin {v.margin_m * 1e3:4.0f}, "
                f"age {age:4.1f}s){flag}")
        return "\n".join(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--capsules", default=os.path.join(RIG, "capsules.yaml"))
    args = ap.parse_args()
    gate = RigGate(args.capsules)
    rclpy.init()
    node = Clearance(gate)
    try:
        t0 = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() - t0 < 1.0:
                continue
            out = node.table()
            if args.once:
                print(out)
                return 0
            sys.stdout.write("\x1b[2J\x1b[H  inter-arm clearance (capsule surface to surface)\n"
                             + out + "\n\n  Ctrl-C to stop\n")
            sys.stdout.flush()
            t0 = time.monotonic() - 0.8
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
