#!/usr/bin/env python3
"""Command any arm over ROS, from outside its container. Dry-run by default.

    ./arm_ctl.py list                          # what each arm can be sent to
    ./arm_ctl.py state                         # where every arm is now
    ./arm_ctl.py go ready --arm /left_arm      # DRY RUN -- prints, sends nothing
    ./arm_ctl.py go ready --arm /left_arm --execute
    ./arm_ctl.py gripper 0.02 --arm /left_arm --execute
    ./arm_ctl.py joints 0 1.0 0.5 0 0 0 --arm /right_arm --execute
    ./arm_ctl.py stop                          # release every arm

The arms' equivalent of drive_test.py: the smallest thing that proves the ROS
path works, from a container with no SDK and no hardware access. If this moves
an arm, then DDS, the agent, the controller and the arm are all fine.

IT IS NOT pose.py. That runs inside the arm's container and talks to the SDK
directly over Ethernet. This publishes topics from outside and never touches the
hardware. Use pose.py to check the arm; use this to check the pipeline -- and
note they cannot both run, because arm_agent.py holds the arm's only connection.

NAMES, NOT VALUES. `go ready` publishes the string "ready" and the arm looks it
up in its OWN config/poses.yaml. This container never reads that file. The same
name means a different point in space on each arm, so the arm that owns the pose
is the only thing that should resolve it.

WHAT --execute ACTUALLY DOES. Publishes enable=true, then the command. The agent
refuses anything while disabled, so a stray command from a crashed script cannot
move an arm that nobody has enabled.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

ARMS = ["/left_arm", "/right_arm", "/middle"]

JOINT_LABELS = ["base", "shoulder", "elbow", "forearm_roll",
                "wrist_angle", "wrist_rotate", "gripper"]


class ArmCtl(Node):
    def __init__(self, arms):
        super().__init__("arm_ctl")
        self.arms = arms
        self.ee = {}
        self.js = {}
        self.names = {}
        self.active = {}
        self.pub_name, self.pub_enable, self.pub_grip, self.pub_joints = {}, {}, {}, {}
        self.pub_save = {}

        for ns in arms:
            self.create_subscription(PoseStamped, f"{ns}/ee_pose",
                                     lambda m, n=ns: self.ee.__setitem__(n, m), 1)
            self.create_subscription(JointState, f"{ns}/joint_states",
                                     lambda m, n=ns: self.js.__setitem__(n, m), 1)
            self.create_subscription(String, f"{ns}/pose_names",
                                     lambda m, n=ns: self._on_names(n, m), 1)
            self.create_subscription(Bool, f"{ns}/active",
                                     lambda m, n=ns: self.active.__setitem__(n, m.data), 1)
            self.pub_name[ns] = self.create_publisher(String, f"{ns}/cmd_pose_name", 1)
            self.pub_save[ns] = self.create_publisher(String, f"{ns}/save_pose", 1)
            self.pub_enable[ns] = self.create_publisher(Bool, f"{ns}/enable", 1)
            self.pub_grip[ns] = self.create_publisher(Float32, f"{ns}/cmd_gripper", 1)
            self.pub_joints[ns] = self.create_publisher(JointState, f"{ns}/cmd_joints", 1)

    def _on_names(self, ns, msg):
        try:
            self.names[ns] = json.loads(msg.data)
        except ValueError:
            pass

    def settle(self, seconds=2.5):
        """Discovery is not instant; asking immediately reports an empty graph."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def agent_up(self, ns):
        return ns in self.ee or ns in self.js

    def save_pose(self, ns, name):
        """Ask the agent to record its current joints under `name`.

        Goes through the agent because the driver connection is exclusive: while
        the rig is up, pose.py cannot reach the arm at all.
        """
        m = String()
        m.data = name
        self.pub_save[ns].publish(m)

    def enable(self, ns, on):
        m = Bool()
        m.data = bool(on)
        for _ in range(3):          # depth-1 topic; a dropped enable is a stuck arm
            self.pub_enable[ns].publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)


def show_state(node):
    for ns in node.arms:
        if not node.agent_up(ns):
            print(f"  {ns:<12} agent not running")
            continue
        act = node.active.get(ns)
        print(f"  {ns:<12} {'ENABLED' if act else 'idle'}")
        js = node.js.get(ns)
        if js:
            row = "  ".join(
                f"{JOINT_LABELS[i] if i < len(JOINT_LABELS) else i}={p:+.3f}"
                for i, p in enumerate(list(js.position)[:7]))
            print(f"    joints  {row}")
        ee = node.ee.get(ns)
        if ee:
            p = ee.pose.position
            print(f"    ee      ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")


# The arms own config/poses.yaml; it is bind-mounted here read-only purely so a
# dry run can show the target. Missing mount is not an error -- the preview just
# falls back to showing the name, exactly as it did before.
ARM_CONFIG = os.environ.get("ARM_CONFIG_DIR", "/home/robot/arm-config")


def _pose_values(ns, name):
    path = os.path.join(ARM_CONFIG, "poses.yaml")
    try:
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return None
    # ARM_NAME is the namespace without its slash: /left_arm -> left_arm.
    entry = data.get(ns.strip("/"), {}).get(name)
    return [float(v) for v in entry[:6]] if entry else None


def _plan_go(node, pose, ns):
    """Print what `pose` means for this arm. False if it cannot be done."""
    if not node.agent_up(ns):
        print(f"  {ns:<12} no agent -- skipped", file=sys.stderr)
        return False
    names = node.names.get(ns, [])
    if names and pose not in names:
        print(f"  {ns:<12} has no pose {pose!r}. Known: {', '.join(names) or '(none)'}",
              file=sys.stderr)
        return False

    print(f"  plan: move {ns} to pose {pose!r}")
    js = node.js.get(ns)
    if not js:
        return True
    cur = list(js.position)[:6]
    print("  from: " + "  ".join(f"{v:+.3f}" for v in cur))
    want = _pose_values(ns, pose)
    if want:
        print("  to:   " + "  ".join(f"{v:+.3f}" for v in want))
        delta = [w - c for w, c in zip(want, cur)]
        print("  move: " + "  ".join(f"{d:+.3f}" for d in delta))
        # Called out by name. A joint that has to travel most of a turn is the
        # one that snags a cable or sweeps through something, and it is
        # invisible in a row of radians -- the number just looks like the others.
        for i, d in enumerate(delta):
            if abs(d) > 1.0:
                print(f"  NOTE: joint {i + 1} travels {math.degrees(d):+.0f} deg "
                      f"-- check clearance and cable routing")
    return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --arm and --execute go on a parent parser so they are accepted on EITHER
    # side of the subcommand. With argparse's default behaviour they would only
    # work before it, and `go ready --arm /left_arm` -- which is the order
    # anyone actually types -- would fail with "unrecognized arguments".
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--arm", default=None,
                        help=f"namespace (default: all of {', '.join(ARMS)})")
    common.add_argument("--execute", action="store_true", help="actually send it")

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[common])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", parents=[common], help="poses each arm knows")
    sub.add_parser("state", parents=[common], help="where the arms are")
    sub.add_parser("stop", parents=[common], help="release every arm (enable=false)")
    p = sub.add_parser("go", parents=[common], help="move to a named pose")
    p.add_argument("name")
    p = sub.add_parser("gripper", parents=[common], help="set the gripper opening, metres")
    p.add_argument("value", type=float)
    p = sub.add_parser("save", parents=[common],
                       help="record where an arm is now, under a name")
    p.add_argument("name")
    p = sub.add_parser("joints", parents=[common], help="absolute joint vector, radians")
    p.add_argument("values", nargs="+", type=float)
    args = ap.parse_args()

    arms = [args.arm] if args.arm else ARMS

    rclpy.init()
    node = ArmCtl(arms)
    try:
        node.settle()

        if args.cmd == "state":
            show_state(node)
            return 0

        if args.cmd == "list":
            for ns in arms:
                if not node.agent_up(ns):
                    print(f"  {ns:<12} agent not running")
                    continue
                names = node.names.get(ns)
                if names is None:
                    print(f"  {ns:<12} (no {ns}/pose_names yet -- old agent?)")
                else:
                    print(f"  {ns:<12} {', '.join(names) or '(none saved)'}")
            return 0

        if args.cmd == "stop":
            for ns in arms:
                node.enable(ns, False)
                print(f"  {ns} released")
            return 0

        # `go` and `save` name a POSE, which every arm interprets in its own
        # joint space -- so fanning them out is meaningful, and "all arms to
        # home" is a thing you genuinely want as one command. `joints` and
        # `gripper` carry raw numbers, where the same vector means a different
        # posture per arm, so those still demand --arm.
        if args.arm:
            targets = [args.arm]
        elif args.cmd in ("go", "save"):
            targets = [ns for ns in arms if node.agent_up(ns)]
            if not targets:
                print("  no arm agents are up.", file=sys.stderr)
                return 3
            print(f"  ALL ARMS: {', '.join(targets)}")
        else:
            print(f"give --arm; '{args.cmd}' takes raw numbers, and the same\n"
                  "values mean a different posture on each arm.", file=sys.stderr)
            return 2

        if args.cmd == "save":
            if not args.execute:
                print(f"  plan: save each arm's CURRENT pose as {args.name!r}")
                for ns in targets:
                    js = node.js.get(ns)
                    if js:
                        print(f"  {ns:<12} " + "  ".join(
                            f"{v:+.3f}" for v in list(js.position)[:6]))
                print("\n  DRY RUN -- nothing written. Re-run with --execute.")
                return 0
            for ns in targets:
                node.save_pose(ns, args.name)
                print(f"  {ns} -> saved as {args.name!r}")
            for _ in range(40):
                rclpy.spin_once(node, timeout_sec=0.05)
            return 0

        if args.cmd == "go":
            ok = [ns for ns in targets if _plan_go(node, args.name, ns)]
            if not ok:
                return 2
            if not args.execute:
                print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
                return 0
            print(f"\n  MOVING {len(ok)} ARM(S) IN 2 SECONDS -- Ctrl-C to abort.")
            time.sleep(2.0)
            for ns in ok:
                node.enable(ns, True)
                m = String()
                m.data = args.name
                node.pub_name[ns].publish(m)
            for _ in range(40):
                rclpy.spin_once(node, timeout_sec=0.05)
            print("  sent. Each arm reports progress in its own log.")
            return 0

        # Only `gripper` and `joints` reach here, and both are single-arm.
        ns = args.arm

        if not node.agent_up(ns):
            print(f"  no agent on {ns} -- nothing is publishing {ns}/ee_pose.\n"
                  f"  Start it:  docker compose up -d {ns.strip('/').replace('_', '-')}\n"
                  "  (or AUTOSTART=false is set on that service)", file=sys.stderr)
            return 3

        if args.cmd == "gripper":
            desc = f"set {ns} gripper to {args.value:.4f} m ({args.value * 1000:.1f} mm)"
        else:
            desc = (f"move {ns} joints to "
                    + " ".join(f"{v:+.3f}" for v in args.values))

        print(f"  plan: {desc}")
        js = node.js.get(ns)
        if js and args.cmd == "joints":
            cur = list(js.position)[:6]
            print("  from: " + "  ".join(f"{p:+.3f}" for p in cur))

            want = list(args.values)[:6]
            if want:
                print("  to:   " + "  ".join(f"{p:+.3f}" for p in want))
                delta = [w - c for w, c in zip(want, cur)]
                print("  move: " + "  ".join(f"{d:+.3f}" for d in delta))
                # Called out by name. A joint that has to travel most of a turn
                # is the one that snags a cable or sweeps through something, and
                # it is invisible in a row of radians -- the number just looks
                # like the others.
                big = [(i, d) for i, d in enumerate(delta) if abs(d) > 1.0]
                for i, d in big:
                    print(f"  NOTE: joint {i + 1} travels {math.degrees(d):+.0f} deg "
                          f"-- check clearance and cable routing")

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
            print("  The arm also prints its own plan; watch it with:")
            print(f"      docker compose logs -f {ns.strip('/').replace('_', '-')}")
            return 0

        print("\n  MOVING IN 2 SECONDS -- Ctrl-C to abort.")
        time.sleep(2.0)

        node.enable(ns, True)
        if args.cmd == "gripper":
            m = Float32()
            m.data = float(args.value)
            node.pub_grip[ns].publish(m)
        else:
            m = JointState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.position = [float(v) for v in args.values]
            node.pub_joints[ns].publish(m)

        # Spin briefly so the message actually leaves before we exit. A depth-1
        # publisher destroyed immediately after publish can drop it.
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.05)
        print("  sent. The arm reports progress in its own log.")
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  aborted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
