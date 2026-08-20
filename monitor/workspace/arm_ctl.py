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

        # Everything below commands one arm.
        if not args.arm:
            print("give --arm; commanding every arm at once is not something\n"
                  "you meant to type.", file=sys.stderr)
            return 2
        ns = args.arm

        if not node.agent_up(ns):
            print(f"  no agent on {ns} -- nothing is publishing {ns}/ee_pose.\n"
                  f"  Start it:  docker compose up -d {ns.strip('/').replace('_', '-')}\n"
                  "  (or AUTOSTART=false is set on that service)", file=sys.stderr)
            return 3

        if args.cmd == "go":
            names = node.names.get(ns, [])
            if names and args.name not in names:
                print(f"  {ns} has no pose {args.name!r}. Known: {', '.join(names)}",
                      file=sys.stderr)
                return 2
            desc = f"move {ns} to pose {args.name!r}"
        elif args.cmd == "gripper":
            desc = f"set {ns} gripper to {args.value:.4f} m ({args.value * 1000:.1f} mm)"
        else:
            desc = (f"move {ns} joints to "
                    + " ".join(f"{v:+.3f}" for v in args.values))

        print(f"  plan: {desc}")
        js = node.js.get(ns)
        if js and args.cmd in ("go", "joints"):
            print("  from: " + "  ".join(f"{p:+.3f}" for p in list(js.position)[:6]))

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
            print("  The arm also prints its own plan; watch it with:")
            print(f"      docker compose logs -f {ns.strip('/').replace('_', '-')}")
            return 0

        print("\n  MOVING IN 2 SECONDS -- Ctrl-C to abort.")
        time.sleep(2.0)

        node.enable(ns, True)
        if args.cmd == "go":
            m = String()
            m.data = args.name
            node.pub_name[ns].publish(m)
        elif args.cmd == "gripper":
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
