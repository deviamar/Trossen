#!/usr/bin/env python3
"""Named poses for the middle arm -- save the arm's current pose, recall it later.

    ./pose.py list                    # all poses, with distance from where you are
    ./pose.py show sleep
    ./pose.py go sleep                # DRY RUN -- prints the move, sends nothing
    ./pose.py go sleep --execute      # actually move
    ./pose.py save inspect_table      # capture the CURRENT pose under a name
    ./pose.py delete inspect_table

Poses live in config/poses.yaml, in the bind-mounted workspace, so they are
editable from the host and survive image rebuilds.

`save` is the intended way to build up the pose library: drive the arm where you
want it with teleop_keyboard.py, confirm it looks right, then name it. Poses
authored by hand from guessed numbers tend to be subtly wrong in ways you only
discover when the arm is already moving.

A whole-arm move commands all six joints at once, so it is inherently a bigger
motion than the single-joint steps in move_joint.py. Three guards apply:
limits are checked before anything is sent, the dry run shows every joint's
delta, and moves are profiled over --profile-ms (default 3000, gentler than the
2000 used elsewhere) so a large pose change is a slow sweep rather than a lunge.
"""
import argparse
import math
import os
import re
import sys

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes

NS = "/middle"
GROUP = "arm"
POSE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "config", "poses.yaml")

# Joints whose motion swings the camera mount around; flagged in dry runs
# because a large move on these is what wraps the camera cable.
ROLL_JOINTS = {"forearm_roll", "wrist_rotate"}
BIG_MOVE_RAD = 1.0


class PoseNode(Node):
    def __init__(self):
        super().__init__("pose_tool")
        self.joints = None
        self.positions = None
        self.velocities = None
        self.limits = {}
        self.create_subscription(JointState, f"{NS}/joint_states", self._js_cb, 1)
        self.create_subscription(
            String, f"{NS}/robot_description", self._urdf_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST))
        self.pub = self.create_publisher(
            JointGroupCommand, f"{NS}/commands/joint_group", 1)
        self.modes = self.create_client(OperatingModes, f"{NS}/set_operating_modes")

    def _js_cb(self, msg):
        self.joints = list(msg.name)
        self.positions = list(msg.position)
        self.velocities = list(msg.velocity)

    def _urdf_cb(self, msg):
        for m in re.finditer(r"<joint name=\"([a-z_]+)\"[^>]*>(.*?)</joint>",
                             msg.data, re.S):
            lim = re.search(r"lower=\"([-0-9.eE]+)\"\s+upper=\"([-0-9.eE]+)\"",
                            m.group(2))
            if lim:
                self.limits[m.group(1)] = (float(lim.group(1)), float(lim.group(2)))

    def ready(self):
        return self.joints is not None and bool(self.limits)

    def stale_reason(self):
        """Detect a driver that has lost its serial link and is publishing junk.

        Learned the hard way: when the U2D2 re-enumerates to a new ttyUSB, the
        running xs_sdk keeps its now-dead fd and publishes -pi on every joint
        with velocities in the tens of millions. Saving that as a pose, or
        computing a move from it, would be actively dangerous -- so refuse.
        """
        if self.velocities and any(abs(v) > 100 for v in self.velocities):
            return "implausible joint velocities (>100 rad/s)"
        for j, p in zip(self.joints, self.positions):
            lo, hi = self.limits.get(j, (-math.inf, math.inf))
            if not (lo - 0.15 <= p <= hi + 0.15):
                return f"{j} reads {p:.4f}, far outside its limits [{lo:.3f}, {hi:.3f}]"
        return None

    def set_profile(self, ms):
        if not self.modes.wait_for_service(timeout_sec=2.0):
            return False
        req = OperatingModes.Request(
            cmd_type="group", name=GROUP, mode="position", profile_type="time",
            profile_velocity=int(ms), profile_acceleration=int(max(1, ms // 6)))
        fut = self.modes.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.done()


def load_builtin_poses():
    """Poses defined in arm_config.py (the ported ALOHA set).

    Imported rather than duplicated into poses.yaml so there is one source of
    truth: downstream Python code does `from arm_config import FORWARD`, and
    this CLI sees the same numbers. A saved pose of the same name shadows the
    built-in, which is how you correct one without editing code.
    """
    try:
        import arm_config
        return {k: [float(x) for x in v] for k, v in arm_config.POSES["middle"].items()}
    except Exception:
        return {}


def load_user_poses():
    if not os.path.exists(POSE_FILE):
        return {}
    with open(POSE_FILE) as f:
        return yaml.safe_load(f) or {}


def load_poses():
    poses = load_builtin_poses()
    poses.update(load_user_poses())
    return poses


def save_poses(poses):
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    with open(POSE_FILE, "w") as f:
        f.write("# Named poses for the middle arm. Radians, in joint_order:\n")
        f.write("#   [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]\n")
        f.write("# Written by pose.py -- 'pose.py save <name>' captures the live pose.\n")
        yaml.safe_dump(poses, f, default_flow_style=False, sort_keys=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["list", "show", "go", "save", "delete"])
    ap.add_argument("name", nargs="?")
    ap.add_argument("--execute", action="store_true", help="actually move (go only)")
    ap.add_argument("--profile-ms", type=int, default=3000,
                    help="ms for the move; larger is gentler (default 3000)")
    args = ap.parse_args()

    poses = load_poses()

    if args.cmd in ("show", "go", "save", "delete") and not args.name:
        ap.error(f"'{args.cmd}' needs a pose name")
    if args.cmd in ("show", "go", "delete") and args.name not in poses:
        print(f"no pose named {args.name!r}. Known: {', '.join(sorted(poses)) or '(none)'}",
              file=sys.stderr)
        return 2

    if args.cmd == "delete":
        user = load_user_poses()
        if args.name not in user:
            print(f"{args.name!r} is a built-in from arm_config.py, not a saved pose -- "
                  "nothing to delete.\nTo change it, save one with the same name; the "
                  "saved value shadows the built-in.", file=sys.stderr)
            return 2
        user.pop(args.name)
        save_poses(user)
        print(f"deleted {args.name!r}")
        return 0

    rclpy.init()
    node = PoseNode()
    try:
        deadline = node.get_clock().now().nanoseconds + int(5e9)
        while rclpy.ok() and not node.ready():
            if node.get_clock().now().nanoseconds > deadline:
                print(f"No data on {NS}/joint_states within 5s. Is the arm launched?",
                      file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        if (why := node.stale_reason()):
            print(f"REFUSING: the driver's joint data looks invalid -- {why}.\n"
                  "This is what a lost serial link looks like (the U2D2 re-enumerating\n"
                  "to a new /dev/ttyUSB* leaves xs_sdk holding a dead descriptor).\n"
                  "Restart the launch, then check with ./read_joints.py", file=sys.stderr)
            return 4

        cur = dict(zip(node.joints, node.positions))

        if args.cmd == "save":
            user = load_user_poses()
            user[args.name] = [round(float(p), 4) for p in node.positions]
            save_poses(user)
            poses[args.name] = user[args.name]
            print(f"saved {args.name!r}:")
            for j, p in zip(node.joints, poses[args.name]):
                print(f"  {j:<14}{p:>9.4f} rad  ({math.degrees(p):>7.2f} deg)")
            return 0

        if args.cmd == "list":
            if not poses:
                print("no poses yet. Drive the arm somewhere and: ./pose.py save <name>")
                return 0
            print(f"  {'pose':<20}{'max joint delta from here':>28}")
            for name, vals in sorted(poses.items()):
                d = max(abs(v - cur.get(j, v)) for j, v in zip(node.joints, vals))
                print(f"  {name:<20}{d:>22.4f} rad")
            return 0

        vals = poses[args.name]
        if len(vals) != len(node.joints):
            print(f"pose {args.name!r} has {len(vals)} values, arm has "
                  f"{len(node.joints)} joints", file=sys.stderr)
            return 3

        errs = []
        for j, v in zip(node.joints, vals):
            lo, hi = node.limits.get(j, (-math.inf, math.inf))
            if not (lo <= v <= hi):
                errs.append(f"  {j}: {v:.4f} outside [{lo:.4f}, {hi:.4f}]")
        if errs:
            print(f"pose {args.name!r} is not reachable:\n" + "\n".join(errs),
                  file=sys.stderr)
            return 3

        print(f"  pose {args.name!r}, profiled over {args.profile_ms} ms\n")
        print(f"  {'joint':<14}{'current':>9}{'target':>9}{'delta':>10}   note")
        print("  " + "-" * 55)
        warn = []
        for j, v in zip(node.joints, vals):
            delta = v - cur[j]
            note = ""
            if abs(delta) >= BIG_MOVE_RAD:
                note = "LARGE"
                warn.append(j)
            if j in ROLL_JOINTS and abs(delta) >= 0.5:
                note = (note + " CAMERA-CABLE").strip()
                if j not in warn:
                    warn.append(j)
            print(f"  {j:<14}{cur[j]:>9.4f}{v:>9.4f}{delta:>+10.4f}   {note}")

        if args.cmd == "show":
            return 0

        if warn:
            print(f"\n  NOTE: large motion on {', '.join(warn)}. If a roll joint is "
                  "listed,\n  check the camera cable's slack before running this.")

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute to move the arm.")
            return 0

        if not node.set_profile(args.profile_ms):
            print("  warning: could not set motion profile; using whatever is current")
        node.pub.publish(JointGroupCommand(name=GROUP, cmd=[float(v) for v in vals]))
        end = node.get_clock().now().nanoseconds + int(1.0e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"\n  sent. Moving over ~{args.profile_ms/1000:.1f} s.")
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
