#!/usr/bin/env python3
"""Show a pose in RViz before committing it to the real arm.

Start RViz in one shell:

    ./launch-rviz.sh

...then in a second shell, drive the model in it:

    ./preview.py pose home_watch          # hold the saved pose, static
    ./preview.py pose home_watch --sweep  # read the arm, then animate current -> pose
    ./preview.py live                     # mirror the real arm, 10 Hz, until Ctrl-C
    ./preview.py values 0 1.0 0.5 0 0 0   # arbitrary joint vector, no pose file

This publishes /joint_states; robot_state_publisher (started by launch-rviz.sh)
turns that into TF and RViz draws it. Nothing here touches the controller unless
you ask for --sweep or `live`, both of which only READ.

WHAT THIS DOES AND DOES NOT TELL YOU
------------------------------------
It answers "is that posture what I meant" -- the thing a table of six radian
values is bad at, and the reason a mistyped joint angle usually only becomes
obvious once the arm is already moving.

It is NOT a simulator and it is NOT collision checking:

  * No physics, no contact. The model will happily pass through your bench.
  * --sweep interpolates linearly in joint space. The real controller runs its
    own profile over goal_time, so the endpoints match and the middle is an
    approximation -- good enough to spot "the elbow swings through the camera
    mount", not a guarantee about the exact path.
  * Self-collision and workspace obstacles are what MoveIt is for
    (trossen_arm_moveit, not built into this image -- see the README).

So: preview, then `./pose.py go <name>` for the dry-run table, then --execute.

MODES THAT TOUCH THE ARM
------------------------
`live` and `--sweep` open the driver, which is exclusive: neither will run while
pose.py, teach.py or a --hold grasp has the connection. `live` mirrors, it does
not command -- but note that idle on this arm is braked, so you cannot push the
real arm by hand and watch it move here. That is what teach.py is for, and
teach.py can publish to this same RViz on its own.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import arm
import arm_config as cfg
import pose as pose_lib

# The URDF's own names, from trossen_arm_description/urdf/macros/_wxai.urdf.xacro.
# joint_0..joint_5 line up 1:1 with the SDK's indices 0..5, which is why the
# CLIs here use those names too.
URDF_ARM_JOINTS = [f"joint_{i}" for i in range(cfg.NUM_ARM_JOINTS)]

# The gripper is two prismatic carriages in the URDF, and right_carriage_joint
# is declared with <mimic joint="left_carriage_joint"/>. robot_state_publisher
# applies the mimic itself, so publishing the left one alone moves both fingers;
# publishing both would just be redundant. Travel is 0..0.044 m per carriage,
# and the SDK's gripper position is per-carriage metres over the same range, so
# the value goes across unscaled.
URDF_GRIPPER_JOINT = "left_carriage_joint"

PUBLISH_HZ = 20.0


class PreviewPublisher(Node):
    """Publishes /joint_states. Latched-ish: it keeps republishing.

    RViz only draws what TF says, and TF only updates when robot_state_publisher
    hears a JointState. A single message that lands before RViz has subscribed
    is silently lost and the model just sits at zero, so every mode here
    republishes rather than sending one and exiting.
    """

    def __init__(self):
        super().__init__("pose_preview")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

    def publish(self, values):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(URDF_ARM_JOINTS)
        msg.position = [float(v) for v in values[:cfg.NUM_ARM_JOINTS]]
        if len(values) > cfg.NUM_ARM_JOINTS:
            msg.name.append(URDF_GRIPPER_JOINT)
            msg.position.append(float(values[cfg.GRIPPER_INDEX]))
        self.pub.publish(msg)

    def hold(self, values, seconds=None):
        """Republish one pose until Ctrl-C (or `seconds` elapse)."""
        period = 1.0 / PUBLISH_HZ
        deadline = None if seconds is None else time.time() + seconds
        while rclpy.ok():
            self.publish(values)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
            if deadline and time.time() >= deadline:
                return

    def sweep(self, start, end, seconds):
        """Animate start -> end, then hold at end."""
        steps = max(2, int(seconds * PUBLISH_HZ))
        for s in range(steps + 1):
            if not rclpy.ok():
                return
            t = s / steps
            self.publish([a + (b - a) * t for a, b in zip(start, end)])
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_HZ)


def describe(values):
    return "\n".join(f"  {cfg.label(i):<14}{arm.fmt(i, v)}"
                     for i, v in enumerate(values))


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("pose", help="preview a saved or built-in pose")
    p.add_argument("name")
    p.add_argument("--arm-name", default=cfg.ARM_NAME,
                   help=f"whose poses to read (default {cfg.ARM_NAME})")
    p.add_argument("--sweep", action="store_true",
                   help="read the arm's current pose first, then animate into the target")
    p.add_argument("--seconds", type=float, default=3.0,
                   help="animation length for --sweep (default 3)")

    p = sub.add_parser("values", help="preview an explicit joint vector")
    p.add_argument("values", nargs="+", metavar="V",
                   help=f"{cfg.NUM_ARM_JOINTS} radians, optionally a 7th for the gripper (m)")
    p.add_argument("--deg", action="store_true", help="arm values are degrees")

    p = sub.add_parser("live", help="mirror the real arm into RViz until Ctrl-C")
    p.add_argument("--hz", type=float, default=10.0, help="poll rate (default 10)")

    args = ap.parse_args()

    # Resolved before rclpy.init so a bad name or a busy arm fails plainly.
    target = None
    start = None

    if args.mode == "pose":
        poses = pose_lib.load_poses(args.arm_name)
        if args.name not in poses:
            print(f"no pose named {args.name!r} for {args.arm_name}. "
                  f"Known: {', '.join(sorted(poses)) or '(none)'}", file=sys.stderr)
            return 2
        target = list(poses[args.name])
        if args.sweep:
            with arm.connect(args) as driver:
                current = list(driver.get_all_positions())
            start = current[:len(target)]

    elif args.mode == "values":
        vals = [float(v) for v in args.values]
        if len(vals) not in (cfg.NUM_ARM_JOINTS, cfg.GRIPPER_INDEX + 1):
            print(f"give {cfg.NUM_ARM_JOINTS} values (or {cfg.GRIPPER_INDEX + 1} "
                  f"with the gripper), got {len(vals)}", file=sys.stderr)
            return 2
        if args.deg:
            # The gripper is linear metres; degrees must not touch it.
            vals = [math.radians(v) if i < cfg.NUM_ARM_JOINTS else v
                    for i, v in enumerate(vals)]
        target = vals

    rclpy.init()
    node = PreviewPublisher()
    try:
        if args.mode == "live":
            print(f"  mirroring {cfg.ARM_NAME} at {args.ip} into RViz -- Ctrl-C to stop.")
            with arm.connect(args) as driver:
                period = 1.0 / args.hz
                while rclpy.ok():
                    node.publish(list(driver.get_all_positions()))
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(period)
        else:
            print(f"  previewing:\n{describe(target)}")
            if start is not None:
                print(f"\n  sweeping from the arm's current pose over "
                      f"{args.seconds:.1f} s (joint-space interpolation, not the\n"
                      f"  controller's own profile), then holding.")
                node.sweep(start, target, args.seconds)
            print("\n  holding in RViz -- Ctrl-C when you have seen enough.")
            print("  Nothing has been sent to the arm. To move it for real:")
            if args.mode == "pose":
                print(f"    ./pose.py go {args.name} --execute")
            node.hold(target)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
