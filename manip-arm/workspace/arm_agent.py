#!/usr/bin/env python3
"""Own this arm's SDK connection and drive it from ROS topics.

    ./arm_agent.py                     # namespace from ARM_NS, arm from ARM_IP
    ./arm_agent.py --ns /left_arm
    ./arm_agent.py --scale 0.5         # halve commanded motion, for first tries
    ./arm_agent.py --dry-run           # publish state, accept commands, send nothing

This is the only script here that is a long-running ROS node rather than a
one-shot CLI, and it is the component's whole interface to the rest of the rig.
Everything it does is on the contract in docs/topic-contract.md:

    subscribes  <ns>/cmd_pose      geometry_msgs/PoseStamped   absolute EE target
                <ns>/cmd_joints    sensor_msgs/JointState      absolute joint target
                <ns>/cmd_pose_name std_msgs/String             a saved pose, by name
                <ns>/cmd_gripper   std_msgs/Float32            opening, metres
                <ns>/enable        std_msgs/Bool               accept commands or not

    publishes   <ns>/ee_pose       geometry_msgs/PoseStamped
                <ns>/joint_states  sensor_msgs/JointState
                <ns>/pose_names    std_msgs/String             JSON list of saved poses
                <ns>/active        std_msgs/Bool

THREE WAYS TO COMMAND IT, and they are for different things:

  cmd_pose       streaming Cartesian, 50 Hz, for teleop. The controller solves
                 the IK. Clamped hard against sudden jumps.
  cmd_joints     one absolute joint vector. Nothing to solve, so this is what
                 you use when you already know the configuration you want.
  cmd_pose_name  a name from this arm's own config/poses.yaml -- "sleep",
                 "ready", whatever teach.py saved.

THE POSE FILE STAYS HERE. cmd_pose_name carries a NAME, not joint values,
because the poses belong to this arm: config/poses.yaml is keyed by ARM_NAME and
the same name is a different point in space for the other arm. A caller that
sent values would have to read this arm's file, and then the pose library would
be shared state between containers instead of something one container owns.
Publish a name; this agent looks it up. <ns>/pose_names says what is available.

A named or joint move is DISCRETE, not streamed: it is sent once with a goal
time computed from the distance, and it cancels any streaming target so the two
cannot fight over the arm mid-motion.

IT HOLDS THE CONNECTION. The controller admits one driver at a time, so while
this runs, pose.py / read_joints.py / teach.py / gripper.py cannot connect to
this arm. That is the hardware's rule, not this script's. Stop the agent to go
back to the CLIs.

ABSOLUTE TARGETS, NOT DELTAS. cmd_pose says where the end effector should be,
not how far to move it. The publisher owns the clutching. If deltas accumulated
here instead, one dropped message would permanently shift the arm's frame of
reference against the operator's -- with absolute targets a dropped message
costs a single frame of lag and the next one corrects it.

ANGLE-AXIS ON THE INSIDE. set_cartesian_positions() takes translation plus an
angle-axis rotation vector. ROS sends quaternions. The conversion is here so the
contract can stay in ordinary ROS types.

SAFETY. --enable false, or no cmd_pose for CMD_TIMEOUT_S, holds position. Every
target is checked against a per-axis workspace box and a maximum step from the
current pose before it is sent: a teleop publisher that jumps -- a lost tracking
frame, a controller waking up across the room -- would otherwise become a
full-speed lunge. Rejected targets are logged and the arm holds.
"""
import argparse
import json
import math
import os
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

import trossen_arm

import arm
import arm_config as cfg
import pose as pose_lib

# Streaming rate. The SDK picks its interpolation from goal_time: over 0.2 s is
# quintic, over 0.001 s is linear, below that the value is applied immediately.
# At 50 Hz with goal_time just over the period we get linear interpolation
# between frames, which is what a stream of live targets wants -- quintic would
# be fighting to settle before the next target arrives.
STREAM_HZ = 50.0
GOAL_TIME_S = 1.0 / STREAM_HZ * 1.5

# Hold position if no cmd_pose arrives for this long. Same reasoning as the
# base's 300 ms deadline: an input that dies should stop the robot, not leave
# the last target standing.
CMD_TIMEOUT_S = 0.3

# Largest jump accepted between the current pose and a new target. A real hand
# moves maybe 2 m/s, so 0.05 m at 50 Hz is already generous; anything past it is
# a tracking glitch, not a person.
MAX_STEP_M = 0.05
MAX_STEP_RAD = 0.35

# Workspace box in the arm's base frame, metres. Deliberately conservative --
# this is a backstop against a bad publisher, not a reach specification. The
# controller's own joint limits still apply underneath and will refuse anything
# unreachable; this catches the targets that are reachable but somewhere you did
# not mean, like straight down into the table.
WORKSPACE = {
    "x": (float(os.environ.get("ARM_WS_X_MIN", -0.10)),
          float(os.environ.get("ARM_WS_X_MAX", 0.75))),
    "y": (float(os.environ.get("ARM_WS_Y_MIN", -0.60)),
          float(os.environ.get("ARM_WS_Y_MAX", 0.60))),
    "z": (float(os.environ.get("ARM_WS_Z_MIN", 0.02)),
          float(os.environ.get("ARM_WS_Z_MAX", 0.90))),
}

URDF_JOINTS = [f"joint_{i}" for i in range(cfg.NUM_ARM_JOINTS)] + ["left_carriage_joint"]


def quat_to_angle_axis(x, y, z, w):
    """Quaternion -> angle-axis vector, the form set_cartesian_positions wants."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    x, y, z, w = x / n, y / n, z / n, w / n
    if w < 0.0:                       # shortest arc
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(max(0.0, 1.0 - w * w))
    angle = 2.0 * math.atan2(s, w)
    if s < 1e-9:                      # no rotation; axis is arbitrary
        return [0.0, 0.0, 0.0]
    return [angle * x / s, angle * y / s, angle * z / s]


def angle_axis_to_quat(rx, ry, rz):
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx / angle, ry / angle, rz / angle
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


class ArmAgent(Node):
    def __init__(self, driver, ns, scale, dry_run):
        super().__init__("arm_agent")
        self.driver = driver
        self.scale = scale
        self.dry_run = dry_run

        self.lock = threading.Lock()
        self.enabled = False
        self.target = None            # [x, y, z, rx, ry, rz], angle-axis
        self.last_cmd = 0.0
        self.gripper = None
        self.rejects = 0

        self.create_subscription(PoseStamped, f"{ns}/cmd_pose", self._on_pose, 1)
        self.create_subscription(JointState, f"{ns}/cmd_joints", self._on_joints, 1)
        self.create_subscription(String, f"{ns}/cmd_pose_name", self._on_pose_name, 1)
        self.create_subscription(Float32, f"{ns}/cmd_gripper", self._on_gripper, 1)
        self.create_subscription(Bool, f"{ns}/enable", self._on_enable, 1)

        self.pub_ee = self.create_publisher(PoseStamped, f"{ns}/ee_pose", 1)
        self.pub_js = self.create_publisher(JointState, f"{ns}/joint_states", 1)
        self.pub_active = self.create_publisher(Bool, f"{ns}/active", 1)
        self.pub_names = self.create_publisher(String, f"{ns}/pose_names", 1)
        self.create_timer(1.0, self._names_tick)

        self.create_timer(1.0 / STREAM_HZ, self._control_tick)
        self.create_timer(1.0 / 20.0, self._state_tick)

        self.base_frame = os.environ.get("ARM_BASE_FRAME", "base_link")

    # ---- inputs ----------------------------------------------------------
    def _on_enable(self, msg):
        with self.lock:
            if msg.data and not self.enabled:
                # Entering teleop: drop any stale target so the first command
                # has to arrive before anything moves. Without this the arm
                # would jump to wherever the operator was standing last time.
                self.target = None
                self.rejects = 0
                if not self.dry_run:
                    self.driver.set_arm_modes(trossen_arm.Mode.position)
                self.get_logger().info("enabled")
            elif not msg.data and self.enabled:
                self.target = None
                self.get_logger().info("disabled -- holding position")
            self.enabled = bool(msg.data)

    def _on_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        want = [p.x, p.y, p.z] + quat_to_angle_axis(o.x, o.y, o.z, o.w)

        with self.lock:
            if not self.enabled:
                return
            current = self.target or list(self.driver.get_cartesian_positions())

            if self.scale != 1.0:
                # Scale about the current pose, so a half-scale command is half
                # the *motion*, not half the coordinate.
                want = [c + (w - c) * self.scale for c, w in zip(current, want)]

            why = self._reject_reason(current, want)
            if why:
                self.rejects += 1
                # Rate-limited: a stuck publisher would otherwise fill the log
                # faster than anyone can read it.
                if self.rejects % 25 == 1:
                    self.get_logger().warn(f"target refused ({self.rejects}): {why}")
                return

            self.target = want
            self.last_cmd = self.get_clock().now().nanoseconds * 1e-9

    def _reject_reason(self, current, want):
        for i, axis in enumerate("xyz"):
            lo, hi = WORKSPACE[axis]
            if not (lo <= want[i] <= hi):
                return f"{axis}={want[i]:.3f} outside workspace [{lo}, {hi}]"
        step = math.dist(current[:3], want[:3])
        if step > MAX_STEP_M:
            return f"jump of {step:.3f} m in one frame (limit {MAX_STEP_M})"
        rot = math.dist(current[3:], want[3:])
        if rot > MAX_STEP_RAD:
            return f"rotation jump of {rot:.3f} rad (limit {MAX_STEP_RAD})"
        return None

    def _names_tick(self):
        """Advertise what this arm can be sent to, re-read each time.

        Re-reading rather than caching so a pose saved by teach.py or edited on
        the host shows up without restarting the agent -- the file is
        bind-mounted, and a cache would make it look like the save failed.
        """
        try:
            names = sorted(pose_lib.load_poses(cfg.ARM_NAME))
        except Exception as e:
            self.get_logger().warn(f"cannot read poses: {e}", throttle_duration_sec=30.0)
            return
        m = String()
        m.data = json.dumps(names)
        self.pub_names.publish(m)

    def _on_pose_name(self, msg):
        name = msg.data.strip()
        try:
            poses = pose_lib.load_poses(cfg.ARM_NAME)
        except Exception as e:
            self.get_logger().error(f"cannot read poses: {e}")
            return
        if name not in poses:
            self.get_logger().error(
                f"no pose {name!r} for {cfg.ARM_NAME}; have: {', '.join(sorted(poses))}")
            return
        self._command_joints(list(poses[name]), why=f"pose {name!r}")

    def _on_joints(self, msg):
        if len(msg.position) < cfg.NUM_ARM_JOINTS:
            self.get_logger().error(
                f"cmd_joints needs at least {cfg.NUM_ARM_JOINTS} positions, "
                f"got {len(msg.position)}")
            return
        self._command_joints(list(msg.position), why="cmd_joints")

    def _command_joints(self, values, why):
        """One discrete joint-space move. Checked, then sent once."""
        with self.lock:
            if not self.enabled:
                self.get_logger().warn(f"{why} ignored -- not enabled")
                return
            # Cancel any streaming target. Otherwise _control_tick would keep
            # pushing the old Cartesian goal at 50 Hz and the two commands would
            # pull the arm in different directions for the length of the move.
            self.target = None

        try:
            lims = arm.limits(self.driver)
            current = list(self.driver.get_all_positions())
        except Exception as e:
            self.get_logger().error(f"lost the arm: {e}")
            return

        n = min(len(values), cfg.GRIPPER_INDEX + 1)
        target = list(current)
        for i in range(n):
            target[i] = float(values[i])

        if errs := [e for i in range(n) if (e := arm.check(i, target[i], lims))]:
            self.get_logger().error(f"{why} refused: {'; '.join(errs)}")
            return

        goal_time = cfg.goal_time_for([target[i] - current[i] for i in range(n)])
        if self.dry_run:
            self.get_logger().info(
                f"DRY RUN {why}: would move over {goal_time:.1f} s to "
                + " ".join(f"{v:+.3f}" for v in target[:cfg.NUM_ARM_JOINTS]))
            return

        self.get_logger().info(f"{why}: moving over {goal_time:.1f} s")
        try:
            self.driver.set_arm_modes(trossen_arm.Mode.position)
            self.driver.set_arm_positions(
                [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], goal_time, False)
            if n > cfg.GRIPPER_INDEX:
                self.driver.set_gripper_mode(trossen_arm.Mode.position)
                self.driver.set_gripper_position(
                    float(target[cfg.GRIPPER_INDEX]), goal_time, False)
        except Exception as e:
            self.get_logger().error(f"the controller rejected {why}: {e}")
            with self.lock:
                self.enabled = False

    def _on_gripper(self, msg):
        with self.lock:
            self.gripper = max(cfg.GRIPPER_CLOSED,
                               min(cfg.GRIPPER_OPEN, float(msg.data)))

    # ---- outputs ---------------------------------------------------------
    def _control_tick(self):
        with self.lock:
            if not self.enabled or self.target is None:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_cmd > CMD_TIMEOUT_S:
                # Input went quiet. Drop the target rather than holding it: the
                # arm stays where it is and a resumed stream starts fresh.
                self.target = None
                self.get_logger().warn("cmd_pose timed out -- holding")
                return
            target, grip = list(self.target), self.gripper

        if self.dry_run:
            return
        try:
            self.driver.set_cartesian_positions(
                target, trossen_arm.InterpolationSpace.cartesian,
                GOAL_TIME_S, False)
            if grip is not None:
                self.driver.set_gripper_position(grip, GOAL_TIME_S, False)
        except Exception as e:
            # The controller refuses anything it cannot follow and drops to
            # idle. Surface it and stop commanding rather than retrying into a
            # latched error.
            self.get_logger().error(f"controller rejected the target: {e}")
            with self.lock:
                self.enabled = False
                self.target = None

    def _state_tick(self):
        try:
            pos = list(self.driver.get_all_positions())
            cart = list(self.driver.get_cartesian_positions())
        except Exception as e:
            self.get_logger().error(f"lost the arm: {e}")
            return

        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = list(URDF_JOINTS)
        js.position = [float(v) for v in pos]
        self.pub_js.publish(js)

        ee = PoseStamped()
        ee.header.stamp = stamp
        ee.header.frame_id = self.base_frame
        ee.pose.position.x, ee.pose.position.y, ee.pose.position.z = cart[:3]
        qx, qy, qz, qw = angle_axis_to_quat(*cart[3:])
        ee.pose.orientation.x = qx
        ee.pose.orientation.y = qy
        ee.pose.orientation.z = qz
        ee.pose.orientation.w = qw
        self.pub_ee.publish(ee)

        active = Bool()
        active.data = bool(self.enabled)
        self.pub_active.publish(active)


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("--ns", default=os.environ.get("ARM_NS") or f"/{cfg.ARM_NAME}",
                    help="ROS namespace (default from ARM_NS)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="motion scale, <1 damps commanded motion (default 1.0)")
    ap.add_argument("--dry-run", action="store_true",
                    help="publish state and accept commands, but send nothing")
    args = ap.parse_args()

    ns = "/" + args.ns.strip("/")

    rclpy.init()
    with arm.connect(args) as driver:
        node = ArmAgent(driver, ns, args.scale, args.dry_run)
        print(f"  arm agent up: {ns} -> {args.ip}"
              f"{'  [DRY RUN]' if args.dry_run else ''}")
        print(f"  waiting for {ns}/enable = true. Ctrl-C to stop.")
        print("  NOTE: this holds the arm's only connection. pose.py and the")
        print("        other CLIs cannot run against this arm until it exits.")
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            # Nothing to undo: leaving the connection drops the arm to idle,
            # which on this hardware is a hold, not an off.
            print("\n  stopping -- the arm holds where it is.")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
