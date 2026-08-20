#!/usr/bin/env python3
"""Drive the active-vision arm from Cartesian targets. The middle arm's agent.

    ./head_agent.py --urdf /path/to/wx250s.urdf     # normal
    ./head_agent.py --dry-run                       # solve and publish, command nothing
    ./head_agent.py --print-links --urdf ...        # list link names and exit

The counterpart to ../../manip-arm/workspace/arm_agent.py, and it exists for one
reason: the WXAI controller solves its own Cartesian IK and this arm's does not.
xs_sdk speaks joint positions only, so somebody has to turn a pose into joint
angles, and that somebody is middle_ik.py on pyroki.

    subscribes  /middle/cmd_pose   geometry_msgs/PoseStamped   absolute target
                /middle/enable     std_msgs/Bool               follow or hold
    publishes   /middle/ee_pose    geometry_msgs/PoseStamped   measured, from FK
                /middle/active     std_msgs/Bool

Contract identical to the manipulators' (docs/topic-contract.md), so the teleop
node drives all three arms through one code path and does not care that this one
needs a solver and the others do not.

NO GRIPPER. This arm carries the ZED where a gripper would be, so there is no
/middle/cmd_gripper. Six joints, and the last one pans the camera.

FIRST SOLVE COMPILES. jax traces on the first call; --warmup (default) pays that
at startup with the arm still. Expect several seconds and a quiet terminal.

SAFETY. enable=false, or no cmd_pose for CMD_TIMEOUT_S, holds position. Targets
are refused if they are outside the workspace box or more than MAX_STEP_M from
the current pose -- a lost tracking frame would otherwise become a lunge. The
solver additionally clamps every joint step to what the arm could travel in one
control period, so an unreachable target degrades into slow drift rather than a
snap.
"""
import argparse
import math
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from interbotix_xs_msgs.msg import JointGroupCommand

import arm_config as cfg

NS = os.environ.get("ROBOT_NAME_NS") or f"/{os.environ.get('ROBOT_NAME', 'middle')}"
GROUP = "arm"

STREAM_HZ = 50.0
CMD_TIMEOUT_S = 0.3
MAX_STEP_M = 0.08
MAX_STEP_RAD = 0.5

# Per-joint velocity ceiling fed to the solver's smoothness scaling and final
# clamp. Deliberately below what the DYNAMIXELs can do: this arm carries a
# camera and a cable loom, and a fast slew is how the loom gets caught.
JOINT_VELOCITY_LIMIT = float(os.environ.get("MIDDLE_VEL_LIMIT", 1.5))  # rad/s

WORKSPACE = {
    "x": (float(os.environ.get("MIDDLE_WS_X_MIN", -0.30)),
          float(os.environ.get("MIDDLE_WS_X_MAX", 0.70))),
    "y": (float(os.environ.get("MIDDLE_WS_Y_MIN", -0.55)),
          float(os.environ.get("MIDDLE_WS_Y_MAX", 0.55))),
    "z": (float(os.environ.get("MIDDLE_WS_Z_MIN", 0.05)),
          float(os.environ.get("MIDDLE_WS_Z_MAX", 1.00))),
}


def quat_wxyz_from_msg(o):
    return np.array([o.w, o.x, o.y, o.z], dtype=np.float32)


class HeadAgent(Node):
    def __init__(self, robot, solve, ee_link, dry_run):
        super().__init__("head_agent")
        self.robot = robot
        self.solve = solve
        self.ee_link = ee_link
        self.dry_run = dry_run

        self.lock = threading.Lock()
        self.enabled = False
        self.target = None            # (position(3), wxyz(4))
        self.last_cmd = 0.0
        self.q_cmd = None             # last commanded configuration
        self.measured = None          # latest /joint_states, arm joints only
        self.rejects = 0

        self.create_subscription(JointState, f"{NS}/joint_states", self._on_js, 1)
        self.create_subscription(PoseStamped, f"{NS}/cmd_pose", self._on_pose, 1)
        self.create_subscription(Bool, f"{NS}/enable", self._on_enable, 1)

        self.pub_ee = self.create_publisher(PoseStamped, f"{NS}/ee_pose", 1)
        self.pub_active = self.create_publisher(Bool, f"{NS}/active", 1)
        # The real xs_sdk interface: one JointGroupCommand for the whole arm
        # group, positions in radians in joint_order. Same topic and message
        # pose.py and move_joint.py already use, so this agent is one more
        # publisher to it rather than a parallel control path.
        self.pub_cmd = self.create_publisher(
            JointGroupCommand, f"{NS}/commands/joint_group", 1)

        self.create_timer(1.0 / STREAM_HZ, self._control_tick)
        self.create_timer(1.0 / 20.0, self._state_tick)

    # ---- inputs ----------------------------------------------------------
    def _on_js(self, msg):
        n = self.robot.joints.num_actuated_joints
        if len(msg.position) < n:
            return
        with self.lock:
            self.measured = np.asarray(msg.position[:n], dtype=np.float64)
            if self.q_cmd is None:
                self.q_cmd = self.measured.copy()

    def _on_enable(self, msg):
        with self.lock:
            if msg.data and not self.enabled:
                # Drop any stale target and re-sync the commanded configuration
                # to where the arm actually is; anchoring on a stale command
                # would step the arm by however far it had drifted.
                self.target = None
                self.rejects = 0
                if self.measured is not None:
                    self.q_cmd = self.measured.copy()
                self.get_logger().info("enabled")
            elif not msg.data and self.enabled:
                self.target = None
                self.get_logger().info("disabled -- holding position")
            self.enabled = bool(msg.data)

    def _on_pose(self, msg):
        p = msg.pose.position
        want_p = np.array([p.x, p.y, p.z], dtype=np.float32)
        want_q = quat_wxyz_from_msg(msg.pose.orientation)

        with self.lock:
            if not self.enabled or self.q_cmd is None:
                return
            current = self._fk(self.q_cmd)
            why = self._reject_reason(current, want_p)
            if why:
                self.rejects += 1
                if self.rejects % 25 == 1:
                    self.get_logger().warn(f"target refused ({self.rejects}): {why}")
                return
            self.target = (want_p, want_q)
            self.last_cmd = self.get_clock().now().nanoseconds * 1e-9

    def _reject_reason(self, current_pos, want_p):
        for i, axis in enumerate("xyz"):
            lo, hi = WORKSPACE[axis]
            if not (lo <= float(want_p[i]) <= hi):
                return f"{axis}={want_p[i]:.3f} outside workspace [{lo}, {hi}]"
        step = float(np.linalg.norm(np.asarray(want_p) - np.asarray(current_pos)))
        if step > MAX_STEP_M:
            return f"jump of {step:.3f} m in one frame (limit {MAX_STEP_M})"
        return None

    # ---- kinematics ------------------------------------------------------
    def _fk(self, q):
        """End-effector position for a configuration."""
        import jaxlie
        idx = self.robot.links.names.index(self.ee_link)
        T = self.robot.forward_kinematics(np.asarray(q))
        return np.asarray(jaxlie.SE3(T[idx]).translation())

    def _fk_pose(self, q):
        import jaxlie
        idx = self.robot.links.names.index(self.ee_link)
        T = jaxlie.SE3(self.robot.forward_kinematics(np.asarray(q))[idx])
        return np.asarray(T.translation()), np.asarray(T.rotation().wxyz)

    # ---- outputs ---------------------------------------------------------
    def _control_tick(self):
        with self.lock:
            if not self.enabled or self.target is None or self.q_cmd is None:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_cmd > CMD_TIMEOUT_S:
                self.target = None
                self.get_logger().warn("cmd_pose timed out -- holding")
                return
            target_p, target_q = self.target
            prev_q = self.q_cmd.copy()

        n = self.robot.joints.num_actuated_joints
        try:
            q_new = self.solve(
                target_position=target_p,
                target_wxyz=target_q,
                prev_q=prev_q,
                dt=1.0 / STREAM_HZ,
                joint_velocity_limits=np.full(n, JOINT_VELOCITY_LIMIT, np.float32),
            )
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return

        if not np.all(np.isfinite(q_new)):
            self.get_logger().error("IK returned non-finite joints -- ignoring")
            return

        with self.lock:
            self.q_cmd = q_new

        if self.dry_run:
            return

        self.pub_cmd.publish(
            JointGroupCommand(name=GROUP, cmd=[float(v) for v in q_new]))

    def _state_tick(self):
        with self.lock:
            q = self.measured.copy() if self.measured is not None else None
            enabled = self.enabled
        if q is None:
            return
        try:
            pos, wxyz = self._fk_pose(q)
        except Exception as e:
            self.get_logger().error(f"FK failed: {e}", throttle_duration_sec=5.0)
            return

        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = os.environ.get("MIDDLE_BASE_FRAME", "base_link")
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in pos]
        m.pose.orientation.w = float(wxyz[0])
        m.pose.orientation.x = float(wxyz[1])
        m.pose.orientation.y = float(wxyz[2])
        m.pose.orientation.z = float(wxyz[3])
        self.pub_ee.publish(m)

        a = Bool()
        a.data = bool(enabled)
        self.pub_active.publish(a)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=os.environ.get("MIDDLE_URDF", ""),
                    help="URDF path (or MIDDLE_URDF). See launch-arm.sh --dump-urdf")
    ap.add_argument("--ee-link", default=os.environ.get("MIDDLE_EE_LINK", "camera_link"),
                    help="link the target pose applies to")
    ap.add_argument("--print-links", action="store_true",
                    help="list the URDF's link names and exit")
    ap.add_argument("--no-warmup", action="store_true",
                    help="skip the compile-at-startup (first move will stall instead)")
    ap.add_argument("--dry-run", action="store_true",
                    help="solve and publish state, but command no motion")
    args = ap.parse_args()

    if not args.urdf:
        print("need --urdf (or MIDDLE_URDF). Generate one with:\n"
              "  ./launch-arm.sh --dump-urdf > /tmp/wx250s.urdf", file=sys.stderr)
        return 2

    try:
        from middle_ik import load_robot, make_middle_arm_ik_solver
    except ImportError as e:
        print(f"pyroki stack missing: {e}\n"
              "  the image needs jax, jaxls, jaxlie, pyroki, yourdfpy -- rebuild",
              file=sys.stderr)
        return 2

    robot = load_robot(args.urdf)

    if args.print_links:
        print("\n".join(robot.links.names))
        return 0

    solve, warmup = make_middle_arm_ik_solver(robot, args.ee_link)
    n = robot.joints.num_actuated_joints

    # The solver returns joints in the URDF's actuated order; xs_sdk applies a
    # JointGroupCommand in its own joint_order. If those disagree the arm moves,
    # smoothly, to entirely the wrong configuration -- so say what the order is
    # and let the operator check it once, rather than discovering it later.
    print(f"  URDF actuated order: {list(robot.joints.actuated_names)}")
    print(f"  xs_sdk joint_order:  {list(cfg.JOINT_NAMES[:n])}")
    if list(robot.joints.actuated_names) != list(cfg.JOINT_NAMES[:n]):
        print("  WARNING: those two lists differ. A JointGroupCommand is applied\n"
              "  positionally, so the solution would be sent to the wrong joints.\n"
              "  Fix the URDF or remap before enabling this agent.", file=sys.stderr)
    print(f"  loaded {args.urdf}: {n} actuated joints, ee link {args.ee_link!r}")

    if not args.no_warmup:
        print("  compiling the IK solver (jax traces on first call) ...")
        warmup(prev_q=np.zeros(n, np.float32), dt=1.0 / STREAM_HZ,
               joint_velocity_limits=np.full(n, JOINT_VELOCITY_LIMIT, np.float32))
        print("  solver ready.")

    rclpy.init()
    node = HeadAgent(robot, solve, args.ee_link, args.dry_run)
    print(f"  head agent up: {NS}{'  [DRY RUN]' if args.dry_run else ''}")
    print(f"  waiting for {NS}/enable = true. Ctrl-C to stop.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  stopping -- the arm holds where it is.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
