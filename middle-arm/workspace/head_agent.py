#!/usr/bin/env python3
"""Drive the active-vision arm from Cartesian targets. The middle arm's agent.

    ./head_agent.py --urdf /path/to/wx250s.urdf     # normal
    ./head_agent.py --dry-run                       # solve and publish, command nothing
    ./head_agent.py --print-links --urdf ...        # list link names and exit

The counterpart to ../../manip-arm/workspace/arm_agent.py, and it exists for one
reason: the WXAI controller solves its own Cartesian IK and this arm's does not.
xs_sdk speaks joint positions only, so somebody has to turn a pose into joint
angles, and that somebody is middle_ik.py on pyroki.

    subscribes  /middle/cmd_pose      geometry_msgs/PoseStamped  absolute target
                /middle/cmd_pose_name std_msgs/String            a saved pose, by name
                /middle/save_pose     std_msgs/String            record where it is now
                /middle/enable        std_msgs/Bool              follow or hold
    publishes   /middle/ee_pose       geometry_msgs/PoseStamped  measured, from FK
                /middle/pose_names    std_msgs/String            JSON list of poses
                /middle/active        std_msgs/Bool

A NAMED MOVE IS JOINT-SPACE AND RAMPED. xs_sdk applies a JointGroupCommand
instantly, so a distant pose sent raw would be a lunge; instead the goal is
walked from the current commanded configuration at MIDDLE_POSE_SPEED rad/s on
the same 50 Hz tick the solver uses. Poses come from arm_config.POSES plus
config/poses.yaml (pose.py save <name>), same split as the manipulators. A
named move cancels a streaming target and vice versa.

Contract identical to the manipulators' (docs/topic-contract.md), so the teleop
node drives all three arms through one code path and does not care that this one
needs a solver and the others do not.

NO GRIPPER. This arm carries the ZED where a gripper would be, so there is no
/middle/cmd_gripper. Six joints, and the last one pans the camera.

FIRST SOLVE COMPILES. jax traces on the first call; --warmup (default) pays that
at startup with the arm still. Expect several seconds and a quiet terminal.

SAFETY. enable=false, or no cmd_pose for CMD_TIMEOUT_S, holds position. Targets
outside the workspace box are refused; everything else is approached under the
solver's per-joint velocity clamp, which turns a lost tracking frame or an
unreachable target into slow drift rather than a snap -- and, unlike a
distance-based refusal, cannot leave the arm permanently ignoring its input.
"""
import argparse
import json
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
from std_msgs.msg import Bool, String
from interbotix_xs_msgs.msg import JointGroupCommand
from std_msgs.msg import Float32

import arm_config as cfg

NS = os.environ.get("ROBOT_NAME_NS") or f"/{os.environ.get('ROBOT_NAME', 'middle')}"
GROUP = "arm"

STREAM_HZ = 50.0
CMD_TIMEOUT_S = 0.3
# No jump rejection -- see _reject_reason. The solver's per-tick joint
# velocity clamp is what bounds motion toward a distant target.
MAX_STEP_M = 0.08          # kept for reference; nothing reads it any more
MAX_STEP_RAD = 0.5

# Named (whole-arm) moves ramp at this many rad/s per joint -- deliberately
# below JOINT_VELOCITY_LIMIT: a pose recall is a big move of every joint at
# once, on the arm that carries the camera and its loom.
POSE_SPEED = float(os.environ.get("MIDDLE_POSE_SPEED", 0.6))

POSES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "config", "poses.yaml")

# THE INTER-ARM COLLISION GATE -- rig/rig_collision.py, mounted read-only.
# Every JointGroupCommand is checked against where the two manipulators were
# last reported before it is published; one that would bring this arm's
# capsules (camera included) within the margin of theirs is not sent and the
# arm holds. Same module, same yaml and same semantics as the manipulators'
# arm_agent.py, so the three agents agree on the geometry by construction.
RIG_GATE = os.environ.get("RIG_GATE", "0").strip() == "1"
RIG_GATE_FILE = os.environ.get("RIG_GATE_FILE", "/home/robot/rig/capsules.yaml")
RIG_GATE_MARGIN = float(os.environ.get("RIG_GATE_MARGIN", 0.03))
RIG_PEERS = {
    "left_arm": "/left_arm/joint_states",
    "right_arm": "/right_arm/joint_states",
    "middle": "/middle/joint_states",
}
LIFT_HEIGHT_TOPIC = os.environ.get("RIG_LIFT_TOPIC", "/slate/lift/height")

# Re-written on every save, because yaml.safe_dump cannot preserve comments and
# a pose file with no explanation of its units is a trap.
POSES_HEADER = """# Named poses for the middle arm. Radians, in joint_order:
#   [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
# Written by pose.py, or by <ns>/save_pose (the save key in rig_key.py, which
# captures all three arms at once). Built-in poses live in arm_config.POSES and
# are overlaid by this file, so a name here wins.
"""


def load_poses():
    """Built-in poses from arm_config, overlaid with config/poses.yaml.

    Re-read on every use rather than cached, for the same reason arm_agent
    re-reads: the yaml is bind-mounted, and a pose saved from the host should
    appear without restarting this agent.
    """
    poses = {k: [float(x) for x in v]
             for k, v in cfg.POSES.get("middle", {}).items()}
    try:
        import yaml
        with open(POSES_FILE) as f:
            user = yaml.safe_load(f) or {}
        poses.update({k: [float(x) for x in v] for k, v in user.items()
                      if isinstance(v, (list, tuple))})
    except FileNotFoundError:
        pass
    return poses

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

        # The neutral posture the solver leans toward when the target does not
        # care -- see middle_ik.CENTER_WEIGHT. The saved 'start' pose, because
        # that is the configuration the operator chose as "how this arm should
        # look"; without it the extra seventh joint is spent arbitrarily.
        self.q_center = None
        try:
            poses = load_poses()
            name = os.environ.get("MIDDLE_CENTER_POSE", "start")
            if name in poses:
                n = robot.joints.num_actuated_joints
                self.q_center = np.asarray(poses[name][:n], dtype=np.float64)
        except Exception:
            pass

        self.lock = threading.Lock()
        self.enabled = False
        self.target = None            # (position(3), wxyz(4))
        self.last_cmd = 0.0
        self.q_cmd = None             # last commanded configuration
        self.measured = None          # latest /joint_states, arm joints only
        self.rejects = 0
        self.joint_goal = None        # named-pose destination, radians
        self.goal_name = ""
        self.pose_queue = []          # [(goal, label), ...] still to visit
        self.gate = None
        self.me = None
        self.gate_holding = False
        self.gate_said = 0.0

        self.create_subscription(JointState, f"{NS}/joint_states", self._on_js, 1)
        self.create_subscription(PoseStamped, f"{NS}/cmd_pose", self._on_pose, 1)
        self.create_subscription(String, f"{NS}/cmd_pose_name", self._on_pose_name, 1)
        self.create_subscription(String, f"{NS}/save_pose", self._on_save_pose, 1)
        self.create_subscription(Bool, f"{NS}/enable", self._on_enable, 1)
        # Exit so start.sh's supervisor relaunches this agent with fresh code.
        # The driver is untouched, so the arm keeps its torque -- this is the
        # way to reload head_agent; restarting the container is not.
        self.create_subscription(Bool, f"{NS}/reset", self._on_reset, 1)

        self.pub_ee = self.create_publisher(PoseStamped, f"{NS}/ee_pose", 1)
        self.pub_active = self.create_publisher(Bool, f"{NS}/active", 1)
        self.pub_names = self.create_publisher(String, f"{NS}/pose_names", 1)
        self.pub_clearance = self.create_publisher(String, f"{NS}/clearance", 1)
        self.create_timer(1.0, self._names_tick)
        if RIG_GATE:
            self._init_gate()
        # The real xs_sdk interface: one JointGroupCommand for the whole arm
        # group, positions in radians in joint_order. Same topic and message
        # pose.py and move_joint.py already use, so this agent is one more
        # publisher to it rather than a parallel control path.
        self.pub_cmd = self.create_publisher(
            JointGroupCommand, f"{NS}/commands/joint_group", 1)

        self.create_timer(1.0 / STREAM_HZ, self._control_tick)
        self.create_timer(1.0 / 20.0, self._state_tick)

    # ---- collision gate --------------------------------------------------
    def _init_gate(self):
        try:
            sys.path.insert(0, os.path.dirname(RIG_GATE_FILE))
            from rig_collision import RigGate
            gate = RigGate(RIG_GATE_FILE, margins={"structure": RIG_GATE_MARGIN,
                                                   "gripper": RIG_GATE_MARGIN})
        except Exception as e:
            self.get_logger().error(
                f"collision gate REQUESTED but unavailable: {e} -- running WITHOUT it. "
                f"Is {os.path.dirname(RIG_GATE_FILE)} mounted (docker-compose.yml)?")
            return
        me = gate.by_ns(NS)
        if me is None:
            self.get_logger().error(
                f"collision gate: no arm with ns {NS!r} in {RIG_GATE_FILE} -- running WITHOUT it")
            return
        n = self.robot.joints.num_actuated_joints
        if gate.arms[me].n != n:
            self.get_logger().error(
                f"collision gate: {RIG_GATE_FILE} models {me!r} with {gate.arms[me].n} joints, "
                f"the URDF has {n} -- re-run tools/fit_capsules.py. Running WITHOUT it")
            return
        self.gate, self.me = gate, me
        for arm, topic in RIG_PEERS.items():
            if arm == me:
                continue
            self.create_subscription(
                JointState, topic, lambda m, a=arm: self.gate.update(a, m.position), 1)
        self.create_subscription(Float32, LIFT_HEIGHT_TOPIC,
                                 lambda m: self.gate.set_lift(m.data), 1)
        self.get_logger().info(
            f"collision gate ON as {me!r}: margin {RIG_GATE_MARGIN * 1e3:.0f} mm "
            f"(camera {gate.margins['camera'] * 1e3:.0f}), peers {[a for a in RIG_PEERS if a != me]}")

    def _gate_allows(self, q_goal, q_now, why):
        if self.gate is None:
            return True
        try:
            v = self.gate.check(self.me, np.asarray(q_goal, float), q_now)
        except Exception as e:
            self.get_logger().error(f"collision gate failed ({e}) -- allowing {why}",
                                    throttle_duration_sec=5.0)
            return True
        now = self.get_clock().now().nanoseconds * 1e-9
        if v.ok:
            if self.gate_holding:
                self.gate_holding = False
                self.get_logger().info(f"collision gate: clear again ({v})")
            return True
        if not self.gate_holding or now - self.gate_said > 2.0:
            self.gate_said = now
            self.get_logger().warn(
                f"COLLISION GATE holding {why}: {v}. Back away; the arm holds until "
                "the pair clears the margin.")
        self.gate_holding = True
        return False

    def _publish_clearance(self, q):
        try:
            v = self.gate.clearance(self.me, q)
        except Exception:
            return
        m = String()
        m.data = json.dumps({
            "arm": self.me, "holding": bool(self.gate_holding),
            "distance_m": None if not math.isfinite(v.distance_m) else round(v.distance_m, 4),
            "margin_m": round(v.margin_m, 4),
            "slack_m": None if not math.isfinite(v.slack_m) else round(v.slack_m, 4),
            "pair": list(v.pair) if v.pair else None,
            "peers": v.peers, "missing": v.skipped,
        })
        self.pub_clearance.publish(m)

    # ---- inputs ----------------------------------------------------------
    def _on_js(self, msg):
        n = self.robot.joints.num_actuated_joints
        if len(msg.position) < n:
            return
        with self.lock:
            self.measured = np.asarray(msg.position[:n], dtype=np.float64)
            if self.q_cmd is None:
                self.q_cmd = self.measured.copy()
        if self.gate is not None:
            self.gate.update(self.me, msg.position[:n])

    def _on_enable(self, msg):
        with self.lock:
            if msg.data and not self.enabled:
                # Drop any stale target and re-sync the commanded configuration
                # to where the arm actually is; anchoring on a stale command
                # would step the arm by however far it had drifted.
                self.target = None
                self.joint_goal = None
                self.pose_queue = []
                self.rejects = 0
                if self.measured is not None:
                    self.q_cmd = self.measured.copy()
                self.get_logger().info("enabled")
            elif not msg.data and self.enabled:
                self.target = None
                self.joint_goal = None
                self.pose_queue = []
                self.get_logger().info("disabled -- holding position")
            self.enabled = bool(msg.data)

    def _on_reset(self, msg):
        if not msg.data:
            return
        self.get_logger().warn("reset requested -- exiting; start.sh relaunches this agent, "
                               "the driver keeps holding the arm")
        with self.lock:
            self.enabled = False
            self.target = None
            self.joint_goal = None
            self.pose_queue = []
        raise SystemExit(17)

    def _on_pose(self, msg):
        p = msg.pose.position
        want_p = np.array([p.x, p.y, p.z], dtype=np.float32)
        want_q = quat_wxyz_from_msg(msg.pose.orientation)

        with self.lock:
            if not self.enabled or self.q_cmd is None:
                return
            why = self._reject_reason(want_p)
            if why:
                self.rejects += 1
                if self.rejects % 25 == 1:
                    self.get_logger().warn(f"target refused ({self.rejects}): {why}")
                return
            self.target = (want_p, want_q)
            # Streaming and a named move must not fight over q_cmd.
            self.joint_goal = None
            self.pose_queue = []
            self.last_cmd = self.get_clock().now().nanoseconds * 1e-9

    def _on_save_pose(self, msg):
        """Record where this arm is right now, under a name.

        The manipulators have had this since the beginning; this arm did not,
        which meant a whole-rig 'start' or 'rest' pose could never be captured
        -- two arms would save and the camera arm would be left out, so the
        rig had no complete configuration to return to. Same topic name and
        same semantics as arm_agent's, so one key in rig_key.py can save all
        three at once.

        Saving does NOT require the arm to be enabled: reading where it is is
        not commanding it, and the natural workflow is to push the arm into
        place with torque off and then record it.
        """
        name = msg.data.strip()
        if not name:
            return
        with self.lock:
            q = None if self.measured is None else [float(v) for v in self.measured]
        if q is None:
            self.get_logger().error(
                f"cannot save {name!r} -- no joint_states yet (is the driver up?)")
            return
        try:
            import yaml
            try:
                with open(POSES_FILE) as f:
                    data = yaml.safe_load(f) or {}
            except FileNotFoundError:
                data = {}
            data[name] = [round(v, 4) for v in q]
            # Write via a temporary file: this file is bind-mounted and read
            # back by load_poses() on every use, so a half-written file would
            # be read by the next pose recall.
            tmp = POSES_FILE + ".tmp"
            with open(tmp, "w") as f:
                f.write(POSES_HEADER)
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)
            os.replace(tmp, POSES_FILE)
        except Exception as e:
            self.get_logger().error(f"could not save {name!r}: {e}")
            return
        self.get_logger().info(
            f"saved pose {name!r}: " + " ".join(f"{v:+.3f}" for v in q))

    def _on_pose_name(self, msg):
        name = msg.data.strip()
        if not name:
            return
        poses = load_poses()
        if name not in poses:
            self.get_logger().error(
                f"no pose {name!r}; have: {', '.join(sorted(poses))}")
            return
        # WAYPOINTS, same convention as arm_agent: '<name>_via1', '_via2', ...
        # are visited in order before '<name>'. This arm's straight ramp from
        # start to rest swings the camera through the left arm's parked
        # forearm; rest_via1 turns the wrist first and folds second.
        labels = [name]
        k = 1
        while f"{name}_via{k}" in poses:
            labels.insert(len(labels) - 1, f"{name}_via{k}")
            k += 1
        queue = []
        for label in labels:
            goal = np.asarray(poses[label], dtype=np.float64)
            lims = [cfg.JOINT_LIMITS[j] for j in cfg.JOINT_NAMES[:len(goal)]]
            bad = [f"{cfg.JOINT_NAMES[i]}={v:.3f} outside [{lo:.3f}, {hi:.3f}]"
                   for i, (v, (lo, hi)) in enumerate(zip(goal, lims))
                   if not lo <= v <= hi]
            if bad:
                self.get_logger().error(f"pose {label!r} refused: {'; '.join(bad)}")
                return
            queue.append((goal, label))
        with self.lock:
            if not self.enabled:
                self.get_logger().warn(f"pose {name!r} ignored -- not enabled")
                return
            if self.q_cmd is None:
                self.get_logger().warn(f"pose {name!r} ignored -- no joint_states yet")
                return
            goal, label = queue.pop(0)
            n = min(len(goal), len(self.q_cmd))
            self.target = None
            self.joint_goal = goal[:n]
            self.goal_name = label
            self.pose_queue = queue
            secs = float(np.max(np.abs(self.joint_goal - self.q_cmd[:n]))) / POSE_SPEED
        self.get_logger().info(
            f"pose {name!r}: ramping over ~{secs:.1f} s at {POSE_SPEED} rad/s"
            + (f" via {' -> '.join(l for _, l in queue)}" if queue else ""))

    def _reject_reason(self, want_p):
        """Gross nonsense only. Distance from the arm is NOT a reason.

        This used to refuse any target more than MAX_STEP_M from the last
        COMMANDED configuration's FK. The solver advances that configuration
        under a joint-velocity clamp, so it necessarily lags a moving target,
        and a commander that accumulates (rig_key adds a step per key repeat)
        pulls ahead of it -- especially if this agent has just dropped a target
        on a timeout and stopped while the operator kept pressing. Once the gap
        passed the limit, every later command failed the same test and the arm
        ignored the keyboard until it was disabled and re-enabled. The
        manipulators had the identical bug; see arm_agent.py's MAX_LEAD_M note
        for the full account.

        Nothing is lost by dropping it: solve() clamps every joint to what it
        could travel in one control period, so a far target is approached at a
        bounded rate rather than lunged at.
        """
        if not np.all(np.isfinite(np.asarray(want_p, dtype=float))):
            return "target contains NaN or infinity"
        for i, axis in enumerate("xyz"):
            lo, hi = WORKSPACE[axis]
            if not (lo <= float(want_p[i]) <= hi):
                return f"{axis}={want_p[i]:.3f} outside workspace [{lo}, {hi}]"
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
    def _names_tick(self):
        try:
            names = sorted(load_poses())
        except Exception as e:
            self.get_logger().warn(f"cannot read poses: {e}",
                                   throttle_duration_sec=30.0)
            return
        m = String()
        m.data = json.dumps(names)
        self.pub_names.publish(m)

    def _pose_tick(self, goal, prev_q, name):
        """One 50 Hz step of a named move: walk q_cmd toward the goal.

        The ramp lives here rather than trusting xs_sdk with the whole goal
        because a JointGroupCommand is applied INSTANTLY -- the driver has no
        goal-time concept on this topic, so the pacing has to be in the stream.
        """
        n = len(goal)
        step = POSE_SPEED / STREAM_HZ
        q_new = prev_q.copy()
        q_new[:n] = prev_q[:n] + np.clip(goal - prev_q[:n], -step, step)
        done = bool(np.max(np.abs(goal - q_new[:n])) < 1e-4)
        # Gate the STEP, not just the destination: a named move is a ramp of
        # small steps, so refusing one holds the arm exactly where the pair
        # would otherwise start closing, and the move resumes by itself when
        # the other arm has gone.
        if not self._gate_allows(q_new, prev_q, f"pose {name!r}"):
            return
        nxt = None
        with self.lock:
            if self.joint_goal is None:      # cancelled while we computed
                return
            self.q_cmd = q_new
            if done:
                if self.pose_queue:
                    goal2, label2 = self.pose_queue.pop(0)
                    self.joint_goal = goal2[:len(self.joint_goal)]
                    self.goal_name = label2
                    nxt = label2
                else:
                    self.joint_goal = None
        if not self.dry_run:
            self.pub_cmd.publish(
                JointGroupCommand(name=GROUP, cmd=[float(v) for v in q_new]))
        if done:
            self.get_logger().info(f"pose {name!r} reached"
                                   + (f" -- next: {nxt!r}" if nxt else ""))

    def _control_tick(self):
        with self.lock:
            if not self.enabled or self.q_cmd is None:
                return
            if self.target is None:
                goal = None if self.joint_goal is None else self.joint_goal.copy()
                gname = self.goal_name
                prev_q = self.q_cmd.copy()
                if goal is None:
                    return
            else:
                goal = None
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self.last_cmd > CMD_TIMEOUT_S:
                    self.target = None
                    self.get_logger().warn("cmd_pose timed out -- holding")
                    return
                target_p, target_q = self.target
                prev_q = self.q_cmd.copy()

        if goal is not None:
            return self._pose_tick(goal, prev_q, gname)

        n = self.robot.joints.num_actuated_joints
        try:
            q_new = self.solve(
                target_position=target_p,
                target_wxyz=target_q,
                prev_q=prev_q,
                dt=1.0 / STREAM_HZ,
                joint_velocity_limits=np.full(n, JOINT_VELOCITY_LIMIT, np.float32),
                q_center=self.q_center,
            )
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return

        if not np.all(np.isfinite(q_new)):
            self.get_logger().error("IK returned non-finite joints -- ignoring")
            return

        if not self._gate_allows(q_new, prev_q, "stream"):
            return          # q_cmd untouched: the arm holds its last command

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
        if self.gate is not None:
            self._publish_clearance(q)


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
