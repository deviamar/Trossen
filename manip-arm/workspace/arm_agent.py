#!/usr/bin/env python3
"""Own this arm's SDK connection and drive it from ROS topics.

    ./arm_agent.py                     # namespace from ARM_NS, arm from ARM_IP
    ./arm_agent.py --ns /left_arm
    ./arm_agent.py --scale 0.5         # halve commanded motion, for first tries
    ./arm_agent.py --dry-run           # publish state, accept commands, send nothing

This is the only script here that is a long-running ROS node rather than a
one-shot CLI, and it is the component's whole interface to the rest of the rig.
Everything it does is on the contract in docs/topic-contract.md:

    subscribes  <ns>/zero          std_msgs/Bool               re-zero the origin
                <ns>/reset         std_msgs/Bool               restart after a fault
                <ns>/cmd_pose      geometry_msgs/PoseStamped   absolute EE target
                <ns>/cmd_joints    sensor_msgs/JointState      absolute joint target
                <ns>/cmd_pose_name std_msgs/String             a saved pose, by name
                <ns>/cmd_gripper   std_msgs/Float32            opening, metres
                <ns>/cmd_grip_force std_msgs/Float32           squeeze, newtons
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

# Workspace box in the arm's base frame, metres. A SANITY BACKSTOP against a
# publisher that has gone wrong, NOT a reach specification.
#
# The first version of this assumed a bench-mounted arm reaching forward and up:
# x in [-0.1, 0.75], z in [0.02, 0.90]. On the real rig both arms rest at
# x=-0.173, z=-0.392 in their own base frames -- the mounting puts the end
# effector below and behind the frame origin -- so every single command was
# refused before it could move, including a command to hold exactly where the
# arm already was. A guard that rejects the robot's own current pose is worse
# than no guard: it fails closed, silently, and looks like a dead pipeline.
#
# So: a generous cube around the base. The WXAI reaches about 0.75 m, so 1 m
# bounds anything physically achievable while still catching a target that is
# wildly wrong (a dropped decimal, an uninitialised value, metres vs
# millimetres). The guard that actually earns its keep for teleop is MAX_STEP_M
# below, which catches the tracking glitch mid-stream.
#
# Tighten these per-arm via the environment once the mounting is measured and
# you know which part of the volume the arm should never enter.
WORKSPACE = {
    "x": (float(os.environ.get("ARM_WS_X_MIN", -1.0)),
          float(os.environ.get("ARM_WS_X_MAX", 1.0))),
    "y": (float(os.environ.get("ARM_WS_Y_MIN", -1.0)),
          float(os.environ.get("ARM_WS_Y_MAX", 1.0))),
    "z": (float(os.environ.get("ARM_WS_Z_MIN", -1.0)),
          float(os.environ.get("ARM_WS_Z_MAX", 1.0))),
}

# ---------------------------------------------------------------------------
# MOUNT FRAME. Where this arm's own axes point in the world you command in.
#
# The WXAI arms are bolted to the vertical face of the scissor lift, so the
# arm's base frame is rotated relative to the rig. Commanding "+x" in that raw
# frame moves the arm in a direction that has nothing to do with the operator's
# +x, and every consumer would have to know how each arm happens to be bolted on
# to compensate. That knowledge belongs here, in the container that owns the
# arm, so that everything outside speaks one world-aligned frame.
#
# Each variable says WHICH ARM AXIS a world axis maps onto, which is how you
# would describe it out loud: "world +x should move the arm along its +z".
#
#   ARM_WORLD_X=+z  ARM_WORLD_Y=-y  ARM_WORLD_Z=+x
#
# Identity (+x/+y/+z) leaves the arm's own frame untouched.
#
# Applied both ways: incoming cmd_pose is rotated world -> arm, and published
# ee_pose is rotated arm -> world. That symmetry matters -- a jog tool anchors
# on ee_pose and adds a delta, so if only one direction were converted the
# anchor and the step would be in different frames and the arm would walk off
# diagonally.
_AXES = {"x": 0, "y": 1, "z": 2}


def _mount_matrix():
    """Columns are the images of world x, y, z in arm coordinates."""
    R = [[0.0] * 3 for _ in range(3)]
    for col, var in enumerate(("ARM_WORLD_X", "ARM_WORLD_Y", "ARM_WORLD_Z")):
        spec = os.environ.get(var, "+xyz"[0] + "xyz"[col]).strip().lower()
        sign = -1.0 if spec.startswith("-") else 1.0
        axis = spec.lstrip("+-")
        if axis not in _AXES:
            raise SystemExit(f"{var}={spec!r}: expected +x/-x/+y/-y/+z/-z")
        R[_AXES[axis]][col] = sign
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    if abs(det - 1.0) > 1e-6:
        # det -1 is a mirror, not a rotation: it would silently flip handedness
        # and make rotations come out backwards while translations looked fine.
        raise SystemExit(
            f"mount frame is not a rotation (det={det:+.1f}). Check the signs in "
            "ARM_WORLD_X/Y/Z -- an even number of minus signs is required.")
    return R


MOUNT_R = _mount_matrix()
MOUNT_IS_IDENTITY = all(
    abs(MOUNT_R[i][j] - (1.0 if i == j else 0.0)) < 1e-9
    for i in range(3) for j in range(3))


def world_to_arm(v):
    return [sum(MOUNT_R[i][k] * v[k] for k in range(3)) for i in range(3)]


def arm_to_world(v):
    return [sum(MOUNT_R[k][i] * v[k] for k in range(3)) for i in range(3)]


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
        self.grip_force = None
        self.rejects = 0
        # World-frame offset making the CURRENT end effector read as (0,0,0).
        # Captured once on connect, and again on <ns>/zero.
        #
        # Position only, never orientation: zeroing a rotation would make
        # "level" mean whatever the wrist happened to be doing at startup,
        # which is far more confusing than a non-zero number.
        # Restored from disk when this arm has been zeroed before. A restart
        # after a fault MUST keep the old frame: re-zeroing to wherever the arm
        # ended up would silently shift every coordinate the operator has
        # written down by however far it moved before it faulted.
        self.origin = pose_lib.load_origin(cfg.ARM_NAME)
        self.zero_on_start = os.environ.get("ARM_ZERO_ON_START", "true").lower() == "true"
        if self.origin is not None:
            # Said out loud on purpose. A restored origin looks identical to a
            # fresh one until the arm has moved, so without this line there is
            # no way to tell from the logs whether the frame survived.
            self.get_logger().info(
                f"origin restored from config/origin.yaml: "
                f"({self.origin[0]:+.3f} {self.origin[1]:+.3f} {self.origin[2]:+.3f}). "
                "Same frame as before the restart.")
        self.reset_requested = False
        self.losses = 0
        # Set when the CONTROLLER itself reports a singularity, so _lost can
        # clear the latched error and keep the connection instead of dying.
        self.singular = False
        # (mode, value) last actually sent to the gripper, so it is not resent
        # every tick. Cleared whenever the connection state changes, because
        # the controller's own mode does not survive a fault or a re-enable.
        self.grip_applied = None
        # The arm mode actually in force, so it is entered once rather than on
        # every command. None means "idle, as the controller left it".
        self.mode_applied = None

        self.create_subscription(PoseStamped, f"{ns}/cmd_pose", self._on_pose, 1)
        self.create_subscription(JointState, f"{ns}/cmd_joints", self._on_joints, 1)
        self.create_subscription(String, f"{ns}/cmd_pose_name", self._on_pose_name, 1)
        self.create_subscription(Float32, f"{ns}/cmd_gripper", self._on_gripper, 1)
        self.create_subscription(Float32, f"{ns}/cmd_grip_force", self._on_grip_force, 1)
        self.create_subscription(Bool, f"{ns}/enable", self._on_enable, 1)
        self.create_subscription(String, f"{ns}/save_pose", self._on_save_pose, 1)
        self.create_subscription(Bool, f"{ns}/zero", self._on_zero, 1)
        self.create_subscription(Bool, f"{ns}/reset", self._on_reset, 1)

        self.pub_ee = self.create_publisher(PoseStamped, f"{ns}/ee_pose", 1)
        self.pub_js = self.create_publisher(JointState, f"{ns}/joint_states", 1)
        self.pub_active = self.create_publisher(Bool, f"{ns}/active", 1)
        self.pub_names = self.create_publisher(String, f"{ns}/pose_names", 1)
        self.create_timer(1.0, self._names_tick)

        self.create_timer(1.0 / STREAM_HZ, self._control_tick)
        self.create_timer(1.0 / 20.0, self._state_tick)

        self.base_frame = os.environ.get("ARM_BASE_FRAME", "base_link")

    # ---- losing the arm --------------------------------------------------
    def _lost(self, where, exc):
        """One place to handle the arm going away mid-callback.

        An exception raised inside a subscription callback propagates out of
        rclpy's executor and kills the process. That is what happened when the
        USB Ethernet adapter dropped: `get_cartesian_positions()` raised
        "Network is unreachable" inside _on_pose, the node died, and the
        container exited -- so a transient link problem looked like the whole
        arm stack falling over.

        Holding position is the right response. The controller drops to idle on
        its own when the connection goes, and idle on this arm is a braked hold,
        so the arm is already safe; what matters is that the node stays up,
        stops commanding, and says so exactly once rather than at 50 Hz.
        """
        with self.lock:
            was_enabled = self.enabled
            self.enabled = False
            self.target = None
            self.losses += 1
            self.grip_applied = None
        if not (was_enabled or self.losses == 1):
            return None
        self.get_logger().error(f"lost the arm in {where}: {exc}")

        # A singularity is not a lost arm. The link is fine; the controller
        # refused an IK solution and LATCHED the error, after which every
        # subsequent call rethrows it -- which is why one bad Cartesian target
        # used to take down the whole connection and need a container restart.
        #
        # clear_error() unlatches it in place. The arm stays connected and
        # braked, and joint-space commands still work: joint control has no IK
        # and so no singularity. That is the way OUT, and the message says so,
        # because "restart the container" is advice that does not help here --
        # the arm would come back up in the same pose and refuse again.
        if "singularity" in str(exc).lower():
            try:
                self.driver.clear_error()
                self.singular = True
                self.get_logger().error(
                    "SINGULARITY, not a lost link. Error cleared, still connected.")
                self.get_logger().error(
                    "Cartesian moves will keep failing HERE. Get out in joint "
                    "space: TAB in rig_key.py, then 1-6 / shift 1-6 -- or "
                    "'make home ARM=<name>' to run the saved start pose.")
                return None
            except Exception as e2:
                self.get_logger().error(f"clear_error() failed too: {e2}")

        self.get_logger().error(
            "disabled and holding. The arm is braked. Check the link "
            "(ping the controller), then: make restart SVC=<this arm>")
        return None

    # ---- inputs ----------------------------------------------------------
    def _on_enable(self, msg):
        with self.lock:
            if msg.data and not self.enabled:
                # Entering teleop: drop any stale target so the first command
                # has to arrive before anything moves. Without this the arm
                # would jump to wherever the operator was standing last time.
                self.target = None
                self.rejects = 0
                self.grip_applied = None
                self.mode_applied = None
                # ENABLING NO LONGER STARTS A SERVO.
                #
                # It used to call set_arm_modes(position) here, taking the arm
                # out of idle -- a braked, torque-capped hold -- and into a
                # closed-loop controller the moment you armed it, with nothing
                # to command yet. Now enable only ARMS the agent; position mode
                # is entered by _control_tick when a real command arrives. Until
                # then the arm stays idle, which is already a hold, so nothing
                # is lost and arming is never itself a motion.
                self.get_logger().info(
                    "armed -- still idle and braked. Position control starts on "
                    "the first motion command.")
            elif not msg.data and self.enabled:
                self.target = None
                self.grip_applied = None
                if not self.dry_run and self.mode_applied is not None:
                    try:
                        self.driver.set_arm_modes(trossen_arm.Mode.idle)
                    except Exception as e:
                        self.get_logger().warn(f"could not return to idle: {e}")
                self.mode_applied = None
                self.get_logger().info("disabled -- holding position")
            self.enabled = bool(msg.data)

    def _on_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        # cmd_pose arrives in the world-aligned frame; the SDK wants the arm's.
        wp = [p.x, p.y, p.z]
        if self.origin is not None:
            wp = [wp[i] + self.origin[i] for i in range(3)]
        pos = world_to_arm(wp)
        aa = world_to_arm(quat_to_angle_axis(o.x, o.y, o.z, o.w))
        want = list(pos) + list(aa)

        with self.lock:
            if not self.enabled:
                return
            if self.target:
                current = list(self.target)
            else:
                try:
                    current = list(self.driver.get_cartesian_positions())
                except Exception as e:
                    return self._lost("_on_pose", e)

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
            return self._lost("_state_tick", e)

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
        if self.singular:
            self.singular = False
            self.get_logger().info(
                "moving in joint space -- Cartesian will be re-checked on the "
                "next cmd_pose")
        # (mode, value) last actually sent to the gripper, so it is not resent
        # every tick. Cleared whenever the connection state changes, because
        # the controller's own mode does not survive a fault or a re-enable.
        self.grip_applied = None
        try:
            self.driver.set_arm_modes(trossen_arm.Mode.position)
            self.mode_applied = "position"
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

    def _on_save_pose(self, msg):
        """Record where the arm is right now under a name.

        This has to go through the agent. pose.py can do the same thing, but
        the driver connection is exclusive and the agent is holding it -- so
        while the rig is running, pose.py cannot reach the arm at all. Without
        this topic the only way to save a pose is to stop the container, which
        drops the arm to idle and loses the very posture you wanted to keep.
        """
        name = msg.data.strip()
        if not name:
            return
        try:
            pos = list(self.driver.get_all_positions())
        except Exception as e:
            return self._lost("_on_save_pose", e)
        try:
            vals = pose_lib.save_current(cfg.ARM_NAME, name, pos)
        except Exception as e:
            self.get_logger().error(f"could not save {name!r}: {e}")
            return
        self.get_logger().info(
            f"saved pose {name!r}: " + " ".join(f"{v:+.3f}" for v in vals[:cfg.NUM_ARM_JOINTS]))

    def _capture_origin(self, cart, why):
        """Make the current end effector read (0,0,0), and remember it."""
        with self.lock:
            self.origin = arm_to_world(cart[:3])
        try:
            pose_lib.save_origin(cfg.ARM_NAME, self.origin, note=why)
            kept = "saved -- survives a restart"
        except Exception as e:
            kept = f"NOT saved ({e}); a restart will re-zero"
        self.get_logger().info(
            f"zeroed ({why}): EE now reads (0, 0, 0); offset "
            f"({self.origin[0]:+.3f} {self.origin[1]:+.3f} {self.origin[2]:+.3f}) -- {kept}")

    def _on_zero(self, msg):
        """Make wherever the arm is right now read as (0, 0, 0)."""
        if not msg.data:
            return
        try:
            cart = list(self.driver.get_cartesian_positions())
        except Exception as e:
            return self._lost("_on_zero", e)
        with self.lock:
            # The commanded target is stored in ARM coordinates, so it does not
            # move when the origin does -- but a stale target would now mean a
            # different displacement, so drop it and make the next command
            # arrive fresh.
            self.target = None
        self._capture_origin(cart, "operator")

    def _on_reset(self, msg):
        """Restart this agent, clearing any latched controller error.

        Exits rather than trying to recover in place. The controller latches a
        fault and stops accepting commands, and clearing it means reconnecting
        with clear_error set -- which means tearing down the driver and building
        a new one. Doing that inside a running node is far more fragile than
        letting the process end and come back: `restart: unless-stopped` brings
        the container straight back, start.sh pings the arm first, and
        arm_agent reconnects with --clear-error. A few seconds, and the state
        afterwards is one nobody has to reason about.

        The arm does not move during any of this. Losing the connection puts
        every joint in idle, which on a WXAI is a braked hold.
        """
        if not msg.data:
            return
        self.get_logger().warn("reset requested -- exiting so the container restarts")
        with self.lock:
            self.enabled = False
            self.target = None
        self.reset_requested = True
        raise SystemExit(17)

    def _on_gripper(self, msg):
        """Position control, metres. For STAGING the fingers, not grasping."""
        with self.lock:
            self.gripper = max(cfg.GRIPPER_CLOSED,
                               min(cfg.GRIPPER_OPEN, float(msg.data)))
            self.grip_force = None      # position wins; they are exclusive modes

    def _on_grip_force(self, msg):
        """Force control, newtons. Positive opens, negative closes.

        THIS is what you grasp with, and the difference is not a nicety.
        set_gripper_position() drives to a commanded opening; close it on an
        object and the finger cannot reach that opening, so the controller sees
        a growing following error, calls it a fault, and drops the arm to idle.
        Meanwhile the force it applied on the way was whatever the position loop
        decided -- there is no limit on it.
        external_effort mode commands a FORCE directly: the finger squeezes at N
        newtons and stops wherever the object is. That bounds what the gripper
        can do to what it is holding, which is the property you want when the
        object is fragile and the controller has no idea it exists.
        """
        f = float(msg.data)
        limit = cfg.GRASP_FORCE_MAX_N
        with self.lock:
            self.grip_force = max(-limit, min(limit, f))
            self.gripper = None

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
            target, grip, grip_force = list(self.target), self.gripper, self.grip_force

        if self.dry_run:
            return
        try:
            if self.mode_applied != "position":
                # Entered lazily, on the first real command rather than on
                # enable, so arming an arm never starts a servo by itself.
                self.driver.set_arm_modes(trossen_arm.Mode.position)
                self.mode_applied = "position"
                self.get_logger().info("position control engaged")

            self.driver.set_cartesian_positions(
                target, trossen_arm.InterpolationSpace.cartesian,
                GOAL_TIME_S, False)
            # EDGE-TRIGGERED, and this matters far more than it looks.
            #
            # This block used to call set_gripper_mode() on EVERY tick -- 50
            # times a second for as long as any gripper command was active.
            # Mode-setting is a configuration call, not a streaming one: each
            # call reconfigures the controller, and doing that continuously
            # disturbed the whole arm. It showed up as the arm shaking whenever
            # the gripper was touched, on both arms, including one whose
            # position control was otherwise fine.
            #
            # The mode and the value are now sent only when they CHANGE. Both
            # modes latch in the controller -- external_effort holds the
            # commanded squeeze, position holds the commanded opening -- so
            # there is nothing to maintain by repetition.
            want = (("effort", grip_force) if grip_force is not None
                    else ("position", grip) if grip is not None
                    else None)
            if want is not None and want != self.grip_applied:
                mode, value = want
                if mode != (self.grip_applied or (None, None))[0]:
                    self.driver.set_gripper_mode(
                        trossen_arm.Mode.external_effort if mode == "effort"
                        else trossen_arm.Mode.position)
                if mode == "effort":
                    self.driver.set_gripper_external_effort(value, GOAL_TIME_S, False)
                else:
                    self.driver.set_gripper_position(value, GOAL_TIME_S, False)
                self.grip_applied = want
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
            return self._lost("_state_tick", e)

        if self.origin is None and self.zero_on_start:
            self._capture_origin(cart, "first start")
            # The posture the arm was in when its frame was defined is worth
            # keeping: it is the one place we know is reachable, singularity-free
            # and consistent with (0,0,0). That makes it the pose to come back
            # to after a fault. Never overwrite an existing 'home' -- the
            # operator may have chosen a better one deliberately.
            try:
                if "home" not in pose_lib.load_user_poses().get(cfg.ARM_NAME, {}):
                    pose_lib.save_current(cfg.ARM_NAME, "home", pos)
                    self.get_logger().info(
                        "saved this start posture as pose 'home'")
            except Exception as e:
                self.get_logger().warn(f"could not save 'home': {e}")

        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = list(URDF_JOINTS)
        js.position = [float(v) for v in pos]
        self.pub_js.publish(js)

        ee = PoseStamped()
        ee.header.stamp = stamp
        ee.header.frame_id = self.base_frame
        wpos = arm_to_world(cart[:3])
        if self.origin is not None:
            wpos = [wpos[i] - self.origin[i] for i in range(3)]
        waa = arm_to_world(cart[3:])
        ee.pose.position.x, ee.pose.position.y, ee.pose.position.z = wpos
        qx, qy, qz, qw = angle_axis_to_quat(*waa)
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
        if not MOUNT_IS_IDENTITY:
            print(f"  mount frame: world x->{os.environ.get('ARM_WORLD_X','+x')} "
                  f"y->{os.environ.get('ARM_WORLD_Y','+y')} "
                  f"z->{os.environ.get('ARM_WORLD_Z','+z')}")
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
