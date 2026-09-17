#!/usr/bin/env python3
"""Own this arm's SDK connection and drive it from ROS topics.

    ./arm_agent.py                     # namespace from ARM_NS, arm from ARM_IP
    ./arm_agent.py --ns /left_arm
    ./arm_agent.py --scale 0.5         # halve commanded motion, for first tries
    ./arm_agent.py --dry-run           # publish state, accept commands, send nothing

This is the only script here that is a long-running ROS node rather than a
one-shot CLI, and it is the component's whole interface to the rest of the rig.
Everything it does is on the contract in docs/topic-contract.md:

    subscribes  <ns>/float         std_msgs/Bool               hand-guide under gravity comp
                <ns>/zero          std_msgs/Bool               re-zero the origin
                <ns>/reset         std_msgs/Bool               restart after a fault
                <ns>/cmd_pose      geometry_msgs/PoseStamped   absolute EE target
                <ns>/cmd_joints    sensor_msgs/JointState      absolute joint target
                <ns>/cmd_pose_name std_msgs/String             a saved pose, by name
                <ns>/cmd_gripper   std_msgs/Float32            opening, metres
                <ns>/cmd_grip_force std_msgs/Float32           squeeze, newtons
                <ns>/enable        std_msgs/Bool               accept commands or not

    publishes   <ns>/ee_pose       geometry_msgs/PoseStamped
                <ns>/joint_states  sensor_msgs/JointState      + effort, N*m
                <ns>/health        std_msgs/String             JSON: temps, efforts,
                                                               gravity comp, external
                <ns>/pose_names    std_msgs/String             JSON list of saved poses
                <ns>/active        std_msgs/Bool

THREE WAYS TO COMMAND IT, and they are for different things:

  cmd_pose       streaming Cartesian, 50 Hz, for teleop. POSITION ONLY while
                 ARM_HOLD_ORIENTATION is true (the default) -- the wrist holds
                 its attitude and the orientation in the message is ignored.
                 The controller solves the IK. Executed as capped VELOCITY
                 (ARM_STREAM_MODE) -- smooth under an explicit speed limit
                 rather than paced by goal_time, and clamped rather than
                 refused when the target runs ahead.
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

A named or joint move is DISCRETE, not streamed: it is sent with a goal time
computed from the distance, and it cancels any streaming target so the two
cannot fight over the arm mid-motion. It is STAGED -- the elbow (ARM_POSE_LEAD_
JOINT) travels alone first, then the remaining joints -- because on this rig the
start/rest swing sweeps an obstruction if the other joints move with it.

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

SAFETY. --enable false, or no cmd_pose for CMD_TIMEOUT_S, holds position. A
target outside the per-axis workspace box is refused outright; everything else
is CLAMPED rather than refused -- the arm chases at most MAX_LEAD per control
period and at most MAX_LIN_VEL / MAX_ANG_VEL of speed, so a teleop publisher
that jumps (a lost tracking frame, a controller waking up across the room)
becomes a brief slow drift that the next good frame corrects, never a lunge and
never a refusal the operator has to notice and clear.
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
from arm_kin import ArmKinematics

# Streaming rate. The SDK picks its interpolation from goal_time: over 0.2 s is
# quintic, over 0.001 s is linear, below that the value is applied immediately.
# At 50 Hz with goal_time just over the period we get linear interpolation
# between frames, which is what a stream of live targets wants -- quintic would
# be fighting to settle before the next target arrives.
STREAM_HZ = 50.0
GOAL_TIME_S = 1.0 / STREAM_HZ * 1.5

# HOW THE STREAM REACHES THE ARM. Two ways to turn a 50 Hz cmd_pose stream into
# motion, chosen by ARM_STREAM_MODE:
#
#   velocity (default)  Mode.velocity + set_cartesian_velocities. Each tick
#                       computes the pose error and commands a velocity toward
#                       the target, clamped to ARM_MAX_LIN_VEL / ARM_MAX_ANG_VEL.
#                       The clamp is the point: motion is smooth at any target
#                       distance, speed is bounded by an explicit number rather
#                       than by whatever goal_time works out to, and the current
#                       draw stays bounded with it. A far target is approached at
#                       the cap instead of being refused or lunged at.
#   position            the earlier behaviour: set_cartesian_positions with a
#                       goal_time of 1.5 control periods. Kept as the fallback;
#                       its speed is an artifact of target spacing, which is why
#                       fast hand motion used to arrive as jerks.
#
# VEL_KP is the proportional gain (1/s) from pose error to commanded velocity:
# below the caps, the arm closes the gap with time constant 1/VEL_KP. The caps
# are deliberately conservative; raise them per-arm in docker-compose.yml once
# the rig is trusted.
#   joint_ik (default)  Our own IK (arm_kin.py, damped least squares on the
#                       URDF) turns the same capped Cartesian velocity into
#                       JOINT velocities, streamed with set_arm_velocities. The
#                       controller's Cartesian IK is bypassed entirely -- it is
#                       unusable with the elbow folded backward, which is where
#                       this rig's rest pose lives. Works on either branch.
STREAM_MODE = os.environ.get("ARM_STREAM_MODE", "joint_ik").strip().lower()
IK_DAMPING = float(os.environ.get("ARM_IK_DAMPING", 0.05))
IK_W_ANG = float(os.environ.get("ARM_IK_W_ANG", 0.3))      # orientation-row weight
# How far the integrated joint command may LEAD the measured joint. This is
# the integral action that overcomes the controller's steady-state sag: with
# gravity compensation computed for an upright base, the loaded elbow sits up
# to ~10 deg short of its command (measured in the settle passes), and at 0.2
# rad of lead the command saturated before the elbow had enough error to
# move -- the light joints tracked, the elbow did not, and the tool left the
# commanded line. 0.5 rad lets a lagging joint be driven; the stall guard
# below covers the case where it is lagging because it is blocked.
IK_JOINT_LEAD = float(os.environ.get("ARM_IK_JOINT_LEAD", 0.5))   # rad, command vs measured
IK_GOAL_TIME_S = 3.0 / STREAM_HZ                            # 60 ms goal window
MAX_JOINT_VEL = float(os.environ.get("ARM_MAX_JOINT_VEL", 1.0))     # rad/s per joint
# Tool point relative to link_6, metres -- measured, see arm_kin.ArmKinematics.
TOOL_XYZ = [float(v) for v in os.environ.get("ARM_TOOL_XYZ", "0.1562 0.0002 -0.0003").split()]
URDF_PATH = os.environ.get("ARM_URDF", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "wxai_follower.urdf"))
MAX_LIN_VEL = float(os.environ.get("ARM_MAX_LIN_VEL", 0.4))    # m/s
MAX_ANG_VEL = float(os.environ.get("ARM_MAX_ANG_VEL", 1.2))    # rad/s
VEL_KP = float(os.environ.get("ARM_VEL_KP", 6.0))              # 1/s

# How far the arm will CHASE its target in one control period. The error is
# clamped to this before it becomes a velocity, which bounds the speed a single
# frame can ask for at VEL_KP x MAX_LEAD (keep VEL_KP x MAX_LEAD_M >=
# ARM_MAX_LIN_VEL if you raise the caps) and makes a distant target a steady
# approach rather than a lunge.
#
# THE CLAMPED VALUE IS NEVER STORED BACK INTO self.target. Writing it back was
# the other half of the deadlock described above: the stored target then
# tracked the arm instead of the operator, so the gap the old jump guard
# measured grew by design and every command eventually failed it. The clamp
# belongs to the velocity law, not to the goal.
#
# Keeping a runaway commander in check is the COMMANDER's job, and rig_key.py
# does it by holding its own accumulated target within a lead of the measured
# pose -- which is also what makes releasing a jog key stop the arm promptly.
MAX_LEAD_M = float(os.environ.get("ARM_MAX_LEAD_M", 0.07))     # m
MAX_LEAD_RAD = float(os.environ.get("ARM_MAX_LEAD_RAD", 0.4))  # rad
# Ticks of zero-velocity ramp-down after a stream ends before dropping to idle.
# Going straight to idle mid-motion is a brake snap; a few ticks of commanded
# zero lets the controller decelerate closed-loop first.
STOP_TICKS = 8

# Re-assert the gripper's operating mode if it has not been asserted within
# this many seconds. See ArmAgent.grip_stamp.
# NEVER re-assert on a timer. set_gripper_mode() briefly drops the arm's
# control loop -- the arm falls a few millimetres and catches itself, which on
# this rig is a bang you feel through the whole frame. So the mode is set ONCE
# on connect (see ArmAgent.__init__) and after that only a FAULT can change it,
# because a fault genuinely does reset it (and _lost() clears the cache).
GRIP_REASSERT_S = float("inf")

# NAMED MOVES ARE STAGED: this joint travels ALONE first, then everything else.
#
# The rig's start and rest poses differ almost entirely in the elbow -- a
# ~190 degree swing -- with every other joint within a few degrees. Sent as one
# simultaneous move, the shoulder and forearm roll drift during that swing and
# the forearm sweeps through an obstruction on the frame; with the elbow moved
# on its own first (the other joints parked where they are) the sweep stays
# clear. Joint index into the arm's 0..5, empty string disables staging.
# ARM_POSE_ORDER generalises the lead joint: stages separated by ';', joints
# within a stage by ','. Every joint not named moves in a final stage. "2" is
# the elbow alone then the rest; "1;2" is shoulder, then elbow, then the rest;
# "1,3;2" moves shoulder and forearm roll together first. Empty = one move.
#
# MEASURED, NOT ASSUMED: the elbow-first order stalled at the rest fold with
# +21..+24 N*m and no motion while gravity accounts for at most 7 N*m over the
# whole swing -- the forearm was being driven into the frame. Which joint has
# to move first to clear it is a fact about the rig, found by jogging in joint
# mode, and it belongs here as a setting.
_order = os.environ.get("ARM_POSE_ORDER", os.environ.get("ARM_POSE_LEAD_JOINT", "2"))
POSE_ORDER = [[int(j) for j in stage.split(",") if j.strip()]
              for stage in _order.split(";") if stage.strip()]
POSE_LEAD_JOINT = POSE_ORDER[0][0] if POSE_ORDER and POSE_ORDER[0] else None

# CARTESIAN CONTROL IS ONLY OFFERED ON THE ELBOW-POSITIVE BRANCH.
#
# Measured on this rig: with the elbow folded backward (past about -110 deg)
# the controller's Cartesian IK is unusable -- at -130 deg a 1 cm request in
# velocity mode swung the shoulder from +2 to +136 deg (0.76 m of end-effector
# travel) before the controller declared a singularity; at -150 and -196 deg it
# refused outright in both modes. Poses on that branch are for PARKING (rest,
# the fold) and are reached in joint space; teleop starts from the positive
# branch. Below this elbow angle, cmd_pose is refused here, with a message,
# rather than handed to the controller.
CART_ELBOW_MIN = float(os.environ.get("ARM_CART_ELBOW_MIN", -1.9))   # rad, ~-109 deg

# SETTLE CORRECTION. The controller's position loop lands short by an amount
# proportional to how wrong its gravity feedforward is -- and on this mount it
# is wrong (observed: shoulder 11 deg and elbow 5.5 deg off after a move whose
# final stage commanded the exact target). Re-sending the same target does
# not help a proportional offset. Commanding target + residual does: after
# each named move the residual is measured and folded into a short corrective
# command, up to POSE_SETTLE_PASSES times, until every joint is within
# POSE_SETTLE_TOL. Streaming teleop needs none of this (its loop is closed on
# the measured pose every tick); this is only for discrete joint moves.
POSE_SETTLE_TOL = float(os.environ.get("ARM_POSE_SETTLE_TOL", 0.02))   # rad
POSE_SETTLE_PASSES = int(os.environ.get("ARM_POSE_SETTLE_PASSES", 4))
# Fraction of the residual folded into each correction. 1.0 assumes the loop
# lands short by a fixed proportion and overshoots when it does not -- the
# shoulder bounced -11 / +2 / -12 / +3 deg under it (that version also did not
# accumulate, which made it worse). Measured at 0.5: each pass removed ~60% of
# the residual, i.e. the loop's shortfall ratio is ~1.2 -- so 0.8 lands inside
# tolerance in ONE pass and still converges if the ratio drifts to 1.5.
POSE_SETTLE_GAIN = float(os.environ.get("ARM_POSE_SETTLE_GAIN", 0.8))
POSE_SETTLE_S = 1.0

# STALL GUARD. A joint pulling more than STALL_NM while not moving for STALL_S
# during a named move is pushing on something. The controller would let it
# push until the rotor hits 95 C (observed: 25 s, twice), so the move is
# abandoned here instead: the arm is told to hold where it is, which drops the
# effort to whatever gravity actually needs, and the log names the joint.
STALL_NM = float(os.environ.get("ARM_STALL_NM", 15.0))
STALL_VEL = 0.02          # rad/s -- below this it is "not moving"
STALL_S = float(os.environ.get("ARM_STALL_S", 1.0))
# Below this the lead joint is not worth a stage of its own.
POSE_LEAD_MIN_RAD = 0.05
# Pause after the lead stage's goal_time before the second stage is sent.
POSE_STAGE_SETTLE_S = 0.3

# MOTOR TEMPERATURES. The controller idles the whole arm the instant any
# rotor or driver passes 95 C -- mid-trajectory, without warning, and to this
# agent it looks identical to any other refusal. Observed: the elbow rotor at
# 96 C killed a start-pose swing a third of the way through. So the
# temperatures are read with the state and surfaced BEFORE the controller acts
# on them: a warning past TEMP_WARN_C, and a named move refused outright past
# TEMP_REFUSE_C rather than started and abandoned at a random angle.
# SETTLE DEADBAND. Inside this much error the stream sends NOTHING: one zero
# velocity on the way in, then silence until the target moves away again. A
# velocity servo with nothing to do was sending near-zero commands at 50 Hz,
# and two things came of that -- a faint buzz at rest, and, at a pose with a
# straight wrist, the controller refusing every one of them as "near the
# current position in Cartesian space" until the fault-loop breaker tripped.
# Holding still should not involve the controller at all.
SETTLE_M = float(os.environ.get("ARM_SETTLE_M", 0.002))
SETTLE_RAD = float(os.environ.get("ARM_SETTLE_RAD", 0.02))

TEMP_WARN_C = float(os.environ.get("ARM_TEMP_WARN_C", 75.0))
TEMP_REFUSE_C = float(os.environ.get("ARM_TEMP_REFUSE_C", 88.0))

# ORIENTATION CONTROL IS OFF while the mount frame is being settled.
#
# The end effector holds whatever attitude it had; only position is commanded.
# Two reasons, and the second is the one that matters:
#
#   * It halves what can go wrong. Position and orientation both pass through
#     the same mount rotation, so when the arm moves oddly there is no way to
#     tell a bad translation from a bad rotation by watching it. With the wrist
#     locked, every remaining surprise is a translation problem.
#   * A wrong mount rotation makes ORIENTATION misbehave in a way that looks
#     like bad translation: the wrist rolls while the arm translates, and the
#     tool tip sweeps an arc nobody asked for.
#
# Set ARM_HOLD_ORIENTATION=false to command attitude again -- which the headset
# will want, since a hand's rotation is half of what it is for.
HOLD_ORIENTATION = os.environ.get(
    "ARM_HOLD_ORIENTATION", "true").strip().lower() == "true"

if STREAM_MODE not in ("velocity", "position", "joint_ik"):
    raise SystemExit(f"ARM_STREAM_MODE={STREAM_MODE!r}: expected joint_ik, velocity or position")

# Hold position if no cmd_pose arrives for this long. Same reasoning as the
# base's 300 ms deadline: an input that dies should stop the robot, not leave
# the last target standing.
CMD_TIMEOUT_S = 0.3

# Seconds between repeats of a "controller refused a command" message. Holding
# a jog key against a joint limit would otherwise fault, clear and log every
# frame, burying everything else.
FAULT_LOG_S = 3.0
# More than FAULT_LOOP_N recoveries inside FAULT_LOOP_S is a loop, not a fault.
FAULT_LOOP_N = 3
FAULT_LOOP_S = 5.0

# THERE IS NO LONGER A "JUMP" REJECTION, and removing it fixed a deadlock that
# made an arm look dead until it was disabled and re-enabled.
#
# It used to refuse any cmd_pose more than 0.05 m from the last accepted target.
# That is a sound guard when the commanded target and the arm are the same
# thing -- which stopped being true the moment this agent started following
# targets at a CAPPED VELOCITY. Under velocity control the arm necessarily
# lags its target, and a commander that accumulates (rig_key adds 1 cm per key
# press, and key repeat fires ~30 times a second) runs its target further ahead
# every frame. Once that gap passed 0.05 m, EVERY later command was refused --
# including the ones that would have brought the two back together. Observed
# directly: `target refused (151): jump of 0.293 m` repeating at a frozen
# distance for half a minute, the arm ignoring the keyboard the whole time.
#
# What replaces it, in three places that each do one job:
#   * the WORKSPACE box below still refuses gross nonsense (a dropped decimal,
#     an uninitialised value, metres/millimetres confusion),
#   * MAX_LEAD_M / MAX_LEAD_RAD clamp how far the arm will CHASE a target in
#     one control period, so a wild target is approached slowly rather than
#     refused, and
#   * MAX_LIN_VEL / MAX_ANG_VEL bound the speed of that chase absolutely.
# A glitched teleop frame now costs 8 mm of travel before the next frame
# corrects it, instead of bricking the arm until someone notices.

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
# millimetres). The guards that actually earn their keep for teleop are the
# MAX_LEAD / velocity caps below, which bound how fast a wrong target can be
# chased instead of refusing it outright.
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


def quat_mul(a, b):
    """Hamilton product, xyzw. Rotation a APPLIED AFTER rotation b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _mat_to_quat(R):
    """Rotation matrix -> quaternion xyzw (Shepperd's method)."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s,
                (R[1][0] - R[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + R[i][i] - R[j][j] - R[k][k])) * 2.0
    q = [0.0, 0.0, 0.0, (R[k][j] - R[j][k]) / s]
    q[i] = 0.25 * s
    q[j] = (R[j][i] + R[i][j]) / s
    q[k] = (R[k][i] + R[i][k]) / s
    return tuple(q)


# The mount rotation as a quaternion, for converting ORIENTATIONS between the
# world-aligned frame and the arm's own. Orientations do not convert the way
# vectors do, and getting this wrong was a real bug: the old code rotated the
# angle-axis VECTOR through the mount matrix, which conjugates the rotation
# (M R M^T) instead of re-expressing it (M R). The two agree only at identity
# mount. The symptom was exactly "the wrist rotates about the wrong axis":
# a commanded world-z twist came out as a twist about whichever arm axis the
# mount maps z onto, while positions -- being vectors -- were fine, so it read
# as bad orientation handling rather than a frame error.
#
#   arm orientation   q_arm   = MOUNT_Q  x  q_world
#   world orientation q_world = MOUNT_Q^-1  x  q_arm
#
# With this convention an end effector whose axes line up with the WORLD axes
# reads identity, whatever the mount -- which is what an operator means by
# "level".
MOUNT_Q = _mat_to_quat(MOUNT_R)
MOUNT_Q_INV = quat_conj(MOUNT_Q)


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
        # Timestamps of recent successful clear_error() recoveries, for the
        # fault-loop breaker in _lost().
        self.recoveries = []
        # Remaining stages of a staged named move -- see _joint_plan_tick.
        self.joint_plan = None
        # Wall time until which a (final or only) stage is expected to be
        # executing, so the stall guard knows a move is in flight.
        self.pose_move_until = 0.0
        self.stall_since = None
        # Waypoints still to visit on a named move: [(joints, label), ...]
        self.path_queue = []
        self.path_next_at = 0.0
        # Last commanded joint goal of a staged move -- see _command_joints.
        self.stage_goal = None
        # Settle correction state: (true target, passes left, why, check_at).
        self.settle = None
        # Hottest rotor/driver per joint from the last state read, degrees C.
        self.temps = None
        # True while the stream is inside the settle deadband and silent.
        self.settled = False
        # Position-mode streaming forced for this enable (singularity fallback).
        self.pos_fallback = False
        # Last measured joint vector, for the elbow-branch gate in _on_pose.
        self.joints_now = None
        self.kin = (ArmKinematics(URDF_PATH, tool_xyz=TOOL_XYZ)
                    if STREAM_MODE == "joint_ik" else None)
        self.q_cmd = None          # joint_ik: integrated joint command
        # THE MODE THE CONTROLLER IS ACTUALLY IN, as opposed to what this agent
        # is doing. They are not the same thing: a named move ("position"), a
        # streamed Cartesian jog ("joint_ik") and a position-mode stream all
        # run in the controller's POSITION mode, so switching between them must
        # not touch the controller at all. Writing a mode drops its control
        # loop for an instant -- the arm sags and catches -- which is what made
        # the first few Cartesian commands after a pose move feel like the arm
        # disengaging and re-engaging.
        self.hw_mode = None
        self.ik_said = 0.0
        self.health_said = 0.0
        self.temp_said = 0.0
        # Rate-limits the "controller refused a command" message.
        self.fault_said = 0.0
        # (mode, value) last actually sent to the gripper, so it is not resent
        # every tick. Cleared whenever the connection state changes, because
        # the controller's own mode does not survive a fault or a re-enable.
        self.grip_applied = None
        # When grip_applied was last actually asserted to the controller. The
        # cache is an optimisation, not a source of truth -- the controller can
        # leave a mode without telling us (a latched fault, a clear_error, a
        # path in this file that forgets to record it) and a cache that is
        # merely believed produces a gripper that silently ignores commands.
        # Anything older than GRIP_REASSERT_S is re-asserted rather than
        # trusted; gripper commands are discrete key presses, so the cost is
        # one extra call and the benefit is that the failure cannot persist.
        self.grip_stamp = 0.0
        # The arm mode actually in force, so it is entered once rather than on
        # every command. None means "idle, as the controller left it".
        self.mode_applied = None
        # Remaining ticks of the zero-velocity ramp after a stream ends.
        self.stop_ticks = 0
        # Last measured Cartesian pose, in ARM coordinates. Cached because the
        # control tick already reads it every 20 ms and _on_pose would
        # otherwise ask the controller again at the same rate for a number it
        # has just been told.
        self.cart = None

        self.create_subscription(PoseStamped, f"{ns}/cmd_pose", self._on_pose, 1)
        self.create_subscription(JointState, f"{ns}/cmd_joints", self._on_joints, 1)
        self.create_subscription(String, f"{ns}/cmd_pose_name", self._on_pose_name, 1)
        self.create_subscription(Float32, f"{ns}/cmd_gripper", self._on_gripper, 1)
        self.create_subscription(Float32, f"{ns}/cmd_grip_force", self._on_grip_force, 1)
        self.create_subscription(Bool, f"{ns}/enable", self._on_enable, 1)
        self.create_subscription(String, f"{ns}/save_pose", self._on_save_pose, 1)
        self.create_subscription(Bool, f"{ns}/zero", self._on_zero, 1)
        self.create_subscription(Bool, f"{ns}/reset", self._on_reset, 1)
        # Hand-guiding: float the arm under gravity compensation so a clear
        # path can be SHOWN and recorded with <ns>/save_pose, instead of
        # searched for one stalled joint at a time. See _on_float.
        self.create_subscription(Bool, f"{ns}/float", self._on_float, 1)

        self.pub_ee = self.create_publisher(PoseStamped, f"{ns}/ee_pose", 1)
        self.pub_js = self.create_publisher(JointState, f"{ns}/joint_states", 1)
        self.pub_active = self.create_publisher(Bool, f"{ns}/active", 1)
        self.pub_names = self.create_publisher(String, f"{ns}/pose_names", 1)
        # JSON, 1 Hz: temperatures, efforts, the controller's own gravity
        # compensation and external-effort estimates. Diagnostic, not control.
        self.pub_health = self.create_publisher(String, f"{ns}/health", 1)
        self.create_timer(1.0, self._names_tick)

        self.create_timer(1.0 / STREAM_HZ, self._control_tick)
        self.create_timer(1.0 / 20.0, self._state_tick)

        self.base_frame = os.environ.get("ARM_BASE_FRAME", "base_link")

        # HOLD THE ARM THE MOMENT WE HAVE IT, rather than leaving it idle.
        #
        # Losing the connection drops the controller to idle, and idle on this
        # rig is NOT a hold: it is a torque-capped PID sized for an upright
        # base, and on the lift's vertical face a loaded arm sags through it --
        # which is the drop felt every time the agents start or restart. The
        # gap itself cannot be closed from here (nothing is connected during
        # it), but it can be ENDED as early as possible: take position control
        # at the pose the arm is actually in, which commands no motion and
        # holds it properly from the first tick.
        #
        # Skipped when a joint sits outside its limits -- entering position
        # mode there faults the controller before it reads any target, and the
        # message it prints is about limits, not about this line. ARM_HOLD_ON_
        # START=false leaves the old idle behaviour.
        if not dry_run and os.environ.get("ARM_HOLD_ON_START", "true").lower() == "true":
            try:
                lims = arm.limits(driver)
                now_q = list(driver.get_all_positions())
                blocked = arm.blocked_by_position(now_q, lims)
                if blocked:
                    self.get_logger().warn(
                        "not taking hold at startup: "
                        + "; ".join(f"{cfg.label(i)} at {p:.3f} outside [{lo:.3f}, {hi:.3f}]"
                                    for i, p, lo, hi, _ in blocked)
                        + " -- the arm stays idle (and may sag). ./recover.py")
                else:
                    driver.set_arm_modes(trossen_arm.Mode.position)
                    driver.set_arm_positions(
                        [float(v) for v in now_q[:cfg.NUM_ARM_JOINTS]], 1.0, False)
                    self.hw_mode = "position"
                    self.mode_applied = "position"
                    self.get_logger().info(
                        "holding at the startup pose (position mode) -- "
                        "idle alone lets a loaded arm sag on this mount")
            except Exception as e:
                self.get_logger().warn(f"could not take hold at startup: {e}")

        # ONE GRIPPER MODE FOR THE WHOLE SESSION: external_effort, set here.
        #
        # Both directions are forces -- close is negative, open is positive --
        # so nothing during operation ever has to switch modes, and the arm
        # never gets that momentary release. Position control of the gripper
        # (<ns>/cmd_gripper) still works, but it costs one mode change, so it
        # is for staging the fingers between tasks rather than for teleop.
        if not dry_run:
            try:
                driver.set_gripper_mode(trossen_arm.Mode.external_effort)
                self.grip_applied = ("effort", 0.0)
                self.grip_stamp = self.get_clock().now().nanoseconds * 1e-9
            except Exception as e:
                self.get_logger().warn(f"could not preset the gripper mode: {e}")

    # ---- losing the arm --------------------------------------------------
    def _ensure_arm_mode(self, name, enum):
        """Set the controller's arm mode only if it is not already there."""
        if self.hw_mode == name:
            return
        self.driver.set_arm_modes(enum)
        self.hw_mode = name

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
            # A fault mid-swing must NOT be followed by stage 2 firing on its
            # timer: the arm is wherever the controller stopped it, and the
            # remaining joints moving on their own is a move nobody asked for.
            self.joint_plan = None
            self.path_queue = []
            self.hw_mode = None        # a fault resets the controller's modes
            self.losses += 1
            self.grip_applied = None
        # ---- is this a REJECTED COMMAND or a LOST ARM? ----------------
        #
        # They look identical from here -- both arrive as an exception out of
        # the SDK -- and they need opposite responses. "Joint limit exceeded",
        # "singularity", a following error: the link is fine, the controller
        # refused one target and LATCHED the error, after which every later call
        # rethrows it. A dropped Ethernet adapter: the arm is genuinely gone.
        #
        # clear_error() is the test as well as the cure. If it works, the
        # connection is alive and the only real casualty was one bad command,
        # so this stays up and stays ARMED and the next command starts fresh.
        # If it throws, the arm really is unreachable and we fall through.
        #
        # Guessing from the message text was the earlier approach and it was
        # wrong: it only recognised "singularity", so a joint-limit error --
        # which is what wrist rotation near a singularity actually produces --
        # took the whole connection down and made every later key press do
        # nothing.
        recovered = False
        try:
            self.driver.clear_error()
            # clear_error() RECONFIGURES the driver, and configure() resets
            # every joint limit to the vendor defaults -- the same fact
            # arm.connect() documents. Without this line the recovery itself
            # was the fault: this rig runs joints 1 and 2 on the negative side,
            # the vendor floor for joint 2 is -0.2, so the instant the arm left
            # idle the controller read its own elbow at -1.4, declared "Joint
            # limit exceeded", dropped to idle, we cleared, it reconfigured,
            # and round again -- eight times a second. That was the jitter.
            arm.apply_limit_overrides(self.driver)
            recovered = True
        except Exception:
            pass

        if recovered:
            # A recovery that keeps recovering is not a recovery. Three inside
            # FAULT_LOOP_S means every command is being refused for a reason
            # clearing cannot fix (a joint outside even the override limits, a
            # target the controller will never accept), and re-entering the
            # servo at 8 Hz shakes the arm. Disarm and say why instead.
            now = self.get_clock().now().nanoseconds * 1e-9
            self.recoveries = [t for t in self.recoveries if now - t < FAULT_LOOP_S]
            self.recoveries.append(now)
            if len(self.recoveries) >= FAULT_LOOP_N:
                self.recoveries.clear()
                with self.lock:
                    self.enabled = False
                    self.target = None
                    self.mode_applied = None
                self.get_logger().error(
                    f"FAULT LOOP: the controller refused {FAULT_LOOP_N} commands "
                    f"in {FAULT_LOOP_S:.0f} s -- disarmed to stop the shaking. "
                    f"Last reason: {exc}")
                self.get_logger().error(
                    "If it says 'Joint limit exceeded', the arm is parked "
                    "outside its limits: walk it back in joint space (TAB in "
                    "rig_key, then 1-6 / shift-1-6), or ./recover.py. Then "
                    "re-enable.")
                return None
            with self.lock:
                self.enabled = was_enabled     # stay armed; only the target died
                self.target = None
                self.grip_applied = None
                # The controller drops out of position mode when it faults, so
                # the next command has to re-enter it rather than assume.
                self.mode_applied = None
                self.losses -= 1               # not a loss; nothing was lost
                now = self.get_clock().now().nanoseconds * 1e-9
                say = now - self.fault_said > FAULT_LOG_S
                if say:
                    self.fault_said = now
            if say:
                self.get_logger().error(f"controller refused a command in {where}: {exc}")
                self.get_logger().error(
                    "Error cleared, STILL CONNECTED and still armed. That target "
                    "was not reachable -- try a smaller step, or move in joint "
                    "space (TAB in rig_debug.py, then 1-6 / shift 1-6).")
            return None

        self.get_logger().error(f"lost the arm in {where}: {exc}")
        text = str(exc)
        if "overheat" in text.lower() or "temperature" in text.lower():
            self.get_logger().error(
                "OVERHEATED: the controller idled the arm to protect a motor. "
                "This is not a link or software problem -- let it cool (the "
                "arm holds), and park it folded rather than cantilevered. "
                "Re-enable when the temperatures below drop.")
        elif "CAN" in text:
            self.get_logger().error(
                "CAN FAULT inside the controller: a motor stopped answering "
                "the controller's own bus. Hot motors and a sagging supply "
                "both do this. Not a software problem; check power and heat.")
        else:
            self.get_logger().error(
                "disabled and holding. The arm is braked. Check the link "
                "(ping the controller), then: make restart SVC=<this arm>")
        return None

    # ---- inputs ----------------------------------------------------------
    def _on_enable(self, msg):
        with self.lock:
            if msg.data and not self.enabled:
                if self.mode_applied == "float" and not self.dry_run:
                    # Arming ends a float: idle first, so position control
                    # starts from a braked hold and not from a free joint.
                    try:
                        self._ensure_arm_mode("idle", trossen_arm.Mode.idle)
                    except Exception as e:
                        self.get_logger().warn(f"could not leave float: {e}")
                    self.mode_applied = None
                # Entering teleop: drop any stale target so the first command
                # has to arrive before anything moves. Without this the arm
                # would jump to wherever the operator was standing last time.
                self.target = None
                self.rejects = 0
                self.grip_applied = None
                self.mode_applied = None
                self.pos_fallback = False
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
                self.joint_plan = None
                self.path_queue = []
                self.grip_applied = None
                if not self.dry_run and self.mode_applied is not None:
                    try:
                        self._ensure_arm_mode("idle", trossen_arm.Mode.idle)
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
        # Orientation converts by composition, not by rotating the angle-axis
        # vector -- see the MOUNT_Q comment up top.
        q_arm = quat_mul(MOUNT_Q, (o.x, o.y, o.z, o.w))
        aa = quat_to_angle_axis(*q_arm)
        want = list(pos) + list(aa)

        with self.lock:
            if not self.enabled:
                return
            elbow = self.joints_now[2] if self.joints_now is not None else None
            if STREAM_MODE != "joint_ik" and elbow is not None and elbow < CART_ELBOW_MIN:
                self.rejects += 1
                if self.rejects % 25 == 1:
                    self.get_logger().warn(
                        f"cmd_pose refused: elbow at {math.degrees(elbow):+.0f} deg is on "
                        "the backward branch, where the controller's Cartesian IK is "
                        "unsafe (it threw the shoulder 130 deg once). Go to a "
                        "positive-elbow pose in joint space (S S / joint mode) first.")
                return

            if self.scale != 1.0 and self.cart is not None:
                # Scale about the MEASURED pose, so a half-scale command is
                # half the *motion*, not half the coordinate. Measured, not the
                # last target: scaling about the target compounds every frame,
                # which quietly turned --scale into an exponential decay toward
                # wherever the stream started.
                want = [c + (w - c) * self.scale
                        for c, w in zip(self.cart, want)]

            why = self._reject_reason(want)
            if why:
                self.rejects += 1
                # Rate-limited: a stuck publisher would otherwise fill the log
                # faster than anyone can read it.
                if self.rejects % 25 == 1:
                    self.get_logger().warn(f"target refused ({self.rejects}): {why}")
                return

            # Stored EXACTLY as the operator asked. The control loop clamps how
            # fast it chases this; it must not edit the goal itself.
            self.target = want
            self.joint_plan = None          # a jog cancels a pending stage 2
            self.settle = None
            self.last_cmd = self.get_clock().now().nanoseconds * 1e-9

    def _reject_reason(self, want):
        """Gross nonsense only -- distance from the arm is NOT a reason.

        See the note beside MAX_LEAD_M: refusing a far target is what made an
        arm ignore the keyboard indefinitely. A far target is now simply
        approached slowly.
        """
        if not all(math.isfinite(v) for v in want):
            return "target contains NaN or infinity"
        for i, axis in enumerate("xyz"):
            lo, hi = WORKSPACE[axis]
            if not (lo <= want[i] <= hi):
                return f"{axis}={want[i]:.3f} outside workspace [{lo}, {hi}]"
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
        # WAYPOINTS. '<name>_via1', '<name>_via2', ... are visited in order
        # before '<name>' itself. This is how a hand-guided clear path gets
        # replayed: the joint-order staging cannot know the frame is in the
        # way, but a path someone pushed the arm along by hand does.
        vias = []
        k = 1
        while f"{name}_via{k}" in poses:
            vias.append((list(poses[f"{name}_via{k}"]), f"{name}_via{k}"))
            k += 1
        with self.lock:
            self.path_queue = (vias[1:] + [(list(poses[name])[:cfg.NUM_ARM_JOINTS], name)]
                               if vias else [])
        # ARM JOINTS ONLY. A pose file entry carries the gripper opening too,
        # but a posture is not a grasp: replaying the gripper value would (a)
        # open or close on whatever is being held and (b) refuse the whole
        # move whenever the recorded opening sits at the mechanical stop --
        # which it does for any pose captured with the fingers pushed open.
        vias = [(v[:cfg.NUM_ARM_JOINTS], lbl) for v, lbl in vias]
        first = vias[0] if vias else (list(poses[name])[:cfg.NUM_ARM_JOINTS], name)
        if vias:
            self.get_logger().info(
                f"pose {name!r}: via {len(vias)} waypoint(s) -- "
                + " -> ".join(v[1] for v in vias) + f" -> {name}")
        self._command_joints(first[0], why=f"pose {first[1]!r}",
                             stage=not vias)

    def _on_joints(self, msg):
        if len(msg.position) < cfg.NUM_ARM_JOINTS:
            self.get_logger().error(
                f"cmd_joints needs at least {cfg.NUM_ARM_JOINTS} positions, "
                f"got {len(msg.position)}")
            return
        self._command_joints(list(msg.position), why="cmd_joints")

    def _command_joints(self, values, why, stage=True):
        """One discrete joint-space move. Checked, then sent once.

        stage=False sends it as a single move regardless of ARM_POSE_ORDER --
        a waypoint of a hand-guided path is already a safe hop.
        """
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

        with self.lock:
            self.joint_plan = None          # a new move supersedes a pending stage
            self.settle = None
            temps = self.temps
        if temps and max(temps) >= TEMP_REFUSE_C:
            i = max(range(len(temps)), key=lambda k: temps[k])
            self.get_logger().error(
                f"{why} REFUSED: {cfg.label(i)} is at {temps[i]:.0f} C. The "
                f"controller would idle the arm mid-move at 95. Let it cool "
                f"below {TEMP_REFUSE_C:.0f} first (fold it; a cantilevered elbow "
                "keeps heating).")
            return

        na = min(n, cfg.NUM_ARM_JOINTS)
        deltas = [target[i] - current[i] for i in range(na)]
        # Build the stage list: configured groups that actually need to move,
        # then everything left over. One stage means one plain move.
        stages, used = [], set()
        for group in POSE_ORDER:
            g = [j for j in group if j < na and abs(deltas[j]) > POSE_LEAD_MIN_RAD]
            if g:
                stages.append(g); used.update(g)
        # The FINAL stage always commands every joint, whatever its delta.
        # Joints left uncommanded through an earlier stage get dragged by the
        # ones that move (observed: the shoulder rode 18 deg up during an
        # elbow-only stage and was never brought back), so the last word has
        # to be the whole target.
        stages.append(list(range(na)))
        staged = stage and len(stages) > 1
        if not staged:
            stages = [list(range(na))]
        lead = stages[0][0] if staged else None

        stage1 = list(current)
        for j in stages[0]:
            stage1[j] = target[j]
        # Remember what was COMMANDED. Later stages build on this rather than
        # on a fresh measurement: a measurement taken while a joint is still
        # settling -- or being dragged by the joints that are moving -- freezes
        # that transient into the hold target (observed: elbow latched 9.5 deg
        # past its goal, shoulder 18 deg up).
        with self.lock:
            self.stage_goal = list(stage1)
        goal_time = cfg.goal_time_for([deltas[j] for j in stages[0]])

        if self.dry_run:
            self.get_logger().info(
                f"DRY RUN {why}: would move over {goal_time:.1f} s to "
                + " ".join(f"{v:+.3f}" for v in target[:cfg.NUM_ARM_JOINTS])
                + (f"  (staged: {cfg.label(lead)} first)" if staged else ""))
            return

        if staged:
            self.get_logger().info(
                f"{why}: stage 1/{len(stages)} -- "
                f"{'+'.join(cfg.label(j) for j in stages[0])} over {goal_time:.1f} s")
        else:
            self.get_logger().info(f"{why}: moving over {goal_time:.1f} s")
        try:
            self._ensure_arm_mode("position", trossen_arm.Mode.position)
            self.mode_applied = "position"
            self.driver.set_arm_positions(
                [float(v) for v in stage1[:cfg.NUM_ARM_JOINTS]], goal_time, False)
            with self.lock:
                now0 = self.get_clock().now().nanoseconds * 1e-9
                self.pose_move_until = now0 + goal_time + 1.0
                self.path_next_at = now0 + goal_time + POSE_STAGE_SETTLE_S
                self.settle = (None if staged else
                               (list(target[:cfg.NUM_ARM_JOINTS]), POSE_SETTLE_PASSES,
                                why, now0 + goal_time + POSE_STAGE_SETTLE_S))
            if staged:
                now = self.get_clock().now().nanoseconds * 1e-9
                with self.lock:
                    # Remaining stages, each sent when the previous has had
                    # its goal_time: (target, n, why, send_after, stages, k).
                    self.joint_plan = (list(target), n, why,
                                       now + goal_time + POSE_STAGE_SETTLE_S,
                                       stages, 1)
            if n > cfg.GRIPPER_INDEX:
                self.driver.set_gripper_mode(trossen_arm.Mode.position)
                self.driver.set_gripper_position(
                    float(target[cfg.GRIPPER_INDEX]), goal_time, False)
                # RECORD IT. This line put the gripper into POSITION mode while
                # grip_applied still said "effort", so _apply_gripper saw a
                # matching mode, skipped set_gripper_mode, and sent a FORCE
                # command to a gripper the controller was holding by position.
                # Nothing happened, and the only way out was disabling and
                # re-enabling the arm -- which clears this cache. Every named
                # pose (shift-R/shift-S) and every joint move went through here,
                # so "the gripper stops working after I use a pose" was this.
                with self.lock:
                    self.grip_applied = ("position",
                                         float(target[cfg.GRIPPER_INDEX]))
                    self.grip_stamp = self.get_clock().now().nanoseconds * 1e-9
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

    def _on_float(self, msg):
        """Gravity-compensated float, for hand-guiding. teach.py's mode, on a topic.

        Every arm joint goes to external_effort with a commanded effort of
        zero: the controller keeps applying its gravity and friction model and
        the zero rides on top, so the arm holds itself up and yields when
        pushed. The gripper stays idle (braked) so it does not fall open.

        NOT TORQUE-OFF, AND ONLY MEANINGFUL ON AN UPRIGHT MOUNT. The controller
        compensates for an upright base. On this rig's vertical mount that
        compensation is a wrong-way push with nothing opposing it -- the first
        attempt flung the shoulder at 6.3 rad/s -- so it is refused unless the
        mount frame is identity or ARM_FLOAT_FORCE=true. Floating disarms
        teleop; enable ends the float (back to idle) before arming.
        """
        if msg.data:
            if not MOUNT_IS_IDENTITY and os.environ.get("ARM_FLOAT_FORCE") != "true":
                # MEASURED: on the lift's vertical face the float threw the
                # shoulder to 6.29 rad/s inside a second and left the arm at
                # +140 deg shoulder / +90 deg roll. The controller's gravity
                # compensation is computed for an upright base; on any other
                # mount it is a push in the wrong direction with nothing to
                # oppose it. Not a drift. Not hand-guidable. Refused.
                self.get_logger().error(
                    "float REFUSED: this arm is not upright-mounted "
                    f"(ARM_WORLD_*), and gravity compensation on this mount "
                    "flings joints rather than floating them (observed: "
                    "shoulder at 6.3 rad/s). Hand-guide against IDLE instead, "
                    "or set ARM_FLOAT_FORCE=true if you really mean it.")
                return
            with self.lock:
                self.enabled = False
                self.target = None
                self.joint_plan = None
                self.path_queue = []
            if self.dry_run:
                return
            try:
                self.driver.set_arm_modes(trossen_arm.Mode.external_effort)
                self.hw_mode = "float"
                self.driver.set_arm_external_efforts(
                    [0.0] * cfg.NUM_ARM_JOINTS, 0.0, False)
            except Exception as e:
                return self._lost("_on_float", e)
            with self.lock:
                self.mode_applied = "float"
            self.get_logger().warn(
                "FLOATING under gravity compensation -- hand-guide it. Support "
                "it: the gravity model assumes an upright base and this arm is "
                "not on one. Record with <ns>/save_pose; float=false to brake.")
        else:
            with self.lock:
                floating = self.mode_applied == "float"
            if not floating:
                return
            if not self.dry_run:
                try:
                    self._ensure_arm_mode("idle", trossen_arm.Mode.idle)
                except Exception as e:
                    return self._lost("_on_float", e)
            with self.lock:
                self.mode_applied = None
            self.get_logger().info("float ended -- idle, braked hold")

    def _on_gripper(self, msg):
        """Position control, metres. For STAGING the fingers, not grasping."""
        with self.lock:
            self.gripper = max(cfg.GRIPPER_CLOSED,
                               min(cfg.GRIPPER_OPEN, float(msg.data)))
            self.grip_force = None      # position wins; they are exclusive modes
        self._apply_gripper()

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
        self._apply_gripper()

    def _apply_gripper(self):
        """Send the gripper command NOW, not on the arm's control tick.

        THIS IS WHY THE GRIPPER USED TO DO NOTHING. It was applied inside
        _control_tick, which begins:

            if not self.enabled or self.target is None:
                return

        -- so the gripper only moved while a Cartesian target was actively
        streaming. Arm an arm, press open, and nothing happened, because no
        cmd_pose had arrived to create a target. Move the arm first and the next
        gripper press worked, then stopped working again 300 ms after the last
        cmd_pose when the target timed out. Both halves of "left did nothing,
        right opened once and then never again" fall out of that one line.

        The gripper has nothing to do with where the arm is going, so it no
        longer rides on the arm's target.

        THE MODE IS SET ONLY WHEN IT CHANGES. It latches in the controller, so
        it does not need re-asserting -- and re-asserting it at 50 Hz was a real
        bug that shook the whole arm. The VALUE is sent on every command, since
        these are discrete key presses and a repeat should re-assert the squeeze.
        """
        with self.lock:
            if self.dry_run:
                return
            # NOT gated on enable. The gripper has its own mode and nothing to
            # do with where the arm is going; requiring the arm to be enabled
            # first was the "press enable, disable, enable again before the
            # gripper works" hassle -- for no safety benefit.
            want = (("effort", self.grip_force) if self.grip_force is not None
                    else ("position", self.gripper) if self.gripper is not None
                    else None)
            applied = self.grip_applied
            now = self.get_clock().now().nanoseconds * 1e-9
            stale = now - self.grip_stamp > GRIP_REASSERT_S
        if want is None:
            return

        mode, value = want
        try:
            if applied is not None and applied[0] != mode:
                self.get_logger().warn(
                    f"gripper mode {applied[0]} -> {mode}: the arm will twitch. "
                    "Force (cmd_grip_force) both ways avoids this.")
            if applied is None or applied[0] != mode or stale:
                self.driver.set_gripper_mode(
                    trossen_arm.Mode.external_effort if mode == "effort"
                    else trossen_arm.Mode.position)
            if mode == "effort":
                self.driver.set_gripper_external_effort(value, GOAL_TIME_S, False)
            else:
                self.driver.set_gripper_position(value, GOAL_TIME_S, False)
        except Exception as e:
            self.get_logger().error(f"gripper command refused: {e}")
            return
        with self.lock:
            self.grip_applied = want
            self.grip_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"gripper: {mode} {value:+.3f}"
            + ("" if applied and applied[0] == mode and not stale
               else f"  (mode -> {mode})"))

    # ---- outputs ---------------------------------------------------------
    def _control_tick(self):
        with self.lock:
            streaming = self.enabled and self.target is not None
            if streaming:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self.last_cmd > CMD_TIMEOUT_S:
                    # Input went quiet. Drop the target rather than holding it:
                    # the arm stays where it is and a resumed stream starts
                    # fresh.
                    self.target = None
                    streaming = False
                    self.get_logger().warn("cmd_pose timed out -- holding")
            target = list(self.target) if streaming else None

        if self.dry_run:
            return
        if not streaming:
            self._joint_plan_tick()
            self._settle_tick()
            self._path_tick()
            self._wind_down()
            return
        try:
            if STREAM_MODE == "joint_ik":
                self._stream_joint_ik(target)
            elif STREAM_MODE == "velocity" and not self.pos_fallback:
                self._stream_velocity(target)
            else:
                self._stream_position(target)
            self.stop_ticks = STOP_TICKS
        except Exception as e:
            # "This is a singularity where velocity control in cartesian space
            # is unsafe" -- the controller's words. Its POSITION-space Cartesian
            # path has no such refusal, so near a singularity the stream drops
            # to position mode for the rest of this enable instead of tripping
            # the fault-loop breaker and disarming. Slightly less smooth; still
            # moving. Cleared on the next enable, so velocity mode is retried
            # from a fresh (hopefully non-singular) posture.
            if (STREAM_MODE == "velocity" and not self.pos_fallback
                    and "singularity" in str(e).lower()):
                self.get_logger().warn(
                    "near a singularity: velocity-mode Cartesian refused -- "
                    "falling back to position-mode streaming until re-enable. "
                    "Bend the wrist (joint 5 in joint mode) to get clear of it.")
                with self.lock:
                    self.pos_fallback = True
                    self.mode_applied = None
                try:
                    self.driver.clear_error()
                    arm.apply_limit_overrides(self.driver)
                except Exception as e2:
                    return self._lost("_control_tick", e2)
                return
            # The controller refuses anything it cannot follow -- a target too
            # near a singularity, a joint limit -- and LATCHES the error with
            # every joint dropped to idle, gripper included. _lost() tells a
            # rejected command from a genuinely dead link: it clears the error
            # and stays ARMED in the recoverable case, so one bad target costs
            # one target rather than a disable/re-enable round trip. (The old
            # handler here disabled outright and left grip_applied standing, so
            # after any refusal the next gripper press skipped re-entering
            # effort mode and squeezed an idle gripper -- the "gripper only
            # works after I cycle enable" bug.)
            return self._lost("_control_tick", e)

    def _settle_tick(self):
        """Fold the measured residual of a finished move into a correction."""
        with self.lock:
            st = self.settle
            if st is None or not self.enabled or self.joint_plan is not None:
                if not self.enabled:
                    self.settle = None
                return
            target, left, why, check_at = st
            now = self.get_clock().now().nanoseconds * 1e-9
            if now < check_at:
                return
            self.settle = None
        try:
            cur = list(self.driver.get_all_positions())[:cfg.NUM_ARM_JOINTS]
            err = [t - c for t, c in zip(target, cur)]
            worst = max(range(len(err)), key=lambda i: abs(err[i]))
            if abs(err[worst]) <= POSE_SETTLE_TOL:
                self.get_logger().info(
                    f"{why}: reached (worst {cfg.label(worst)} "
                    f"{math.degrees(err[worst]):+.1f} deg)")
                return
            if left <= 0:
                self.get_logger().warn(
                    f"{why}: settled with {cfg.label(worst)} still "
                    f"{math.degrees(err[worst]):+.1f} deg off after "
                    f"{POSE_SETTLE_PASSES} corrections")
                return
            # Command past the target by the residual; the loop's proportional
            # shortfall then lands on the target itself.
            # Accumulate: each pass adds a damped share of the NEW residual to
            # the last commanded goal, so corrections build instead of restart.
            with self.lock:
                base = self.stage_goal if self.stage_goal is not None else list(target)
            goal = [b + POSE_SETTLE_GAIN * e for b, e in zip(base[:len(err)], err)]
            self.get_logger().info(
                f"{why}: settle pass {POSE_SETTLE_PASSES - left + 1} -- "
                f"{cfg.label(worst)} {math.degrees(err[worst]):+.1f} deg off, correcting")
            self._ensure_arm_mode("position", trossen_arm.Mode.position)
            self.mode_applied = "position"
            self.driver.set_arm_positions([float(v) for v in goal], POSE_SETTLE_S, False)
            with self.lock:
                self.stage_goal = list(goal)
                self.pose_move_until = now + POSE_SETTLE_S + 1.0
                self.settle = (target, left - 1, why, now + POSE_SETTLE_S + 0.4)
        except Exception as e:
            return self._lost("_settle_tick", e)

    def _path_tick(self):
        """Send the next waypoint of a named path when the last hop is done."""
        with self.lock:
            if (not self.path_queue or not self.enabled or self.joint_plan is not None
                    or self.settle is not None):
                if not self.enabled:
                    self.path_queue = []
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if now < self.path_next_at:
                return
            values, label = self.path_queue.pop(0)
        self._command_joints(values, why=f"pose {label!r}", stage=False)

    def _stall_guard(self, pos, eff, vel):
        """Abandon a named move that is pushing on something. See STALL_NM."""
        with self.lock:
            moving = (self.mode_applied == "position" and (
                self.joint_plan is not None or self.pose_move_until > 0)
                or (self.mode_applied == "joint_ik" and self.target is not None))
        if not moving:
            self.stall_since = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stuck = [i for i in range(cfg.NUM_ARM_JOINTS)
                 if abs(eff[i]) > STALL_NM and abs(vel[i]) < STALL_VEL]
        if not stuck:
            self.stall_since = None
            return
        if self.stall_since is None:
            self.stall_since = now
            return
        if now - self.stall_since < STALL_S:
            return
        self.stall_since = None
        with self.lock:
            self.joint_plan = None
            self.path_queue = []
            self.settle = None
            self.pose_move_until = 0.0
            self.target = None          # a streaming jog into a wall stops too
            self.q_cmd = None
        names = ", ".join(f"{cfg.label(i)} {eff[i]:+.0f} N*m" for i in stuck)
        try:
            # Hold HERE: the position error goes to zero and with it the effort
            # that was being spent on the obstruction.
            self.driver.set_arm_positions(
                [float(v) for v in pos[:cfg.NUM_ARM_JOINTS]], 0.5, False)
        except Exception as e:
            return self._lost("_stall_guard", e)
        self.get_logger().error(
            f"STALLED -- move abandoned: {names} with no motion for "
            f"{STALL_S:.0f} s. Gravity here needs ~7 N*m at most, so this is "
            "an obstruction. Jog in joint mode (TAB) to find which joint "
            "clears it, then set ARM_POSE_ORDER accordingly.")

    def _joint_plan_tick(self):
        """Send the next stage of a staged named move once the last has had its time."""
        with self.lock:
            plan = self.joint_plan
            if plan is None or not self.enabled:
                self.joint_plan = None
                return
            target, n, why, send_after, stages, k = plan
            now = self.get_clock().now().nanoseconds * 1e-9
            if now < send_after:
                return
            self.joint_plan = None
        try:
            current = list(self.driver.get_all_positions())
            with self.lock:
                base = self.stage_goal if self.stage_goal is not None else list(current)
            goal = list(base)
            for j in stages[k]:
                goal[j] = target[j]
            with self.lock:
                self.stage_goal = list(goal)
            goal_time = cfg.goal_time_for([goal[j] - current[j] for j in stages[k]])
            self.get_logger().info(
                f"{why}: stage {k + 1}/{len(stages)} -- "
                f"{'+'.join(cfg.label(j) for j in stages[k])} over {goal_time:.1f} s")
            self._ensure_arm_mode("position", trossen_arm.Mode.position)
            self.mode_applied = "position"
            self.driver.set_arm_positions(
                [float(v) for v in goal[:cfg.NUM_ARM_JOINTS]], goal_time, False)
            with self.lock:
                self.pose_move_until = now + goal_time + 1.0
                if k + 1 >= len(stages):
                    self.settle = (list(target[:cfg.NUM_ARM_JOINTS]), POSE_SETTLE_PASSES,
                                   why, now + goal_time + POSE_STAGE_SETTLE_S)
            if k + 1 < len(stages):
                with self.lock:
                    self.joint_plan = (target, n, why,
                                       now + goal_time + POSE_STAGE_SETTLE_S,
                                       stages, k + 1)
        except Exception as e:
            return self._lost("_joint_plan_tick", e)

    def _stream_position(self, target):
        """The time-based path: a position goal 1.5 control periods out."""
        if HOLD_ORIENTATION and self.cart is not None:
            # Same intent as the velocity path: keep the attitude the arm has
            # rather than the one the commander asked for.
            target = list(target[:3]) + list(self.cart[3:])
        if self.mode_applied != "position":
            # Entered lazily, on the first real command rather than on
            # enable, so arming an arm never starts a servo by itself.
            self._ensure_arm_mode("position", trossen_arm.Mode.position)
            self.mode_applied = "position"
            self.get_logger().info("position control engaged")
        self.driver.set_cartesian_positions(
            target, trossen_arm.InterpolationSpace.cartesian,
            GOAL_TIME_S, False)

    def _stream_velocity(self, target):
        """The velocity path: close the pose error at a capped speed.

        A proportional law -- velocity = VEL_KP x error, clamped -- rather than
        the SDK's own position interpolation, because the clamp is the feature:
        however far the target is, the arm moves toward it at no more than
        MAX_LIN_VEL / MAX_ANG_VEL, decelerating smoothly (the error shrinks, so
        the command does) as it arrives. Speed is a stated number instead of a
        side effect of how far apart two stream frames happened to land.
        """
        if self.mode_applied != "velocity":
            self._ensure_arm_mode("velocity", trossen_arm.Mode.velocity)
            self.mode_applied = "velocity"
            self.get_logger().info(
                f"velocity control engaged (caps {MAX_LIN_VEL:.2f} m/s, "
                f"{MAX_ANG_VEL:.2f} rad/s)")

        cur = list(self.driver.get_cartesian_positions())
        with self.lock:
            self.cart = list(cur)

        # Clamp the error the velocity law acts on. Bounded chase, unbounded
        # goal: self.target is left exactly as the operator sent it, so the
        # next command is judged against reality rather than against a target
        # this loop has been quietly editing. See MAX_LEAD_M.
        err_p = [t - c for t, c in zip(target[:3], cur[:3])]
        n = math.sqrt(sum(v * v for v in err_p))
        if n > MAX_LEAD_M:
            err_p = [v * MAX_LEAD_M / n for v in err_p]

        # The rotation still to make, as seen from the base frame -- which is
        # the frame the SDK measures angular velocity in.
        q_cur = angle_axis_to_quat(*cur[3:])
        q_err = quat_mul(angle_axis_to_quat(*target[3:]), quat_conj(q_cur))
        e_rot = quat_to_angle_axis(*q_err)
        n = math.sqrt(sum(v * v for v in e_rot))
        if n > MAX_LEAD_RAD:
            e_rot = [v * MAX_LEAD_RAD / n for v in e_rot]

        v_lin = [e * VEL_KP for e in err_p]
        n = math.sqrt(sum(v * v for v in v_lin))
        if n > MAX_LIN_VEL:
            v_lin = [v * MAX_LIN_VEL / n for v in v_lin]

        lin_err = math.sqrt(sum(v * v for v in err_p))
        ang_err = 0.0 if HOLD_ORIENTATION else math.sqrt(sum(v * v for v in e_rot))
        if lin_err < SETTLE_M and ang_err < SETTLE_RAD:
            if not self.settled:
                # Once, on arrival: stop, then go quiet.
                self.driver.set_cartesian_velocities(
                    [0.0] * 6, trossen_arm.InterpolationSpace.cartesian,
                    GOAL_TIME_S, False)
                self.settled = True
            return
        self.settled = False

        if HOLD_ORIENTATION:
            # Command zero angular velocity rather than simply ignoring the
            # target's orientation: the controller then actively holds the
            # current attitude through the translation instead of letting the
            # wrist drift wherever the IK finds convenient.
            v_ang = [0.0, 0.0, 0.0]
        else:
            v_ang = [e * VEL_KP for e in e_rot]
            n = math.sqrt(sum(v * v for v in v_ang))
            if n > MAX_ANG_VEL:
                v_ang = [v * MAX_ANG_VEL / n for v in v_ang]

        self.driver.set_cartesian_velocities(
            v_lin + v_ang, trossen_arm.InterpolationSpace.cartesian,
            GOAL_TIME_S, False)

    def _cart_velocity_cmd(self, target, cur, servo_orientation=False):
        """The capped, dead-banded Cartesian velocity toward target (base frame).

        Shared by the velocity-mode and joint_ik streams. Returns None when
        settled (nothing to send).
        """
        hold = HOLD_ORIENTATION and not servo_orientation
        err_p = [t - c for t, c in zip(target[:3], cur[:3])]
        n = math.sqrt(sum(v * v for v in err_p))
        if n > MAX_LEAD_M:
            err_p = [v * MAX_LEAD_M / n for v in err_p]
        q_cur = angle_axis_to_quat(*cur[3:])
        e_rot = quat_to_angle_axis(*quat_mul(angle_axis_to_quat(*target[3:]), quat_conj(q_cur)))
        n = math.sqrt(sum(v * v for v in e_rot))
        if n > MAX_LEAD_RAD:
            e_rot = [v * MAX_LEAD_RAD / n for v in e_rot]
        lin_err = math.sqrt(sum(v * v for v in err_p))
        ang_err = 0.0 if hold else math.sqrt(sum(v * v for v in e_rot))
        if lin_err < SETTLE_M and ang_err < SETTLE_RAD:
            return None
        v_lin = [e * VEL_KP for e in err_p]
        n = math.sqrt(sum(v * v for v in v_lin))
        if n > MAX_LIN_VEL:
            v_lin = [v * MAX_LIN_VEL / n for v in v_lin]
        if hold:
            v_ang = [0.0, 0.0, 0.0]
        else:
            v_ang = [e * VEL_KP for e in e_rot]
            n = math.sqrt(sum(v * v for v in v_ang))
            if n > MAX_ANG_VEL:
                v_ang = [v * MAX_ANG_VEL / n for v in v_ang]
        return v_lin + v_ang

    def _stream_joint_ik(self, target):
        """Our IK: Cartesian velocity -> joint velocities via damped least squares.

        The error is still measured against the controller's reported EE (so
        the point that tracks the target is its tool point), and the mapping
        to joint rates comes from the URDF Jacobian. With orientation held the
        two agree exactly: a pure translation moves every point on the tool
        the same way.
        """
        if self.mode_applied != "joint_ik":
            # POSITION mode, streamed. The solved joint rates are integrated
            # here and sent as positions 1.5 control periods out -- the middle
            # arm's proven pattern. Joint-velocity mode was tried first and
            # moved the arm the wrong way by a consistent sign; every joint-
            # position command today has agreed with the URDF, so positions it
            # is. Smoothness comes from our own 50 Hz integration, not from
            # the controller's velocity loop.
            self._ensure_arm_mode("position", trossen_arm.Mode.position)
            self.mode_applied = "joint_ik"
            self.q_cmd = None
            self.get_logger().info(
                f"joint-IK control engaged (caps {MAX_LIN_VEL:.2f} m/s, "
                f"{MAX_JOINT_VEL:.1f} rad/s per joint, damping {IK_DAMPING})")
        cur = list(self.driver.get_cartesian_positions())
        q = list(self.driver.get_arm_positions())
        with self.lock:
            self.cart = list(cur)
            self.joints_now = list(q)
        if self.q_cmd is None:
            self.q_cmd = list(q)
        # Orientation is SERVOED here, not merely un-commanded: with damping and
        # joint caps the angular rows are only approximately satisfied, and any
        # drift would otherwise accumulate. The commanded attitude is the one the
        # commander anchored at enable, so this holds it.
        v = self._cart_velocity_cmd(target, cur, servo_orientation=True)
        if v is None:
            # Settled: position mode holds the last goal by itself.
            self.settled = True
            return
        self.settled = False
        dq = self.kin.dls(q, v, damping=IK_DAMPING, max_dq=MAX_JOINT_VEL, w_ang=IK_W_ANG)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.ik_said > 1.0:
            self.ik_said = now
            self.get_logger().info(
                "ik: v=(%+.3f %+.3f %+.3f | %+.2f %+.2f %+.2f)  dq=(%s) rad/s" % (
                    *v, " ".join(f"{x:+.2f}" for x in dq)))
        dt = 1.0 / STREAM_HZ
        # Integrate from the COMMANDED configuration. It may LEAD the measured
        # one -- that lead is what makes the controller move -- but never by
        # more than IK_JOINT_LEAD, so a stalled joint cannot let the command
        # run away. (A 10 %-per-tick pull toward measured was tried first: the
        # command never got ahead of a lagging controller and the arm sat
        # still while the solver kept asking.)
        self.q_cmd = [c + float(d) * dt for c, d in zip(self.q_cmd, dq)]
        self.q_cmd = [m + max(-IK_JOINT_LEAD, min(IK_JOINT_LEAD, c - m))
                      for c, m in zip(self.q_cmd, q)]
        # Goal a few ticks out WITH the solved rates as feed-forward, so the
        # controller tracks a moving target instead of restarting a 30 ms
        # trajectory from rest every 20 ms.
        self.driver.set_arm_positions(
            [float(x) for x in self.q_cmd], IK_GOAL_TIME_S, False,
            [float(x) for x in dq])

    def _wind_down(self):
        """After a velocity stream ends: ramp to zero, then idle (braked hold).

        Position mode needs nothing here -- the controller holds its last goal.
        Velocity mode would keep executing the last commanded velocity, so the
        stream ending MUST be followed by an explicit stop, and going straight
        to idle from speed is a brake snap. So: one zero-velocity command with
        a ramp the length of the wind-down window, then idle once it has had
        time to land.
        """
        if self.mode_applied not in ("velocity", "joint_ik") or self.stop_ticks <= 0:
            return
        try:
            if self.stop_ticks == STOP_TICKS:
                if self.mode_applied == "joint_ik":
                    pass        # position mode: the last goal is already a hold
                else:
                    self.driver.set_cartesian_velocities(
                        [0.0] * 6, trossen_arm.InterpolationSpace.cartesian,
                        STOP_TICKS / STREAM_HZ, False)
            self.stop_ticks -= 1
            if self.stop_ticks == 0:
                if self.mode_applied == "joint_ik":
                    # Position mode is already a hold, and a real one: idle on
                    # this mount lets a folded elbow sag (measured: a jog that
                    # converged to +1 cm sagged back to zero after idling).
                    self.get_logger().info("stream ended -- holding in position mode")
                else:
                    self._ensure_arm_mode("idle", trossen_arm.Mode.idle)
                    self.get_logger().info("stream ended -- idle, braked hold")
                with self.lock:
                    if self.mode_applied != "joint_ik":
                        self.mode_applied = None
                    self.settled = False
        except Exception as e:
            return self._lost("_wind_down", e)

    def _state_tick(self):
        try:
            pos = list(self.driver.get_all_positions())
            cart = list(self.driver.get_cartesian_positions())
            rotor = list(self.driver.get_all_rotor_temperatures())
            drv = list(self.driver.get_all_driver_temperatures())
            eff = list(self.driver.get_all_efforts())
            comp = list(self.driver.get_all_compensation_efforts())
            ext = list(self.driver.get_all_external_efforts())
            vel = list(self.driver.get_all_velocities())
        except Exception as e:
            return self._lost("_state_tick", e)
        self._stall_guard(pos, eff, vel)
        temps = [max(r, d) for r, d in zip(rotor, drv)]
        with self.lock:
            self.cart = list(cart)
            self.joints_now = list(pos)
            self.temps = temps
        hottest = max(range(len(temps)), key=lambda i: temps[i])
        if temps[hottest] >= TEMP_WARN_C:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.temp_said > 5.0:
                self.temp_said = now
                # Effort alongside temperature: a joint that is hot AND at a
                # steady high effort is stalled against a load it cannot move
                # -- gravity it is not compensating for, or an obstruction --
                # and no amount of retrying the move will help.
                self.get_logger().warn(
                    f"HOT: {cfg.label(hottest)} at {temps[hottest]:.0f} C, "
                    f"effort {eff[hottest]:+.1f} Nm (controller idles at 95 C). "
                    "All: " + " ".join(f"{cfg.label(i)}={t:.0f}C/{e:+.0f}Nm"
                                       for i, (t, e) in enumerate(zip(temps, eff))))

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
        # The standard field, used for what it is for: the effort each joint
        # is actually applying, N*m (gripper: N). A joint at high effort that
        # is not moving is stalled, and that is visible from any subscriber.
        js.effort = [float(v) for v in eff]
        self.pub_js.publish(js)

        # Once a second, what the CONTROLLER believes about the load. comp is
        # its gravity/friction feedforward for this pose -- computed for an
        # upright base, which this arm is not on. When comp says a few N*m and
        # the joint is pulling 20+, the model is wrong for the mount and the
        # position loop is supplying gravity as error, up to the 27 N*m cap.
        # ext is what it attributes to the outside world: a joint pressing on
        # an obstruction shows here.
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.health_said >= 1.0:
            self.health_said = now
            h = String()
            h.data = json.dumps({
                "temp_c": [round(t, 1) for t in temps],
                "effort_nm": [round(v, 2) for v in eff],
                "compensation_nm": [round(v, 2) for v in comp],
                "external_nm": [round(v, 2) for v in ext],
                "joints": cfg.DISPLAY_NAMES,
            })
            self.pub_health.publish(h)

        ee = PoseStamped()
        ee.header.stamp = stamp
        ee.header.frame_id = self.base_frame
        wpos = arm_to_world(cart[:3])
        if self.origin is not None:
            wpos = [wpos[i] - self.origin[i] for i in range(3)]
        ee.pose.position.x, ee.pose.position.y, ee.pose.position.z = wpos
        # Composition, mirroring _on_pose -- see the MOUNT_Q comment up top.
        qx, qy, qz, qw = quat_mul(MOUNT_Q_INV, angle_axis_to_quat(*cart[3:]))
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
