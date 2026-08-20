"""Model, joints, limits, and named poses for the WXAI manipulator arms.

The middle arm's arm_config.py describes a DYNAMIXEL arm driven through xs_sdk
over ROS 2 topics. None of that applies here, and the difference is not cosmetic:

  * No ROS. The WXAI arms are reached by the `trossen_arm` SDK over Ethernet,
    talking to a controller box that runs its own firmware. There is no
    xs_sdk, no /joint_states, no JointGroupCommand, and no interbotix_xs_msgs.
  * No joint names on the wire. The SDK is index-based: joints are 0-indexed
    and the gripper is always last. The names below are labels for the CLI, and
    they match trossen_arm_description's URDF (joint_0 .. joint_5), so a name
    printed here means the same thing in RViz or MoveIt.
  * The gripper is a linear joint in METRES, not a radian-valued DYNAMIXEL.
    0.0 is closed, ~0.04 open. Effort is in newtons, not a raw current-limit
    register value.

The one thing that carries over unchanged is the shape of the workflow: named
poses in a YAML file, dry-run-by-default CLIs, limits checked before anything
is sent.

Which arm a script talks to comes from ARM_IP, set per-container in
docker-compose.yml. Every script also takes --ip to override it.
"""
import os

import trossen_arm

# Per-container, from docker-compose.yml. ARM_NAME only namespaces saved poses
# (config/poses.yaml is bind-mounted into BOTH containers, so arm-1's "pick"
# and arm-2's "pick" are different points in space and must not collide).
ARM_IP = os.environ.get("ARM_IP", "192.168.1.2")
ARM_MODEL = os.environ.get("ARM_MODEL", "wxai_v0")
ARM_NAME = os.environ.get("ARM_NAME") or f"arm@{ARM_IP}"

# The end effector argument to configure() is NOT cosmetic: it feeds the
# gravity/friction compensation model. A wrong value here means the arm sags or
# fights itself in external_effort mode, which is exactly the mode teach.py and
# the force-based gripper commands rely on. Standard variants are base (bare
# flange), leader (trigger handle), follower (gripper).
ARM_EE = os.environ.get("ARM_EE", "wxai_v0_base")

# joint_0 .. joint_5 are the URDF names. The parenthesised aliases are this
# repo's shorthand -- accepted anywhere a joint is named, purely for legibility.
JOINT_NAMES = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"]
JOINT_ALIASES = {
    "base": 0,
    "shoulder": 1,
    "elbow": 2,
    "forearm_roll": 3,
    "wrist_angle": 4,
    "wrist_rotate": 5,
    "gripper": 6,
}

NUM_ARM_JOINTS = 6
GRIPPER_INDEX = 6

# What human-facing output prints. JOINT_NAMES stays the URDF mapping, because
# that correspondence is the reason it exists -- a joint named there means the
# same joint in RViz, MoveIt and trossen_arm_description -- but "joint_3" is
# not what anyone calls the forearm roll while standing at the arm, and reading
# a fault about joint_2 means translating before you can look at the right part
# of the robot. Derived from JOINT_ALIASES rather than written out again, so
# there is one list to keep in step with the hardware, not two.
DISPLAY_NAMES = [n for n, _ in sorted(JOINT_ALIASES.items(), key=lambda kv: kv[1])]


def label(index):
    """Human-facing name for a joint index: the alias, not the URDF name."""
    return DISPLAY_NAMES[index]

# Vendor defaults, from the SDK's default_configurations_wxai_v0.yaml. The live
# values are read from the arm with get_joint_limits() whenever a driver is
# connected; these exist so a dry run can still be checked when it is not, and
# so a divergence between the two is visible rather than silent.
DEFAULT_JOINT_LIMITS = [
    (-3.141593, 3.141593),
    (0.0, 3.141593),
    (0.0, 2.356194),
    (-1.570796, 1.570796),
    (-1.570796, 1.570796),
    (-3.141593, 3.141593),
    (0.0, 0.04),          # gripper, metres
]

# Per-joint (position_min, position_max) this rig needs instead of the vendor
# defaults above. Applied by arm.connect() on EVERY connection, because the
# controller does not persist joint limits -- each configure() resets them to
# the defaults, so a range the rig genuinely needs has to be re-asserted per
# session rather than written once.
#
# joint_1 and joint_2: the vendor ranges for these two are entirely
# non-negative ([0, 3.142] and [0, 2.356]), and this rig works entirely on the
# negative side of both. Two measured poses:
#
#   home/rest   j1 = +0.0006   j2 = -1.5520
#   working     j1 = -2.8193   j2 = -2.6553
#
# Both are real -- forward kinematics from this arm's own saved URDF puts the
# working pose 0.52 m forward and 0.37 m up with every link above the base
# plane, which is a sane posture, and no sign flip or position_offset
# correction brings both poses inside the stock ranges. The stock limits simply
# do not describe how this arm is mounted and used.
#
# Because the controller faults on REPORTED position the moment any joint
# leaves idle, the arm cannot be commanded at all until the range covers where
# it already sits.
#
# The floors are -pi rather than "furthest seen so far, plus a margin". The
# first version of this table used the latter, and it was wrong twice over.
# It was wrong in practice: it was measured off arm-1, and arm-2 promptly
# parked its elbow at -2.9734, which the -2.90 floor then refused -- a pose the
# hardware reached perfectly happily. And it was wrong in principle: a bound
# derived from wherever the arm has been driven so far is not a safety limit,
# it is a record of the past, and it re-blocks legitimate poses every time the
# workspace grows. -pi is at least a real statement -- half a turn negative,
# the same magnitude the vendor already allows joint_0 and joint_5 -- and it
# stops the floor from being a thing to keep bumping.
#
# Be clear about what that costs: these two joints now have effectively no
# software lower bound. What protects them is the mechanical stops and the
# 27 Nm effort limits, nothing here. The upper bounds are left at stock
# because nothing has needed them moved.
#
# If the two arms ever need genuinely DIFFERENT envelopes -- a different mount,
# a different reachable set -- this wants to become a per-arm table keyed by IP
# (connect() always knows the IP; ARM_NAME is per-container and wrong for the
# second arm in both.py). It is one table today because both arms want the
# same numbers today.
#
# gripper: reads about -0.0017 m at rest, a hair under the stock 0.0 floor.
# Inside the controller's 0.004 feedback tolerance, so it never faults, but
# arm.check() refuses any target below 0 -- which blocks `pose.py go <name>
# --with-gripper` for poses saved here. A -0.005 floor covers the resting
# value without opening up meaningful new travel.
#
# NOTHING HERE IS A FIX FOR A BAD READING. Widening a limit is right when the
# joint really is parked where it says it is. If joint_2 reports -1.552 while
# the arm sits in the FOLDED shape (which is what all-zeros means on this arm
# -- links 2 and 3 are 0.264 and 0.245 and cancel at zero), then the reading is
# ~-pi/2 off and the honest repair is that joint's position_offset, not this
# table: widening here would let the controller drive a joint whose commanded
# angles are all 90 deg from reality.
JOINT_LIMIT_OVERRIDES = {
    1: (-3.141593, 3.141593),
    2: (-3.141593, 2.356194),
    GRIPPER_INDEX: (-0.005, 0.04),
}

# No HOME constant here on purpose. This rig's home is a property of how the
# arm is mounted, not of the model, so it belongs in config/poses.yaml keyed by
# ARM_NAME (`pose.py save home`) alongside the other per-arm poses -- the same
# split POSES above already draws. A second copy hardcoded here would be one
# more thing to keep in sync with the arm, and it would lose silently.

# Joint characteristics from the vendor's default_configurations_wxai_v0.yaml,
# for telling a calibrated controller from a reset one.
#
# position_offset is 0 for every joint in that file, so all-zero offsets on a
# real arm are NORMAL -- they correct homing error, and an arm that homes true
# needs none. What actually distinguishes a calibrated unit is that its effort
# corrections and friction terms DIFFER from these: those are measured per arm
# at manufacturing. An arm reading back exactly the values below is one whose
# EEPROM has been reset to generic defaults.
DEFAULT_EFFORT_CORRECTIONS = [1.10, 1.10, 1.10, 1.25, 1.15, 1.15, 1.15]
DEFAULT_FRICTION_CONSTANTS = [0.24, 0.08, 0.16, -0.01, 0.04, 0.06, 7.0]

# Gripper travel. 0.04 is the position limit the controller enforces; the
# mechanism physically reaches ~0.044, which is why the URDF and the position
# tolerance both mention that number. Commanding 0.044 trips a limit error, so
# OPEN stops at the enforced limit.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 0.04

# Newtons, applied by set_gripper_external_effort. Positive opens, negative
# closes. This is the replacement for the ROS 1 Current_Limit register trick:
# instead of putting a DYNAMIXEL into current_based_position mode and capping
# its current, you command a force directly and the finger stops where the
# object is. 20 N is the SDK demo's gentle default; 100 N is a firm grasp.
GRASP_FORCE_N = 20.0
GRASP_FORCE_MAX_N = 100.0

# Poses, in radians, arm joints only (no gripper). These three are the named
# states from trossen_arm_moveit's wxai.srdf.xacro, i.e. the vendor's own
# definitions -- not values guessed here.
SLEEP = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
UPRIGHT = [0.0, 1.5708, 1.5708, 0.0, 0.0, 0.0]
READY = [0.0, 1.04719755, 0.523598776, 0.628318531, 0.0, 0.0]

POSES = {
    "sleep": SLEEP,
    "upright": UPRIGHT,
    "ready": READY,
}

# Motion pacing. A goal_time is picked from the distance to travel rather than
# fixed, because the SDK's own guard trips on discontinuity: a large step with a
# short goal_time is a command the arm cannot follow, and it errors out and goes
# idle mid-motion. Slow is the safe direction to be wrong in.
DEFAULT_SPEED_RAD_S = 0.6
MIN_GOAL_TIME_S = 2.0


def model_enum(name=None):
    """trossen_arm.Model for a model string like 'wxai_v0'."""
    name = name or ARM_MODEL
    try:
        return getattr(trossen_arm.Model, name)
    except AttributeError:
        known = [n for n in dir(trossen_arm.Model) if not n.startswith("_")
                 and n not in ("name", "value")]
        raise SystemExit(f"unknown ARM_MODEL {name!r}; known: {', '.join(known)}")


def end_effector(name=None):
    """trossen_arm.StandardEndEffector entry for a variant string."""
    name = name or ARM_EE
    try:
        return getattr(trossen_arm.StandardEndEffector, name)
    except AttributeError:
        known = [n for n in dir(trossen_arm.StandardEndEffector) if not n.startswith("_")]
        raise SystemExit(f"unknown ARM_EE {name!r}; known: {', '.join(known)}")


def joint_index(token):
    """Resolve '3', 'joint_3', or 'forearm_roll' to an index. None if unknown."""
    token = str(token).strip()
    if token.isdigit():
        i = int(token)
        return i if 0 <= i < len(JOINT_NAMES) else None
    if token in JOINT_NAMES:
        return JOINT_NAMES.index(token)
    return JOINT_ALIASES.get(token)


def goal_time_for(deltas, speed=DEFAULT_SPEED_RAD_S, minimum=MIN_GOAL_TIME_S):
    """Seconds to allow for a move, from its largest joint delta."""
    return max(minimum, max((abs(d) for d in deltas), default=0.0) / speed)
