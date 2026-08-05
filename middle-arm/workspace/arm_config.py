"""Robot definition, joint names, and predefined poses -- ROS 2 port for THIS arm.

Ported from the stationary-ALOHA ROS 1 arm_config.py. Scoped to the middle arm
only; the left/right vx300s entries are gone because this rig has one arm on
this ROS graph.

Three substantive differences from the original, all forced by the hardware:

1. SIX joints, not seven. `ARM_CONFIG["middle"]` upstream declares
   `num_joints: 7` with 7-element M_* poses. This arm has 8 DYNAMIXELs -- IDs 3
   and 5 are shadow motors slaved to 2 and 4 -- which the driver resolves to 6
   logical joints. Any 7-wide pose or recorded trajectory from the old rig will
   not line up here.

2. The last joint is the CAMERA PAN. ID 9 (gripper) was removed and a camera
   mounted in its place; `wrist_rotate` (ID 8) rotates that camera about Z. The
   original's commented-out joint list called it `camera_roll` -- same joint.
   CAMERA_PAN_JOINT below is the canonical alias, so downstream code can say
   what it means instead of hardcoding "wrist_rotate".

3. REST is different, because the original is UNREACHABLE on a wx250s. Upstream
   REST = [0.0, -1.9, 1.635, 0.0, 0.7, 0.0], but this arm's limits are
   shoulder >= -1.885 and elbow <= 1.6057, so both of those overshoot the
   mechanical stop. Those poses were authored for the vx300s left/right arms and
   reused for the middle arm, which has tighter shoulder and elbow travel. The
   observable symptom is an arm parked against both stops drawing constant
   holding effort. See REST below for the replacement.
"""
import numpy as np

# Namespace of the running driver: `robot_name:=middle` in launch-arm.sh, so all
# topics live under /middle/... . The old rig used "puppet_middle".
ROBOT_NAME = "middle"
ROBOT_MODEL = "wx250s"

# Order matches `joint_order` in config/wx250s_nogripper.yaml, which is the
# order xs_sdk publishes and expects in a JointGroupCommand.
JOINT_NAMES = [
    "waist",
    "shoulder",
    "elbow",
    "forearm_roll",
    "wrist_angle",
    "wrist_rotate",   # == camera pan, see CAMERA_PAN_JOINT
]

# The camera's aiming DOF. Named separately so intent survives in calling code.
CAMERA_PAN_JOINT = "wrist_rotate"
CAMERA_PAN_INDEX = JOINT_NAMES.index(CAMERA_PAN_JOINT)

ARM_CONFIG = {
    "middle": {
        "robot_name": ROBOT_NAME,
        "robot_model": ROBOT_MODEL,
        "has_gripper": False,
        "num_joints": 6,
        "joint_names": JOINT_NAMES,
    },
}

# Read from the live robot_description published by this arm's
# robot_state_publisher. Duplicated here so pose validation works without a
# running driver; robot_control.validate_pose() prefers the live values when a
# node is available, and these are the fallback.
JOINT_LIMITS = {
    "waist":        (-3.141583,  3.141583),
    "shoulder":     (-1.884956,  1.989675),
    "elbow":        (-2.146755,  1.605703),
    "forearm_roll": (-3.141583,  3.141583),
    "wrist_angle":  (-1.745329,  2.146755),
    "wrist_rotate": (-3.141583,  3.141583),
}

# ---------------------------------------------------------------------------
# Poses. Radians, in JOINT_NAMES order.
#
# HIGH / FORWARD / LOW carry over from the original unchanged -- all three are
# inside this arm's limits, verified against JOINT_LIMITS above.
# ---------------------------------------------------------------------------
HIGH = np.array([0.11, -0.48, 0.33, -0.03, 1.35, 0.05], dtype=float)
LOW = np.array([0.02, 0.037, 0.598, -0.143, 0.986, 0.038], dtype=float)
FORWARD = np.array([0.0, -1.27, 0.99, 0.0, 0.35, 0.0], dtype=float)

# Replaces the upstream REST = [0.0, -1.9, 1.635, 0.0, 0.7, 0.0], which asks for
# shoulder -1.900 (limit -1.885) and elbow 1.635 (limit 1.606) -- both past the
# stop. These values are `sleep_positions` from wx250s_nogripper.yaml, i.e. the
# vendor's own folded pose for this exact arm, and sit comfortably in range.
REST = np.array([0.0, -1.80, 1.55, 0.0, 0.8, 0.0], dtype=float)

DEFAULT_RESET_POSE = "forward"

POSES = {
    "middle": {
        "high": HIGH,
        "forward": FORWARD,
        "rest": REST,
        "low": LOW,
    },
}


def get_pose(pose_name, arm_name="middle"):
    """Return a named pose. Raises ValueError if undefined."""
    if pose_name not in POSES[arm_name]:
        raise ValueError(
            f"Pose {pose_name!r} not defined for arm {arm_name!r}. "
            f"Known: {', '.join(sorted(POSES[arm_name]))}"
        )
    return POSES[arm_name][pose_name]
