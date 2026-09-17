"""Teleop mapping, lifted from giava@real-v2-spr26 data_col_config.py.

WHAT THIS IS AND WHERE IT CAME FROM
-----------------------------------
The controller-to-arm mapping from the GIAVA rig, extracted verbatim apart from
the changes noted below. These constants -- position_scale, alpha, max_ee_step,
the remap matrices -- are tuned on real hardware over real sessions, which is
exactly why they are worth porting rather than reinventing.

Upstream lives in data_col_config.py alongside dataset paths, task lists, a
hardcoded sys.path.append("/home/devi/giava/pyroki/examples"), and a pyroki
import. None of that belongs in a teleop container, so this module carries only
the mapping and its dependencies are numpy + scipy + transform_utils.

CHANGES FROM UPSTREAM
---------------------
1. start_teleop_session() took `mode` and looked arms up in ARM_MODES. It now
   takes an explicit list of arm names -- this repo namespaces arms as
   left_arm/right_arm rather than by collection mode, and importing a task
   taxonomy to enumerate two arms was the wrong dependency.
2. solve_single_arm_ik() is dropped. It wraps pyroki_snippets for the DYNAMIXEL
   arms; here the WXAI controller solves its own Cartesian IK and the middle arm
   has its own solver in ../../middle-arm/workspace/middle_ik.py.
3. Dataset/task/action-layout config dropped -- not teleoperation.
4. SESSION YAW CALIBRATION, ported from upstream's later real-v2-spr26 state.
   The app's "world" is right-handed z-up but its yaw is wherever the headset
   happened to face when the app started -- arbitrary, different every session.
   A fixed remap matrix therefore cannot be right twice in a row. At engage,
   session_yaw_remap() reads the operator's horizontal gaze direction from the
   head pose and folds it into the remap, so "the way you are facing" becomes
   the rig's +x every session.
5. ORIENTATION IS COMPOSED IN THE WORLD FRAME, for the gripper arms as well as
   the camera arm. The original gripper-arm path took the controller-LOCAL
   rotation delta and hand-shuffled its axis components ([-pitch, yaw, roll]) --
   a per-rig compensation baked in as arithmetic, and one of the reasons EE
   orientation never felt right off giava's own rig. Upstream itself moved the
   CAMERA arm to the principled form: take the world-frame delta, conjugate by
   the remap, apply. This file uses that form for every arm: turn your hand
   about the room's vertical and the EE turns about the rig's vertical.

THE CONSTANT THAT DOES NOT TRANSFER
-----------------------------------
position_scale amplifies hand motion 1.35x for GIAVA's workspace; it is per-rig
(quest_config.py exposes it as QUEST_POS_SCALE). R_arm_remap is now only the
OPERATOR-to-rig part of the mapping -- the session yaw is measured, not assumed
-- so identity is the right default when the operator faces the same way as the
rig's +x. Everything else (alpha, max_ee_step, the filter structure) is about
how a human hand moves and should transfer.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from .transform_utils import HEAD_LOCAL_FWD, quat_xyzw_to_wxyz


def clamp_joint_step(q_curr, q_target, max_step):
    dq = np.clip(q_target - q_curr, -max_step, max_step)
    return q_curr + dq

def clamp_cartesian_step(target, prev_target, max_step):
    delta = target - prev_target
    norm = np.linalg.norm(delta)
    if norm > max_step and norm > 1e-9:
        delta *= (max_step / norm)
    return prev_target + delta

@dataclass
class TeleopConfig:
    control_dt: float = 1.0 / 50.0
    position_scale: float = 1.35
    alpha: float = 0.3
    arm_cmd_dt: float = 1.0 / 50.0
    moving_time: float = 0.14
    accel_time: float = 0.04
    max_ee_step: float = 0.02
    # Camera-arm variants, from upstream: the head moves differently from a
    # hand. A soft deadband swallows postural sway without a snap at its edge,
    # the scale is gentler, and the step clamp looser (the head arm carries no
    # payload to be careful with).
    cam_position_scale: float = 0.6
    cam_deadband_m: float = 0.015
    cam_max_ee_step: float = 0.05
    # ROTATION is scaled separately from translation, and separately per arm.
    # 1.0 is "the tool turns exactly as your hand does", which is what you want
    # for a gripper -- a hand knows how far it has twisted. The CAMERA arm is
    # the opposite case: the operator turns their head only as far as is
    # comfortable, and amplifying that is how the arm looks somewhere the neck
    # will not go. >1 means the view turns further than your head did, so it
    # costs some of the one-to-one feel; this is a comfort knob, not a
    # correctness one.
    rotation_scale: float = 1.0
    cam_rotation_scale: float = 1.5
    pos_weight: float = 40.0
    ori_weight: float = 0.25
    dq_weight: float = 0.18
    joint_reached_tol: float = 0.03
    ee_reached_tol: float = 0.01
    cmd_timeout: float = 0.25
    full_joint_velocity_limits_value: float = 2.3

    # Per-instance NumPy arrays via default_factory
    max_joint_step: np.ndarray = field(
        default_factory=lambda: np.array(
            [0.05, 0.05, 0.06, 0.08, 0.08, 0.10],
            dtype=float,
        )
    )
    R_arm_remap: np.ndarray = field(
        default_factory=lambda: np.array(
            [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],
            dtype=float,
        )
    )
    R_cam_remap: np.ndarray = field(
        default_factory=lambda: np.array(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            dtype=float,
        )
    )

# State for each arm during teleoperation, tracking the initial controller and robot poses, as well as a filtered target position for smooth motion.
@dataclass
class ArmTeleopState:
    active: bool = False
    start_controller_pos: Optional[np.ndarray] = None
    start_controller_rot: Optional[np.ndarray] = None
    start_robot_pos: Optional[np.ndarray] = None
    start_robot_rot: Optional[np.ndarray] = None
    filtered_target_pos: Optional[np.ndarray] = None
    # Session-calibrated remap (base remap x measured yaw), captured at engage.
    # None falls back to the raw remap matrix the caller passes.
    remap: Optional[np.ndarray] = None

# Overall teleoperation session state, including whether it's active and the state for each arm.
@dataclass
class TeleopSessionState:
    active: bool = False
    arms: dict[str, ArmTeleopState] = field(default_factory=dict)

# State for tracking the last command times and values for each arm, used to implement command timeouts and ensure smooth control.
@dataclass
class RobotCommandState:
    last_arm_cmd_time: float = 0.0
    last_cmds: dict[str, np.ndarray] = field(default_factory=dict)

# Kinematics state for the commanded target poses of each arm, used to compute the desired end-effector positions and orientations based on the controller input and initial poses.
@dataclass
class CommandKinematicsState:
    q_cmd: Optional[np.ndarray] = None
    T_cmd: dict[str, np.ndarray] = field(default_factory=dict)

def session_yaw_remap(head_rot, base_remap):
    """Fold the operator's measured facing into a base remap matrix.

    The app world's yaw is wherever the headset faced at app start. This takes
    the horizontal projection of the head's forward axis at ENGAGE time as the
    operator's +x, so base_remap only has to describe how the operator stands
    relative to the rig -- identity when they face the same way as rig +x.
    """
    # WHICH LOCAL AXIS THE HEAD LOOKS ALONG IS APP-SPECIFIC, and getting it
    # wrong does not look like a frame bug -- it looks like the robot is
    # rotated. This read the head's local +x, which is the WebRTC build's
    # convention; the v2 (gvlink) app looks along local +z. Ninety degrees of
    # error about the vertical: forward commands came out sideways, sideways
    # came out forward, and z -- which the two conventions share -- was
    # perfect, which is exactly what makes it read as "the axes are swapped"
    # rather than as a calibration constant. Taken from transform_utils so
    # there is one place that knows, and it is the file giava re-measured.
    fwd = np.asarray(head_rot, dtype=float)[:3, :3] @ HEAD_LOCAL_FWD
    fwd = np.array([fwd[0], fwd[1], 0.0])
    n = np.linalg.norm(fwd)
    # A head pitched straight down has no horizontal forward; fall back to the
    # app world's own +x rather than dividing by nearly zero.
    fwd = fwd / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, fwd)
    W = np.stack([fwd, left, up])          # rows: app world -> operator frame
    return np.asarray(base_remap, dtype=float) @ W


# Initializes the teleoperation session state for the active arms based on the current controller poses and commanded kinematics.
def start_teleop_session(state, arm_names, controller_poses, cmd_kin,
                         base_remaps=None, head_pose=None):
    """Anchor each named arm: remember where the controller and the arm both are.

    base_remaps maps arm name -> that arm's operator-to-rig matrix; combined
    with the session yaw measured from head_pose (falling back to the "middle"
    controller pose, which IS the head on this rig) it becomes the remap used
    for the whole session. Omit both to keep the raw fixed-matrix behaviour.
    """
    state.active = True

    if head_pose is None:
        head_pose = controller_poses.get("middle")

    for arm in arm_names:

        pose = cmd_kin.T_cmd[arm]

        quat_wxyz = pose[:4]
        pos = pose[4:]

        quat_xyzw = np.array([
            quat_wxyz[1],
            quat_wxyz[2],
            quat_wxyz[3],
            quat_wxyz[0],
        ])

        rot = R.from_quat(quat_xyzw).as_matrix()

        remap = None
        if base_remaps is not None and arm in base_remaps:
            base = np.asarray(base_remaps[arm], dtype=float)
            remap = (session_yaw_remap(head_pose[:3, :3], base)
                     if head_pose is not None else base)

        state.arms[arm] = ArmTeleopState(
            start_controller_pos=controller_poses[arm][:3, 3].copy(),
            start_controller_rot=controller_poses[arm][:3, :3].copy(),

            start_robot_pos=np.asarray(pos).copy(),
            start_robot_rot=rot.copy(),

            filtered_target_pos=np.asarray(pos).copy(),
            remap=remap,
        )

def _target_orientation(arm_state, controller_pose, remap, scale=1.0):
    """World-frame relative rotation, remapped into the rig's frame.

    R_delta is the rotation the controller has made since engage, seen from the
    app world; conjugating by the remap re-expresses that same physical
    rotation in the rig's world; left-composing applies it to where the EE was
    anchored. Net effect: turn your hand about any room axis and the EE turns
    about the corresponding rig axis, wherever either of them is pointing.
    """
    R_delta_world = controller_pose[:3, :3] @ arm_state.start_controller_rot.T
    R_delta_robot = remap @ R_delta_world @ remap.T
    if scale != 1.0:
        # Scale the ANGLE about the same axis: as a rotation vector, magnitude
        # is the angle and direction is the axis, so multiplying scales the
        # turn and leaves the axis exactly where the operator put it.
        rotvec = R.from_matrix(R_delta_robot).as_rotvec() * scale
        R_delta_robot = R.from_rotvec(rotvec).as_matrix()
    return R_delta_robot @ arm_state.start_robot_rot


def _remap_for(arm_state, remap_matrix):
    return (arm_state.remap if arm_state.remap is not None
            else np.asarray(remap_matrix, dtype=float))


# Computes the target end-effector position and orientation for a given arm based on the current controller pose, the initial poses, and the configuration parameters.
def compute_gripper_arm_target(cfg, arm_state, controller_pose, current_pose, remap_matrix):
    remap = _remap_for(arm_state, remap_matrix)

    delta_ctrl = controller_pose[:3, 3] - arm_state.start_controller_pos
    delta_robot = remap @ delta_ctrl
    raw_target = arm_state.start_robot_pos + cfg.position_scale * delta_robot
    arm_state.filtered_target_pos = cfg.alpha * raw_target + (1.0 - cfg.alpha) * arm_state.filtered_target_pos
    target_pos = clamp_cartesian_step(arm_state.filtered_target_pos, np.asarray(current_pose[4:], dtype=float), cfg.max_ee_step)

    target_rot = _target_orientation(arm_state, controller_pose, remap,
                                    cfg.rotation_scale)
    target_wxyz = quat_xyzw_to_wxyz(R.from_matrix(target_rot).as_quat())

    return target_pos, target_wxyz

def compute_camera_arm_target(cfg, arm_state, controller_pose, current_pose, remap_matrix):
    remap = _remap_for(arm_state, remap_matrix)

    delta_ctrl = controller_pose[:3, 3] - arm_state.start_controller_pos
    # Soft deadband: shrink the delta's magnitude rather than gating it, so
    # postural sway is swallowed but deliberate motion does not snap in at the
    # threshold.
    mag = np.linalg.norm(delta_ctrl)
    if mag > 1e-9:
        delta_ctrl = delta_ctrl * (max(0.0, mag - cfg.cam_deadband_m) / mag)
    delta_robot = remap @ delta_ctrl
    raw_target = arm_state.start_robot_pos + cfg.cam_position_scale * delta_robot
    arm_state.filtered_target_pos = cfg.alpha * raw_target + (1.0 - cfg.alpha) * arm_state.filtered_target_pos
    target_pos = clamp_cartesian_step(arm_state.filtered_target_pos, np.asarray(current_pose[4:], dtype=float), cfg.cam_max_ee_step)

    target_rot = _target_orientation(arm_state, controller_pose, remap,
                                    cfg.cam_rotation_scale)
    target_wxyz = quat_xyzw_to_wxyz(R.from_matrix(target_rot).as_quat())

    return target_pos, target_wxyz

def stop_teleop_session(state):
    state.active = False

