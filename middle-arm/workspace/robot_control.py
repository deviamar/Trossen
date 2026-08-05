"""Robot creation, configuration, state queries, and motion helpers -- ROS 2 port.

Ported from the stationary-ALOHA ROS 1 robot_control.py. The API is kept
deliberately close to the original so downstream code ports with minimal edits,
but four things genuinely changed in the move to ROS 2 Humble:

  rospy.init_node(...)          ->  robot_startup() / robot_shutdown()
                                    (from interbotix_common_modules)
  interbotix_xs_modules.arm     ->  interbotix_xs_modules.xs_robot.arm
  bot.dxl                       ->  bot.core
                                    (the ROS 2 API renamed the XS core handle;
                                     bot.dxl does not exist and raises
                                     AttributeError at runtime, not import time)
  rospy.sleep(x)                ->  time.sleep(x)

`init_node=False` is simply gone -- node lifetime is now handled by
robot_startup(), which creates and spins a global node. Passing init_node= to
the ROS 2 constructor is a TypeError.

Safety notes carried over and strengthened:

- Nothing here moves the arm on construction. InterbotixArmXSInterface.__init__
  reads group info and writes profile registers; it issues no motion command.
  The arm moves only when you call an interpolate_/move_/safe_move_ function.
- validate_pose() is called before every motion helper sends anything. The
  original had no such check, which is how an unreachable REST pose ended up
  parked against two mechanical stops.
"""
import time

import numpy as np

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from arm_config import (
    ARM_CONFIG,
    DEFAULT_RESET_POSE,
    JOINT_LIMITS,
    JOINT_NAMES,
    get_pose,
)

# Per-joint cap on how far a single interpolation waypoint may travel. Carried
# over from the original's REPLAY_MAX_JOINT_STEP_LR (6-wide). The 7-wide
# ..._STEP_M variant is dropped -- that was for the old rig's 7-joint middle arm.
# Ordering matches JOINT_NAMES; the last entry is the camera pan.
MAX_JOINT_STEP = np.array([0.06, 0.06, 0.08, 0.12, 0.12, 0.14], dtype=float)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_pose(pose, limits=None):
    """Return a list of human-readable problems with `pose`; empty means OK."""
    limits = limits or JOINT_LIMITS
    pose = np.asarray(pose, dtype=float)
    problems = []
    if pose.shape != (len(JOINT_NAMES),):
        return [f"pose has {pose.size} values, this arm has {len(JOINT_NAMES)} joints "
                f"({', '.join(JOINT_NAMES)})"]
    for name, value in zip(JOINT_NAMES, pose):
        lo, hi = limits.get(name, (-np.inf, np.inf))
        if not (lo <= value <= hi):
            problems.append(
                f"{name}: {value:.4f} outside [{lo:.4f}, {hi:.4f}] rad"
            )
    return problems


def require_valid(pose, what="pose"):
    problems = validate_pose(pose)
    if problems:
        raise ValueError(f"unreachable {what}:\n  " + "\n  ".join(problems))


# ---------------------------------------------------------------------------
# Robot creation
# ---------------------------------------------------------------------------
def create_robot(arm_name="middle", moving_time=0.14, accel_time=0.04):
    """Build the manipulator handle. Does NOT move the arm."""
    cfg = ARM_CONFIG[arm_name]
    return InterbotixManipulatorXS(
        robot_model=cfg["robot_model"],
        group_name="arm",
        # Must be None, not "gripper": there is no gripper motor on this arm, and
        # asking for one makes the gripper interface block waiting for an ID 9
        # that will never answer.
        gripper_name="gripper" if cfg["has_gripper"] else None,
        robot_name=cfg["robot_name"],
        moving_time=moving_time,
        accel_time=accel_time,
    )


def create_and_configure_robot(arm_name="middle", moving_time=0.14, accel_time=0.04,
                               startup=True):
    """Create the robot, start the ROS 2 node, and put the arm in position mode.

    `startup=False` is for callers that have already run robot_startup()
    themselves -- calling it twice is not harmful but is confusing to read.
    """
    bot = create_robot(arm_name, moving_time, accel_time)
    if startup:
        robot_startup()

    # bot.core, not bot.dxl -- see module docstring.
    bot.core.robot_set_operating_modes("group", "arm", "position")
    time.sleep(0.5)
    bot.core.robot_torque_enable("group", "arm", True)
    time.sleep(0.5)
    return bot


def shutdown():
    """Tear down the global Interbotix node. Leaves torque ON deliberately --
    dropping it would let the arm fall."""
    robot_shutdown()


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
def get_joint_positions(bot):
    """Current joint positions as a float array, in JOINT_NAMES order."""
    return np.asarray(bot.arm.get_joint_positions(), dtype=float)


def get_camera_pan(bot):
    """Just the camera pan angle (wrist_rotate)."""
    from arm_config import CAMERA_PAN_INDEX
    return float(get_joint_positions(bot)[CAMERA_PAN_INDEX])


def joint_states_look_valid(bot):
    """Cheap sanity check that the driver still has its serial link.

    When the U2D2 re-enumerates to a new /dev/ttyUSB*, the running xs_sdk keeps
    the dead descriptor and publishes -pi on every joint with velocities in the
    tens of millions. Acting on that data is dangerous, so callers should check
    before commanding anything after a long idle or a suspected dropout.
    """
    try:
        pos = get_joint_positions(bot)
        vel = np.asarray(bot.arm.get_joint_velocities(), dtype=float)
    except Exception:
        return False
    if np.any(np.abs(vel) > 100.0):
        return False
    return not validate_pose(pos)


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
def interpolate_to_pose(bot, pose, moving_time=0.2, accel_time=0.1, blocking=True):
    """Move to `pose` as a series of bounded waypoints rather than one jump.

    Same approach as the original: split the move so no single segment asks any
    joint to travel more than MAX_JOINT_STEP, which keeps commanded velocity
    predictable regardless of how far away the target is.
    """
    require_valid(pose, "target pose")

    current_q = get_joint_positions(bot)
    target_q = np.asarray(pose, dtype=float)
    delta = np.abs(target_q - current_q)

    num_steps = max(1, int(np.ceil(np.max(delta / MAX_JOINT_STEP))))
    waypoints = np.linspace(current_q, target_q, num_steps + 1)[1:]

    print(f"max_delta={np.max(delta):.3f}  steps={num_steps}  "
          f"total_time={num_steps * moving_time:.1f}s")

    for i, q in enumerate(waypoints):
        bot.arm.set_joint_positions(
            q.tolist(),
            moving_time=moving_time,
            accel_time=min(accel_time, 0.5 * moving_time),
            blocking=blocking or i < len(waypoints) - 1,
        )
    return target_q


def move_to_named_pose(bot, pose_name, moving_time=0.2, accel_time=0.1, blocking=True):
    return interpolate_to_pose(bot, get_pose(pose_name), moving_time, accel_time, blocking)


def reset_arm(bot, pose_name=DEFAULT_RESET_POSE):
    return move_to_named_pose(bot, pose_name)


def set_camera_pan(bot, angle, moving_time=0.5, accel_time=0.2, blocking=True):
    """Aim the camera without disturbing the rest of the arm."""
    from arm_config import CAMERA_PAN_JOINT
    lo, hi = JOINT_LIMITS[CAMERA_PAN_JOINT]
    if not (lo <= angle <= hi):
        raise ValueError(f"camera pan {angle:.4f} outside [{lo:.4f}, {hi:.4f}] rad")
    bot.arm.set_single_joint_position(
        joint_name=CAMERA_PAN_JOINT, position=float(angle),
        moving_time=moving_time, accel_time=accel_time, blocking=blocking,
    )


def stop_arm(bot):
    """Command the arm to hold exactly where it is right now."""
    bot.arm.set_joint_positions(
        get_joint_positions(bot).tolist(),
        moving_time=0.05, accel_time=0.02, blocking=False,
    )


def safe_move_arm_joints(bot, target_q, total_time=3.0, step_time=0.25,
                         accel_ratio=0.35):
    """Slower, more conservative variant of interpolate_to_pose.

    Two differences: tighter per-step caps, and a floor on the number of steps
    so the move takes at least `total_time` even when the distance is short.
    """
    require_valid(target_q, "target pose")

    current_q = get_joint_positions(bot)
    target_q = np.asarray(target_q, dtype=float)
    max_step = np.array([0.05, 0.05, 0.06, 0.10, 0.10, 0.12], dtype=float)

    delta = target_q - current_q
    n_steps = int(np.ceil(np.max(np.abs(delta) / max_step)))
    n_steps = max(n_steps, int(np.ceil(total_time / step_time)), 1)

    waypoints = np.linspace(current_q, target_q, n_steps + 1)[1:]
    accel_t = min(accel_ratio * step_time, 0.5 * step_time)

    for q_cmd in waypoints:
        bot.arm.set_joint_positions(
            q_cmd.tolist(), moving_time=step_time, accel_time=accel_t, blocking=True
        )
    return target_q


# ---------------------------------------------------------------------------
# Torque
# ---------------------------------------------------------------------------
def torque_on(bot):
    bot.core.robot_torque_enable("group", "arm", True)


def torque_off(bot):
    """Let the arm be back-driven. It is then held up by friction alone --
    support it or park it low first."""
    bot.core.robot_torque_enable("group", "arm", False)
