"""Streaming IK for the active-vision arm, on pyroki.

Structure follows giava@real-v2-spr26 three_arm_ik.py: a jitted least-squares
problem with a pose cost, joint-limit constraints, and a previous-configuration
residual scaled by the per-step velocity budget, plus a hard velocity clamp on
the way out. Reduced from three coupled arms to one, because on this rig the two
manipulators solve their own IK inside the WXAI controller and are not part of
the same kinematic problem.

WHAT THAT COSTS, STATED PLAINLY
-------------------------------
giava solved all three arms in ONE problem, which is what let the solver trade
one arm's posture off against another's when they share a workspace. Here the
arms are solved independently and none of them knows the others exist. Nothing
in this file prevents the camera arm and a manipulator occupying the same space.
On a rig where they can reach each other, that is the operator's problem now --
or an argument for building a single URDF of all three arms and going back to a
coupled solve. See docs/topic-contract.md.

WHY THE PREVIOUS-CONFIGURATION RESIDUAL MATTERS
-----------------------------------------------
Streaming IK at 50 Hz without it produces elbow flips: two configurations reach
the same end-effector pose, the solver picks whichever wins by a hair this
frame, and the arm slams between them. Penalising distance from the last
commanded configuration -- scaled by how far each joint could travel in one dt
-- makes the solution continuous in time. It is the difference between usable
teleop and a machine that occasionally throws itself across its workspace.

FIRST CALL COMPILES. jax traces and compiles on the first solve, which takes
seconds. warmup() exists to pay that before anything is moving rather than as a
stall in the middle of a live session.
"""
from __future__ import annotations

import os

import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import numpy as np
import pyroki as pk

EPS = 1e-6

# ORIENTATION OUTWEIGHS POSITION ON THIS ARM, which is the opposite of a
# manipulator and deliberate: it carries a camera, and where the camera LOOKS
# is the thing the operator is steering. These are giava's own middle-arm
# weights (pos 10 / ori 25); they were 40 / 0.25 here, inherited from a gripper
# arm, and that ratio made the solver almost indifferent to orientation.
#
# It showed up the moment the seventh joint arrived. camera_yaw is the last
# joint and the camera sits on its axis, so yaw changes ONLY orientation -- at
# an orientation weight of 0.25 against a position weight of 40 the solver had
# no reason to turn it at all, and head rotation moved the camera barely or not
# at all while translation worked fine.
POS_WEIGHT = float(os.environ.get("MIDDLE_POS_WEIGHT", 10.0))
ORI_WEIGHT = float(os.environ.get("MIDDLE_ORI_WEIGHT", 25.0))

# THE ARM IS REDUNDANT NOW, and that has to be resolved deliberately.
#
# Seven joints for a six-DOF target leaves a one-dimensional family of
# configurations that all put the camera in exactly the same place. Nothing in
# a pose cost distinguishes them, so the solver is free to slide along that
# family -- which is what "the motion went wild" is: joints travelling a long
# way to produce a camera pose a small motion would also have produced.
#
# Two terms choose for it, and their sizes are RELATIVE to the pose weights
# above, which is why raising ORI_WEIGHT from 0.25 to 25 without touching these
# made it worse rather than better:
#
#   DQ_WEIGHT      penalises moving at all since the last command. Keeps the
#                  solution continuous in time -- the anti-elbow-flip term.
#   CENTER_WEIGHT  pulls the whole arm toward a NEUTRAL POSTURE (the saved
#                  'start' pose by default). This is the one that actually
#                  resolves redundancy: of all the configurations that aim the
#                  camera correctly, prefer the one that looks like the pose
#                  you chose. giava uses the same idea (their centering term).
DQ_WEIGHT = float(os.environ.get("MIDDLE_DQ_WEIGHT", 2.0))
CENTER_WEIGHT = float(os.environ.get("MIDDLE_CENTER_WEIGHT", 1.0))


@jaxls.Cost.create_factory
def previous_configuration_residual_scaled(vals, joint_var, prev_q, smoothness_scales):
    """Penalise change from the previously commanded configuration."""
    q = vals[joint_var]
    return smoothness_scales * (q - prev_q)


@jaxls.Cost.create_factory
def centering_residual(vals, joint_var, q_center, weight):
    """Pull toward a neutral posture -- how the extra joint is spent.

    Applies to every joint, but it only CHANGES anything in the directions the
    pose cost does not constrain: with six constraints on seven joints there is
    one such direction, and without this term the solver wanders along it.
    """
    return weight * (vals[joint_var] - q_center)


def make_middle_arm_ik_solver(robot: pk.Robot, target_link_name: str):
    """Return (solve, warmup) for one arm.

    solve(target_position, target_wxyz, prev_q, dt, joint_velocity_limits, ...)
    returns a joint configuration, velocity-clamped against prev_q.
    """
    if target_link_name not in robot.links.names:
        raise ValueError(
            f"link {target_link_name!r} is not in the robot model; "
            f"have: {', '.join(robot.links.names)}")

    target_link_index = jnp.asarray(robot.links.names.index(target_link_name))

    @jdc.jit
    def _solve_jax(prev_q, target_position, target_wxyz, dt,
                   joint_velocity_limits, position_weight, orientation_weight,
                   active, dq_weight, q_center, center_weight):
        joint_var = robot.joint_var_cls(0)
        costs = [
            pk.costs.pose_cost_analytic_jac(
                robot,
                joint_var,
                jaxlie.SE3.from_rotation_and_translation(
                    jaxlie.SO3(target_wxyz), target_position),
                target_link_index,
                # active=0 removes the pose cost without changing the problem
                # structure, so the same compiled solver handles "hold still"
                # and would not need retracing.
                pos_weight=position_weight * active,
                ori_weight=orientation_weight * active,
            ),
            pk.costs.limit_constraint(robot, joint_var),
        ]

        max_dq = jnp.maximum(joint_velocity_limits * dt, EPS)
        costs.append(
            previous_configuration_residual_scaled(
                joint_var=joint_var,
                prev_q=prev_q,
                smoothness_scales=dq_weight / max_dq,
            )
        )

        costs.append(
            centering_residual(
                joint_var=joint_var, q_center=q_center, weight=center_weight)
        )

        problem = jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var])
        solution = problem.analyze().solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
        )
        return solution[joint_var]

    def _checked(name, value, shape):
        arr = np.asarray(value, dtype=np.float32)
        if arr.shape != shape:
            raise ValueError(f"{name} must have shape {shape}; got {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{name} contains NaN or infinite values")
        return arr

    def solve(target_position, target_wxyz, prev_q, dt, joint_velocity_limits,
              position_weight=POS_WEIGHT, orientation_weight=ORI_WEIGHT, active=1.0,
              dq_weight=DQ_WEIGHT, q_center=None, center_weight=CENTER_WEIGHT,
              block_until_ready=True):
        target_position = _checked("target_position", target_position, (3,))
        target_wxyz = _checked("target_wxyz", target_wxyz, (4,))
        prev_q = np.asarray(prev_q, dtype=np.float32)
        vel = np.asarray(joint_velocity_limits, dtype=np.float32)
        # No neutral posture given: centre on where the arm already is, which
        # makes the term a no-op rather than a pull toward an arbitrary zero.
        qc = (np.asarray(prev_q, dtype=np.float32) if q_center is None
              else np.asarray(q_center, dtype=np.float32))

        q = _solve_jax(
            jnp.asarray(prev_q), jnp.asarray(target_position),
            jnp.asarray(target_wxyz), jnp.asarray(np.float32(dt)),
            jnp.asarray(vel), jnp.asarray(np.float32(position_weight)),
            jnp.asarray(np.float32(orientation_weight)),
            jnp.asarray(np.float32(active)), jnp.asarray(np.float32(dq_weight)),
            jnp.asarray(qc), jnp.asarray(np.float32(center_weight)),
        )
        if block_until_ready:
            q = jax.block_until_ready(q)
        q = np.asarray(q, dtype=np.float64)

        # Hard clamp after the solve. The residual above is a soft preference
        # that a strong pose cost can outvote; this is the guarantee. Without
        # it a single infeasible target can still produce a joint step the arm
        # will try to execute in one control period.
        max_dq = np.maximum(vel.astype(np.float64) * float(dt), EPS)
        return np.clip(q, prev_q - max_dq, prev_q + max_dq)

    def warmup(prev_q, dt, joint_velocity_limits, **kwargs):
        """Force compilation. Seconds on the first call, microseconds after."""
        solve(np.zeros(3, dtype=np.float32), np.array([1, 0, 0, 0], np.float32),
              prev_q, dt, joint_velocity_limits, active=0.0, **kwargs)

    return solve, warmup


def load_robot(urdf_path: str) -> pk.Robot:
    """pk.Robot from a URDF on disk.

    Kept separate from the solver so the URDF source is a deployment decision.
    The wx250s description is generated by the interbotix xacro already in this
    image -- see head_agent.py --urdf.
    """
    import yourdfpy
    return pk.Robot.from_urdf(yourdfpy.URDF.load(urdf_path))
