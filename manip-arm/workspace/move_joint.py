#!/usr/bin/env python3
"""Command one joint (or all of them) on a WXAI manipulator. Dry-run by default.

    ./move_joint.py shoulder 1.2            # absolute radians
    ./move_joint.py 1 1.2                   # same joint, by index
    ./move_joint.py --rel wrist_rotate 0.2  # relative to where it is now
    ./move_joint.py --deg base 15           # degrees instead of radians
    ./move_joint.py --group 0 1.57 1.57 0 0 0
    ./move_joint.py gripper 0.02            # gripper is METRES, see gripper.py
    ./move_joint.py shoulder 1.2 --execute  # actually move

Nothing is sent without --execute. The default prints the move it *would* make
with the delta from the current position, because a mistyped radian value looks
exactly like a correct one until the arm moves.

Limits come from the arm itself (get_joint_limits), not from a table in this
repo, so they track whatever the controller is actually enforcing. Targets
outside the range are refused rather than clamped, unless you ask for --clamp.

goal_time is computed from the distance to travel (see arm_config.goal_time_for)
rather than fixed. This is not just about gentleness: the controller rejects a
step it cannot physically follow, errors out, and drops every joint to idle
mid-motion. Asking for a big move in a short time is how you trigger that.
"""
import math
import sys

import trossen_arm

import arm
import arm_config as cfg


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("joint", nargs="?", help="index, joint_N, or alias (base, shoulder, ...)")
    ap.add_argument("value", nargs="?", help="target position (absolute unless --rel)")
    # Explicit flag rather than inferring from a leading minus: "-1.2" is
    # genuinely ambiguous between a negative absolute target and a decrease,
    # and guessing wrong turns a small correction into a large swing.
    ap.add_argument("--rel", action="store_true", help="value is a delta from current")
    ap.add_argument("--group", nargs="+", metavar="V",
                    help=f"command all {cfg.NUM_ARM_JOINTS} arm joints at once (no gripper)")
    ap.add_argument("--deg", action="store_true", help="values are degrees (not the gripper)")
    ap.add_argument("--clamp", action="store_true", help="clamp to the limit instead of refusing")
    ap.add_argument("--goal-time", type=float, help="seconds for the move (default: from distance)")
    ap.add_argument("--execute", action="store_true", help="actually send it")
    args = ap.parse_args()

    if not args.group and (not args.joint or args.value is None):
        ap.error("give a joint and a value, or use --group")

    # Resolved before connecting: a typo'd joint name should not cost you a
    # connection attempt, and it is the one error that needs nothing from the arm.
    single = None
    if not args.group:
        single = cfg.joint_index(args.joint)
        if single is None:
            print(f"unknown joint {args.joint!r}; have: "
                  f"{', '.join(cfg.DISPLAY_NAMES)}\n"
                  f"  also accepted: an index 0-{cfg.GRIPPER_INDEX}, or the URDF "
                  f"names {', '.join(cfg.JOINT_NAMES)}", file=sys.stderr)
            return 2
        if args.deg and single == cfg.GRIPPER_INDEX:
            # The gripper is linear; degrees would silently turn "20" into
            # 0.35 m of travel it does not have.
            print("--deg does not apply to the gripper (metres)", file=sys.stderr)
            return 2

    with arm.connect(args) as driver:
        lims = arm.limits(driver)
        current = list(driver.get_all_positions())
        target = list(current)

        if args.group:
            if len(args.group) != cfg.NUM_ARM_JOINTS:
                print(f"--group needs {cfg.NUM_ARM_JOINTS} values "
                      f"({', '.join(cfg.DISPLAY_NAMES[:cfg.NUM_ARM_JOINTS])}), "
                      f"got {len(args.group)}", file=sys.stderr)
                return 2
            for i, v in enumerate(args.group):
                target[i] = math.radians(float(v)) if args.deg else float(v)
            indices = list(range(cfg.NUM_ARM_JOINTS))
        else:
            i = single
            v = float(args.value)
            if args.deg:
                v = math.radians(v)
            target[i] = current[i] + v if args.rel else v
            indices = [i]

        if args.clamp:
            for i in indices:
                lo, hi, _ = lims[i]
                clamped = min(hi, max(lo, target[i]))
                if clamped != target[i]:
                    print(f"  clamped {cfg.label(i)} "
                          f"{target[i]:.4f} -> {clamped:.4f} (joint limit)")
                    target[i] = clamped

        if errs := [e for i in indices if (e := arm.check(i, target[i], lims))]:
            print("\n".join(errs), file=sys.stderr)
            print("  use --clamp to go as far as the limit allows", file=sys.stderr)
            return 3

        print(f"  {cfg.ARM_NAME} at {args.ip}\n")
        print(arm.move_table(current, target, indices))

        goal_time = args.goal_time or cfg.goal_time_for(
            [target[i] - current[i] for i in indices])
        if not arm.confirm_or_dry_run(args, goal_time):
            return 0

        # An arm parked outside its limits faults on the mode change, before
        # the target is ever read. Say so here, not in the traceback.
        if bad := arm.blocked_by_position(current, lims):
            print("\n" + arm.explain_blocked(bad), file=sys.stderr)
            return 5

        try:
            # The gripper and the arm joints are separate scopes in this SDK,
            # so a command that touches both has to set both modes and send two
            # commands. Sending an arm-scope vector that includes the gripper
            # value is the mistake this branch exists to avoid.
            if any(i != cfg.GRIPPER_INDEX for i in indices):
                driver.set_arm_modes(trossen_arm.Mode.position)
                driver.set_arm_positions(
                    [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], goal_time, True)
            if cfg.GRIPPER_INDEX in indices:
                driver.set_gripper_mode(trossen_arm.Mode.position)
                driver.set_gripper_position(
                    float(target[cfg.GRIPPER_INDEX]), goal_time, True)
        except KeyboardInterrupt:
            print("\n  interrupted -- the arm stops and holds where it is")
            return 0
        except Exception as e:
            print(f"\n  the controller rejected the move: {e}\n"
                  "  it has set every joint to idle (braked, holding). Read the\n"
                  "  message above, then reconnect with --clear-error.", file=sys.stderr)
            return 4

        if not arm.verify(driver, target, indices):
            return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
