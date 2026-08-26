#!/usr/bin/env python3
"""Named poses for a WXAI manipulator -- save where the arm is, recall it later.

    ./pose.py list                    # poses for this arm, with distance from here
    ./pose.py show ready
    ./pose.py go ready                # DRY RUN -- prints the move, sends nothing
    ./pose.py go ready --execute      # actually move
    ./pose.py save pick_bin           # capture the CURRENT pose under a name
    ./pose.py delete pick_bin

Three built-ins come from trossen_arm_moveit's SRDF, so they mean the same thing
here as in MoveIt: `sleep` (folded, all zeros), `upright`, `ready`. Saved poses
live in config/poses.yaml in the bind-mounted workspace, editable from the host
and surviving image rebuilds.

POSES ARE PER ARM. That file is mounted into BOTH containers, so it is keyed by
ARM_NAME: arm-1's "pick_bin" and arm-2's "pick_bin" are different points in
space, and sharing one entry between them would drive one arm to the other's
target. The built-ins are model-level and shared, since they are joint-space
postures with no relation to where the arm is bolted down.

Saving is the intended way to build the library: float the arm with teach.py,
hand-guide it where you want it, and name it there -- teach.py can save without
dropping the connection. Poses typed in by hand tend to be subtly wrong in ways
you discover only once the arm is moving.

The gripper is NOT commanded by `go` unless you ask for --with-gripper. A pose
recall that also opened the fingers would drop whatever the arm is holding.
"""
import os
import sys

import yaml
import trossen_arm

import arm
import arm_config as cfg

POSE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "config", "poses.yaml")

BIG_MOVE_RAD = 1.0


def load_user_poses():
    if not os.path.exists(POSE_FILE):
        return {}
    with open(POSE_FILE) as f:
        return yaml.safe_load(f) or {}


def load_poses(arm_name):
    """Built-ins, shadowed by anything saved under this arm's name."""
    poses = {k: list(v) for k, v in cfg.POSES.items()}
    poses.update(load_user_poses().get(arm_name, {}))
    return poses


def save_user_poses(all_poses):
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    with open(POSE_FILE, "w") as f:
        f.write("# Named poses for the WXAI manipulators:\n")
        f.write("#   [joint_0 .. joint_5 in rad, gripper opening in m]\n")
        f.write("# The gripper value is recorded but only commanded by 'go --with-gripper'.\n")
        f.write("# Keyed by ARM_NAME -- this file is mounted into BOTH containers,\n")
        f.write("# and the same name means a different point in space per arm.\n")
        f.write("# Written by pose.py -- 'pose.py save <name>' captures the live pose.\n")
        yaml.safe_dump(all_poses, f, default_flow_style=False, sort_keys=True)


def save_current(arm_name, name, positions):
    """Persist `positions` (arm joints + gripper) under `name`. Returns the list."""
    all_poses = load_user_poses()
    vals = [round(float(p), 4) for p in positions[:cfg.GRIPPER_INDEX + 1]]
    all_poses.setdefault(arm_name, {})[name] = vals
    save_user_poses(all_poses)
    return vals


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("cmd", choices=["list", "show", "go", "save", "delete"])
    ap.add_argument("name", nargs="?")
    ap.add_argument("--arm-name", default=cfg.ARM_NAME,
                    help=f"which arm's poses to use (default {cfg.ARM_NAME})")
    ap.add_argument("--with-gripper", action="store_true",
                    help="also command the gripper to the saved opening")
    ap.add_argument("--goal-time", type=float, help="seconds for the move")
    ap.add_argument("--execute", action="store_true", help="actually move (go only)")
    args = ap.parse_args()

    poses = load_poses(args.arm_name)

    if args.cmd != "list" and not args.name:
        ap.error(f"'{args.cmd}' needs a pose name")
    if args.cmd in ("show", "go", "delete") and args.name not in poses:
        print(f"no pose named {args.name!r} for {args.arm_name}. "
              f"Known: {', '.join(sorted(poses)) or '(none)'}", file=sys.stderr)
        return 2

    if args.cmd == "delete":
        all_poses = load_user_poses()
        if args.name not in all_poses.get(args.arm_name, {}):
            print(f"{args.name!r} is a built-in from arm_config.py, not a saved pose "
                  "-- nothing to delete.\nTo change it, save one with the same name; "
                  "the saved value shadows the built-in.", file=sys.stderr)
            return 2
        all_poses[args.arm_name].pop(args.name)
        save_user_poses(all_poses)
        print(f"deleted {args.name!r} for {args.arm_name}")
        return 0

    with arm.connect(args) as driver:
        current = list(driver.get_all_positions())
        lims = arm.limits(driver)

        # Checked before anything else that moves: an arm parked outside its
        # limits faults on the mode change, not on the command, so the useful
        # place to say so is here rather than in the traceback afterwards.
        if args.cmd == "go" and args.execute:
            if bad := arm.blocked_by_position(current, lims):
                print(arm.explain_blocked(bad), file=sys.stderr)
                return 5

        if args.cmd == "save":
            vals = save_current(args.arm_name, args.name, current)
            print(f"saved {args.name!r} for {args.arm_name}:")
            for i, v in enumerate(vals):
                print(f"  {cfg.label(i):<14}{arm.fmt(i, v)}")

            # Saving is allowed regardless -- capturing where the arm actually
            # is stays useful even when it is somewhere the limits refuse. But
            # say so now: a pose stored outside the limits in force is refused
            # by every later 'go', and that failure would otherwise show up
            # detached from the decision that caused it.
            if bad := arm.uncovered_by_limits(vals, lims):
                print()
                print(arm.explain_uncovered(bad, f"pose {args.name!r}"),
                      file=sys.stderr)
            return 0

        if args.cmd == "list":
            print(f"  {args.arm_name} at {args.ip}\n")
            print(f"  {'pose':<20}{'max joint delta from here':>28}")
            for name, vals in sorted(poses.items()):
                # Arm joints only: the gripper's metres do not belong in a
                # max() next to radians.
                d = max(abs(v - current[i])
                        for i, v in enumerate(vals[:cfg.NUM_ARM_JOINTS]))
                print(f"  {name:<20}{d:>22.4f} rad")
            return 0

        # Saved poses carry 7 values (arm + gripper); the built-ins carry 6,
        # since they are postures with nothing to say about the fingers.
        vals = list(poses[args.name])
        if len(vals) not in (cfg.NUM_ARM_JOINTS, cfg.GRIPPER_INDEX + 1):
            print(f"pose {args.name!r} has {len(vals)} values, expected "
                  f"{cfg.NUM_ARM_JOINTS} or {cfg.GRIPPER_INDEX + 1}", file=sys.stderr)
            return 3

        target = list(current)
        for i, v in enumerate(vals[:cfg.NUM_ARM_JOINTS]):
            target[i] = float(v)
        indices = list(range(cfg.NUM_ARM_JOINTS))
        grip_target = (float(vals[cfg.GRIPPER_INDEX])
                       if len(vals) > cfg.NUM_ARM_JOINTS else None)
        if args.with_gripper:
            if grip_target is None:
                print(f"\n  pose {args.name!r} has no gripper value "
                      "(built-ins do not) -- --with-gripper ignored")
            else:
                target[cfg.GRIPPER_INDEX] = grip_target
                indices.append(cfg.GRIPPER_INDEX)

        if errs := [e for i in indices if (e := arm.check(i, target[i], lims))]:
            print(f"pose {args.name!r} is not reachable:\n" + "\n".join(errs),
                  file=sys.stderr)
            return 3

        print(f"  pose {args.name!r} on {args.arm_name}\n")
        print(arm.move_table(current, target, indices))

        if big := [cfg.label(i) for i in indices
                   if abs(target[i] - current[i]) >= BIG_MOVE_RAD]:
            print(f"\n  NOTE: large motion on {', '.join(big)}. A whole-arm recall "
                  "commands\n  every joint at once -- check the path is clear before "
                  "running this.")

        if args.cmd == "show":
            return 0

        goal_time = args.goal_time or cfg.goal_time_for(
            [target[i] - current[i] for i in indices])
        if not arm.confirm_or_dry_run(args, goal_time):
            return 0

        try:
            driver.set_arm_modes(trossen_arm.Mode.position)
            driver.set_arm_positions(
                [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], goal_time, True)
            if args.with_gripper and grip_target is not None:
                driver.set_gripper_mode(trossen_arm.Mode.position)
                driver.set_gripper_position(grip_target, goal_time, True)
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


# ---- the origin: where "(0, 0, 0)" is ------------------------------------
# Kept in its own file, not in poses.yaml, because the two answer different
# questions. A pose is a place to GO; the origin is the frame everything is
# MEASURED IN. Mixing them would let `pose.py save` rewrite the coordinate
# system as a side effect.
#
# This has to survive a restart. The arm re-zeroing to wherever it happens to
# be sitting is exactly wrong after a fault: the operator's mental map of the
# workspace, and every coordinate they wrote down, silently shifts by however
# far the arm drifted. Captured once, then reused until someone asks for a new
# one -- so a restart reads the live end effector and applies the ORIGINAL
# offset, which is the whole point.
ORIGIN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "config", "origin.yaml")


def load_origin(arm_name):
    """The saved world-frame offset for this arm, or None if never set."""
    if not os.path.exists(ORIGIN_FILE):
        return None
    with open(ORIGIN_FILE) as f:
        data = yaml.safe_load(f) or {}
    entry = data.get(arm_name)
    if not entry:
        return None
    xyz = entry.get("offset") if isinstance(entry, dict) else entry
    if not xyz or len(xyz) != 3:
        return None
    return [float(v) for v in xyz]


def save_origin(arm_name, xyz, note=""):
    """Persist this arm's origin. Read back verbatim by load_origin."""
    data = {}
    if os.path.exists(ORIGIN_FILE):
        with open(ORIGIN_FILE) as f:
            data = yaml.safe_load(f) or {}
    entry = {"offset": [round(float(v), 6) for v in xyz]}
    if note:
        entry["note"] = note
    data[arm_name] = entry
    os.makedirs(os.path.dirname(ORIGIN_FILE), exist_ok=True)
    with open(ORIGIN_FILE, "w") as f:
        f.write("# Where (0, 0, 0) is, per arm, in WORLD axes (after the\n")
        f.write("# ARM_WORLD_* mount remap). Captured at first start and kept\n")
        f.write("# across restarts so a fault does not move the coordinate\n")
        f.write("# system. Re-capture with the zero keys (4/5/6) in rig_key.py,\n")
        f.write("# or delete a key here to re-zero on next start.\n")
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)
    return entry["offset"]
