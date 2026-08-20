#!/usr/bin/env python3
"""Drive both WXAI arms from one process, and save poses for the pair together.

    ./both.py read                     # where both arms are, side by side
    ./both.py list                     # names saved for BOTH arms
    ./both.py save handoff             # capture both arms' current pose as 'handoff'
    ./both.py go handoff               # DRY RUN -- prints both moves, sends nothing
    ./both.py go handoff --execute     # move both, together
    ./both.py go handoff --execute --with-gripper

WHY THIS EXISTS. The one-driver-at-a-time rule is per ARM, not per process: the
two arms are separate controllers on separate IPs, so a single process can hold
both connections at once. Two shells in two containers can drive both arms, but
they cannot start a move at the same moment, and for a bimanual pose that is
usually the point.

A "set of poses" here is not a new file format. config/poses.yaml is already
keyed by arm name, and a paired pose is just the same name saved under both
arms -- so `./both.py save handoff` writes arm-1.handoff and arm-2.handoff, and
`./pose.py go handoff` in either container still works on that arm's half.
Nothing here is a special kind of pose that only this script can read.

HOW SIMULTANEOUS IT IS. Both moves are started from their own thread against
their own driver and given the same goal_time, so they set off within a few
milliseconds of each other and are meant to land together. That is close
enough to look synchronised and to hold a bimanual shape; it is not hardware
sync, and nothing here interpolates the two arms against a common clock. Do not
use it for anything where a few ms of skew between the arms matters.

THE ARMS DO NOT KNOW ABOUT EACH OTHER. There is no shared collision model, no
awareness of the other arm's links, nothing. Two arms that can reach the same
volume can absolutely drive into each other, and each will happily keep pushing
because from its own point of view it is simply carrying a load. Check the pair
of poses, not each pose on its own -- ./preview.py draws one arm at a time,
which is exactly the blind spot here.
"""
import sys
import threading
import contextlib

import trossen_arm

import arm
import arm_config as cfg
import pose as pose_lib

ARM_1 = ("arm-1", "192.168.1.2")
ARM_2 = ("arm-2", "192.168.1.3")

BIG_MOVE_RAD = 1.0


def drive(slot, driver, target, goal_time, with_gripper, out):
    """One arm's half of a paired move. Runs in its own thread.

    The mode change happens in here rather than in the caller so the two arms
    leave idle at as nearly the same moment as this can manage -- setting them
    sequentially first would put a whole round trip between them.
    """
    try:
        driver.set_arm_modes(trossen_arm.Mode.position)
        driver.set_arm_positions(
            [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], goal_time, True)
        if with_gripper and len(target) > cfg.NUM_ARM_JOINTS:
            driver.set_gripper_mode(trossen_arm.Mode.position)
            driver.set_gripper_position(
                float(target[cfg.GRIPPER_INDEX]), goal_time, True)
        out[slot] = None
    except Exception as e:
        out[slot] = e


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("cmd", choices=["read", "list", "save", "go"])
    ap.add_argument("name", nargs="?")
    ap.add_argument("--ip-1", default=ARM_1[1], help=f"default {ARM_1[1]}")
    ap.add_argument("--ip-2", default=ARM_2[1], help=f"default {ARM_2[1]}")
    ap.add_argument("--name-1", default=ARM_1[0], help=f"default {ARM_1[0]}")
    ap.add_argument("--name-2", default=ARM_2[0], help=f"default {ARM_2[0]}")
    ap.add_argument("--with-gripper", action="store_true",
                    help="also command each arm's saved gripper opening")
    ap.add_argument("--goal-time", type=float, help="seconds, shared by both arms")
    ap.add_argument("--execute", action="store_true", help="actually move")
    args = ap.parse_args()

    if args.cmd in ("save", "go") and not args.name:
        ap.error(f"'{args.cmd}' needs a pose name")

    arms = [(args.name_1, args.ip_1), (args.name_2, args.ip_2)]

    # --ip from add_common_args means "the one arm" everywhere else in this
    # directory; here there are two, so it is ignored rather than silently
    # pointing both halves at the same controller.
    if args.ip != cfg.ARM_IP:
        print("  note: --ip does not apply to both.py; use --ip-1 / --ip-2",
              file=sys.stderr)

    # Answered from the pose file alone, so it works with the arms powered
    # down. Everything below needs both controllers; this does not, and having
    # it fail because one arm is off would be gratuitous.
    if args.cmd == "list":
        saved = pose_lib.load_user_poses()
        names = [set(saved.get(n, {})) for n, _ in arms]
        both = sorted(names[0] & names[1])
        print(f"\n  saved for BOTH {arms[0][0]} and {arms[1][0]}:")
        print("   ", ", ".join(both) if both else "(none)")
        for k, (name, _) in enumerate(arms):
            if only := sorted(names[k] - names[1 - k]):
                print(f"  {name} only: {', '.join(only)}  "
                      f"(./both.py go needs it on both)")
        return 0

    with contextlib.ExitStack() as stack:
        drivers = []
        for name, ip in arms:
            try:
                drivers.append(stack.enter_context(arm.connect(args, ip=ip)))
            except SystemExit as e:
                print(f"\n  could not open {name} at {ip}.\n"
                      "  Both arms have to be reachable for a paired move. Check "
                      "./discover-arms.py,\n  and that no other script is holding "
                      "that arm.", file=sys.stderr)
                raise

        current = [list(d.get_all_positions()) for d in drivers]
        lims = [arm.limits(d) for d in drivers]

        if args.cmd == "read":
            print()
            for (name, ip), cur in zip(arms, current):
                print(f"  {name} at {ip}")
                for i in range(cfg.GRIPPER_INDEX + 1):
                    print(f"    {cfg.label(i):<14}{arm.fmt(i, cur[i])}")
                print()
            return 0

        if args.cmd == "save":
            print()
            for k, (name, _ip) in enumerate(arms):
                vals = pose_lib.save_current(name, args.name, current[k])
                print(f"  saved {args.name!r} for {name}:")
                for i, v in enumerate(vals):
                    print(f"    {cfg.label(i):<14}{arm.fmt(i, v)}")
                if bad := arm.uncovered_by_limits(vals, lims[k]):
                    print(arm.explain_uncovered(bad, f"{name}'s {args.name!r}"),
                          file=sys.stderr)
                print()
            print(f"  './both.py go {args.name}' now drives the pair; "
                  f"'./pose.py go {args.name}'\n  still drives one arm from its "
                  "own container.")
            return 0

        # go
        targets, missing = [], []
        for k, (name, _) in enumerate(arms):
            vals = pose_lib.load_poses(name).get(args.name)
            if vals is None:
                missing.append(name)
                targets.append(None)
                continue
            t = list(current[k])
            for i, v in enumerate(vals[:cfg.NUM_ARM_JOINTS]):
                t[i] = float(v)
            if args.with_gripper and len(vals) > cfg.NUM_ARM_JOINTS:
                t[cfg.GRIPPER_INDEX] = float(vals[cfg.GRIPPER_INDEX])
            targets.append(t)

        if missing:
            print(f"\n  no pose named {args.name!r} for {', '.join(missing)}.\n"
                  "  A paired move needs it saved on both arms -- park them and "
                  f"run\n  './both.py save {args.name}'.", file=sys.stderr)
            return 2

        indices = list(range(cfg.NUM_ARM_JOINTS))
        if args.with_gripper:
            indices.append(cfg.GRIPPER_INDEX)

        for k, (name, ip) in enumerate(arms):
            if errs := [e for i in indices
                        if (e := arm.check(i, targets[k][i], lims[k]))]:
                print(f"{name}: pose {args.name!r} is not reachable:\n"
                      + "\n".join(errs), file=sys.stderr)
                return 3
            if bad := arm.blocked_by_position(current[k], lims[k]):
                print(f"{name}:\n" + arm.explain_blocked(bad), file=sys.stderr)
                return 5

        # One goal_time for both, from whichever arm has furthest to travel, so
        # they arrive together instead of the shorter move finishing early and
        # holding a half-formed shape.
        goal_time = args.goal_time or max(
            cfg.goal_time_for([targets[k][i] - current[k][i] for i in indices])
            for k in range(len(arms)))

        for k, (name, ip) in enumerate(arms):
            print(f"\n  {name} at {ip}\n")
            print(arm.move_table(current[k], targets[k], indices))
            if big := [cfg.label(i) for i in indices
                       if abs(targets[k][i] - current[k][i]) >= BIG_MOVE_RAD]:
                print(f"    large motion on {', '.join(big)}")

        print(f"\n  Both arms move together over {goal_time:.1f} s. They do not "
              "know about\n  each other -- check the PAIR of poses for a "
              "collision, not each one alone.")

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
            return 0

        print(f"\n  moving both over {goal_time:.1f} s ...")
        out = {}
        threads = [
            threading.Thread(target=drive,
                             args=(k, drivers[k], targets[k], goal_time,
                                   args.with_gripper, out))
            for k in range(len(arms))
        ]
        try:
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        except KeyboardInterrupt:
            print("\n  interrupted -- both arms stop and hold where they are")
            for t in threads:
                t.join()
            return 0

        rc = 0
        for k, (name, _) in enumerate(arms):
            if out.get(k) is not None:
                print(f"  {name}: the controller rejected the move: {out[k]}\n"
                      "    that arm is idle (braked, holding). The OTHER arm may "
                      "have completed\n    its move, so the pair is now in a shape "
                      "neither pose describes.", file=sys.stderr)
                rc = 4
                continue
            print(f"  {name}:", end=" ")
            if not arm.verify(drivers[k], targets[k], indices):
                rc = 4
        return rc


if __name__ == "__main__":
    sys.exit(main())
