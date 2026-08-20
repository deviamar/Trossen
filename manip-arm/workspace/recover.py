#!/usr/bin/env python3
"""Walk a joint that is parked outside its limits back into range.

    ./recover.py                        # report + plan, sends nothing
    ./recover.py --execute              # widen, move back in, restore
    ./recover.py --ratchet --execute    # walk it out in short bursts (see below)
    ./recover.py --joint 2 --target 0.3 # a specific destination for one joint

WHY THIS IS NEEDED. The controller clips the positions you *command* to
[position_min, position_max], but it faults on the positions the motors
*report*, outside [position_min - tolerance, position_max + tolerance]. Those
checks are skipped in idle. So an arm can rest happily outside its own limits
-- read_joints.py shows it, nothing complains -- and then fault instantly the
moment any joint leaves idle, before it has looked at your target. That is the
"Joint limit exceeded ... motor reported X. Setting to idle." message, and no
amount of choosing a better target will get past it.

HOW IT GETS OUT. Joint limits are ordinary driver-session configuration, not
calibration: get_joint_limits(), edit, set_joint_limits(). They are NOT stored
in the controller's EEPROM, so they last only for this connection and come back
as the defaults on the next configure() -- which is what makes widening one
safe to do here. This script widens only the offending joint, only far enough
to cover where it actually is, moves it just inside the real limit, and puts
the original limits back before it disconnects.

WHEN ONE LONG MOVE IS NOT ENOUGH (--ratchet). On this rig the widened limit
does not survive the whole move: the joint travels roughly 0.15 rad and then
the Motor Interface faults against the ORIGINAL range again, as if its copy of
the limits gets refreshed underneath the widened ones. The progress is real and
repeatable, so --ratchet takes it in short bursts instead -- reconnect, widen,
move ~0.2 rad, disconnect, repeat -- and stops on its own if a burst fails to
move the joint, rather than straining it against an obstruction.

BEFORE YOU RUN IT, LOOK AT THE ARM. Widening a limit is right when the joint is
genuinely parked past it (folded for shipping, hand-posed while powered down).
It is wrong when the reading is bogus -- a bad position offset makes a joint
report a number that has nothing to do with where it is, and moving to "0.1"
would then drive it into a hard stop. The check: at all-zeros this arm is
FOLDED and compact -- links 2 and 3 run opposite ways and nearly cancel, so the
tool sits about 0.25 m out and 0.16 m up. (This docstring used to say "stands
straight up", which is wrong and inverts the test.) If the reported angles do
not match the shape in front of you, stop and fix the calibration instead
(get_position_offsets / joint characteristics), do not widen anything.

A joint that is permanently outside the stock range on this rig, by design
rather than by accident, belongs in cfg.JOINT_LIMIT_OVERRIDES instead -- that
is re-applied on every connect, so the joint stops reading as blocked at all
and this script correctly reports nothing to recover.
"""
import sys
import time

import trossen_arm

import arm
import arm_config as cfg

# How far inside the real limit to park the joint, and how much slack to give
# the temporarily widened limit beyond where the joint actually is.
MARGIN = 0.10
WIDEN_SLACK = 0.30

# Deliberately slower than a normal move: this one starts from a pose the arm
# is not supposed to be in, often folded against itself.
RECOVER_SPEED_RAD_S = 0.3

# Ratchet mode. Observed on this rig: a widened limit holds for roughly a
# second of motion and then the Motor Interface faults against the ORIGINAL
# range again, having let the joint travel ~0.15 rad in the meantime. Whatever
# the firmware is doing there, the progress is real and it is repeatable, so a
# short burst per connection walks the joint out where one long move cannot.
STEP_RAD = 0.20
STEP_TIME_S = 1.0
STALL_RAD = 0.02
MAX_BURSTS = 15


def burst(args, targets):
    """One short widened move on every still-blocked joint in `targets`.

    `targets` is {joint_index: final target}. Returns
    (before, after, faulted, still_blocked) where the first two are dicts and
    still_blocked lists the joints that remain outside their range.

    A fresh connection per burst is not optional: the fault latches an error
    and drops the link, so the next attempt has to reconnect and clear anyway.
    """
    with arm.connect(args, clear_error=True) as driver:
        pos = list(driver.get_all_positions())
        lims = arm.limits(driver)

        blocked = [j for j in targets
                   if not (lims[j][0] - lims[j][2] <= pos[j] <= lims[j][1] + lims[j][2])]
        before = {j: pos[j] for j in targets}
        if not blocked:
            return before, before, False, []

        widened = driver.get_joint_limits()
        target = list(pos)
        for j in blocked:
            lo, hi, _ = lims[j]
            direction = 1.0 if targets[j] > pos[j] else -1.0
            goal = pos[j] + direction * min(STEP_RAD, abs(targets[j] - pos[j]))
            target[j] = goal
            widened[j].position_min = min(lo, pos[j] - WIDEN_SLACK, goal - WIDEN_SLACK)
            widened[j].position_max = max(hi, pos[j] + WIDEN_SLACK, goal + WIDEN_SLACK)

        faulted = False
        try:
            driver.set_joint_limits(widened)
            driver.set_arm_modes(trossen_arm.Mode.position)
            driver.set_arm_positions(
                [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], STEP_TIME_S, True)
        except Exception:
            faulted = True

        try:
            now = list(driver.get_all_positions())
            after = {j: now[j] for j in targets}
        except Exception:
            after = None  # link died with the fault; the next burst re-reads
        return before, after, faulted, blocked


def ratchet(args, targets):
    """Repeat short bursts until every joint is back in range, or motion stops."""
    names = ", ".join(cfg.label(j) for j in targets)
    print(f"\n  ratcheting {names} in {STEP_RAD} rad bursts (max {MAX_BURSTS}).\n"
          "  Ctrl-C between bursts stops it; the arm holds where it is.\n")

    for n in range(1, MAX_BURSTS + 1):
        before, after, faulted, blocked = burst(args, targets)
        if not blocked:
            print(f"  burst {n}: every joint is inside its limits.")
            print("\n  recovered. Re-read with ./read_joints.py --limits, then "
                  "dry-run\n  ./pose.py go sleep before sending anything.")
            return 0
        if after is None:
            print(f"  burst {n}: link dropped with the fault; re-reading next burst")
            time.sleep(1.0)
            continue

        moved = 0.0
        for j in blocked:
            d = after[j] - before[j]
            moved = max(moved, abs(d))
            print(f"  burst {n}: {cfg.label(j)} {before[j]:+.4f} -> "
                  f"{after[j]:+.4f} ({d:+.4f} rad)"
                  f"{'  [faulted]' if faulted else ''}")

        if moved < STALL_RAD:
            print(f"\n  STALLED -- the best joint moved {moved:.4f} rad, under the "
                  f"{STALL_RAD} threshold.\n"
                  "  Stopping rather than straining against whatever is holding it.\n"
                  "  Hand-pose it instead, watching ./read_joints.py --watch --limits,\n"
                  "  or treat it as a support case: "
                  "https://www.trossenrobotics.com/support", file=sys.stderr)
            return 7
        time.sleep(1.0)  # let the controller settle before reconnecting

    print("\n  ran out of bursts. Re-run to continue -- each run starts from "
          "wherever\n  the joints got to.", file=sys.stderr)
    return 7


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("--joint", type=int, help="only recover this joint index")
    ap.add_argument("--target", type=float,
                    help="where to send it (default: just inside the nearest limit)")
    ap.add_argument("--ratchet", action="store_true",
                    help="walk it out in short bursts instead of one long move")
    ap.add_argument("--execute", action="store_true", help="actually move")
    args = ap.parse_args()

    # A blocked arm is nearly always sitting on a latched error from the last
    # attempt, so clearing is the default here rather than opt-in as elsewhere.
    with arm.connect(args, clear_error=True) as driver:
        current = list(driver.get_all_positions())
        lims = arm.limits(driver)
        bad = arm.blocked_by_position(current, lims)

        if args.joint is not None:
            bad = [b for b in bad if b[0] == args.joint]

        print(f"  {cfg.ARM_NAME} at {args.ip}\n")
        if not bad:
            print("  every joint is inside its limits -- nothing to recover.")
            print("  If a move still faults, the log line names the joint; it may be")
            print("  a velocity or effort limit rather than a position one.")
            return 0

        target = list(current)
        for i, p, lo, hi, tol in bad:
            if args.target is not None:
                target[i] = args.target
            else:
                target[i] = lo + MARGIN if p < lo else hi - MARGIN
            print(f"  {cfg.label(i)} is at {p:.4f}, outside "
                  f"[{lo:.4f}, {hi:.4f}] (tolerance {tol:.2f})")
            print(f"    -> move to {target[i]:.4f}, "
                  f"delta {target[i] - p:+.4f} rad "
                  f"({(target[i] - p) * 57.29578:+.1f} deg)")

        indices = [b[0] for b in bad]
        goal_time = cfg.goal_time_for([target[i] - current[i] for i in indices],
                                      speed=RECOVER_SPEED_RAD_S, minimum=3.0)

        if any(i == cfg.GRIPPER_INDEX for i in indices):
            print("\n  NOTE: the gripper is one of the blocked joints. Its target is "
                  "in metres.")

        print(f"\n  This is a {goal_time:.1f} s move from a pose the arm is not "
              "supposed to be in.\n"
              "  Check the path is clear and keep a hand near the estop.")

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute"
                  f"{', or --ratchet --execute' if len(bad) == 1 else ''}.")
            return 0

    # Ratchet runs its own connection per burst, so it has to be outside the
    # `with` above -- two drivers on one arm is a refused connection.
    if args.ratchet:
        return ratchet(args, {b[0]: target[b[0]] for b in bad})

    with arm.connect(args, clear_error=True) as driver:

        # Widen only the offending joints, and only enough to cover where they
        # are. Everything else keeps the limits that are protecting it.
        #
        # The originals are snapshotted as plain numbers rather than kept as
        # JointLimit objects: two get_joint_limits() calls may hand back
        # references to the same underlying objects, in which case editing the
        # "copy" silently edits the "original" too and the restore below writes
        # the widened values straight back.
        original = [(l.position_min, l.position_max)
                    for l in driver.get_joint_limits()]
        widened = driver.get_joint_limits()
        for i, p, lo, hi, tol in bad:
            widened[i].position_min = min(lo, p - WIDEN_SLACK)
            widened[i].position_max = max(hi, p + WIDEN_SLACK)
            print(f"  widening {cfg.label(i)} to "
                  f"[{widened[i].position_min:.4f}, {widened[i].position_max:.4f}] "
                  "for this move only")

        ok = False
        try:
            driver.set_joint_limits(widened)

            # Read back rather than assume. If the controller clamps or ignores
            # the write, the move is about to fault for the same reason as
            # before, and the readback is the only place that shows it.
            applied = arm.limits(driver)
            for i, p, _, _, _ in bad:
                lo, hi, tol = applied[i]
                inside = lo - tol <= p <= hi + tol
                print(f"  {cfg.label(i)} limits now [{lo:.4f}, {hi:.4f}] "
                      f"+/- {tol:.2f} -- current {p:.4f} is "
                      f"{'INSIDE' if inside else 'STILL OUTSIDE'}")
                if not inside:
                    print("\n  The controller did not accept the widened limit, so "
                          "the move would\n  fault exactly as before. Stopping here "
                          "rather than trying it.\n"
                          "  This is a case for Trossen support: "
                          "https://www.trossenrobotics.com/support",
                          file=sys.stderr)
                    return 6
            driver.set_arm_modes(trossen_arm.Mode.position)
            driver.set_arm_positions(
                [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], goal_time, True)
            if cfg.GRIPPER_INDEX in indices:
                driver.set_gripper_mode(trossen_arm.Mode.position)
                driver.set_gripper_position(
                    float(target[cfg.GRIPPER_INDEX]), goal_time, True)
            ok = True
        except KeyboardInterrupt:
            print("\n  interrupted -- the arm stops and holds where it is")
        except Exception as e:
            print(f"\n  the controller rejected the move: {type(e).__name__}: {e}",
                  file=sys.stderr)
        finally:
            # Always attempted, including after a fault: leaving an arm running
            # on widened limits is exactly the state this script exists to get
            # out of. It can fail if the connection died with the fault, which
            # is harmless -- the limits are session-scoped and the next
            # configure() restores the defaults anyway.
            try:
                restore = driver.get_joint_limits()
                for i, (lo, hi) in enumerate(original):
                    restore[i].position_min = lo
                    restore[i].position_max = hi
                driver.set_joint_limits(restore)
                print("  original limits restored.")
            except Exception:
                print("  could not restore limits over this connection; they are\n"
                      "  session-scoped, so reconnecting resets them to the defaults.")

        if not ok:
            return 4

        if not arm.verify(driver, target, indices):
            return 4
        print("\n  back inside the limits. ./pose.py go sleep should work now "
              "-- dry-run it first.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
