#!/usr/bin/env python3
"""Measure how far a joint actually lands from where it was told to go.

    ./tracking.py elbow --from -1.8 --to -1.2 --steps 7            # DRY RUN
    ./tracking.py elbow --from -1.8 --to -1.2 --steps 7 --execute
    ./tracking.py shoulder --from -2.6 --to -0.5 --steps 8 --execute --settle 4
    ./tracking.py elbow --from -1.8 --to -1.2 --steps 5 --one-way --execute

"current and target are really off" has at least four causes on this arm, and
they need different fixes. Guessing between them from a single failed move does
not work, because they all look identical from there -- one number that is not
the number you asked for. They are easy to tell apart from a SWEEP, because
they differ in how the error behaves as the joint moves and as the load on it
changes:

  * A CONSTANT error, same size and sign everywhere and in both directions, is
    a calibration offset. The joint is where it thinks it is minus a fixed
    amount. Fix: that joint's position_offset (joint characteristics, EEPROM).

  * An error that TRACKS THE LOAD -- small where the joint is balanced, large
    where gravity pulls hardest, always in the direction gravity pulls -- is
    under-compensated gravity. The controller is not feeding forward enough
    torque to hold the arm against the model it was given. Fix: ARM_EE first
    (a wrong end effector means the model does not know about the mass on the
    flange), then effort_correction.

  * An error that DEPENDS ON APPROACH DIRECTION -- the joint lands short from
    above and short from below, so the up-sweep and down-sweep readings
    straddle the target -- is friction or backlash. Fix: the friction terms in
    joint characteristics; a bit of it is mechanical and simply lives there.

  * CREEP after the move ends -- it lands close, then drifts while sitting --
    is the hold not holding. Some of this is expected: idle is a torque-capped
    PID, deliberately soft (see arm.py). A lot of it, in position mode, is not.

So this sweeps a joint across a range, stopping at each step, and records what
it was told, what it reached, where it had drifted to after settling, and what
external effort it was carrying there. Then it sweeps back down the same points
so every target is measured from both directions. The summary at the end names
which of the four patterns the numbers actually fit rather than leaving it to
be eyeballed.

Nothing moves without --execute. This drives one joint repeatedly across a
range you choose, so dry-run it first and read the plan: the range is yours to
get wrong, and a sweep is a lot of motion to discover that during.

Arm joints only. The gripper is a linear, force-native joint whose "error" at a
target means something different (it stops on the object, which is the point),
and mixing it into these statistics would only produce a misleading average.
"""
import math
import sys
import time

import trossen_arm

import arm
import arm_config as cfg

SETTLE_S = 2.0
STEP_SPEED_RAD_S = 0.4      # gentler than the default: many small moves, no rush
MIN_STEP_TIME_S = 1.5

# Reporting thresholds. Deliberately loose -- these decide which paragraph gets
# printed, not whether anything is wrong, and a borderline case should say so
# rather than be silently sorted into one bucket.
FLAT_SPREAD_RAD = 0.02      # error this consistent across the sweep reads as constant
LOAD_CORRELATION = 0.6      # |r| above this reads as load-tracking
HYSTERESIS_RAD = 0.02       # up/down gap above this reads as friction
CREEP_RAD = 0.01            # drift while settling above this is worth naming


def pearson(xs, ys):
    """Correlation of two samples, or None if either does not vary."""
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx < 1e-9 or sy < 1e-9:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


def step_to(driver, index, value, current, settle):
    """Command one joint to `value`, then measure. Returns a row dict or None.

    Two reads, not one: the position the move ends at and the position it has
    drifted to after settling are different measurements of different things,
    and collapsing them hides creep entirely.
    """
    target = list(current)
    target[index] = value
    goal_time = max(MIN_STEP_TIME_S,
                    abs(value - current[index]) / STEP_SPEED_RAD_S)

    try:
        driver.set_arm_positions(
            [float(v) for v in target[:cfg.NUM_ARM_JOINTS]], goal_time, True)
    except Exception as e:
        print(f"\n  the controller rejected the step to {value:.4f}: {e}",
              file=sys.stderr)
        return None

    if (err := driver.get_error_information()) and err != "No error":
        print(f"\n  controller faulted at target {value:.4f}: {err!r}",
              file=sys.stderr)
        return None

    reached = list(driver.get_all_positions())[index]
    time.sleep(settle)
    pos = list(driver.get_all_positions())
    eff = list(driver.get_all_external_efforts())
    return {"target": value, "reached": reached, "settled": pos[index],
            "effort": eff[index], "pose": pos}


def sweep(driver, index, targets, settle, direction):
    """Walk `targets` in order, measuring each. Stops early on a fault."""
    rows = []
    for k, v in enumerate(targets, 1):
        current = list(driver.get_all_positions())
        print(f"  {direction} {k}/{len(targets)}: -> {v:+.4f} ", end="", flush=True)
        row = step_to(driver, index, v, current, settle)
        if row is None:
            print("FAILED")
            break
        row["direction"] = direction
        rows.append(row)
        print(f"reached {row['reached']:+.4f}  settled {row['settled']:+.4f}  "
              f"err {row['settled'] - v:+.4f}")
    return rows


def report(index, rows):
    """The table, then which of the four patterns the numbers fit."""
    name = cfg.label(index)
    print(f"\n  {name}: {len(rows)} measurements\n")
    print(f"  {'dir':<5}{'target':>10}{'reached':>10}{'settled':>10}"
          f"{'error':>10}{'creep':>9}{'ext eff':>10}")
    print("  " + "-" * 64)
    for r in rows:
        print(f"  {r['direction']:<5}{r['target']:>10.4f}{r['reached']:>10.4f}"
              f"{r['settled']:>10.4f}{r['settled'] - r['target']:>+10.4f}"
              f"{r['settled'] - r['reached']:>+9.4f}{r['effort']:>10.3f}")

    errs = [r["settled"] - r["target"] for r in rows]
    mean_err = sum(errs) / len(errs)
    spread = max(errs) - min(errs)
    creeps = [abs(r["settled"] - r["reached"]) for r in rows]
    mean_creep = sum(creeps) / len(creeps)

    print(f"\n  mean error {mean_err:+.4f} rad ({math.degrees(mean_err):+.2f} deg)"
          f",  spread {spread:.4f} rad,  mean creep {mean_creep:.4f} rad")

    # Up/down gap at matched targets: the direction-dependence test.
    up = {round(r["target"], 4): r["settled"] for r in rows if r["direction"] == "up"}
    down = {round(r["target"], 4): r["settled"] for r in rows if r["direction"] == "down"}
    shared = sorted(set(up) & set(down))
    hyst = None
    if shared:
        gaps = [down[t] - up[t] for t in shared]
        hyst = sum(gaps) / len(gaps)
        print(f"  up/down gap at the {len(shared)} shared targets: "
              f"{hyst:+.4f} rad mean")

    r_load = pearson([abs(x["effort"]) for x in rows], [abs(e) for e in errs])
    if r_load is not None:
        print(f"  correlation |error| vs |external effort|: r = {r_load:+.2f}")

    print("\n  READING THIS")
    hits = []
    if abs(mean_err) > FLAT_SPREAD_RAD and spread < FLAT_SPREAD_RAD * 2:
        note = _frac(mean_err)
        hits.append(
            f"  * CONSTANT OFFSET. The error is {mean_err:+.4f} rad almost "
            f"everywhere\n    (spread only {spread:.4f}). That is the shape of a "
            "calibration offset,\n    not a control problem: this joint's "
            f"position_offset is out by about\n    {-mean_err:+.4f} rad."
            + (f"\n    That is close to {note}" if note else ""))
    if r_load is not None and abs(r_load) > LOAD_CORRELATION:
        hits.append(
            f"  * LOAD-TRACKING. |error| follows |external effort| (r = {r_load:+.2f}),\n"
            "    so the joint lands short exactly where it is working hardest. That "
            "is\n    under-compensated gravity. Check ARM_EE first -- it is "
            f"{cfg.ARM_EE!r},\n    and if the flange carries a gripper that value "
            "makes the model wrong by\n    the whole end effector. Then "
            "effort_correction for this joint.")
    if hyst is not None and abs(hyst) > HYSTERESIS_RAD:
        hits.append(
            f"  * DIRECTION-DEPENDENT. Approaching from below and from above lands "
            f"{abs(hyst):.4f} rad\n    apart. That is friction or backlash, not "
            "offset -- an offset would shift\n    both sweeps the same way. The "
            "friction terms in joint characteristics\n    are the knob; some of it "
            "is mechanical and will not tune out.")
    if mean_creep > CREEP_RAD:
        hits.append(
            f"  * CREEP. It drifts {mean_creep:.4f} rad after the move ends, while "
            "still\n    commanded. Position mode is not holding it. Same root cause "
            "as\n    load-tracking above if the drift is downhill.")
    if not hits:
        hits.append("  * Nothing stands out. The error is small, consistent, "
                    "direction-independent,\n    and not creeping -- which is what "
                    "a healthy joint looks like.")
    print("\n".join(hits))

    print("\n  None of these are changed by this script. It measures; the fixes "
          "are in\n  docker-compose.yml (ARM_EE) and joint characteristics "
          "(offsets, friction,\n  effort correction) -- and see check-config.py "
          "before touching the latter,\n  since they are per-arm EEPROM values "
          "that a reset destroys.")


def _frac(v):
    """Name the pi fraction an offset is near, for the constant-offset note."""
    for nm, f in (("pi", math.pi), ("pi/2", math.pi / 2), ("pi/3", math.pi / 3),
                  ("pi/4", math.pi / 4), ("pi/6", math.pi / 6)):
        if abs(abs(v) - f) <= 0.05:
            return f"{'-' if v < 0 else '+'}{nm}, the classic homing error."
    return None


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("joint", help="index, URDF name, or alias (not the gripper)")
    ap.add_argument("--from", dest="lo", type=float, required=True,
                    help="start of the sweep, radians")
    ap.add_argument("--to", dest="hi", type=float, required=True,
                    help="end of the sweep, radians")
    ap.add_argument("--steps", type=int, default=7, help="points per sweep (default 7)")
    ap.add_argument("--settle", type=float, default=SETTLE_S,
                    help=f"seconds to wait before the second read (default {SETTLE_S})")
    ap.add_argument("--one-way", action="store_true",
                    help="skip the return sweep (loses the friction/backlash test)")
    ap.add_argument("--execute", action="store_true", help="actually move")
    args = ap.parse_args()

    index = cfg.joint_index(args.joint)
    if index is None:
        print(f"unknown joint {args.joint!r}; have: {', '.join(cfg.DISPLAY_NAMES)}",
              file=sys.stderr)
        return 2
    if index == cfg.GRIPPER_INDEX:
        print("the gripper is not a position-tracking joint in the sense this "
              "script\nmeasures -- see the docstring. Use ./gripper.py status.",
              file=sys.stderr)
        return 2
    if args.steps < 3:
        print("--steps needs at least 3 to say anything about a trend",
              file=sys.stderr)
        return 2

    span = args.hi - args.lo
    targets = [args.lo + span * k / (args.steps - 1) for k in range(args.steps)]

    with arm.connect(args) as driver:
        lims = arm.limits(driver)
        current = list(driver.get_all_positions())

        if errs := [e for t in targets if (e := arm.check(index, t, lims))]:
            print("\n".join(dict.fromkeys(errs)), file=sys.stderr)
            return 3
        if bad := arm.blocked_by_position(current, lims):
            print(arm.explain_blocked(bad), file=sys.stderr)
            return 5

        print(f"  {cfg.ARM_NAME} at {args.ip}\n")
        print(f"  sweeping {cfg.label(index)} over "
              f"[{min(targets):+.4f}, {max(targets):+.4f}] rad "
              f"in {args.steps} steps")
        print(f"  now at {current[index]:+.4f}, settle {args.settle:.1f} s per point")
        print(f"  {'return sweep: no (--one-way)' if args.one_way else 'return sweep: yes'}"
              f"  -> {args.steps if args.one_way else args.steps * 2} moves total")
        print("\n  Every other joint holds where it is. Check that this joint has "
              "clear\n  travel across the whole range before running it.")

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
            return 0

        print()
        driver.set_arm_modes(trossen_arm.Mode.position)
        try:
            rows = sweep(driver, index, targets, args.settle, "up")
            if not args.one_way and rows:
                rows += sweep(driver, index, list(reversed(targets)),
                              args.settle, "down")
        except KeyboardInterrupt:
            print("\n  interrupted -- the arm holds where it is")
            rows = []

        if not rows:
            print("\n  no measurements taken.", file=sys.stderr)
            return 4
        report(index, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
