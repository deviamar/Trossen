"""Shared connection and formatting helpers for the manipulator CLIs.

Everything in this directory is a small one-shot script: connect, do one thing,
disconnect. That works here for a reason worth stating, because it is the
opposite of what the DYNAMIXEL arms do.

On the middle arm the driver node owns the arm for as long as it runs, and
killing the launch drops torque. There is no equivalent process here. The
controller is powered whenever the arm's supply is on, it is always in some
mode, and your script is only a client of it: when the connection drops, the
controller falls back to `idle` on its own.

`idle` is a hold, not an off. The docs call it "braked", but it is a
torque-capped PID holding the joint at its current position (the changelog
tunes idle's imax per joint so the arm can hold itself extended), which means
two things: a script exiting leaves the arm parked rather than collapsing, and
the hold is soft enough to push through by hand. The only true power-off is the
arm's power supply.

The corollary: anything that has to keep *applying* something -- gravity
compensation in teach.py, a live squeeze from `gripper.py grasp --hold` -- only
lasts while its process is alive. That is why those two are the scripts that
block instead of returning.

Only one driver may be connected to an arm at a time. "Connection refused" or a
configure() timeout while another script is running is that, not a network
fault.
"""
import argparse
import contextlib
import math
import sys

import trossen_arm

import arm_config as cfg


def add_common_args(ap):
    """--ip / --model / --end-effector / --clear-error, for every CLI here."""
    ap.add_argument("--ip", default=cfg.ARM_IP,
                    help=f"arm controller IP (default from ARM_IP: {cfg.ARM_IP})")
    ap.add_argument("--model", default=cfg.ARM_MODEL, help=f"default {cfg.ARM_MODEL}")
    ap.add_argument("--end-effector", default=cfg.ARM_EE, help=f"default {cfg.ARM_EE}")
    # Deliberately opt-in. An arm sitting in an error state has already stopped
    # itself for a reason -- a joint limit, an overheat, a following error --
    # and clearing that on every connect would hide the reason from you while
    # letting the next command through. Read the message, then pass the flag.
    ap.add_argument("--clear-error", action="store_true",
                    help="clear a latched controller error on connect")
    return ap


@contextlib.contextmanager
def connect(args=None, ip=None, clear_error=False, timeout=20.0, overrides=True):
    """Configured driver, cleaned up on the way out.

    cleanup() sets every joint to idle: braked, holding position. It does not
    move the arm and it does not release it.

    cfg.JOINT_LIMIT_OVERRIDES is applied here unless overrides=False. Pass
    False from anything whose job is to report what the CONTROLLER holds rather
    than what this repo wants it to hold -- check-config.py's audit and backup,
    for instance, would otherwise describe our own overrides as the arm's
    configuration and write them into the backup file.
    """
    ip = ip or (args.ip if args else cfg.ARM_IP)
    model = cfg.model_enum(args.model if args else None)
    ee = cfg.end_effector(args.end_effector if args else None)
    clear = clear_error or bool(args and getattr(args, "clear_error", False))

    driver = trossen_arm.TrossenArmDriver()
    try:
        driver.configure(model, ee, ip, clear, timeout)
    except Exception as e:
        raise SystemExit(
            f"could not connect to the arm at {ip}: {e}\n"
            "  - another script still holding the connection? only one at a time\n"
            "  - arm powered and on the switch?   ./discover-arms.py\n"
            "  - host NIC up with 192.168.1.x?    ip -br addr show enp0s31f6\n"
            "  - latched error state? re-run with --clear-error once you have\n"
            "    read the controller's message above"
        )

    if (err := driver.get_error_information()):
        print(f"  note: controller reports {err!r}"
              f"{' (cleared)' if clear else ''}", file=sys.stderr)

    if overrides:
        apply_limit_overrides(driver)

    try:
        yield driver
    finally:
        driver.cleanup()


def apply_limit_overrides(driver, overrides=None):
    """Re-assert cfg.JOINT_LIMIT_OVERRIDES on a freshly configured driver.

    Called from connect() rather than left to each script, because the reason
    it is needed applies to every one of them: joint limits are session state,
    so configure() has just wiped whatever the last connection set back to the
    vendor defaults.

    The write is read back instead of assumed. A controller that clamps or
    ignores it leaves the arm in exactly the state the override exists to get
    out of, and the readback is the only place that shows the difference --
    otherwise the next command faults and looks like a fresh problem.
    """
    overrides = cfg.JOINT_LIMIT_OVERRIDES if overrides is None else overrides
    if not overrides:
        return

    lims = driver.get_joint_limits()
    for i, (lo, hi) in overrides.items():
        lims[i].position_min = lo
        lims[i].position_max = hi

    try:
        driver.set_joint_limits(lims)
    except Exception as e:
        print(f"  WARNING: could not apply the joint limit overrides: {e}\n"
              "  Running on vendor defaults -- see JOINT_LIMIT_OVERRIDES in "
              "arm_config.py.", file=sys.stderr)
        return

    applied = [(l.position_min, l.position_max) for l in driver.get_joint_limits()]
    for i, (lo, hi) in overrides.items():
        if abs(applied[i][0] - lo) > 1e-4 or abs(applied[i][1] - hi) > 1e-4:
            print(f"  WARNING: {cfg.label(i)} limits were not accepted -- "
                  f"asked for [{lo:.4f}, {hi:.4f}], controller reports "
                  f"[{applied[i][0]:.4f}, {applied[i][1]:.4f}].", file=sys.stderr)


def limits(driver=None):
    """[(lo, hi, tolerance)] per joint -- live from the arm, or vendor defaults.

    Two different windows come out of these numbers, and conflating them is how
    you end up debugging the wrong thing:

      * COMMANDS are clipped to [lo, hi].
      * FEEDBACK errors out beyond [lo - tolerance, hi + tolerance].

    So a joint sitting outside the second window faults the arm the moment it
    leaves idle, no matter what you command -- see blocked_by_position().
    """
    if driver is None:
        return [cfg.JOINT_LIMIT_OVERRIDES.get(i, (lo, hi)) + (0.2,)
                for i, (lo, hi) in enumerate(cfg.DEFAULT_JOINT_LIMITS)]
    return [(l.position_min, l.position_max, l.position_tolerance)
            for l in driver.get_joint_limits()]


def check(index, target, lims):
    """Error string for an out-of-range target, or None.

    Reports the remaining travel rather than just refusing: 'outside limits' on
    its own reads as "this joint will not move" when the truth is usually "your
    request overshot by 0.02".
    """
    lo, hi, _ = lims[index]
    if lo <= target <= hi:
        return None
    return (f"{cfg.label(index)}: target {target:.4f} is outside "
            f"[{lo:.4f}, {hi:.4f}] -- refusing")


def blocked_by_position(current, lims):
    """Joints already parked outside the controller's feedback window.

    Limit checks are skipped in idle, so an arm can sit like this indefinitely
    and read out fine. The instant any joint leaves idle, the controller sees
    the out-of-range feedback, raises Joint Limit Exceeded, and drops
    everything back to idle -- before your target is ever considered.

    Returns [(index, position, lo, hi, tol)].
    """
    bad = []
    for i, p in enumerate(current[:len(lims)]):
        lo, hi, tol = lims[i]
        if not (lo - tol <= p <= hi + tol):
            bad.append((i, p, lo, hi, tol))
    return bad


def explain_blocked(bad):
    """The message for a blocked arm, with the way out."""
    lines = ["  REFUSING: the arm is parked outside its own limits, so any move "
             "would fault it:"]
    for i, p, lo, hi, tol in bad:
        lines.append(f"    {cfg.label(i)} reads {p:.4f}, outside "
                     f"[{lo:.4f}, {hi:.4f}] +/- {tol:.2f} tolerance")
    lines += [
        "",
        "  The target is not the problem -- the controller checks the joint's",
        "  PRESENT position when it leaves idle, and faults before it reads your",
        "  command. It has been sitting there quietly because idle skips the check.",
        "",
        "  Recover with:  ./recover.py            (dry run, shows the plan)",
        "                 ./recover.py --execute  (widens the limit just enough,",
        "                                          walks the joint back in, restores)",
    ]
    return "\n".join(lines)


def uncovered_by_limits(positions, lims, indices=None):
    """Joints whose value falls outside the limits currently in force.

    `lims` is expected to come from limits(driver), so cfg.JOINT_LIMIT_OVERRIDES
    is already folded in: anything this returns is a position the overrides do
    NOT cover, not merely one the vendor defaults refuse.

    Returns [(index, value, lo, hi)].
    """
    indices = range(min(len(positions), len(lims))) if indices is None else indices
    out = []
    for i in indices:
        lo, hi, _ = lims[i]
        if not (lo <= positions[i] <= hi):
            out.append((i, positions[i], lo, hi))
    return out


def explain_uncovered(bad, what="this pose"):
    """Message for positions the overrides do not cover, with the edit to make.

    Worth saying at SAVE time rather than leaving to recall: a pose stored
    outside the limits in force is one that check() refuses every time it is
    recalled, and the failure surfaces later, somewhere else, as a pose that
    "does not work" rather than as a limit that was never widened.
    """
    lines = [f"  WARNING: {what} is outside the joint limits currently in force,",
             "  so recalling it will be refused:"]
    suggest = {}
    for i, v, lo, hi in bad:
        lines.append(f"    {cfg.label(i)} at {v:.4f}, limits [{lo:.4f}, {hi:.4f}]")
        # Round outward so the suggested bound clears the value rather than
        # landing exactly on it, and keep the untouched side as it stands.
        margin = 0.005 if i == cfg.GRIPPER_INDEX else 0.25
        nd = 3 if i == cfg.GRIPPER_INDEX else 2
        scale = 10 ** nd
        new_lo = math.floor((v - margin) * scale) / scale if v < lo else lo
        new_hi = math.ceil((v + margin) * scale) / scale if v > hi else hi
        suggest[i] = (new_lo, new_hi)
    lines += ["", "  To make it reachable, widen JOINT_LIMIT_OVERRIDES in "
                  "arm_config.py:"]
    for i, (lo, hi) in suggest.items():
        key = "GRIPPER_INDEX" if i == cfg.GRIPPER_INDEX else str(i)
        lines.append(f"    {key}: ({lo}, {hi}),")
    lines += ["", "  Look at the arm before you do. Widening is right when the joint "
                  "really is",
              "  where it says it is; it is wrong when the reading is off, and it "
              "removes",
              "  the only software check on that joint."]
    return "\n".join(lines)


def verify(driver, target, indices, tol=0.05):
    """Report what actually happened, instead of assuming the command landed.

    set_arm_positions() can return normally while the controller faults in its
    daemon thread, so 'the call returned' is not evidence the arm moved. Read
    the state back and say so.
    """
    try:
        err = driver.get_error_information()
        pos = list(driver.get_all_positions())
    except Exception as e:
        print(f"  MOVE FAILED -- lost contact with the controller: {e}\n"
              "  Reconnect with --clear-error and re-read with ./read_joints.py",
              file=sys.stderr)
        return False

    if err and err != "No error":
        print(f"  MOVE FAILED -- controller reports {err!r}.\n"
              "  Every joint is idle (braked, holding). Read the log line above:\n"
              "  it names the joint and the offending value.", file=sys.stderr)
        return False

    off = [(i, pos[i], target[i]) for i in indices if abs(pos[i] - target[i]) > tol]
    if off:
        print("  MOVE INCOMPLETE -- these joints did not reach their target:",
              file=sys.stderr)
        for i, p, t in off:
            print(f"    {cfg.label(i):<14}at {p:.4f}, wanted {t:.4f}",
                  file=sys.stderr)
        return False

    print("  done.")
    return True


def fmt(index, value):
    """One joint's value, in its own units."""
    if index == cfg.GRIPPER_INDEX:
        return f"{value:>9.4f} m  ({value * 1000:>6.1f} mm)"
    return f"{value:>9.4f} rad ({math.degrees(value):>7.2f} deg)"


def move_table(current, target, indices):
    """The current/target/delta table every commanding script prints."""
    rows = [f"  {'joint':<14}{'current':>10}{'target':>10}{'delta':>10}",
            "  " + "-" * 44]
    for i in indices:
        rows.append(f"  {cfg.label(i):<14}{current[i]:>10.4f}"
                    f"{target[i]:>10.4f}{target[i] - current[i]:>+10.4f}")
    return "\n".join(rows)


def confirm_or_dry_run(args, goal_time):
    """True if the caller should actually send. Prints the dry-run notice if not."""
    if args.execute:
        print(f"\n  moving over {goal_time:.1f} s ...")
        return True
    print(f"\n  DRY RUN -- nothing sent. Would move over {goal_time:.1f} s.\n"
          "  Re-run with --execute to move the arm.")
    return False


def parser(doc):
    return argparse.ArgumentParser(
        description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
