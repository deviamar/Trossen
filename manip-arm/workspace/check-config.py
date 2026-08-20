#!/usr/bin/env python3
"""Back up and audit an arm's stored configuration. Read-only on the arm.

    ./check-config.py                       # backup + full audit
    ./check-config.py --no-backup           # audit only
    ./check-config.py --backup-to /path.yaml

Run this before changing anything on a controller you suspect is misconfigured.
It writes the arm's live configuration to config/arm-config-<name>.yaml via the
SDK's own save_configs_to_file(), then reports what differs from the vendor
defaults and what looks suspicious.

WHAT IS AND IS NOT SAFE TO RESET
--------------------------------
Joint limits, modes, motor parameters and the end-effector model are ordinary
session configuration: wrong values there are harmless to overwrite, and they
already reset to defaults on every configure().

Joint characteristics are NOT. position_offset (homing error),
effort_correction, and the four friction terms are calibrated per arm at
manufacturing, stored in the controller's EEPROM, and different for every
unit -- the vendor's default_configurations_wxai_v0.yaml does NOT contain your
arm's values. So:

  * set_factory_reset_flag(True) wipes them at the next boot.
  * load_configs_from_file(<the vendor default file>) overwrites them.

Either one turns a possible calibration problem into a definite one. Neither is
something this script does, and neither should be a first move. Take the backup
this script writes before anyone tries it.

THE TEST THAT ACTUALLY DECIDES IT
---------------------------------
A joint limit complaint does not tell you whether the limits are wrong or the
arm is simply parked outside them -- and on this rig the limits read back as
exactly the vendor defaults, so they are not the thing that is off.

What decides it is the arm's zero. At all joints zero this arm is FOLDED, not
standing up: links 2 and 3 are 0.264 m and 0.245 m and run in opposite
directions, so they very nearly cancel and the tool sits about 0.25 m out and
0.16 m up -- compact, close to the base. That is what `sleep` means, and it is
the shape to compare against. (Earlier revisions of this file and the README
said "stands straight up". That was wrong, and it inverts the test below.)

Hand-pose the arm into the folded shape and run:

    ./read_joints.py --limits

If the joints read near 0, the calibration is fine and the only problem was
where the arm was parked. If a joint reads a long way off -- a clean fraction
of pi is the classic homing error -- that joint's position_offset is wrong, and
that is a Trossen support conversation, not something to fix by widening a
limit.

The pi-fraction test below is applied to position_offset, which is 0 from the
factory on every joint and therefore says nothing on its own. Apply the same
eye to the POSITIONS in the limits table: a joint resting at a clean fraction
of pi from where the shape in front of you says it should be is the same
homing error, seen from the other side.
"""
import os
import sys

import arm
import arm_config as cfg

BACKUP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")

# Suspicious-looking offsets: a homing error is usually a clean fraction of pi,
# which is what makes it recognisable at a glance.
NOTABLE_FRACTIONS = {
    "pi": 3.141593, "pi/2": 1.570796, "pi/3": 1.047198,
    "pi/4": 0.785398, "pi/6": 0.523599,
}


def near_fraction(value, tol=0.05):
    """Name of the pi fraction this value is close to, or None."""
    for name, v in NOTABLE_FRACTIONS.items():
        if abs(abs(value) - v) <= tol:
            return ("-" if value < 0 else "+") + name
    return None


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("--no-backup", action="store_true", help="skip the config backup")
    ap.add_argument("--backup-to", help="where to write the backup YAML")
    args = ap.parse_args()

    # overrides=False: this script audits and backs up what the CONTROLLER
    # holds. With the overrides applied it would report arm_config.py's
    # JOINT_LIMIT_OVERRIDES as the arm's own configuration and bake them into
    # the backup, which is the one file you would reach for to find out what
    # the arm had before anyone started changing things.
    with arm.connect(args, overrides=False) as driver:
        print(f"  {cfg.ARM_NAME} at {args.ip}\n")

        if not args.no_backup:
            path = args.backup_to or os.path.join(
                BACKUP_DIR, f"arm-config-{cfg.ARM_NAME}.yaml")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            try:
                driver.save_configs_to_file(path)
                print(f"  backup written: {path}\n")
            except Exception as e:
                print(f"  WARNING: could not write the backup: {e}\n", file=sys.stderr)

        print(f"  driver {driver.get_driver_version()}, "
              f"controller {driver.get_controller_version()}")
        print(f"  error state: {driver.get_error_information() or 'none'}")

        flag = driver.get_factory_reset_flag()
        print(f"  factory reset flag: {flag}"
              + ("   <-- set: the NEXT BOOT wipes this arm's calibration"
                 if flag else ""))

        print("\n  joint characteristics (EEPROM, calibrated per arm):")
        print(f"    {'joint':<10}{'pos offset':>12}{'eff corr':>10}"
              f"{'fric const':>12}   note")
        chars = driver.get_joint_characteristics()
        generic = 0
        for i, c in enumerate(chars):
            # position_offset 0 is the stock value for every joint, so it is
            # not by itself a sign of anything. A non-zero one near a fraction
            # of pi is worth a second look, since that is the shape a homing
            # error takes.
            note = ""
            if c.position_offset and (frac := near_fraction(c.position_offset)):
                note = f"~{frac}, looks like a homing error"
            if (abs(c.effort_correction - cfg.DEFAULT_EFFORT_CORRECTIONS[i]) < 1e-4
                    and abs(c.friction_constant_term
                            - cfg.DEFAULT_FRICTION_CONSTANTS[i]) < 1e-4):
                generic += 1
                note = (note + " generic values").strip()
            print(f"    {cfg.label(i):<14}{c.position_offset:>12.6f}"
                  f"{c.effort_correction:>10.4f}{c.friction_constant_term:>12.6f}"
                  f"   {note}")

        if generic == len(chars):
            print("\n    WARNING: every effort correction and friction term matches the\n"
                  "    vendor's generic defaults. Those are measured per arm at\n"
                  "    manufacturing, so this arm appears to have lost its own values --\n"
                  "    the EEPROM was reset. Contact Trossen for this arm's calibration.")
        else:
            print("\n    Calibration looks intact: the effort corrections and friction\n"
                  "    terms differ from the vendor's generic defaults, which is what a\n"
                  "    per-arm calibrated controller looks like. All-zero position\n"
                  "    offsets are normal -- 0 is the stock value for every joint.")

        print("\n  joint limits vs vendor defaults:")
        lims = arm.limits(driver)
        current = list(driver.get_all_positions())
        differs = False
        for i, (lo, hi, tol) in enumerate(lims):
            dlo, dhi = cfg.DEFAULT_JOINT_LIMITS[i]
            same = abs(lo - dlo) < 1e-4 and abs(hi - dhi) < 1e-4
            differs = differs or not same
            inside = lo - tol <= current[i] <= hi + tol
            print(f"    {cfg.label(i):<14}[{lo:>8.4f},{hi:>8.4f}] "
                  f"+/-{tol:.2f}   {'stock' if same else 'CHANGED from stock':<18}"
                  f"  now at {current[i]:>8.4f} "
                  f"{'ok' if inside else '<-- OUT OF RANGE'}")

        if not differs:
            print("\n    All limits are the vendor defaults -- so the limits are not\n"
                  "    what is misconfigured here. Anything flagged OUT OF RANGE is a\n"
                  "    joint parked where the controller will not accept it, which is\n"
                  "    either where the arm was left, or a wrong position offset above.")

        print("\n  end effector configured: " + args.end_effector)
        print("  (ARM_EE in docker-compose.yml -- it must match what is bolted on,\n"
              "   or gravity compensation and every force command are wrong)")

        print("\n  Decisive check: hand-pose the arm into the FOLDED shape (all-zeros\n"
              "  on this arm is folded and compact, NOT standing up -- links 2 and 3\n"
              "  cancel at zero), then run ./read_joints.py --limits. Near zero\n"
              "  everywhere means the calibration is fine and the joint was merely\n"
              "  parked out of range. A joint sitting a clean fraction of pi away from\n"
              "  zero in that shape is a homing error: fix its position_offset rather\n"
              "  than widening its limit. See this file's docstring before resetting\n"
              "  anything -- a factory reset destroys per-arm calibration.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
