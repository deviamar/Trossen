#!/usr/bin/env python3
"""Reboot a WXAI arm controller. Clears a latched error without the power switch.

    ./reboot-controller.py --execute

This is the software equivalent of power-cycling the controller, and the
vendor's recommended way to clear an error state. What it does NOT do is move
the arm: the joints stay exactly where they are, and every configuration that
lives outside EEPROM -- joint limits, modes, motor parameters -- comes back at
its default. If the arm is parked outside its limits, it still is after this,
and ./recover.py is still what gets it out.

There is nothing else to shut down. Unlike the middle arm, which runs a
persistent xs_sdk node you launch and kill, every script in this directory is
one-shot: it connects, acts, and disconnects. The container running is not the
same as the arm being held -- only a running script holds it. So there is no
"launch" to terminate, and restarting the container changes nothing about the
arm's state.

Takes about 10 seconds to come back. ./discover-arms.py confirms it is up.
"""
import sys
import time

import arm


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("--execute", action="store_true", help="actually reboot")
    args = ap.parse_args()

    if not args.execute:
        print(f"  DRY RUN -- would reboot the controller at {args.ip}.\n"
              "  The arm does not move. Non-EEPROM settings return to defaults.\n"
              "  Re-run with --execute.")
        return 0

    with arm.connect(args, clear_error=True) as driver:
        print("  positions before: "
              + ", ".join(f"{p:.4f}" for p in driver.get_all_positions()))
        print("  rebooting ...")
        try:
            driver.reboot_controller()
        except Exception as e:
            # The connection drops as the controller goes down, so an exception
            # here is expected rather than a failure. Say so instead of looking
            # like a crash.
            print(f"  connection dropped during reboot ({type(e).__name__}) "
                  "-- expected.")

    time.sleep(10)
    print("  should be back up. Check with:  ./discover-arms.py")
    print("  then read the state with:       ./read_joints.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
