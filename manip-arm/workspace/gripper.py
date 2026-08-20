#!/usr/bin/env python3
"""Open, close, and grasp with a WXAI gripper.

    ./gripper.py status                  # opening in mm, load in N
    ./gripper.py open                    # force-based, opens until the stop
    ./gripper.py close                   # force-based, closes until it meets something
    ./gripper.py grasp --force 40        # close with a specific squeeze
    ./gripper.py set 0.02                # position control, metres (20 mm)
    ./gripper.py set 20 --mm             # the same, in mm
    ./gripper.py grasp --hold            # keep squeezing until Ctrl-C
    ./gripper.py release                 # gripper to idle: braked, holding

The ROS 1 version of this on the DYNAMIXEL arms had to fake force control:
drop the motor's Current_Limit register to ~100, switch it to
current_based_position mode, then command a position past the object and let the
current cap stall it. Here that is the native interface. `external_effort` mode
takes newtons directly -- positive opens, negative closes -- so `close` means
"squeeze with N newtons and stop wherever the object is", with no register
writes, no reboot-the-motor recovery, and no position command that lies about
where you want the finger to end up.

Position mode (`set`) is still there for when you want a known opening rather
than a known force -- staging the fingers before a grasp, mostly. Closing on an
object in position mode is the one thing to avoid: the finger cannot reach the
commanded position, and the controller eventually calls that a following error
and drops the whole arm to idle.

WHAT SURVIVES THE SCRIPT EXITING: the opening does, the force does not. On
disconnect every joint goes to idle, which on this arm is braked and holding
position -- so a grasped object stays grasped by the brake. If you need the
squeeze itself maintained (a compliant hold on something soft or slipping), use
--hold, which keeps the process and the connection alive.
"""
import sys
import time

import trossen_arm

import arm
import arm_config as cfg


def show(driver):
    pos = driver.get_gripper_position()
    print(f"  opening      {pos:.4f} m  ({pos * 1000:.1f} mm)")
    print(f"  external eff {driver.get_gripper_external_effort():+.2f} N")
    print(f"  velocity     {driver.get_gripper_velocity():+.4f} m/s")


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("cmd", choices=["status", "open", "close", "grasp", "set", "release"])
    ap.add_argument("value", nargs="?", type=float, help="target opening for 'set'")
    ap.add_argument("--mm", action="store_true", help="'set' value is millimetres")
    ap.add_argument("--force", type=float, default=cfg.GRASP_FORCE_N,
                    help=f"newtons for open/close/grasp (default {cfg.GRASP_FORCE_N})")
    ap.add_argument("--hold", action="store_true",
                    help="keep applying the force until Ctrl-C (else it becomes a brake hold)")
    ap.add_argument("--goal-time", type=float, default=1.0,
                    help="seconds to ramp the command (default 1.0)")
    ap.add_argument("--execute", action="store_true", help="actually send it")
    args = ap.parse_args()

    if args.cmd == "set" and args.value is None:
        ap.error("'set' needs a target opening")
    if abs(args.force) > cfg.GRASP_FORCE_MAX_N:
        ap.error(f"--force above {cfg.GRASP_FORCE_MAX_N} N is past what the "
                 "gripper's effort limit allows; the controller will fault")

    with arm.connect(args) as driver:
        print(f"  {cfg.ARM_NAME} at {args.ip}\n")
        show(driver)

        if args.cmd == "status":
            return 0

        if args.cmd == "release":
            if not args.execute:
                print("\n  DRY RUN -- would set the gripper to idle (braked, holding).")
                return 0
            driver.set_gripper_mode(trossen_arm.Mode.idle)
            print("\n  gripper idle: braked and holding its current opening.")
            return 0

        if args.cmd == "set":
            target = args.value / 1000.0 if args.mm else args.value
            lims = arm.limits(driver)
            if err := arm.check(cfg.GRIPPER_INDEX, target, lims):
                print("\n" + err, file=sys.stderr)
                return 3
            print(f"\n  set opening -> {target:.4f} m ({target * 1000:.1f} mm), "
                  f"over {args.goal_time:.1f} s")
            if not args.execute:
                print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
                return 0
            try:
                driver.set_gripper_mode(trossen_arm.Mode.position)
                driver.set_gripper_position(float(target), args.goal_time, True)
            except Exception as e:
                print(f"\n  the controller rejected it: {e}\n"
                      "  closing on an object in position mode does this -- use "
                      "'grasp' instead.", file=sys.stderr)
                return 4
            show(driver)
            return 0

        # open / close / grasp: force control. Sign is the whole interface.
        effort = abs(args.force) if args.cmd == "open" else -abs(args.force)
        verb = "open" if args.cmd == "open" else "close on whatever it meets"
        print(f"\n  {verb} with {effort:+.1f} N, ramped over {args.goal_time:.1f} s")
        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute.")
            return 0

        try:
            driver.set_gripper_mode(trossen_arm.Mode.external_effort)
            # blocking=True returns when the commanded effort is reached, which
            # is not the same as the finger having stopped moving -- so give it
            # a moment before reading back an opening that is still changing.
            driver.set_gripper_external_effort(float(effort), args.goal_time, True)
            time.sleep(0.5)
            show(driver)

            if args.hold:
                print("\n  holding the force. Ctrl-C to stop (the opening then "
                      "stays, braked).")
                while True:
                    time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n  released the force; the gripper brakes where it is.")
        except Exception as e:
            print(f"\n  the controller rejected it: {e}", file=sys.stderr)
            return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
