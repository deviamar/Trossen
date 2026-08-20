#!/usr/bin/env python3
"""Read joint state from a WXAI manipulator. Read-only -- commands nothing.

    ./read_joints.py              # one snapshot: position, velocity, ext. effort
    ./read_joints.py --watch      # live table until Ctrl-C
    ./read_joints.py --temps      # add driver / rotor temperatures
    ./read_joints.py --raw        # one line of bare numbers, for piping
    ./read_joints.py --ip 192.168.1.3

Positions are radians for joints 0-5 and METRES for the gripper. External
effort is the load the joint is feeling on top of gravity and friction
compensation -- a non-zero value on a stationary arm means something is pushing
on it (or the end-effector model is wrong, which is the same reading from the
controller's point of view).

    ./read_joints.py --limits     # flag any joint sitting outside its range

Connecting does not change what the arm is doing: configure() leaves every
joint idle, and cleanup() puts it back the same way. Reading is safe at any
time -- but only one process may hold the connection, so this will not run
while teach.py or a --hold grasp is up.

--watch --limits is the tool for hand-posing an arm back into a legal
configuration. Idle holds with a torque-capped PID rather than a mechanical
brake, so the arm can be pushed by hand, and this shows you live whether each
joint has come back inside the range the controller will accept.
"""
import sys
import time

import arm
import arm_config as cfg


def snapshot(driver, temps=False, show_limits=False):
    pos = list(driver.get_all_positions())
    vel = list(driver.get_all_velocities())
    ext = list(driver.get_all_external_efforts())
    dtemp = list(driver.get_all_driver_temperatures()) if temps else None
    rtemp = list(driver.get_all_rotor_temperatures()) if temps else None
    lims = arm.limits(driver) if show_limits else None

    head = f"  {'joint':<14}{'position':>10}{'':>12}{'vel':>9}{'ext eff':>10}"
    if temps:
        head += f"{'drv C':>8}{'rot C':>8}"
    if show_limits:
        head += f"  {'limits':<20}{'':<8}"
    rows = [head, "  " + "-" * (len(head) - 2)]

    for i, name in enumerate(cfg.DISPLAY_NAMES):
        if i == cfg.GRIPPER_INDEX:
            shown = f"{pos[i]:>10.4f}{'m (' + format(pos[i] * 1000, '.1f') + ' mm)':>12}"
        else:
            shown = f"{pos[i]:>10.4f}{'rad (' + format(pos[i] * 57.29578, '.1f') + ' deg)':>12}"
        row = f"  {name:<14}{shown}{vel[i]:>9.3f}{ext[i]:>10.3f}"
        if temps:
            row += f"{dtemp[i]:>8.1f}{rtemp[i]:>8.1f}"
        if show_limits:
            lo, hi, tol = lims[i]
            inside = lo - tol <= pos[i] <= hi + tol
            row += (f"  [{lo:>7.3f},{hi:>7.3f}]  "
                    f"{'ok' if inside else 'OUT OF RANGE'}")
        rows.append(row)
    return "\n".join(rows)


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("--watch", action="store_true", help="live table until Ctrl-C")
    ap.add_argument("--temps", action="store_true", help="add motor temperatures")
    # The column to watch while hand-posing an arm back into a legal
    # configuration: it says per joint whether the controller would accept the
    # position it is reading right now.
    ap.add_argument("--limits", action="store_true",
                    help="show each joint's range and flag anything out of it")
    ap.add_argument("--raw", action="store_true", help="one line of bare numbers")
    args = ap.parse_args()

    with arm.connect(args) as driver:
        try:
            if args.raw:
                print(" ".join(f"{p:.6f}" for p in driver.get_all_positions()))
            elif args.watch:
                print(f"{cfg.ARM_NAME} at {args.ip} -- Ctrl-C to stop.\n")
                lines = 0
                while True:
                    block = snapshot(driver, args.temps, args.limits)
                    if lines:  # redraw in place rather than scrolling
                        sys.stdout.write(f"\033[{lines}A")
                    lines = block.count("\n") + 1
                    print(block)
                    time.sleep(0.1)
            else:
                print(f"  {cfg.ARM_NAME} at {args.ip}\n")
                print(snapshot(driver, args.temps, args.limits))
        except KeyboardInterrupt:
            print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
