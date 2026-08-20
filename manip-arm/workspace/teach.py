#!/usr/bin/env python3
"""Float a WXAI arm under gravity compensation so you can hand-guide it, and
save poses from where you put it.

    ./teach.py                 # float the arm, prompt for pose names
    ./teach.py --arm-name arm-2

This is the manipulator equivalent of driving the middle arm around with
teleop_keyboard.py before `pose.py save`, and it exists as one script rather
than two because only one process may hold the arm's connection at a time. A
separate `pose.py save` while this is running would be refused, so the saving
happens here.

How the float works: every arm joint goes into external_effort mode with a
commanded effort of zero. The controller keeps applying its gravity and
friction compensation, and your zero rides on top -- so the arm holds itself up
and moves freely when you push it. It is not torque-off; nothing is limp.

TWO THINGS TO KNOW BEFORE RUNNING IT:

  * Compensation is only as good as the end-effector model. If the real payload
    differs from ARM_EE (a camera, a custom finger, anything bolted to the
    flange), the arm will drift up or sag. Have a hand on it the first time.
  * The gripper is deliberately left idle -- braked -- rather than floated, so
    the fingers do not fall open and drop whatever is in them.

On exit, every joint returns to idle: braked, holding the position you left it
in. The arm does not sag when this script stops.
"""
import sys

import trossen_arm

import arm
import arm_config as cfg
import pose as pose_tool


def main():
    ap = arm.parser(__doc__)
    arm.add_common_args(ap)
    ap.add_argument("--arm-name", default=cfg.ARM_NAME,
                    help=f"whose pose library to save into (default {cfg.ARM_NAME})")
    ap.add_argument("--with-gripper", action="store_true",
                    help="float the gripper too (it will open under its own weight)")
    args = ap.parse_args()

    with arm.connect(args) as driver:
        print(f"  {cfg.ARM_NAME} at {args.ip}, end effector {args.end_effector}\n")
        print("  About to float the arm. Support it if the payload does not match\n"
              "  the end-effector model above -- it will drift if the model is wrong.")
        if input("  Type 'float' to continue: ").strip() != "float":
            print("  cancelled; nothing changed.")
            return 0

        try:
            driver.set_arm_modes(trossen_arm.Mode.external_effort)
            driver.set_arm_external_efforts([0.0] * cfg.NUM_ARM_JOINTS, 0.0, False)
        except Exception as e:
            print(f"\n  could not start gravity compensation: {e}", file=sys.stderr)
            return 4

        if args.with_gripper:
            driver.set_gripper_mode(trossen_arm.Mode.external_effort)
            driver.set_gripper_external_effort(0.0, 0.0, False)

        print("\n  floating. Move the arm by hand.\n"
              "  Type a name + Enter to save this pose, 'p' to print it, "
              "or Enter alone to quit.\n")
        try:
            while True:
                name = input("  pose name> ").strip()
                positions = list(driver.get_all_positions())
                if not name:
                    break
                if name == "p":
                    for i, v in enumerate(positions):
                        print(f"    {cfg.label(i):<14}{arm.fmt(i, v)}")
                    continue
                vals = pose_tool.save_current(args.arm_name, name, positions)
                print(f"    saved {name!r} for {args.arm_name}: "
                      + ", ".join(f"{v:.4f}" for v in vals))
        except (KeyboardInterrupt, EOFError):
            print()

        print("  returning to idle -- braked, holding this position.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
