#!/usr/bin/env python3
"""Move the SLATE base a measured distance or angle. DRY RUN BY DEFAULT.

    ./move_base.py forward 0.5             # 0.5 m ahead -- prints the plan only
    ./move_base.py forward 0.5 --execute   # actually move
    ./move_base.py forward -0.3 --execute  # negative = backwards
    ./move_base.py turn 90 --deg --execute # positive = counter-clockwise (left)
    ./move_base.py vel 0.2 0.0 --for 3 --execute   # raw velocity for 3 s
    ./move_base.py stop                    # zero it, no --execute needed

Needs ./launch-base.sh running in another shell.

CLOSED LOOP ON ODOMETRY, not on a stopwatch. `forward` and `turn` watch /odom
and stop when the distance is covered, rather than driving at v for d/v seconds
-- open-loop timing is wrong by whatever the acceleration ramp and the load cost
you, and it is wrong in the direction of overshoot. A hard timeout still bounds
the move, so a base that is blocked, E-stopped, or reporting frozen odometry
stops instead of pushing indefinitely.

Wheel odometry drifts, so "0.5 m" is 0.5 m as the wheels count it. On a clean
floor that is close; on carpet, over a threshold, or with a wheel slipping it is
not. This is a convenience for repeatable nudges, not a positioning system.

`stop` needs no --execute. Refusing to stop until a flag is passed would be the
wrong default for the one command whose whole job is to make the robot safe.
"""
import argparse
import math
import sys

import rclpy
from nav_msgs.msg import Odometry

import base_config as cfg
import slate


class Mover(slate.VelocityDriver):
    """VelocityDriver that also watches odom, so a move can close the loop."""

    def __init__(self):
        super().__init__("slate_move_base")
        self.odom = None
        self.create_subscription(Odometry, cfg.TOPIC_ODOM, self._odom_cb, 1)

    def _odom_cb(self, msg):
        self.odom = msg


def travelled(start, now):
    """Straight-line metres between two Odometry samples."""
    return math.hypot(now.pose.pose.position.x - start.pose.pose.position.x,
                      now.pose.pose.position.y - start.pose.pose.position.y)


def turned(start, now):
    """Unwrapped radians between two Odometry samples' yaw.

    Accumulated across calls by the caller, because a single wrapped difference
    cannot tell 350 deg from -10 deg -- and a turn command is perfectly entitled
    to exceed pi.
    """
    d = slate.yaw_of(now) - slate.yaw_of(start)
    return math.atan2(math.sin(d), math.cos(d))


def run_closed_loop(node, linear, angular, target, measure, unit):
    """Drive until `measure` reaches `target`. Returns what it actually got to.

    The timeout is generous (2x the ideal time, plus 2 s) because the ideal time
    ignores acceleration entirely. It exists to bound a move that is not
    progressing at all -- blocked wheels, an engaged E-stop, a driver that has
    stopped updating odometry -- not to be the thing that normally ends the move.
    """
    period = 1.0 / cfg.PUBLISH_HZ
    speed = abs(linear) if linear else abs(angular)
    timeout_s = (abs(target) / speed) * 2.0 + 2.0

    start = node.odom
    t0 = node.get_clock().now()
    progress = 0.0
    last = start

    try:
        while rclpy.ok():
            node.send(linear, angular)
            rclpy.spin_once(node, timeout_sec=period)

            elapsed = (node.get_clock().now() - t0).nanoseconds / 1e9
            if measure is turned:
                progress += turned(last, node.odom)
                last = node.odom
            else:
                progress = measure(start, node.odom)

            done = abs(progress) >= abs(target)
            sys.stdout.write(f"\r    {progress:+.3f} / {target:+.3f} {unit}"
                             f"   ({elapsed:.1f}s)   ")
            sys.stdout.flush()

            if done:
                break
            if elapsed > timeout_s:
                print(f"\n  TIMED OUT after {elapsed:.1f}s at {progress:+.3f} "
                      f"{unit} of {target:+.3f}.", file=sys.stderr)
                print("  The base stopped. Blocked wheels, an engaged E-stop, or\n"
                      "  odometry that is not updating -- check ./read_base.py.",
                      file=sys.stderr)
                break
    except KeyboardInterrupt:
        node.stop()
        print("\n  interrupted -- base stopped.", file=sys.stderr)
        raise
    finally:
        node.stop()

    print()
    return progress


def main():
    ap = slate.parser(__doc__)

    # --execute and --speed hang off a parent parser rather than the top-level
    # one so that they are accepted AFTER the subcommand. argparse only matches
    # a parent parser's options before the subcommand name, which would make
    # the documented `./move_base.py forward 0.5 --execute` an error -- and the
    # failure mode for the one flag that gates movement should not be "argparse
    # rejects it", nor anything that could be mistaken for it having applied.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--execute", action="store_true",
                        help="actually move (everything is a dry run without it)")
    common.add_argument("--speed", type=float, default=None,
                        help="m/s for forward, rad/s for turn "
                             f"(defaults: {cfg.CLAMP_VEL_X / 2} / {cfg.CLAMP_VEL_Z / 2})")

    sub = ap.add_subparsers(dest="cmd", required=True)

    p_fwd = sub.add_parser("forward", parents=[common], help="drive straight, metres")
    p_fwd.add_argument("distance", type=float)

    p_turn = sub.add_parser("turn", parents=[common],
                            help="turn in place, radians (or --deg)")
    p_turn.add_argument("angle", type=float)
    p_turn.add_argument("--deg", action="store_true", help="angle is in degrees")

    p_vel = sub.add_parser("vel", parents=[common],
                           help="hold a raw velocity for a fixed time")
    p_vel.add_argument("linear", type=float, help="m/s")
    p_vel.add_argument("angular", type=float, help="rad/s")
    p_vel.add_argument("--for", dest="duration", type=float, default=1.0,
                       help="seconds (default 1.0)")

    sub.add_parser("stop", help="publish zero velocity (no --execute needed)")

    args = ap.parse_args()

    # Caught here rather than in run_closed_loop, where it divides by the speed
    # to size the timeout. A zero speed is a move that never arrives, and the
    # honest answer is to reject it rather than to drive at zero for a computed
    # eternity.
    if getattr(args, "speed", None) == 0:
        print("  --speed 0 would never get there. Pick a speed above zero.",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = Mover()
    try:
        # `stop` deliberately skips every check below. It must work even when
        # the driver is misbehaving -- that is when it is most needed.
        if args.cmd == "stop":
            node.stop()
            print("  stop published.")
            return 0

        if args.cmd == "forward":
            linear = args.speed if args.speed is not None else cfg.CLAMP_VEL_X / 2
            linear = math.copysign(abs(linear), args.distance)
            angular = 0.0
            measure, unit = travelled, "m"
            # travelled() is a straight-line distance and cannot go negative, so
            # a backwards move is tracked on magnitude alone -- the direction
            # lives in the sign of `linear`.
            target = abs(args.distance)
        elif args.cmd == "turn":
            angle = math.radians(args.angle) if args.deg else args.angle
            angular = args.speed if args.speed is not None else cfg.CLAMP_VEL_Z / 2
            angular = math.copysign(abs(angular), angle)
            linear, target = 0.0, angle
            measure, unit = turned, "rad"
        else:  # vel
            linear, angular = args.linear, args.angular

        linear, angular, was_clamped = cfg.clamp(linear, angular)
        if was_clamped:
            print(f"  NOTE: clamped to {linear:+.3f} m/s / {angular:+.3f} rad/s "
                  f"(limits {cfg.CLAMP_VEL_X} / {cfg.CLAMP_VEL_Z} in "
                  "base_config.py).\n"
                  "  These are this repo's limits, not the base's -- the driver's\n"
                  "  own clamp is not in the /cmd_vel path at all.")

        # Built AFTER the clamp, so the plan quotes the speed that will actually
        # be sent. Describing the requested speed here would be worse than
        # useless: the dry run exists to be read and approved, and one that
        # advertises 5 m/s for a move that will go out at 0.3 is telling you
        # about a command that no longer exists.
        if args.cmd == "forward":
            desc = (f"drive {args.distance:+.3f} m at {abs(linear):.2f} m/s, "
                    "stopping on odometry.")
        elif args.cmd == "turn":
            desc = (f"turn {math.degrees(angle):+.1f} deg at "
                    f"{abs(angular):.2f} rad/s, stopping on odometry.")
        else:
            desc = (f"hold linear={linear:+.3f} m/s angular={angular:+.3f} rad/s "
                    f"for {args.duration:.1f} s.")

        if not slate.confirm_or_dry_run(args, desc):
            return 0

        # Both checks only matter once we are actually sending. A dry run is
        # useful with no driver running at all.
        if not slate.wait_for_subscriber(node):
            print(f"\n  Nothing is subscribed to {cfg.TOPIC_CMD_VEL_TELEOP} --\n"
                  "  the GOVERNOR is not running, so this would publish into the\n"
                  "  void. Commands go through it to be clamped; it is not\n"
                  "  optional and it is not the driver.\n"
                  "      ./governor.py          (or set AUTOSTART=true)\n"
                  "  If the governor IS up, check the driver under it:\n"
                  "      cat ~/workspace/driver.log",
                  file=sys.stderr)
            return 1

        if args.cmd == "vel":
            node.drive_for(linear, angular, args.duration)
            print(f"  held for {args.duration:.1f} s, then stopped.")
            return 0

        if slate.wait_for_odom(node) is None:
            print("\n" + slate.no_driver_message(cfg.TOPIC_ODOM), file=sys.stderr)
            return 1

        estop = slate.estop_from_odom(node.odom)
        if estop:
            print("\n  REFUSING: the base reports EMERGENCY STOP engaged.\n"
                  "  Release the E-stop, then re-run.", file=sys.stderr)
            return 1

        got = run_closed_loop(node, linear, angular, target, measure, unit)
        print(f"  done -- {got:+.3f} {unit} of {target:+.3f} {unit} requested.")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
