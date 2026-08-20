#!/usr/bin/env python3
"""Drive the base forward for three seconds, then stop. The smoke test.

    ./drive_test.py                    # DRY RUN -- prints the plan, sends nothing
    ./drive_test.py --execute          # actually drive
    ./drive_test.py --execute --seconds 2 --speed 0.1
    ./drive_test.py --execute --turn   # rotate in place instead

The simplest possible end-to-end check: one command in, the base moves, it
stops. If this works the whole chain works -- DDS discovery, the governor, the
driver, the serial link, the motors -- and if it does not, the failure is
somewhere in a path short enough to read in one sitting.

    publishes  /slate/cmd_vel_teleop   geometry_msgs/Twist   at 20 Hz

It publishes to the TELEOP topic, not /slate/cmd_vel, so the governor clamps it
like any other input. Publishing to /slate/cmd_vel directly would bypass the one
thing standing between a typo and a collision, and would also fight the governor
for the topic.

BEFORE IT WILL DO ANYTHING:
  * the driver must be running        -- automatic with AUTOSTART=true
  * the governor must be running      -- likewise
  * the motors must be TORQUED        -- ./base_ctl.py torque on, in slate-base
The base accepts velocity commands with torque off and silently discards them,
so "nothing happened" is far more often torque than a broken pipeline. This
script checks what it can and says which it is.

STOPPING IS NOT OPTIONAL. The stop burst is published from a finally block, so
Ctrl-C mid-run stops the base rather than leaving the last setpoint standing for
the driver's 300 ms deadline to expire.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

TOPIC = "/slate/cmd_vel_teleop"
GOVERNOR_OUTPUT = "/slate/cmd_vel"
PUBLISH_HZ = 20.0


class DriveTest(Node):
    def __init__(self):
        super().__init__("drive_test")
        self.pub = self.create_publisher(Twist, TOPIC, 1)

    def preflight(self):
        """What we can check without moving anything."""
        names = dict(self.get_topic_names_and_types())
        problems = []
        if GOVERNOR_OUTPUT not in names:
            problems.append(
                f"{GOVERNOR_OUTPUT} does not exist -- the governor is not running.\n"
                "      In slate-base: ./governor.py   (or AUTOSTART=true)")
        else:
            info = self.get_publishers_info_by_topic(GOVERNOR_OUTPUT)
            subs = self.get_subscriptions_info_by_topic(GOVERNOR_OUTPUT)
            if not info:
                problems.append(f"{GOVERNOR_OUTPUT} has no publisher -- governor down.")
            if not subs:
                problems.append(
                    f"{GOVERNOR_OUTPUT} has no subscriber -- the base driver is not\n"
                    "      listening. In slate-base: ./launch-base.sh, then check\n"
                    "      workspace/driver.log for 'Failed to initialize base port'.")
        if "/slate/odom" not in names:
            problems.append("/slate/odom is missing -- the driver is not publishing.")

        # Another publisher on OUR topic is the trap this check exists for.
        # quest_teleop publishes a dead-man zero at 72 Hz whenever it is running,
        # including on the sim backend. The governor keeps the latest value it
        # saw on the topic, so 72 Hz of zeros beats 20 Hz of commands roughly
        # four times in five and the base barely twitches. Nothing errors; it
        # just does not move, which is the worst way for this to fail.
        others = sorted({
            e.node_name for e in self.get_publishers_info_by_topic(TOPIC)
            if e.node_name != self.get_name()
        })
        if others:
            problems.append(
                f"something else is already publishing {TOPIC}: "
                f"{', '.join(others)}.\n"
                "      Two sources on one command topic is a fight neither wins.\n"
                "      Stop it first, e.g.:  docker compose stop quest")
        return problems

    def send(self, lin, ang):
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub.publish(m)

    def run(self, lin, ang, seconds):
        """Hold a velocity, then stop. Velocity is a setpoint with a 300 ms
        deadline at the driver, so this must repeat -- one message would buy
        300 ms of motion, not `seconds` of it."""
        period = 1.0 / PUBLISH_HZ
        end = time.monotonic() + seconds
        try:
            while rclpy.ok() and time.monotonic() < end:
                self.send(lin, ang)
                rclpy.spin_once(self, timeout_sec=0.0)
                remaining = end - time.monotonic()
                print(f"\r  driving ... {remaining:4.1f} s left", end="", flush=True)
                time.sleep(period)
            print("\r  driving ... done.          ")
        finally:
            # Several, not one: a dropped stop on a depth-1 queue would leave
            # the base running until the deadline expired.
            for _ in range(10):
                self.send(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.01)
            print("  stopped.")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true", help="actually drive")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=0.15, help="m/s (or rad/s with --turn)")
    ap.add_argument("--turn", action="store_true", help="rotate in place instead")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    lin, ang = (0.0, args.speed) if args.turn else (args.speed, 0.0)

    rclpy.init()
    node = DriveTest()
    try:
        # Discovery is not instant; asking immediately reports an empty graph.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)

        if not args.skip_preflight:
            problems = node.preflight()
            if problems:
                print("  preflight found problems:\n")
                for p in problems:
                    print(f"    - {p}")
                print("\n  refusing to drive. --skip-preflight to try anyway.")
                return 3
            print("  preflight OK: governor publishing, driver subscribed.")

        what = "rotate" if args.turn else "drive forward"
        unit = "rad/s" if args.turn else "m/s"
        print(f"\n  plan: {what} at {args.speed} {unit} for {args.seconds} s, then stop.")
        print(f"  -> {TOPIC} at {PUBLISH_HZ:.0f} Hz (the governor clamps it)")

        if not args.execute:
            print("\n  DRY RUN -- nothing sent. Re-run with --execute to move the base.")
            print("  Remember the base ignores this until its motors are torqued:")
            print("      docker compose exec slate-base bash -lc './base_ctl.py torque on'")
            return 0

        print("\n  MOVING IN 2 SECONDS -- Ctrl-C to abort.")
        time.sleep(2.0)
        node.run(lin, ang, args.seconds)
        print("\n  If nothing moved: torque. ./base_ctl.py torque on")
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  aborted -- base stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
