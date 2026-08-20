#!/usr/bin/env python3
"""Own /slate/cmd_vel: mux the command sources, clamp them, keep them alive.

    ./governor.py                    # run it; needs launch-base.sh already up
    ./governor.py --max-x 0.2        # tighter than the config default
    ./governor.py --verbose          # log every source switch

    /slate/cmd_vel_teleop  ─┐
                            ├─▶  governor  ─▶  /slate/cmd_vel  ─▶  driver
    /slate/cmd_vel_nav     ─┘

WHY THIS EXISTS
---------------
The driver does not clamp. MAX_VEL_X and MAX_VEL_Z are #defined to 1.0 in
trossen_slate.hpp and TrossenSlate::set_cmd_vel() enforces them, but
SlateBase::cmd_vel_callback() never calls that function -- it assigns
msg->linear.x straight into the chassis data struct. Whatever arrives on
/slate/cmd_vel is what the base is asked to do.

Every script in this directory used to clamp itself, which was safe while every
publisher was a script in this repo. It stops being safe the moment an input
device is a separate container that can be rebuilt, replaced or written by
someone who has not read base_config.py. So the clamp moved here, into the
container that owns the serial port. A teleop container can now be wrong without
the base being able to act on it.

WHAT IT GUARANTEES
------------------
1. Output is clamped to CLAMP_VEL_X / CLAMP_VEL_Z regardless of input.
2. Exactly one source drives at a time, by priority. Teleop outranks autonomy,
   so grabbing the controller stops the robot driving itself -- no arbitration
   protocol between the two publishers, and no need for them to know about each
   other.
3. A source that goes quiet for SOURCE_TIMEOUT_S is dropped and the next
   priority takes over; with no sources left the output is zero.
4. Output republishes at cfg.PUBLISH_HZ whether or not input arrives, because
   the driver's own 300 ms deadline would otherwise stop the base mid-command
   on any input hiccup.

NOT A SAFETY CERTIFICATION. It bounds velocity, nothing else. It knows nothing
about obstacles, the E-stop, or whether the arms are extended past the base
footprint. The person watching is still the safety system.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

import base_config as cfg

# Highest priority first. A source is "active" if it has published within
# SOURCE_TIMEOUT_S; the first active one in this list wins outright.
#
# Deliberately not a blend: two sources averaged is a velocity neither of them
# asked for, and on a machine that moves across a floor "something in between"
# is the wrong answer to a disagreement.
SOURCES = [
    ("teleop", cfg.TOPIC_CMD_VEL_TELEOP),
    ("nav", cfg.TOPIC_CMD_VEL_NAV),
]

# Longer than the driver's 300 ms so a source that is merely stuttering is not
# handed over mid-motion, short enough that a dead publisher stops the base
# quickly. The driver's own deadline is the backstop underneath this one.
SOURCE_TIMEOUT_S = 0.5


class Governor(Node):
    def __init__(self, max_x, max_z, verbose):
        super().__init__("slate_governor")
        self.max_x = max_x
        self.max_z = max_z
        self.verbose = verbose

        self.latest = {name: (0.0, 0.0) for name, _ in SOURCES}
        self.stamp = {name: 0.0 for name, _ in SOURCES}
        self.current = None

        for name, topic in SOURCES:
            # Depth 1: a velocity setpoint has no value once a newer one exists,
            # so queueing them would only add latency.
            self.create_subscription(
                Twist, topic, self._make_cb(name), 1)

        self.pub = self.create_publisher(Twist, cfg.TOPIC_CMD_VEL, 1)
        self.create_timer(1.0 / cfg.PUBLISH_HZ, self._tick)

    def _make_cb(self, name):
        def cb(msg):
            self.latest[name] = (msg.linear.x, msg.angular.z)
            self.stamp[name] = time.monotonic()
        return cb

    def _active(self):
        now = time.monotonic()
        for name, _ in SOURCES:
            if now - self.stamp[name] < SOURCE_TIMEOUT_S:
                return name
        return None

    def _tick(self):
        name = self._active()

        if name != self.current:
            # Log the handover: "the base is ignoring my publisher" is otherwise
            # invisible, and it is almost always a priority question.
            if self.verbose or name is None or self.current is None:
                self.get_logger().info(
                    f"source: {self.current or 'none'} -> {name or 'none'}")
            self.current = name

        x, z = self.latest[name] if name else (0.0, 0.0)

        msg = Twist()
        msg.linear.x = max(-self.max_x, min(self.max_x, float(x)))
        msg.angular.z = max(-self.max_z, min(self.max_z, float(z)))
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-x", type=float, default=cfg.CLAMP_VEL_X,
                    help=f"m/s ceiling (default {cfg.CLAMP_VEL_X})")
    ap.add_argument("--max-z", type=float, default=cfg.CLAMP_VEL_Z,
                    help=f"rad/s ceiling (default {cfg.CLAMP_VEL_Z})")
    ap.add_argument("--verbose", action="store_true", help="log every source switch")
    args = ap.parse_args()

    # Refuse to raise the ceiling past what the vendor header itself allows.
    # Lowering is always fine; this only catches a typo'd order of magnitude.
    if args.max_x > cfg.VENDOR_MAX_VEL_X or args.max_z > cfg.VENDOR_MAX_VEL_Z:
        print(f"refusing: limits above the vendor maximum "
              f"({cfg.VENDOR_MAX_VEL_X} m/s, {cfg.VENDOR_MAX_VEL_Z} rad/s)",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = Governor(args.max_x, args.max_z, args.verbose)
    print(f"  governor up: {' > '.join(n for n, _ in SOURCES)} "
          f"-> {cfg.TOPIC_CMD_VEL}")
    print(f"  ceiling {args.max_x} m/s, {args.max_z} rad/s. Ctrl-C to stop.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Leave the base stopped, not coasting on the last setpoint. On SIGTERM
        # the context may already be gone, in which case the driver's own 300 ms
        # deadline is what stops it.
        if rclpy.ok():
            stop = Twist()
            for _ in range(5):
                node.pub.publish(stop)
                time.sleep(0.01)
        print("\n  stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
