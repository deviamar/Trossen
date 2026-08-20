#!/usr/bin/env python3
"""Read the SLATE base's odometry and battery. Read-only -- commands nothing.

    ./read_base.py              # one snapshot
    ./read_base.py --watch      # live-updating block until Ctrl-C
    ./read_base.py --raw        # one line: x y yaw vx wz pct volts

Needs ./launch-base.sh running in another shell -- this is a subscriber, not a
second driver, and the base's serial port only admits one.

What "odometry" means here is worth pinning down, because the number is not what
you might expect. The driver subtracts the first sample it ever sees, so the
origin is wherever the base happened to be when the DRIVER started, not where
the base was powered on and not any fixed point in the room. Restart the driver
and x/y/yaw jump back to zero without the base moving. It is wheel odometry
besides -- it accumulates error on every slip and every carpet edge, and nothing
here corrects it.
"""
import sys

import rclpy

import base_config as cfg
import slate


def render(node):
    return "\n".join([
        slate.render_odom(node.odom),
        slate.render_battery(node.battery),
    ])


def main():
    ap = slate.parser(__doc__)
    ap.add_argument("--watch", action="store_true", help="live block until Ctrl-C")
    ap.add_argument("--raw", action="store_true",
                    help="one line of numbers, for piping")
    args = ap.parse_args()

    rclpy.init()
    node = slate.BaseListener("slate_read_base")
    try:
        if slate.wait_for_odom(node) is None:
            print(slate.no_driver_message(cfg.TOPIC_ODOM), file=sys.stderr)
            return 1

        if not args.watch:
            # Odometry publishes at update_frequency (20 Hz) and battery_state
            # at every 10th update (~2 Hz), so a snapshot that returns the
            # moment odom lands beats the battery topic almost every time and
            # prints "no sample yet" on a perfectly healthy base. Spin a little
            # longer for it. --watch needs none of this: it keeps spinning
            # anyway, so the battery block fills itself in within a second.
            for _ in range(20):
                if node.battery is not None:
                    break
                rclpy.spin_once(node, timeout_sec=0.1)

        if args.raw:
            o, b = node.odom, node.battery
            print(" ".join(f"{v:.6f}" for v in [
                o.pose.pose.position.x,
                o.pose.pose.position.y,
                slate.yaw_of(o),
                o.twist.twist.linear.x,
                o.twist.twist.angular.z,
                b.percentage if b else float("nan"),
                b.voltage if b else float("nan"),
            ]))
        elif args.watch:
            print("Reading (Ctrl-C to stop). Push the base by hand to see it move.\n")
            lines = 0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
                block = render(node)
                if lines:
                    # Move up, then ERASE TO END OF SCREEN before redrawing.
                    # Overwriting alone is not enough: this block changes height
                    # (the battery section is one line until the first ~2 Hz
                    # sample lands, then four) and changes width per line, so a
                    # plain overwrite leaves the tail of any longer previous
                    # line on screen -- which showed up as "battery (no sample
                    # yet)" sitting above the battery values it had supposedly
                    # been replaced by.
                    sys.stdout.write(f"\033[{lines}A\033[J")
                lines = block.count("\n") + 1
                print(block)
        else:
            print(render(node))
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
