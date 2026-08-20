#!/usr/bin/env python3
"""The rig's z axis. SIMULATOR -- no lift hardware is connected.

    ./lift_agent.py                  # simulate, publish /slate/lift/height
    ./lift_agent.py --start 0.25     # begin at a given height

The base drives x and y; this is z. One container, because it is one machine and
one operator intent -- "put the arms at this height" is not a different system
from "put the arms here".

    subscribes  /slate/lift/cmd_velocity  std_msgs/Float32   m/s, up positive
                /slate/lift/cmd_height    std_msgs/Float32   m, absolute
    publishes   /slate/lift/height        std_msgs/Float32   m
                /slate/lift/moving        std_msgs/Bool

It integrates a height from the commands it receives and publishes the result.
It moves nothing, because nothing is connected, and it says so at startup and
every thirty seconds. A silent simulator that looks like a driver is how someone
ends up trusting a number that describes nothing.

WHEN THE HARDWARE ARRIVES: keep the topics, replace _integrate() with real reads
and writes, delete the warnings. See base_config.py for the three shapes the
hardware might take and why only one fits this contract.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

import base_config as cfg


class LiftAgent(Node):
    def __init__(self, start_height):
        super().__init__("lift_agent")
        self.height = max(cfg.LIFT_HEIGHT_MIN, min(cfg.LIFT_HEIGHT_MAX, start_height))
        self.cmd_vel = 0.0
        self.cmd_height = None
        self.last_cmd = 0.0
        self.last_tick = time.monotonic()
        self.last_warn = 0.0

        self.create_subscription(Float32, cfg.TOPIC_LIFT_CMD_VELOCITY, self._on_vel, 1)
        self.create_subscription(Float32, cfg.TOPIC_LIFT_CMD_HEIGHT, self._on_height, 1)
        self.pub_height = self.create_publisher(Float32, cfg.TOPIC_LIFT_HEIGHT, 1)
        self.pub_moving = self.create_publisher(Bool, cfg.TOPIC_LIFT_MOVING, 1)
        self.create_timer(1.0 / cfg.PUBLISH_HZ, self._tick)

    def _on_vel(self, msg):
        self.cmd_vel = max(-cfg.LIFT_MAX_VEL, min(cfg.LIFT_MAX_VEL, float(msg.data)))
        self.cmd_height = None          # an explicit velocity overrides a setpoint
        self.last_cmd = time.monotonic()

    def _on_height(self, msg):
        self.cmd_height = max(cfg.LIFT_HEIGHT_MIN,
                              min(cfg.LIFT_HEIGHT_MAX, float(msg.data)))
        self.last_cmd = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now

        vel = 0.0
        if (now - self.last_cmd) <= cfg.LIFT_CMD_TIMEOUT_S:
            if self.cmd_height is not None:
                error = self.cmd_height - self.height
                # Proportional approach, capped. Enough to converge without
                # modelling an acceleration profile for hardware nobody has seen.
                vel = max(-cfg.LIFT_MAX_VEL, min(cfg.LIFT_MAX_VEL, error * 2.0))
                if abs(error) < 1e-4:
                    vel = 0.0
            else:
                vel = self.cmd_vel

        self._integrate(vel, dt)

        h = Float32()
        h.data = float(self.height)
        self.pub_height.publish(h)
        m = Bool()
        m.data = abs(vel) > 1e-6
        self.pub_moving.publish(m)

        if now - self.last_warn > 30.0:
            self.get_logger().warn(
                "SIMULATED -- no lift hardware is connected. "
                f"{cfg.TOPIC_LIFT_HEIGHT} = {self.height:.3f} m is not a measurement.")
            self.last_warn = now

    def _integrate(self, vel, dt):
        """The whole simulation. Replace with real hardware I/O."""
        self.height = max(cfg.LIFT_HEIGHT_MIN,
                          min(cfg.LIFT_HEIGHT_MAX, self.height + vel * dt))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", type=float, default=cfg.LIFT_HEIGHT_MIN,
                    help="initial simulated height, m")
    args = ap.parse_args()

    rclpy.init()
    node = LiftAgent(args.start)
    print("  =================================================")
    print("  LIFT SIMULATOR -- no hardware is connected.")
    print(f"  {cfg.TOPIC_LIFT_HEIGHT} is computed, not measured.")
    print("  =================================================")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
