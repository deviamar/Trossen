#!/usr/bin/env python3
"""Stand in for the SLATE driver. No hardware, no serial port.

    ./sim_base.py                    # pretend to be the base
    ./sim_base.py --torque-on        # start already torqued
    ./sim_base.py --estop            # start with the E-stop engaged

The real driver has NO simulation mode -- `slate_base_node` calls init_base() in
its constructor and goes straight to a serial port, and with no base attached it
logs FATAL and then keeps running with a timer that reads nothing. So there was
no way to exercise the velocity chain without a robot on the floor. This is that
way.

    subscribes  /slate/cmd_vel            geometry_msgs/Twist
    publishes   /slate/odom               nav_msgs/Odometry
                /slate/battery_state      sensor_msgs/BatteryState
    serves      /slate/set_motor_torque_status   std_srvs/SetBool
                /slate/enable_charging           std_srvs/SetBool

It integrates a differential-drive pose from the velocities it is given and
publishes odometry, so `move_base.py forward 0.5 --execute` actually converges
and stops -- the closed loop closes, on fake odometry.

IT REPRODUCES THE THREE THINGS THAT ACTUALLY CATCH PEOPLE, which is the only
reason a simulator like this is worth having over a print statement:

  1. VELOCITY EXPIRES. A command older than CMD_TIMEOUT_S (300 ms) is dropped
     and the base stops. Publish once and it moves for 300 ms, exactly like the
     real one -- so a publisher that forgets to repeat fails here too.
  2. TORQUE GATES EVERYTHING. It powers on with motors released and silently
     discards every /cmd_vel until `./base_ctl.py torque on`. No error, no
     warning, same as the hardware. This is the single most common "why isn't
     it moving".
  3. NO CLAMP. Whatever arrives on /slate/cmd_vel is acted on, because
     SlateBase::cmd_vel_callback() does not go through set_cmd_vel(). If you
     bypass the governor here, the sim will happily "drive" at 5 m/s -- which is
     the point: the clamp you are relying on is the governor's, and this proves
     whether it is in the path.

WHAT IT IS NOT. No mass, no wheel slip, no acceleration limit, no floor. Odometry
is the exact integral of the commanded velocity, so it is perfect in a way the
real base never is -- the real one accumulates error on every carpet edge. Use
it to prove the plumbing, not to predict where the robot ends up.

Run this INSTEAD of ./launch-base.sh -- both would subscribe to /slate/cmd_vel
and both would think they were the base.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

import base_config as cfg


class SimBase(Node):
    def __init__(self, torque_on, estop):
        super().__init__("slate_base_node_sim")
        self.x = self.y = self.yaw = 0.0
        self.vx = self.wz = 0.0
        self.cmd = (0.0, 0.0)
        self.last_cmd = 0.0
        self.torque = torque_on
        self.estop = estop
        self.charging = False
        self.battery = 87.0
        self.last_tick = time.monotonic()
        self.discarded = 0
        self.last_note = 0.0

        self.create_subscription(Twist, cfg.TOPIC_CMD_VEL, self._on_cmd, 1)
        self.pub_odom = self.create_publisher(Odometry, cfg.TOPIC_ODOM, 1)
        self.pub_batt = self.create_publisher(BatteryState, cfg.TOPIC_BATTERY, 1)

        self.create_service(SetBool, cfg.SRV_MOTOR_TORQUE, self._srv_torque)
        self.create_service(SetBool, cfg.SRV_ENABLE_CHARGING, self._srv_charging)

        # The two interbotix_slate_msgs services only exist if that package is
        # installed. It is, in this image -- but guarding keeps sim_base usable
        # from a plain ROS container, which is where you would run it to test
        # the chain with nothing else present.
        try:
            from interbotix_slate_msgs.srv import SetString, SetLightState
            self.create_service(SetString, cfg.SRV_SET_TEXT, self._srv_text)
            self.create_service(SetLightState, cfg.SRV_SET_LIGHT, self._srv_light)
        except ImportError:
            self.get_logger().info(
                "interbotix_slate_msgs not available -- set_text and "
                "set_light_state are not simulated")

        self.create_timer(1.0 / cfg.PUBLISH_HZ, self._tick)

    # ---- inputs ----------------------------------------------------------
    def _on_cmd(self, msg):
        # NO CLAMP, deliberately. The real cmd_vel_callback assigns straight
        # into the chassis struct, so this is where a missing governor becomes
        # visible instead of being quietly corrected.
        self.cmd = (float(msg.linear.x), float(msg.angular.z))
        self.last_cmd = time.monotonic()

    def _srv_torque(self, request, response):
        self.torque = bool(request.data)
        response.success = True
        response.message = f"motor torque {'on' if self.torque else 'off'} (SIMULATED)"
        self.get_logger().info(response.message)
        return response

    def _srv_charging(self, request, response):
        self.charging = bool(request.data)
        response.success = True
        response.message = f"charging {'enabled' if self.charging else 'disabled'} (SIMULATED)"
        return response

    def _srv_text(self, request, response):
        self.get_logger().info(f"screen: {request.data!r} (SIMULATED)")
        response.success = True
        response.message = "ok"
        return response

    def _srv_light(self, request, response):
        self.get_logger().info(f"light: {request.state} (SIMULATED)")
        response.success = True
        response.message = "ok"
        return response

    # ---- integrate -------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now

        expired = (now - self.last_cmd) > cfg.CMD_TIMEOUT_S
        wanted = (0.0, 0.0) if expired else self.cmd

        if self.estop:
            # E-stop brakes the wheels; the real base cannot even be pushed.
            self.vx = self.wz = 0.0
        elif not self.torque:
            self.vx = self.wz = 0.0
            if wanted != (0.0, 0.0):
                self.discarded += 1
                if now - self.last_note > 3.0:
                    self.get_logger().warn(
                        f"discarded {self.discarded} velocity commands -- motors are "
                        "NOT torqued. ./base_ctl.py torque on")
                    self.last_note = now
        else:
            self.vx, self.wz = wanted

        # Differential drive, exact integral. No slip, no acceleration limit.
        self.yaw += self.wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        if self.charging:
            self.battery = min(100.0, self.battery + 0.05 * dt)
        elif abs(self.vx) > 1e-6 or abs(self.wz) > 1e-6:
            self.battery = max(0.0, self.battery - 0.02 * dt)

        self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = cfg.DEFAULT_ODOM_FRAME
        o.child_frame_id = cfg.DEFAULT_BASE_FRAME
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        o.twist.twist.linear.x = self.vx
        o.twist.twist.angular.z = self.wz
        # The driver smuggles the E-stop into covariance[0] as -1 or 1 rather
        # than publishing a state topic; read_base.py reads it there, so the sim
        # has to put it in the same odd place to be useful.
        o.pose.covariance[0] = -1.0 if self.estop else 1.0
        self.pub_odom.publish(o)

        b = BatteryState()
        b.header.stamp = stamp
        b.percentage = float(self.battery)
        b.voltage = 24.0 + (self.battery / 100.0) * 4.0
        b.present = True
        self.pub_batt.publish(b)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--torque-on", action="store_true",
                    help="start torqued (the real base does not)")
    ap.add_argument("--estop", action="store_true", help="start with E-stop engaged")
    args = ap.parse_args()

    rclpy.init()
    node = SimBase(args.torque_on, args.estop)
    print("  =======================================================")
    print("  SLATE BASE SIMULATOR -- no hardware, no serial port.")
    print(f"  {cfg.TOPIC_ODOM} is an integral of what you command,")
    print("  not a measurement. Do not run this alongside the real driver.")
    print("  =======================================================")
    print(f"  torque: {'ON' if args.torque_on else 'OFF (commands will be discarded)'}")
    if args.estop:
        print("  E-STOP: ENGAGED")
    print("  Ctrl-C to stop.\n")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"\n  stopped at x={node.x:.3f} y={node.y:.3f} "
              f"yaw={math.degrees(node.yaw):.1f} deg")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
