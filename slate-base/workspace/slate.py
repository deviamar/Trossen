"""Shared rclpy helpers for the SLATE base CLIs.

Everything in this directory is a client of the driver node, never a second
driver. That is not a style preference -- the base speaks over one serial port
and `SerialDriver::init` opens it exclusively, so exactly one process can talk
to the hardware. The shape is the same constraint the WXAI arms have ("one
connection at a time", see ../../manip-arm/workspace/arm.py), reached by a
different route.

The practical consequence: `./launch-base.sh` has to be running in another
shell before any of these do anything. If it is not, they time out waiting for
a topic rather than falling back to opening the port themselves -- which they
could not do anyway.

There is also no Python binding available in this image. trossen_slate ships
pybind11 bindings (`pytrossen_slate`), but they open the same exclusive port, so
having them here would only invite running two drivers and having neither work.
ROS 2 topics are the supported way in, and they are what these scripts use.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

import base_config as cfg


def parser(doc):
    return argparse.ArgumentParser(
        description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)


class BaseListener(Node):
    """Subscribes to odom and battery_state. Commands nothing."""

    def __init__(self, name="slate_listener", battery=True):
        super().__init__(name)
        self.odom = None
        self.battery = None
        # depth=1 throughout: these are state topics, and after any pause in
        # processing a deeper queue just hands us a backlog of stale samples.
        self.create_subscription(Odometry, cfg.TOPIC_ODOM, self._odom_cb, 1)
        if battery:
            self.create_subscription(
                BatteryState, cfg.TOPIC_BATTERY, self._batt_cb, 1)

    def _odom_cb(self, msg):
        self.odom = msg

    def _batt_cb(self, msg):
        self.battery = msg


def wait_for_odom(node, timeout_s=5.0):
    """Spin until an Odometry message lands. Returns it, or None on timeout."""
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while rclpy.ok() and node.odom is None:
        if node.get_clock().now().nanoseconds > deadline:
            return None
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.odom


def no_driver_message(topic=None):
    """The message for 'nothing is publishing', with the way out."""
    return (
        f"No message on {topic or cfg.TOPIC_ODOM} within 5s.\n"
        "  Is the driver running?   ./launch-base.sh    (in another shell)\n"
        "  Is the base connected?   ls -l /dev/ttySLATE\n"
        "  Is the base powered on?  its screen should be lit\n"
        f"  Does the topic exist?    ros2 topic list | grep {cfg.NS.lstrip('/')}\n"
        "\n"
        "  If the topic list is empty from inside this container but the driver\n"
        "  is up in another, that is a DDS mismatch, not a wiring problem --\n"
        "  check ROS_DOMAIN_ID and RMW_IMPLEMENTATION match across the compose\n"
        "  files."
    )


def estop_from_odom(msg):
    """True if the base reports EMERGENCY STOP, None if it cannot be told.

    The driver keeps SystemState to itself -- there is no topic carrying it.
    The single bit that escapes is this, from SlateBase::update():

        odom.pose.covariance[0] = (system_state == SYS_ESTOP) ? -1 : 1;

    which is a covariance field being used as a flag. Reading it is the only way
    a ROS client can see the E-stop at all, so these scripts read it, but treat
    the encoding as what it is: undocumented, and liable to go away the moment
    upstream decides to publish the state properly or to fill in a real
    covariance. Anything else in that slot returns None -- unknown -- rather
    than being guessed at in either direction.

    NOT A SAFETY INTERLOCK. It is a status readout, sampled at 20 Hz over a
    serial link, and the E-stop button is the safety device.
    """
    c = msg.pose.covariance[0]
    if c == -1.0:
        return True
    if c == 1.0:
        return False
    return None


def estop_label(state):
    if state is True:
        return "E-STOP ENGAGED"
    if state is False:
        return "clear"
    return "unknown"


def yaw_of(msg):
    """Yaw in radians from an Odometry message's quaternion.

    The base is planar, so roll and pitch are always zero and the full
    quaternion-to-Euler conversion reduces to one atan2.
    """
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def render_odom(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = yaw_of(msg)
    return "\n".join([
        "  odometry (relative to wherever the driver started)",
        f"    x        {x:>9.3f} m",
        f"    y        {y:>9.3f} m",
        f"    yaw      {yaw:>9.3f} rad  ({math.degrees(yaw):>7.2f} deg)",
        "  measured velocity",
        f"    linear   {msg.twist.twist.linear.x:>9.3f} m/s",
        f"    angular  {msg.twist.twist.angular.z:>9.3f} rad/s",
        f"  state      {estop_label(estop_from_odom(msg)):>9}",
    ])


def render_battery(msg):
    if msg is None:
        # The driver publishes battery_state only every 10th update (~2 Hz at
        # the default 20 Hz loop), so a snapshot can legitimately arrive before
        # the first one. Say so rather than printing zeros.
        return "  battery    (no sample yet -- published at ~2 Hz)"
    # temperature/charge/capacity/design_capacity are explicitly set to NaN by
    # the driver; only voltage, current and percentage carry real values.
    #
    # `percentage` is 0-100, NOT the 0-1 that sensor_msgs/BatteryState
    # documents: the driver assigns it straight from ChassisData.charge, a
    # uint32. Measured on this rig at 81.00 alongside 27.68 V, which settles it.
    #
    # The sign of `current` is still unconfirmed -- it read 0.00 A sitting idle
    # under E-stop, which says nothing. Whether positive means charging or
    # discharging is the base's convention, not the message's, so no label is
    # guessed here. Read it once while driving and once on the dock, then label
    # it and drop this note.
    return "\n".join([
        "  battery",
        f"    charge   {msg.percentage:>9.1f} %",
        f"    voltage  {msg.voltage:>9.2f} V",
        f"    current  {msg.current:>9.2f} A",
    ])


class VelocityDriver(Node):
    """Publishes Twist at cfg.PUBLISH_HZ for as long as it is asked to.

    A publish LOOP rather than a single message, because /cmd_vel is a setpoint
    with a 300 ms deadline (cfg.CMD_TIMEOUT_S): the driver zeroes the stored
    command when nothing has arrived for that long. One message buys 300 ms of
    motion, not a completed move.

    stop() publishes zeros several times rather than once. A single stop message
    that is dropped -- and a depth-1 best-effort-ish path can drop one -- would
    leave the base running until the deadline expires. Repeating costs
    milliseconds and removes that.
    """

    def __init__(self, name="slate_velocity"):
        super().__init__(name)
        # Through the governor, not straight at the driver. See
        # base_config.TOPIC_CMD_VEL_TELEOP for why -- briefly, the governor
        # republishes at 20 Hz and a second publisher on its output topic
        # interleaves with it rather than overriding it.
        #
        # Consequence worth knowing: with no governor running, these commands go
        # nowhere. That is the right failure -- the clamp is not optional -- and
        # the governor is autostarted, but it does mean `./governor.py` has to
        # be up before move_base.py or teleop_keyboard.py do anything.
        self.pub = self.create_publisher(Twist, cfg.TOPIC_CMD_VEL_TELEOP, 1)

    def send(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def drive_for(self, linear, angular, duration_s, on_tick=None):
        """Hold a velocity for duration_s, then stop. Returns seconds elapsed.

        Ctrl-C stops the base before re-raising: the interesting failure mode
        here is a human aborting a move that is going wrong, and inheriting the
        keyboard interrupt without zeroing first would leave the base running
        for another 300 ms.
        """
        period = 1.0 / cfg.PUBLISH_HZ
        start = self.get_clock().now()
        elapsed = 0.0
        try:
            while rclpy.ok() and elapsed < duration_s:
                self.send(linear, angular)
                rclpy.spin_once(self, timeout_sec=period)
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                if on_tick:
                    on_tick(elapsed)
        except KeyboardInterrupt:
            self.stop()
            print("\n  interrupted -- base stopped.", file=sys.stderr)
            raise
        self.stop()
        return elapsed

    def stop(self, repeats=5):
        for _ in range(repeats):
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)


def wait_for_subscriber(node, timeout_s=5.0):
    """Spin until the driver is subscribed to cmd_vel. False on timeout.

    Publishing into a topic nobody listens to succeeds silently, so without this
    a `--execute` against a stopped driver prints a normal-looking move and the
    base never twitches. Checking for a subscriber turns that into an error
    before anything is sent.
    """
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while rclpy.ok():
        if node.pub.get_subscription_count() > 0:
            return True
        if node.get_clock().now().nanoseconds > deadline:
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def confirm_or_dry_run(args, description):
    """True if the caller should actually send. Prints the dry-run notice if not."""
    if getattr(args, "execute", False):
        print(f"\n  {description}")
        return True
    print(f"\n  DRY RUN -- nothing sent. Would {description}\n"
          "  Re-run with --execute to move the base.\n"
          "  Check the path is clear first: this moves the whole robot.")
    return False
