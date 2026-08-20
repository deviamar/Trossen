"""Topics, services, limits and state codes for the SLATE mobile base.

The values here are not guesses -- every one is read out of the driver source
(Interbotix/interbotix_ros_core @ humble, interbotix_ros_slate/, plus its
trossen_slate submodule) and the file notes where. That matters more than usual
because the vendor documentation for this base does not list its topics,
services or parameters at all.

Two things about this hardware that shape every script here, both different
from the arms:

  * IT MOVES THE WHOLE ROBOT. The arms fail safe: a bad command hits a joint
    limit and the controller refuses it. There is no equivalent here. There is
    no position limit on a floor, and the only thing between a wrong number and
    a collision is the person watching. That is why the commanding scripts are
    dry-run by default and why they clamp rather than trusting the driver to.

  * VELOCITY COMMANDS EXPIRE. /cmd_vel is a setpoint with a 300 ms deadline,
    not a move-to command. Publish once and the base moves for 300 ms and
    stops; stop publishing mid-motion and it coasts to a halt within 300 ms.
    Every commanding script here therefore runs a publish loop for as long as
    it wants motion, and a script that dies takes the motion with it.
"""
import os

# The driver node creates every topic and service RELATIVE to its own node
# namespace, and takes no namespace argument of its own -- launch-base.sh
# applies this with `--ros-args -r __ns:=`. Set in docker-compose.yml.
#
# Why namespace at all: a bare /cmd_vel on a shared DDS graph is anything on the
# domain that publishes the conventional topic name driving this base. The arms
# tolerate that; a mobile base does not.
NS = os.environ.get("SLATE_NS", "/slate").rstrip("/")

# Node name is fixed in the driver's constructor (rclcpp::Node("slate_base")),
# so it is not configurable the way the namespace is.
NODE_NAME = "slate_base"

TOPIC_CMD_VEL = f"{NS}/cmd_vel"            # geometry_msgs/Twist    (subscribed)

# WHERE CLIENTS PUBLISH. Not TOPIC_CMD_VEL -- that one belongs to governor.py,
# which is the only thing that should write to the driver. Everything else
# (move_base.py, teleop_keyboard.py, the Quest, the monitor's drive_test) goes
# here and is clamped and prioritised on the way through.
#
# Two publishers on TOPIC_CMD_VEL is not a style problem: the governor
# republishes at 20 Hz to hold the driver's deadline open, so a second publisher
# does not override it, it INTERLEAVES with it. The base then alternates between
# two setpoints 50 ms apart, which reads as juddering rather than as a conflict.
TOPIC_CMD_VEL_TELEOP = f"{NS}/cmd_vel_teleop"   # human input, highest priority
TOPIC_CMD_VEL_NAV = f"{NS}/cmd_vel_nav"         # autonomy, yields to teleop
TOPIC_ODOM = f"{NS}/odom"                  # nav_msgs/Odometry      (published)
TOPIC_BATTERY = f"{NS}/battery_state"      # sensor_msgs/BatteryState

SRV_SET_TEXT = f"{NS}/set_text"                        # interbotix_slate_msgs/SetString
SRV_MOTOR_TORQUE = f"{NS}/set_motor_torque_status"     # std_srvs/SetBool
SRV_ENABLE_CHARGING = f"{NS}/enable_charging"          # std_srvs/SetBool
SRV_SET_LIGHT = f"{NS}/set_light_state"                # interbotix_slate_msgs/SetLightState

# Node parameters, with the driver's own defaults (slate_base.cpp constructor).
DEFAULT_UPDATE_FREQUENCY = 20        # Hz, the driver's own control loop
DEFAULT_PUBLISH_TF = False           # odom -> base_link
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_link"

# ---------------------------------------------------------------------------
# Velocity limits
#
# MAX_VEL_X / MAX_VEL_Z are #defined as 1.0 in trossen_slate.hpp, and
# TrossenSlate::set_cmd_vel() clamps to them.
#
# THE ROS NODE DOES NOT GO THROUGH THAT FUNCTION. SlateBase::cmd_vel_callback()
# assigns msg->linear.x and msg->angular.z straight into the chassis data struct
# and the next update() writes it to the holding registers. So the clamp that
# exists in the C++ API is simply not in the path a /cmd_vel message takes, and
# whatever you publish is what the base is asked to do.
#
# Hence CLAMP_* below, applied client-side by every script here. They are set
# well under the vendor maximum on purpose: 1.0 m/s is a fast walk, and a rig
# carrying three arms and a camera mast has a high centre of mass. Raise them
# deliberately once you know how this base behaves loaded, not to make a script
# stop complaining.
VENDOR_MAX_VEL_X = 1.0     # m/s,   trossen_slate.hpp MAX_VEL_X
VENDOR_MAX_VEL_Z = 1.0     # rad/s, trossen_slate.hpp MAX_VEL_Z

CLAMP_VEL_X = float(os.environ.get("SLATE_MAX_VEL_X", 0.3))    # m/s
CLAMP_VEL_Z = float(os.environ.get("SLATE_MAX_VEL_Z", 0.8))    # rad/s

# CMD_TIME_OUT in trossen_slate.hpp, in ms. The driver zeroes the stored command
# once this much time has passed since the last /cmd_vel message, so a publisher
# has to repeat itself to keep the base moving.
CMD_TIMEOUT_S = 0.300

# Comfortably inside the timeout, and matched to the driver's own 20 Hz update
# loop -- publishing faster than the driver reads gains nothing. At 20 Hz a
# missed message costs 50 ms of the 300 ms budget, so the base keeps moving
# smoothly through ordinary scheduling jitter but still stops promptly when the
# publisher genuinely dies.
PUBLISH_HZ = 20.0

# ---------------------------------------------------------------------------
# System state
#
# The SystemState enum from trossen_slate/serial_driver.hpp. The driver does NOT
# publish this as a topic -- see estop_from_odom() in slate.py for the one bit
# of it that does escape, and read that function's docstring before relying on
# any of this.
SYSTEM_STATES = {
    0x00: ("SYS_INIT", "initializing"),
    0x01: ("SYS_NORMAL", "normal"),
    0x02: ("SYS_REMOTE", "driven by the handheld remote"),
    0x03: ("SYS_ESTOP", "EMERGENCY STOP engaged"),
    0x04: ("SYS_CALIB", "calibrating"),
    0x05: ("SYS_TEST", "self-test"),
    0x06: ("SYS_CHARGING", "charging"),
    0x10: ("SYS_ERR", "unspecified error"),
    0x11: ("SYS_ERR_ID", "motor ID error"),
    0x12: ("SYS_ERR_COM", "communication error"),
    0x13: ("SYS_ERR_ENC", "encoder error"),
    0x14: ("SYS_ERR_COLLISION", "collision detected"),
    0x15: ("SYS_ERR_LOW_VOLTAGE", "battery voltage too low"),
    0x16: ("SYS_ERR_OVER_VOLTAGE", "battery voltage too high"),
    0x17: ("SYS_ERR_OVER_CURRENT", "over-current"),
    0x18: ("SYS_ERR_OVER_TEMP", "over-temperature"),
}

# Light states, from interbotix_slate_msgs/srv/SetLightState.srv. Note the gap:
# the solid colours are 0-7 and the flashing ones 9-15. 8 is not defined.
LIGHT_STATES = {
    "off": 0,
    "red": 1,
    "green": 2,
    "yellow": 3,
    "blue": 4,
    "purple": 5,
    "cyan": 6,
    "white": 7,
    "red-flash": 9,
    "green-flash": 10,
    "yellow-flash": 11,
    "blue-flash": 12,
    "purple-flash": 13,
    "cyan-flash": 14,
    "white-flash": 15,
}


def clamp(linear, angular):
    """Clamp a velocity pair to CLAMP_VEL_*. Returns (x, z, was_clamped)."""
    x = max(-CLAMP_VEL_X, min(CLAMP_VEL_X, linear))
    z = max(-CLAMP_VEL_Z, min(CLAMP_VEL_Z, angular))
    return x, z, (x != linear or z != angular)


# ---------------------------------------------------------------------------
# LIFT -- the z axis.
#
# The base moves the rig in x and y; the lift moves it in z. They were briefly
# separate containers, which was the wrong cut: one physical machine, one
# operator intent ("put the arms here"), and every consumer wanted both. Same
# container now, adjacent namespaces.
#
# NO HARDWARE IS CONNECTED. lift_agent.py integrates a height and publishes it,
# so the contract is live and the rest of the rig can be built against it. It
# moves nothing and says so, loudly and repeatedly.
#
# When real hardware arrives, the honest question is which of three things it
# is, because only the first matches the contract below:
#   * position-controlled actuator that reports height -> this is right
#   * relay/GPIO up-down with no feedback -> cmd_height is a fiction; the
#     contract collapses to an up/down/stop enum
#   * vendor ROS driver -> wrap it the way this container wraps the base driver
LIFT_NS = f"{NS}/lift"

TOPIC_LIFT_CMD_VELOCITY = f"{LIFT_NS}/cmd_velocity"   # std_msgs/Float32, m/s, up positive
TOPIC_LIFT_CMD_HEIGHT = f"{LIFT_NS}/cmd_height"       # std_msgs/Float32, m, absolute
TOPIC_LIFT_HEIGHT = f"{LIFT_NS}/height"               # std_msgs/Float32, m
TOPIC_LIFT_MOVING = f"{LIFT_NS}/moving"               # std_msgs/Bool

# Placeholders, every one a guess until there is a lift to measure. They exist
# so the simulator behaves plausibly, not because they describe any hardware.
LIFT_HEIGHT_MIN = float(os.environ.get("LIFT_MIN_M", 0.0))
LIFT_HEIGHT_MAX = float(os.environ.get("LIFT_MAX_M", 0.5))
LIFT_MAX_VEL = float(os.environ.get("LIFT_MAX_VEL", 0.05))

# Same expiring-setpoint rule as /cmd_vel, and worth keeping whatever the
# hardware turns out to be: a lift that keeps rising because a node died is the
# first failure mode to design out.
LIFT_CMD_TIMEOUT_S = 0.3
