"""Frames, button layout, and teleop mapping for the Meta Quest.

Everything here describes THE INPUT DEVICE, not the robots. The robot-facing
half -- which topic a thumbstick ends up driving -- is deliberately in
quest_teleop.py, and the robots themselves know nothing about any of it. Swap
the Quest for a gamepad and no robot container changes.

THE FRAME PROBLEM, which is the one thing to get right here
-----------------------------------------------------------
Unity is LEFT-handed with Y up. ROS (REP-103) is RIGHT-handed with Z up. Going
between them is a handedness flip, not a permutation of axis labels, and getting
it wrong yields teleop that feels almost correct with exactly one axis mirrored
-- which reads as "the tracking is bad" rather than "the maths is wrong".

The conversion lives in the backend, next to the app that defines the frame,
because only that app knows what convention it is sending. unity_to_ros() below
is the standard mapping for a Unity app that has not already converted:

    ROS x (forward) =  Unity z
    ROS y (left)    = -Unity x
    ROS z (up)      =  Unity y

and for the quaternion, the same axis remap plus a sign flip on w to reverse the
rotation's handedness.

Only CHANGES in controller pose are used while teleoperating, so a constant
offset between quest_origin and the room cancels out. A handedness error does
not cancel, which is why this is worth being careful about.
"""
import os

NS = os.environ.get("QUEST_NS", "/quest").rstrip("/")

TOPIC_HEAD_POSE = f"{NS}/head/pose"
TOPIC_LEFT_POSE = f"{NS}/left/pose"
TOPIC_RIGHT_POSE = f"{NS}/right/pose"
TOPIC_LEFT_JOY = f"{NS}/left/joy"
TOPIC_RIGHT_JOY = f"{NS}/right/joy"
TOPIC_CONNECTED = f"{NS}/connected"

FRAME_ID = os.environ.get("QUEST_FRAME", "quest_origin")

# Mounted read-only; see quest/docker-compose.yml. Never baked into the image.
SECRETS_DIR = os.environ.get("QUEST_SECRETS_DIR", "/secrets")

# ---------------------------------------------------------------------------
# The return path. The headset is the only device in this rig that is also a
# display, so alone among the components it is both subscriber and publisher.
#
# Feedback is a JSON blob on a std_msgs/String rather than a typed message. It
# is device-specific data going to a device-specific Unity app -- the fields are
# whatever that app renders -- so a custom .msg would force every container to
# rebuild for a change only two nodes care about. See docs/topic-contract.md.
TOPIC_FEEDBACK = f"{NS}/feedback"

# Stereo view. Published by the middle arm's ZED, consumed here and pushed down
# the same WebRTC connection as two video tracks.
TOPIC_STEREO_LEFT = os.environ.get(
    "QUEST_STEREO_LEFT", "/middle_cam/zed_node/left/image_rect_color")
TOPIC_STEREO_RIGHT = os.environ.get(
    "QUEST_STEREO_RIGHT", "/middle_cam/zed_node/right/image_rect_color")

# The Quest renders a fixed stereo pair; anything else gets letterboxed or
# stretched by the app. Resize once here rather than reconfiguring the camera,
# so the ZED can keep publishing whatever resolution the rest of the rig wants.
STEREO_WIDTH = int(os.environ.get("QUEST_STEREO_WIDTH", 640))
STEREO_HEIGHT = int(os.environ.get("QUEST_STEREO_HEIGHT", 480))

# ---------------------------------------------------------------------------
# Joy layout. Same for both hands; see docs/topic-contract.md.
AXIS_STICK_X = 0
AXIS_STICK_Y = 1
AXIS_TRIGGER = 2      # index finger, analog 0..1
AXIS_GRIP = 3         # middle finger, analog 0..1

BTN_PRIMARY = 0       # X on the left controller, A on the right
BTN_SECONDARY = 1     # Y on the left, B on the right
BTN_STICK = 2
BTN_MENU = 3

# ---------------------------------------------------------------------------
# Publish rate for the raw device topics. 72 Hz is the Quest's own tracking
# rate; there is nothing to gain by resampling it upward and a downstream 50 Hz
# arm loop is happy with it.
PUBLISH_HZ = float(os.environ.get("QUEST_HZ", 72.0))

# Frames with no update for this long mean the headset went to sleep, the app
# crashed, or the link dropped. /quest/connected goes false and quest_teleop
# releases everything it was driving.
STALE_S = 0.35

# ---------------------------------------------------------------------------
# Base driving. Thumbstick -> Twist.
#
# The mapping the rig was specified with: LEFT stick forward drives the base
# forward, RIGHT stick forward rotates it clockwise. Note that "right stick
# forward = turn" is unusual -- most rigs put turning on the right stick's X
# axis -- so if it feels wrong in the hand, set QUEST_TURN_AXIS=x and it moves
# to left/right without touching any code.
BASE_MAX_VEL_X = float(os.environ.get("QUEST_BASE_MAX_X", 0.25))   # m/s
BASE_MAX_VEL_Z = float(os.environ.get("QUEST_BASE_MAX_Z", 0.6))    # rad/s
TURN_AXIS = os.environ.get("QUEST_TURN_AXIS", "y").lower()         # "y" or "x"

# Sticks do not rest at exactly zero, and a base that creeps while nobody is
# touching it is both alarming and hard to diagnose.
STICK_DEADZONE = 0.12

# The base clamps again in slate-base/workspace/governor.py. These are comfort
# limits for the operator; that one is the safety limit, and it is enforced in
# the container that owns the serial port precisely so this file cannot raise it.

# ---------------------------------------------------------------------------
# Arm teleop.
#
# Hold-to-engage, not toggle. Releasing the button must stop the arm following
# you -- a toggle leaves an armed robot behind when you put the controller down,
# and the failure is silent until you move.
ARM_SCALE = float(os.environ.get("QUEST_ARM_SCALE", 1.0))
ARM_NS_LEFT = os.environ.get("QUEST_ARM_NS_LEFT", "/left_arm")
ARM_NS_RIGHT = os.environ.get("QUEST_ARM_NS_RIGHT", "/right_arm")
BASE_NS = os.environ.get("QUEST_BASE_NS", "/slate")

# The active-vision arm. Driven by HEAD pose, not a controller, and engaged
# whenever either hand is engaged -- you want the camera to follow you while
# your hands are busy, and to stop when you let go of both.
MIDDLE_NS = os.environ.get("QUEST_MIDDLE_NS", "/middle")

# Trigger 0..1 -> gripper opening in metres, open at rest and closed when
# squeezed. GRIPPER_OPEN matches the controller's enforced position limit; the
# mechanism physically reaches ~0.044 but commanding that trips a limit error.
GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.0
TRIGGER_DEADZONE = 0.05


def unity_to_ros_position(x, y, z):
    """Unity (left-handed, Y up) -> ROS (right-handed, Z up)."""
    return (z, -x, y)


def unity_to_ros_quaternion(x, y, z, w):
    """Same remap for the rotation, with the handedness flip."""
    return (z, -x, y, -w)


def apply_deadzone(v, dz=STICK_DEADZONE):
    """Deadzone that rescales the remainder, so motion still starts from zero.

    Subtracting the deadzone without rescaling would make the stick jump to
    `dz` of output the moment it passes the threshold.
    """
    if abs(v) < dz:
        return 0.0
    return (v - dz * (1.0 if v > 0 else -1.0)) / (1.0 - dz)
