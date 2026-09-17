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

# The camera's own geometry, published by the ZED wrapper beside each image
# stream. The v2 Unity viewer places each eye from these measured intrinsics
# instead of a field-of-view slider, so they are worth carrying: CameraInfo.P
# is already the RECTIFIED projection, which is exactly the form the viewer
# wants (backends/gvlink.zed_camera_params).
TOPIC_STEREO_LEFT_INFO = os.environ.get(
    "QUEST_STEREO_LEFT_INFO", TOPIC_STEREO_LEFT.rsplit("/", 1)[0] + "/camera_info")
TOPIC_STEREO_RIGHT_INFO = os.environ.get(
    "QUEST_STEREO_RIGHT_INFO", TOPIC_STEREO_RIGHT.rsplit("/", 1)[0] + "/camera_info")

# Shown in the headset's robot picker, and how the beacon identifies this rig.
ROBOT_NAME = os.environ.get("GIAVA_ROBOT_NAME") or os.environ.get("QUEST_ROBOT_NAME", "trossen")

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
# Base driving and the lift. One stick each.
#
# LEFT stick owns the base: forward/back drives, left/right rotates -- the
# whole 2D plane on one thumb, the way a differential drive is normally driven.
# RIGHT stick forward/back is the scissor lift's z velocity. The lift hardware
# is not connected yet; the topic it publishes to (/slate/lift/cmd_velocity)
# is currently served by the SIMULATED lift_agent, so the stick moves a number,
# not a machine, until the lift is wired in -- and nothing here changes when
# it is.
BASE_MAX_VEL_X = float(os.environ.get("QUEST_BASE_MAX_X", 0.25))   # m/s
BASE_MAX_VEL_Z = float(os.environ.get("QUEST_BASE_MAX_Z", 0.6))    # rad/s
# Stick pushed LEFT should turn the rig LEFT (+yaw in REP-103). If the app
# reports stick-left as positive x that needs a sign flip; -1 here does it
# without touching code.
TURN_SIGN = float(os.environ.get("QUEST_TURN_SIGN", -1.0))
LIFT_MAX_VEL = float(os.environ.get("QUEST_LIFT_MAX_VEL", 0.05))   # m/s

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
ARM_NS_LEFT = os.environ.get("QUEST_ARM_NS_LEFT", "/left_arm")
ARM_NS_RIGHT = os.environ.get("QUEST_ARM_NS_RIGHT", "/right_arm")
BASE_NS = os.environ.get("QUEST_BASE_NS", "/slate")

# Hand motion -> EE motion amplification. giava ran 1.35 for their workspace;
# 1.0 is the safe default until this rig's own value is dialled in by feel.
POSITION_SCALE = float(os.environ.get("QUEST_POS_SCALE", 1.0))

# The camera arm is scaled separately and LOWER than the hands (giava ran 0.6):
# a head sweeps further than a hand does for the same intent, and the operator
# is looking through this one, so amplifying it makes the view swim. Nudged up
# from 0.6 because it barely moved in practice -- small steps here, since this
# is the arm whose motion you feel as the world moving rather than as a tool.
CAM_POSITION_SCALE = float(os.environ.get("QUEST_CAM_POS_SCALE", 0.68))

# How far the end effector TURNS per unit of hand/head rotation. Separate from
# the position scales above because the two do not want the same number: a hand
# holding a gripper wants 1:1, while the camera arm is amplified so the view can
# look further than a comfortable neck turn.
ROTATION_SCALE = float(os.environ.get("QUEST_ROT_SCALE", 1.0))
CAM_ROTATION_SCALE = float(os.environ.get("QUEST_CAM_ROT_SCALE", 1.5))

# Operator-to-rig yaw, degrees, for the hands and the head respectively. The
# SESSION yaw (which way the app's arbitrary world points) is measured from
# your gaze at engage time -- see teleop_map.session_yaw_remap -- so all this
# has to say is how you STAND relative to the rig: 0 when you face the same
# way as the rig's +x (standing behind it), 180 if you face it head-on.
ARM_REMAP_YAW_DEG = float(os.environ.get("QUEST_ARM_REMAP_YAW_DEG", 0.0))
CAM_REMAP_YAW_DEG = float(os.environ.get("QUEST_CAM_REMAP_YAW_DEG", 0.0))

# The active-vision arm. Driven by HEAD pose, not a controller, and engaged
# whenever either hand is engaged -- you want the camera to follow you while
# your hands are busy, and to stop when you let go of both.
MIDDLE_NS = os.environ.get("QUEST_MIDDLE_NS", "/middle")

# Trigger -> gripper. Released, the fingers are STAGED with a position command
# (cmd_gripper, metres). Squeezed past the deadzone, the trigger commands a
# closing FORCE (cmd_grip_force, newtons, negative = close), analog between the
# two bounds below.
#
# Force, not position, for the squeeze -- this is the WXAI equivalent of the
# DYNAMIXEL current_based_position + Current_Limit register trick the older
# rigs needed. Commanding a position onto an object the fingers cannot pass
# through gives the controller a growing following error, which it calls a
# fault and drops the arm to idle mid-grasp; commanding a force stops the
# finger wherever the object is, squeezing exactly as hard as asked.
GRIPPER_OPEN = 0.04             # metres; the controller's enforced limit
GRIPPER_CLOSED = 0.0
TRIGGER_DEADZONE = 0.05
GRASP_FORCE_MIN_N = float(os.environ.get("QUEST_GRASP_MIN_N", 5.0))
GRASP_FORCE_MAX_N = float(os.environ.get("QUEST_GRASP_MAX_N", 40.0))
# Releasing the trigger pushes the fingers APART with a force, it does not
# command an opening. Both directions are then the same control mode, so the
# gripper never changes mode mid-session -- a mode write drops the arm's
# control loop for an instant, which is felt as the whole arm twitching, and
# commanding the open POSITION drove the fingers into their mechanical stop and
# faulted the entire arm with "Joint 6 position limit exceeded".
OPEN_FORCE_N = float(os.environ.get("QUEST_OPEN_N", 15.0))

# ---------------------------------------------------------------------------
# One button parks the whole rig: every arm to its saved 'rest' pose, in joint
# space, by name. "right" = B, "left" = Y, "off" = no such button.
#
# Ignored while an arm is engaged, which is the safety: you cannot fire it
# mid-motion with a thumb, only after letting go -- and "let go, then park" is
# the order you would do it in anyway.
REST_BUTTON = os.environ.get("QUEST_REST_BUTTON", "right").strip().lower()
REST_POSE = os.environ.get("QUEST_REST_POSE", "rest")

# ...and then ends the session, so one button is the whole shutdown: park the
# rig, release it, exit. Set QUEST_REST_QUITS=false to have the button park the
# arms and leave teleop running.
#
# THE WAIT IS NOT OPTIONAL. A named move is executed BY THE AGENT and only
# while the arm stays enabled -- quitting immediately would publish enable=false
# mid-move and drop each arm wherever it had got to, which is the opposite of
# parking it. So the button sends the pose, keeps the link alive long enough
# for the slowest arm to finish, and only then releases.
# HELD, not tapped. B and Y both open the Unity app's own menu, so a single tap
# is something you do by accident several times a session -- and the first time
# it happened it parked all three arms and ended teleop, which read as the link
# dying. A double press was the first fix and it is the wrong shape for this
# button: the app opens its menu on the first press, so the second one is aimed
# at a screen that just appeared, and whether it registers depends on what the
# menu did with it. A HOLD has no such conflict -- the menu opens, you keep
# holding, and the rig parks. Set 0 to fire on release of any press.
REST_HOLD_S = float(os.environ.get("QUEST_REST_HOLD_S", 1.5))

REST_QUITS = os.environ.get("QUEST_REST_QUITS", "true").strip().lower() == "true"
REST_QUIT_WAIT_S = float(os.environ.get("QUEST_REST_QUIT_WAIT", 12.0))


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
