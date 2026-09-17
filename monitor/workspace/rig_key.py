#!/usr/bin/env python3
"""Drive the whole rig from one keyboard, all components at once.

    ./rig_key.py                 # everything, disabled until you enable it
    ./rig_key.py --step 0.005    # finer arm steps
    ./rig_key.py --dry-run       # print, publish nothing

KEY MAP -- no modes, no selection. Each key belongs to one component, so you can
move the base and both arms in the same second without switching anything.

    ENABLE     1 left arm   2 middle arm   3 right arm   0 base torque
               SPACE  toggle EVERYTHING on/off  (also the panic key)
    ZERO       4 / 5 / 6    make that arm's current position read (0, 0, 0)
    RESET      7 / 8 / 9    restart that arm's agent after a fault

               +x +y +z    -x -y -z     gripper (force)
    LEFT ARM    q  w  e     a  s  d      z open   x close
    MIDDLE      r  t  y     f  g  h      (none)
    RIGHT ARM   u  i  o     j  k  l      n open   m close

    ROTATE     `  toggles: the arm keys then turn the end effector about the
                  world axes (q/a = x roll, w/s = y pitch, e/d = z yaw, and the
                  same shape for the other clusters), 5 deg per press.
    BASE       arrow keys -- up/down travel, left/right turn
    LIFT       , down   . up
    POSES      shift-K  SAVE every arm's current pose under a name you type
                        ('start' and 'rest' are the two that matter). All three
                        arms at once, because a start configuration is a
                        property of the RIG -- a set with one arm missing is a
                        set you cannot return to. Saving does not require the
                        arm to be enabled.
               shift-R  every enabled arm -> its 'rest' pose
               shift-S  every enabled arm -> its 'start' pose
               Pressed TWICE within 1.5 s -- a whole-rig move must be
               deliberate. Arms are skipped (and named) if disabled or if the
               pose was never saved for them ('s' in rig_debug saves one).
               Save them once with shift-K, then --start-on-launch moves the
               rig to 'start' as this tool comes up and --rest-on-quit returns
               it to 'rest' as it exits (both off by default, or set
               RIG_START_ON_LAUNCH=1 / RIG_REST_ON_QUIT=1). Rename the poses
               with --start-pose / --rest-pose.
    [ / ]      step size          ESC, ^C   quit

    JOINT MODE  TAB  cycle: off -> left -> middle -> right -> off
                1-6 joint 1-6 positive,  shift (!@#$%^) negative
                p   run this arm's saved 'home' pose
    Use it when Cartesian keys are refused: at a singularity the controller
    will not solve IK, so q/w/e do nothing and joint space is the way out.
    While joint mode is on the digits are joints -- SPACE, arrows and RESET
    (7/8/9) still work.

TWO KEYS PER AXIS, NO MODIFIER. Positive on one row, negative on the row
directly below, so each pair sits under the same finger: q/a is left x, w/s is
left y. An earlier version used shift for negative, which meant an extra key on
every negative move of a control you use constantly.

THE CLUSTERS ARE SPATIAL. qwe / rty / uio run left-to-right in the same order
the arms sit on the robot. With no selection mode, muscle memory is the only
thing keeping you off the wrong arm, so the layout is built to create the right
one.

THE GRIPPER IS FORCE-CONTROLLED, not position. One press squeezes at
--grip-force newtons and stops wherever the object is. Position control on a
held object is a following error waiting to happen, and applies whatever force
the position loop felt like on the way there.

NOTHING MOVES UNTIL ENABLED. Every component starts disabled. Enabling an arm
anchors its target on the arm's measured pose, so the first command sent is
exactly where the arm already is -- enabling is never a move. Enabling the base
only powers its motors; it does not command a velocity.

THE BASE IS DEAD-MAN, THE ARMS ARE NOT. A held base or lift key keeps it moving
and it stops ~0.3 s after release, because terminal key-repeat is the only
"still pressed" signal available. An arm key is discrete: press once, the target
moves one step, and it stays there. A base that keeps rolling because a key
repeated is a collision; an arm holding a commanded pose is just an arm.

HOLDING an arm key does move it continuously, and the target is kept within
JOG_LEAD_M of where the arm actually is so that it stops promptly on release.
The alternative -- letting key repeat pile up 30 steps a second onto a target
the arm chases at a capped speed -- is how an arm ends up still travelling
seconds after you let go.

THE BASE CANNOT STRAFE. It is differential drive: two driven wheels, four
passive casters. g/G rotate rather than sliding sideways, because no mechanism
could slide sideways. The rig's real z is the scissor lift.

MUST be run on a real terminal (`docker compose exec monitor bash`), not
`exec -T` -- it puts the tty in cbreak mode to read single keys.
"""
import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty

import control_lock
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool

UP, DOWN, RIGHT, LEFT = "\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D"
BASE_NS = os.environ.get("RIG_BASE_NS", "/slate")

# name -> (namespace, has_gripper). Order is the order they appear on screen.
ARMS = {
    "left":   (os.environ.get("RIG_LEFT_NS", "/left_arm"), True),
    "middle": (os.environ.get("RIG_MIDDLE_NS", "/middle"), False),
    "right":  (os.environ.get("RIG_RIGHT_NS", "/right_arm"), True),
}

# key -> (arm, axis, sign). TWO KEYS PER AXIS, no modifier.
#
# Each arm gets a row for positive and the row below it for negative, so the
# pairs sit vertically under the same finger: q/a is left x, w/s is left y, and
# so on. Shift+letter was the first design and it was worse -- an extra key for
# every negative move, on a control you use constantly.
#
# The three clusters run left-to-right in the same order the arms sit on the
# robot. With no selection mode, muscle memory is the only thing keeping you off
# the wrong arm, so the layout is built to create the right one.
ARM_KEYS = {}
for _arm, _pos, _neg in (("left", "qwe", "asd"),
                         ("middle", "rty", "fgh"),
                         ("right", "uio", "jkl")):
    for _axis, _p, _n in zip("xyz", _pos, _neg):
        ARM_KEYS[_p] = (_arm, _axis, +1)
        ARM_KEYS[_n] = (_arm, _axis, -1)

# Gripper: FORCE, not position. open = push apart, close = squeeze at N newtons
# and stop wherever the object is. See arm_agent._on_grip_force for why this
# matters -- position control on a held object is a following error waiting to
# happen, and applies whatever force the position loop felt like on the way.
GRIPPER_KEYS = {"z": ("left", +1), "x": ("left", -1),
                "n": ("right", +1), "m": ("right", -1)}

# The base moved to the arrow keys because f/g/h are now the middle arm's
# negative row. Arrows are also the right shape for it: up/down is travel,
# left/right is turn.
BASE_KEYS = {UP: ("x", +1), DOWN: ("x", -1),
             LEFT: ("yaw", +1), RIGHT: ("yaw", -1)}

# Lift on , and . -- it has no hardware yet, so it gets the leftover keys.
LIFT_KEYS = {",": -1, ".": +1}

# ROTATE MODE: backtick toggles. While on, the arm clusters rotate the end
# effector about the WORLD axes instead of translating -- q/a roll (x), w/s
# pitch (y), e/d yaw (z) for the left arm, same shape for the other two. Same
# keys, so muscle memory carries over; the status line says [ROT] while it is
# on. Six DOF on one keyboard without doubling the key map.
ROT_MODE_KEY = "`"
DEFAULT_ROT_STEP = math.radians(5.0)
JOG_LEAD_RAD = 0.35

# Number row arms one component each; SPACE does all of them at once.
ENABLE_KEYS = {"1": "left", "2": "middle", "3": "right"}

# Same digits, shifted row: restart an arm's agent after a fault, and re-zero
# its origin. Kept on the number row because they are per-component actions
# like enabling, not motion.
RESET_KEYS = {"7": "left", "8": "middle", "9": "right"}
ZERO_KEYS = {"4": "left", "5": "middle", "6": "right"}

# ---- joint mode ----------------------------------------------------------
# Cartesian control has one failure it cannot argue with: at a singularity the
# controller refuses the IK outright, so every q/w/e press is rejected and the
# arm will not move at all. Joint space has no IK and therefore no singularity,
# which makes it the only way out -- and the reason this mode exists.
#
# It is MODAL, unlike everything else here, because the digits are already
# spoken for by enable/zero. TAB cycles which arm is selected; the digits mean
# joints only while one is. That shadows ZERO (4/5/6) and ENABLE (1/2/3) for
# the duration -- SPACE, the arrows and RESET (7/8/9) all keep working, so the
# panic key is never behind a mode.
JOINT_MODE_KEY = "\t"
JOINT_HOME_KEY = "p"          # only in joint mode, where "which arm" is unambiguous

# Shift+digit for the negative direction. On the number row shift produces a
# DIFFERENT CHARACTER (!@#$%^), so unlike shift+letter it survives cbreak mode
# without any modifier tracking -- the terminal has already done the work.
# The wxai has six revolute joints; the gripper is not one of them and is never
# jogged here. Hardcoded rather than imported from arm_config, which lives in
# the arm image and is not on this container's path.
NUM_ARM_JOINTS = 6

JOINT_POS = "123456"
JOINT_NEG = "!@#$%^"

DEFAULT_JOINT_STEP = 0.05     # rad/press, ~2.9 deg
MAX_JOINT_STEP = 0.35

GRIPPER_OPEN, GRIPPER_CLOSED = 0.04, 0.0
REPUBLISH_HZ = 20.0
HOLD_S = 0.3          # dead-man window for base and lift

# How far this tool's accumulated target may get ahead of where the arm
# actually is. THE ACCUMULATOR IS THE THING THAT RUNS AWAY: each key press adds
# --step to the target, terminal key repeat fires ~30 times a second, and the
# arm follows at a capped velocity -- so holding a jog key for one second asks
# for ~30 cm of travel the arm cannot have delivered. The excess is not
# "queued motion", it is a lie about where the operator wants the arm, and it
# has two bad consequences: the arm keeps moving for seconds after the key is
# released, and the target drifts so far from the arm that the agent's guards
# start firing (it used to refuse every command from then on -- see
# arm_agent.py's MAX_LEAD_M note).
#
# Clamping the target to a fixed lead makes a held key mean what it looks like:
# the target sits JOG_LEAD_M in front of the arm, the arm chases it at the
# velocity that error commands, and releasing the key stops it within that same
# distance. Press-once still steps exactly --step, because one step is well
# inside the lead.
JOG_LEAD_M = 0.06

# Whole-rig pose recall: capital letters, pressed TWICE within this window.
# Capitals because every convenient lowercase key is a motion key, and a pose
# recall moves EVERY enabled arm several seconds' worth -- exactly the command
# that must never fire off a slipped finger. The confirm makes it deliberate.
POSE_ALL_KEYS = {"R": "rest", "S": "start"}      # key -> args attribute
POSE_CONFIRM_S = 1.5

# Capital K asks for a name and records EVERY reporting arm under it. Handled
# in the main loop rather than in handle(), because it has to put the terminal
# back into line mode to read a word, and only the loop owns the terminal.
SAVE_KEY = "K"
PROMPT_SAVE = object()


def parse_pose_names(data):
    """<ns>/pose_names carries a JSON list. The old split-on-comma parse left
    brackets and quotes glued to the names, so 'home' never matched '[\"home\"'
    -- which made every name lookup in this tool silently fail."""
    try:
        v = json.loads(data)
        return [str(x) for x in v] if isinstance(v, list) else []
    except ValueError:
        return [x.strip(' "[]') for x in data.split(",") if x.strip(' "[]')]


def quat_mul(a, b):
    """Hamilton product, (x, y, z, w) convention -- the one ROS uses."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_about(axis, angle):
    """Rotation of `angle` about world x, y or z."""
    h = angle / 2.0
    q = [0.0, 0.0, 0.0, math.cos(h)]
    q["xyz".index(axis)] = math.sin(h)
    return q


def quat_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_angle(a, b):
    d = sum(x * y for x, y in zip(a, b))
    return 2.0 * math.acos(min(1.0, abs(d)))


def quat_slerp_toward(frm, to, frac):
    d = sum(x * y for x, y in zip(frm, to))
    if d < 0.0:
        to = [-v for v in to]; d = -d
    if d > 0.9995:
        return quat_norm([f + (t - f) * frac for f, t in zip(frm, to)])
    th = math.acos(min(1.0, d)); s = math.sin(th)
    w0, w1 = math.sin((1.0 - frac) * th) / s, math.sin(frac * th) / s
    return quat_norm([f * w0 + t * w1 for f, t in zip(frm, to)])


def quat_norm(q):
    """Renormalise. Composing many small rotations lets rounding error grow, and
    a quaternion that has drifted off the unit sphere is no longer a rotation --
    it shows up as the wrist slowly scaling or skewing rather than turning."""
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return [v / n for v in q]


class RigKey(Node):
    def __init__(self, args):
        super().__init__("rig_key")
        self.args = args
        self.step = args.step

        self.measured, self.target, self.enabled, self.gripper = {}, {}, {}, {}
        # Per arm: has a jog key been pressed since it was enabled? Nothing
        # Cartesian is streamed until then -- see toggle_arm.
        self.jogged = {}
        self.pub_cmd, self.pub_en, self.pub_grip, self.pub_grip_pos = {}, {}, {}, {}
        self.pub_zero, self.pub_reset, self.pub_save = {}, {}, {}
        self.pub_joints, self.pub_name = {}, {}
        # Live joint vector per arm, from <ns>/joint_states. cmd_joints takes
        # ABSOLUTE positions, so a jog is measured + delta -- which means a jog
        # is only possible once this arm has actually reported.
        self.joints = {}
        # What each arm says it can go to, from <ns>/pose_names. Absent until
        # that arm reports; a missing entry means "unknown", not "none".
        self.pose_names = {}
        # What each AGENT says about itself, from <ns>/active -- as opposed to
        # self.enabled, which is only what this tool believes. The two diverge
        # whenever an agent restarts or disarms itself, and until this existed
        # the tool went on publishing to an agent that was ignoring it, with no
        # sign on screen. Every key press looked like it did nothing.
        self.agent_active = {}
        self.joint_arm = None            # None = digits mean enable/zero
        self.joint_step = DEFAULT_JOINT_STEP
        self.rot_mode = False
        self.rot_step = DEFAULT_ROT_STEP
        for name, (ns, has_grip) in ARMS.items():
            self.create_subscription(
                PoseStamped, f"{ns}/ee_pose", lambda m, n=name: self._on_ee(n, m), 1)
            self.create_subscription(
                JointState, f"{ns}/joint_states",
                lambda m, n=name: self.joints.__setitem__(n, list(m.position)), 1)
            self.pub_cmd[name] = self.create_publisher(PoseStamped, f"{ns}/cmd_pose", 1)
            self.pub_joints[name] = self.create_publisher(JointState, f"{ns}/cmd_joints", 1)
            self.pub_name[name] = self.create_publisher(String, f"{ns}/cmd_pose_name", 1)
            self.create_subscription(
                String, f"{ns}/pose_names",
                lambda m, n=name: self.pose_names.__setitem__(
                    n, parse_pose_names(m.data)), 1)
            self.create_subscription(
                Bool, f"{ns}/active",
                lambda m, n=name: self.agent_active.__setitem__(n, m.data), 1)
            self.pub_en[name] = self.create_publisher(Bool, f"{ns}/enable", 1)
            self.pub_zero[name] = self.create_publisher(Bool, f"{ns}/zero", 1)
            self.pub_reset[name] = self.create_publisher(Bool, f"{ns}/reset", 1)
            self.pub_save[name] = self.create_publisher(String, f"{ns}/save_pose", 1)
            if has_grip:
                self.pub_grip[name] = self.create_publisher(
                    Float32, f"{ns}/cmd_grip_force", 1)
                self.pub_grip_pos[name] = self.create_publisher(
                    Float32, f"{ns}/cmd_gripper", 1)
            self.enabled[name] = False
            self.gripper[name] = GRIPPER_OPEN

        self.base_cmd = (0.0, 0.0)
        self.base_stamp = 0.0
        self.lift_vel = 0.0
        self.lift_stamp = 0.0
        self.torque = False
        self.pub_base = self.create_publisher(Twist, f"{BASE_NS}/cmd_vel_teleop", 1)
        self.pub_lift = self.create_publisher(Float32, f"{BASE_NS}/lift/cmd_velocity", 1)
        self.cli_torque = self.create_client(SetBool, f"{BASE_NS}/set_motor_torque_status")

        self.create_timer(1.0 / REPUBLISH_HZ, self._tick)

    def _on_ee(self, name, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.measured[name] = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    # ---- enable ----------------------------------------------------------
    def toggle_arm(self, name):
        ns = ARMS[name][0]
        if self.enabled[name]:
            self.enabled[name] = False
            self.target.pop(name, None)
            self._enable(name, False)
            return f"{name} DISABLED"
        if name not in self.measured:
            return f"{name}: no {ns}/ee_pose -- is its agent running?"
        pos, quat = self.measured[name]
        # Anchor on the measurement, so enabling is not a move -- and DO NOT
        # START STREAMING IT. Streaming the anchor at 20 Hz put the agent into
        # a Cartesian velocity servo with nothing to do, and at a pose with a
        # straight wrist (a singularity, which the saved start pose is) the
        # controller refuses every one of those commands: "Please avoid
        # operating near the current position in Cartesian space", eight times
        # a second, until the fault-loop breaker disarmed the arm. Enabling now
        # only arms; the first jog key starts the stream. Named moves (S S,
        # R R) are joint-space and never needed the stream at all.
        self.target[name] = (list(pos), list(quat))
        self.jogged[name] = False
        self.enabled[name] = True
        self._enable(name, True)
        return f"{name} ENABLED at ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"

    def toggle_base(self, want=None):
        want = (not self.torque) if want is None else want
        if self.args.dry_run:
            self.torque = want
            return f"base torque {'ON' if want else 'OFF'} (dry run)"
        if not self.cli_torque.service_is_ready():
            # A tenth of a second, not a whole one. wait_for_service(1.0) blocks
            # this thread, and while it blocks the republish timer does not run
            # -- so pressing 0 (or SPACE, which torques the base too) with the
            # base unplugged timed out every ARM at 300 ms and dropped them all
            # to idle. A missing base must cost a message, not the arms.
            self.cli_torque.wait_for_service(timeout_sec=0.1)
        if not self.cli_torque.service_is_ready():
            return "base torque: service missing -- is slate-base running?"
        req = SetBool.Request()
        req.data = want
        fut = self.cli_torque.call_async(req)
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            # Drain, so the arms keep being republished while we wait on the
            # base's reply.
            drain_callbacks(self, seconds=0.02)
        if not fut.done():
            return "base torque: timed out"
        res = fut.result()
        if res is not None and res.success:
            self.torque = want
            return f"base torque {'ON' if want else 'OFF'}"
        return f"base torque FAILED: {getattr(res, 'message', '?')}"

    def toggle_all(self):
        """SPACE. On if anything is off; otherwise everything off."""
        any_off = (not self.torque) or any(not v for v in self.enabled.values())
        notes = []
        for name in ARMS:
            if self.enabled[name] != any_off:
                notes.append(self.toggle_arm(name))
        if self.torque != any_off:
            notes.append(self.toggle_base(any_off))
        return ("ALL ON: " if any_off else "ALL OFF: ") + "; ".join(notes)[:120]

    def zero(self, name):
        """Make this arm's current position read as (0, 0, 0)."""
        if self.args.dry_run:
            return f"{name} zero (dry run)"
        m = Bool()
        m.data = True
        for _ in range(3):
            self.pub_zero[name].publish(m)
            drain_callbacks(self, seconds=0.02)
        # The anchor is now in different coordinates, so drop it -- re-enable to
        # pick up a fresh one rather than stepping from a stale reference.
        if self.enabled.get(name):
            self.enabled[name] = False
            self.target.pop(name, None)
            self._enable(name, False)
            return f"{name} zeroed -- DISABLED, press its number to re-enable"
        return f"{name} zeroed"

    def reset(self, name):
        """Restart the arm's agent, clearing a latched controller fault."""
        if self.args.dry_run:
            return f"{name} reset (dry run)"
        m = Bool()
        m.data = True
        for _ in range(3):
            self.pub_reset[name].publish(m)
            drain_callbacks(self, seconds=0.02)
        self.enabled[name] = False
        self.target.pop(name, None)
        return (f"{name} RESET -- its container restarts and reconnects "
                "(~10 s). The arm holds position throughout.")

    def _disarmed_by_agent(self, name):
        """Message if this tool thinks `name` is armed but the agent disagrees.

        The agent is the authority: it holds the arm. It disarms itself when it
        loses the controller, when the container restarts, and when a fault
        drops it -- none of which this tool would otherwise notice.
        """
        if not self.enabled.get(name):
            return None
        if self.agent_active.get(name) is not False:
            return None          # agreed, or the agent has not reported yet
        key = [k for k, v in ENABLE_KEYS.items() if v == name][0]
        self.enabled[name] = False
        self.target.pop(name, None)
        return (f"{name} was DISARMED by its agent (restart or fault) -- "
                f"commands were going nowhere. Press {key} to re-arm.")

    # ---- joint mode ------------------------------------------------------
    def cycle_joint_mode(self):
        """TAB: off -> left -> middle -> right -> off."""
        order = [None] + list(ARMS)
        self.joint_arm = order[(order.index(self.joint_arm) + 1) % len(order)]
        if self.joint_arm is None:
            return "joint mode OFF -- digits are enable/zero again"
        have = self.joint_arm in self.joints
        warn = "" if have else "  (no joint_states yet -- is it up?)"
        return (f"JOINT MODE: {self.joint_arm}   1-6 = joint 1-6 +, "
                f"shift = -,  step {math.degrees(self.joint_step):.1f}deg,  "
                f"p = home{warn}")

    def move_joint(self, index, sign):
        stale = self._disarmed_by_agent(self.joint_arm)
        if stale:
            return stale
        name = self.joint_arm
        if not self.enabled.get(name):
            key = [k for k, v in ENABLE_KEYS.items() if v == name][0]
            return (f"{name} not enabled -- TAB out of joint mode, press {key}, "
                    "then TAB back")
        cur = self.joints.get(name)
        if cur is None:
            return f"{name}: no joint_states yet -- cannot jog blind"
        if index >= len(cur):
            return f"{name}: only {len(cur)} joints reported"

        # A joint move and a streamed Cartesian target fight each other, so drop
        # the Cartesian one first. The agent does this too, but doing it here
        # keeps this tool's idea of the target from going stale behind the arm.
        self.target.pop(name, None)

        want = list(cur[:NUM_ARM_JOINTS])
        want[index] += sign * self.joint_step
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in want]
        if not self.args.dry_run:
            self.pub_joints[name].publish(msg)
        return (f"{name} joint {index + 1} {'+' if sign > 0 else '-'} -> "
                f"{math.degrees(want[index]):+.1f}deg")

    # ---- motion ----------------------------------------------------------
    def move_arm(self, name, axis, sign):
        stale = self._disarmed_by_agent(name)
        if stale:
            return stale
        if not self.enabled.get(name):
            return f"{name} not enabled -- press {[k for k, v in ENABLE_KEYS.items() if v == name][0]}"
        anchored = self._anchor(name)
        if anchored is None:
            return f"{name}: no ee_pose yet -- is its agent running?"
        pos, quat = anchored
        if self.rot_mode:
            # Pre-multiply: a rotation about a WORLD axis goes on the left.
            quat = quat_norm(quat_mul(quat_about(axis, sign * self.rot_step), list(quat)))
            meas = self.measured.get(name)
            if meas is not None:
                ang = quat_angle(meas[1], quat)
                if ang > JOG_LEAD_RAD:
                    quat = quat_slerp_toward(quat, list(meas[1]), 1.0 - JOG_LEAD_RAD / ang)
            self.target[name] = (list(pos), quat)
            self.jogged[name] = True
            self._send_arm(name)
            return (f"{name} rot {axis}{'+' if sign > 0 else '-'} "
                    f"({math.degrees(self.rot_step):.0f} deg)")
        pos = list(pos)
        pos["xyz".index(axis)] += sign * self.step
        pos = self._clamp_lead(name, pos)
        self.target[name] = (pos, quat)
        self.jogged[name] = True
        self._send_arm(name)
        note = ""
        if name in self.measured:
            d = math.dist(pos, self.measured[name][0])
            if d > 0.02:
                note = f" ({d * 1000:.0f}mm behind)"
        return f"{name} {axis}{'+' if sign > 0 else '-'} -> ({pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}){note}"

    def _anchor(self, name):
        """This tool's target for `name`, created from the measurement if need be.

        A pose recall drops the streaming target on purpose (a named move and a
        streamed one would otherwise pull the arm in two directions). The jog
        keys then read self.target[name] and raised KeyError -- which is not
        caught anywhere in the key loop, so pressing a direction key after
        shift-R or shift-S killed the whole controller. Re-anchoring on the
        measured pose is both the crash fix and the right behaviour: after a
        pose move the arm IS somewhere new, and jogging should start from
        there rather than from wherever the target used to be.
        """
        if name in self.target:
            return self.target[name]
        meas = self.measured.get(name)
        if meas is None:
            return None
        self.target[name] = (list(meas[0]), list(meas[1]))
        return self.target[name]

    def _clamp_lead(self, name, pos):
        """Keep the target within JOG_LEAD_M of the measured pose.

        Without a measurement there is nothing to clamp against, so the target
        is left alone -- that only happens before the arm's first ee_pose, when
        nothing has been commanded yet either.
        """
        meas = self.measured.get(name)
        if meas is None:
            return pos
        mp = meas[0]
        gap = math.dist(pos, mp)
        if gap <= JOG_LEAD_M or gap < 1e-9:
            return pos
        k = JOG_LEAD_M / gap
        return [mp[i] + (pos[i] - mp[i]) * k for i in range(3)]

    def move_base(self, axis, sign):
        if axis == "x":
            self.base_cmd = (sign * self.args.linear, self.base_cmd[1])
        else:
            self.base_cmd = (self.base_cmd[0], sign * self.args.angular)
        self.base_stamp = time.monotonic()
        lin, ang = self.base_cmd
        warn = "" if self.torque else "  (TORQUE OFF -- press 0)"
        return f"base lin {lin:+.2f} ang {ang:+.2f}{warn}"

    def move_lift(self, sign):
        self.lift_vel = sign * self.args.lift_speed
        self.lift_stamp = time.monotonic()
        return f"lift {'up' if sign > 0 else 'down'} {abs(self.lift_vel):.3f} m/s"

    def grip(self, name, sign):
        """Binary open/close, force-controlled.

        Not a position step: one press commands a squeeze (or a push apart) and
        the gripper holds it. Closing at --grip-force newtons stops wherever the
        object is, so an object too big to fully close on is gripped rather than
        crushed, and the controller never sees a following error.
        """
        stale = self._disarmed_by_agent(name)
        if stale:
            return stale
        if name not in self.pub_grip:
            return f"{name} has no gripper"
        # No enable needed: the agent applies gripper commands in any state.
        # FORCE BOTH WAYS -- open is +N, close is -N -- so the agent never has
        # to change the gripper's mode, which is what made the arm drop and
        # catch itself on every press. Safe to push open against the stop now
        # that the gripper's limit covers it (arm_config.JOINT_LIMIT_OVERRIDES).
        force = sign * abs(self.args.grip_force)
        self.gripper[name] = force
        if not self.args.dry_run:
            m = Float32()
            m.data = float(force)
            self.pub_grip[name].publish(m)
        return (f"{name} gripper {'OPEN' if sign > 0 else 'CLOSE'} "
                f"at {abs(force):.0f} N")

    # ---- named poses -----------------------------------------------------
    def _go_pose(self, name, pose):
        """Send one arm to a named pose, with every reason it can't said aloud."""
        stale = self._disarmed_by_agent(name)
        if stale:
            return stale
        if not self.enabled.get(name):
            key = [k for k, v in ENABLE_KEYS.items() if v == name][0]
            return f"{name} not enabled -- press {key} first"
        names = self.pose_names.get(name)
        if names is not None and pose not in names:
            return (f"{name} has no pose {pose!r} (has: {', '.join(names) or 'none'}). "
                    "Drive it there and save: 's' in rig_debug, or pose.py save")
        self.target.pop(name, None)      # a pose move cancels the Cartesian target
        if not self.args.dry_run:
            self.pub_name[name].publish(String(data=pose))
        return f"{name} -> {pose!r}  (joint-space, several seconds -- WATCH IT)"

    def go_home(self, name):
        return self._go_pose(name, "home")

    def send_pose_all(self, pose, enable_first=False):
        """Send every arm to `pose`. Returns (went, skipped-with-reasons).

        enable_first arms anything that is disarmed first: an agent ignores a
        named move while disabled, so a whole-rig recall has to arm each arm
        for the length of the move. Used by the R/S keys and by the quit path.
        """
        went, skipped = [], []
        # Arm first, THEN wait for each agent to say so, THEN send the name.
        # enable and cmd_pose_name are different topics and nothing orders
        # them: the middle arm logged "pose 'start' ignored -- not enabled"
        # 0.3 ms before "enabled". The agents publish <ns>/active at 20 Hz, so
        # a fresh True is the proof that the enable has landed.
        newly = []
        if enable_first:
            for name in ARMS:
                self._disarmed_by_agent(name)
                if not self.enabled.get(name) and name in self.measured:
                    self.enabled[name] = True
                    self._enable(name, True)
                    newly.append(name)
            if newly:
                end = time.monotonic() + 1.5
                while time.monotonic() < end and not all(
                        self.agent_active.get(n) for n in newly):
                    drain_callbacks(self)
                    time.sleep(0.02)
                late = [n for n in newly if not self.agent_active.get(n)]
                if late:
                    skipped.append("not confirmed enabled: " + ", ".join(late))
                    for n in late:
                        self.enabled[n] = False
        for name in ARMS:
            if self._disarmed_by_agent(name) and not enable_first:
                skipped.append(f"{name}: disabled")
                continue
            if not self.enabled.get(name):
                skipped.append(f"{name}: " + ("no ee_pose" if name not in self.measured
                                              else "disabled"))
                continue
            names = self.pose_names.get(name)
            if names is not None and pose not in names:
                skipped.append(f"{name}: no {pose!r} saved")
                continue
            # Drop the streaming target: a named move and a streamed one would
            # fight for the arm. _anchor() re-creates it on the next jog.
            self.target.pop(name, None)
            self.jogged[name] = False
            if not self.args.dry_run:
                self.pub_name[name].publish(String(data=pose))
            went.append(name)
        return went, skipped

    def wait_moving(self, seconds):
        """Let the agents execute a named move before we touch anything else.

        A named move is carried out BY THE AGENT, but it only runs while the
        arm stays enabled -- publishing enable=false mid-move drops it to idle
        wherever it happens to be. So the quit path has to wait here rather
        than releasing immediately.
        """
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            drain_callbacks(self)
            time.sleep(0.02)

    def save_all(self, name):
        """Record every reporting arm's current pose under one name.

        Deliberately not gated on enable: reading where an arm is is not
        commanding it, and the natural way to author a pose is to put the arm
        where you want it -- by hand with torque off, or by jogging -- and then
        capture it. All three arms at once, because 'start' and 'rest' are
        properties of the RIG, and a set where one arm is missing is a set that
        cannot be returned to.
        """
        name = (name or "").strip()
        if not name:
            return "save cancelled -- no name given"
        # "start_via2 left" saves ONE arm. Waypoints of a hand-guided path are
        # per arm by nature -- the other arm is wherever it was left, and a
        # waypoint recorded there would be replayed as a real move later.
        only = None
        if " " in name:
            name, only = name.split(None, 1)
            only = only.strip().lower()
            if only not in ARMS:
                return f"unknown arm {only!r}; use one of {', '.join(ARMS)}"
        if "/" in name or name.startswith("."):
            return f"refusing {name!r} as a pose name"
        saved, skipped = [], []
        for arm in ARMS:
            if only and arm != only:
                continue
            if arm not in self.joints:
                skipped.append(f"{arm}: not reporting")
                continue
            if not self.args.dry_run:
                self.pub_save[arm].publish(String(data=name))
            saved.append(arm)
        if not saved:
            return f"saved nothing ({'; '.join(skipped)})"
        out = f"saved {name!r} for: {', '.join(saved)}"
        if skipped:
            out += f"   [skipped {'; '.join(skipped)}]"
        return out

    def go_all(self, key, pose):
        """R/S: every ENABLED arm to a named pose. Pressed twice to fire.

        Arms that are disabled or do not know the pose are skipped and named,
        rather than refusing the whole gesture -- 'the two that could go, went'
        is more useful mid-session than all-or-nothing.
        """
        now = time.monotonic()
        pending = getattr(self, "pending_pose", None)
        if not (pending and pending[0] == key and now < pending[1]):
            self.pending_pose = (key, now + POSE_CONFIRM_S)
            return (f"press {key} again to send EVERY enabled arm to {pose!r}")
        self.pending_pose = None
        # enable_first: a whole-rig pose recall arms whatever is disarmed on
        # the way. A disarmed arm is usually one the agent dropped on a fault
        # (overheat, CAN) and the operator has not noticed; making them press
        # 1/2/3 first just added a step to the recovery. The double press is
        # the confirmation.
        went, skipped = self.send_pose_all(pose, enable_first=True)
        if not went:
            return (f"nobody went to {pose!r} ({'; '.join(skipped)}). Enable arms "
                    "first; save missing poses with 's' in rig_debug or pose.py save")
        out = f"ALL -> {pose!r}: {', '.join(went)}"
        if skipped:
            out += f"   [skipped {'; '.join(skipped)}]"
        return out + "  -- WATCH THEM"

    # ---- publishing ------------------------------------------------------
    def _enable(self, name, on):
        if self.args.dry_run:
            return
        m = Bool()
        m.data = bool(on)
        for _ in range(3):          # depth-1: a dropped enable is a stuck arm
            self.pub_en[name].publish(m)

    def _send_arm(self, name):
        if self.args.dry_run or name not in self.target:
            return
        pos, quat = self.target[name]
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in pos]
        m.pose.orientation.x, m.pose.orientation.y = float(quat[0]), float(quat[1])
        m.pose.orientation.z, m.pose.orientation.w = float(quat[2]), float(quat[3])
        self.pub_cmd[name].publish(m)

    def _tick(self):
        now = time.monotonic()
        lin, ang = self.base_cmd
        if now - self.base_stamp > HOLD_S:
            lin, ang = 0.0, 0.0
            self.base_cmd = (0.0, 0.0)
        if now - self.lift_stamp > HOLD_S:
            self.lift_vel = 0.0
        if self.args.dry_run:
            return
        t = Twist()
        t.linear.x, t.angular.z = float(lin), float(ang)
        self.pub_base.publish(t)
        f = Float32()
        f.data = float(self.lift_vel)
        self.pub_lift.publish(f)
        # An engaged arm's target is dropped by its agent after 300 ms of
        # silence, so it has to be repeated even when nothing is pressed --
        # but only once a jog has started the stream (see toggle_arm).
        for name, on in self.enabled.items():
            if on and self.jogged.get(name):
                self._send_arm(name)

    def stop_all(self):
        """Release everything, and SPIN so the messages actually leave.

        Publishing and then destroying the node immediately drops the message:
        these are depth-1 publishers and nothing has flushed them yet. Without
        the spin below, quitting the controller left every arm ENABLED with a
        stale target -- observed directly: rig_key exited, `active` stayed true
        on all three arms.
        """
        for name, on in list(self.enabled.items()):
            if on:
                self.enabled[name] = False
                self._enable(name, False)
        self.base_cmd = (0.0, 0.0)
        self.lift_vel = 0.0
        if not self.args.dry_run:
            for _ in range(5):
                self.pub_base.publish(Twist())
                self.pub_lift.publish(Float32())
                for name in ARMS:
                    self._enable(name, False)
                rclpy.spin_once(self, timeout_sec=0.02)

    def status_line(self, width):
        if self.joint_arm is not None:
            head = f"[JOINT {self.joint_arm} {math.degrees(self.joint_step):.0f}deg] "
        elif self.rot_mode:
            head = f"[ROT {math.degrees(self.rot_step):.0f}deg] "
        else:
            head = ""
        bits = []
        for name in ARMS:
            # '!' is the case worth seeing at a glance: this tool thinks the arm
            # is armed and the agent says it is not, so every key press is being
            # published into a void. '?' means the agent has not reported yet.
            live = self.agent_active.get(name)
            if self.enabled[name] and live is False:
                mark = "!"
            elif self.enabled[name]:
                mark = "*" if live else "?"
            else:
                mark = "-"
            bits.append(f"{name[0].upper()}{mark}")
        bits.append(f"B{'*' if self.torque else '-'}")
        return (head + " ".join(bits))[:width]


def prompt_line(question, cooked):
    """Read a word with the terminal briefly back in its normal line mode.

    cbreak delivers single keys with no echo and no line editing, which is what
    every other control here wants and exactly wrong for typing a name. The
    attributes are restored on the way out even if the read fails, so a Ctrl-C
    at the prompt cannot leave the terminal unusable.
    """
    fd = sys.stdin.fileno()
    raw = termios.tcgetattr(fd)
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, cooked)
        sys.stdout.write("\r\033[K" + question)
        sys.stdout.flush()
        return sys.stdin.readline().strip()
    except (KeyboardInterrupt, EOFError):
        return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, raw)


def drain_callbacks(node, budget=40, seconds=0.008):
    """Run every callback that is READY, not just one of them.

    THIS IS WHY ADDING THE MIDDLE ARM MADE THE OTHER TWO WORSE. rclpy's
    spin_once executes AT MOST ONE callback per call, and this loop called it
    once per keyboard poll (~50 times a second). Every subscription and the
    republish timer compete for those slots: with two arms that was 8
    subscriptions + 1 timer, and the timer came round about every 180 ms --
    just inside the arm agents' 300 ms cmd_pose deadline. Adding the middle arm
    took it to 12 subscriptions, one of which (its joint_states) publishes at
    100 Hz and is therefore ready on every single wait, and the timer slipped
    to ~260 ms and past 300 ms under any jitter.

    The agent then did exactly what it is supposed to do on an input that has
    gone quiet -- "cmd_pose timed out -- holding", drop the target, wind down
    to idle -- so the arms went dead mid-jog for reasons that had nothing to do
    with the arms. 11 such timeouts in one session before this was found.

    Draining instead of single-stepping keeps the republish timer on schedule
    no matter how many arms are on the graph. rig_debug.py already did this;
    this file did not.
    """
    end = time.monotonic() + seconds
    for _ in range(budget):
        rclpy.spin_once(node, timeout_sec=0.0)
        if time.monotonic() >= end:
            break


def read_keys(timeout_s):
    """Every key token readable within timeout_s.

    os.read on the raw fd, NOT sys.stdin.read: sys.stdin is buffered, so
    reading one character pulls the whole chunk into Python's own buffer and a
    following select() reports nothing -- an arrow key is then seen as a bare
    ESC and silently does nothing.
    """
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout_s)[0]:
        return []
    data = os.read(fd, 64)
    if not data:
        raise EOFError("stdin closed")
    text = data.decode(errors="ignore")
    tokens, i = [], 0
    while i < len(text):
        if text[i] == "\x1b":
            if text[i:i + 3] in (UP, DOWN, LEFT, RIGHT):
                tokens.append(text[i:i + 3])
                i += 3
            else:
                tokens.append("ESC")
                i += 1
        else:
            tokens.append(text[i])
            i += 1
    return tokens


def handle(node, key):
    # SPACE first, unconditionally: the panic key must never sit behind a mode.
    if key == " ":
        return node.toggle_all()
    if key == JOINT_MODE_KEY:
        return node.cycle_joint_mode()

    # Joint mode shadows the digits, and only the digits.
    if node.joint_arm is not None:
        if key in JOINT_POS:
            return node.move_joint(JOINT_POS.index(key), +1)
        if key in JOINT_NEG:
            return node.move_joint(JOINT_NEG.index(key), -1)
        if key == JOINT_HOME_KEY:
            return node.go_home(node.joint_arm)
        if key in "[]":
            f = 1 / 1.5 if key == "[" else 1.5
            node.joint_step = max(0.005, min(MAX_JOINT_STEP, node.joint_step * f))
            return f"joint step {math.degrees(node.joint_step):.1f} deg"

    if key in ENABLE_KEYS:
        return node.toggle_arm(ENABLE_KEYS[key])
    if key == "0":
        return node.toggle_base()
    if key in ZERO_KEYS:
        return node.zero(ZERO_KEYS[key])
    if key in RESET_KEYS:
        return node.reset(RESET_KEYS[key])
    if key == ROT_MODE_KEY:
        node.rot_mode = not node.rot_mode
        return ("ROTATE mode: qwe/asd etc. now turn the end effector about world "
                "x/y/z" if node.rot_mode else "translate mode")
    if key in ARM_KEYS:
        return node.move_arm(*ARM_KEYS[key])
    if key in GRIPPER_KEYS:
        return node.grip(*GRIPPER_KEYS[key])
    if key == SAVE_KEY:
        return PROMPT_SAVE
    if key in POSE_ALL_KEYS:
        return node.go_all(key, getattr(node.args, f"{POSE_ALL_KEYS[key]}_pose"))
    if key in BASE_KEYS:
        return node.move_base(*BASE_KEYS[key])
    if key in LIFT_KEYS:
        return node.move_lift(LIFT_KEYS[key])
    if key == "[":
        node.step = max(0.001, node.step / 1.5)
        return f"arm step {node.step * 100:.2f} cm"
    if key == "]":
        node.step = min(0.05, node.step * 1.5)
        return f"arm step {node.step * 100:.2f} cm"
    return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", type=float, default=0.01, help="arm metres per press")
    ap.add_argument("--grip-force", type=float, default=20.0,
                    help="newtons to squeeze with (20 gentle, 100 firm)")
    # 'home' exists everywhere out of the box: the manipulators save it
    # automatically at first start and the middle arm ships it. 'rest' is
    # built into the middle arm; the manipulators need one saved per arm
    # (their vendor 'sleep' assumes a table mount, not this rig's sideways
    # one, so nothing here guesses it for you).
    ap.add_argument("--rest-pose", default="rest",
                    help="pose name shift-R sends every enabled arm to")
    ap.add_argument("--start-pose", default="start",
                    help="pose name shift-S sends every enabled arm to")
    # Both default OFF and both move three arms, so they are opt-in -- but
    # settable from the environment, which is how tmux-rig.sh turns them on
    # without anyone editing a command line.
    ap.add_argument("--start-on-launch", action="store_true",
                    default=os.environ.get("RIG_START_ON_LAUNCH", "") == "1",
                    help="on startup, enable every arm and send it to --start-pose")
    ap.add_argument("--rest-on-quit", action="store_true",
                    default=os.environ.get("RIG_REST_ON_QUIT", "") == "1",
                    help="on quit, send every arm to --rest-pose before releasing")
    ap.add_argument("--pose-wait", type=float, default=6.0,
                    help="seconds to let a whole-rig pose move finish")
    ap.add_argument("--linear", type=float, default=0.15, help="base m/s")
    ap.add_argument("--angular", type=float, default=0.4, help="base rad/s")
    ap.add_argument("--lift-speed", type=float, default=0.02, help="lift m/s")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    busy = control_lock.refuse_if_busy("rig_key.py", sys.stderr)
    if busy:
        return busy

    if not sys.stdin.isatty():
        print("needs a real terminal -- `docker compose exec monitor bash`, "
              "not `exec -T`.", file=sys.stderr)
        return 2

    rclpy.init()
    node = RigKey(args)
    print("  enable  1 left   2 middle   3 right   0 base    SPACE all")
    print("  left  qwe/asd    middle rty/fgh    right uio/jkl   (+xyz / -xyz)")
    print("  grip  z/x left   n/m right         base  arrow keys")
    print("  lift  , / .      step [ ]          quit  ESC")
    print("  `    toggle ROTATE mode: the same arm keys turn the end effector about world x/y/z")
    print(f"  R R  all arms -> {args.rest_pose!r}    S S  all arms -> "
          f"{args.start_pose!r}   (twice = confirm)")
    print(f"  K    save every arm's pose under a name you type  (or 'name left' for one arm)"
          + ("   [start-on-launch]" if args.start_on_launch else "")
          + ("   [rest-on-quit]" if args.rest_on_quit else ""))
    print("  4/5/6 zero arm (current pos -> 0,0,0)   7/8/9 reset after a fault")
    print(f"  arm step {args.step * 100:.1f} cm | grip {args.grip_force:.0f} N"
          f"{'   [DRY RUN]' if args.dry_run else ''}\n")

    old = termios.tcgetattr(sys.stdin)
    status = "nothing enabled -- press SPACE, or 1/2/3/0"
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Turn OFF ISIG so Ctrl-C arrives as a BYTE we read, not as SIGINT.
        #
        # tty.setcbreak clears ICANON and ECHO but leaves ISIG set, so Ctrl-C
        # raises a signal -- and rclpy's own SIGINT handler invalidates the
        # context before our `finally` runs. The release then publishes into a
        # dead context and silently does nothing, which left every arm ENABLED
        # after quitting. Observed directly.
        #
        # With ISIG off, Ctrl-C is just another key the loop handles, so
        # shutdown runs in order with a live context and the release lands.
        _attrs = termios.tcgetattr(sys.stdin.fileno())
        _attrs[3] &= ~termios.ISIG          # index 3 is lflag
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _attrs)
        end = time.monotonic() + 1.5
        while rclpy.ok() and time.monotonic() < end:
            drain_callbacks(node, seconds=0.05)

        if args.start_on_launch:
            # After the settle above, so every arm has reported a pose and its
            # pose_names and nothing is published into the dark.
            went, skipped = node.send_pose_all(args.start_pose, enable_first=True)
            if went:
                print(f"  moving to {args.start_pose!r}: {', '.join(went)}"
                      "   -- WATCH THEM")
                node.wait_moving(args.pose_wait)
                status = f"at {args.start_pose!r}"
            if skipped:
                print(f"  not moved: {'; '.join(skipped)}")

        import shutil
        while rclpy.ok():
            drain_callbacks(node)
            try:
                keys = read_keys(0.02)
            except EOFError:
                break
            stop = False
            for key in keys:
                if key in ("\x03", "ESC"):
                    stop = True
                    break
                line = handle(node, key)
                if line is PROMPT_SAVE:
                    line = node.save_all(prompt_line(
                        "  save every arm's pose as (e.g. start, rest): ", old))
                    sys.stdout.write("\n")
                if line is not None:
                    status = line
            if stop:
                break
            if keys:
                w = shutil.get_terminal_size((80, 24)).columns
                flags = node.status_line(14)
                sys.stdout.write("\r\033[K" + f"  {flags} {status}"[:w - 1])
                sys.stdout.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if args.rest_on_quit and rclpy.ok():
            # BEFORE stop_all: releasing first would drop each arm to idle and
            # the move would never happen.
            try:
                went, skipped = node.send_pose_all(args.rest_pose,
                                                   enable_first=True)
                if went:
                    print(f"\n  returning to {args.rest_pose!r}: "
                          f"{', '.join(went)}   -- WATCH THEM")
                    node.wait_moving(args.pose_wait)
                if skipped:
                    print(f"  not moved: {'; '.join(skipped)}")
            except Exception as e:
                print(f"  rest-on-quit failed: {e}")
        # No rclpy.ok() guard: if the context somehow died the calls are
        # no-ops, but skipping them outright is how the arms stayed enabled.
        try:
            node.stop_all()
        except Exception:
            pass
        print("\n  stopped: arms released, base and lift zeroed.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
