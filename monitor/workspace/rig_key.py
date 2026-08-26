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

    BASE       arrow keys -- up/down travel, left/right turn
    LIFT       , down   . up
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

THE BASE CANNOT STRAFE. It is differential drive: two driven wheels, four
passive casters. g/G rotate rather than sliding sideways, because no mechanism
could slide sideways. The rig's real z is the scissor lift.

MUST be run on a real terminal (`docker compose exec monitor bash`), not
`exec -T` -- it puts the tty in cbreak mode to read single keys.
"""
import argparse
import math
import os
import select
import sys
import termios
import time
import tty

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
        self.pub_cmd, self.pub_en, self.pub_grip = {}, {}, {}
        self.pub_zero, self.pub_reset = {}, {}
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
                    n, [x for x in m.data.split(",") if x]), 1)
            self.create_subscription(
                Bool, f"{ns}/active",
                lambda m, n=name: self.agent_active.__setitem__(n, m.data), 1)
            self.pub_en[name] = self.create_publisher(Bool, f"{ns}/enable", 1)
            self.pub_zero[name] = self.create_publisher(Bool, f"{ns}/zero", 1)
            self.pub_reset[name] = self.create_publisher(Bool, f"{ns}/reset", 1)
            if has_grip:
                self.pub_grip[name] = self.create_publisher(
                    Float32, f"{ns}/cmd_grip_force", 1)
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
        # Anchor on the measurement, so enabling is not a move.
        self.target[name] = (list(pos), list(quat))
        self.enabled[name] = True
        self._enable(name, True)
        self._send_arm(name)
        return f"{name} ENABLED at ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"

    def toggle_base(self, want=None):
        want = (not self.torque) if want is None else want
        if self.args.dry_run:
            self.torque = want
            return f"base torque {'ON' if want else 'OFF'} (dry run)"
        if not self.cli_torque.service_is_ready():
            # Non-blocking: a blocking wait would freeze the loop, including the
            # keys that stop things.
            self.cli_torque.wait_for_service(timeout_sec=1.0)
        if not self.cli_torque.service_is_ready():
            return "base torque: service missing -- is slate-base running?"
        req = SetBool.Request()
        req.data = want
        fut = self.cli_torque.call_async(req)
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
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
            rclpy.spin_once(self, timeout_sec=0.02)
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
            rclpy.spin_once(self, timeout_sec=0.02)
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
        pos, quat = self.target[name]
        pos = list(pos)
        pos["xyz".index(axis)] += sign * self.step
        self.target[name] = (pos, quat)
        self._send_arm(name)
        note = ""
        if name in self.measured:
            d = math.dist(pos, self.measured[name][0])
            if d > 0.02:
                note = f" ({d * 1000:.0f}mm behind)"
        return f"{name} {axis}{'+' if sign > 0 else '-'} -> ({pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}){note}"

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
        stale = self._disarmed_by_agent(name)
        if stale:
            return stale
        """Binary open/close, force-controlled.

        Not a position step: one press commands a squeeze (or a push apart) and
        the gripper holds it. Closing at --grip-force newtons stops wherever the
        object is, so an object too big to fully close on is gripped rather than
        crushed, and the controller never sees a following error.
        """
        if name not in self.pub_grip:
            return f"{name} has no gripper"
        force = sign * self.args.grip_force
        self.gripper[name] = force
        if not self.args.dry_run:
            m = Float32()
            m.data = float(force)
            self.pub_grip[name].publish(m)
        return (f"{name} gripper {'OPEN' if sign > 0 else 'CLOSE'} "
                f"at {abs(force):.0f} N")

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
        # silence, so it has to be repeated even when nothing is pressed.
        for name, on in self.enabled.items():
            if on:
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


def other_instances():
    """Other rig_key.py processes in this container, as (pid, cmdline).

    Read from /proc rather than shelling out to pgrep: pgrep -f matches the full
    command line of every process INCLUDING the one doing the matching, so the
    obvious version finds itself and refuses to ever start.
    """
    me = os.getpid()
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            # comm is what the process IS. Matching only the command line finds
            # any shell that happens to MENTION rig_key.py -- including the one
            # that launched this -- so the check is anchored on the executable.
            with open(f"/proc/{entry}/comm") as f:
                comm = f.read().strip()
            if not comm.startswith("python") and "rig_key" not in comm:
                continue
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="ignore").strip()
        except OSError:
            continue          # it exited while we were looking
        if "rig_key.py" in cmd:
            found.append((entry, cmd))
    return found


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
    if key in ARM_KEYS:
        return node.move_arm(*ARM_KEYS[key])
    if key in GRIPPER_KEYS:
        return node.grip(*GRIPPER_KEYS[key])
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
    ap.add_argument("--linear", type=float, default=0.15, help="base m/s")
    ap.add_argument("--angular", type=float, default=0.4, help="base rad/s")
    ap.add_argument("--lift-speed", type=float, default=0.02, help="lift m/s")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    others = other_instances()
    if others:
        print("another rig_key.py is already running in this container:",
              file=sys.stderr)
        for pid, cmd in others:
            print(f"    pid {pid}  {cmd}", file=sys.stderr)
        print("\nTwo of these FIGHT. Both publish to the same command topics at\n"
              "20 Hz and the newest message wins, so an old one whose dead-man has\n"
              "expired injects zeroes into the base's velocity and a stale anchor\n"
              "into the arms'. That reads as juddering motion and as commands\n"
              "being ignored -- not as a duplicate process, which is why this\n"
              "refuses to start rather than letting you find out.\n\n"
              "    make kill        end the session and clean up\n"
              "    make orphans     list them without killing anything",
              file=sys.stderr)
        return 2

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
            rclpy.spin_once(node, timeout_sec=0.05)

        import shutil
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
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
