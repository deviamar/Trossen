#!/usr/bin/env python3
"""Turn Quest input into robot commands, using giava's tuned mapping.

    ./quest_teleop.py                  # everything
    ./quest_teleop.py --no-base        # arms only
    ./quest_teleop.py --no-middle      # leave the camera arm alone
    ./quest_teleop.py --dry-run        # log what it would send, publish nothing

CONTROLS
--------
    X  (left, hold)                  left arm follows the left controller
    A  (right, hold)                 right arm follows the right controller
    either X or A                    camera arm follows your HEAD
    index trigger                    that arm's gripper: released stages the
                                     fingers open, squeezed commands an analog
                                     CLOSING FORCE (see quest_config.py)
    left thumbstick                  base: fwd/back drives, left/right turns
    right thumbstick fwd/back        scissor lift z velocity (simulated until
                                     the lift hardware is wired in)

Subscribes only to /quest/* and each component's own <ns>/ee_pose. Publishes
only to components' command topics. No robot container knows this node exists --
swap the Quest for a gamepad or a policy and nothing downstream changes.

THE MAPPING IS NOT MINE. giava/teleop_map.py is lifted from the GIAVA rig, where
the constants were tuned over real sessions: 1.35x position scale, alpha=0.3
exponential smoothing on the target, and a 2 cm per-step Cartesian clamp. The
clamp is the part worth understanding -- it CLAMPS an over-large step rather
than rejecting it, so a tracking glitch becomes a slightly slower follow instead
of a dropped frame. Rejecting reads to the operator as the arm stuttering.

The per-rig parts are now explicit: the session's arbitrary yaw is MEASURED
from your gaze at engage (teleop_map.session_yaw_remap), and what remains --
how you stand relative to the rig, and how far a hand-metre moves the EE -- is
QUEST_ARM_REMAP_YAW_DEG / QUEST_POS_SCALE in quest_config.py, defaulting to
"facing rig-forward" and 1:1.

HOLD TO ENGAGE. Releasing the button stops the arm following you. A toggle
leaves an armed robot behind when you set the controller down, and you find out
the next time you move your hand.

ABSOLUTE TARGETS. cmd_pose says where the end effector should be, not how far to
move. This node owns the clutch and the accumulation; a dropped message costs one
frame of lag rather than permanently shifting the operator's frame against the
robot's.

DRIVING IS LOCKED OUT WHILE AN ARM IS ENGAGED (--allow-drive-while-engaged to
override). The rig is mobile and the arm anchors are captured in the arm's base
frame: drive while holding a target and the arm holds station relative to a base
that is moving under it, which is correct but rarely what anyone means. Let go,
drive, re-engage.
"""
import argparse
import json
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String

import quest_config as cfg
from giava.teleop_map import (
    TeleopConfig,
    TeleopSessionState,
    CommandKinematicsState,
    start_teleop_session,
    stop_teleop_session,
    compute_gripper_arm_target,
    compute_camera_arm_target,
)
from giava.transform_utils import (
    pose2mat,
    quat2mat,
    transform_coordinates,
    within_pose_threshold,
)
from giava.headset_utils import convert_right_to_left_coordinates


def pose7(pos, quat_xyzw):
    """[qw, qx, qy, qz, x, y, z] -- the layout giava's teleop_map expects.

    Quaternion first. This is jaxlie's SE3 wxyz_xyz convention, which is what
    the upstream pyroki FK produced, and start_teleop_session/
    compute_*_arm_target both index it that way. Note it is NOT the order
    transform_utils.matrix_to_pose7() returns, which is position first -- mixing
    the two silently swaps rotation and translation.
    """
    x, y, z, w = quat_xyzw
    return np.array([w, x, y, z, pos[0], pos[1], pos[2]], dtype=float)


def msg_to_mat(msg):
    """PoseStamped -> 4x4."""
    p, o = msg.pose.position, msg.pose.orientation
    return pose2mat(np.array([p.x, p.y, p.z], dtype=float),
                    np.array([o.x, o.y, o.z, o.w], dtype=float))


class Link:
    """One driven component: its topics, its anchor, its last commanded pose."""

    def __init__(self, node, ns, key, kind, dry_run, has_gripper):
        self.node = node
        self.ns = ns
        self.key = key                 # "left" / "right" / "middle"
        self.kind = kind               # "gripper" or "camera"
        self.dry_run = dry_run
        self.has_gripper = has_gripper

        self.measured = None           # 4x4 from <ns>/ee_pose
        self.engaged = False
        self.grip_sent = None          # last (kind, value) actually published

        node.create_subscription(PoseStamped, f"{ns}/ee_pose", self._on_ee, 1)
        # What the AGENT says about itself, at 20 Hz. A fresh True after we
        # publish enable is the only proof the enable has landed.
        self.active = False
        node.create_subscription(Bool, f"{ns}/active", self._on_active, 1)
        self.pub_cmd = node.create_publisher(PoseStamped, f"{ns}/cmd_pose", 1)
        self.pub_name = node.create_publisher(String, f"{ns}/cmd_pose_name", 1)
        self.pub_enable = node.create_publisher(Bool, f"{ns}/enable", 1)
        self.pub_grip = (node.create_publisher(Float32, f"{ns}/cmd_gripper", 1)
                         if has_gripper else None)
        self.pub_grip_force = (node.create_publisher(Float32, f"{ns}/cmd_grip_force", 1)
                               if has_gripper else None)

    def _on_ee(self, msg):
        self.measured = msg_to_mat(msg)

    def _on_active(self, msg):
        self.active = bool(msg.data)

    def ready(self):
        return self.measured is not None

    def publish_enable(self, on):
        if self.dry_run:
            return
        m = Bool()
        m.data = bool(on)
        self.pub_enable.publish(m)

    def publish_target(self, pos, quat_wxyz):
        if self.dry_run:
            self.node.get_logger().info(
                f"{self.ns}: would send ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            return
        m = PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in pos]
        m.pose.orientation.w = float(quat_wxyz[0])
        m.pose.orientation.x = float(quat_wxyz[1])
        m.pose.orientation.y = float(quat_wxyz[2])
        m.pose.orientation.z = float(quat_wxyz[3])
        self.pub_cmd.publish(m)

    def publish_gripper(self, trigger):
        """Released: stage the fingers open (position). Squeezed: closing FORCE.

        giava's DYNAMIXEL grippers did this with current_based_position and a
        Current_Limit register; the WXAI's native form is external_effort via
        <ns>/cmd_grip_force -- the finger stops where the object is and
        squeezes exactly as hard as asked, instead of a position loop faulting
        the arm on a following error it can never close. The trigger is analog,
        so squeeze harder to grip harder.

        Deduplicated: a held trigger would otherwise be 50 Hz of identical
        commands and log lines at the arm. Force is quantised to 0.5 N so
        analog jitter does not defeat the dedup.
        """
        if self.pub_grip is None or self.dry_run:
            return
        t = max(0.0, min(1.0, float(trigger)))
        if t < cfg.TRIGGER_DEADZONE:
            # FORCE, not position. See cfg.OPEN_FORCE_N: opening by position
            # both changed the gripper's mode on every release (a mode write
            # momentarily drops the arm's control loop -- the twitch) and drove
            # the fingers into their stop, which the controller answers by
            # faulting the WHOLE ARM. Pushing apart with a force stops wherever
            # the fingers stop and never leaves effort mode.
            want = ("open", abs(cfg.OPEN_FORCE_N))
        else:
            span = cfg.GRASP_FORCE_MAX_N - cfg.GRASP_FORCE_MIN_N
            f = cfg.GRASP_FORCE_MIN_N + span * (t - cfg.TRIGGER_DEADZONE) / (1.0 - cfg.TRIGGER_DEADZONE)
            want = ("close", -round(f * 2.0) / 2.0)   # negative closes
        if want == self.grip_sent:
            return
        m = Float32()
        m.data = float(want[1])
        self.pub_grip_force.publish(m)        # one topic, one mode, both ways
        self.grip_sent = want


def _yaw_matrix(deg):
    a = np.radians(float(deg))
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class QuestTeleop(Node):
    def __init__(self, args):
        super().__init__("quest_teleop")
        self.args = args
        self.cfg = TeleopConfig()
        # Per-rig tuning comes from the environment (see quest_config.py); the
        # TeleopConfig defaults are giava's rig, not this one.
        self.cfg.position_scale = cfg.POSITION_SCALE * args.scale
        self.cfg.cam_position_scale = cfg.CAM_POSITION_SCALE * args.scale
        self.cfg.rotation_scale = cfg.ROTATION_SCALE
        self.cfg.cam_rotation_scale = cfg.CAM_ROTATION_SCALE
        self.cfg.R_arm_remap = _yaw_matrix(cfg.ARM_REMAP_YAW_DEG)
        self.cfg.R_cam_remap = _yaw_matrix(cfg.CAM_REMAP_YAW_DEG)
        self.connected = False
        self.pose = {"left": None, "right": None, "head": None}
        self.joy = {"left": None, "right": None}

        self.links = {}
        self.links["left"] = Link(self, cfg.ARM_NS_LEFT, "left", "gripper",
                                  args.dry_run, has_gripper=True)
        self.links["right"] = Link(self, cfg.ARM_NS_RIGHT, "right", "gripper",
                                   args.dry_run, has_gripper=True)
        if not args.no_middle:
            self.links["middle"] = Link(self, cfg.MIDDLE_NS, "middle", "camera",
                                        args.dry_run, has_gripper=False)

        self._rest_was = False
        self._rest_since = 0.0
        self._quit_at = None
        self._rest_pending = {}        # key -> deadline; enabled, name not yet sent
        self.state = TeleopSessionState()
        self.cmd_kin = CommandKinematicsState(T_cmd={})

        self.create_subscription(PoseStamped, cfg.TOPIC_LEFT_POSE,
                                 lambda m: self._set_pose("left", m), 1)
        self.create_subscription(PoseStamped, cfg.TOPIC_RIGHT_POSE,
                                 lambda m: self._set_pose("right", m), 1)
        self.create_subscription(PoseStamped, cfg.TOPIC_HEAD_POSE,
                                 lambda m: self._set_pose("head", m), 1)
        self.create_subscription(Joy, cfg.TOPIC_LEFT_JOY,
                                 lambda m: self._set_joy("left", m), 1)
        self.create_subscription(Joy, cfg.TOPIC_RIGHT_JOY,
                                 lambda m: self._set_joy("right", m), 1)
        self.create_subscription(Bool, cfg.TOPIC_CONNECTED, self._on_conn, 1)

        self.pub_base = self.create_publisher(
            Twist, f"{cfg.BASE_NS}/cmd_vel_teleop", 1)
        self.pub_lift = self.create_publisher(
            Float32, f"{cfg.BASE_NS}/lift/cmd_velocity", 1)
        self.pub_feedback = self.create_publisher(String, cfg.TOPIC_FEEDBACK, 1)

        # giava runs its loop at 50 Hz (TeleopConfig.control_dt) and the arm
        # agent streams at the same rate, so matching it keeps one target per
        # command rather than resampling between two unrelated clocks.
        self.create_timer(self.cfg.control_dt, self._tick)

    def _set_pose(self, which, msg):
        self.pose[which] = msg_to_mat(msg)

    def _set_joy(self, which, msg):
        self.joy[which] = msg

    def _on_conn(self, msg):
        was = self.connected
        self.connected = bool(msg.data)
        if was and not self.connected:
            self.get_logger().warn("headset lost -- releasing everything")
            self._release_all()
            self._publish_base(0.0, 0.0)
            self._publish_lift(0.0)

    # ---- helpers ---------------------------------------------------------
    def _button(self, hand, index=None):
        j = self.joy.get(hand)
        i = cfg.BTN_PRIMARY if index is None else index
        return bool(j and len(j.buttons) > i and j.buttons[i])

    def _axis(self, hand, index):
        j = self.joy.get(hand)
        return float(j.axes[index]) if j and len(j.axes) > index else 0.0

    def _controller_poses(self):
        return {"left": self.pose["left"],
                "right": self.pose["right"],
                "middle": self.pose["head"]}

    def _release_all(self):
        stop_teleop_session(self.state)
        for link in self.links.values():
            if link.engaged:
                link.engaged = False
                link.publish_enable(False)

    # ---- main loop -------------------------------------------------------
    def _rest_pump(self):
        """Second half of the B hold: send `rest` to each arm that has
        confirmed its enable; keep re-asserting enable for the rest until
        their deadline, then give up on those loudly."""
        if not self._rest_pending:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for key in list(self._rest_pending):
            link = self.links[key]
            if link.active:
                if not self.args.dry_run:
                    link.pub_name.publish(String(data=cfg.REST_POSE))
                self.get_logger().info(f"{link.ns}: enabled -- {cfg.REST_POSE!r} sent")
                del self._rest_pending[key]
            elif now >= self._rest_pending[key]:
                self.get_logger().error(
                    f"{link.ns}: never confirmed enable -- NOT parked, check its log")
                del self._rest_pending[key]
            else:
                link.publish_enable(True)

    def _tick(self):
        self._rest_pump()
        if self._quit_at is not None:
            # Parking. Drive nothing -- a streamed target would fight the
            # named move -- and hold the link open until the arms are there.
            if self.get_clock().now().nanoseconds * 1e-9 >= self._quit_at:
                self.get_logger().info("parked -- ending the session")
                raise SystemExit(0)
            return
        if not self.connected:
            return

        poses = self._controller_poses()
        want = {"left": self._button("left"), "right": self._button("right")}
        # The camera arm follows your head whenever either hand is working --
        # you want the view to track you while your hands are busy, and to stop
        # when you let go of both. Straight from giava's arm_active.
        want["middle"] = want["left"] or want["right"]

        engaging = any(want[k] for k in self.links)

        if engaging and not self.state.active:
            if not self._anchor(poses, want):
                return
        elif not engaging and self.state.active:
            stop_teleop_session(self.state)
            self.get_logger().info("teleop DISABLED")

        for key, link in self.links.items():
            self._drive(key, link, poses, want[key])

        self._grippers()
        self._rest_button()
        self._base()
        self._feedback(poses)

    def _anchor(self, poses, want):
        """Capture controller and arm anchors. False if we cannot yet."""
        missing = [k for k, l in self.links.items()
                   if want[k] and (not l.ready() or poses[k] is None)]
        if missing:
            # Anchoring against a pose we do not have would send the arm to
            # wherever the controller happens to be. Refuse and say why.
            self.get_logger().warn(
                f"cannot engage: no ee_pose or controller pose for {missing}. "
                "Is arm_agent.py running on those arms?", throttle_duration_sec=5.0)
            return False

        for key, link in self.links.items():
            if link.measured is None:
                continue
            # Re-sync the commanded pose to where the arm actually is. Anchoring
            # against a stale command would step the arm by whatever it had
            # drifted since the last session.
            p = link.measured[:3, 3]
            q = _mat_to_quat_xyzw(link.measured[:3, :3])
            self.cmd_kin.T_cmd[key] = pose7(p, q)

        arms = [k for k in self.links if poses[k] is not None]
        # base_remaps + the head pose let the anchor fold the session's
        # measured yaw into each arm's remap -- the app world's yaw is
        # arbitrary per session, so a fixed matrix alone cannot be right.
        base_remaps = {k: (self.cfg.R_cam_remap if self.links[k].kind == "camera"
                           else self.cfg.R_arm_remap) for k in arms}
        start_teleop_session(self.state, arms, poses, self.cmd_kin,
                             base_remaps=base_remaps, head_pose=self.pose["head"])

        # start_teleop_session builds fresh ArmTeleopState objects, whose
        # `active` defaults to False. Upstream re-sets the flags at the top of
        # its next loop iteration, so it self-corrects after one tick; setting
        # them here removes that tick of dead input.
        for key in arms:
            self.state.arms[key].active = want[key]

        self.get_logger().info(f"teleop ENABLED: {[k for k in arms if want[k]]}")
        return True

    def _drive(self, key, link, poses, active):
        if active and not link.engaged:
            link.engaged = True
            link.publish_enable(True)
        elif not active and link.engaged:
            link.engaged = False
            link.publish_enable(False)

        if not active or not self.state.active:
            # Not engaged: keep the commanded pose tracking the measured one so
            # the next engage anchors on reality.
            if link.measured is not None:
                p = link.measured[:3, 3]
                q = _mat_to_quat_xyzw(link.measured[:3, :3])
                self.cmd_kin.T_cmd[key] = pose7(p, q)
            return

        arm_state = self.state.arms.get(key)
        if arm_state is None or poses[key] is None:
            return
        arm_state.active = True

        fn = compute_camera_arm_target if link.kind == "camera" else compute_gripper_arm_target
        remap = self.cfg.R_cam_remap if link.kind == "camera" else self.cfg.R_arm_remap

        target_pos, target_wxyz = fn(
            self.cfg, arm_state, poses[key], self.cmd_kin.T_cmd[key], remap)

        link.publish_target(target_pos, target_wxyz)
        # The commanded pose is the target we just sent, not the measured pose.
        # compute_*_arm_target clamps each step against it, so feeding back the
        # measurement instead would let a lagging arm drag the target backwards
        # and turn following error into a slow crawl.
        self.cmd_kin.T_cmd[key] = np.concatenate(
            [np.asarray(target_wxyz, dtype=float), np.asarray(target_pos, dtype=float)])

    def _rest_button(self):
        """B (or Y) parks every arm at its saved rest pose.

        Edge-triggered, so holding it sends one command rather than one per
        tick, and refused outright while an arm is engaged: a whole-rig move
        that can start under your thumb mid-teleop is not a convenience.

        The arms are ENABLED first and deliberately left enabled -- an agent
        ignores a named move while disabled, and disabling it mid-move would
        drop the arm wherever it had got to.
        """
        if self._quit_at is not None:
            return                       # already parking; one press is enough
        if cfg.REST_BUTTON not in ("left", "right"):
            return
        pressed = self._button(cfg.REST_BUTTON, cfg.BTN_SECONDARY)
        was, self._rest_was = self._rest_was, pressed
        now = self.get_clock().now().nanoseconds * 1e-9
        if not pressed:
            self._rest_since = 0.0
            return
        if not was:
            self._rest_since = now          # just went down; start the clock
            return
        if now - self._rest_since < cfg.REST_HOLD_S:
            return                          # still holding, not long enough yet
        self._rest_since = now + 1e9        # fire once per hold, not every tick
        if self.state.active:
            self.get_logger().warn(
                "rest button ignored while an arm is engaged -- let go first")
            return
        # Enable now; send the NAME only once each agent reports active.
        # enable and cmd_pose_name are different topics and nothing orders
        # them: sent back-to-back, an agent can see the name first and log
        # "pose 'rest' ignored -- not enabled" -- B then quit the session
        # with the arms still where they were. _rest_pump() finishes the job.
        names = []
        for key, link in self.links.items():
            if link.measured is None:
                continue
            link.active = False           # want a FRESH True, not a stale one
            link.publish_enable(True)
            self._rest_pending[key] = now + 1.5
            names.append(key)
        self.get_logger().info(
            f"REST: {', '.join(names) or 'nothing'} -> {cfg.REST_POSE!r} "
            "(joint space, several seconds -- watch them)")
        if cfg.REST_QUITS:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._quit_at = now + cfg.REST_QUIT_WAIT_S
            self.get_logger().info(
                f"then ending the session in {cfg.REST_QUIT_WAIT_S:.0f} s "
                "-- the arms are released only after they have parked")

    def _grippers(self):
        for hand in ("left", "right"):
            link = self.links.get(hand)
            if link is not None:
                link.publish_gripper(self._axis(hand, cfg.AXIS_TRIGGER))

    def _base(self):
        # The lift is gated exactly like driving: the arms are bolted to the
        # lift's face, so raising it while an arm holds an anchored target is
        # the same base-moves-under-the-anchor problem as driving is.
        locked = self.state.active and not self.args.allow_drive_while_engaged
        lift = 0.0 if locked else (
            cfg.apply_deadzone(self._axis("right", cfg.AXIS_STICK_Y)) * cfg.LIFT_MAX_VEL)
        self._publish_lift(lift)

        if self.args.no_base:
            return
        if locked:
            self._publish_base(0.0, 0.0)
            return
        # One stick for the whole plane: forward/back drives, left/right turns.
        lin = cfg.apply_deadzone(self._axis("left", cfg.AXIS_STICK_Y)) * cfg.BASE_MAX_VEL_X
        ang = (cfg.TURN_SIGN
               * cfg.apply_deadzone(self._axis("left", cfg.AXIS_STICK_X))
               * cfg.BASE_MAX_VEL_Z)
        self._publish_base(lin, ang)

    def _publish_base(self, lin, ang):
        if self.args.dry_run:
            return
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub_base.publish(m)

    def _publish_lift(self, vel):
        if self.args.dry_run:
            return
        m = Float32()
        m.data = float(vel)
        self.pub_lift.publish(m)

    # ---- feedback to the headset ----------------------------------------
    def _feedback(self, poses):
        """Arm poses and out-of-sync flags, for the Unity app to render.

        Out-of-sync is giava's idea and worth keeping: it compares where the arm
        WAS TOLD to go against where it IS, and shows the operator in VR when
        the robot cannot keep up. Without it, a lagging or refused arm feels
        identical to bad tracking.
        """
        out = {"info": "", "head_out_of_sync": False,
               "left_out_of_sync": False, "right_out_of_sync": False}

        name = {"left": "left_arm", "right": "right_arm", "middle": "middle_arm"}
        sync_key = {"left": "left_out_of_sync", "right": "right_out_of_sync",
                    "middle": "head_out_of_sync"}

        for key, link in self.links.items():
            if link.measured is None:
                continue
            cmd = self.cmd_kin.T_cmd.get(key)
            if cmd is not None and link.engaged:
                target_mat = pose2mat(np.asarray(cmd[4:], dtype=float),
                                      _wxyz_to_xyzw(cmd[:4]))
                ok = within_pose_threshold(
                    link.measured[:3, 3], link.measured[:3, :3],
                    target_mat[:3, 3], target_mat[:3, :3],
                    self.cfg.ee_reached_tol * 4.0, 0.3)
                out[sync_key[key]] = not bool(ok)

            # Express the measured pose in the operator's frame, then flip to
            # Unity's, so the app draws the arm where the hand that commands it
            # is -- not in the robot's coordinates, which mean nothing in VR.
            pos, quat = link.measured[:3, 3], _mat_to_quat_xyzw(link.measured[:3, :3])
            arm_state = self.state.arms.get(key)
            if arm_state is not None and arm_state.start_robot_pos is not None:
                start_robot = np.eye(4)
                start_robot[:3, :3] = arm_state.start_robot_rot
                start_robot[:3, 3] = arm_state.start_robot_pos
                start_ctrl = np.eye(4)
                start_ctrl[:3, :3] = arm_state.start_controller_rot
                start_ctrl[:3, 3] = arm_state.start_controller_pos
                in_ctrl = transform_coordinates(link.measured, start_robot, start_ctrl)
                pos, quat = in_ctrl[:3, 3], _mat_to_quat_xyzw(in_ctrl[:3, :3])

            u_pos, u_quat = convert_right_to_left_coordinates(
                np.ascontiguousarray(pos, dtype=np.float64),
                np.ascontiguousarray(quat, dtype=np.float64))
            out[f"{name[key]}_position"] = [float(v) for v in u_pos]
            out[f"{name[key]}_rotation"] = [float(v) for v in u_quat]

        if self.args.dry_run:
            return
        m = String()
        m.data = json.dumps(out)
        self.pub_feedback.publish(m)


def _mat_to_quat_xyzw(rot):
    from scipy.spatial.transform import Rotation as R
    return R.from_matrix(np.asarray(rot, dtype=float)).as_quat()


def _wxyz_to_xyzw(q):
    return np.array([q[1], q[2], q[3], q[0]], dtype=float)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-base", action="store_true", help="do not drive the base")
    ap.add_argument("--no-middle", action="store_true",
                    help="do not drive the active-vision arm")
    ap.add_argument("--allow-drive-while-engaged", action="store_true",
                    help="permit base motion while an arm is following you")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="extra multiplier on hand->EE motion (on top of "
                         "QUEST_POS_SCALE); 0.5 for cautious first sessions")
    ap.add_argument("--dry-run", action="store_true",
                    help="log intent, publish no robot commands")
    args = ap.parse_args()

    rclpy.init()
    node = QuestTeleop(args)
    print(f"  quest teleop up{'  [DRY RUN]' if args.dry_run else ''}")
    print(f"    hold X   -> {cfg.ARM_NS_LEFT}")
    print(f"    hold A   -> {cfg.ARM_NS_RIGHT}")
    if not args.no_middle:
        print(f"    either   -> {cfg.MIDDLE_NS}  (follows your head)")
    if cfg.REST_BUTTON in ("left", "right"):
        b = "B" if cfg.REST_BUTTON == "right" else "Y"
        tail = " then QUITS" if cfg.REST_QUITS else ""
        print(f"    {b} (hold {cfg.REST_HOLD_S:.0f}s) -> all arms to "
              f"{cfg.REST_POSE!r}{tail}, and only when nothing is engaged")
    print(f"    L stick  -> {cfg.BASE_NS}/cmd_vel_teleop  (fwd/back + turn)")
    print(f"    R stick  -> {cfg.BASE_NS}/lift/cmd_velocity  (z, sim until wired)")
    print("  waiting for the headset. Ctrl-C to stop.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        if rclpy.ok():
            node._release_all()
            node._publish_base(0.0, 0.0)
            node._publish_lift(0.0)
            print("\n  released everything.")
        else:
            print("\n  shut down externally -- downstream timeouts will stop the robots.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
