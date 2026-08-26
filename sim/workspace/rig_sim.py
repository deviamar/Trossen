#!/usr/bin/env python3
"""The rig, drawn from its own topics, in a browser.

    ./rig_sim.py                    # serve on :8080
    ./rig_sim.py --port 9000
    ./rig_sim.py --urdf <path>

Open http://<this machine>:8080 and the model moves as the real robot moves.

WHAT THIS IS
------------
A MIRROR, not a simulator. Every joint angle comes from a real encoder, over
the same topics the dashboard reads. There is no physics, no contact, and no
prediction: if the model is somewhere the robot is not, the model is wrong.

That makes it useful for the things a table of numbers is bad at -- whether the
arms are about to reach the same point, which way the wrist is actually facing,
whether the base has drifted from where you think it is -- and useless for
"would this trajectory collide", which needs a planner.

It publishes NOTHING. It cannot move the robot. That is deliberate: it is safe
to leave open while you drive.

WHY NOT RVIZ
------------
RViz needs a display. This rig is driven over ssh as often as at the bench, and
`xhost +local:docker` plus a forwarded X socket is a lot of moving parts for a
picture. Viser serves a web page, so the view works from any machine that can
reach this one -- including a phone, which is genuinely useful when you are
standing next to the robot with your hands full.

THE MODEL
---------
`mobile_ai.urdf` ships with trossen_arm_description and already describes this
rig: a SLATE base with wheels and casters, and two wxai followers on it. The
arms' own `joint_states` name their joints `joint_0..joint_5`; in the combined
model those live under `follower_left/` and `follower_right/` prefixes, which is
the only mapping this file has to do.

The middle arm and the scissor lift are NOT in this model -- they are not in
Trossen's URDF because they are not part of a stock Mobile AI. They are drawn
as placeholders when their topics are live, and the placeholder is deliberately
crude so it never reads as a measured pose.
"""
import argparse
import math
import os
import threading
import time

import numpy as np
import rclpy
import viser
import viser.extras
import yourdfpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

DESCRIPTION = os.environ.get("RIG_DESCRIPTION", "/home/robot/description")
# The COMPOSED model -- vendor parts plus this rig's own geometry, written by
# tools/build_rig_urdf.py. Falls back to Trossen's stock file so the visualiser
# still draws something if the builder has never been run.
DEFAULT_URDF = os.path.join(DESCRIPTION, "urdf", "rig.urdf")
FALLBACK_URDF = os.path.join(DESCRIPTION, "trossen_arm_description", "urdf",
                             "generated", "mobile_ai.urdf")

# <ns> -> the prefix its joints carry in the combined model.
ARM_PREFIX = {
    os.environ.get("RIG_LEFT_NS", "/left_arm"): "follower_left",
    os.environ.get("RIG_RIGHT_NS", "/right_arm"): "follower_right",
}
BASE_NS = os.environ.get("RIG_BASE_NS", "/slate")
MIDDLE_NS = os.environ.get("RIG_MIDDLE_NS", "/middle")

def resolve_mesh(fname):
    """package:// -> a real file under DESCRIPTION.

    yourdfpy has no idea what a ROS package is; ament isn't running here and
    there is no workspace to look in. The URDF is vendored, so a prefix swap is
    both sufficient and honest about what is happening.
    """
    # yourdfpy calls this with fname= as a KEYWORD, so the parameter name is
    # part of the interface, not an implementation detail.
    if fname.startswith("package://"):
        # description/ is laid out as real package directories, so the package
        # name IS the directory and this is a plain join. That is why the layout
        # is what it is: the alternative was a per-vendor rewrite table, and a
        # table is a thing that goes stale silently when a vendor moves a file.
        return os.path.join(DESCRIPTION, fname[len("package://"):])
    return fname


class RigSim(Node):
    """Subscribes to everything, publishes nothing."""

    def __init__(self, urdf):
        super().__init__("rig_sim")
        self.urdf = urdf
        self.lock = threading.Lock()

        # Joint name -> angle, in the combined model's namespace.
        self.cfg = {}
        # Base pose in the odom frame: x, y, yaw.
        self.base = [0.0, 0.0, 0.0]
        self.status = {"battery": None, "estop": None, "lift": None, "arms": {}}

        for ns, prefix in ARM_PREFIX.items():
            self.create_subscription(
                JointState, f"{ns}/joint_states",
                lambda m, p=prefix: self._on_joints(p, m), 10)
            self.create_subscription(
                PoseStamped, f"{ns}/ee_pose",
                lambda m, n=ns: self._on_ee(n, m), 10)
            self.create_subscription(
                Bool, f"{ns}/active",
                lambda m, n=ns: self.status["arms"].setdefault(n, {}).update(
                    active=m.data), 10)

        # The middle arm names its joints bare -- waist, shoulder, elbow, ... --
        # because that is what the Interbotix driver publishes and what the
        # composed URDF therefore uses. No prefix to add, and deliberately so;
        # see tools/build_rig_urdf.py.
        self.create_subscription(
            JointState, f"{MIDDLE_NS}/joint_states", self._on_middle_joints, 10)

        self.create_subscription(Odometry, f"{BASE_NS}/odom", self._on_odom, 10)
        self.create_subscription(
            Float32, f"{BASE_NS}/lift/height", self._on_lift, 10)

    def _on_joints(self, prefix, msg):
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                self.cfg[f"{prefix}/{name}"] = float(value)

        # The gripper needs no special case: right_carriage_joint carries a
        # <mimic> of left_carriage_joint, yourdfpy applies it, and the URDF
        # therefore does not list it as actuated. Verified rather than assumed --
        # commanding left to 0.03 moves the right carriage -0.03. Setting it
        # here as well would be dead weight at best.

    def _on_middle_joints(self, msg):
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                self.cfg[name] = float(value)

    def _on_lift(self, msg):
        with self.lock:
            self.cfg["lift_joint"] = float(msg.data)
            self.status["lift"] = float(msg.data)

    def _on_ee(self, ns, msg):
        p = msg.pose.position
        with self.lock:
            self.status["arms"].setdefault(ns, {})["ee"] = (p.x, p.y, p.z)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.base = [p.x, p.y, yaw]
            # The base driver signals E-stop through the odometry covariance --
            # see watch.py. It is an odd channel, but it is the one the firmware
            # actually uses, and a visualiser that ignored it would happily draw
            # a robot that cannot move.
            self.status["estop"] = msg.pose.covariance[0] < 0

    def snapshot(self):
        with self.lock:
            return dict(self.cfg), list(self.base), {
                "estop": self.status["estop"],
                "lift": self.status["lift"],
                "arms": {k: dict(v) for k, v in self.status["arms"].items()},
            }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--port", type=int, default=int(os.environ.get("RIG_SIM_PORT", 8080)))
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        raise SystemExit(f"no URDF at {args.urdf}")

    print(f"  loading {args.urdf}")
    urdf = yourdfpy.URDF.load(args.urdf,
                              filename_handler=resolve_mesh,
                              load_meshes=True, build_collision_scene_graph=False)

    server = viser.ViserServer(host="0.0.0.0", port=args.port)

    # The root frame is created ONCE and then moved by assigning to .position /
    # .wxyz. Re-adding a node every frame would rebuild it and its whole subtree
    # -- which here is the entire robot, meshes included -- 30 times a second.
    base_frame = server.scene.add_frame("/rig", show_axes=False)
    viser_urdf = viser.extras.ViserUrdf(server, urdf, root_node_name="/rig")

    actuated = list(urdf.actuated_joint_names)
    index = {n: i for i, n in enumerate(actuated)}
    print(f"  {len(actuated)} actuated joints; serving on :{args.port}")

    # A joint name that is not in the model is the failure mode this whole file
    # is most exposed to, and it is SILENT: the link just stays at zero, the
    # picture looks plausible, and nothing anywhere says the arm you are
    # watching is not the arm that is moving. Reported once per name.
    warned = set()

    def cfg_vector(cfg):
        for name in cfg:
            if name not in index and name not in warned:
                warned.add(name)
                print(f"  WARNING: {name} is not an actuated joint in this model "
                      f"-- it is being ignored, and that part of the robot will "
                      f"not move on screen")
        out = np.zeros(len(actuated), dtype=np.float64)
        for name, value in cfg.items():
            i = index.get(name)
            if i is not None:
                out[i] = value
        return out

    rclpy.init()
    node = RigSim(urdf)

    def spin_quietly():
        # rclpy.spin raises ExternalShutdownException when the context goes away
        # during shutdown. That is the normal way this thread ends, not a fault,
        # and letting it propagate prints a traceback on every restart that
        # looks exactly like a crash.
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, RuntimeError):
            pass

    spin = threading.Thread(target=spin_quietly, daemon=True)
    spin.start()

    with server.gui.add_folder("Rig"):
        g_estop = server.gui.add_text("E-stop", initial_value="--", disabled=True)
        g_base = server.gui.add_text("Base x y yaw", initial_value="--", disabled=True)
        g_left = server.gui.add_text("Left EE", initial_value="--", disabled=True)
        g_right = server.gui.add_text("Right EE", initial_value="--", disabled=True)
        g_lift = server.gui.add_text("Lift", initial_value="--", disabled=True)

    # A ground grid, so "the base moved" is visible at all. Without a fixed
    # reference the whole scene translates and nothing appears to happen.
    server.scene.add_grid("/ground", width=6.0, height=6.0)

    period = 1.0 / args.hz
    try:
        while rclpy.ok():
            cfg, base, status = node.snapshot()
            viser_urdf.update_cfg(cfg_vector(cfg))

            # The base drives the whole model's root, so the arms ride on it --
            # which is the entire reason for using the combined URDF rather than
            # two arm models side by side.
            x, y, yaw = base
            base_frame.position = (x, y, 0.0)
            base_frame.wxyz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

            g_base.value = f"{x:+.2f}  {y:+.2f}  {math.degrees(yaw):+.0f}deg"
            g_lift.value = ("--" if status["lift"] is None
                            else f"{status['lift']:.3f} m")
            g_estop.value = ("--" if status["estop"] is None
                             else ("ENGAGED" if status["estop"] else "clear"))
            for ns, box in ((next(iter(ARM_PREFIX)), g_left),
                            (list(ARM_PREFIX)[1], g_right)):
                ee = status["arms"].get(ns, {}).get("ee")
                box.value = "--" if ee is None else f"{ee[0]:+.3f} {ee[1]:+.3f} {ee[2]:+.3f}"

            time.sleep(period)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
