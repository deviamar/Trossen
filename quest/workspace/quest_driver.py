#!/usr/bin/env python3
"""Publish raw Quest state. Knows nothing about any robot.

    ./quest_driver.py --backend sim          # no hardware, fake motion
    ./quest_driver.py --backend udp          # app on the headset, JSON datagrams
    ./quest_driver.py --backend udp --port 9871

Publishes exactly what the device reports, on the topics in
docs/topic-contract.md:

    /quest/head/pose    geometry_msgs/PoseStamped
    /quest/left/pose    geometry_msgs/PoseStamped
    /quest/right/pose   geometry_msgs/PoseStamped
    /quest/left/joy     sensor_msgs/Joy
    /quest/right/joy    sensor_msgs/Joy
    /quest/connected    std_msgs/Bool

No robot command ever leaves this node. That split is what lets you run the
driver on its own and watch `ros2 topic echo /quest/right/joy` to work out which
button is which, with every robot in the rig powered down. The half that decides
what a button DOES is quest_teleop.py.

Transport is a swappable backend (backends/), because the link to the headset is
the least settled part of this rig.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Joy
from std_msgs.msg import Bool, String

import backends
import quest_config as cfg


class QuestDriver(Node):
    def __init__(self, backend, camera=None):
        super().__init__("quest_driver")
        self.backend = backend
        self.camera = camera
        self.last_frame = 0.0
        self.was_connected = None

        self.pub_head = self.create_publisher(PoseStamped, cfg.TOPIC_HEAD_POSE, 1)
        self.pub_lpose = self.create_publisher(PoseStamped, cfg.TOPIC_LEFT_POSE, 1)
        self.pub_rpose = self.create_publisher(PoseStamped, cfg.TOPIC_RIGHT_POSE, 1)
        self.pub_ljoy = self.create_publisher(Joy, cfg.TOPIC_LEFT_JOY, 1)
        self.pub_rjoy = self.create_publisher(Joy, cfg.TOPIC_RIGHT_JOY, 1)
        # Latched: a subscriber that starts late still learns the link is down
        # rather than sitting silent until the next state change.
        self.pub_conn = self.create_publisher(Bool, cfg.TOPIC_CONNECTED, 1)

        # The return path exists only for backends that are also a display.
        # sim and udp have no way back, so the subscriptions are simply not
        # created rather than every backend carrying no-op stubs.
        self.can_return = hasattr(backend, "send_feedback")
        self.frame_id = 0
        self._cam_frame_seen = -1
        self.stereo = {"left": None, "right": None}

        # The viewer can be told the camera geometry (gvlink only). Subscribed
        # separately from the images because CameraInfo is latched-ish and
        # low-rate: one message per eye is enough, and waiting for it must not
        # hold up the video path.
        self.cam_info = {"left": None, "right": None}
        self.cam_params_sent = False
        if hasattr(backend, "set_camera_params"):
            self.create_subscription(
                CameraInfo, cfg.TOPIC_STEREO_LEFT_INFO,
                lambda m: self._on_cam_info("left", m), 1)
            self.create_subscription(
                CameraInfo, cfg.TOPIC_STEREO_RIGHT_INFO,
                lambda m: self._on_cam_info("right", m), 1)

        if self.can_return:
            self.create_subscription(String, cfg.TOPIC_FEEDBACK, self._on_feedback, 1)
            self.create_subscription(
                Image, cfg.TOPIC_STEREO_LEFT, lambda m: self._on_image("left", m), 1)
            self.create_subscription(
                Image, cfg.TOPIC_STEREO_RIGHT, lambda m: self._on_image("right", m), 1)
            self.get_logger().info(
                f"return path on: {cfg.TOPIC_FEEDBACK}, "
                f"{cfg.TOPIC_STEREO_LEFT}, {cfg.TOPIC_STEREO_RIGHT}")

        self.create_timer(1.0 / cfg.PUBLISH_HZ, self._tick)

    # ---- return path ----------------------------------------------------
    def _on_feedback(self, msg):
        """JSON from quest_teleop -> giava HeadsetFeedback -> the Unity app."""
        try:
            import json
            import numpy as np
            from giava.headset_utils import HeadsetFeedback
        except ImportError:
            return
        try:
            d = json.loads(msg.data)
        except ValueError as e:
            self.get_logger().warn(f"bad feedback JSON: {e}")
            return

        fb = HeadsetFeedback()
        fb.info = str(d.get("info", ""))
        fb.head_out_of_sync = bool(d.get("head_out_of_sync", False))
        fb.left_out_of_sync = bool(d.get("left_out_of_sync", False))
        fb.right_out_of_sync = bool(d.get("right_out_of_sync", False))
        for name in ("left_arm", "right_arm", "middle_arm"):
            setattr(fb, f"{name}_position",
                    np.asarray(d.get(f"{name}_position", [0.0, 0.0, 0.0]), dtype=float))
            setattr(fb, f"{name}_rotation",
                    np.asarray(d.get(f"{name}_rotation", [0.0, 0.0, 0.0, 1.0]), dtype=float))
        self.backend.send_feedback(fb)

    def _on_cam_info(self, side, msg):
        """Build the viewer's camera geometry once both eyes have reported.

        Sent once. The intrinsics are scaled to the size the frames are
        actually resized to before sending -- intrinsics are in pixels, so a
        resize that the geometry does not follow puts a correctly rectified
        image at the wrong scale, which reads as the world being the wrong
        size rather than as a calibration error.
        """
        if self.cam_params_sent:
            return
        self.cam_info[side] = msg
        if self.cam_info["left"] is None or self.cam_info["right"] is None:
            return
        try:
            from backends.gvlink import zed_camera_params
            wire = zed_camera_params(self.cam_info["left"], self.cam_info["right"],
                                     cfg.STEREO_WIDTH, cfg.STEREO_HEIGHT)
        except Exception as e:
            self.get_logger().warn(f"camera geometry not usable yet: {e}",
                                   throttle_duration_sec=10.0)
            return
        if self.backend.set_camera_params(wire):
            self.cam_params_sent = True

    def _on_image(self, side, msg):
        """sensor_msgs/Image -> ndarray, without cv_bridge.

        cv_bridge is a surprisingly heavy dependency for one conversion, and it
        is a common source of ABI mismatches between a container's OpenCV and
        the one ROS was built against. The two encodings the ZED publishes are
        trivial to unpack by hand.
        """
        import numpy as np
        enc = msg.encoding
        if enc in ("rgb8", "bgr8"):
            channels = 3
        elif enc in ("rgba8", "bgra8"):
            channels = 4
        else:
            if not getattr(self, "_enc_warned", False):
                self.get_logger().warn(
                    f"unhandled image encoding {enc!r} -- expected rgb8/bgr8/rgba8/bgra8")
                self._enc_warned = True
            return

        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.width, channels)
        if channels == 4:
            img = img[:, :, :3]
        if enc.startswith("bgr"):
            img = img[:, :, ::-1]           # the video track wants rgb24

        if (msg.height, msg.width) != (cfg.STEREO_HEIGHT, cfg.STEREO_WIDTH):
            try:
                import cv2
                img = cv2.resize(img, (cfg.STEREO_WIDTH, cfg.STEREO_HEIGHT))
            except ImportError:
                pass
        self.stereo[side] = np.ascontiguousarray(img)

    def _send_stereo(self):
        if self.camera is not None:
            # A locally captured pair is already BGR and already the right
            # size; the backend's ROS path expects RGB, so hand it over in the
            # same convention the ROS images use and let the backend flip once.
            # ONLY WHEN THE CAMERA HAS A NEW FRAME. This tick runs at
            # PUBLISH_HZ (72) and the camera delivers 30, so handing over on
            # every tick offered the same pair two or three times; the encoder
            # mailbox is newest-only, so the duplicates were dropped -- 690 of
            # them in an 18 s run, which is a scary-looking counter for work
            # nobody asked for.
            pair = self.camera.read()
            if pair is None or self.camera.frames == self._cam_frame_seen:
                return
            self._cam_frame_seen = self.camera.frames
            left, right = pair[0][:, :, ::-1], pair[1][:, :, ::-1]
            if not self.cam_params_sent and hasattr(self.backend, "set_camera_params"):
                if self.backend.set_camera_params(self.camera.camera_params()):
                    self.cam_params_sent = True
            self.backend.send_stereo(left, right, self.frame_id)
            self.frame_id += 1
            return
        left, right = self.stereo["left"], self.stereo["right"]
        if left is None and right is None:
            return
        # Send whatever we have: one eye is better than a frozen view, and the
        # ZED's two topics do not always arrive in the same tick.
        self.backend.send_stereo(left if left is not None else right,
                                 right if right is not None else left,
                                 self.frame_id)
        self.frame_id += 1

    def _tick(self):
        frame = self.backend.read()
        now = time.monotonic()

        if frame is not None:
            self.last_frame = now
            self._publish(frame)

        connected = (now - self.last_frame) < cfg.STALE_S
        if connected != self.was_connected:
            self.get_logger().info(
                "headset connected" if connected else "headset lost -- no frames")
            self.was_connected = connected
        msg = Bool()
        msg.data = connected
        self.pub_conn.publish(msg)

        if self.can_return and connected:
            self._send_stereo()

    def _publish(self, frame):
        stamp = self.get_clock().now().to_msg()

        def pose(pub, p, q):
            m = PoseStamped()
            m.header.stamp = stamp
            m.header.frame_id = cfg.FRAME_ID
            m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in p]
            m.pose.orientation.x = float(q[0])
            m.pose.orientation.y = float(q[1])
            m.pose.orientation.z = float(q[2])
            m.pose.orientation.w = float(q[3])
            pub.publish(m)

        def joy(pub, hand):
            m = Joy()
            m.header.stamp = stamp
            m.header.frame_id = cfg.FRAME_ID
            m.axes = [float(v) for v in hand.axes]
            m.buttons = [int(v) for v in hand.buttons]
            pub.publish(m)

        pose(self.pub_head, frame.head_position, frame.head_orientation)
        pose(self.pub_lpose, frame.left.position, frame.left.orientation)
        pose(self.pub_rpose, frame.right.position, frame.right.orientation)
        joy(self.pub_ljoy, frame.left)
        joy(self.pub_rjoy, frame.right)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default="gvlink",
                    choices=["gvlink", "udp", "sim", "webrtc"])
    # A side-by-side USB stereo camera plugged into this machine, captured here
    # instead of arriving as ROS images. The ZED path (the middle arm's own
    # camera, on the topic contract) is unaffected and still preferred; this is
    # for a camera that has no ROS driver. Empty = ROS topics only.
    ap.add_argument("--camera", default=os.environ.get("QUEST_CAMERA", ""),
                    help="SBS stereo device, e.g. /dev/video4")
    ap.add_argument("--port", type=int, default=9871, help="udp backend port")
    ap.add_argument("--bind", default="0.0.0.0", help="udp backend bind address")
    ap.add_argument("--press", nargs="*", default=[],
                    help="sim backend: hold these, e.g. right_primary right_trigger")
    args = ap.parse_args()

    backend = backends.load(args.backend, port=args.port, bind=args.bind,
                            press=args.press)
    backend.start()

    camera = None
    if args.camera:
        from stereo_cam import StereoCamera
        camera = StereoCamera(args.camera,
                              out_size=(cfg.STEREO_WIDTH, cfg.STEREO_HEIGHT))
        camera.open().start()

    rclpy.init()
    node = QuestDriver(backend, camera)
    print(f"  quest driver up on {cfg.NS}, backend={args.backend}. Ctrl-C to stop.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  stopping.")
    finally:
        if camera is not None:
            camera.close()
        backend.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
