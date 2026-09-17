"""Direct-UDP link to the v2 Unity app. The current headset transport.

    ./quest_driver.py --backend gvlink

WHAT THIS IS. The Unity viewer moved off WebRTC/Firestore onto its own UDP
stack, `gvlink`: the robot broadcasts a discovery beacon, the headset opens a
TCP control channel, video goes out over UDP and head/hand/controller poses
come back at the display rate. No cloud, no signalling handshake, no secrets
to mount -- which is why this replaces backends/webrtc.py for the v2 app.

    15550  robot broadcasts   discovery beacon (name, cameras, ports)
    15551  headset -> robot   TCP control: session setup, camera geometry, stats
    15552  robot -> headset   UDP video, fragmented H.264, one atlas per eye
    15553  headset -> robot   UDP input: head/hand/controller poses + gaze, 90 Hz

THE PROTOCOL LIBRARY IS NOT IN THIS REPO. `gvlink` lives beside the C# that has
to match it byte for byte -- av-aloha-unity, branch v2, Guided-Vision/python --
so there is exactly one copy of it. The container bind-mounts that checkout and
GVLINK_PATH points at it; a missing checkout surfaces as an ImportError here and
nowhere else. giava's gvlink_headset.py is vendored into ../giava/ unchanged,
so fixes upstream are a file copy.

STEREO COMES FROM THE ZED, OVER ROS. giava drives an OAK directly from this
process; this rig does not. The middle arm owns its camera, publishes rectified
images on the contract, and quest_driver subscribes -- so the frames arrive here
as ordinary ROS messages and this backend only forwards them. The camera
GEOMETRY travels the same way: see zed_camera_params().
"""
import os
import time

from . import Frame, Hand
import quest_config as cfg


def zed_camera_params(left_info, right_info, width=None, height=None):
    """ROS CameraInfo pair -> the gvlink camera wire dict.

    The viewer places each eye from measured intrinsics rather than from an
    operator's field-of-view guess, so it has to be told the geometry of the
    frames ACTUALLY SENT -- the rectified ones.

    For a ZED that is a straight read, and this is the one place the ZED is
    genuinely easier than the OAK giava uses: CameraInfo.P is already the
    rectified projection matrix (the same thing cv2.stereoRectify returns as
    P1/P2), so there is no rectification to redo and no chance of describing a
    different rectification than the one in the pixels. fx = P[0], cx = P[2],
    fy = P[5], cy = P[6], and the baseline falls out of the right eye's fourth
    column: P2[3] = -fx * b.

    Scaled when quest_driver resizes for the headset: intrinsics are in pixels,
    so they must follow the resize or the viewer places a correctly rectified
    image at the wrong scale.
    """
    P1, P2 = list(left_info.p), list(right_info.p)
    w, h = int(left_info.width), int(left_info.height)
    fx = float(P1[0])
    if fx <= 0.0:
        raise ValueError("left camera_info has no projection matrix yet")
    baseline = abs(float(P2[3]) / fx)

    sx = sy = 1.0
    if width and height and (width != w or height != h):
        sx, sy = float(width) / w, float(height) / h
        w, h = int(width), int(height)

    def eye(P):
        return {"fx": float(P[0]) * sx, "fy": float(P[5]) * sy,
                "cx": float(P[2]) * sx, "cy": float(P[6]) * sy}

    return {"w": w, "h": h, "b": baseline, "rect": True,
            "l": eye(P1), "r": eye(P2)}


class GvLinkBackend:
    def __init__(self, **_):
        self.headset = None
        self._params_sent = False

    def start(self):
        # Imported here, not at module scope, so the other backends still work
        # in an image without av/msgpack or without the gvlink checkout.
        try:
            from giava.gvlink_headset import GvLinkHeadset
        except ImportError as e:
            raise SystemExit(
                f"the gvlink backend needs the protocol library: {e}\n"
                "  clone av-aloha-unity (branch v2) and point GVLINK_PATH at\n"
                "  its Guided-Vision/python directory -- see quest/README.md")

        kwargs = {}
        if cfg.ROBOT_NAME:
            kwargs["name"] = cfg.ROBOT_NAME
        # Source size: what quest_driver resizes frames to before sending.
        kwargs["src_size"] = (cfg.STEREO_WIDTH, cfg.STEREO_HEIGHT)
        self.headset = GvLinkHeadset(**kwargs)
        self.headset.run_in_thread()
        print(f"  gvlink backend up: beacon out, waiting for the headset to "
              f"connect (robot name {cfg.ROBOT_NAME or 'default'})")

    # ---- device state ----------------------------------------------------
    def read(self):
        if self.headset is None:
            return None
        d = self.headset.receive_data()
        if d is None:
            return None
        return self._to_frame(d)

    def _to_frame(self, d):
        """HeadsetData -> Frame. Already right-handed; see giava/headset_utils.

        A ZERO QUATERNION IS A NORMAL PACKET, not corruption: Unity sends
        default(GvControllerState) for both controllers whenever the runtime is
        hand-tracking, and C# zero-initialises it. The pose is passed through
        as it arrives and `tracked` says whether it means anything, so the
        mapping layer can hold that arm rather than driving it from zeros.
        """
        def hand(pos, quat, sx, sy, trig, grip, b1, b2, bstick, tracked):
            return Hand(
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                orientation=(float(quat[0]), float(quat[1]),
                             float(quat[2]), float(quat[3])),
                axes=(float(sx), float(sy), float(trig), float(grip)),
                buttons=(int(bool(b1)), int(bool(b2)), int(bool(bstick)), 0),
                tracked=bool(tracked),
            )

        def is_tracked(pos, quat):
            return any(abs(float(v)) > 1e-9 for v in pos) or \
                   any(abs(float(v)) > 1e-9 for v in quat)

        return Frame(
            head_position=(float(d.h_pos[0]), float(d.h_pos[1]), float(d.h_pos[2])),
            head_orientation=(float(d.h_quat[0]), float(d.h_quat[1]),
                              float(d.h_quat[2]), float(d.h_quat[3])),
            left=hand(d.l_pos, d.l_quat, d.l_thumbstick_x, d.l_thumbstick_y,
                      d.l_index_trigger, d.l_hand_trigger,
                      d.l_button_one, d.l_button_two, d.l_button_thumbstick,
                      is_tracked(d.l_pos, d.l_quat)),
            right=hand(d.r_pos, d.r_quat, d.r_thumbstick_x, d.r_thumbstick_y,
                       d.r_index_trigger, d.r_hand_trigger,
                       d.r_button_one, d.r_button_two, d.r_button_thumbstick,
                       is_tracked(d.r_pos, d.r_quat)),
            stamp=time.monotonic(),
        )

    # ---- the return path -------------------------------------------------
    def send_feedback(self, feedback):
        if self.headset is not None:
            self.headset.send_feedback(feedback)

    def send_stereo(self, left_image, right_image, frame_id=0):
        """One stereo pair. Returns immediately -- encoding is on its own thread.

        If capture outruns the encoder the intermediate pairs are DISCARDED
        rather than queued: a frame from two captures ago is worth less than
        the current one, and a queue would only add latency the operator feels
        as the view lagging their head.
        """
        if self.headset is None or left_image is None or right_image is None:
            return
        # ROS images arrive rgb24 (quest_driver unpacks them that way for the
        # legacy webrtc track); gvlink encodes bgr24.
        self.headset.send_images(left_image[:, :, ::-1], right_image[:, :, ::-1])

    def set_camera_params(self, wire):
        """Tell the viewer the real geometry. Safe to call before it connects."""
        if self.headset is None or wire is None:
            return False
        ok = self.headset.set_camera_params(wire)
        if ok and not self._params_sent:
            self._params_sent = True
            print(f"  camera geometry -> headset: {wire['w']}x{wire['h']} "
                  f"fx={wire['l']['fx']:.0f} baseline={wire['b'] * 1000:.0f} mm")
        return ok

    def stop(self):
        if self.headset is not None:
            self.headset.close()
            self.headset = None
