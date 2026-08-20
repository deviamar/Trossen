"""WebRTC data channel from the Unity app on the headset. The real backend.

Wraps giava's WebRTCHeadset -- aiortc peer connection, Firestore signalling, a
"control" data channel carrying controller state, and two outbound video tracks
for the stereo view. That code is proven against the Unity APK you already have,
so this file adds no protocol of its own: it starts the connection, converts
HeadsetData into the Frame this container's driver expects, and gets out of the
way.

    ./quest_driver.py --backend webrtc

NEEDS TWO SECRETS, mounted read-only at QUEST_SECRETS_DIR (default /secrets):

    serviceAccountKey.json    Firestore service account, for signalling
    signalingSettings.json    robotID, password, TURN server settings

Both are gitignored upstream and must stay out of the image. See
quest/docker-compose.yml for the mount, and quest/README.md for how to get them
into place.

FRAME CONVENTION. giava's on_message() already calls
convert_left_to_right_coordinates() on every pose, which flips Unity's
left-handed Y-up frame into a right-handed one and applies TRANSFORM_TO_WORLD
(-90 deg about x, -90 about z). So HeadsetData is already right-handed by the
time it reaches here and this backend does no further conversion -- unlike
backends/udp.py, which has to do it itself for an app that sends raw Unity
values. Doing it twice would look like a tracking fault, not a bug.

EYE TRACKING is carried on the same channel (l_eye/r_eye pixel coordinates,
frame ids) and is dropped here -- Frame has nowhere to put it and nothing in
this rig consumes it yet. If you want gaze in the ROS graph, that is a new topic
on the contract, not a change to this file.
"""
import time

from . import Frame, Hand
import quest_config as cfg


class WebRtcBackend:
    def __init__(self, **_):
        self.headset = None
        self._warned = False

    def start(self):
        # Imported here, not at module scope, so `--backend sim` and
        # `--backend udp` still work in an image without aiortc/firestore.
        try:
            from giava.webrtc_headset import WebRTCHeadset
        except ImportError as e:
            raise SystemExit(
                f"the webrtc backend needs the vendored giava deps: {e}\n"
                "  pip install aiortc av opencv-python google-cloud-firestore "
                "scipy numba\n"
                "  (they are in quest/requirements.txt -- rebuild the image)"
            )

        self.headset = WebRTCHeadset()
        # run_in_thread() owns its own asyncio loop on a background thread and
        # returns immediately; the connection completes asynchronously. read()
        # returning None until then is the normal startup path, not an error.
        self.headset.run_in_thread()
        print("  webrtc backend: offer placed, waiting for the headset to answer")
        print(f"  secrets from {cfg.SECRETS_DIR}")

    def read(self):
        if self.headset is None:
            return None
        data = self.headset.receive_data()
        if data is None:
            return None
        return self._to_frame(data)

    def _to_frame(self, d):
        """HeadsetData -> Frame. Already right-handed; see the module docstring."""
        def hand(pos, quat, stick_x, stick_y, trigger, grip, b1, b2, bstick):
            return Hand(
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                orientation=(float(quat[0]), float(quat[1]),
                             float(quat[2]), float(quat[3])),
                axes=(float(stick_x), float(stick_y), float(trigger), float(grip)),
                # Menu is index 3 on the contract; the Unity app does not send it,
                # so it is reported as unpressed rather than omitted -- a short
                # buttons list would make every consumer bounds-check.
                buttons=(int(bool(b1)), int(bool(b2)), int(bool(bstick)), 0),
                tracked=True,
            )

        return Frame(
            head_position=(float(d.h_pos[0]), float(d.h_pos[1]), float(d.h_pos[2])),
            head_orientation=(float(d.h_quat[0]), float(d.h_quat[1]),
                              float(d.h_quat[2]), float(d.h_quat[3])),
            left=hand(d.l_pos, d.l_quat, d.l_thumbstick_x, d.l_thumbstick_y,
                      d.l_index_trigger, d.l_hand_trigger,
                      d.l_button_one, d.l_button_two, d.l_button_thumbstick),
            right=hand(d.r_pos, d.r_quat, d.r_thumbstick_x, d.r_thumbstick_y,
                       d.r_index_trigger, d.r_hand_trigger,
                       d.r_button_one, d.r_button_two, d.r_button_thumbstick),
            stamp=time.monotonic(),
        )

    # ---- the return path: feedback and video ----------------------------
    # Only this backend has one. The headset is the sole "device" in the rig
    # that is also a display, so quest_driver checks for these by name rather
    # than every backend having to carry no-op stubs.

    def send_feedback(self, feedback):
        """feedback: a giava HeadsetFeedback. Renders the arms in VR."""
        if self.headset is not None:
            self.headset.send_feedback(feedback)

    def send_stereo(self, left_image, right_image, frame_id=0):
        if self.headset is None:
            return
        if left_image is not None:
            self.headset.send_left_image(left_image, frame_id)
        if right_image is not None:
            self.headset.send_right_image(right_image, frame_id)

    def stop(self):
        if self.headset is not None:
            self.headset.close()
            self.headset = None
