"""Side-by-side USB stereo camera -> a left/right pair for the headset.

    ./quest_driver.py --backend gvlink --camera /dev/video4

WHY THIS EXISTS AND WHAT IT IS NOT. The rig's intended eye is the ZED on the
middle arm, published by that container on the topic contract (the quest driver
subscribes to those topics and this file is not involved). This is the other
case: an ordinary UVC stereo camera plugged into the machine, which delivers
BOTH eyes in ONE frame, side by side -- 2560x720 is two 1280x720 images glued
together -- and needs no SDK, no GPU image and no vendor wrapper.

Captured here rather than published as ROS topics on purpose: this process is
the one that sends video to the headset, and a round trip through DDS for
frames that are about to be re-encoded anyway only adds latency and a copy.
The ZED path stays as it is; this is a second source, not a replacement.

NO CALIBRATION. The device reports no intrinsics, so the geometry handed to the
viewer is SYNTHESISED from a field of view and a baseline you give it
(QUEST_CAM_HFOV_DEG, QUEST_CAM_BASELINE_M). That is a real limitation and worth
being honest about: the stereo will fuse and the scale will be roughly right,
but it is not measured. Calibrate the pair and set the numbers, or use the ZED,
if depth judgement matters for the task.
"""
import os
import threading
import time

import numpy as np

DEVICE = os.environ.get("QUEST_CAMERA", "")
# 2560x720 is the useful middle mode: two 1280x720 eyes, MJPG, and it downsizes
# cleanly to the 640x480 the headset is sent.
CAPTURE_W = int(os.environ.get("QUEST_CAM_W", 2560))
CAPTURE_H = int(os.environ.get("QUEST_CAM_H", 720))
CAPTURE_FPS = int(os.environ.get("QUEST_CAM_FPS", 30))
# Uncalibrated: the viewer is told this instead of measured intrinsics.
HFOV_DEG = float(os.environ.get("QUEST_CAM_HFOV_DEG", 70.0))
BASELINE_M = float(os.environ.get("QUEST_CAM_BASELINE_M", 0.06))
SWAP_EYES = os.environ.get("QUEST_CAM_SWAP_EYES", "false").lower() == "true"


def synth_camera_params(width, height, hfov_deg=HFOV_DEG, baseline_m=BASELINE_M):
    """The gvlink camera wire dict for a camera nobody has calibrated.

    Square pixels, centred principal point -- which is what the viewer would
    have assumed from a field-of-view slider anyway, so this is not pretending
    to know more than it does; it just moves the assumption to the end that
    owns the camera. `rect` is True because the two halves of an SBS frame are
    already coplanar and row-aligned as far as anything here can tell, and the
    viewer will not undistort either way.
    """
    import math
    fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    eye = {"fx": fx, "fy": fx, "cx": width / 2.0, "cy": height / 2.0}
    return {"w": int(width), "h": int(height), "b": float(baseline_m),
            "rect": True, "l": dict(eye), "r": dict(eye)}


class StereoCamera:
    """Grabs in a thread and keeps only the newest pair.

    Newest-only, not a queue: a frame from two captures ago is worth less than
    the current one to someone turning their head, and a queue would show up as
    the view lagging.
    """

    def __init__(self, device=None, width=CAPTURE_W, height=CAPTURE_H,
                 fps=CAPTURE_FPS, out_size=None):
        self.device = device or DEVICE
        self.width, self.height, self.fps = width, height, fps
        self.out_size = out_size
        self.cap = None
        self._pair = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.frames = 0
        self.failures = 0

    def open(self):
        import cv2
        dev = self.device
        # cv2 wants an index for V4L2; "/dev/video4" works as a path too, but the
        # index form is what honours CAP_PROP_* reliably on this driver.
        src = int(dev[len("/dev/video"):]) if dev.startswith("/dev/video") else dev
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open {dev!r} -- is it plugged in, and is "
                             "the device mounted into this container?")
        # MJPG before the size: at 2560x720 the YUYV bandwidth does not fit USB2
        # and the driver silently falls back to a smaller mode.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (w, h) != (self.width, self.height):
            print(f"  camera gave {w}x{h}, not {self.width}x{self.height} "
                  "-- using what it gave")
            self.width, self.height = w, h
        print(f"  stereo camera {dev}: {w}x{h} side-by-side "
              f"-> two {w // 2}x{h} eyes @ {self.fps} fps")
        return self

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        import cv2
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.failures += 1
                time.sleep(0.01)
                continue
            half = frame.shape[1] // 2
            left, right = frame[:, :half], frame[:, half:]
            if SWAP_EYES:
                left, right = right, left
            if self.out_size and (left.shape[1], left.shape[0]) != tuple(self.out_size):
                left = cv2.resize(left, tuple(self.out_size))
                right = cv2.resize(right, tuple(self.out_size))
            with self._lock:
                self._pair = (np.ascontiguousarray(left), np.ascontiguousarray(right))
            self.frames += 1

    def read(self):
        """Newest (left, right) BGR pair, or None if nothing has arrived yet."""
        with self._lock:
            return self._pair

    def camera_params(self):
        w = (self.out_size[0] if self.out_size else self.width // 2)
        h = (self.out_size[1] if self.out_size else self.height)
        return synth_camera_params(w, h)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
            self.cap = None
