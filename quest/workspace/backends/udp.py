"""UDP JSON datagrams from an app on the headset. The reference backend.

This is the simplest thing that works and the easiest to debug: one JSON object
per datagram, one datagram per tracking frame, fire and forget. If a frame is
lost, the next one is along in ~14 ms and it carries absolute state, so there is
nothing to reassemble and no session to recover.

It is here as the backend you can actually run today, and as a worked example of
what a backend has to do. If the rig settles on WebRTC, backends/webrtc.py is
the place for it and nothing outside this directory changes.

PACKET FORMAT
-------------
Send to this machine's IP on QUEST_UDP_PORT (default 9871):

    {
      "unity": true,
      "head":  {"p": [x, y, z], "q": [x, y, z, w]},
      "left":  {"p": [...], "q": [...], "axes": [sx, sy, trig, grip],
                "buttons": [primary, secondary, stick, menu], "tracked": true},
      "right": { ... same ... }
    }

"unity": true means the app is sending Unity's own left-handed Y-up frame and
this backend converts. Send "unity": false (or omit it) if the app has already
converted to ROS convention -- x forward, y left, z up, right-handed.

Getting that flag wrong is the single most likely source of "teleop works but
one axis is mirrored". See quest_config for why it cannot be fixed downstream.
"""
import json
import socket

from . import Frame, Hand
import quest_config as cfg


class UdpBackend:
    def __init__(self, port=9871, bind="0.0.0.0", **_):
        self.port = int(port)
        self.bind = bind
        self.sock = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind, self.port))
        # Non-blocking: read() is called from the node's timer and must never
        # stall the executor waiting for a headset that has gone away.
        self.sock.setblocking(False)
        print(f"  udp backend listening on {self.bind}:{self.port}")

    def read(self):
        if self.sock is None:
            return None
        newest = None
        # Drain the socket every tick and keep only the last packet. Under load
        # the queue holds stale frames, and acting on the oldest one first would
        # add latency that grows the busier things get.
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            newest = data
        if newest is None:
            return None
        try:
            return self._parse(json.loads(newest.decode("utf-8")))
        except (ValueError, KeyError, TypeError) as e:
            print(f"  malformed packet ignored: {e}")
            return None

    def _parse(self, d):
        unity = bool(d.get("unity", True))

        def conv_p(p):
            return cfg.unity_to_ros_position(*p) if unity else tuple(p)

        def conv_q(q):
            return cfg.unity_to_ros_quaternion(*q) if unity else tuple(q)

        def hand(key):
            h = d.get(key) or {}
            return Hand(
                position=conv_p(h.get("p", [0, 0, 0])),
                orientation=conv_q(h.get("q", [0, 0, 0, 1])),
                axes=tuple(float(v) for v in h.get("axes", [0, 0, 0, 0])),
                buttons=tuple(int(v) for v in h.get("buttons", [0, 0, 0, 0])),
                tracked=bool(h.get("tracked", True)),
            )

        head = d.get("head") or {}
        return Frame(
            head_position=conv_p(head.get("p", [0, 0, 0])),
            head_orientation=conv_q(head.get("q", [0, 0, 0, 1])),
            left=hand("left"),
            right=hand("right"),
        )

    def stop(self):
        if self.sock:
            self.sock.close()
            self.sock = None
