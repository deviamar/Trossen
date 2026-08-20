"""Transport backends: whatever gets controller state off the headset.

The rest of the container does not care how frames arrive. A backend is any
object with start(), read() and stop(), returning Frame objects in ROS
convention. That seam exists because the link to the headset is the least
settled part of this rig -- WebRTC today, possibly something else tomorrow --
and none of the robot-facing code should have to care.

    start()   open the link. Raise on unrecoverable setup failure.
    read()    return the newest Frame, or None if nothing new. Must not block
              for longer than a frame period.
    stop()    close cleanly. Called on Ctrl-C.

A backend converts to ROS convention BEFORE returning (see
quest_config.unity_to_ros_*). It is the only place that knows what the app on
the headset actually sends.
"""
import dataclasses
import time


@dataclasses.dataclass
class Hand:
    """One controller. Poses in ROS convention, metres, quaternion normalised."""
    position: tuple = (0.0, 0.0, 0.0)
    orientation: tuple = (0.0, 0.0, 0.0, 1.0)   # x, y, z, w
    axes: tuple = (0.0, 0.0, 0.0, 0.0)          # stick x, stick y, trigger, grip
    buttons: tuple = (0, 0, 0, 0)               # primary, secondary, stick, menu
    tracked: bool = False


@dataclasses.dataclass
class Frame:
    """One tracking update: head plus both hands."""
    head_position: tuple = (0.0, 0.0, 0.0)
    head_orientation: tuple = (0.0, 0.0, 0.0, 1.0)
    left: Hand = dataclasses.field(default_factory=Hand)
    right: Hand = dataclasses.field(default_factory=Hand)
    stamp: float = dataclasses.field(default_factory=time.monotonic)


def load(name, **kwargs):
    """Backend by name. Imported lazily so one backend's deps stay optional."""
    if name == "udp":
        from .udp import UdpBackend
        return UdpBackend(**kwargs)
    if name == "sim":
        from .sim import SimBackend
        return SimBackend(**kwargs)
    if name == "webrtc":
        from .webrtc import WebRtcBackend
        return WebRtcBackend(**kwargs)
    raise SystemExit(f"unknown backend {name!r}; have: udp, sim, webrtc")
