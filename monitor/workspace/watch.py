#!/usr/bin/env python3
"""Watch every topic on the rig, live, in one table.

    ./watch.py                    # everything
    ./watch.py --filter slate     # only topics whose name contains this
    ./watch.py --once             # one snapshot, then exit (for piping)
    ./watch.py --hz 2             # redraw rate

Subscribes to every topic it can find and shows name, type, rate and the latest
value. It publishes nothing and commands nothing -- it is safe to leave running
next to anything.

WHY NOT `ros2 topic list` AND A PILE OF ECHOES. Because the question is almost
never "what is on this one topic". It is "which half of the rig has gone quiet",
and that needs everything on one screen with rates next to it. A topic that
exists but publishes at 0 Hz looks identical to a healthy one in `topic list`,
and that is the single most common failure here: a node that started, holds its
publisher, and is not actually talking to hardware.

DISCOVERY IS CONTINUOUS. New topics appear as their nodes start, so you can
leave this running through a `docker compose up` and watch the graph assemble.

The value column is a summary, not the message. Use `ros2 topic echo` when you
need the whole thing.
"""
import argparse
import importlib
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Topics that are noise for this purpose: every node publishes them and nobody
# is ever debugging the rig by watching them.
SKIP = ("/parameter_events", "/rosout")


def import_type(type_string):
    """'std_msgs/msg/Bool' -> the class."""
    pkg, kind, name = type_string.split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), name)


def summarize(msg):
    """One short line. Deliberately lossy."""
    t = type(msg).__name__
    try:
        if t == "Twist":
            return f"x {msg.linear.x:+.3f} m/s   yaw {msg.angular.z:+.3f} rad/s"
        if t in ("Bool",):
            return str(msg.data)
        if t in ("Float32", "Float64", "Int32", "Int64", "String"):
            v = msg.data
            return (v[:60] + "...") if isinstance(v, str) and len(v) > 60 else str(v)
        if t == "JointState":
            return " ".join(f"{p:+.3f}" for p in list(msg.position)[:7])
        if t == "PoseStamped":
            p, o = msg.pose.position, msg.pose.orientation
            return (f"({p.x:+.3f} {p.y:+.3f} {p.z:+.3f}) "
                    f"q({o.x:+.2f} {o.y:+.2f} {o.z:+.2f} {o.w:+.2f})")
        if t == "Odometry":
            p = msg.pose.pose.position
            tw = msg.twist.twist
            return (f"x {p.x:+.3f} y {p.y:+.3f}  v {tw.linear.x:+.3f} "
                    f"w {tw.angular.z:+.3f}")
        if t == "BatteryState":
            return f"{msg.percentage:.0f}%  {msg.voltage:.1f} V"
        if t == "Joy":
            ax = " ".join(f"{a:+.2f}" for a in list(msg.axes)[:4])
            btn = "".join(str(int(b)) for b in list(msg.buttons)[:6])
            return f"axes[{ax}] buttons[{btn}]"
        if t == "Image":
            return f"{msg.width}x{msg.height} {msg.encoding}"
    except Exception:
        pass
    return f"<{t}>"


class Watcher(Node):
    def __init__(self, args):
        super().__init__("rig_watch")
        self.args = args
        self.subs = {}
        self.last = {}      # topic -> (summary, monotonic stamp)
        self.count = {}
        self.window = {}    # topic -> [stamps] for the rate estimate
        self.create_timer(1.0, self._discover)
        self._discover()

    def _discover(self):
        for name, types in self.get_topic_names_and_types():
            if name in self.subs or name in SKIP:
                continue
            if self.args.filter and self.args.filter not in name:
                continue
            try:
                cls = import_type(types[0])
            except Exception:
                continue
            # Best-effort, depth 1, volatile: this must never slow a publisher
            # down or hold a queue on its behalf. Sensor-style QoS also matches
            # the camera topics, which reliable QoS would silently miss.
            qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
            try:
                self.subs[name] = self.create_subscription(
                    cls, name, lambda m, n=name: self._on(n, m), qos)
                self.count[name] = 0
                self.window[name] = []
                self.last[name] = (types[0].split("/")[-1], "waiting", 0.0)
            except Exception:
                continue

    def _on(self, name, msg):
        now = time.monotonic()
        self.count[name] += 1
        w = self.window[name]
        w.append(now)
        if len(w) > 20:
            del w[0]
        kind = self.last[name][0]
        self.last[name] = (kind, summarize(msg), now)

    def rate(self, name):
        w = self.window[name]
        if len(w) < 2:
            return 0.0
        span = w[-1] - w[0]
        return (len(w) - 1) / span if span > 1e-6 else 0.0

    def render(self):
        now = time.monotonic()
        rows = [f"  {'topic':<44}{'type':<16}{'Hz':>7}  value",
                "  " + "-" * 104]
        for name in sorted(self.last):
            kind, value, stamp = self.last[name]
            hz = self.rate(name)
            age = now - stamp if stamp else 999
            # A topic that has published and then stopped is the interesting
            # case, so it gets called out rather than just decaying to 0.0 Hz.
            if stamp and age > 3.0:
                flag = f"STALE {age:.0f}s"
            elif not stamp:
                flag = "--"
            else:
                flag = f"{hz:6.1f}"
            rows.append(f"  {name:<44}{kind:<16}{flag:>7}  {value}")
        rows.append("")
        rows.append(f"  {len(self.last)} topics.  Ctrl-C to stop.")
        return "\n".join(rows)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--filter", help="only topics whose name contains this")
    ap.add_argument("--hz", type=float, default=4.0, help="redraw rate")
    ap.add_argument("--once", action="store_true", help="one snapshot, then exit")
    args = ap.parse_args()

    rclpy.init()
    node = Watcher(args)
    lines = 0
    try:
        # Let discovery and a first message land before drawing, or the first
        # frame is a screen of "waiting" that scrolls away.
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        if args.once:
            print(node.render())
            return 0

        while rclpy.ok():
            end = time.monotonic() + 1.0 / args.hz
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            block = node.render()
            if lines:
                sys.stdout.write(f"\033[{lines}A")
            lines = block.count("\n") + 1
            sys.stdout.write("\033[J" + block + "\n")
            sys.stdout.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
