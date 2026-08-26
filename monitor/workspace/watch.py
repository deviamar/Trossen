#!/usr/bin/env python3
"""Watch every topic on the rig, live, in one table.

    ./watch.py --dash             # ONE LINE PER SUBSYSTEM -- start here
    ./watch.py                    # every topic, one line each
    ./watch.py --filter slate     # only topics whose name contains this
    ./watch.py --once             # one snapshot, then exit (for piping)
    ./watch.py --hz 2             # redraw rate

Subscribes to every topic it can find and shows name, type, rate and the latest
value. It publishes nothing and commands nothing -- it is safe to leave running
next to anything.

--dash IS THE ONE TO LEAVE RUNNING. The full table is one line per topic, which
is 40-odd lines once the whole rig is up -- too tall for a tmux pane and mostly
things you are not watching. The dashboard collapses it to a line per
subsystem: where the base is, where each arm is, what is connected. Use the
full table when you are asking "is this topic alive at all", the dashboard when
you are driving.

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
            # covariance[0] is -1 when the E-stop is engaged and 1 otherwise --
            # the driver's own way of reporting it, since it publishes no state
            # topic. With the E-stop in, the wheels are braked and every
            # velocity command is accepted and ignored, silently.
            estop = " ESTOP" if msg.pose.covariance[0] < 0 else ""
            return (f"x {p.x:+.3f} y {p.y:+.3f}  v {tw.linear.x:+.3f} "
                    f"w {tw.angular.z:+.3f}{estop}")
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


    # ---- compact dashboard ----------------------------------------------
    # Deliberately hand-written rather than generated from the topic list: the
    # point is to show the few numbers that matter while driving, in a fixed
    # layout that does not reflow when a topic appears or goes quiet. A
    # dashboard whose lines move around is worse than no dashboard.
    def _get(self, topic):
        """(summary, age_seconds) or (None, None) if never seen."""
        e = self.last.get(topic)
        if not e or not e[2]:
            return None, None
        return e[1], time.monotonic() - e[2]

    def _cell(self, topic, width, missing="--"):
        """One value, TRUNCATED to fit. Never padded past the width.

        A tmux pane is often 40 columns; a cell padded to a fixed 44 wraps, and
        a wrapped dashboard is unreadable in exactly the situation it exists
        for. Truncate and let the full table carry the detail.
        """
        val, age = self._get(topic)
        if val is None:
            text = missing
        elif age > 3.0:
            text = f"STALE {int(age)}s"
        else:
            text = val
        return text[:width]

    def _width(self):
        import shutil
        # Pane width, minus the indent. Floor of 32 so a very narrow pane
        # degrades to something clipped rather than something reflowed.
        return max(32, shutil.get_terminal_size((80, 24)).columns - 2)

    def _xyz(self, topic):
        """Just the position from a PoseStamped summary.

        The full summary carries the quaternion too, which is the right thing
        in the wide table and simply does not fit a quarter-pane column -- it
        gets truncated mid-number, which is worse than omitting it.
        """
        val, age = self._get(topic)
        if val is None:
            return "--"
        if age > 3.0:
            return f"STALE {int(age)}s"
        return val.split(")")[0] + ")" if "(" in val else val

    def render_dash(self):
        """One line per thing, sized to the pane.

        No blank spacer lines and no trailing hint: a quarter of an 80x24
        terminal is about 12 rows, and anything taller scrolls the top off --
        so the BASE block, which is the part you are usually watching,
        disappears first. Every line here earns its row.
        """
        import shutil
        size = shutil.get_terminal_size((80, 24))
        w = max(32, size.columns - 1)
        val_w = max(14, w - 10)

        base_rows = [
            f"BASE odom {self._cell('/slate/odom', val_w)}",
            f"     cmd  {self._cell('/slate/cmd_vel', val_w)}",
            f"     batt {self._cell('/slate/battery_state', val_w)}",
            f"     lift {self._cell('/slate/lift/height', val_w, '(none)')}",
        ]
        odom = self._get("/slate/odom")[0]
        if odom is not None:
            # Its own line rather than relying on the tag surviving the odom
            # string's truncation in a narrow pane. "Nothing moves" has several
            # causes and this is the one with no error message anywhere.
            base_rows.append("     ESTOP ENGAGED -- wheels braked"
                             if "ESTOP" in odom else "     estop clear")

        arm_w = max(12, w - 12)
        arm_rows = []
        for ns, label in (("/left_arm", "left"), ("/right_arm", "right"),
                          ("/middle", "cam")):
            act, _ = self._get(f"{ns}/active")
            flag = {"True": "ON  ", "False": "idle"}.get(str(act), "--  ")
            arm_rows.append(f"{label:<5} {flag} {self._xyz(f'{ns}/ee_pose')[:arm_w]}")

        conn, _ = self._get("/quest/connected")
        tail = [f"quest {conn if conn is not None else 'off'}"
                f"  |  {len(self.last)} topics"]

        rows = base_rows + arm_rows + tail
        # Spend rows on separators only if the pane can spare them, and insert
        # them by position rather than a hardcoded index -- the BASE block
        # changes length depending on whether odom is present.
        if size.lines > len(rows) + 2:
            rows = base_rows + [""] + arm_rows + [""] + tail
        return "\n".join(rows)

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
    ap.add_argument("--dash", action="store_true",
                    help="one line per subsystem instead of one per topic")
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

        draw = node.render_dash if args.dash else node.render

        if args.once:
            print(draw())
            return 0

        while rclpy.ok():
            end = time.monotonic() + 1.0 / args.hz
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            block = draw()
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
