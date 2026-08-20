#!/usr/bin/env python3
"""Arrow-key teleop for the SLATE mobile base.

    ./teleop_keyboard.py                    # 0.15 m/s, 0.4 rad/s
    ./teleop_keyboard.py --linear 0.25 --angular 0.6
    ./teleop_keyboard.py --hold 0.4         # longer coast after a keypress

MUST be run on a real terminal (an interactive `docker compose exec slate-base
bash`), not through `exec -T` -- it puts the tty in cbreak mode to read single
keys. Needs ./launch-base.sh running in another shell.

Controls
    up / down       drive forwards / backwards
    left / right    turn counter-clockwise / clockwise
    (arrows combine: up+left held together is an arc)
    [ / ]           speed down / up
    space           STOP now
    ESC or Ctrl-C   stop and quit

DEAD-MAN BY DESIGN. This is not the arm teleop, where each keypress nudges a
joint to a new position and it stays there. Here a keypress means "keep moving
while I hold this", and the base stops when you let go -- after --hold seconds
with no key repeat. That is the safe default for a machine that is moving
across a floor rather than holding a pose.

The mechanism is the terminal's own key-repeat: holding a key sends the escape
sequence over and over, and each arrival pushes the deadline out. So the coast
after release is --hold, and the delay before repeat kicks in (a few hundred ms
on most terminals) shows up as a brief stutter at the start of a hold. Raising
--hold smooths that out at the cost of a longer coast; it is capped below at the
driver's own 300 ms /cmd_vel deadline, since anything shorter would be overruled
by the driver stopping first anyway.
"""
import os
import select
import sys
import termios
import tty

import rclpy

import base_config as cfg
import slate

# Escape sequences for the arrow keys, as xterm and everything compatible sends
# them. (linear_sign, angular_sign) -- positive angular is counter-clockwise,
# per REP-103.
KEYMAP = {
    "\x1b[A": (+1, 0),   # up
    "\x1b[B": (-1, 0),   # down
    "\x1b[D": (0, +1),   # left
    "\x1b[C": (0, -1),   # right
}

HELP = ("  arrows drive/turn   [ ] speed   space stop   ESC quit")


def parse_keys(data):
    """Split raw terminal bytes into key tokens.

    An arrow key is three bytes -- ESC [ A/B/C/D -- and has to come back as one
    token, or it reads as a bare ESC (quit) followed by stray letters.
    """
    keys = []
    i = 0
    while i < len(data):
        if data[i:i + 1] == b"\x1b" and data[i + 1:i + 2] == b"[" and len(data) > i + 2:
            keys.append(data[i:i + 3].decode("latin-1"))
            i += 3
        else:
            keys.append(data[i:i + 1].decode("latin-1"))
            i += 1
    return keys


def read_keys(timeout_s):
    """Everything readable within timeout_s, as a list of key tokens.

    os.read on the raw fd, NOT sys.stdin.read -- and that is the whole reason
    this function exists rather than being three lines inline.

    sys.stdin is a buffered TextIOWrapper. Reading one character from it pulls
    a whole chunk out of the OS buffer into Python's own, so a following
    select() on the file descriptor reports "nothing to read" while the rest of
    the escape sequence is sitting in userspace. The first version of this did
    exactly that: pressing an arrow key read the ESC, found no more bytes on the
    fd, decided it was a bare ESC, and quit. The base could not be driven at all
    and it looked like the script was exiting on startup. select() and buffered
    IO cannot be mixed on the same stream.

    Draining everything available in one read also keeps the dead-man honest.
    Key repeat during a hold arrives faster than the 20 Hz publish loop consumes
    it, so a one-key-per-tick read would fall progressively further behind and
    the base would keep moving well after release.

    Raises EOFError when stdin closes, so a piped or detached run ends cleanly
    instead of spinning on an endless stream of empty reads.
    """
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout_s)[0]:
        return []
    data = os.read(fd, 1024)
    if not data:
        raise EOFError

    # A keypress lands in one read in practice, but an escape sequence CAN be
    # split across two if the terminal is slow. Treating that as a bare ESC
    # would quit on an arrow key, so give a trailing lone ESC a moment to be
    # completed before believing it.
    if data.endswith(b"\x1b") and select.select([fd], [], [], 0.03)[0]:
        data += os.read(fd, 1024)

    return parse_keys(data)


def main():
    ap = slate.parser(__doc__)
    ap.add_argument("--linear", type=float, default=0.15, help="m/s (default 0.15)")
    ap.add_argument("--angular", type=float, default=0.4, help="rad/s (default 0.4)")
    ap.add_argument("--hold", type=float, default=0.25,
                    help="seconds to keep moving after the last keypress")
    args = ap.parse_args()

    if not sys.stdin.isatty():
        print("teleop needs a real terminal -- run it from inside an interactive\n"
              "`docker compose exec slate-base bash`, not through `exec -T`.",
              file=sys.stderr)
        return 2

    # Below the driver's own deadline, --hold does nothing: the driver would
    # zero the command before our timer fired.
    hold = max(args.hold, cfg.CMD_TIMEOUT_S)
    linear_step, angular_step = abs(args.linear), abs(args.angular)

    rclpy.init()
    node = slate.VelocityDriver("slate_teleop")
    try:
        if not slate.wait_for_subscriber(node):
            print(f"Nothing is subscribed to {cfg.TOPIC_CMD_VEL_TELEOP} -- the\n"
                  "governor is not running. Commands go through it to be\n"
                  "clamped, so without it nothing reaches the base.\n"
                  "    ./governor.py     (or set AUTOSTART=true)",
                  file=sys.stderr)
            return 1

        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        period = 1.0 / cfg.PUBLISH_HZ
        lin_sign = ang_sign = 0
        # One deadline per AXIS, not one for the keyboard. A single shared
        # timestamp means holding `up` after having once tapped `left` keeps
        # refreshing the turn as well, and the base arcs indefinitely on a key
        # nobody is pressing. Each axis has to expire on its own key repeat.
        lin_at = ang_at = None
        print(HELP + "\n")

        try:
            while rclpy.ok():
                for key in read_keys(period):
                    if key in ("\x1b", "\x03"):        # ESC, Ctrl-C
                        raise KeyboardInterrupt
                    if key == " ":
                        lin_sign = ang_sign = 0
                        lin_at = ang_at = None
                        node.stop()
                        continue
                    if key == "]":
                        linear_step = min(cfg.CLAMP_VEL_X, linear_step + 0.05)
                        angular_step = min(cfg.CLAMP_VEL_Z, angular_step + 0.1)
                        continue
                    if key == "[":
                        linear_step = max(0.05, linear_step - 0.05)
                        angular_step = max(0.1, angular_step - 0.1)
                        continue
                    if key in KEYMAP:
                        l, a = KEYMAP[key]
                        # Arrows combine rather than replace, so up and left
                        # held together is an arc. Each arrow refreshes only the
                        # axis it names.
                        now = node.get_clock().now()
                        if l:
                            lin_sign, lin_at = l, now
                        if a:
                            ang_sign, ang_at = a, now

                # Dead-man, per axis: an arrow not repeating means release it.
                now = node.get_clock().now()
                if lin_at is not None and (now - lin_at).nanoseconds / 1e9 > hold:
                    lin_sign, lin_at = 0, None
                if ang_at is not None and (now - ang_at).nanoseconds / 1e9 > hold:
                    ang_sign, ang_at = 0, None

                lin = lin_sign * linear_step
                ang = ang_sign * angular_step
                lin, ang, _ = cfg.clamp(lin, ang)
                node.send(lin, ang)
                rclpy.spin_once(node, timeout_sec=0.0)

                moving = "MOVING" if (lin or ang) else "  --  "
                sys.stdout.write(
                    f"\r  {moving}  linear {lin:+.2f} m/s  angular {ang:+.2f} rad/s"
                    f"   step {linear_step:.2f}/{angular_step:.2f}    ")
                sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            # Stop under the restored terminal settings, and before printing
            # anything -- the base coasting while a goodbye message scrolls is
            # the wrong order of events.
            node.stop()
            print("\n  stopped.")
        return 0
    except (KeyboardInterrupt, EOFError):
        # EOFError is read_keys reporting that stdin closed. Same handling as
        # Ctrl-C: the finally above has already stopped the base and put the
        # terminal back.
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
