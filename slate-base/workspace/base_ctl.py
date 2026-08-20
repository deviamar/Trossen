#!/usr/bin/env python3
"""The SLATE base's four service calls: screen text, light, torque, charging.

    ./base_ctl.py text "middle arm homing"   # write to the base's screen
    ./base_ctl.py light green                # solid green
    ./base_ctl.py light red-flash
    ./base_ctl.py light list                 # every accepted colour
    ./base_ctl.py torque off                 # let the base be pushed by hand
    ./base_ctl.py torque on
    ./base_ctl.py charge on                  # allow charging while docked

Needs ./launch-base.sh running in another shell.

None of these move the base, so unlike move_base.py there is no --execute gate.
`torque off` is the one to think about before running: it releases the drive
motors so the base rolls freely, which is what you want for repositioning it by
hand and emphatically not what you want on a slope or a ramp. Chock it first.

There is no service to reset odometry. trossen_slate exposes reset_odometry()
and its demo uses it, but interbotix_slate_driver does not wrap it -- the ROS
node's only odometry origin is the first sample after it starts. Restarting
./launch-base.sh is the way to re-zero.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from interbotix_slate_msgs.srv import SetLightState, SetString

import base_config as cfg
import slate


class Caller(Node):
    """Service client for the driver's four services.

    Two details here are load-bearing, and both were learned the hard way on
    real hardware -- the first version of this file got them wrong and produced
    a completely misleading diagnosis.

    ONE CLIENT PER SERVICE, REUSED. create_client() used to run on every call,
    so a retry loop built fifteen clients on one node and destroyed none. The
    driver then could not route its reply and logged
    `failed to send response ... (timeout)` -- while having executed the command
    perfectly. Fifty of those in one session, against fifty-seven successful
    torque writes and zero failed ones.

    SETTLE BEFORE THE FIRST REQUEST. wait_for_service() returns once the CLIENT
    has discovered the service. It says nothing about the server having
    discovered the client's reply reader, and these CLIs are short-lived
    processes that send a request milliseconds after starting. Spinning briefly
    first gives that second half of discovery time to complete, which is what
    stops the response going missing.
    """

    def __init__(self):
        super().__init__("slate_base_ctl")
        # NOT self._clients -- that is rclpy Node's own list of service clients,
        # which create_client() appends to. Shadowing it with a dict makes the
        # very first create_client() raise AttributeError: 'dict' object has no
        # attribute 'append'.
        self._client_cache = {}

    def _client(self, srv_type, name, timeout_s, settle_s):
        """Get-or-create the client for `name`, settling discovery on creation."""
        if name in self._client_cache:
            return self._client_cache[name]

        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None

        deadline = self.get_clock().now().nanoseconds + int(settle_s * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        self._client_cache[name] = client
        return client

    def call(self, srv_type, name, request, timeout_s=5.0, settle_s=0.5):
        """Call a service and return its response, or None if it did not answer."""
        client = self._client(srv_type, name, timeout_s, settle_s)
        if client is None:
            print(f"  service {name} did not appear within {timeout_s:.0f}s.\n"
                  "  Is the driver running?  ./launch-base.sh  (in another shell)\n"
                  f"  Does it exist?          ros2 service list | grep {cfg.NS.lstrip('/')}",
                  file=sys.stderr)
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            return None
        return future.result()


def call_retrying(node, srv_type, name, request, attempts):
    """Call a service, retrying while the BASE reports failure. (res, tries).

    Only a `success: false` is retried -- that is the driver telling us its
    Modbus write did not round-trip, which is worth one more go. A None (no
    response at all) is returned immediately: it means the driver is not there,
    and hammering it only delays the error that actually helps.

    Both retried commands are idempotent -- each sets one bit in sys_cmd_ and
    writes the word -- so repeating one that may already have taken effect
    cannot do anything the single call would not.

    A WARNING ABOUT WHAT THIS IS NOT FOR. During bring-up on this rig,
    `torque on` appeared to fail about four times in five while the driver log
    showed nothing but successes. It looked exactly like a flaky serial write,
    and an earlier version of this function said so at length. It was not. Two
    slate_base_node processes were running -- one left over from a closed
    terminal, holding a /dev/ttyUSB0 that had since re-enumerated away. Both
    offered the same services, requests round-robined between them, and the one
    with the dead file descriptor failed every call it happened to receive.
    With a single driver: six calls, six successes, no retries, and zero
    `failed to send response` warnings in the log.

    So if these calls start failing again, DO NOT reach for a bigger retry
    count. Check for a second driver first -- launch-base.sh now refuses to
    start one, but a driver started some other way will still do it.
    """
    res = None
    for attempt in range(1, attempts + 1):
        res = node.call(srv_type, name, request)
        if res is None or res.success:
            return res, attempt
    return res, attempts


def report(res, tries=1):
    """Print a SetBool/SetString/SetLightState response. Returns an exit code.

    All four services share the same (success, message) response shape, and the
    driver fills the message in with something specific, so it is worth showing
    rather than translating into a generic OK.

    The attempt count is printed rather than hidden. A command that needed four
    tries is working, but it is also telling you something about the serial link
    -- and silently swallowing that would turn a visible flake into a mystery
    the next time it gets worse.
    """
    if res is None:
        return 1
    suffix = f"  (took {tries} attempts)" if tries > 1 else ""
    print(f"  {'ok' if res.success else 'FAILED'}: {res.message}{suffix}")
    if not res.success:
        print(
            f"  Gave up after {tries} attempts.\n"
            "\n"
            "  FIRST, CHECK FOR A SECOND DRIVER. Two slate_base_node processes\n"
            "  offer the same services, requests round-robin between them, and\n"
            "  only one holds the serial port -- so the other fails every call\n"
            "  it receives while the driver log shows nothing but successes.\n"
            "  That is what this looked like during bring-up, and it was not a\n"
            "  serial fault at all:\n"
            "      pgrep -af '[s]late_base_node'      # expect ONE driver\n"
            "      ros2 node list                     # expect ONE slate_base\n"
            "\n"
            "  If there is genuinely only one, then the base really is refusing\n"
            "  the write. Check its screen for a mode or error. Note that there\n"
            "  is no way to read the setting back -- the driver publishes no\n"
            "  torque or charging state -- so the practical test is to try what\n"
            "  you wanted to do: if the base moves, torque is on.",
            file=sys.stderr)
    return 0 if res.success else 1


def main():
    ap = slate.parser(__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_text = sub.add_parser("text", help="write to the base's screen")
    p_text.add_argument("message", help="text to display")

    p_light = sub.add_parser("light", help="set the light bar")
    p_light.add_argument("colour", help="a colour name, or 'list'")

    # --retries belongs to the two subcommands that need it, not to the top-level
    # parser: an option added to `ap` is only accepted BEFORE the subcommand
    # name, so `./base_ctl.py torque on --retries 30` would be an error. Same
    # argparse trap as move_base.py's --execute.
    #
    # 3, not 15. An earlier version used 15 to paper over what looked like a
    # 1-in-5 serial flake and was actually a duplicate driver -- so it turned
    # one command into fifteen executions of it against real hardware while
    # reporting failure. With a single driver these calls do not fail at all,
    # and a retry count large enough to hide a structural problem is a liability,
    # not insurance.
    retry = argparse.ArgumentParser(add_help=False)
    retry.add_argument("--retries", type=int, default=3,
                       help="attempts before giving up (default 3)")

    p_torque = sub.add_parser("torque", parents=[retry],
                              help="drive motor torque on/off")
    p_torque.add_argument("state", choices=["on", "off"])

    p_charge = sub.add_parser("charge", parents=[retry],
                              help="allow/disallow charging")
    p_charge.add_argument("state", choices=["on", "off"])

    args = ap.parse_args()

    # Handled before rclpy.init() so it works with no driver and no ROS graph.
    if args.cmd == "light" and args.colour == "list":
        print("  " + "\n  ".join(
            f"{name:<14}{value}" for name, value in cfg.LIGHT_STATES.items()))
        print("\n  (8 is undefined upstream -- the solid colours are 0-7 and the\n"
              "  flashing ones 9-15.)")
        return 0

    rclpy.init()
    node = Caller()
    try:
        if args.cmd == "text":
            req = SetString.Request()
            req.data = args.message
            return report(node.call(SetString, cfg.SRV_SET_TEXT, req))

        if args.cmd == "light":
            if args.colour not in cfg.LIGHT_STATES:
                print(f"  unknown colour {args.colour!r}. "
                      "Try: ./base_ctl.py light list", file=sys.stderr)
                return 2
            req = SetLightState.Request()
            req.light_state = cfg.LIGHT_STATES[args.colour]
            return report(node.call(SetLightState, cfg.SRV_SET_LIGHT, req))

        if args.cmd == "torque":
            if args.state == "off":
                print("  Releasing the drive motors -- the base will roll freely.\n"
                      "  Make sure it is on level ground and chocked if not.")
            req = SetBool.Request()
            req.data = (args.state == "on")
            return report(*call_retrying(node, SetBool, cfg.SRV_MOTOR_TORQUE,
                                         req, args.retries))

        if args.cmd == "charge":
            req = SetBool.Request()
            req.data = (args.state == "on")
            return report(*call_retrying(node, SetBool, cfg.SRV_ENABLE_CHARGING,
                                         req, args.retries))

        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
