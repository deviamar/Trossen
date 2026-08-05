#!/usr/bin/env python3
"""
List every WXAI arm controller answering on the arm subnet.

Run this from inside either manip-arm container after wiring the switch. It is
the fastest way to tell the three failure modes apart:

  0 arms found   -> host NIC is down / has no 192.168.1.x address, wrong switch
                    port, or the arms are not powered
  1 arm found    -> both arms are still on the factory IP (192.168.1.2) and the
                    switch is forwarding only one of them, OR only one arm is
                    actually plugged in. Unplug one and re-run to tell which.
  2 arms found   -> wiring is good; the IPs printed here are what belong in
                    ARM_1_IP / ARM_2_IP.

Usage:
    ./discover-arms.py                # scans 192.168.1.1 .. 192.168.1.254
    ./discover-arms.py 192.168.1 2 10 # subnet prefix, first octet, last octet
"""

import sys

import trossen_arm


def main() -> int:
    subnet = sys.argv[1] if len(sys.argv) > 1 else "192.168.1"
    ip_start = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    ip_end = int(sys.argv[3]) if len(sys.argv) > 3 else 254

    print(f"scanning {subnet}.{ip_start} .. {subnet}.{ip_end} ...")
    # The default 10 ms per-host timeout is tuned for a local switch; a busy or
    # long cable run can need more, and a false "0 arms found" is the symptom.
    results = trossen_arm.TrossenArmDriver.discover(
        subnet=subnet, ip_start=ip_start, ip_end=ip_end, timeout=0.05
    )

    if not results:
        print("no arms responded -- see the failure modes in this file's docstring")
        return 1

    for r in results:
        print(f"  {r.ip}  model={r.model}  fw={r.firmware_version}  error={r.error_state}")

    if len(results) > 1 and len({r.ip for r in results}) == 1:
        print("\nWARNING: multiple replies from one IP -- two arms share an address.")
        print("Unplug one and re-address it with ./set-arm-ip.py")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
