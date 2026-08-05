#!/usr/bin/env python3
"""
Move one WXAI arm off the factory IP so two arms can share the switch.

Both arms ship on 192.168.1.2. Two arms answering to one address is not a clean
failure -- the driver talks to whichever replies first, so both containers end
up driving the same physical arm while the other sits idle. Re-address one arm
before they are ever on the switch together.

PLUG IN ONLY THE ARM YOU ARE RE-ADDRESSING. With both connected there is no way
to say which one this script reaches.

Procedure:
  1. Plug in arm A only. Run:  ./set-arm-ip.py 192.168.1.3
  2. Power-cycle arm A -- the controller reads the new address at boot.
  3. Plug arm B in as well. Run ./discover-arms.py; expect .2 and .3.

The setting lives in the controller's own configuration, so it survives power
cycles and does not need re-running. To undo, run this against the arm again
with 192.168.1.2, or use set_factory_reset_flag.

Usage:
    ./set-arm-ip.py <new_ip> [current_ip]     # current_ip defaults to 192.168.1.2
"""

import sys

import trossen_arm


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__.strip())
        return 2

    new_ip = sys.argv[1]
    current_ip = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.2"

    found = trossen_arm.TrossenArmDriver.discover(subnet="192.168.1", timeout=0.05)
    if len(found) > 1:
        print(f"REFUSING: {len(found)} arms are on the subnet: {[r.ip for r in found]}")
        print("Unplug all but the one you are re-addressing, then re-run.")
        return 1
    if not found:
        print("no arm responded -- check power, cabling, and the host NIC address")
        return 1

    print(f"found one arm at {found[0].ip} (model={found[0].model})")

    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        model=trossen_arm.Model.wxai_v0,
        end_effector=trossen_arm.StandardEndEffector.wxai_v0_base,
        serv_ip=current_ip,
        clear_error=True,
    )

    # Order matters: manual mode first, then the address. Writing the address
    # while the controller is still in DHCP mode stores a value it will not use.
    driver.set_ip_method(trossen_arm.IPMethod.manual)
    driver.set_manual_ip(new_ip)

    print(f"wrote manual IP {new_ip} (was {current_ip})")
    print("POWER-CYCLE THE ARM NOW -- the new address takes effect at boot.")
    print(f"Then verify with:  ./discover-arms.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
