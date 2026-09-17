"""Give the camera-yaw motor the identity the driver expects.

It left the factory as ID 1 at 57600 baud and was never configured, so it sits
on the DXL chain invisible to a driver that scans at 1 Mbps. Two EEPROM writes
fix that: ID 1 -> 9 (the slot giava's wx250s_7dof config calls camera_yaw) and
baud 57600 -> 1 Mbps. Nothing moves; torque stays off throughout.

Reversible: run it again with --undo to put the motor back to ID 1 / 57600.
"""
import sys, time
from dynamixel_sdk import PortHandler, PacketHandler

UNDO = "--undo" in sys.argv
ADDR_ID, ADDR_BAUD, ADDR_TORQUE = 7, 8, 64
XL430 = 1060

port = PortHandler("/dev/ttyDXL"); pk = PacketHandler(2.0)
assert port.openPort(), "cannot open /dev/ttyDXL"

src_baud, src_id = (1000000, 9) if UNDO else (57600, 1)
dst_id, dst_code, dst_baud = (1, 1, 57600) if UNDO else (9, 3, 1000000)

port.setBaudRate(src_baud)
model, res, _ = pk.ping(port, src_id)
print(f"found at {src_baud} baud, ID {src_id}: model {model} (res {res})")
if res != 0 or model != XL430:
    print(f"  expected an XL430 (model {XL430}) -- refusing to write"); sys.exit(2)

t, _, _ = pk.read1ByteTxRx(port, src_id, ADDR_TORQUE)
if t:
    pk.write1ByteTxRx(port, src_id, ADDR_TORQUE, 0); time.sleep(0.1)   # EEPROM needs torque off

r, e = pk.write1ByteTxRx(port, src_id, ADDR_ID, dst_id)
print(f"  ID {src_id} -> {dst_id}: {'ok' if r == 0 and e == 0 else f'res={r} err={e}'}")
time.sleep(0.3)
r, e = pk.write1ByteTxRx(port, dst_id, ADDR_BAUD, dst_code)
print(f"  baud -> {dst_baud}: {'ok' if r == 0 and e == 0 else f'res={r} err={e}'}")
time.sleep(0.3)

port.setBaudRate(dst_baud)
model, res, _ = pk.ping(port, dst_id)
print(f"verify at {dst_baud} baud, ID {dst_id}: model {model} (res {res})")
found = [i for i in range(1, 12) if pk.ping(port, i)[1] == 0]
print(f"bus at {dst_baud}: IDs {found}")
port.closePort()
sys.exit(0 if res == 0 else 1)
