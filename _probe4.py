"""
SPARK MAX USB — step 4:
Try the accepted host→device control writes (0x00,0x01,0x02,0x07,0x09)
as potential "enable" / "connect" / "start stream" commands,
then watch the bulk IN endpoint for any traffic.
Also: try correct SPARK MAX heartbeat after each one.
"""
import struct, sys, time
import usb.core, usb.util

dev = usb.core.find(idVendor=0x0483, idProduct=0xA30E)
if dev is None:
    sys.exit("Device not found")
cfg  = dev.get_active_configuration()
intf = cfg[(0, 0)]
ep_in  = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)

def ctrl_write(bReq, data=b"", wValue=0, wIndex=0):
    return dev.ctrl_transfer(0x40, bReq, wValue, wIndex, data, timeout=300)

def ctrl_read(bReq, wValue=0, wIndex=0, length=64):
    return bytes(dev.ctrl_transfer(0xC0, bReq, wValue, wIndex, length, timeout=300))

def clear_halt():
    try: dev.ctrl_transfer(0x02, 0x01, 0, ep_out.bEndpointAddress, timeout=300)
    except: pass
    try: dev.ctrl_transfer(0x02, 0x01, 0, ep_in.bEndpointAddress, timeout=300)
    except: pass

def drain_in(duration=1.0):
    """Read bulk IN for `duration` seconds, return all packets."""
    pkts = []
    deadline = time.time() + duration
    while time.time() < deadline:
        try:
            data = bytes(dev.read(ep_in.bEndpointAddress, 512, timeout=100))
            pkts.append(data)
        except usb.core.USBTimeoutError:
            pass
        except usb.core.USBError as e:
            print(f"    IN err: {e}")
            break
    return pkts

# ──────────────────────────────────────────────────────────────────────────────
print("Device firmware:", end=" ")
fw = ctrl_read(0x05)
print(f"v{fw[4]}.{fw[5]}.{fw[6]} build {fw[7]}")

# SPARK MAX CAN ID helpers (v25 uses same bit layout as before)
def make_id(devtype, mfr, cls, idx, devid):
    return (devtype << 24) | (mfr << 16) | (cls << 10) | (idx << 6) | devid

# Heartbeat (FRC-style): device type=2, mfr=5, cls=5, idx=8, dev=1
HB_ID    = make_id(2, 5, 5, 8, 1)
HB_FRAME = struct.pack("<I", HB_ID) + b"\x00" * 8   # 12 bytes

print(f"Heartbeat CAN ID: 0x{HB_ID:08x}\n")

# ── Try each control write code as "session opener", then watch bulk IN ───────
for enable_req in [0x01, 0x07, 0x02, 0x09, 0x00]:
    clear_halt()
    print(f"--- ctrl_write(0x{enable_req:02x}) then watching bulk IN ---")
    try:
        ctrl_write(enable_req, b"\x00"*8)
    except usb.core.USBError as e:
        print(f"  ctrl_write err: {e}")
    # Send a couple of heartbeats
    for _ in range(5):
        try: dev.write(ep_out.bEndpointAddress, HB_FRAME, timeout=100)
        except: pass
        time.sleep(0.011)
    pkts = drain_in(0.5)
    if pkts:
        for p in pkts:
            print(f"  RX {len(p)}b: {p.hex()}")
    else:
        print(f"  (no data)")

# ── Try requesting specific status frames by sending CAN queries ──────────────
print("\n--- Send status-frame-rate commands + watch bulk ---")
clear_halt()
# Status frame period set: API class=0x06, idx=varies
# Format: 4 bytes CAN ID + 2 bytes period_ms (LE uint16) + 6 zeros
status_apis = {
    "Status0 rate": make_id(2, 5, 0x06, 0x00, 1),
    "Status1 rate": make_id(2, 5, 0x06, 0x01, 1),
    "Status2 rate": make_id(2, 5, 0x06, 0x02, 1),
    "Status0 req":  make_id(2, 5, 0x60, 0x00, 1),
    "Status1 req":  make_id(2, 5, 0x61, 0x00, 1),
}
for label, can_id in status_apis.items():
    clear_halt()
    frame = struct.pack("<IH", can_id, 10) + b"\x00"*6  # 12 bytes: ID + 10ms period
    try:
        dev.write(ep_out.bEndpointAddress, frame, timeout=200)
        print(f"  Sent [{label}] CAN 0x{can_id:08x}", end="")
    except usb.core.USBError as e:
        print(f"  Sent [{label}] FAIL: {e}", end="")
    pkts = drain_in(0.3)
    if pkts:
        for p in pkts: print(f"\n    RX {len(p)}b: {p.hex()}")
    else:
        print("  -> (no response)")

# ── Passive listen for 5s after all the prods ─────────────────────────────────
print("\n--- Passive bulk IN listen 5s ---")
pkts = drain_in(5.0)
print(f"  {len(pkts)} packets")
for p in pkts[:20]:
    print(f"  {len(p)}b: {p.hex()}")
