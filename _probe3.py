"""
SPARK MAX USB — step 3:
1. Scan all vendor control request codes 0x00–0x3F to find every working one
2. Decode 0x04 and 0x05 responses
3. Reset/clear the stalled OUT endpoint and try different frame sizes/formats
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

# ── helpers ───────────────────────────────────────────────────────────────────
def ctrl_read(bReq, wValue=0, wIndex=0, length=64):
    return dev.ctrl_transfer(0xC0, bReq, wValue, wIndex, length, timeout=300)

def ctrl_write(bReq, data=b"", wValue=0, wIndex=0):
    return dev.ctrl_transfer(0x40, bReq, wValue, wIndex, data, timeout=300)

def clear_ep_halt(ep_addr):
    """Clear STALL/HALT on a bulk endpoint."""
    dev.ctrl_transfer(0x02, 0x01, 0, ep_addr, timeout=300)   # CLEAR_FEATURE(ENDPOINT_HALT)

def bulk_write(data, timeout=200):
    return dev.write(ep_out.bEndpointAddress, data, timeout=timeout)

def bulk_read(timeout=300):
    return bytes(dev.read(ep_in.bEndpointAddress, 512, timeout=timeout))

# ── 1. Full control-request scan ──────────────────────────────────────────────
print("=== Control request scan (device→host, 0x00–0x3F) ===")
working = {}
for bReq in range(0x00, 0x40):
    try:
        data = bytes(ctrl_read(bReq, length=64))
        working[bReq] = data
        print(f"  0x{bReq:02x}  OK  {len(data)} bytes:  {data.hex()}")
    except usb.core.USBError:
        pass   # timeout or STALL = not supported

print(f"\n{len(working)} working request codes found: {[hex(k) for k in working]}\n")

# ── 2. Decode known responses ─────────────────────────────────────────────────
if 0x04 in working:
    d = working[0x04]
    print("=== 0x04 decoded ===")
    for i in range(0, min(len(d), 32), 4):
        val = struct.unpack_from("<I", d, i)[0]
        print(f"  [{i:2d}] uint32LE = {val:10d}  0x{val:08x}")

if 0x05 in working:
    d = working[0x05]
    print("\n=== 0x05 decoded ===")
    # Try to read as firmware version: major.minor.fix
    major = d[4] if len(d) > 4 else 0
    minor = d[5] if len(d) > 5 else 0
    fix   = d[6] if len(d) > 6 else 0
    build = struct.unpack_from("<H", d, 6)[0] if len(d) > 7 else 0
    print(f"  Firmware (guess bytes 4,5,6,7): v{major}.{minor}.{fix} build={d[7] if len(d)>7 else '?'}")
    for i in range(0, min(len(d), 32), 4):
        val = struct.unpack_from("<I", d, i)[0]
        print(f"  [{i:2d}] uint32LE = {val:10d}  0x{val:08x}")

# ── 3. Try host→device control writes ────────────────────────────────────────
print("\n=== Control writes (host→device, scanning 0x00–0x3F) ===")
for bReq in range(0x00, 0x40):
    try:
        n = ctrl_write(bReq, b"\x00"*8)
        # Check if we get any spontaneous bulk data after the write
        try:
            pkt = bulk_read(timeout=100)
            print(f"  write 0x{bReq:02x}  OK  -> bulk RX {len(pkt)} bytes: {pkt.hex()}")
        except usb.core.USBTimeoutError:
            print(f"  write 0x{bReq:02x}  OK  -> (no bulk)")
        except usb.core.USBError as e:
            print(f"  write 0x{bReq:02x}  OK  -> bulk err: {e}")
    except usb.core.USBError:
        pass

# ── 4. Try clearing OUT endpoint HALT and re-sending with different sizes ─────
print("\n=== Clear OUT HALT and retry bulk frames ===")
try:
    clear_ep_halt(ep_out.bEndpointAddress)
    print("  Cleared HALT on OUT endpoint")
except usb.core.USBError as e:
    print(f"  clear HALT err: {e}")

# Try frame sizes: 4, 8, 12, 16, 20, 24, 32 bytes (all zeros)
for sz in [4, 8, 12, 16, 20, 24, 32]:
    try:
        bulk_write(b"\x00" * sz)
        try:
            pkt = bulk_read(timeout=200)
            print(f"  {sz}b zero-frame  -> RX {len(pkt)} bytes: {pkt.hex()}")
        except usb.core.USBTimeoutError:
            print(f"  {sz}b zero-frame  -> (no response)")
    except usb.core.USBError as e:
        print(f"  {sz}b zero-frame  -> write err: {e}")
        try: clear_ep_halt(ep_out.bEndpointAddress)
        except: pass
