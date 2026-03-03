"""
Deeper SPARK MAX USB probe:
1. Try vendor control transfers to see if the device acknowledges any
2. Try 32-byte (full packet size) frames instead of 12-byte frames
3. Try firmware version query which should always be answered
"""
import struct, sys, time
import usb.core, usb.util

dev = usb.core.find(idVendor=0x0483, idProduct=0xA30E)
if dev is None:
    sys.exit("Device not found")

print(f"Device: {dev.manufacturer} {dev.product} (fw s/n {dev.serial_number})")

cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]
ep_in  = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
print(f"ep_in={hex(ep_in.bEndpointAddress)} ep_out={hex(ep_out.bEndpointAddress)} pktsize={ep_in.wMaxPacketSize}\n")

# ── 1. Try control transfers ─────────────────────────────────────────────────
# bmRequestType=0x40 = vendor, device, host-to-device
# bmRequestType=0xC0 = vendor, device, device-to-host
print("=== Control transfer probes ===")
for bReq in [0x00, 0x01, 0x02, 0x04, 0x05, 0x10, 0x20, 0x30, 0xFF]:
    try:
        ret = dev.ctrl_transfer(
            bmRequestType=0xC0,   # vendor, device→host
            bRequest=bReq,
            wValue=0, wIndex=0,
            data_or_wLength=32,
            timeout=200
        )
        print(f"  ctrl bRequest=0x{bReq:02x} -> {len(ret)} bytes: {bytes(ret).hex()}")
    except usb.core.USBError as e:
        print(f"  ctrl bRequest=0x{bReq:02x} ->  err: {e}")

# ── 2. Try bulk writes with full 32-byte frames ──────────────────────────────
print("\n=== Bulk writes: full 32-byte packets with CAN frames ===")
def make_id(devtype, mfr, cls, idx, devid):
    return (devtype << 24) | (mfr << 16) | (cls << 10) | (idx << 6) | devid

# API commands to try (device type=2, mfr=5, device ID=1)
cmds = {
    "fw version query":     make_id(2, 5, 0x01, 0x06, 1),
    "fw version query2":    make_id(2, 5, 0x01, 0x00, 1),
    "get device id":        make_id(2, 5, 0x16, 0x00, 1),
    "heartbeat cls5 idx8":  make_id(2, 5, 5, 8, 1),
    "heartbeat cls5 idx0":  make_id(2, 5, 5, 0, 1),
    "status0 request":      make_id(2, 5, 0x06, 0x00, 1),
    "status1 request":      make_id(2, 5, 0x06, 0x01, 1),
    "enumerate":            make_id(2, 5, 0x00, 0x00, 0),
}

def try_write_read(label, can_id, payload=b"\x00"*8):
    # Try both 12-byte and 32-byte (padded) frames
    for sz, name in [(12, "12b"), (32, "32b-padded")]:
        frame = (struct.pack("<I", can_id) + payload).ljust(sz, b"\x00")
        try:
            n = dev.write(ep_out.bEndpointAddress, frame, timeout=200)
        except usb.core.USBError as e:
            print(f"  [{label:30s}] {name} write err: {e}")
            continue
        time.sleep(0.05)
        try:
            data = bytes(dev.read(ep_in.bEndpointAddress, 512, timeout=150))
            print(f"  [{label:30s}] {name} -> RX {len(data)}b: {data.hex()}")
        except usb.core.USBTimeoutError:
            print(f"  [{label:30s}] {name} -> (no response)")
        except usb.core.USBError as e:
            print(f"  [{label:30s}] {name} -> USB err: {e}")

for label, can_id in cmds.items():
    try_write_read(label, can_id)

# ── 3. Listen for spontaneous packets with no writes ─────────────────────────
print("\n=== Passive listen 5s (no writes) ===")
deadline = time.time() + 5
pkt = 0
while time.time() < deadline:
    try:
        data = bytes(dev.read(ep_in.bEndpointAddress, 512, timeout=200))
        print(f"  RX {len(data)}b: {data.hex()}")
        pkt += 1
    except usb.core.USBTimeoutError:
        pass
    except usb.core.USBError as e:
        print(f"  USB err: {e}"); break
print(f"  {pkt} spontaneous packets")
