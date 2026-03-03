"""Probe the SPARK MAX to figure out what CAN frames solicit a response."""
import struct, sys, time
import usb.core, usb.util

dev = usb.core.find(idVendor=0x0483, idProduct=0xA30E)
if dev is None:
    sys.exit("Not found")
cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]
ep_in  = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
for i in range(cfg.bNumInterfaces):
    try:
        if dev.is_kernel_driver_active(i):
            dev.detach_kernel_driver(i)
    except Exception:
        pass
# Skip set_configuration if already correct (avoids USB reset on macOS)
try:
    if dev.get_active_configuration().bConfigurationValue != cfg.bConfigurationValue:
        dev.set_configuration()
except usb.core.USBError:
    dev.set_configuration()

# SPARK MAX CAN ID bit layout (REV docs / sparkmax-firmware):
# bits 28-24 (5): device type
# bits 23-16 (8): manufacturer
# bits 15-10 (6): API class
# bits  9- 6 (4): API index
# bits  5- 0 (6): device ID
def make_id(devtype, mfr, cls, idx, devid):
    return (devtype << 24) | (mfr << 16) | (cls << 10) | (idx << 6) | devid

# Heartbeat variants to try
variants = {
    "hb cls=5 idx=8 dev=1": make_id(2, 5, 5, 8, 1),
    "hb cls=5 idx=8 dev=0": make_id(2, 5, 5, 8, 0),
    "hb cls=5 idx=0 dev=1": make_id(2, 5, 5, 0, 1),
    "hb cls=0 idx=0 dev=1": make_id(2, 5, 0, 0, 1),
    "hb cls=6 idx=0 dev=1": make_id(2, 5, 6, 0, 1),
    "identify dev=1":        make_id(2, 5, 0x16, 0, 1),  # Identify cmd
    "get param dev=1":       make_id(2, 5, 0x1, 0x30, 1),
}

print("Sending probe frames and watching for any response...\n")
for name, can_id in variants.items():
    frame = struct.pack("<I", can_id) + b"\x00" * 8
    try:
        dev.write(ep_out.bEndpointAddress, frame, timeout=200)
        print(f"  Sent  [{name}]  CAN 0x{can_id:08x}", flush=True)
    except usb.core.USBError as e:
        print(f"  FAIL  [{name}]: {e}", flush=True)
    # Give the device a moment then try to read
    time.sleep(0.05)
    try:
        data = bytes(dev.read(ep_in.bEndpointAddress, 512, timeout=100))
        print(f"        -> RX {len(data)} bytes: {data.hex()}", flush=True)
    except usb.core.USBTimeoutError:
        print(f"        -> (no response)", flush=True)

print("\nNow listening 8s with continuous heartbeats (cls=5 idx=8 dev=1)...")
hb_id = make_id(2, 5, 5, 8, 1)
hb_frame = struct.pack("<I", hb_id) + b"\x00" * 8
deadline = time.time() + 8
last_hb = 0
got = 0
while time.time() < deadline:
    now = time.time()
    if now - last_hb >= 0.010:
        try: dev.write(ep_out.bEndpointAddress, hb_frame, timeout=50)
        except Exception: pass
        last_hb = now
    try:
        data = bytes(dev.read(ep_in.bEndpointAddress, 512, timeout=50))
        can_id = struct.unpack_from("<I", data)[0] if len(data) >= 4 else 0
        print(f"  RX {len(data)}b  CAN 0x{can_id:08x}  {data.hex()}", flush=True)
        got += 1
    except usb.core.USBTimeoutError:
        pass
    except usb.core.USBError as e:
        print(f"  USB err: {e}")
        break

print(f"\nTotal frames received: {got}")
