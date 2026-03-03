"""
SPARK MAX live monitor using robotpy-rev.

Usage:
    python3 serial_monitor.py [--can-id N] [--brushed]

Keys:
    UP / w    : increase setpoint (+0.05)
    DOWN / s  : decrease setpoint (-0.05)
    0 / SPACE : stop (set to 0)
    r         : reset encoder position to 0
    q / ESC   : quit
"""

import argparse
import sys
import time

import hal
import rev

# Must initialize HAL before creating any SPARK MAX object.
# Without this, robotpy-rev silently runs in simulation mode
# (returns default values and ignores motor.set() calls).
hal.initialize(hal.RuntimeType.HAL_Runtime_RoboRIO2, 0)

parser = argparse.ArgumentParser(description="SPARK MAX live monitor")
parser.add_argument("--can-id", type=int, default=1)
parser.add_argument("--brushed", action="store_true")
args = parser.parse_args()

motor_type = rev.SparkMax.MotorType.kBrushed if args.brushed else rev.SparkMax.MotorType.kBrushless

print(f"Connecting to SPARK MAX CAN ID {args.can_id} ({motor_type.name})...")
motor   = rev.SparkMax(args.can_id, motor_type)
encoder = motor.getEncoder()
time.sleep(0.2)

print(f"Firmware : {motor.getFirmwareString()}")
sn = motor.getSerialNumber()
print(f"Serial # : {sn if sn else chr(40)+chr(117)+chr(110)+chr(97)+chr(118)+chr(97)+chr(105)+chr(108)+chr(97)+chr(98)+chr(108)+chr(101)+chr(41)}")
print(f"Device ID: {motor.getDeviceId()}")
print()

import queue, threading

_key_queue = queue.SimpleQueue()

def _key_reader():
    """Background thread: read stdin in raw mode and push decoded key tokens."""
    try:
        import tty, termios
    except ImportError:
        return
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.buffer.read(1)
            if not ch:
                break
            # Arrow keys arrive as ESC [ A/B/C/D — read the rest immediately
            if ch == b'\x1b':
                next1 = sys.stdin.buffer.read(1)
                if next1 == b'[':
                    next2 = sys.stdin.buffer.read(1)
                    mapping = {b'A': 'UP', b'B': 'DOWN', b'C': 'RIGHT', b'D': 'LEFT'}
                    _key_queue.put(mapping.get(next2, 'ESC'))
                else:
                    _key_queue.put('ESC')
            else:
                _key_queue.put(ch.decode('utf-8', errors='replace'))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

_reader_thread = threading.Thread(target=_key_reader, daemon=True)
_reader_thread.start()

def _raw_print(msg):
    """Print a line correctly while the terminal is in raw mode (\n → \r\n)."""
    sys.stdout.write(msg.replace('\n', '\r\n'))
    sys.stdout.flush()

def _get_keys():
    """Drain all pending key tokens from the queue."""
    keys = []
    while True:
        try:
            keys.append(_key_queue.get_nowait())
        except queue.Empty:
            break
    return keys

def faults_active(f):
    names = ["brownout","motorFault","sensorFault","stall",
             "EEPROM","CANTx","CANRx","hasReset","DRVFault",
             "softLimitFwd","softLimitRev","hardLimitFwd","hardLimitRev"]
    active = [n for n in names if getattr(f, n, False)]
    return active if active else ["none"]

STEP     = 0.05
setpoint = 0.0
loop_hz  = 20

_raw_print("Controls:  UP/w +speed   DOWN/s -speed   0/SPACE stop   r reset-pos   q quit\n")
_raw_print("-" * 72 + "\n")
motor.set(0.0)

try:
    while True:
        t0 = time.time()
        for key in _get_keys():
            if   key == 'UP'   or key == 'w': setpoint = min( 1.0, setpoint + STEP)
            elif key == 'DOWN' or key == 's': setpoint = max(-1.0, setpoint - STEP)
            elif key in ('0', ' '):           setpoint = 0.0
            elif key == 'r':                  encoder.setPosition(0.0)
            elif key in ('q', 'Q', 'ESC'):    raise SystemExit

        motor.set(setpoint)
        applied = motor.getAppliedOutput()
        voltage = motor.getBusVoltage()
        current = motor.getOutputCurrent()
        temp    = motor.getMotorTemperature()
        pos     = encoder.getPosition()
        vel     = encoder.getVelocity()
        err     = motor.getLastError()
        faults  = faults_active(motor.getFaults())

        BAR = 28
        mid = max(0, min(BAR, int((applied + 1.0) / 2.0 * BAR)))
        bar = "[" + "-" * mid + "#" + "-" * (BAR - mid) + "]"

        line = (
            f"  cmd={setpoint:+.2f}  out={applied:+.4f} {bar}  "
            f"vel={vel:8.1f}rpm  pos={pos:8.3f}rot  "
            f"{voltage:.2f}V {current:.2f}A {temp:.0f}C  "
            f"err={err.name}  faults={chr(44).join(faults)}    "
        )
        print("\r" + line, end="", flush=True)

        elapsed = time.time() - t0
        time.sleep(max(0, 1.0 / loop_hz - elapsed))

except (KeyboardInterrupt, SystemExit):
    pass
finally:
    motor.set(0.0)
    motor.stopMotor()
    sys.stdout.write("\r\n\r\nMotor stopped.\r\n")
    sys.stdout.flush()
