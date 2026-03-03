"""
Motor controller module.
Sends speed commands to the Arduino over serial.
Protocol: 2 bytes big-endian (uint16), range 0–65535, 32767 = stopped.
Maps on the Arduino side to 1000–2000 µs ESC pulse width.
"""

import serial
import struct
import time


class MotorController:
    # 16-bit protocol constants
    NEUTRAL = 32767
    MAX_VAL = 65535
    MIN_VAL = 0
    # Scale factor: multiply old 0-255 style offsets by this to get 16-bit equivalents
    UNIT = MAX_VAL / 255  # ≈ 257

    def __init__(self, port='/dev/cu.usbserial-10', baudrate=115200):
        """
        Initialize serial connection to Arduino motor controller.
        
        Args:
            port: Serial port path.
            baudrate: Communication speed.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        self.current_speed = self.NEUTRAL  # Neutral

    def connect(self):
        """Open serial connection."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate)
            time.sleep(2)  # Wait for Arduino reset
            self.connected = True
            print(f"Motor controller connected on {self.port}")
            self.set_speed(self.NEUTRAL)  # Start at neutral
        except serial.SerialException as e:
            print(f"Could not open serial port {self.port}: {e}")
            self.connected = False

    def set_speed(self, value):
        """
        Set motor speed.
        
        Args:
            value: 0-65535 (0=max negative, 32767=stop, 65535=max positive)
        """
        value = max(self.MIN_VAL, min(self.MAX_VAL, int(value)))
        if self.connected and self.ser:
            self.ser.write(struct.pack('>H', value))  # Big-endian uint16
            self.current_speed = value

    def set_speed_relative(self, offset):
        """
        Set motor speed as offset from neutral (32767).
        
        Args:
            offset: Raw 16-bit offset. Clamped to valid range.
        """
        self.set_speed(self.NEUTRAL + offset)

    def stop(self):
        """Stop the motor (set to neutral)."""
        self.set_speed(self.NEUTRAL)

    def disconnect(self):
        """Stop motor and close serial connection."""
        if self.connected:
            self.stop()
            time.sleep(0.1)
            self.ser.close()
            self.connected = False
            print("Motor controller disconnected.")

    def __del__(self):
        self.disconnect()
