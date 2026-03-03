"""
Minimal OpenCV GUI for SPARK MAX over Arduino CAN bridge.

Requirements:
- Arduino running Arduino/CAN_SparkMAX_Bridge/CAN_SparkMAX_Bridge.ino
- pyserial, opencv-python, numpy

Serial line protocol expected from bridge:
  RX 0x<id_hex> <dlc> <data_hex>
  ACK ...

Keyboard:
  SPACE / s : stop (0 V)
  q / ESC   : quit
"""

from __future__ import annotations

import argparse
import collections
import queue
import struct
import threading
import time
from collections import Counter
from dataclasses import dataclass
from typing import Deque, Optional

import cv2
import numpy as np
import serial


def make_spark_id(device_type: int, mfr: int, api_class: int, api_index: int, device_id: int) -> int:
    return (
        ((device_type & 0x1F) << 24)
        | ((mfr & 0xFF) << 16)
        | ((api_class & 0x3F) << 10)
        | ((api_index & 0x0F) << 6)
        | (device_id & 0x3F)
    )


def split_spark_id(can_id: int) -> tuple[int, int, int, int, int]:
    device_type = (can_id >> 24) & 0x1F
    mfr = (can_id >> 16) & 0xFF
    api_class = (can_id >> 10) & 0x3F
    api_index = (can_id >> 6) & 0x0F
    device_id = can_id & 0x3F
    return device_type, mfr, api_class, api_index, device_id


@dataclass
class SparkTelemetry:
    applied_duty: float = 0.0
    applied_voltage: float = 0.0
    bus_voltage: float = 0.0
    current_a: float = 0.0
    last_frame_id: int = 0
    last_update_t: float = 0.0


class BridgeClient:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        time.sleep(2.0)

        self.rx_queue: queue.Queue[str] = queue.Queue()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._reader_thread, daemon=True)
        self._thread.start()

    def _reader_thread(self):
        while not self._stop_evt.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            if line:
                self.rx_queue.put(line)

    def send_line(self, line: str):
        self.ser.write((line.strip() + "\n").encode("utf-8"))

    def close(self):
        self._stop_evt.set()
        try:
            self._thread.join(timeout=0.3)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


class SparkCanGui:
    def __init__(self, port: str, can_id: int, status_period_ms: int):
        self.bridge = BridgeClient(port)
        self.can_id = can_id
        self.status_period_ms = max(5, min(1000, status_period_ms))

        self.window = "SPARK MAX CAN Utility"
        self.slider_name = "Voltage cmd [-12..12V]"
        self.slider_max = 240
        self.telemetry = SparkTelemetry()

        self.target_voltage = 0.0
        self.last_sent_voltage = None
        self.last_send_t = 0.0
        self.send_period_s = 0.05

        self.log_lines: Deque[str] = collections.deque(maxlen=22)
        self.rx_total = 0
        self.rx_for_device = 0
        self.rx_status_frames = 0
        self.api10_counts: Counter[int] = Counter()
        self.api10_last_data: dict[int, bytes] = {}

        # Commands we transmit (can appear if adapter echoes own TX on CAN RX)
        self.tx_api10_ids = {0x22, 0x58, 0x60, 0x61}

    def setup(self):
        self.bridge.send_line(f"HB {self.can_id} 1")
        self.bridge.send_line(f"RATE {self.can_id} 0 {self.status_period_ms}")
        self.bridge.send_line(f"RATE {self.can_id} 1 {self.status_period_ms}")
        # Some firmware stacks only answer after explicit request-style frames.
        req0 = make_spark_id(2, 5, 0x06, 0x00, self.can_id)
        req1 = make_spark_id(2, 5, 0x06, 0x01, self.can_id)
        self.bridge.send_line(f"TX 0x{req0:08X} 8 0000000000000000")
        self.bridge.send_line(f"TX 0x{req1:08X} 8 0000000000000000")
        self.bridge.send_line(f"VOLTS {self.can_id} 0")

        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 900, 700)
        cv2.createTrackbar(self.slider_name, self.window, 120, self.slider_max, self._on_slider)

        self.log("Started bridge session")
        self.log(f"CAN ID = {self.can_id}")

    def _on_slider(self, value: int):
        self.target_voltage = (float(value) / 10.0) - 12.0

    def log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self.log_lines.appendleft(f"[{ts}] {text}")

    def parse_rx_line(self, line: str):
        # Expected: RX 0x2051841 8 0011223344556677
        parts = line.split()
        if len(parts) < 4 or parts[0] != "RX":
            return

        try:
            can_id = int(parts[1], 16)
            dlc = int(parts[2])
            data = bytes.fromhex(parts[3])
        except Exception:
            return

        if dlc > len(data):
            dlc = len(data)
        data = data[:dlc]

        self.rx_total += 1
        self.decode_frame(can_id, data)

    def decode_frame(self, can_id: int, data: bytes):
        dev_type, mfr, api_class, api_idx, dev_id = split_spark_id(can_id)
        if dev_type != 2 or mfr != 5 or dev_id != self.can_id:
            return

        self.rx_for_device += 1
        api10 = (can_id >> 6) & 0x3FF
        self.api10_counts[api10] += 1
        self.api10_last_data[api10] = data

        now = time.time()
        self.telemetry.last_frame_id = can_id
        self.telemetry.last_update_t = now

        # Status IDs are represented as a 10-bit API field in the FRC CAN ID.
        status0 = api10 == 0x60
        status1 = api10 == 0x61

        if status0 and len(data) >= 6:
            self.rx_status_frames += 1
            applied_raw = struct.unpack_from("<h", data, 0)[0]
            self.telemetry.applied_duty = float(applied_raw) / 32767.0

            # Common REV scaling observed in FRC APIs:
            # voltage: uint16 / 128, current: uint16 / 32
            bus_raw = struct.unpack_from("<H", data, 2)[0]
            cur_raw = struct.unpack_from("<H", data, 4)[0]

            self.telemetry.bus_voltage = bus_raw / 128.0
            self.telemetry.current_a = cur_raw / 32.0

            if self.telemetry.bus_voltage > 0.1:
                self.telemetry.applied_voltage = self.telemetry.applied_duty * self.telemetry.bus_voltage
            else:
                self.telemetry.applied_voltage = self.telemetry.applied_duty * 12.0

        elif status1:
            self.rx_status_frames += 1
            # Keep frame visible in log for debugging, but no extra decode needed for MVP
            pass

        # Fallback decode: if status IDs differ on this firmware, decode any non-command
        # API frame that contains plausible applied/bus/current packed values.
        elif len(data) >= 6 and api10 not in self.tx_api10_ids:
            applied_raw = struct.unpack_from("<h", data, 0)[0]
            bus_raw = struct.unpack_from("<H", data, 2)[0]
            cur_raw = struct.unpack_from("<H", data, 4)[0]

            bus_v = bus_raw / 128.0
            current_a = cur_raw / 32.0

            # Plausibility gate to avoid junk decode.
            if 1.0 <= bus_v <= 40.0 and 0.0 <= current_a <= 500.0:
                self.rx_status_frames += 1
                self.telemetry.applied_duty = float(applied_raw) / 32767.0
                self.telemetry.bus_voltage = bus_v
                self.telemetry.current_a = current_a
                self.telemetry.applied_voltage = self.telemetry.applied_duty * self.telemetry.bus_voltage

    def send_voltage_if_needed(self, force: bool = False):
        now = time.time()
        should_send = force

        if self.last_sent_voltage is None:
            should_send = True
        elif abs(self.target_voltage - self.last_sent_voltage) >= 0.02:
            should_send = True
        elif (now - self.last_send_t) >= self.send_period_s:
            should_send = True

        if should_send:
            self.bridge.send_line(f"VOLTS {self.can_id} {self.target_voltage:.3f}")
            self.last_sent_voltage = self.target_voltage
            self.last_send_t = now

    def stop_motor(self):
        self.target_voltage = 0.0
        cv2.setTrackbarPos(self.slider_name, self.window, 120)
        self.send_voltage_if_needed(force=True)
        self.log("STOP command")

    def draw(self) -> np.ndarray:
        canvas = np.zeros((700, 900, 3), dtype=np.uint8)

        cv2.putText(canvas, "SPARK MAX CAN Utility", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2)
        cv2.putText(canvas, f"Target Voltage: {self.target_voltage:+.2f} V", (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        age = time.time() - self.telemetry.last_update_t if self.telemetry.last_update_t > 0 else 999
        stale = age > 0.5
        status_color = (0, 200, 0) if not stale else (0, 140, 255)
        status_text = "LIVE" if not stale else "WAITING FOR STATUS"

        cv2.putText(canvas, f"Status: {status_text}", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(canvas, f"Applied Voltage: {self.telemetry.applied_voltage:+.2f} V", (20, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, f"Bus Voltage:     {self.telemetry.bus_voltage:6.2f} V", (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, f"Output Current:  {self.telemetry.current_a:6.2f} A", (20, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, f"Applied Duty:    {self.telemetry.applied_duty:+.3f}", (20, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, f"RX total/device/status: {self.rx_total}/{self.rx_for_device}/{self.rx_status_frames}", (20, 312), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (180, 180, 180), 1)

        top = self.api10_counts.most_common(3)
        if top:
            top_text = " ".join([f"0x{k:03X}:{v}" for k, v in top])
            cv2.putText(canvas, f"Top API10: {top_text}", (420, 312), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

        cv2.rectangle(canvas, (20, 330), (880, 680), (80, 80, 80), 1)
        cv2.putText(canvas, "Serial/CAN log", (30, 355), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

        y = 382
        for line in list(self.log_lines)[:18]:
            cv2.putText(canvas, line[:110], (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (170, 170, 170), 1)
            y += 17

        if self.telemetry.last_frame_id:
            cv2.putText(canvas, f"Last frame: 0x{self.telemetry.last_frame_id:08X}", (560, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

        cv2.putText(canvas, "SPACE/s: STOP   q/ESC: quit", (20, 690), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 200, 255), 1)
        return canvas

    def run(self):
        self.setup()
        try:
            while True:
                while not self.bridge.rx_queue.empty():
                    line = self.bridge.rx_queue.get_nowait()
                    if line.startswith("RX "):
                        self.parse_rx_line(line)
                    else:
                        self.log(line)

                self.send_voltage_if_needed()

                frame = self.draw()
                cv2.imshow(self.window, frame)

                key = cv2.waitKey(10) & 0xFF
                if key in (27, ord("q")):
                    break
                if key in (32, ord("s")):
                    self.stop_motor()

        finally:
            try:
                self.bridge.send_line(f"STOP {self.can_id}")
                self.bridge.send_line(f"HB {self.can_id} 0")
            except Exception:
                pass
            self.bridge.close()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="SPARK MAX CAN OpenCV utility")
    parser.add_argument("--port", default="/dev/cu.usbserial-10", help="Arduino serial port")
    parser.add_argument("--can-id", type=int, default=1, help="SPARK MAX CAN device ID")
    parser.add_argument("--status-period-ms", type=int, default=20, help="Requested status frame period")
    args = parser.parse_args()

    gui = SparkCanGui(port=args.port, can_id=args.can_id, status_period_ms=args.status_period_ms)
    gui.run()


if __name__ == "__main__":
    main()
