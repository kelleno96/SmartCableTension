import cv2
import numpy as np
import threading
import time
from collections import deque
import speech_recognition as sr
from threading import Lock
import math

class SmartCableSimulator:
    def __init__(self):
        self.height = 480
        self.width = 1280
        self.max_history = 1280

        # Simulation state
        self.position = 0.0
        self.velocity = 0.0
        self.torque_command = 0.0
        self.stop_flag = False

        # Signal history
        self.history = {
            "position": deque(maxlen=self.max_history),
            "velocity": deque(maxlen=self.max_history),
            "torque": deque(maxlen=self.max_history),
        }

        # Mutex for thread safety
        self.lock = Lock()

        # Start voice detection thread
        threading.Thread(target=self.voice_listener, daemon=True).start()

    def voice_listener(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        with mic as source:
            recognizer.adjust_for_ambient_noise(source)
            while True:
                try:
                    print("Listening for voice command...")
                    audio = recognizer.listen(source, timeout=3)
                    command = recognizer.recognize_google(audio)
                    print(f"Heard: {command}")
                    if "stop" in command.lower():
                        with self.lock:
                            self.stop_flag = True
                except Exception:
                    continue

    def update_state(self, mouse_y):
        with self.lock:
            if self.stop_flag:
                self.torque_command = 0
                self.velocity = 0
                return

            new_position = 1 - mouse_y / self.height  # Normalize [0,1]
            new_velocity = (new_position - self.position) / 0.033  # dt ~30fps
            self.position = new_position
            self.velocity = new_velocity

            # Simple resistance curve
            # Use a logistic function to smoothly transition between 100 and 200
            # when self.position goes from 0.2 to 0.8.
            L = lambda x: 1 / (1 + math.exp(-15 * (x - 0.5)))
            low_val = L(0.2)
            high_val = L(0.8)
            f = (L(self.position) - low_val) / (high_val - low_val)
            f = max(0, min(f, 1))  # Clamp between 0 and 1
            base_torque = 100 + f * 100

            # Smoothly reduce torque in the upper range (spotter region) using another logistic curve.
            if self.position > 0.9:
                spot_multiplier = 0.5 + 0.5 / (1 + math.exp(50 * (self.position - 0.95)))
                base_torque *= spot_multiplier

            # Dynamic torque adjustment based on velocity
            target_velocity = 0.6
            k_p = 150
            deltaV = self.velocity - target_velocity
            additionalTorque = 0
            if self.velocity < 0:
                additionalTorque = 0
            
            self.torque_command = base_torque + k_p * (self.velocity - target_velocity)
            
            self.torque_command = max(0, min(400, self.torque_command))

            # Log values
            self.history["position"].append(self.position)
            self.history["velocity"].append(self.velocity)
            self.history["torque"].append(self.torque_command)

    def draw_plot(self):
        plot_img = np.zeros((300, self.width, 3), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for i, (label, color) in enumerate(zip(self.history, [(0,255,0),(255,0,0),(0,0,255)])):
            data = list(self.history[label])
            if len(data) < 2:
                continue

            # Normalize and plot
            norm = np.interp(data, (min(data), max(data)), (280 - i*100, 200 - i*100))
            for j in range(1, len(norm)):
                cv2.line(plot_img, (j-1, int(norm[j-1])), (j, int(norm[j])), color, 1)

            # Draw label and most recent value
            latest_value = data[-1]
            display_text = f"{label}: {latest_value:.3f}"
            cv2.putText(plot_img, display_text, (10, 280 - i*100), font, 0.5, color, 1)

        return plot_img

    def run(self):
        cv2.namedWindow("Position Input")
        cv2.namedWindow("Signal Plot")
        mouse = {"x": self.width // 2, "y": self.height // 2}

        def callback(event, mx, my, flags, param):
            if event == cv2.EVENT_MOUSEMOVE:
                mouse["x"], mouse["y"] = mx, my

        cv2.setMouseCallback("Position Input", callback)

        while True:
            input_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            cv2.circle(input_frame, (mouse["x"], mouse["y"]), 10, (0, 255, 0), -1)
            self.update_state(mouse["y"])

            plot_img = self.draw_plot()

            cv2.imshow("Position Input", input_frame)
            cv2.imshow("Signal Plot", plot_img)

            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()

# Run the simulator
simulator = SmartCableSimulator()
simulator.run()