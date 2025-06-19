import cv2
import mediapipe as mp
import threading
from collections import deque
import numpy as np
import speech_recognition as sr
import time

class HandTracker:
    def __init__(self):
        # Thread synchronization
        self.data_lock = threading.Lock()
        self.voice_lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # Shared state between threads
        self.latest_frame = None
        self.current_hand = None
        
        # Position and torque settings
        self.low_position = None
        self.high_position = None
        self.voice_pause = False
        self.MAX_HISTORY = 640
        self.PIXEL_BUFFER_SIZE = 80
        
        # History tracking
        self.position_history = deque(maxlen=self.MAX_HISTORY)
        self.torque_history = deque(maxlen=self.MAX_HISTORY)
        self.velocity_history = deque(maxlen=self.MAX_HISTORY)
        self.acceleration_history = deque(maxlen=self.MAX_HISTORY)
        
        # Time tracking for derivatives
        self.last_time = time.time()
        self.last_velocity = 0
        self.last_position = None
        
    def hand_tracking(self):
        cap = cv2.VideoCapture(0)
        mp_hands = mp.solutions.hands
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5
        ) as hands:
            i = 0
            while not self.stop_event.is_set():
                ret, frame = cap.read()
                if not ret:
                    continue
                i+=1
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = hands.process(frame_rgb)
                hand_point = None
                if results.multi_hand_landmarks:
                    # Use index finger tip (landmark 8) for XY position.
                    for hand_landmarks in results.multi_hand_landmarks:
                        h, w, _ = frame.shape
                        cx = int(hand_landmarks.landmark[8].x * w)
                        cy = int(hand_landmarks.landmark[8].y * h)
                        hand_point = (cx, cy)
                        # Draw a circle at the hand position.
                        cv2.circle(frame, hand_point, 10, (0, 255, 0), -1)
                        break
                with self.data_lock:
                    self.latest_frame = frame.copy()
                    self.current_hand = hand_point
        cap.release()

    def calculate_torque(self, y_pos):
        with self.voice_lock:
            if self.voice_pause:
                return 0.0
        
        if self.low_position is None or self.high_position is None or y_pos is None:
            return 0.0
            
        if y_pos > self.low_position:  # Remember, higher y means lower in image
            return 0.0
        if y_pos < self.high_position:  # Above high position
            return 0.0
            
        # Calculate the transition zones
        low_trans = self.low_position - self.PIXEL_BUFFER_SIZE
        high_trans = self.high_position + self.PIXEL_BUFFER_SIZE
        
        if y_pos > low_trans:  # In the lower transition zone
            ratio = (self.low_position - y_pos) / self.PIXEL_BUFFER_SIZE
            return 50.0 * ratio
        elif y_pos < high_trans:  # In the upper transition zone
            ratio = (y_pos - self.high_position) / self.PIXEL_BUFFER_SIZE
            return 50.0 * ratio
        
        return 50.0  # Constant torque in the middle range

    def draw_plots(self):
        plot_height = 600  # Increased height to accommodate new graphs
        plot_width = self.MAX_HISTORY
        plot_img = np.zeros((plot_height, plot_width, 3), dtype=np.uint8)
        
        if len(self.position_history) > 1:
            # Plot position (green) in top section
            position_data = list(self.position_history)
            position_norm = np.interp(position_data, (0, 1080), (0, 180))
            for i in range(1, len(position_norm)):
                cv2.line(plot_img, 
                        (i-1, int(position_norm[i-1])), 
                        (i, int(position_norm[i])), 
                        (0, 255, 0), 1)
            
            # Plot torque (red) sharing space with position
            if len(self.torque_history) > 1:
                torque_data = list(self.torque_history)
                torque_norm = np.interp(torque_data, (0, 100), (180, 100))
                for i in range(1, len(torque_norm)):
                    cv2.line(plot_img, 
                            (i-1, int(torque_norm[i-1])), 
                            (i, int(torque_norm[i])), 
                            (0, 0, 255), 1)
            
            # Plot velocity (blue) in middle section
            if len(self.velocity_history) > 1:
                velocity_data = list(self.velocity_history)
                velocity_norm = np.interp(velocity_data, (-2000, 2000), (380, 300))
                for i in range(1, len(velocity_norm)):
                    cv2.line(plot_img, 
                            (i-1, int(velocity_norm[i-1])), 
                            (i, int(velocity_norm[i])), 
                            (255, 255, 0), 1)
            
            # Plot acceleration (cyan) in bottom section
            if len(self.acceleration_history) > 1:
                accel_data = list(self.acceleration_history)
                accel_norm = np.interp(accel_data, (-10000, 10000), (580, 500))
                for i in range(1, len(accel_norm)):
                    cv2.line(plot_img, 
                            (i-1, int(accel_norm[i-1])), 
                            (i, int(accel_norm[i])), 
                            (255, 255, 255), 1)
            
            # Add labels and current values
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(plot_img, f"Position: {position_data[-1]:.1f} px", 
                       (10, 20), font, 0.5, (0, 255, 0), 1)
            cv2.putText(plot_img, f"Torque: {self.torque_history[-1]:.1f}%", 
                       (10, 40), font, 0.5, (0, 0, 255), 1)
            cv2.putText(plot_img, f"Velocity: {self.velocity_history[-1]:.1f} px/s", 
                       (10, 300), font, 0.5, (255, 255, 0), 1)
            cv2.putText(plot_img, f"Acceleration: {self.acceleration_history[-1]:.1f} px/s²", 
                       (10, 500), font, 0.5, (255, 255, 255), 1)
            
            # Draw markers for low and high positions on position plot
            if self.low_position is not None:
                low_y = int(np.interp(self.low_position, (0, 1080), (0, 180)))
                cv2.line(plot_img, (0, low_y), (plot_width, low_y), (255, 255, 255), 1)
            if self.high_position is not None:
                high_y = int(np.interp(self.high_position, (0, 1080), (0, 180)))
                cv2.line(plot_img, (0, high_y), (plot_width, high_y), (255, 255, 255), 1)
            
            # Draw zero lines for velocity and acceleration
            cv2.line(plot_img, (0, 340), (plot_width, 340), (128, 128, 128), 1)  # Velocity zero line
            cv2.line(plot_img, (0, 540), (plot_width, 540), (128, 128, 128), 1)  # Acceleration zero line
        
        return plot_img

    def voice_listener(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        with mic as source:
            recognizer.adjust_for_ambient_noise(source)
            print("Voice commands ready! Say 'stop', 'begin', 'set low position', or 'set high position'")
            
            while not self.stop_event.is_set():
                try:
                    audio = recognizer.listen(source, timeout=1, phrase_time_limit=3)
                    try:
                        command = recognizer.recognize_google(audio).lower()
                        print(f"Heard: {command}")
                        
                        with self.voice_lock:
                            if "stop" in command:
                                self.voice_pause = True
                                print("System paused")
                            elif "begin" in command:
                                self.voice_pause = False
                                print("System resumed")
                            elif "set low position" in command:
                                with self.data_lock:
                                    if self.current_hand:
                                        self.low_position = self.current_hand[1]
                                        print(f"Set low position to {self.low_position}")
                            elif "set high position" in command:
                                with self.data_lock:
                                    if self.current_hand:
                                        self.high_position = self.current_hand[1]
                                        print(f"Set high position to {self.high_position}")
                    except sr.UnknownValueError:
                        pass  # Speech was unintelligible
                    except sr.RequestError:
                        print("Could not request results from speech recognition service")
                except sr.WaitTimeoutError:
                    continue  # No speech detected within timeout

    def display_and_print(self):
        cv2.namedWindow("Hand Tracking")
        cv2.namedWindow("Signals")
        
        while not self.stop_event.is_set():
            with self.data_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
                hand = self.current_hand

            if frame is not None:
                # Draw markers for low and high positions
                if self.low_position is not None:
                    cv2.line(frame, (0, self.low_position), (frame.shape[1], self.low_position), 
                            (255, 0, 0), 1)
                if self.high_position is not None:
                    cv2.line(frame, (0, self.high_position), (frame.shape[1], self.high_position), 
                            (0, 0, 255), 1)

                if hand:
                    # Update position and calculate derivatives
                    y_pos = hand[1]
                    alpha = 0.15
                    y_pos = alpha * y_pos + (1 - alpha) * (self.position_history[-1] if self.position_history else y_pos)
                    velocity, acceleration = self.calculate_derivatives(y_pos)
                    
                    # Update histories
                    self.position_history.append(y_pos)
                    self.velocity_history.append(velocity)
                    self.acceleration_history.append(acceleration)
                    
                    torque = self.calculate_torque(y_pos)
                    self.torque_history.append(torque)
                    
                    # Display current values
                    cv2.putText(frame, f"Torque: {torque:.1f}%", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # else:
                #     # When no hand is detected, update histories with default values
                #     self.position_history.append(480)
                #     self.velocity_history.append(0)
                #     self.acceleration_history.append(0)
                #     self.torque_history.append(0)
                #     self.last_position = None  # Reset for velocity calculation
                    
                # Show the frame and plots
                cv2.imshow("Hand Tracking", frame)
                cv2.imshow("Signals", self.draw_plots())

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stop_event.set()
                break
            elif key == ord('l'):
                if hand:
                    self.low_position = hand[1]
                    print(f"Set low position to {self.low_position}")
            elif key == ord('h'):
                if hand:
                    self.high_position = hand[1]
                    print(f"Set high position to {self.high_position}")

        cv2.destroyAllWindows()

    def calculate_derivatives(self, current_y):
        current_time = time.time()
        dt = current_time - self.last_time
        # Calculate velocity (pixels per second)
        if self.last_position is not None and dt > 0:
            velocity = (current_y - self.last_position) / dt
        else:
            velocity = 0
            
        # Calculate acceleration (pixels per second squared)
        if dt > 0:
            acceleration = (velocity - self.last_velocity) / dt
        else:
            acceleration = 0
            
        
        
        # Apply some smoothing
        alpha = 0.15
        velocity = alpha * velocity + (1-alpha) * self.last_velocity
        acceleration = alpha * acceleration + (1-alpha) * (self.acceleration_history[-1] if self.acceleration_history else 0)
        
        # Update last values
        self.last_time = current_time
        self.last_position = current_y
        self.last_velocity = velocity
        return velocity, acceleration

    def run(self):
        hand_thread = threading.Thread(target=self.hand_tracking)
        voice_thread = threading.Thread(target=self.voice_listener, daemon=True)
        
        hand_thread.start()
        voice_thread.start()
        self.display_and_print()
        hand_thread.join()

def main():
    tracker = HandTracker()
    tracker.run()

if __name__ == '__main__':
    main()