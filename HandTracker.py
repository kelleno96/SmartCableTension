import cv2
import mediapipe as mp
import threading
from collections import deque
import numpy as np

# Globals shared between threads
latest_frame = None
current_hand = None
data_lock = threading.Lock()
stop_event = threading.Event()

# Position and torque settings
low_position = None
high_position = None
MAX_HISTORY = 1280  # Same as in Simulation.py

# History tracking
position_history = deque(maxlen=MAX_HISTORY)
torque_history = deque(maxlen=MAX_HISTORY)

def hand_tracking():
    global latest_frame, current_hand
    cap = cv2.VideoCapture(0)
    mp_hands = mp.solutions.hands
    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5
    ) as hands:
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                continue
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
            with data_lock:
                latest_frame = frame.copy()
                current_hand = hand_point
    cap.release()

def calculate_torque(y_pos):
    global low_position, high_position
    
    if low_position is None or high_position is None or y_pos is None:
        return 0.0
        
    if y_pos > low_position:  # Remember, higher y means lower in image
        return 0.0
    if y_pos < high_position:  # Above high position
        return 0.0
        
    pixel_buffer_size = 80
    # Calculate the transition zones
    low_trans = low_position - pixel_buffer_size
    high_trans = high_position + pixel_buffer_size
    
    if y_pos > low_trans:  # In the lower transition zone
        ratio = (low_position - y_pos) / pixel_buffer_size
        return 50.0 * ratio
    elif y_pos < high_trans:  # In the upper transition zone
        ratio = (y_pos - high_position) / pixel_buffer_size
        return 50.0 * ratio
    
    return 50.0  # Constant torque in the middle range

def draw_plots():
    plot_height = 300
    plot_width = MAX_HISTORY
    plot_img = np.zeros((plot_height, plot_width, 3), dtype=np.uint8)
    
    if len(position_history) > 1 and len(torque_history) > 1:
        # Plot position (green)
        position_data = list(position_history)
        position_norm = np.interp(position_data, (0, 480), (0, 280))  # Normalize to plot height
        for i in range(1, len(position_norm)):
            cv2.line(plot_img, 
                    (i-1, int(position_norm[i-1])), 
                    (i, int(position_norm[i])), 
                    (0, 255, 0), 1)
        
        # Plot torque (red)
        torque_data = list(torque_history)
        torque_norm = np.interp(torque_data, (0, 100), (280, 200))
        for i in range(1, len(torque_norm)):
            cv2.line(plot_img, 
                    (i-1, int(torque_norm[i-1])), 
                    (i, int(torque_norm[i])), 
                    (0, 0, 255), 1)
            
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(plot_img, f"Position: {position_data[-1]:.1f}", 
                   (10, 20), font, 0.5, (0, 255, 0), 1)
        cv2.putText(plot_img, f"Torque: {torque_data[-1]:.1f}%", 
                   (10, 40), font, 0.5, (0, 0, 255), 1)
        
        # Draw markers for low and high positions if set
        if low_position is not None:
            low_y = int(np.interp(low_position, (0, 480), (0, 280)))
            cv2.line(plot_img, (0, low_y), (plot_width, low_y), (255, 255, 255), 1)
        if high_position is not None:
            high_y = int(np.interp(high_position, (0, 480), (0, 280)))
            cv2.line(plot_img, (0, high_y), (plot_width, high_y), (255, 255, 255), 1)
    
    return plot_img

def display_and_print():
    global latest_frame, current_hand, low_position, high_position
    
    cv2.namedWindow("Hand Tracking")
    cv2.namedWindow("Signals")
    
    while not stop_event.is_set():
        with data_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
            hand = current_hand

        if frame is not None:
            # Draw markers for low and high positions
            if low_position is not None:
                cv2.line(frame, (0, low_position), (frame.shape[1], low_position), 
                        (255, 0, 0), 1)
            if high_position is not None:
                cv2.line(frame, (0, high_position), (frame.shape[1], high_position), 
                        (0, 0, 255), 1)

            if hand:
                # Update histories
                position_history.append(hand[1])  # Y position only
                torque = calculate_torque(hand[1])
                torque_history.append(torque)
                
                # Display current values
                cv2.putText(frame, f"Torque: {torque:.1f}%", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                position_history.append(480)  # Bottom of frame when hand not found
                torque_history.append(0)
                
            # Show the frame and plots
            cv2.imshow("Hand Tracking", frame)
            cv2.imshow("Signals", draw_plots())

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            stop_event.set()
            break
        elif key == ord('l'):
            if hand:
                low_position = hand[1]
                print(f"Set low position to {low_position}")
        elif key == ord('h'):
            if hand:
                high_position = hand[1]
                print(f"Set high position to {high_position}")

    cv2.destroyAllWindows()

def main():
    t1 = threading.Thread(target=hand_tracking)
    t1.start()
    display_and_print()
    t1.join()

if __name__ == '__main__':
    main()