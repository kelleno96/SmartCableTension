"""
Smart Cable Tension - Main Control Script

Integrated system that:
  1. Captures video from external webcam (index 0)
  2. Detects orange tape on a black disc mounted on the motor axle
  3. Tracks cumulative angle (unwrapped, multi-revolution)
  4. Displays live angle graph and camera overlay
  5. Allows motor control via keyboard

Controls:
  UP/DOWN arrows or W/S: Increase/decrease motor speed offset
  SPACE: Stop motor (set to neutral)
  R: Reset cumulative angle to zero
  Q: Quit (stops motor first)

Motor speed range is clamped to ±20 from neutral for safety.
"""

import cv2
import time
from camera import Camera
from detector import OrangeDetector
from angle_tracker import AngleTracker
from live_plotter import LivePlotter
from motor_controller import MotorController


# Safety limits
MAX_OFFSET = 20  # Max speed offset from neutral (±20 logical units)
SPEED_STEP = 2   # How much each keypress changes speed


def main():
    # --- Initialize modules ---
    print("Initializing camera...")
    cam = Camera(index=0, width=320, height=240, fps=60)

    print("Initializing detector...")
    det = OrangeDetector()

    print("Initializing angle tracker...")
    tracker = AngleTracker()

    print("Initializing plotter...")
    plotter = LivePlotter(width=640, height=300, max_points=600)

    print("Initializing motor controller...")
    motor = MotorController(port='/dev/cu.usbserial-10', baudrate=115200)
    motor.connect()

    # --- Create windows ---
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Angle Plot", cv2.WINDOW_NORMAL)

    # Click handler to adjust center
    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            det.center = (x, y)
            print(f"Center set to ({x}, {y})")

    cv2.setMouseCallback("Detection", on_click)

    # --- State ---
    speed_offset = 0  # Current speed offset from neutral
    frame_count = 0
    fps_timer = time.time()
    fps_display = 0

    print("\n=== Smart Cable Tension System ===")
    print(f"  Motor speed range: neutral ± {MAX_OFFSET}")
    print(f"  Speed step: {SPEED_STEP} per keypress")
    print("  Controls:")
    print("    W/UP    = increase speed (positive direction)")
    print("    S/DOWN  = decrease speed (negative direction)")
    print("    SPACE   = stop motor")
    print("    R       = reset angle to zero")
    print("    Q       = quit")
    print()

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                continue

            # --- Detect orange tape ---
            result = det.detect(frame)

            # --- Update cumulative angle ---
            cumulative = tracker.update(result['angle_deg'])

            # --- Draw camera overlay ---
            display = frame.copy()
            det.draw_overlay(display, result)

            # Show cumulative angle on camera view
            cv2.putText(display, f"Cum: {cumulative:.1f} ({cumulative/360:.2f} rev)",
                        (10, display.shape[0] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)

            # Show motor speed
            cv2.putText(display, f"Motor: offset {speed_offset:+d}",
                        (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

            # FPS
            frame_count += 1
            elapsed = time.time() - fps_timer
            if elapsed > 0.5:
                fps_display = frame_count / elapsed
                frame_count = 0
                fps_timer = time.time()
            cv2.putText(display, f"FPS: {fps_display:.0f}", (display.shape[1] - 80, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # --- Update plotter ---
            plotter.update(cumulative, speed_offset)
            plot_img = plotter.render()

            # --- Display ---
            cv2.imshow("Detection", display)
            cv2.imshow("Angle Plot", plot_img)

            # --- Handle input ---
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('w') or key == 0:  # W or UP arrow
                speed_offset = min(MAX_OFFSET, speed_offset + SPEED_STEP)
                motor.set_speed(motor.NEUTRAL + int(speed_offset * motor.UNIT))
                print(f"Speed: offset {speed_offset:+d}")
            elif key == ord('s') or key == 1:  # S or DOWN arrow
                speed_offset = max(-MAX_OFFSET, speed_offset - SPEED_STEP)
                motor.set_speed(motor.NEUTRAL + int(speed_offset * motor.UNIT))
                print(f"Speed: offset {speed_offset:+d}")
            elif key == ord(' '):
                speed_offset = 0
                motor.stop()
                print("Motor stopped.")
            elif key == ord('r'):
                tracker.reset()
                plotter.reset()
                print("Angle reset to zero.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Shutting down...")
        motor.stop()
        motor.disconnect()
        cam.release()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
