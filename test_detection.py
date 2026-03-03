"""
Test script for orange tape detection.
Opens external webcam, detects the orange tape, and shows:
  - Live camera feed with detection overlay
  - HSV mask (what the detector sees as orange)

Controls:
  - 'q' to quit
  - 'c' to set/reset center point (click on image)
  - Trackbars to tune HSV thresholds in real-time

Press 'q' to exit.
"""

import cv2
import numpy as np
from camera import Camera
from detector import OrangeDetector


def nothing(x):
    pass


def main():
    # Use external webcam at index 0, low resolution for speed
    cam = Camera(index=0, width=320, height=240, fps=60)
    det = OrangeDetector()

    # Create windows
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.namedWindow("HSV Tuning", cv2.WINDOW_NORMAL)

    # Create trackbars for HSV tuning
    cv2.createTrackbar("H Low", "HSV Tuning", det.hsv_lower[0], 180, nothing)
    cv2.createTrackbar("H High", "HSV Tuning", det.hsv_upper[0], 180, nothing)
    cv2.createTrackbar("S Low", "HSV Tuning", det.hsv_lower[1], 255, nothing)
    cv2.createTrackbar("S High", "HSV Tuning", det.hsv_upper[1], 255, nothing)
    cv2.createTrackbar("V Low", "HSV Tuning", det.hsv_lower[2], 255, nothing)
    cv2.createTrackbar("V High", "HSV Tuning", det.hsv_upper[2], 255, nothing)

    # Make a small blank image for the trackbar window
    tuning_img = np.zeros((1, 400, 3), dtype=np.uint8)

    # Click handler to set center point
    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            det.center = (x, y)
            print(f"Center set to ({x}, {y})")

    cv2.setMouseCallback("Detection", on_click)

    print("\n=== Orange Tape Detection Test ===")
    print("  - Adjust HSV sliders to tune detection")
    print("  - Click on the 'Detection' window to set rotation center")
    print("  - Press 'q' to quit\n")

    frame_count = 0
    fps_timer = cv2.getTickCount()

    while True:
        ret, frame = cam.read()
        if not ret:
            continue

        # Read trackbar positions and update detector thresholds
        det.hsv_lower[0] = cv2.getTrackbarPos("H Low", "HSV Tuning")
        det.hsv_upper[0] = cv2.getTrackbarPos("H High", "HSV Tuning")
        det.hsv_lower[1] = cv2.getTrackbarPos("S Low", "HSV Tuning")
        det.hsv_upper[1] = cv2.getTrackbarPos("S High", "HSV Tuning")
        det.hsv_lower[2] = cv2.getTrackbarPos("V Low", "HSV Tuning")
        det.hsv_upper[2] = cv2.getTrackbarPos("V High", "HSV Tuning")

        # Detect orange tape
        result = det.detect(frame)

        # Draw overlay on frame
        display = frame.copy()
        det.draw_overlay(display, result)

        # FPS counter
        frame_count += 1
        elapsed = (cv2.getTickCount() - fps_timer) / cv2.getTickFrequency()
        if elapsed > 0.5:
            fps = frame_count / elapsed
            frame_count = 0
            fps_timer = cv2.getTickCount()
        else:
            fps = 0
        cv2.putText(display, f"FPS: {fps:.0f}", (10, display.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Show detection info
        if result['found']:
            cx, cy = result['centroid']
            area = cv2.contourArea(result['contour'])
            info = f"Centroid: ({cx},{cy})  Area: {area:.0f}"
            cv2.putText(display, info, (10, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Show mask as color
        mask_color = cv2.cvtColor(result['mask'], cv2.COLOR_GRAY2BGR)

        cv2.imshow("Detection", display)
        cv2.imshow("Mask", mask_color)
        cv2.imshow("HSV Tuning", tuning_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Print final HSV values for reference
    print(f"\nFinal HSV range:")
    print(f"  Lower: [{det.hsv_lower[0]}, {det.hsv_lower[1]}, {det.hsv_lower[2]}]")
    print(f"  Upper: [{det.hsv_upper[0]}, {det.hsv_upper[1]}, {det.hsv_upper[2]}]")

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
