"""
Camera capture module.
Handles webcam initialization, frame capture, and resolution configuration.
"""

import cv2


class Camera:
    def __init__(self, index=0, width=320, height=240, fps=60):
        """
        Initialize camera capture.
        
        Args:
            index: Camera device index (1 = external webcam, 0 = built-in).
            width: Capture width in pixels (lower = faster).
            height: Capture height in pixels (lower = faster).
            fps: Requested frame rate.
        """
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {index}")
        
        # Set low resolution for speed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Read back actual values
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        print(f"Camera opened: {self.width}x{self.height} @ {self.fps}fps")

    def read(self):
        """Read a frame. Returns (success, frame)."""
        return self.cap.read()

    def release(self):
        """Release the camera."""
        self.cap.release()

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
