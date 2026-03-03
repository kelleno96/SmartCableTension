"""
Live angle plotter module.
Renders a real-time scrolling graph of cumulative angle using OpenCV
(no matplotlib — pure OpenCV for maximum speed).
"""

import cv2
import numpy as np
from collections import deque
import time


class LivePlotter:
    def __init__(self, width=640, height=300, max_points=600, title="Cumulative Angle"):
        """
        Initialize live plotter.
        
        Args:
            width: Plot image width in pixels.
            height: Plot image height in pixels.
            max_points: Number of data points to display (scrolling window).
            title: Window/plot title.
        """
        self.width = width
        self.height = height
        self.max_points = max_points
        self.title = title

        self.angle_history = deque(maxlen=max_points)
        self.time_history = deque(maxlen=max_points)
        self.speed_history = deque(maxlen=max_points)
        self.start_time = time.time()

        # Plot margins
        self.margin_left = 60
        self.margin_right = 10
        self.margin_top = 30
        self.margin_bottom = 30
        self.plot_w = width - self.margin_left - self.margin_right
        self.plot_h = height - self.margin_top - self.margin_bottom

    def update(self, cumulative_angle, motor_speed=None):
        """Add a new data point."""
        now = time.time() - self.start_time
        self.angle_history.append(cumulative_angle)
        self.time_history.append(now)
        if motor_speed is not None:
            self.speed_history.append(motor_speed)

    def render(self):
        """Render the plot as a BGR image."""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:] = (30, 30, 30)  # Dark background

        if len(self.angle_history) < 2:
            cv2.putText(img, "Waiting for data...", (self.margin_left, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
            return img

        angles = list(self.angle_history)
        times = list(self.time_history)

        # Auto-scale Y axis
        y_min = min(angles)
        y_max = max(angles)
        y_range = y_max - y_min
        if y_range < 10:
            y_mid = (y_max + y_min) / 2
            y_min = y_mid - 5
            y_max = y_mid + 5
            y_range = 10
        # Add 10% padding
        y_min -= y_range * 0.1
        y_max += y_range * 0.1
        y_range = y_max - y_min

        t_min = times[0]
        t_max = times[-1]
        t_range = t_max - t_min
        if t_range < 0.1:
            t_range = 0.1

        # Draw grid lines
        for i in range(5):
            y_val = y_min + (i / 4.0) * y_range
            py = self.margin_top + int((1 - (y_val - y_min) / y_range) * self.plot_h)
            cv2.line(img, (self.margin_left, py), (self.width - self.margin_right, py),
                     (60, 60, 60), 1)
            label = f"{y_val:.0f}"
            cv2.putText(img, label, (5, py + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        # Draw zero line if in range
        if y_min <= 0 <= y_max:
            py_zero = self.margin_top + int((1 - (0 - y_min) / y_range) * self.plot_h)
            cv2.line(img, (self.margin_left, py_zero), (self.width - self.margin_right, py_zero),
                     (100, 100, 100), 1)

        # Plot angle curve
        points = []
        for t, a in zip(times, angles):
            px = self.margin_left + int(((t - t_min) / t_range) * self.plot_w)
            py = self.margin_top + int((1 - (a - y_min) / y_range) * self.plot_h)
            points.append((px, py))

        for i in range(1, len(points)):
            cv2.line(img, points[i - 1], points[i], (0, 255, 0), 2)

        # Plot motor speed if available
        if len(self.speed_history) >= 2:
            speeds = list(self.speed_history)
            # Normalize motor offset: 0 = neutral, range is ±max_offset
            s_min, s_max = -50, 50  # offset range in logical units
            for i in range(1, len(speeds)):
                t1, t2 = times[i - 1], times[i]
                s1, s2 = speeds[i - 1], speeds[i]
                px1 = self.margin_left + int(((t1 - t_min) / t_range) * self.plot_w)
                px2 = self.margin_left + int(((t2 - t_min) / t_range) * self.plot_w)
                # Map speed to plot y range
                py1 = self.margin_top + int((1 - (s1 - s_min) / (s_max - s_min)) * self.plot_h)
                py2 = self.margin_top + int((1 - (s2 - s_min) / (s_max - s_min)) * self.plot_h)
                py1 = max(self.margin_top, min(self.margin_top + self.plot_h, py1))
                py2 = max(self.margin_top, min(self.margin_top + self.plot_h, py2))
                cv2.line(img, (px1, py1), (px2, py2), (255, 100, 0), 1)

        # Title and current value
        cv2.putText(img, self.title, (self.margin_left, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2)
        current = angles[-1]
        revs = current / 360.0
        cv2.putText(img, f"{current:.1f} deg ({revs:.2f} rev)",
                    (self.width - 260, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Legend
        cv2.putText(img, "Angle", (self.margin_left + 5, self.height - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        if len(self.speed_history) >= 2:
            cv2.putText(img, "Motor", (self.margin_left + 70, self.height - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 0), 1)

        # Border
        cv2.rectangle(img, (self.margin_left, self.margin_top),
                      (self.width - self.margin_right, self.margin_top + self.plot_h),
                      (100, 100, 100), 1)

        return img

    def reset(self):
        """Clear all history."""
        self.angle_history.clear()
        self.time_history.clear()
        self.speed_history.clear()
        self.start_time = time.time()
