"""
Orange tape detector module.
Detects dull orange tape on a black circle using HSV color filtering,
finds the centroid of the tape, and computes its angle relative to a center point.
"""

import cv2
import numpy as np
import math


class OrangeDetector:
    # HSV range for dull orange tape (tuned)
    HSV_LOWER = np.array([84, 138, 64])
    HSV_UPPER = np.array([129, 255, 213])

    # Minimum contour area to consider (filters noise)
    MIN_CONTOUR_AREA = 30

    # Circular ROI mask radius (pixels) centered on rotation center
    MASK_RADIUS = 100

    def __init__(self, center=(160, 111)):
        """Initialize detector with default HSV thresholds."""
        self.hsv_lower = self.HSV_LOWER.copy()
        self.hsv_upper = self.HSV_UPPER.copy()
        self.center = center  # Center of rotation (motor axle)
        self.mask_radius = self.MASK_RADIUS

    def detect(self, frame):
        """
        Detect the orange tape in a BGR frame.
        
        Returns:
            result dict with keys:
                - 'found': bool
                - 'mask': binary mask of orange regions
                - 'contour': largest contour (or None)
                - 'centroid': (x, y) of tape centroid (or None)
                - 'angle_deg': angle in degrees from center, 0=right, CCW positive (or None)
                - 'center': (cx, cy) center point used for angle computation
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Apply circular ROI mask centered on the rotation center
        circle_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(circle_mask, self.center, self.mask_radius, 255, -1)
        mask = cv2.bitwise_and(mask, circle_mask)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = {
            'found': False,
            'mask': mask,
            'contour': None,
            'centroid': None,
            'angle_deg': None,
            'center': self.center,
        }

        if not contours:
            return result

        # Filter by minimum area and find the largest qualifying contour
        valid = [c for c in contours if cv2.contourArea(c) >= self.MIN_CONTOUR_AREA]
        if not valid:
            return result

        largest = max(valid, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return result

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Angle from center: atan2 with y-axis flipped (image coords)
        dx = cx - self.center[0]
        dy = -(cy - self.center[1])  # Flip y so up is positive
        angle = math.degrees(math.atan2(dy, dx))

        result['found'] = True
        result['contour'] = largest
        result['centroid'] = (cx, cy)
        result['angle_deg'] = angle

        return result

    def draw_overlay(self, frame, result):
        """
        Draw detection overlay on the frame.
        
        Args:
            frame: BGR image to draw on (will be modified in place).
            result: dict from detect().
            
        Returns:
            frame with overlay drawn.
        """
        center = result['center']

        # Draw ROI circle
        cv2.circle(frame, center, self.mask_radius, (80, 80, 80), 1)

        # Draw center crosshair
        cv2.drawMarker(frame, center, (0, 255, 255), cv2.MARKER_CROSS, 20, 1)

        if result['found']:
            # Draw contour
            cv2.drawContours(frame, [result['contour']], -1, (0, 255, 0), 2)

            # Draw centroid
            cv2.circle(frame, result['centroid'], 6, (0, 0, 255), -1)

            # Draw line from center to centroid
            cv2.line(frame, center, result['centroid'], (255, 0, 255), 2)

            # Show angle text (with dark background for readability)
            angle_text = f"{result['angle_deg']:.1f} deg"
            (tw, th), _ = cv2.getTextSize(angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(frame, (8, 6), (14 + tw, 30 + th), (0, 0, 0), -1)
            cv2.putText(frame, angle_text, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.rectangle(frame, (8, 6), (200, 34), (0, 0, 0), -1)
            cv2.putText(frame, "NO DETECTION", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return frame
