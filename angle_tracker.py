"""
Angle tracker module.
Converts raw 0-360 angle readings into cumulative (unwrapped) angle,
tracking total rotation including multiple revolutions.
"""

import math


class AngleTracker:
    def __init__(self):
        """Initialize angle tracker."""
        self.cumulative_angle = 0.0
        self.last_raw_angle = None
        self.revolution_count = 0

    def update(self, raw_angle_deg):
        """
        Update with a new raw angle reading (from atan2, range -180 to +180).
        
        Returns the cumulative angle in degrees.
        """
        if raw_angle_deg is None:
            return self.cumulative_angle

        if self.last_raw_angle is None:
            self.last_raw_angle = raw_angle_deg
            # Keep cumulative_angle as-is (0 after reset) — just record reference
            return self.cumulative_angle

        # Compute the shortest angular difference
        delta = raw_angle_deg - self.last_raw_angle

        # Wrap delta to [-180, +180]
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.cumulative_angle += delta
        self.last_raw_angle = raw_angle_deg

        return self.cumulative_angle

    def reset(self):
        """Reset cumulative angle to zero."""
        self.cumulative_angle = 0.0
        self.last_raw_angle = None
        self.revolution_count = 0

    @property
    def revolutions(self):
        """Number of full revolutions."""
        return self.cumulative_angle / 360.0
