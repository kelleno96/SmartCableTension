"""
Structured text logger for the Smart Cable Tension system.
Writes timestamped log entries to a text file for later review/analysis.
"""

import os
import time
from datetime import datetime


class Logger:
    def __init__(self, log_dir="logs"):
        """
        Initialize logger. Creates a new timestamped log file.
        
        Args:
            log_dir: Directory to store log files.
        """
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"session_{timestamp}.log")
        self.start_time = time.time()
        
        with open(self.log_path, 'w') as f:
            f.write(f"=== Smart Cable Tension Session Log ===\n")
            f.write(f"Started: {datetime.now().isoformat()}\n")
            f.write(f"{'='*50}\n\n")
        
        print(f"Logging to: {self.log_path}")

    def _elapsed(self):
        return time.time() - self.start_time

    def log(self, category, message):
        """Write a log entry with timestamp and category."""
        entry = f"[{self._elapsed():8.2f}s] [{category:^12}] {message}\n"
        with open(self.log_path, 'a') as f:
            f.write(entry)

    def log_calibration(self, min_angle, max_angle, motor_direction):
        """Log calibration event."""
        self.log("CALIBRATE", f"min_angle={min_angle:.1f}, max_angle={max_angle:.1f}, "
                 f"motor_dir={'positive' if motor_direction > 0 else 'negative'}")

    def log_motor(self, speed_byte, offset):
        """Log motor speed change."""
        self.log("MOTOR", f"speed={speed_byte}, offset={offset:+d}")

    def log_workout_mode(self, mode, params=None):
        """Log workout mode change."""
        self.log("WORKOUT", f"mode={mode}" + (f", params={params}" if params else ""))

    def log_angle(self, raw_angle, cumulative_angle):
        """Log angle measurement (call sparingly, e.g. every N frames)."""
        self.log("ANGLE", f"raw={raw_angle:.1f}, cumulative={cumulative_angle:.1f}")

    def log_safety(self, message):
        """Log safety event."""
        self.log("SAFETY", message)

    def log_event(self, message):
        """Log general event."""
        self.log("EVENT", message)

    def log_control(self, mode, target, actual, motor_output):
        """Log control loop state (call sparingly)."""
        self.log("CONTROL", f"mode={mode}, target={target:.1f}, actual={actual:.1f}, "
                 f"motor_out={motor_output}")

    def close(self):
        """Write session end marker."""
        self.log("EVENT", "Session ended.")
        with open(self.log_path, 'a') as f:
            f.write(f"\n{'='*50}\n")
            f.write(f"Ended: {datetime.now().isoformat()}\n")
            f.write(f"Duration: {self._elapsed():.1f}s\n")
