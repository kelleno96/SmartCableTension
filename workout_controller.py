"""
Workout controller module.
Manages calibration, workout modes, closed-loop motor control, and safety limits.

Workout modes:
  - IDLE: Motor off, no control
  - CONSTANT_SPEED: Resist the user so they pull at a target speed
  - RUBBER_BAND: Increase resistance as cable is pulled further out
  - CONSTANT_WEIGHT: Constant resistance force regardless of position

Safety:
  - Motor MUST NOT drive past min or max calibrated angles
  - Motor auto-stops if angle goes out of bounds
"""

import time


class WorkoutMode:
    IDLE = "idle"
    RETRACTING = "retracting"
    CONSTANT_SPEED = "constant_speed"
    RUBBER_BAND = "rubber_band"
    CONSTANT_WEIGHT = "constant_weight"


class CalibrationState:
    UNCALIBRATED = "uncalibrated"
    MIN_SET = "min_set"
    WAITING_FOR_MAX = "waiting_for_max"
    DETECTING_DIRECTION = "detecting_direction"
    CALIBRATED = "calibrated"


class WorkoutController:
    def __init__(self, motor, logger, max_offset=20):
        """
        Initialize workout controller.
        
        Args:
            motor: MotorController instance.
            logger: Logger instance.
            max_offset: Maximum motor speed offset from neutral (safety limit).
        """
        self.motor = motor
        self.logger = logger
        self.max_offset = max_offset

        # Retract stop threshold (percentage of range)
        self.retract_stop_pct = 0.5

        # Calibration
        self.cal_state = CalibrationState.UNCALIBRATED
        self.min_angle = 0.0
        self.max_angle = 0.0
        self.angle_range = 0.0
        # motor_sign: +1 means positive motor offset pulls cable toward min (retracts)
        #             -1 means negative motor offset pulls cable toward min
        self.motor_sign = 1  # Will be auto-detected

        # Workout state
        self.mode = WorkoutMode.IDLE
        self.power_level = 5.0  # Base power level (0-50), float, user adjustable
        self.target_speed = 90.0  # degrees/sec for constant speed mode

        # Control loop state
        self.last_angle = None
        self.last_time = None
        self.angular_velocity = 0.0  # deg/sec (smoothed)
        self._raw_velocity = 0.0  # deg/sec (unsmoothed)
        self.integral_error = 0.0
        self._velocity_alpha = 0.3  # EMA smoothing: lower = smoother (0.0–1.0)

        # PI gains for constant speed mode (user-adjustable)
        self.pi_kp = 0.15
        self.pi_ki = 0.03

        # Auto-retract state (constant weight mode)
        self._auto_retract_timer = None  # time when cable first became stationary at max
        self._auto_retract_threshold_pct = 98.0  # % of range
        self._auto_retract_stationary_secs = 1.0  # seconds of stillness required
        self._auto_retract_velocity_threshold = 5.0  # deg/s — considered "stationary"
        self._auto_retract_power = 80  # retract power (fast)

        # Direction detection state
        self._dir_detect_start_time = None
        self._dir_detect_start_angle = None
        self._dir_detect_phase = 0  # 0=not started, 1=pulsing positive, 2=done

    # ---- Calibration ----

    def set_min_position(self, current_angle):
        """Set current angle as the minimum (start) position."""
        self.min_angle = current_angle
        self.cal_state = CalibrationState.WAITING_FOR_MAX
        self.logger.log_event(f"Min position set to {current_angle:.1f} deg")
        return f"Min set: {current_angle:.1f}°"

    def set_max_position(self, current_angle):
        """
        Set current angle as the maximum (extended) position.
        Triggers motor direction auto-detection.
        """
        self.max_angle = current_angle
        self.angle_range = abs(self.max_angle - self.min_angle)

        if self.angle_range < 10:
            msg = f"Angle range too small ({self.angle_range:.1f}°). Pull cable further."
            self.logger.log_event(msg)
            return msg

        self.cal_state = CalibrationState.DETECTING_DIRECTION
        self._dir_detect_phase = 1
        self._dir_detect_start_time = time.time()
        self._dir_detect_start_angle = current_angle

        # Pulse motor in positive offset direction to see which way it moves
        test_offset = min(8, self.max_offset)
        self.motor.set_speed(self.motor.NEUTRAL + int(test_offset * self.motor.UNIT))
        self.logger.log_event(
            f"Max position set to {current_angle:.1f}° (range={self.angle_range:.1f}°). "
            f"Detecting motor direction with offset +{test_offset}...")

        return f"Max set: {current_angle:.1f}°. Detecting motor direction..."

    def update_direction_detection(self, current_angle):
        """
        Called each frame during direction detection.
        After a brief pulse, determines which motor direction retracts the cable.
        Returns status message or None if still detecting.
        """
        if self.cal_state != CalibrationState.DETECTING_DIRECTION:
            return None

        elapsed = time.time() - self._dir_detect_start_time

        if self._dir_detect_phase == 1 and elapsed > 0.8:
            # Check which way the angle moved
            self.motor.stop()
            delta = current_angle - self._dir_detect_start_angle

            if abs(delta) < 2.0:
                # Didn't move enough, try with more power
                self.logger.log_event(f"Direction detection: delta={delta:.1f}° (too small), retrying with more power")
                test_offset = min(15, self.max_offset)
                self.motor.set_speed(self.motor.NEUTRAL + int(test_offset * self.motor.UNIT))
                self._dir_detect_start_time = time.time()
                self._dir_detect_start_angle = current_angle
                self._dir_detect_phase = 1  # retry
                return "Retrying direction detection..."

            # Positive offset moved the angle by `delta`
            # We want the motor to pull TOWARD min (retract)
            # "Toward min" means angle should decrease toward min_angle
            desired_direction = -1 if self.max_angle > self.min_angle else 1

            if (delta > 0 and desired_direction > 0) or (delta < 0 and desired_direction < 0):
                # Positive offset moves toward min — motor_sign = +1
                self.motor_sign = 1
            else:
                # Positive offset moves AWAY from min — motor_sign = -1
                self.motor_sign = -1

            self.cal_state = CalibrationState.CALIBRATED
            self._dir_detect_phase = 0

            msg = (f"Direction detected: motor_sign={self.motor_sign} "
                   f"(delta={delta:.1f}°). Calibration complete! "
                   f"Range: {self.min_angle:.1f}° to {self.max_angle:.1f}°")
            self.logger.log_calibration(self.min_angle, self.max_angle, self.motor_sign)
            self.logger.log_event(msg)

            # Now retract the cable back to min
            self._start_retract()
            return msg

        return None

    def _start_retract(self, power=10):
        """Pull cable back to start position at given power."""
        self.mode = WorkoutMode.RETRACTING
        retract_power = min(power, self.max_offset)
        # motor_sign tells us which direction retracts
        offset = self.motor_sign * retract_power
        self.motor.set_speed(self.motor.NEUTRAL + int(offset * self.motor.UNIT))
        self.logger.log_event(f"Retracting cable to start position (power={retract_power}, offset={offset:+d})")

    def start_manual_retract(self, power=20):
        """User-triggered retract at the specified power level."""
        if self.cal_state != CalibrationState.CALIBRATED:
            return "Must calibrate first!"
        self._start_retract(power=power)
        return f"Retracting at power {power}"

    # ---- Workout Modes ----

    def set_mode(self, mode):
        """Switch workout mode."""
        if self.cal_state != CalibrationState.CALIBRATED:
            return "Must calibrate first!"
        self.mode = mode
        self.integral_error = 0.0
        self._auto_retract_timer = None
        self.logger.log_workout_mode(mode, {"power": self.power_level})
        if mode == WorkoutMode.IDLE:
            self.motor.stop()
        return f"Mode: {mode}"

    def increase_power(self):
        """Increase base power level (0.5 steps)."""
        self.power_level = min(float(self.max_offset), self.power_level + 0.02)
        self.logger.log_event(f"Power level: {self.power_level}")
        return self.power_level

    def decrease_power(self):
        """Decrease base power level (0.5 steps)."""
        self.power_level = max(0.0, self.power_level - 0.02)
        self.logger.log_event(f"Power level: {self.power_level}")
        return self.power_level

    def increase_target_speed(self, step=5.0):
        """Increase target speed for constant speed mode."""
        self.target_speed = min(360.0, self.target_speed + step)
        self.logger.log_event(f"Target speed: {self.target_speed} deg/s")
        return self.target_speed

    def decrease_target_speed(self, step=5.0):
        """Decrease target speed for constant speed mode."""
        self.target_speed = max(5.0, self.target_speed - step)
        self.logger.log_event(f"Target speed: {self.target_speed} deg/s")
        return self.target_speed

    def emergency_stop(self):
        """Immediately stop the motor."""
        self.motor.stop()
        self.mode = WorkoutMode.IDLE
        self.integral_error = 0.0
        self._auto_retract_timer = None
        self.logger.log_safety("EMERGENCY STOP")
        return "STOPPED"

    # ---- Control Loop ----

    def update(self, current_angle, dt):
        """
        Main control loop update. Call every frame.
        
        Args:
            current_angle: Current cumulative angle in degrees.
            dt: Time since last update in seconds.
            
        Returns:
            dict with control state info.
        """
        # Compute angular velocity with EMA smoothing
        if self.last_angle is not None and dt > 0:
            self._raw_velocity = (current_angle - self.last_angle) / dt
            self.angular_velocity = (
                self._velocity_alpha * self._raw_velocity +
                (1 - self._velocity_alpha) * self.angular_velocity
            )
        self.last_angle = current_angle

        # During direction detection, delegate
        if self.cal_state == CalibrationState.DETECTING_DIRECTION:
            msg = self.update_direction_detection(current_angle)
            return {"status": msg or "detecting direction...", "motor_offset": 0}

        # ---- SAFETY: Always enforce angle bounds when calibrated ----
        if self.cal_state == CalibrationState.CALIBRATED:
            at_min_end = self._past_min(current_angle)
            at_max_end = self._past_max(current_angle)

            if at_min_end or at_max_end:
                self.motor.stop()
                which = "MIN" if at_min_end else "MAX"
                if self.mode == WorkoutMode.RETRACTING:
                    self.mode = WorkoutMode.IDLE
                    self.logger.log_event(f"Retract complete — reached {which} limit ({current_angle:.1f}°)")
                    return {"status": "Retract complete", "motor_offset": 0}
                self.logger.log_safety(f"Angle at {which} limit ({current_angle:.1f}°), motor stopped")
                return {"status": f"AT {which} LIMIT", "motor_offset": 0}

        # Handle retracting: check if we've reached the min position
        if self.mode == WorkoutMode.RETRACTING:
            # Stop when angle has returned to min_angle (or past it)
            if self.max_angle > self.min_angle:
                reached_min = current_angle <= self.min_angle
            else:
                reached_min = current_angle >= self.min_angle

            if reached_min:
                self.motor.stop()
                self.mode = WorkoutMode.IDLE
                self.logger.log_event(f"Retract complete at {current_angle:.1f}° (min={self.min_angle:.1f}°)")
                return {"status": "Retract complete — select a mode", "motor_offset": 0}
            pct = self._position_percent(current_angle)
            return {"status": f"Retracting... {pct:.0f}%", "motor_offset": round((self.motor.current_speed - self.motor.NEUTRAL) / self.motor.UNIT)}

        # If not calibrated or idle, nothing to do
        if self.cal_state != CalibrationState.CALIBRATED or self.mode == WorkoutMode.IDLE:
            return {"status": self.mode, "motor_offset": 0}

        # ---- Auto-retract in constant weight mode ----
        if self.mode == WorkoutMode.CONSTANT_WEIGHT:
            pct = self._position_percent(current_angle)
            stationary = abs(self.angular_velocity) < self._auto_retract_velocity_threshold
            if pct >= self._auto_retract_threshold_pct and stationary:
                if self._auto_retract_timer is None:
                    self._auto_retract_timer = time.time()
                elif time.time() - self._auto_retract_timer >= self._auto_retract_stationary_secs:
                    self.logger.log_event(
                        f"Auto-retract triggered: {pct:.0f}% extended, "
                        f"stationary for {self._auto_retract_stationary_secs}s")
                    self._auto_retract_timer = None
                    self._start_retract(power=self._auto_retract_power)
                    return {"status": "Auto-retracting...", "motor_offset": self.motor_sign * self._auto_retract_power}
            else:
                self._auto_retract_timer = None

        # ---- Compute motor output based on mode ----
        offset = self._compute_motor_output(current_angle, dt)

        # Clamp to safe range
        offset = max(-self.max_offset, min(self.max_offset, offset))

        # Additional safety: don't let motor push past limits
        offset = self._apply_boundary_safety(current_angle, offset)

        self.motor.set_speed(self.motor.NEUTRAL + int(offset * self.motor.UNIT))

        return {
            "status": self.mode,
            "motor_offset": offset,
            "velocity": self.angular_velocity,
            "position_pct": self._position_percent(current_angle),
        }

    def _compute_motor_output(self, current_angle, dt):
        """Compute motor offset based on current workout mode."""
        if self.mode == WorkoutMode.CONSTANT_WEIGHT:
            # Simple constant force — motor_sign determines direction
            return self.motor_sign * self.power_level

        elif self.mode == WorkoutMode.RUBBER_BAND:
            # Force increases linearly with extension from min
            pct = self._position_percent(current_angle)
            scaled_power = self.power_level * (pct / 100.0)
            return int(self.motor_sign * scaled_power)

        elif self.mode == WorkoutMode.CONSTANT_SPEED:
            # PI control on angular velocity
            # Goal: user pulls cable at target_speed deg/s
            # Motor provides resistance (retraction force) that adapts so the
            # user must maintain the target speed to overcome it.
            #
            # When user pulls faster → increase resistance
            # When user pulls slower → decrease resistance
            # When user stops       → motor should NOT actively retract (hold position)
            
            # Determine which velocity direction means "user pulling out"
            pulling_dir = 1 if self.max_angle > self.min_angle else -1
            user_pull_velocity = self.angular_velocity * pulling_dir

            # Error: positive = user pulling too fast, negative = too slow
            error = user_pull_velocity - self.target_speed
            
            # Tuned PI gains — proportional drives fast response,
            # integral eliminates steady-state offset
            kp = self.pi_kp
            ki = self.pi_ki
            self.integral_error += error * dt
            self.integral_error = max(-100, min(100, self.integral_error))  # anti-windup

            # Base resistance = power_level; PI adjusts around it
            output = self.power_level + kp * error + ki * self.integral_error

            # If user isn't pulling at all (or letting go), provide only
            # minimal hold — don't actively retract
            if user_pull_velocity < 5.0:
                output = max(0.0, min(output, self.power_level * 0.3))
                # Slowly bleed integral to avoid windup while idle
                self.integral_error *= 0.95

            # NEVER cross zero — only resist (retract direction), never feed out
            output = max(0.0, output)

            return self.motor_sign * output

        return 0

    def _position_percent(self, angle):
        """Current position as 0-100% of the calibrated range."""
        if self.angle_range < 1:
            return 0
        if self.max_angle > self.min_angle:
            pct = (angle - self.min_angle) / self.angle_range * 100
        else:
            pct = (self.min_angle - angle) / self.angle_range * 100
        return max(0, min(100, pct))

    def _past_min(self, angle):
        """Check if angle has gone past the minimum bound."""
        margin = 5  # degrees of safety margin
        if self.max_angle > self.min_angle:
            return angle < (self.min_angle - margin)
        else:
            return angle > (self.min_angle + margin)

    def _past_max(self, angle):
        """Check if angle has gone past the maximum bound."""
        margin = 5
        if self.max_angle > self.min_angle:
            return angle > (self.max_angle + margin)
        else:
            return angle < (self.max_angle - margin)

    def _apply_boundary_safety(self, current_angle, offset):
        """
        Prevent the motor from driving toward a boundary it's close to.
        Only allow motor force that moves AWAY from a nearby boundary.
        """
        pct = self._position_percent(current_angle)

        # Near min end (< 5%) — don't allow motor to push further toward min
        if pct < 5:
            # motor_sign * positive offset = toward min
            # So if offset has same sign as motor_sign, it's pushing toward min
            if offset * self.motor_sign > 0:
                return 0  # Block motor from pushing toward min

        # Near max end (> 95%) — don't allow motor to push toward max
        if pct > 95:
            # motor_sign * negative offset = toward max
            if offset * self.motor_sign < 0:
                return 0

        return offset

    @property
    def is_calibrated(self):
        return self.cal_state == CalibrationState.CALIBRATED

    @property
    def status_text(self):
        """Human-readable status."""
        if self.cal_state == CalibrationState.UNCALIBRATED:
            return "Press 'Set Min' at cable rest position"
        elif self.cal_state == CalibrationState.WAITING_FOR_MAX:
            return "Pull cable to max, then press 'Set Max'"
        elif self.cal_state == CalibrationState.DETECTING_DIRECTION:
            return "Detecting motor direction..."
        elif self.cal_state == CalibrationState.CALIBRATED:
            mode_info = self.mode
            if self.mode == WorkoutMode.CONSTANT_SPEED:
                mode_info = f"{self.mode} ({self.target_speed:.0f}°/s)"
            return f"Calibrated: {self.min_angle:.0f}° — {self.max_angle:.0f}° | Mode: {mode_info}"
        return ""
