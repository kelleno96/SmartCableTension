"""
Smart Cable Tension — Workout GUI

Tkinter-based GUI that integrates:
  - Live camera feed with orange tape detection overlay
  - Real-time cumulative angle + motor power graph
  - Workout mode buttons (Constant Speed, Rubber Band, Constant Weight)
  - Calibration buttons (Set Min, Set Max)
  - Power level +/- controls
  - BIG emergency stop button

Run:  python3 workout_gui.py
"""

import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image, ImageTk
import time
import threading

from camera import Camera
from detector import OrangeDetector
from angle_tracker import AngleTracker
from motor_controller import MotorController
from workout_controller import WorkoutController, WorkoutMode, CalibrationState
from live_plotter import LivePlotter
from logger import Logger
from settings import load_settings, save_settings


class WorkoutApp:
    UPDATE_INTERVAL_MS = 16  # ~60fps GUI update target

    def __init__(self):
        # ---- Load saved settings ----
        self.settings = load_settings()

        # ---- Init modules ----
        self.logger = Logger()
        self.logger.log_event("Workout GUI starting")

        self.cam = Camera(index=0, width=320, height=240, fps=60)
        self.detector = OrangeDetector()
        self.tracker = AngleTracker()
        self.plotter = LivePlotter(width=640, height=250, max_points=600)

        self.motor = MotorController(port='/dev/cu.usbserial-10', baudrate=115200)
        self.motor.connect()

        # Apply saved settings to detector
        self.detector.hsv_lower[0] = self.settings['hsv_h_low']
        self.detector.hsv_upper[0] = self.settings['hsv_h_high']
        self.detector.hsv_lower[1] = self.settings['hsv_s_low']
        self.detector.hsv_upper[1] = self.settings['hsv_s_high']
        self.detector.hsv_lower[2] = self.settings['hsv_v_low']
        self.detector.hsv_upper[2] = self.settings['hsv_v_high']
        self.detector.center = (self.settings['center_x'], self.settings['center_y'])
        self.detector.mask_radius = self.settings.get('mask_radius', 100)

        max_off = self.settings.get('max_offset', 50)
        self.workout = WorkoutController(self.motor, self.logger, max_offset=max_off)
        self.workout.power_level = float(self.settings.get('power_level', 5.0))
        self.workout.target_speed = float(self.settings.get('target_speed', 90.0))
        self.workout.pi_kp = float(self.settings.get('pi_kp', 0.15))
        self.workout.pi_ki = float(self.settings.get('pi_ki', 0.03))
        self.workout._velocity_alpha = float(self.settings.get('velocity_alpha', 0.3))

        # ---- State ----
        self.running = True
        self.current_angle = 0.0
        self.last_update_time = time.time()
        self.frame_count = 0
        self.fps = 0
        self.fps_timer = time.time()
        self.log_frame_counter = 0

        # ---- Build GUI ----
        self.root = tk.Tk()
        self.root.title("Smart Cable Tension — Workout")
        self.root.configure(bg='#1e1e1e')
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._build_gui()

        # ---- Start update loop ----
        self.root.after(self.UPDATE_INTERVAL_MS, self.update_loop)

    def _build_gui(self):
        """Build the tkinter GUI layout."""
        root = self.root

        # ---- Style ----
        style = ttk.Style()
        style.theme_use('default')
        style.configure('TFrame', background='#1e1e1e')
        style.configure('TLabel', background='#1e1e1e', foreground='#33ff66',
                        font=('Helvetica', 13))
        style.configure('Header.TLabel', background='#1e1e1e', foreground='#55ff88',
                        font=('Helvetica', 15, 'bold'))
        style.configure('Status.TLabel', background='#2a2a2a', foreground='#00ff88',
                        font=('Helvetica', 14, 'bold'))
        style.configure('Info.TLabel', background='#1e1e1e', foreground='#00ff88',
                        font=('Helvetica', 14, 'bold'))
        style.configure('TButton', font=('Helvetica', 12))
        style.configure('TScale', background='#1e1e1e')
        style.configure('TLabelframe', background='#1e1e1e')
        style.configure('TLabelframe.Label', background='#1e1e1e', foreground='#33ff66',
                        font=('Helvetica', 12, 'bold'))

        # ---- Top: Status bar ----
        status_frame = ttk.Frame(root)
        status_frame.pack(fill=tk.X, padx=5, pady=(5, 0))

        self.status_label = ttk.Label(status_frame, text="Initializing...",
                                       style='Status.TLabel', anchor='center')
        self.status_label.pack(fill=tk.X, ipady=4)

        # ---- Middle: Camera + Graph side by side ----
        mid_frame = ttk.Frame(root)
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Camera panel (left)
        cam_frame = ttk.Frame(mid_frame)
        cam_frame.pack(side=tk.LEFT, padx=(0, 3))

        ttk.Label(cam_frame, text="Camera", style='Header.TLabel').pack()
        self.cam_label = ttk.Label(cam_frame)
        self.cam_label.pack()

        # Graph panel (right)
        graph_frame = ttk.Frame(mid_frame)
        graph_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(3, 0))

        ttk.Label(graph_frame, text="Angle / Motor Graph", style='Header.TLabel').pack()
        self.graph_label = ttk.Label(graph_frame)
        self.graph_label.pack()

        # ---- Info row ----
        info_frame = ttk.Frame(root)
        info_frame.pack(fill=tk.X, padx=5, pady=(0, 3))

        self.angle_var = tk.StringVar(value="Angle: --")
        self.velocity_var = tk.StringVar(value="Vel: --")
        self.motor_var = tk.StringVar(value="Motor: 0")
        self.power_info_var = tk.StringVar(value=f"Power: {self.workout.power_level:.2f}")
        self.fps_var = tk.StringVar(value="FPS: --")

        for var in [self.angle_var, self.velocity_var, self.motor_var, self.power_info_var, self.fps_var]:
            ttk.Label(info_frame, textvariable=var, style='Info.TLabel', width=16).pack(side=tk.LEFT, padx=5)

        # ---- Bottom: Controls ----
        ctrl_frame = ttk.Frame(root)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        # Row 1: Calibration
        cal_frame = ttk.LabelFrame(ctrl_frame, text="Calibration", padding=5)
        cal_frame.pack(fill=tk.X, pady=(0, 3))

        self.btn_set_min = tk.Button(cal_frame, text="Set Min (Rest Pos)",
                                      command=self.on_set_min,
                                      bg='#2196F3', fg='black', font=('Helvetica', 11),
                                      width=18, height=1)
        self.btn_set_min.pack(side=tk.LEFT, padx=3)

        self.btn_set_max = tk.Button(cal_frame, text="Set Max (Extended)",
                                      command=self.on_set_max,
                                      bg='#FF9800', fg='black', font=('Helvetica', 11),
                                      width=18, height=1)
        self.btn_set_max.pack(side=tk.LEFT, padx=3)

        self.btn_reset_angle = tk.Button(cal_frame, text="Reset Angle",
                                         command=self.on_reset_angle,
                                         bg='#607D8B', fg='black', font=('Helvetica', 11),
                                         width=12, height=1)
        self.btn_reset_angle.pack(side=tk.LEFT, padx=3)

        # Row 2: Workout Modes
        mode_frame = ttk.LabelFrame(ctrl_frame, text="Workout Mode", padding=5)
        mode_frame.pack(fill=tk.X, pady=(0, 3))

        modes = [
            ("Idle (Off)", WorkoutMode.IDLE, '#607D8B', 'black'),
            ("Constant Speed", WorkoutMode.CONSTANT_SPEED, '#4CAF50', 'black'),
            ("Rubber Band", WorkoutMode.RUBBER_BAND, '#9C27B0', 'black'),
            ("Constant Weight", WorkoutMode.CONSTANT_WEIGHT, '#FF5722', 'black'),
        ]
        self.mode_buttons = {}
        for label, mode, color, fgcolor in modes:
            btn = tk.Button(mode_frame, text=label,
                           command=lambda m=mode: self.on_set_mode(m),
                           bg=color, fg=fgcolor, font=('Helvetica', 11),
                           width=15, height=1)
            btn.pack(side=tk.LEFT, padx=3)
            self.mode_buttons[mode] = btn

        # Row 3: Power Slider + Target Speed + Retract + Stop
        power_frame = ttk.Frame(ctrl_frame)
        power_frame.pack(fill=tk.X, pady=(0, 0))

        # Power slider  −15 … 0 … +15, resolution 0.02
        power_left = ttk.LabelFrame(power_frame, text="Power Level", padding=5)
        power_left.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))

        self.power_var = tk.DoubleVar(value=self.workout.power_level)
        self.power_slider = tk.Scale(
            power_left, from_=-15, to=15, orient=tk.HORIZONTAL,
            variable=self.power_var, resolution=0.02,
            bg='#2a2a2a', fg='#00ff88', highlightthickness=0,
            troughcolor='#444444', length=300,
            font=('Helvetica', 12, 'bold'),
            command=self._on_power_slider_change)
        self.power_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Target Speed (deg/sec) for constant speed mode
        speed_left = ttk.LabelFrame(power_frame, text="Target Speed (°/s)", padding=5)
        speed_left.pack(side=tk.LEFT, padx=(0, 5))

        self.btn_speed_down = tk.Button(speed_left, text="−",
                                         command=self.on_target_speed_down,
                                         bg='#455A64', fg='black',
                                         font=('Helvetica', 12, 'bold'),
                                         width=3, height=1)
        self.btn_speed_down.pack(side=tk.LEFT, padx=2)

        init_speed = f"{self.workout.target_speed:.0f}"
        self.speed_display = tk.Label(speed_left, text=init_speed,
                                       bg='#1e1e1e', fg='#00ff88',
                                       font=('Helvetica', 20, 'bold'), width=5)
        self.speed_display.pack(side=tk.LEFT, padx=3)

        self.btn_speed_up = tk.Button(speed_left, text="+",
                                       command=self.on_target_speed_up,
                                       bg='#455A64', fg='black',
                                       font=('Helvetica', 12, 'bold'),
                                       width=3, height=1)
        self.btn_speed_up.pack(side=tk.LEFT, padx=2)

        # Retract button
        self.btn_retract = tk.Button(power_frame, text="Retract",
                                      command=self.on_retract,
                                      bg='#00897B', fg='black',
                                      font=('Helvetica', 12, 'bold'),
                                      width=8, height=2)
        self.btn_retract.pack(side=tk.RIGHT, padx=3)

        # BIG STOP BUTTON
        self.btn_stop = tk.Button(power_frame, text="⬛  STOP  ⬛",
                                   command=self.on_emergency_stop,
                                   bg='#d32f2f', fg='black', activebackground='#f44336',
                                   font=('Helvetica', 20, 'bold'),
                                   width=14, height=2)
        self.btn_stop.pack(side=tk.RIGHT, padx=3)

        # ---- HSV / Center Tuning Panel ----
        tuning_frame = ttk.LabelFrame(ctrl_frame, text="Detection Tuning (HSV + Center)", padding=5)
        tuning_frame.pack(fill=tk.X, pady=(3, 0))

        # HSV sliders — row 1: H, row 2: S, row 3: V, row 4: Center X/Y
        slider_font = ('Helvetica', 11)
        self.hsv_vars = {}
        hsv_defs = [
            ("H Low", 0, 180, self.detector.hsv_lower[0]),
            ("H High", 0, 180, self.detector.hsv_upper[0]),
            ("S Low", 0, 255, self.detector.hsv_lower[1]),
            ("S High", 0, 255, self.detector.hsv_upper[1]),
            ("V Low", 0, 255, self.detector.hsv_lower[2]),
            ("V High", 0, 255, self.detector.hsv_upper[2]),
        ]
        row_frame_1 = ttk.Frame(tuning_frame)
        row_frame_1.pack(fill=tk.X)
        row_frame_2 = ttk.Frame(tuning_frame)
        row_frame_2.pack(fill=tk.X)

        for i, (name, lo, hi, default) in enumerate(hsv_defs):
            parent = row_frame_1 if i < 3 else row_frame_2
            f = ttk.Frame(parent)
            f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            var = tk.IntVar(value=default)
            self.hsv_vars[name] = var
            tk.Label(f, text=name, bg='#1e1e1e', fg='#33ff66', font=slider_font,
                     width=6, anchor='e').pack(side=tk.LEFT)
            tk.Scale(f, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=var,
                     bg='#2a2a2a', fg='#33ff66', highlightthickness=0,
                     troughcolor='#444444', length=120,
                     command=lambda v, n=name: self._on_hsv_change(n, v)).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Center X / Y
        center_frame = ttk.Frame(tuning_frame)
        center_frame.pack(fill=tk.X, pady=(3, 0))

        cx, cy = self.detector.center
        self.center_x_var = tk.IntVar(value=cx)
        self.center_y_var = tk.IntVar(value=cy)

        for label_text, var, hi in [("Center X", self.center_x_var, 320), ("Center Y", self.center_y_var, 240)]:
            f = ttk.Frame(center_frame)
            f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            tk.Label(f, text=label_text, bg='#1e1e1e', fg='#33ff66', font=slider_font,
                     width=8, anchor='e').pack(side=tk.LEFT)
            tk.Scale(f, from_=0, to=hi, orient=tk.HORIZONTAL, variable=var,
                     bg='#2a2a2a', fg='#33ff66', highlightthickness=0,
                     troughcolor='#444444', length=180,
                     command=lambda v: self._on_center_change()).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # ---- PI Gains Tuning Panel ----
        pi_frame = ttk.LabelFrame(ctrl_frame, text="Constant Speed PI Gains", padding=5)
        pi_frame.pack(fill=tk.X, pady=(3, 0))

        pi_row = ttk.Frame(pi_frame)
        pi_row.pack(fill=tk.X)

        # Kp slider: 0.00 – 1.00
        f_kp = ttk.Frame(pi_row)
        f_kp.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_kp, text="Kp", bg='#1e1e1e', fg='#33ff66', font=slider_font,
                 width=4, anchor='e').pack(side=tk.LEFT)
        self.kp_var = tk.DoubleVar(value=self.workout.pi_kp)
        tk.Scale(f_kp, from_=0, to=1.0, orient=tk.HORIZONTAL,
                 variable=self.kp_var, resolution=0.005,
                 bg='#2a2a2a', fg='#33ff66', highlightthickness=0,
                 troughcolor='#444444', length=200,
                 command=lambda v: self._on_pi_change()).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Ki slider: 0.00 – 0.50
        f_ki = ttk.Frame(pi_row)
        f_ki.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_ki, text="Ki", bg='#1e1e1e', fg='#33ff66', font=slider_font,
                 width=4, anchor='e').pack(side=tk.LEFT)
        self.ki_var = tk.DoubleVar(value=self.workout.pi_ki)
        tk.Scale(f_ki, from_=0, to=0.5, orient=tk.HORIZONTAL,
                 variable=self.ki_var, resolution=0.005,
                 bg='#2a2a2a', fg='#33ff66', highlightthickness=0,
                 troughcolor='#444444', length=200,
                 command=lambda v: self._on_pi_change()).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Velocity smoothing slider: 0.05 – 1.00
        f_alpha = ttk.Frame(pi_row)
        f_alpha.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_alpha, text="Vel α", bg='#1e1e1e', fg='#33ff66', font=slider_font,
                 width=5, anchor='e').pack(side=tk.LEFT)
        self.alpha_var = tk.DoubleVar(value=self.workout._velocity_alpha)
        tk.Scale(f_alpha, from_=0.05, to=1.0, orient=tk.HORIZONTAL,
                 variable=self.alpha_var, resolution=0.05,
                 bg='#2a2a2a', fg='#33ff66', highlightthickness=0,
                 troughcolor='#444444', length=200,
                 command=lambda v: self._on_pi_change()).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # ---- Keyboard shortcuts ----
        root.bind('<space>', lambda e: self.on_emergency_stop())
        root.bind('<Escape>', lambda e: self.on_emergency_stop())
        root.bind('q', lambda e: self.on_close())
        root.bind('r', lambda e: self.on_reset_angle())
        root.bind('m', lambda e: self.on_set_min())
        root.bind('t', lambda e: self.on_set_max())
        root.bind('<Up>', lambda e: self.on_power_up())
        root.bind('<Down>', lambda e: self.on_power_down())
        root.bind('<Right>', lambda e: self.on_target_speed_up())
        root.bind('<Left>', lambda e: self.on_target_speed_down())
        root.bind('1', lambda e: self.on_set_mode(WorkoutMode.IDLE))
        root.bind('2', lambda e: self.on_set_mode(WorkoutMode.CONSTANT_SPEED))
        root.bind('3', lambda e: self.on_set_mode(WorkoutMode.RUBBER_BAND))
        root.bind('4', lambda e: self.on_set_mode(WorkoutMode.CONSTANT_WEIGHT))
        root.bind('5', lambda e: self.on_retract())

    # ---- Event handlers ----

    def _on_hsv_change(self, name, value):
        """Update detector HSV thresholds from sliders."""
        mapping = {
            "H Low": (0, 'lower'), "H High": (0, 'upper'),
            "S Low": (1, 'lower'), "S High": (1, 'upper'),
            "V Low": (2, 'lower'), "V High": (2, 'upper'),
        }
        idx, which = mapping[name]
        if which == 'lower':
            self.detector.hsv_lower[idx] = int(value)
        else:
            self.detector.hsv_upper[idx] = int(value)
        self._save_settings()

    def _on_center_change(self):
        """Update detector center from sliders."""
        self.detector.center = (self.center_x_var.get(), self.center_y_var.get())
        self._save_settings()

    def _on_pi_change(self):
        """Update PI gains from sliders."""
        self.workout.pi_kp = self.kp_var.get()
        self.workout.pi_ki = self.ki_var.get()
        self.workout._velocity_alpha = self.alpha_var.get()
        self._save_settings()

    def on_set_min(self):
        self.tracker.reset()
        self.plotter.reset()
        self.current_angle = 0.0
        msg = self.workout.set_min_position(0.0)
        self.logger.log_event(f"User set min position: {msg}")

    def on_set_max(self):
        msg = self.workout.set_max_position(self.current_angle)
        self.logger.log_event(f"User set max position: {msg}")

    def on_reset_angle(self):
        self.tracker.reset()
        self.plotter.reset()
        self.current_angle = 0.0
        self.logger.log_event("Angle reset to zero")

    def on_set_mode(self, mode):
        msg = self.workout.set_mode(mode)
        self.logger.log_event(f"Mode set: {msg}")
        # Update button highlighting
        for m, btn in self.mode_buttons.items():
            if m == mode:
                btn.config(relief=tk.SUNKEN, bd=3)
            else:
                btn.config(relief=tk.RAISED, bd=2)

    def _on_power_slider_change(self, value):
        """Update power level from slider."""
        self.workout.power_level = float(value)
        self._save_settings()

    def on_power_up(self):
        new = min(15.0, self.workout.power_level + 0.02)
        self.workout.power_level = round(new, 2)
        self.power_var.set(self.workout.power_level)
        self._save_settings()

    def on_power_down(self):
        new = max(-15.0, self.workout.power_level - 0.02)
        self.workout.power_level = round(new, 2)
        self.power_var.set(self.workout.power_level)
        self._save_settings()

    def on_retract(self):
        """Manually retract cable back to min position at power 20."""
        msg = self.workout.start_manual_retract(power=20)
        self.logger.log_event(f"Manual retract: {msg}")

    def on_target_speed_up(self):
        spd = self.workout.increase_target_speed()
        self.speed_display.config(text=f"{spd:.0f}")
        self._save_settings()

    def on_target_speed_down(self):
        spd = self.workout.decrease_target_speed()
        self.speed_display.config(text=f"{spd:.0f}")
        self._save_settings()

    def on_emergency_stop(self):
        self.workout.emergency_stop()
        self.logger.log_safety("USER EMERGENCY STOP")

    def on_close(self):
        self._save_settings()
        self.running = False
        self.workout.emergency_stop()
        self.motor.disconnect()
        self.cam.release()
        self.logger.close()
        self.root.destroy()

    def _save_settings(self):
        """Gather current settings and persist to disk."""
        cx, cy = self.detector.center
        self.settings.update({
            'hsv_h_low': int(self.detector.hsv_lower[0]),
            'hsv_h_high': int(self.detector.hsv_upper[0]),
            'hsv_s_low': int(self.detector.hsv_lower[1]),
            'hsv_s_high': int(self.detector.hsv_upper[1]),
            'hsv_v_low': int(self.detector.hsv_lower[2]),
            'hsv_v_high': int(self.detector.hsv_upper[2]),
            'center_x': cx,
            'center_y': cy,
            'mask_radius': self.detector.mask_radius,
            'power_level': self.workout.power_level,
            'target_speed': self.workout.target_speed,
            'max_offset': self.workout.max_offset,
            'pi_kp': self.workout.pi_kp,
            'pi_ki': self.workout.pi_ki,
            'velocity_alpha': self.workout._velocity_alpha,
        })
        save_settings(self.settings)

    # ---- Main update loop ----

    def update_loop(self):
        if not self.running:
            return

        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        # ---- Capture frame ----
        ret, frame = self.cam.read()
        if ret:
            # ---- Detect orange tape ----
            result = self.detector.detect(frame)

            # ---- Update angle ----
            self.current_angle = self.tracker.update(result['angle_deg'])

            # ---- Workout controller update ----
            ctrl_info = self.workout.update(self.current_angle, dt)

            # ---- Draw camera overlay ----
            display = frame.copy()
            self.detector.draw_overlay(display, result)

            # Add angle + motor info to camera view with background boxes
            motor_offset = round((self.motor.current_speed - self.motor.NEUTRAL) / self.motor.UNIT)
            h_disp = display.shape[0]
            cv2.rectangle(display, (0, h_disp - 42), (220, h_disp), (0, 0, 0), -1)
            cv2.putText(display, f"Cumulative: {self.current_angle:.1f} deg",
                        (6, h_disp - 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 50), 2)
            cv2.putText(display, f"Motor: {motor_offset:+d}",
                        (6, h_disp - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 120, 30), 2)

            # Scale camera image up 2x for readability in the GUI
            display_big = cv2.resize(display, (display.shape[1] * 2, display.shape[0] * 2),
                                     interpolation=cv2.INTER_NEAREST)

            # Convert to tkinter image
            display_rgb = cv2.cvtColor(display_big, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(display_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.cam_label.imgtk = imgtk
            self.cam_label.configure(image=imgtk)

            # ---- Update plotter ----
            self.plotter.update(self.current_angle, motor_offset)
            plot_img = self.plotter.render()
            plot_rgb = cv2.cvtColor(plot_img, cv2.COLOR_BGR2RGB)
            plot_pil = Image.fromarray(plot_rgb)
            plot_tk = ImageTk.PhotoImage(image=plot_pil)
            self.graph_label.imgtk = plot_tk
            self.graph_label.configure(image=plot_tk)

            # ---- Update info labels ----
            self.angle_var.set(f"Angle: {self.current_angle:.1f}°")
            vel = self.workout.angular_velocity
            self.velocity_var.set(f"Vel: {vel:.0f}°/s")
            self.motor_var.set(f"Motor: {motor_offset:+d}")
            self.power_info_var.set(f"Power: {self.workout.power_level:.2f}")

            # FPS
            self.frame_count += 1
            if now - self.fps_timer > 0.5:
                self.fps = self.frame_count / (now - self.fps_timer)
                self.frame_count = 0
                self.fps_timer = now
            self.fps_var.set(f"FPS: {self.fps:.0f}")

            # ---- Status bar ----
            self.status_label.config(text=self.workout.status_text)

            # ---- Periodic logging ----
            self.log_frame_counter += 1
            if self.log_frame_counter % 30 == 0:  # Log every ~30 frames
                raw = result['angle_deg'] if result['angle_deg'] is not None else 0
                self.logger.log_angle(raw, self.current_angle)

        # Schedule next update
        self.root.after(self.UPDATE_INTERVAL_MS, self.update_loop)

    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()


def main():
    app = WorkoutApp()
    app.run()


if __name__ == "__main__":
    main()
