"""
Encoder-driven workout GUI for the Smart Cable Tension project.

Flow:
  1. Connect to the CAN bridge.
  2. Set the cable rest position as MIN.
  3. Pull to full extension and set MAX.
  4. Use Retract To Min to return home with position control.
  5. Start the workout with a velocity setpoint in the retract direction.

Safety rules:
  - Motor output is disabled on startup, connect, disconnect, and close.
  - If position goes below MIN or above MAX, output is disabled immediately.
  - Reaching the top of the stroke ends the rep and allows retracting for the next rep.

Run: python workout_gui.py
"""

import collections
import datetime
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import scrolledtext, ttk

import matplotlib
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

matplotlib.use("TkAgg")


DEVICE_ID = 1
BAUDRATE = 500000
POSITION_SLOT = 0
VELOCITY_SLOT = 1
POLL_MS = 20
PLOT_WINDOW = 25.0
MAX_LOG_LINES = 400
STATUS_RATES = ((0, 50), (2, 20), (3, 50))

BG = "#f4f1ea"
CARD = "#ffffff"
ALT = "#e9e3d6"
TEXT = "#1e1d1a"
MUTED = "#645f52"
ACCENT = "#1c6e8c"
GREEN = "#2d7d46"
RED = "#b42318"
ORANGE = "#b35c1e"
BLUE = "#205a8d"
PLOT_GRID = "#ddd5c7"


class SerialReader(threading.Thread):
    def __init__(self, ser, rx_queue):
        super().__init__(daemon=True)
        self.ser = ser
        self.rx_queue = rx_queue
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode("ascii", errors="replace").strip()
            except Exception:
                break
            if line:
                self.rx_queue.put(line)


class SessionLogger:
    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"workout_gui_{stamp}.log")
        self._handle = open(self.path, "a", buffering=1)
        self.write("SESSION", "start")

    def write(self, category, message):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self._handle.write(f"{timestamp} [{category}] {message}\n")

    def close(self):
        self.write("SESSION", "end")
        self._handle.close()


class WorkoutApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Smart Cable Tension Workout")
        self.root.configure(bg=BG)
        self.root.geometry("1320x920")
        self.root.minsize(1180, 820)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.logger = SessionLogger(os.path.join(os.path.dirname(__file__), "logs"))

        self.ser = None
        self.reader = None
        self.rx_queue = queue.Queue()
        self.connected_port = None

        self.position = 0.0
        self.velocity_rpm = 0.0
        self.abs_position = 0.0
        self.output = 0.0
        self.current = 0.0

        self.min_position = None
        self.max_position = None
        self.stroke_direction = None
        self.recommended_rpm = None
        self.measured_pull_rpm = None
        self.pull_duration_10_90 = None
        self.calibration_samples = []

        self.phase = "disconnected"
        self.last_disable_reason = ""
        self.last_disable_time = 0.0

        self.t0 = None
        max_points = 4000
        self.time_position = collections.deque(maxlen=max_points)
        self.data_position = collections.deque(maxlen=max_points)
        self.time_velocity = collections.deque(maxlen=max_points)
        self.data_velocity = collections.deque(maxlen=max_points)
        self.time_current = collections.deque(maxlen=max_points)
        self.data_current = collections.deque(maxlen=max_points)

        self.plot_counter = 0
        self.last_snapshot_time = 0.0

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Disconnected")
        self.phase_var = tk.StringVar(value="Disconnected")
        self.guidance_var = tk.StringVar(value="Connect to the bridge. Motor output stays disabled until you start a workout.")
        self.speed_var = tk.StringVar(value="0")
        self.suggested_speed_var = tk.StringVar(value="Not available yet")
        self.range_var = tk.StringVar(value="Stroke range: --")
        self.pull_stats_var = tk.StringVar(value="Calibration pull: --")

        self._build_ui()
        self._refresh_ports()
        self._set_phase(
            "disconnected",
            "Disconnected",
            "Connect to the bridge. Motor output stays disabled until you start a workout.",
            MUTED,
        )
        self._poll()

    def _build_ui(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TCombobox", fieldbackground=CARD, background=CARD)

        outer = tk.Frame(self.root, bg=BG)
        outer.pack(fill="both", expand=True, padx=16, pady=16)

        title = tk.Frame(outer, bg=BG)
        title.pack(fill="x", pady=(0, 12))
        tk.Label(
            title,
            text="Smart Cable Tension Workout",
            bg=BG,
            fg=TEXT,
            font=("Helvetica", 26, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title,
            text="Encoder min/max setup, position retract, and velocity-based workout control.",
            bg=BG,
            fg=MUTED,
            font=("Helvetica", 12),
        ).pack(anchor="w", pady=(3, 0))

        top = tk.Frame(outer, bg=BG)
        top.pack(fill="x", pady=(0, 12))

        connection_card = self._card(top)
        connection_card.pack(side="left", fill="x", expand=True, padx=(0, 8))
        controls_card = self._card(top)
        controls_card.pack(side="left", fill="x", expand=True, padx=(8, 0))

        self._build_connection_card(connection_card)
        self._build_controls_card(controls_card)

        middle = tk.Frame(outer, bg=BG)
        middle.pack(fill="x", pady=(0, 12))

        telemetry_card = self._card(middle)
        telemetry_card.pack(side="left", fill="both", expand=True, padx=(0, 8))
        calibration_card = self._card(middle)
        calibration_card.pack(side="left", fill="both", expand=True, padx=(8, 0))

        self._build_telemetry_card(telemetry_card)
        self._build_calibration_card(calibration_card)

        bottom = tk.Frame(outer, bg=BG)
        bottom.pack(fill="both", expand=True)

        plot_card = self._card(bottom)
        plot_card.pack(side="left", fill="both", expand=True, padx=(0, 8))
        log_card = self._card(bottom)
        log_card.pack(side="left", fill="both", expand=True, padx=(8, 0))

        self._build_plot_card(plot_card)
        self._build_log_card(log_card)

        self.root.bind("<space>", lambda _: self._disable_output("Manual stop"))
        self.root.bind("<Escape>", lambda _: self._disable_output("Manual stop"))
        self.root.bind("m", lambda _: self._set_min())
        self.root.bind("x", lambda _: self._set_max())
        self.root.bind("r", lambda _: self._retract_to_min())
        self.root.bind("s", lambda _: self._start_workout())

    def _card(self, parent):
        return tk.Frame(parent, bg=CARD, highlightbackground=ALT, highlightthickness=1)

    def _section_title(self, parent, text):
        tk.Label(parent, text=text, bg=CARD, fg=TEXT, font=("Helvetica", 16, "bold")).pack(anchor="w")

    def _build_connection_card(self, parent):
        self._section_title(parent, "Connection")
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=(12, 6))

        tk.Label(row, text="Serial Port", bg=CARD, fg=MUTED, font=("Helvetica", 11, "bold")).pack(side="left")
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, width=24, state="readonly")
        self.port_combo.pack(side="left", padx=8)

        tk.Button(
            row,
            text="Refresh",
            command=self._refresh_ports,
            bg=ALT,
            fg=TEXT,
            relief="flat",
            padx=12,
            pady=6,
            font=("Helvetica", 11, "bold"),
        ).pack(side="left", padx=(0, 8))

        self.connect_button = tk.Button(
            row,
            text="Connect",
            command=self._toggle_connect,
            bg=ACCENT,
            fg="white",
            relief="flat",
            padx=16,
            pady=6,
            font=("Helvetica", 11, "bold"),
            cursor="hand2",
        )
        self.connect_button.pack(side="left")

        tk.Label(parent, textvariable=self.connection_var, bg=CARD, fg=TEXT, font=("Helvetica", 14, "bold")).pack(anchor="w", pady=(4, 10))

        phase_box = tk.Frame(parent, bg=ALT)
        phase_box.pack(fill="x", pady=(0, 10))
        tk.Label(phase_box, text="Phase", bg=ALT, fg=MUTED, font=("Helvetica", 10, "bold")).pack(anchor="w", padx=12, pady=(10, 2))
        self.phase_label = tk.Label(phase_box, textvariable=self.phase_var, bg=ALT, fg=TEXT, font=("Helvetica", 18, "bold"))
        self.phase_label.pack(anchor="w", padx=12)
        tk.Label(
            phase_box,
            textvariable=self.guidance_var,
            bg=ALT,
            fg=TEXT,
            justify="left",
            wraplength=520,
            font=("Helvetica", 12),
        ).pack(anchor="w", padx=12, pady=(4, 12))

    def _build_controls_card(self, parent):
        self._section_title(parent, "Workout Controls")

        speed_row = tk.Frame(parent, bg=CARD)
        speed_row.pack(fill="x", pady=(12, 6))
        tk.Label(speed_row, text="Workout Speed (RPM)", bg=CARD, fg=MUTED, font=("Helvetica", 11, "bold")).pack(anchor="w")

        entry_row = tk.Frame(parent, bg=CARD)
        entry_row.pack(fill="x", pady=(6, 0))
        self.speed_entry = tk.Entry(
            entry_row,
            textvariable=self.speed_var,
            width=10,
            justify="center",
            relief="flat",
            bg=ALT,
            fg=TEXT,
            insertbackground=TEXT,
            font=("Helvetica", 22, "bold"),
        )
        self.speed_entry.pack(side="left")
        tk.Button(
            entry_row,
            text="Use Suggested",
            command=self._apply_suggested_speed,
            bg=BLUE,
            fg="white",
            relief="flat",
            padx=16,
            pady=8,
            font=("Helvetica", 11, "bold"),
            cursor="hand2",
        ).pack(side="left", padx=10)

        tk.Label(parent, textvariable=self.suggested_speed_var, bg=CARD, fg=TEXT, font=("Helvetica", 12)).pack(anchor="w", pady=(10, 0))
        tk.Label(parent, textvariable=self.pull_stats_var, bg=CARD, fg=MUTED, font=("Helvetica", 11)).pack(anchor="w", pady=(2, 0))

        button_row = tk.Frame(parent, bg=CARD)
        button_row.pack(fill="x", pady=(18, 8))

        self.set_min_button = self._action_button(button_row, "Set Min", self._set_min, BLUE)
        self.set_max_button = self._action_button(button_row, "Set Max", self._set_max, ORANGE)
        self.retract_button = self._action_button(button_row, "Retract To Min", self._retract_to_min, GREEN)
        self.start_button = self._action_button(button_row, "Start Workout", self._start_workout, ACCENT)
        self.stop_button = self._action_button(button_row, "STOP", lambda: self._disable_output("Manual stop"), RED)

        self.set_min_button.pack(side="left", padx=(0, 8))
        self.set_max_button.pack(side="left", padx=(0, 8))
        self.retract_button.pack(side="left", padx=(0, 8))
        self.start_button.pack(side="left", padx=(0, 8))
        self.stop_button.pack(side="left")

        tk.Label(
            parent,
            text="Start sends a velocity command in the retract direction. Retract uses position control to return to MIN.",
            bg=CARD,
            fg=MUTED,
            wraplength=520,
            justify="left",
            font=("Helvetica", 11),
        ).pack(anchor="w", pady=(8, 0))

    def _action_button(self, parent, text, command, color):
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=color,
            fg="white",
            relief="flat",
            padx=14,
            pady=10,
            font=("Helvetica", 11, "bold"),
            cursor="hand2",
        )

    def _build_telemetry_card(self, parent):
        self._section_title(parent, "Live Telemetry")
        grid = tk.Frame(parent, bg=CARD)
        grid.pack(fill="both", expand=True, pady=(12, 0))

        self.position_label = self._metric(grid, 0, 0, "Position (rot)", BLUE)
        self.velocity_label = self._metric(grid, 0, 1, "Velocity (RPM)", GREEN)
        self.output_label = self._metric(grid, 1, 0, "Output", ORANGE)
        self.current_label = self._metric(grid, 1, 1, "Current (A)", RED)
        self.abs_label = self._metric(grid, 2, 0, "Analog Pos", ACCENT)
        self.range_label = tk.Label(grid, textvariable=self.range_var, bg=ALT, fg=TEXT, font=("Helvetica", 15, "bold"), padx=18, pady=18, anchor="w", justify="left")
        self.range_label.grid(row=2, column=1, sticky="nsew", padx=8, pady=8)

        for row in range(3):
            grid.rowconfigure(row, weight=1)
        for column in range(2):
            grid.columnconfigure(column, weight=1)

    def _metric(self, parent, row, column, label, color):
        frame = tk.Frame(parent, bg=ALT, padx=18, pady=18)
        frame.grid(row=row, column=column, sticky="nsew", padx=8, pady=8)
        tk.Label(frame, text=label, bg=ALT, fg=MUTED, font=("Helvetica", 11, "bold")).pack(anchor="w")
        value = tk.Label(frame, text="--", bg=ALT, fg=color, font=("Helvetica", 26, "bold"))
        value.pack(anchor="w", pady=(8, 0))
        return value

    def _build_calibration_card(self, parent):
        self._section_title(parent, "Calibration")

        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=(12, 8))

        self.min_box = self._calibration_box(row, "MIN")
        self.min_box.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.max_box = self._calibration_box(row, "MAX")
        self.max_box.pack(side="left", fill="x", expand=True)

        self.position_bar = tk.Canvas(parent, height=64, bg=ALT, highlightthickness=0)
        self.position_bar.pack(fill="x", pady=(8, 8))
        self.position_bar.bind("<Configure>", lambda _: self._draw_position_bar())

        tk.Label(
            parent,
            text="Set MIN at the cable start position. Pull to full extension, set MAX, then retract back to MIN before starting.",
            bg=CARD,
            fg=MUTED,
            wraplength=520,
            justify="left",
            font=("Helvetica", 11),
        ).pack(anchor="w", pady=(0, 8))

    def _calibration_box(self, parent, label):
        frame = tk.Frame(parent, bg=ALT, padx=18, pady=18)
        tk.Label(frame, text=label, bg=ALT, fg=MUTED, font=("Helvetica", 11, "bold")).pack(anchor="w")
        value = tk.Label(frame, text="Not set", bg=ALT, fg=TEXT, font=("Helvetica", 24, "bold"))
        value.pack(anchor="w", pady=(10, 0))
        frame.value_label = value
        return frame

    def _build_plot_card(self, parent):
        self._section_title(parent, "Position, Velocity, and Current")
        figure = Figure(figsize=(8.2, 5.2), dpi=100, facecolor=CARD, tight_layout=True)
        self.ax_position = figure.add_subplot(3, 1, 1, facecolor=CARD)
        self.ax_velocity = figure.add_subplot(3, 1, 2, facecolor=CARD)
        self.ax_current = figure.add_subplot(3, 1, 3, facecolor=CARD)

        for axis in (self.ax_position, self.ax_velocity, self.ax_current):
            axis.tick_params(colors=MUTED, labelsize=9)
            axis.grid(color=PLOT_GRID, linewidth=0.8)
            for spine in axis.spines.values():
                spine.set_color(ALT)

        self.ax_position.set_ylabel("Rotations", color=MUTED)
        self.ax_velocity.set_ylabel("RPM", color=MUTED)
        self.ax_current.set_ylabel("Amps", color=MUTED)
        self.ax_current.set_xlabel("Seconds", color=MUTED)

        self.position_line, = self.ax_position.plot([], [], color=BLUE, linewidth=2.0)
        self.velocity_line, = self.ax_velocity.plot([], [], color=GREEN, linewidth=2.0)
        self.current_line, = self.ax_current.plot([], [], color=RED, linewidth=2.0)
        self.min_line = self.ax_position.axhline(0.0, color=ACCENT, linestyle="--", linewidth=1.2, visible=False)
        self.max_line = self.ax_position.axhline(0.0, color=ORANGE, linestyle="--", linewidth=1.2, visible=False)

        self.plot_canvas = FigureCanvasTkAgg(figure, master=parent)
        self.plot_canvas.get_tk_widget().pack(fill="both", expand=True, pady=(12, 0))

    def _build_log_card(self, parent):
        self._section_title(parent, "Session Log")
        self.log_widget = scrolledtext.ScrolledText(
            parent,
            height=18,
            wrap="word",
            bg="#f8f6f0",
            fg=TEXT,
            relief="flat",
            font=("Courier", 11),
            state="disabled",
        )
        self.log_widget.pack(fill="both", expand=True, pady=(12, 0))

    def _refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            self._ui_log("No serial port selected.")
            return
        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=0.1)
        except Exception as exc:
            self._ui_log(f"Connect failed: {exc}")
            self.logger.write("ERROR", f"connect failed: {exc}")
            return

        self.connected_port = port
        self.reader = SerialReader(self.ser, self.rx_queue)
        self.reader.start()
        self.connection_var.set(f"Connected: {port}")
        self.connect_button.config(text="Disconnect", bg=RED)
        self.logger.write("EVENT", f"connected to {port}")
        self._ui_log(f"Connected to {port}")

        self._send(f"HB {DEVICE_ID} 1")
        self._send(f"STOP {DEVICE_ID}")
        self._send(f"VOLTS {DEVICE_ID} 0")
        for status_idx, period_ms in STATUS_RATES:
            self._send(f"RATE {DEVICE_ID} {status_idx} {period_ms}")

        self._set_phase(
            "connected",
            "Connected - Set MIN",
            "Motor output is disabled. Move the cable to the starting position, then press Set Min.",
            BLUE,
        )
        self._update_controls()

    def _disconnect(self):
        if self.ser and self.ser.is_open:
            self._disable_output("Disconnect", update_phase=False)
            self._send(f"HB {DEVICE_ID} 0")
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.connection_var.set("Disconnected")
        self.connect_button.config(text="Connect", bg=ACCENT)
        self.connected_port = None
        self._set_phase(
            "disconnected",
            "Disconnected",
            "Connect to the bridge. Motor output stays disabled until you start a workout.",
            MUTED,
        )
        self._ui_log("Disconnected")
        self.logger.write("EVENT", "disconnected")
        self._update_controls()

    def _send(self, command):
        if not self.ser or not self.ser.is_open:
            return False
        try:
            self.ser.write((command + "\n").encode("ascii"))
            self.logger.write("TX", command)
            return True
        except Exception as exc:
            self.logger.write("ERROR", f"write failed for '{command}': {exc}")
            self._ui_log(f"Write failed: {exc}")
            return False

    def _set_phase(self, phase, title, guidance, color):
        self.phase = phase
        self.phase_var.set(title)
        self.guidance_var.set(guidance)
        self.phase_label.config(fg=color)
        self.logger.write("STATE", f"phase={phase} title='{title}' guidance='{guidance}'")

    def _ui_log(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_widget.config(state="normal")
        self.log_widget.insert("end", f"{timestamp}  {message}\n")
        line_count = int(self.log_widget.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log_widget.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")
        self.log_widget.see("end")
        self.log_widget.config(state="disabled")

    def _limit_tolerance(self):
        if self.min_position is None or self.max_position is None:
            return 0.01
        return max(0.01, abs(self.max_position - self.min_position) * 0.005)

    def _home_tolerance(self):
        return max(0.02, self._limit_tolerance() * 2.0)

    def _position_within_limits(self):
        if self.min_position is None or self.max_position is None:
            return False
        low = min(self.min_position, self.max_position)
        high = max(self.min_position, self.max_position)
        tolerance = self._limit_tolerance()
        return low - tolerance <= self.position <= high + tolerance

    def _is_near_min(self):
        if self.min_position is None:
            return False
        return abs(self.position - self.min_position) <= self._home_tolerance()

    def _set_min(self):
        if not self._ensure_connected("Set Min"):
            return
        self.min_position = self.position
        self.max_position = None
        self.stroke_direction = None
        self.recommended_rpm = None
        self.measured_pull_rpm = None
        self.pull_duration_10_90 = None
        self.calibration_samples = [(time.monotonic(), self.position)]
        self._update_calibration_labels()
        self._update_limit_lines()
        self._draw_position_bar()
        self._set_phase(
            "capturing_max",
            "MIN Set - Pull To MAX",
            "Pull the cable to full extension. The position history is being recorded so MAX can produce a suggested workout speed.",
            BLUE,
        )
        self._ui_log(f"MIN set at {self.min_position:+.4f} rotations")
        self.logger.write("CALIBRATE", f"min={self.min_position:.5f}")
        self._update_controls()

    def _set_max(self):
        if not self._ensure_connected("Set Max"):
            return
        if self.min_position is None:
            self._ui_log("Set MIN before MAX.")
            return
        self.max_position = self.position
        stroke = self.max_position - self.min_position
        if abs(stroke) < 0.02:
            self._ui_log("MAX is too close to MIN. Pull farther before setting MAX.")
            self.logger.write("CALIBRATE", f"max rejected pos={self.position:.5f} min={self.min_position:.5f}")
            self.max_position = None
            return

        self.stroke_direction = 1.0 if stroke > 0 else -1.0
        self.calibration_samples.append((time.monotonic(), self.position))
        self._compute_suggested_speed()
        self._update_calibration_labels()
        self._update_limit_lines()
        self._draw_position_bar()
        self._set_phase(
            "calibrated",
            "MAX Set - Retract To MIN",
            "Calibration is complete. Use Retract To Min to return home with position control before starting the workout.",
            GREEN,
        )
        self._ui_log(f"MAX set at {self.max_position:+.4f} rotations")
        self.logger.write("CALIBRATE", f"max={self.max_position:.5f} stroke={stroke:.5f}")
        self._update_controls()

    def _compute_suggested_speed(self):
        if self.min_position is None or self.max_position is None or len(self.calibration_samples) < 2:
            self.recommended_rpm = None
            self.suggested_speed_var.set("Not available yet")
            self.pull_stats_var.set("Calibration pull: --")
            return

        stroke = self.max_position - self.min_position
        p10 = self.min_position + 0.1 * stroke
        p90 = self.min_position + 0.9 * stroke
        t10 = self._find_crossing_time(p10)
        t90 = self._find_crossing_time(p90)

        if t10 is None or t90 is None or t90 <= t10:
            self.recommended_rpm = None
            self.measured_pull_rpm = None
            self.pull_duration_10_90 = None
            self.suggested_speed_var.set("Suggested speed unavailable: could not resolve 10%-90% pull time")
            self.pull_stats_var.set("Calibration pull: insufficient data")
            self.logger.write("CALIBRATE", "speed suggestion unavailable")
            return

        duration = t90 - t10
        measured_rpm = (abs(0.8 * stroke) / duration) * 60.0
        recommended_rpm = max(5.0, min(5000.0, measured_rpm * 2.0))

        self.pull_duration_10_90 = duration
        self.measured_pull_rpm = measured_rpm
        self.recommended_rpm = recommended_rpm
        self.speed_var.set(f"{recommended_rpm:.1f}")
        self.suggested_speed_var.set(
            f"Suggested workout speed: {recommended_rpm:.1f} RPM (2x the measured average from 10% to 90% stroke)"
        )
        self.pull_stats_var.set(
            f"Calibration pull: {duration:.2f} s from 10% to 90%, measured average {measured_rpm:.1f} RPM"
        )
        self.logger.write(
            "CALIBRATE",
            f"pull_duration_10_90={duration:.4f}s measured_avg_rpm={measured_rpm:.3f} recommended_rpm={recommended_rpm:.3f}",
        )

    def _find_crossing_time(self, target):
        for index in range(1, len(self.calibration_samples)):
            t0, p0 = self.calibration_samples[index - 1]
            t1, p1 = self.calibration_samples[index]
            if p0 == p1:
                continue
            if (p0 <= target <= p1) or (p1 <= target <= p0):
                fraction = (target - p0) / (p1 - p0)
                return t0 + fraction * (t1 - t0)
        return None

    def _apply_suggested_speed(self):
        if self.recommended_rpm is None:
            self._ui_log("No suggested speed is available yet.")
            return
        self.speed_var.set(f"{self.recommended_rpm:.1f}")
        self._ui_log(f"Applied suggested speed: {self.recommended_rpm:.1f} RPM")

    def _retract_to_min(self):
        if not self._ensure_calibrated("Retract"):
            return
        if self.min_position is None:
            return
        if self.position < min(self.min_position, self.max_position) - self._limit_tolerance():
            self._ui_log("Cannot retract while below MIN. Move back into range or recalibrate.")
            return
        self._disable_output("Preparing retract", update_phase=False)
        if self._send(f"POS {DEVICE_ID} {self.min_position:.5f} {POSITION_SLOT}"):
            self._set_phase(
                "retracting",
                "Retracting To MIN",
                "Position control is returning the cable to the MIN position. Output will be disabled as soon as MIN is reached or crossed.",
                ORANGE,
            )
            self._ui_log(f"Retracting to MIN at {self.min_position:+.4f} rotations")
            self.logger.write("COMMAND", f"retract target={self.min_position:.5f}")
            self._update_controls()

    def _start_workout(self):
        if not self._ensure_calibrated("Start"):
            return
        if not self._is_near_min():
            self._ui_log("Move back to MIN or use Retract To Min before starting.")
            return
        try:
            speed = abs(float(self.speed_var.get().strip()))
        except ValueError:
            self._ui_log("Enter a valid RPM value before starting.")
            return
        if speed <= 0.0:
            self._ui_log("Workout speed must be greater than zero.")
            return
        command_rpm = -self.stroke_direction * speed
        if self._send(f"VEL {DEVICE_ID} {command_rpm:.3f} {VELOCITY_SLOT}"):
            self._set_phase(
                "workout",
                "Workout Running",
                "User pull should move from MIN toward MAX while the motor resists in the retract direction. Output will be disabled immediately if MIN or MAX is crossed.",
                GREEN,
            )
            self._ui_log(f"Workout started at {speed:.1f} RPM magnitude, command {command_rpm:+.1f} RPM")
            self.logger.write("COMMAND", f"workout speed={speed:.3f} command_rpm={command_rpm:.3f}")
            self._update_controls()

    def _ensure_connected(self, action):
        if self.ser and self.ser.is_open:
            return True
        self._ui_log(f"{action} requires an active connection.")
        return False

    def _ensure_calibrated(self, action):
        if not self._ensure_connected(action):
            return False
        if self.min_position is None or self.max_position is None or self.stroke_direction is None:
            self._ui_log(f"{action} requires MIN and MAX to be set first.")
            return False
        return True

    def _disable_output(self, reason, update_phase=True):
        now = time.monotonic()
        repeated = reason == self.last_disable_reason and (now - self.last_disable_time) < 0.25
        self.last_disable_reason = reason
        self.last_disable_time = now

        if self.ser and self.ser.is_open:
            self._send(f"STOP {DEVICE_ID}")
            self._send(f"VOLTS {DEVICE_ID} 0")

        if not repeated:
            self._ui_log(f"Output disabled: {reason}")
            self.logger.write("SAFETY", f"output disabled: {reason}")

        if update_phase:
            self._set_phase("idle", "Output Disabled", reason, RED)
        self._update_controls()

    def _update_controls(self):
        connected = self.ser is not None and self.ser.is_open
        calibrated = self.min_position is not None and self.max_position is not None and self.stroke_direction is not None
        busy = self.phase in {"retracting", "workout"}
        start_ready = connected and calibrated and not busy and self._is_near_min() and self._position_within_limits()

        self.set_min_button.config(state="normal" if connected and not busy else "disabled")
        self.set_max_button.config(state="normal" if connected and self.min_position is not None and not busy else "disabled")
        self.retract_button.config(state="normal" if connected and calibrated and self.phase != "retracting" else "disabled")
        self.start_button.config(state="normal" if start_ready else "disabled")
        self.stop_button.config(state="normal" if connected else "disabled")

    def _update_calibration_labels(self):
        min_text = "Not set" if self.min_position is None else f"{self.min_position:+.4f} rot"
        max_text = "Not set" if self.max_position is None else f"{self.max_position:+.4f} rot"
        self.min_box.value_label.config(text=min_text)
        self.max_box.value_label.config(text=max_text)

        if self.min_position is None or self.max_position is None:
            self.range_var.set("Stroke range: --")
        else:
            stroke = self.max_position - self.min_position
            self.range_var.set(f"Stroke range: {abs(stroke):.4f} rotations\nDirection: {'positive' if stroke > 0 else 'negative'}")

    def _update_limit_lines(self):
        if self.min_position is None:
            self.min_line.set_visible(False)
        else:
            self.min_line.set_ydata([self.min_position, self.min_position])
            self.min_line.set_visible(True)
        if self.max_position is None:
            self.max_line.set_visible(False)
        else:
            self.max_line.set_ydata([self.max_position, self.max_position])
            self.max_line.set_visible(True)

    def _draw_position_bar(self):
        canvas = self.position_bar
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        if width <= 4 or height <= 4:
            return

        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill=ALT, outline="")

        if self.min_position is None or self.max_position is None:
            canvas.create_text(
                width / 2,
                height / 2,
                text="Set MIN and MAX to enable the stroke display.",
                fill=MUTED,
                font=("Helvetica", 11, "bold"),
            )
            return

        low = min(self.min_position, self.max_position)
        high = max(self.min_position, self.max_position)
        span = high - low
        if span <= 0:
            return

        fraction = (self.position - low) / span
        bar_left = 20
        bar_right = width - 20
        bar_top = 18
        bar_bottom = height - 18
        usable = bar_right - bar_left

        canvas.create_rectangle(bar_left, bar_top, bar_right, bar_bottom, fill="#d8d0c2", outline="")

        clamped = max(0.0, min(1.0, fraction))
        fill_right = bar_left + usable * clamped
        fill_color = GREEN if 0.0 <= fraction <= 1.0 else RED
        canvas.create_rectangle(bar_left, bar_top, fill_right, bar_bottom, fill=fill_color, outline="")
        canvas.create_line(bar_left, 10, bar_left, height - 10, fill=ACCENT, width=2)
        canvas.create_line(bar_right, 10, bar_right, height - 10, fill=ORANGE, width=2)
        canvas.create_text(
            width / 2,
            height / 2,
            text=f"{fraction * 100:.1f}% of stroke   Current position {self.position:+.4f} rot",
            fill=TEXT,
            font=("Helvetica", 11, "bold"),
        )

    def _handle_position_update(self, timestamp, value):
        self.position = value
        self.position_label.config(text=f"{self.position:+.4f}")
        self.time_position.append(timestamp)
        self.data_position.append(value)
        self._draw_position_bar()

        if self.phase == "capturing_max":
            self.calibration_samples.append((time.monotonic(), value))

        if self.phase == "retracting" and self.min_position is not None and abs(value - self.min_position) <= self._home_tolerance():
            self._disable_output("Retract reached MIN", update_phase=False)
            self._set_phase(
                "ready",
                "Ready To Start",
                "Retract is complete. The cable is back at MIN. Start Workout when ready.",
                GREEN,
            )
            self._ui_log("Retract complete")
            self._update_controls()
            return

        self._enforce_limits()

    def _enforce_limits(self):
        if self.min_position is None or self.max_position is None:
            self._update_controls()
            return

        tolerance = self._limit_tolerance()
        low = min(self.min_position, self.max_position)
        high = max(self.min_position, self.max_position)

        if self.position < low - tolerance:
            self._disable_output("Below MIN limit")
            if self.phase != "retracting":
                self._set_phase(
                    "limit_fault",
                    "Below MIN - Output Disabled",
                    "Motor output is off because position crossed MIN. Move back into range or recalibrate if needed.",
                    RED,
                )
            self._update_controls()
            return

        if self.position > high + tolerance:
            if self.phase == "workout":
                self._set_phase(
                    "rep_complete",
                    "Top Of Stroke Reached",
                    "The rep is complete. Use Retract To Min to return for the next rep. Upper-limit crossing does not force output off.",
                    ORANGE,
                )
                self.logger.write("EVENT", f"above max during workout pos={self.position:.5f}")
            elif self.phase != "retracting":
                self._set_phase(
                    "above_max",
                    "Above MAX - Retract Allowed",
                    "Position is above MAX. You can press Retract To Min to return for the next rep.",
                    ORANGE,
                )
                self.logger.write("EVENT", f"above max idle pos={self.position:.5f}")
            self._update_controls()
            return

        self._update_controls()

    def _parse_rx(self, line):
        self.logger.write("RX", line)
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        timestamp = now - self.t0

        parts = line.split()
        if len(parts) < 3:
            return
        if parts[1] != str(DEVICE_ID):
            return

        try:
            value = float(parts[2])
        except ValueError:
            return

        if parts[0] == "POS":
            self._handle_position_update(timestamp, value)
        elif parts[0] == "VEL":
            self.velocity_rpm = value
            self.velocity_label.config(text=f"{value:+.1f}")
            self.time_velocity.append(timestamp)
            self.data_velocity.append(value)
        elif parts[0] == "ABSPOS":
            self.abs_position = value
            self.abs_label.config(text=f"{value:.4f}")
        elif parts[0] == "OUTPUT":
            self.output = value
            self.output_label.config(text=f"{value:+.3f}")
        elif parts[0] == "CURRENT":
            self.current = value
            self.current_label.config(text=f"{value:.2f}")
            self.time_current.append(timestamp)
            self.data_current.append(value)

    def _log_snapshot(self):
        if not (self.ser and self.ser.is_open):
            return
        now = time.monotonic()
        if now - self.last_snapshot_time < 1.0:
            return
        self.last_snapshot_time = now
        self.logger.write(
            "SNAPSHOT",
            f"phase={self.phase} pos={self.position:.5f} vel={self.velocity_rpm:.3f} abs={self.abs_position:.5f} output={self.output:.4f} current={self.current:.3f}",
        )

    def _update_plot(self):
        if self.t0 is None:
            return
        now = time.monotonic()
        current_time = now - self.t0
        cutoff = current_time - PLOT_WINDOW

        def trim(times, data):
            times_list = list(times)
            data_list = list(data)
            start_index = next((idx for idx, item in enumerate(times_list) if item >= cutoff), 0)
            return times_list[start_index:], data_list[start_index:]

        tp, dp = trim(self.time_position, self.data_position)
        tv, dv = trim(self.time_velocity, self.data_velocity)
        tc, dc = trim(self.time_current, self.data_current)

        self.position_line.set_data(tp, dp)
        self.velocity_line.set_data(tv, dv)
        self.current_line.set_data(tc, dc)

        for axis in (self.ax_position, self.ax_velocity, self.ax_current):
            axis.set_xlim(max(0.0, current_time - PLOT_WINDOW), max(PLOT_WINDOW, current_time))
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        self.plot_canvas.draw_idle()

    def _poll(self):
        updated = False
        for _ in range(400):
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._parse_rx(line)
            updated = True

        self._log_snapshot()

        self.plot_counter += 1
        if updated and self.plot_counter >= 5:
            self.plot_counter = 0
            self._update_plot()

        self.root.after(POLL_MS, self._poll)

    def _on_close(self):
        try:
            self._disconnect()
        finally:
            self.logger.close()
            self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = WorkoutApp(root)
    root.mainloop()
