"""
Smart Cable Tension — Workout GUI

Uses the CAN_SparkMAX_Bridge Arduino to read encoder position and send
velocity commands directly, replacing the old camera-based angle tracker.

Workflow:
  1. Connect to the Arduino serial port.
  2. Move cable to rest position → press Set Min.
  3. Move cable to full extension → press Set Max.
  4. Set workout velocity (RPM) and press Start.
  5. Motor runs at that velocity; auto-stops when a limit is hit.
  6. STOP button (or Space/Escape) stops immediately at any time.

Run:  python3 workout_gui.py
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import queue
import time
import collections
import datetime
import os
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── Serial reader thread ──────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    def __init__(self, ser, rx_queue):
        super().__init__(daemon=True)
        self.ser = ser
        self.rx_queue = rx_queue
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    self.rx_queue.put(line)
            except Exception:
                break


# ── Constants ─────────────────────────────────────────────────────────────────

DEVICE_ID    = 1
VEL_SLOT     = 1          # PID slot for velocity control
POLL_MS      = 20         # GUI poll interval
PLOT_WINDOW  = 30.0       # seconds of history in position plot
MAX_LOG_LINES = 300
LOG_FILE = os.path.join(os.path.dirname(__file__), "logs", "workout_session.log")

BG       = '#1a1a1a'
BG2      = '#252525'
GREEN    = '#00e676'
YELLOW   = '#ffea00'
RED      = '#ff1744'
BLUE     = '#2979ff'
GRAY     = '#607d8b'
FG       = '#e0e0e0'


# ── Main app ──────────────────────────────────────────────────────────────────

class WorkoutApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Smart Cable Tension — Workout")
        self.root.configure(bg=BG)
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Serial state
        self.ser    = None
        self.reader = None
        self.rx_queue = queue.Queue()

        # Motor telemetry
        self.pos     = 0.0   # rotations (relative, from Status 2)
        self.vel_rpm = 0.0   # RPM (from Status 2)
        self.abspos  = 0.0   # 0-1 per revolution (analog encoder, Status 3)
        self.output  = 0.0   # applied output -1..1

        # Calibration limits
        self.pos_min  = None   # rotations at rest position
        self.pos_max  = None   # rotations at full extension

        # Workout state
        self.running      = False   # motor currently commanded to run
        self.target_rpm   = 0.0
        self.stop_reason  = ""

        # Plot data
        self.t0       = None
        maxpts        = 3000
        self.t_pos    = collections.deque(maxlen=maxpts)
        self.d_pos    = collections.deque(maxlen=maxpts)
        self.t_vel    = collections.deque(maxlen=maxpts)
        self.d_vel    = collections.deque(maxlen=maxpts)

        # Logging
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
        self._logfile = open(LOG_FILE, "a", buffering=1)
        self._logfile.write(f"\n{'='*60}\n")
        self._logfile.write(f"SESSION START  {datetime.datetime.now().isoformat()}\n")
        self._logfile.write(f"{'='*60}\n")

        self._build_ui()
        self._refresh_ports()
        self._poll()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = self.root

        # ── Connection bar ────────────────────────────────────────────────────
        conn = tk.Frame(root, bg=BG, pady=4)
        conn.pack(fill="x", padx=8)

        tk.Label(conn, text="Port:", bg=BG, fg=FG, font=("Helvetica", 11)).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(conn, text="↺", width=2, command=self._refresh_ports).pack(side="left")
        self.conn_btn = tk.Button(conn, text="Connect", command=self._toggle_connect,
                                  bg=BLUE, fg="white", font=("Helvetica", 11, "bold"),
                                  relief="flat", padx=10)
        self.conn_btn.pack(side="left", padx=8)
        self.conn_lbl = tk.Label(conn, text="Disconnected", bg=BG, fg=GRAY,
                                  font=("Helvetica", 11))
        self.conn_lbl.pack(side="left")

        tk.Frame(root, bg='#333', height=1).pack(fill="x")

        # ── Live readback bar ─────────────────────────────────────────────────
        rb = tk.Frame(root, bg=BG2, pady=6)
        rb.pack(fill="x", padx=0)

        def _stat(parent, label):
            f = tk.Frame(parent, bg=BG2)
            f.pack(side="left", padx=18)
            tk.Label(f, text=label, bg=BG2, fg=GRAY, font=("Helvetica", 10)).pack()
            val = tk.Label(f, text="—", bg=BG2, fg=GREEN,
                           font=("Helvetica", 22, "bold"), width=9, anchor="center")
            val.pack()
            return val

        self.lbl_pos    = _stat(rb, "Position (rot)")
        self.lbl_vel    = _stat(rb, "Velocity (RPM)")
        self.lbl_abspos = _stat(rb, "Abs Pos (0-1)")
        self.lbl_output = _stat(rb, "Output")

        # ── Position limit bar ────────────────────────────────────────────────
        lim = tk.Frame(root, bg=BG, pady=4)
        lim.pack(fill="x", padx=8)

        tk.Label(lim, text="Min:", bg=BG, fg=GRAY, font=("Helvetica", 11)).pack(side="left")
        self.lbl_min = tk.Label(lim, text="not set", bg=BG, fg=YELLOW,
                                 font=("Helvetica", 12, "bold"), width=10)
        self.lbl_min.pack(side="left", padx=(2, 16))

        tk.Label(lim, text="Max:", bg=BG, fg=GRAY, font=("Helvetica", 11)).pack(side="left")
        self.lbl_max = tk.Label(lim, text="not set", bg=BG, fg=YELLOW,
                                 font=("Helvetica", 12, "bold"), width=10)
        self.lbl_max.pack(side="left", padx=(2, 16))

        # Position fraction bar (visual)
        self.pos_canvas = tk.Canvas(lim, height=20, bg='#333', highlightthickness=0)
        self.pos_canvas.pack(side="left", fill="x", expand=True, padx=(8, 0))
        self.pos_canvas.bind("<Configure>", lambda _: self._update_pos_bar())

        # ── Calibration buttons ───────────────────────────────────────────────
        cal = tk.Frame(root, bg=BG, pady=4)
        cal.pack(fill="x", padx=8)

        btn_style = dict(font=("Helvetica", 12, "bold"), relief="flat",
                         padx=12, pady=6, cursor="hand2")

        tk.Button(cal, text="Set Min  (rest position)",
                  command=self._set_min, bg='#1565c0', fg="white",
                  **btn_style).pack(side="left", padx=(0, 6))

        tk.Button(cal, text="Set Max  (full extension)",
                  command=self._set_max, bg='#e65100', fg="white",
                  **btn_style).pack(side="left", padx=(0, 6))

        tk.Button(cal, text="Clear Limits",
                  command=self._clear_limits, bg=GRAY, fg="white",
                  **btn_style).pack(side="left")

        # ── Velocity control ──────────────────────────────────────────────────
        vel_frame = tk.Frame(root, bg=BG, pady=6)
        vel_frame.pack(fill="x", padx=8)

        tk.Label(vel_frame, text="Workout Velocity (RPM)", bg=BG, fg=FG,
                 font=("Helvetica", 12, "bold")).pack(anchor="w")

        slider_row = tk.Frame(vel_frame, bg=BG)
        slider_row.pack(fill="x")

        tk.Button(slider_row, text="−100", command=lambda: self._step_rpm(-100),
                  bg=BG2, fg=FG, font=("Helvetica", 11), relief="flat",
                  padx=8, pady=4).pack(side="left")
        tk.Button(slider_row, text="−10", command=lambda: self._step_rpm(-10),
                  bg=BG2, fg=FG, font=("Helvetica", 11), relief="flat",
                  padx=8, pady=4).pack(side="left", padx=(2, 0))

        self.rpm_var = tk.DoubleVar(value=0.0)
        self.rpm_slider = tk.Scale(
            slider_row, from_=-3000, to=3000, orient="horizontal",
            variable=self.rpm_var, resolution=10,
            bg=BG, fg=GREEN, troughcolor='#333', highlightthickness=0,
            font=("Helvetica", 11, "bold"), length=400,
            command=self._on_rpm_slider)
        self.rpm_slider.pack(side="left", padx=6, fill="x", expand=True)

        tk.Button(slider_row, text="+10", command=lambda: self._step_rpm(10),
                  bg=BG2, fg=FG, font=("Helvetica", 11), relief="flat",
                  padx=8, pady=4).pack(side="left", padx=(0, 2))
        tk.Button(slider_row, text="+100", command=lambda: self._step_rpm(100),
                  bg=BG2, fg=FG, font=("Helvetica", 11), relief="flat",
                  padx=8, pady=4).pack(side="left")

        entry_row = tk.Frame(vel_frame, bg=BG)
        entry_row.pack(anchor="w", pady=(2, 0))
        tk.Label(entry_row, text="RPM:", bg=BG, fg=GRAY,
                 font=("Helvetica", 11)).pack(side="left")
        self.rpm_entry = tk.Entry(entry_row, width=8, font=("Helvetica", 11),
                                   bg=BG2, fg=GREEN, insertbackground=GREEN)
        self.rpm_entry.insert(0, "0")
        self.rpm_entry.pack(side="left", padx=4)
        self.rpm_entry.bind("<Return>", self._on_rpm_entry)

        tk.Button(entry_row, text="Start", command=self._cmd_start,
                  bg='#2e7d32', fg="white", font=("Helvetica", 12, "bold"),
                  relief="flat", padx=16, pady=4).pack(side="left", padx=(12, 0))

        # ── Status bar ────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_lbl = tk.Label(root, textvariable=self.status_var,
                                    bg=BG2, fg=GREEN, font=("Helvetica", 13, "bold"),
                                    anchor="center", pady=6)
        self.status_lbl.pack(fill="x")

        # ── STOP button ───────────────────────────────────────────────────────
        self.stop_btn = tk.Button(root, text="⏹  STOP",
                                   command=self._cmd_stop,
                                   bg=RED, fg="white", activebackground='#ff6d00',
                                   font=("Helvetica", 28, "bold"),
                                   relief="flat", pady=14, cursor="hand2")
        self.stop_btn.pack(fill="x", padx=8, pady=6)

        # ── Live plot ─────────────────────────────────────────────────────────
        plot_frame = tk.LabelFrame(root, text="Live Position & Velocity",
                                    bg=BG, fg=GRAY, font=("Helvetica", 10))
        plot_frame.pack(fill="both", expand=True, padx=8, pady=(0, 4))

        fig = Figure(figsize=(8, 2.8), dpi=96, facecolor=BG, tight_layout=True)
        self.ax_pos = fig.add_subplot(2, 1, 1, facecolor=BG2)
        self.ax_vel = fig.add_subplot(2, 1, 2, facecolor=BG2)
        for ax in (self.ax_pos, self.ax_vel):
            ax.tick_params(colors=GRAY, labelsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor('#444')
        self.ax_pos.set_ylabel("Position (rot)", color=GRAY, fontsize=8)
        self.ax_vel.set_ylabel("Velocity (RPM)", color=GRAY, fontsize=8)
        self.ax_vel.set_xlabel("Time (s)", color=GRAY, fontsize=8)
        self.line_pos, = self.ax_pos.plot([], [], color=GREEN, linewidth=1.5)
        self.line_vel, = self.ax_vel.plot([], [], color=YELLOW, linewidth=1.5)
        # Horizontal limit lines
        self.hline_min = self.ax_pos.axhline(y=0, color=BLUE,  linestyle='--', linewidth=1, visible=False)
        self.hline_max = self.ax_pos.axhline(y=0, color='#ff9100', linestyle='--', linewidth=1, visible=False)

        self.canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        self.canvas.get_tk_widget().configure(bg=BG)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ── Serial log ────────────────────────────────────────────────────────
        from tkinter import scrolledtext
        log_frame = tk.LabelFrame(root, text="Log", bg=BG, fg=GRAY,
                                   font=("Helvetica", 10))
        log_frame.pack(fill="x", padx=8, pady=(0, 6))
        self.log_widget = scrolledtext.ScrolledText(
            log_frame, height=4, font=("Courier", 9),
            bg='#111', fg='#888', state="disabled", wrap="none")
        self.log_widget.pack(fill="both", expand=True)

        # ── Keyboard shortcuts ────────────────────────────────────────────────
        root.bind("<space>",  lambda _: self._cmd_stop())
        root.bind("<Escape>", lambda _: self._cmd_stop())
        root.bind("<Up>",     lambda _: self._step_rpm(10))
        root.bind("<Down>",   lambda _: self._step_rpm(-10))
        root.bind("m",        lambda _: self._set_min())
        root.bind("x",        lambda _: self._set_max())

    # ── Serial ────────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            return
        try:
            self.ser = serial.Serial(port, 500000, timeout=0.1)
            self.t0 = None
            self.reader = SerialReader(self.ser, self.rx_queue)
            self.reader.start()
            self.conn_btn.config(text="Disconnect")
            self.conn_lbl.config(text=f"Connected: {port}", fg=GREEN)
            self._log(f"[Connected to {port}]")
        except Exception as e:
            self._log(f"[Connect error: {e}]")

    def _disconnect(self):
        self._cmd_stop()
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.conn_btn.config(text="Connect")
        self.conn_lbl.config(text="Disconnected", fg=GRAY)
        self._log("[Disconnected]")

    def _send(self, cmd):
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
        except Exception as e:
            self._log(f"[Write error: {e}]")

    # ── Commands ──────────────────────────────────────────────────────────────

    def _cmd_stop(self):
        self.running = False
        self._send(f"STOP {DEVICE_ID}")
        self.status_var.set("STOPPED")
        self.status_lbl.config(fg=RED)

    def _cmd_start(self):
        if not self.ser or not self.ser.is_open:
            self._log("[Not connected]")
            return
        if self.pos_min is None or self.pos_max is None:
            self.status_var.set("Set Min AND Max before starting")
            self.status_lbl.config(fg=RED)
            self._log("[Blocked: limits not set]")
            return
        rpm = self.rpm_var.get()
        if rpm == 0.0:
            self._cmd_stop()
            return
        self.target_rpm = rpm
        self.running = True
        self._send(f"VEL {DEVICE_ID} {rpm:.1f} {VEL_SLOT}")
        self.status_var.set(f"Running  {rpm:+.0f} RPM")
        self.status_lbl.config(fg=GREEN)
        self._log(f"Start  {rpm:+.0f} RPM")

    def _on_rpm_slider(self, _=None):
        v = self.rpm_var.get()
        self.rpm_entry.delete(0, "end")
        self.rpm_entry.insert(0, f"{v:.0f}")

    def _on_rpm_entry(self, _=None):
        try:
            v = float(self.rpm_entry.get())
            v = max(-3000, min(3000, v))
            self.rpm_var.set(v)
        except ValueError:
            pass

    def _step_rpm(self, delta):
        v = max(-3000, min(3000, self.rpm_var.get() + delta))
        self.rpm_var.set(v)
        self._on_rpm_slider()

    # ── Calibration ───────────────────────────────────────────────────────────

    def _set_min(self):
        self.pos_min = self.pos
        self.lbl_min.config(text=f"{self.pos_min:+.3f}")
        self._update_limit_lines()
        self._log(f"Min set: {self.pos_min:+.4f} rot")

    def _set_max(self):
        self.pos_max = self.pos
        self.lbl_max.config(text=f"{self.pos_max:+.3f}")
        self._update_limit_lines()
        self._log(f"Max set: {self.pos_max:+.4f} rot")

    def _clear_limits(self):
        self.pos_min = None
        self.pos_max = None
        self.lbl_min.config(text="not set")
        self.lbl_max.config(text="not set")
        self.hline_min.set_visible(False)
        self.hline_max.set_visible(False)
        self._log("Limits cleared")

    def _update_limit_lines(self):
        if self.pos_min is not None:
            self.hline_min.set_ydata([self.pos_min, self.pos_min])
            self.hline_min.set_visible(True)
        if self.pos_max is not None:
            self.hline_max.set_ydata([self.pos_max, self.pos_max])
            self.hline_max.set_visible(True)

    def _update_pos_bar(self):
        """Draw a horizontal position bar between min and max."""
        c = self.pos_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 4:
            return
        c.delete("all")
        c.create_rectangle(0, 0, w, h, fill='#333', outline='')
        if self.pos_min is None or self.pos_max is None:
            c.create_text(w // 2, h // 2, text="set min and max to calibrate",
                          fill=GRAY, font=("Helvetica", 9))
            return
        span = self.pos_max - self.pos_min
        if abs(span) < 1e-6:
            return
        frac = (self.pos - self.pos_min) / span
        frac = max(0.0, min(1.0, frac))
        # Background track
        c.create_rectangle(2, 4, w - 2, h - 4, fill='#444', outline='')
        # Fill
        bar_color = GREEN if 0.05 <= frac <= 0.95 else RED
        c.create_rectangle(2, 4, 2 + int((w - 4) * frac), h - 4,
                            fill=bar_color, outline='')
        # Min / max markers
        c.create_line(2, 0, 2, h, fill=BLUE, width=2)
        c.create_line(w - 2, 0, w - 2, h, fill='#ff9100', width=2)
        c.create_text(w // 2, h // 2,
                      text=f"{frac * 100:.0f}%  ({self.pos:+.3f} rot)",
                      fill="white", font=("Helvetica", 9, "bold"))

    # ── RX parsing ────────────────────────────────────────────────────────────

    def _parse_rx(self, line):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        parts = line.split()
        if len(parts) >= 3 and parts[0] == "POS":
            try:
                self.pos = float(parts[2])
                self.t_pos.append(t)
                self.d_pos.append(self.pos)
                self.lbl_pos.config(text=f"{self.pos:+.3f}")
                self._update_pos_bar()
                self._check_limits()
            except ValueError:
                pass
        elif len(parts) >= 3 and parts[0] == "VEL":
            try:
                self.vel_rpm = float(parts[2])
                self.t_vel.append(t)
                self.d_vel.append(self.vel_rpm)
                self.lbl_vel.config(text=f"{self.vel_rpm:+.0f}")
            except ValueError:
                pass
        elif len(parts) >= 3 and parts[0] == "ABSPOS":
            try:
                self.abspos = float(parts[2])
                self.lbl_abspos.config(text=f"{self.abspos:.4f}")
            except ValueError:
                pass
        elif len(parts) >= 3 and parts[0] == "OUTPUT":
            try:
                self.output = float(parts[2])
                self.lbl_output.config(text=f"{self.output:+.3f}")
            except ValueError:
                pass

    def _check_limits(self):
        """Auto-stop if position is outside calibrated limits."""
        if not self.running:
            return
        if self.pos_min is None or self.pos_max is None:
            return
        lo = min(self.pos_min, self.pos_max)
        hi = max(self.pos_min, self.pos_max)
        margin = abs(hi - lo) * 0.02  # 2% grace margin
        if self.pos < lo - margin:
            self._cmd_stop()
            self.status_var.set("Stopped — hit MIN limit")
            self._log("Auto-stop: hit MIN limit")
        elif self.pos > hi + margin:
            self._cmd_stop()
            self.status_var.set("Stopped — hit MAX limit")
            self._log("Auto-stop: hit MAX limit")

    # ── Poll loop ─────────────────────────────────────────────────────────────

    _plot_tick = 0

    def _poll(self):
        updated = False
        for _ in range(300):
            try:
                line = self.rx_queue.get_nowait()
                self._logfile.write(
                    f"{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}  << {line}\n")
                self._parse_rx(line)
                updated = True
            except queue.Empty:
                break

        self._plot_tick += 1
        if updated and self._plot_tick >= 5:
            self._plot_tick = 0
            self._update_plot()

        self.root.after(POLL_MS, self._poll)

    def _update_plot(self):
        now = time.monotonic()
        t_cur = (now - self.t0) if self.t0 else 0
        cutoff = t_cur - PLOT_WINDOW

        def trim(ts, ds):
            tl, dl = list(ts), list(ds)
            i = next((j for j, v in enumerate(tl) if v >= cutoff), 0)
            return tl[i:], dl[i:]

        tp, dp = trim(self.t_pos, self.d_pos)
        tv, dv = trim(self.t_vel, self.d_vel)

        self.line_pos.set_data(tp, dp)
        self.line_vel.set_data(tv, dv)

        for ax in (self.ax_pos, self.ax_vel):
            ax.set_xlim(max(0, t_cur - PLOT_WINDOW), max(PLOT_WINDOW, t_cur))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.canvas.draw_idle()

    # ── Log widget ────────────────────────────────────────────────────────────

    def _log(self, text):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self._logfile.write(f"{ts}  {text}\n")
        self.log_widget.config(state="normal")
        self.log_widget.insert("end", f"{ts}  {text}\n")
        lines = int(self.log_widget.index("end-1c").split(".")[0])
        if lines > MAX_LOG_LINES:
            self.log_widget.delete("1.0", f"{lines - MAX_LOG_LINES}.0")
        self.log_widget.see("end")
        self.log_widget.config(state="disabled")

    # ── Close ─────────────────────────────────────────────────────────────────

    def _on_close(self):
        self._disconnect()
        self._logfile.write(f"SESSION END    {datetime.datetime.now().isoformat()}\n")
        self._logfile.close()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkoutApp(root)
    root.mainloop()
