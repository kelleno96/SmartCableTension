"""
SPARK MAX Motor GUI
Communicates with CAN_SparkMAX_Bridge running on Arduino.

Dependencies (already in venv): pyserial, matplotlib, numpy
Run: python motor_gui.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue
import struct
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


# ── Main GUI ──────────────────────────────────────────────────────────────────

PLOT_WINDOW = 10.0      # seconds of history shown in plot
MAX_LOG_LINES = 500
LOG_FILE = os.path.join(os.path.dirname(__file__), "logs", "motor_gui_session.log")

class MotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SPARK MAX Motor GUI")
        self.root.resizable(True, True)

        self.ser = None
        self.reader = None
        self.rx_queue = queue.Queue()

        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
        self._logfile = open(LOG_FILE, "a", buffering=1)  # line-buffered
        self._logfile.write(f"\n{'='*60}\n")
        self._logfile.write(f"SESSION START  {datetime.datetime.now().isoformat()}\n")
        self._logfile.write(f"{'='*60}\n")

        # Rolling data for plots
        self.t0 = None
        maxpts = 2000
        self.times_pos   = collections.deque(maxlen=maxpts)
        self.data_pos    = collections.deque(maxlen=maxpts)   # Status 2 (relative)
        self.times_abs   = collections.deque(maxlen=maxpts)
        self.data_abs    = collections.deque(maxlen=maxpts)   # Status 5 (absolute)
        self.times_vel   = collections.deque(maxlen=maxpts)
        self.data_vel    = collections.deque(maxlen=maxpts)   # Status 1 (RPM)

        self._build_ui()
        self._refresh_ports()
        self._poll()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")

        # Port row
        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=18, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="↺", width=2, command=self._refresh_ports).grid(row=0, column=2)
        self.conn_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=3, padx=6)
        self.status_lbl = ttk.Label(top, text="Disconnected", foreground="gray")
        self.status_lbl.grid(row=0, column=4, padx=6)

        # Device + HB row
        ttk.Label(top, text="Device ID:").grid(row=1, column=0, sticky="w", pady=(4,0))
        self.dev_var = tk.IntVar(value=1)
        ttk.Spinbox(top, from_=1, to=62, textvariable=self.dev_var, width=5).grid(row=1, column=1, sticky="w", padx=4)

        self.hb_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Heartbeat", variable=self.hb_var,
                        command=self._send_hb_toggle).grid(row=1, column=2, columnspan=2, sticky="w")
        ttk.Button(top, text="PING", command=self._send_ping).grid(row=1, column=4, padx=6)

        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=4)

        # Control notebook
        nb = ttk.Notebook(self.root)
        nb.pack(fill="x", padx=6)

        self._build_voltage_tab(nb)
        self._build_velocity_tab(nb)
        self._build_position_tab(nb)

        # STOP button
        stop_btn = tk.Button(self.root, text="⏹  STOP", font=("", 14, "bold"),
                             bg="#c0392b", fg="white", activebackground="#e74c3c",
                             command=self._send_stop)
        stop_btn.pack(fill="x", padx=6, pady=6)

        # Live readback row
        rb = ttk.LabelFrame(self.root, text="Live Readback", padding=4)
        rb.pack(fill="x", padx=6, pady=(0,4))
        ttk.Label(rb, text="Position (rot):").grid(row=0, column=0, sticky="w")
        self.pos_lbl = ttk.Label(rb, text="—", width=12, anchor="e")
        self.pos_lbl.grid(row=0, column=1, sticky="e")
        ttk.Label(rb, text="  Analog Pos (0-1):").grid(row=0, column=2, sticky="w", padx=(12,0))
        self.abspos_lbl = ttk.Label(rb, text="—", width=12, anchor="e")
        self.abspos_lbl.grid(row=0, column=3, sticky="e")
        ttk.Label(rb, text="  Velocity (RPM):").grid(row=0, column=4, sticky="w", padx=(12,0))
        self.vel_lbl = ttk.Label(rb, text="—", width=12, anchor="e")
        self.vel_lbl.grid(row=0, column=5, sticky="e")
        ttk.Label(rb, text="  Output:").grid(row=0, column=6, sticky="w", padx=(12,0))
        self.output_lbl = ttk.Label(rb, text="—", width=8, anchor="e")
        self.output_lbl.grid(row=0, column=7, sticky="e")

        # Plot
        plot_frame = ttk.LabelFrame(self.root, text="Live Plot", padding=4)
        plot_frame.pack(fill="both", expand=True, padx=6, pady=(0,4))

        fig = Figure(figsize=(7, 3), dpi=96, tight_layout=True)
        self.ax_pos = fig.add_subplot(2, 1, 1)
        self.ax_vel = fig.add_subplot(2, 1, 2)
        self.ax_pos.set_ylabel("Position (rot)")
        self.ax_vel.set_ylabel("Velocity (RPM)")
        self.ax_vel.set_xlabel("Time (s)")
        self.line_pos,  = self.ax_pos.plot([], [], color="#2980b9",  label="relative")
        self.line_abs,  = self.ax_pos.plot([], [], color="#27ae60",  label="absolute", linestyle="--")
        self.line_vel,  = self.ax_vel.plot([], [], color="#e67e22")
        self.ax_pos.legend(fontsize=7, loc="upper left")
        self.canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Serial log
        log_frame = ttk.LabelFrame(self.root, text="Serial Log", padding=4)
        log_frame.pack(fill="both", expand=False, padx=6, pady=(0,6))
        self.log = scrolledtext.ScrolledText(log_frame, height=7, font=("Courier", 9),
                                             state="disabled", wrap="none")
        self.log.pack(fill="both", expand=True)

        # Rate config row (collapsed by default)
        rate_frame = ttk.LabelFrame(self.root, text="Status Frame Rates (ms, 0=default)", padding=4)
        rate_frame.pack(fill="x", padx=6, pady=(0,6))
        labels = ["S0", "S1\n(vel)", "S2\n(pos)", "S3\n(analog)", "S4\n(alt enc)", "S5\n(abs enc)", "S6\n(abs vel)"]
        self.rate_vars = []
        for i, lbl in enumerate(labels):
            ttk.Label(rate_frame, text=lbl, justify="center").grid(row=0, column=i*2, padx=2)
            v = tk.StringVar(value="")
            e = ttk.Entry(rate_frame, textvariable=v, width=5)
            e.grid(row=1, column=i*2, padx=2)
            self.rate_vars.append(v)
            if i < len(labels)-1:
                ttk.Separator(rate_frame, orient="vertical").grid(row=0, column=i*2+1, rowspan=2, sticky="ns", padx=2)
        ttk.Button(rate_frame, text="Apply Rates", command=self._send_rates).grid(
            row=0, column=len(labels)*2, rowspan=2, padx=8)

    def _build_voltage_tab(self, nb):
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="Voltage")
        ttk.Label(f, text="Voltage (V):").grid(row=0, column=0, sticky="w")
        self.volt_var = tk.DoubleVar(value=0.0)
        sl = ttk.Scale(f, from_=-12, to=12, orient="horizontal",
                       variable=self.volt_var, length=260,
                       command=lambda _: self._update_volt_entry())
        sl.grid(row=0, column=1, padx=6)
        self.volt_entry = ttk.Entry(f, width=8)
        self.volt_entry.insert(0, "0.0")
        self.volt_entry.grid(row=0, column=2)
        self.volt_entry.bind("<Return>", lambda _: self._entry_to_slider(self.volt_entry, self.volt_var, -12, 12))
        ttk.Button(f, text="Send", command=self._send_volts).grid(row=0, column=3, padx=6)

    def _build_velocity_tab(self, nb):
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="Velocity (closed-loop)")
        ttk.Label(f, text="RPM:").grid(row=0, column=0, sticky="w")
        self.vel_sp_var = tk.DoubleVar(value=0.0)
        self.vel_max_var = tk.DoubleVar(value=5000.0)
        sl = ttk.Scale(f, from_=-5000, to=5000, orient="horizontal",
                       variable=self.vel_sp_var, length=260,
                       command=lambda _: self._update_vel_entry())
        sl.grid(row=0, column=1, padx=6)
        self.vel_entry = ttk.Entry(f, width=8)
        self.vel_entry.insert(0, "0.0")
        self.vel_entry.grid(row=0, column=2)
        self.vel_entry.bind("<Return>", lambda _: self._entry_to_slider(self.vel_entry, self.vel_sp_var, -5000, 5000))
        ttk.Button(f, text="Send", command=self._send_vel).grid(row=0, column=3, padx=6)
        ttk.Label(f, text="PID slot:").grid(row=0, column=4, padx=(12,2))
        self.vel_slot_var = tk.IntVar(value=1)
        ttk.Spinbox(f, from_=0, to=3, textvariable=self.vel_slot_var, width=3).grid(row=0, column=5)
        ttk.Label(f, text="(Requires PID gains configured in REV Hardware Client)", foreground="gray").grid(
            row=1, column=0, columnspan=6, sticky="w", pady=(4,0))

    def _build_position_tab(self, nb):
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="Position (closed-loop)")
        ttk.Label(f, text="Rotations:").grid(row=0, column=0, sticky="w")
        self.pos_sp_var = tk.DoubleVar(value=0.0)
        sl = ttk.Scale(f, from_=-20, to=20, orient="horizontal",
                       variable=self.pos_sp_var, length=260,
                       command=lambda _: self._update_pos_entry())
        sl.grid(row=0, column=1, padx=6)
        self.pos_entry = ttk.Entry(f, width=8)
        self.pos_entry.insert(0, "0.0")
        self.pos_entry.grid(row=0, column=2)
        self.pos_entry.bind("<Return>", lambda _: self._entry_to_slider(self.pos_entry, self.pos_sp_var, -20, 20))
        ttk.Button(f, text="Send", command=self._send_pos).grid(row=0, column=3, padx=6)
        ttk.Label(f, text="PID slot:").grid(row=0, column=4, padx=(12,2))
        self.pos_slot_var = tk.IntVar(value=0)
        ttk.Spinbox(f, from_=0, to=3, textvariable=self.pos_slot_var, width=3).grid(row=0, column=5)
        ttk.Label(f, text="(Requires PID gains configured in REV Hardware Client)", foreground="gray").grid(
            row=1, column=0, columnspan=6, sticky="w", pady=(4,0))

    # ── Slider / entry sync ───────────────────────────────────────────────────

    def _update_volt_entry(self):
        self.volt_entry.delete(0, "end")
        self.volt_entry.insert(0, f"{self.volt_var.get():.2f}")

    def _update_vel_entry(self):
        self.vel_entry.delete(0, "end")
        self.vel_entry.insert(0, f"{self.vel_sp_var.get():.1f}")

    def _update_pos_entry(self):
        self.pos_entry.delete(0, "end")
        self.pos_entry.insert(0, f"{self.pos_sp_var.get():.3f}")

    def _entry_to_slider(self, entry, var, lo, hi):
        try:
            v = float(entry.get())
            v = max(lo, min(hi, v))
            var.set(v)
        except ValueError:
            pass

    # ── Serial connect ────────────────────────────────────────────────────────

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
            self.status_lbl.config(text=f"Connected: {port}", foreground="#27ae60")
            self._log(f"[GUI] Connected to {port}")
        except Exception as e:
            self._log(f"[GUI] Connect error: {e}")

    def _disconnect(self):
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
        self.status_lbl.config(text="Disconnected", foreground="gray")
        self._log("[GUI] Disconnected")

    # ── Send helpers ──────────────────────────────────────────────────────────

    def _send(self, line):
        if not self.ser or not self.ser.is_open:
            self._log("[GUI] Not connected")
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
            self._log(f">> {line}")
        except Exception as e:
            self._log(f"[GUI] Write error: {e}")

    def _send_stop(self):
        self._send(f"STOP {self.dev_var.get()}")

    def _send_volts(self):
        self._entry_to_slider(self.volt_entry, self.volt_var, -12, 12)
        self._send(f"VOLTS {self.dev_var.get()} {self.volt_var.get():.3f}")

    def _send_vel(self):
        self._entry_to_slider(self.vel_entry, self.vel_sp_var, -5000, 5000)
        self._send(f"VEL {self.dev_var.get()} {self.vel_sp_var.get():.2f} {self.vel_slot_var.get()}")

    def _send_pos(self):
        self._entry_to_slider(self.pos_entry, self.pos_sp_var, -20, 20)
        self._send(f"POS {self.dev_var.get()} {self.pos_sp_var.get():.4f} {self.pos_slot_var.get()}")

    def _send_hb_toggle(self):
        en = 1 if self.hb_var.get() else 0
        self._send(f"HB {self.dev_var.get()} {en}")

    def _send_ping(self):
        self._send("PING")

    def _send_rates(self):
        dev = self.dev_var.get()
        for idx, v in enumerate(self.rate_vars):
            s = v.get().strip()
            if s:
                try:
                    ms = int(s)
                    self._send(f"RATE {dev} {idx} {ms}")
                except ValueError:
                    pass

    # ── RX parsing ────────────────────────────────────────────────────────────

    def _parse_rx(self, line):
        """Called on main thread for each decoded status line from Arduino."""
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        parts = line.split()
        if len(parts) >= 3 and parts[0] == "POS":
            try:
                val = float(parts[2])
                self.times_pos.append(t)
                self.data_pos.append(val)
                self.pos_lbl.config(text=f"{val:.4f}")
            except ValueError:
                pass
        elif len(parts) >= 3 and parts[0] == "ABSPOS":
            try:
                val = float(parts[2])
                self.times_abs.append(t)
                self.data_abs.append(val)
                self.abspos_lbl.config(text=f"{val:.5f}")
            except ValueError:
                pass
        elif len(parts) >= 3 and parts[0] == "VEL":
            try:
                val = float(parts[2])
                self.times_vel.append(t)
                self.data_vel.append(val)
                self.vel_lbl.config(text=f"{val:.1f}")
            except ValueError:
                pass
        elif len(parts) >= 3 and parts[0] == "OUTPUT":
            try:
                val = float(parts[2])
                self.output_lbl.config(text=f"{val:.3f}")
            except ValueError:
                pass

    # ── Poll loop ─────────────────────────────────────────────────────────────

    _plot_counter = 0

    def _poll(self):
        # Drain queue
        changed = False
        for _ in range(200):
            try:
                line = self.rx_queue.get_nowait()
                self._log(f"<< {line}")
                self._parse_rx(line)
                changed = True
            except queue.Empty:
                break

        # Redraw plot every ~10 polls (~100ms)
        self._plot_counter += 1
        if changed and self._plot_counter >= 5:
            self._plot_counter = 0
            self._update_plot()

        self.root.after(20, self._poll)

    def _update_plot(self):
        now = time.monotonic()
        t_cur = (now - self.t0) if self.t0 else 0

        def trim(times, data):
            cutoff = t_cur - PLOT_WINDOW
            t = list(times)
            d = list(data)
            idx = next((i for i, v in enumerate(t) if v >= cutoff), 0)
            return t[idx:], d[idx:]

        tp, dp = trim(self.times_pos, self.data_pos)
        ta, da = trim(self.times_abs, self.data_abs)
        tv, dv = trim(self.times_vel, self.data_vel)

        self.line_pos.set_data(tp, dp)
        self.line_abs.set_data(ta, da)
        self.line_vel.set_data(tv, dv)

        for ax in (self.ax_pos, self.ax_vel):
            ax.set_xlim(max(0, t_cur - PLOT_WINDOW), max(PLOT_WINDOW, t_cur))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.canvas.draw_idle()

    # ── Log ───────────────────────────────────────────────────────────────────

    def _log(self, text):
        # Write to file with timestamp
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._logfile.write(f"{ts}  {text}\n")

        # Write to on-screen widget
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > MAX_LOG_LINES:
            self.log.delete("1.0", f"{lines - MAX_LOG_LINES}.0")
        self.log.see("end")
        self.log.config(state="disabled")

    def on_close(self):
        self._disconnect()
        self._logfile.write(f"SESSION END    {datetime.datetime.now().isoformat()}\n")
        self._logfile.close()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
