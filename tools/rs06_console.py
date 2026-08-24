#!/usr/bin/env python3
"""
RS06 brake actuator console.

Control the Core2 over USB, watch the result as a live time-series, and pull
the firmware's 1 kHz ring buffer into an interactive analyser window.

    python tools/rs06_console.py            # pick the port in the GUI
    python tools/rs06_console.py COM7       # or name it up front

Needs: pyserial, matplotlib, numpy, tkinter.
"""

import sys
import csv
import time
import queue
import threading
import collections
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

BAUD = 921600
STREAM_HZ = 100
HISTORY_S = 20.0
OUT_DIR = Path(__file__).resolve().parent / "logs"

# T,<t_ms>,<state>,<events>,<active>,<fault>,<enabled>,<p_cmd>,<p_act>,
#   <v_cmd>,<v_act>,<t_cmd>,<t_act>,<iq>,<vbus>,<temp>,<reach_ms>,<peak_vel>,
#   <tx>,<rx>,<miss>
#
# events is latched until the fault is cleared; active is rebuilt by the
# firmware every control tick and says what is limiting the axis right now.
# Keeping them apart is what stops a one-millisecond regen cap from sitting on
# the status line for the rest of the session.
T_FIELDS = ["t_ms", "state", "events", "active", "fault", "enabled",
            "p_cmd", "p_act", "v_cmd", "v_act", "t_cmd", "t_act", "iq",
            "vbus", "temp", "reach_ms", "peak_vel", "tx", "rx", "miss"]

# Columns of the 1 kHz dump, in firmware order (logger::csvHeader()).
D_FIELDS = ["t_us", "p_cmd", "p_act", "v_cmd", "v_act", "t_cmd", "t_act",
            "iq", "vbus", "temp", "state", "events", "active"]

# axes index, data key, label, unit, decimals
SERIES = [
    (0, "p_cmd", "angle cmd",  "deg",   2),
    (0, "p_act", "angle act",  "deg",   2),
    (1, "v_cmd", "speed cmd",  "rad/s", 3),
    (1, "v_act", "speed act",  "rad/s", 3),
    (2, "t_cmd", "torque cmd", "Nm",    3),
    (2, "t_act", "torque act", "Nm",    3),
    (3, "iq",    "current",    "A",     3),
    (3, "vbus",  "bus volt",   "V",     2),
]
AXIS_LABEL = ["angle [deg]", "speed [rad/s]", "torque [Nm]", "current / bus"]

# Mirrors EventBits in src/types.h.
EVENT_NAMES = [
    (1 << 0,  "CAN-MISS"),    (1 << 1,  "BUS-OFF"),
    (1 << 2,  "DERATE"),      (1 << 3,  "OVERTEMP"),
    (1 << 4,  "OVERVOLT"),    (1 << 5,  "UNDERVOLT"),
    (1 << 6,  "OVERCURRENT"), (1 << 7,  "ENCODER"),
    (1 << 8,  "STALL"),       (1 << 9,  "CMD-STALE"),
    (1 << 10, "POS-LIMIT"),   (1 << 11, "REGEN-LIM"),
    (1 << 12, "UNCAL"),       (1 << 13, "DRIVER-IC"),
    (1 << 14, "POS-INIT"),    (1 << 15, "HW-ID"),
    (1 << 16, "AUX-QUIET"),
]

# Mirrors FaultBits in src/rs06_proto.h - the raw Type21 word, reported so a
# cause with no event bit of its own is still identifiable from here.
FAULT_NAMES = [
    (1 << 0,  "overtemp"),      (1 << 1,  "driver-chip"),
    (1 << 2,  "undervoltage"),  (1 << 3,  "overvoltage"),
    (1 << 4,  "Ib-overcur"),    (1 << 5,  "Ic-overcur"),
    (1 << 7,  "enc-uncal"),     (1 << 8,  "hw-id"),
    (1 << 9,  "pos-init"),      (1 << 14, "stall-algo"),
    (1 << 16, "Ia-overcur"),
]


def _decode(mask, table):
    if not mask:
        return "-"
    return " ".join(n for b, n in table if mask & b) or hex(mask)


def decode_events(mask):
    return _decode(mask, EVENT_NAMES)


def decode_faults(mask):
    return _decode(mask, FAULT_NAMES)


def stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


# Column aliases, so every CSV this tool can emit - the 1 kHz capture, the SD
# card dump, the live recording and the live buffer - loads back without the
# user having to say which is which.
COL_ALIAS = {
    "p_cmd_deg": "p_cmd", "p_act_deg": "p_act",
    "angle_cmd": "p_cmd", "angle_act": "p_act",
    "speed_cmd": "v_cmd", "speed_act": "v_act",
    "torque_cmd": "t_cmd", "torque_act": "t_act",
    "current": "iq", "bus": "vbus",
}
# name -> seconds multiplier, most specific first
TIME_COLS = [("t_us", 1e-6), ("t_ms", 1e-3), ("t_s", 1.0), ("host_time", 1.0)]


def load_csv(path):
    """Read one of our CSV files back.

    Returns (t_seconds, {series key: ndarray}, raw_rows, header_line, note).
    Missing series are filled with zeros rather than refused, so a partial
    file still plots whatever it does contain.
    """
    text = Path(path).read_text(encoding="utf-8-sig", errors="replace")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    if len(lines) < 2:
        raise ValueError("file has no data rows")

    header_line = lines[0]
    names = [COL_ALIAS.get(h.strip(), h.strip()) for h in header_line.split(",")]
    if not any(n in [c for c, _ in TIME_COLS] for n in names):
        raise ValueError(f"no recognised time column in: {header_line[:60]}")

    rows = lines[1:]
    cols = {n: [] for n in names}
    bad = 0
    for ln in rows:
        f = ln.split(",")
        if len(f) != len(names):
            bad += 1
            continue
        for n, v in zip(names, f):
            try:
                cols[n].append(float(v))
            except ValueError:
                cols[n].append(float("nan"))
    if not any(cols[n] for n in cols):
        raise ValueError("no parsable rows")

    tname, scale = next((c, m) for c, m in TIME_COLS if c in names and cols[c])
    traw = np.array(cols[tname], dtype=float)
    t = (traw - traw[0]) * scale

    n = len(t)
    data = {}
    missing = []
    for _, key, _, _, _ in SERIES:
        if key in cols and len(cols[key]) == n:
            data[key] = np.array(cols[key], dtype=float)
        else:
            data[key] = np.zeros(n)
            missing.append(key)

    dt = np.median(np.diff(t)) if n > 2 else 0.0
    note = f"{n} samples, {tname}"
    if dt > 0:
        note += f", {1.0 / dt:,.0f} Hz"
    if bad:
        note += f", {bad} malformed rows skipped"
    if missing:
        note += f", missing: {' '.join(missing)}"
    return t, data, rows, header_line, note


# ===========================================================================
class Link:
    """Serial reader thread. Everything it sees lands in .rx as tagged tuples."""

    def __init__(self):
        self.ser = None
        self.rx = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self._dump = None          # collects rows while a DUMP is streaming

    def open(self, port):
        self.close()
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, line):
        if not self.is_open:
            return
        self.ser.write((line + "\n").encode("ascii", "ignore"))
        self.rx.put(("tx", line))

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:
                self.rx.put(("err", str(exc)))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._line(raw.decode("ascii", "replace").strip())

    def _line(self, line):
        if not line:
            return
        if self._dump is not None:
            if line == "DEND":
                rows, self._dump = self._dump, None
                self.rx.put(("dump", rows))
            elif line.startswith("OK") or line.startswith("ERR"):
                self.rx.put(("log", line))
            elif line.startswith("t_us"):
                pass                                   # header row
            else:
                self._dump.append(line)
            return

        if line.startswith("T,"):
            parts = line[2:].split(",")
            if len(parts) == len(T_FIELDS):
                self.rx.put(("telem", parts))
            return
        if line.startswith("D,"):
            self._dump = []
            return
        self.rx.put(("log", line))


# ===========================================================================
class PlotPane:
    """Four stacked, x-shared axes with logger-style zoom and two cursors.

    Cursor A (left click) reads every trace at one instant; cursor B (right
    click) adds a second instant and the readout switches to differences.
    Values are pinned to their sample with a leader line so it is always clear
    which curve a number belongs to.
    """

    CUR_A_COLOR = "#d62728"
    CUR_B_COLOR = "#1f77b4"

    def __init__(self, parent, on_log=None):
        self.on_log = on_log or (lambda s: None)
        self.t = np.zeros(0)
        self.data = {k: np.zeros(0) for _, k, _, _, _ in SERIES}
        self.cursor = {"A": None, "B": None}      # x positions
        self._notes = []
        self._vlines = {"A": [], "B": []}
        self.show_notes = tk.BooleanVar(value=True)

        self.frame = ttk.Frame(parent)

        bar = ttk.Frame(self.frame)
        bar.pack(fill="x")

        self.fig = Figure(figsize=(9, 7), dpi=100, layout="constrained")
        self.axes = []
        for i in range(4):
            ax = self.fig.add_subplot(4, 1, i + 1,
                                      sharex=self.axes[0] if self.axes else None)
            ax.set_ylabel(AXIS_LABEL[i], fontsize=9)
            ax.grid(alpha=.3)
            self.axes.append(ax)
        self.axes[-1].set_xlabel("time [s]")

        self.lines = {}
        for ai, key, label, unit, _ in SERIES:
            (ln,) = self.axes[ai].plot([], [], lw=1.0, label=f"{label} [{unit}]")
            self.lines[key] = ln
        for ax in self.axes:
            ax.legend(loc="upper right", fontsize=7, ncols=2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.toolbar = NavigationToolbar2Tk(self.canvas, bar, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(side="left")

        for text, fn in (("X +", lambda: self._zoom_x(0.7)),
                         ("X -", lambda: self._zoom_x(1 / 0.7)),
                         ("Y +", lambda: self._zoom_y(0.7)),
                         ("Y -", lambda: self._zoom_y(1 / 0.7)),
                         ("auto Y", self.autoscale_y),
                         ("auto XY", self.autoscale)):
            ttk.Button(bar, text=text, width=7, command=fn).pack(side="left", padx=1)
        ttk.Checkbutton(bar, text="labels", variable=self.show_notes,
                        command=self._draw_cursors).pack(side="left", padx=6)
        ttk.Button(bar, text="clear cursors", command=self.clear_cursors).pack(side="left")

        # readout ---------------------------------------------------------
        ro = ttk.Frame(self.frame)
        ro.pack(fill="x")
        self.delta_lbl = ttk.Label(
            ro, font=("Consolas", 10),
            text="left click = cursor A,  right click = cursor B,  "
                 "wheel = zoom X,  shift+wheel = zoom Y")
        self.delta_lbl.pack(anchor="w", padx=4)

        self.tree = ttk.Treeview(ro, height=9, show="headings",
                                 columns=("sig", "a", "b", "d"))
        for col, txt, w in (("sig", "signal", 150), ("a", "A", 120),
                            ("b", "B", 120), ("d", "B - A", 120)):
            self.tree.heading(col, text=txt)
            self.tree.column(col, width=w, anchor="e" if col != "sig" else "w")
        self.tree.pack(fill="x", padx=4, pady=(2, 4))
        for _, key, label, unit, _ in SERIES:
            self.tree.insert("", "end", iid=key, values=(f"{label} [{unit}]", "", "", ""))

        self.canvas.mpl_connect("button_press_event", self._on_click)
        self.canvas.mpl_connect("scroll_event", self._on_scroll)

    def pack(self, **kw):
        self.frame.pack(**kw)

    # -- data -------------------------------------------------------------
    def set_data(self, t, data):
        self.t = np.asarray(t, dtype=float)
        for key in self.data:
            self.data[key] = np.asarray(data[key], dtype=float)
            self.lines[key].set_data(self.t, self.data[key])
        self.clear_cursors()
        self.autoscale()

    # -- zoom -------------------------------------------------------------
    def _zoom_x(self, f):
        x0, x1 = self.axes[0].get_xlim()
        c = 0.5 * (x0 + x1)
        self.axes[0].set_xlim(c + (x0 - c) * f, c + (x1 - c) * f)
        self.canvas.draw_idle()

    def _zoom_y(self, f):
        for ax in self.axes:
            y0, y1 = ax.get_ylim()
            c = 0.5 * (y0 + y1)
            ax.set_ylim(c + (y0 - c) * f, c + (y1 - c) * f)
        self.canvas.draw_idle()

    def autoscale(self):
        if self.t.size:
            self.axes[0].set_xlim(self.t[0], self.t[-1])
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view(scalex=False)
        self.autoscale_y()

    def autoscale_y(self):
        """Fit Y to what is actually visible on X - the thing plain autoscale
        will not do, and the reason zoomed-in detail usually looks flat."""
        if not self.t.size:
            return
        x0, x1 = self.axes[0].get_xlim()
        m = (self.t >= x0) & (self.t <= x1)
        if not m.any():
            return
        for ai, ax in enumerate(self.axes):
            keys = [k for a, k, _, _, _ in SERIES if a == ai]
            lo = min(self.data[k][m].min() for k in keys)
            hi = max(self.data[k][m].max() for k in keys)
            if hi - lo < 1e-9:
                lo, hi = lo - 0.5, hi + 0.5
            pad = (hi - lo) * 0.08
            ax.set_ylim(lo - pad, hi + pad)
        self.canvas.draw_idle()

    def _on_scroll(self, ev):
        if ev.inaxes is None or not self.t.size:
            return
        f = 0.8 if ev.button == "up" else 1.25
        if ev.key == "shift":
            ax = ev.inaxes
            y0, y1 = ax.get_ylim()
            c = ev.ydata
            ax.set_ylim(c + (y0 - c) * f, c + (y1 - c) * f)
        else:
            x0, x1 = self.axes[0].get_xlim()
            c = ev.xdata
            self.axes[0].set_xlim(c + (x0 - c) * f, c + (x1 - c) * f)
        self.canvas.draw_idle()

    # -- cursors ----------------------------------------------------------
    def clear_cursors(self):
        self.cursor = {"A": None, "B": None}
        self._draw_cursors()

    def _on_click(self, ev):
        # Do not steal clicks while the toolbar owns the mouse.
        if self.toolbar.mode or ev.inaxes is None or not self.t.size:
            return
        if ev.button == 1:
            self.cursor["A"] = ev.xdata
        elif ev.button == 3:
            self.cursor["B"] = ev.xdata
        else:
            return
        self._draw_cursors()

    def _index(self, x):
        i = int(np.clip(np.searchsorted(self.t, x), 1, len(self.t) - 1))
        return i if abs(self.t[i] - x) < abs(self.t[i - 1] - x) else i - 1

    def _draw_cursors(self):
        for note in self._notes:
            note.remove()
        self._notes = []
        for name in ("A", "B"):
            for ln in self._vlines[name]:
                ln.remove()
            self._vlines[name] = []

        idx = {}
        for name, color in (("A", self.CUR_A_COLOR), ("B", self.CUR_B_COLOR)):
            x = self.cursor[name]
            if x is None or not self.t.size:
                continue
            i = self._index(x)
            idx[name] = i
            for ax in self.axes:
                self._vlines[name].append(
                    ax.axvline(self.t[i], color=color, ls="--", lw=1.0, alpha=.9))

        if self.show_notes.get():
            for name, color in (("A", self.CUR_A_COLOR), ("B", self.CUR_B_COLOR)):
                if name not in idx:
                    continue
                self._annotate(idx[name], color, right=(name == "A"), tag=name)

        self._fill_readout(idx)
        self.canvas.draw_idle()

    def _annotate(self, i, color, right, tag):
        """One boxed number per trace, tied to its sample by a leader line."""
        per_axis = collections.defaultdict(int)
        for ai, key, label, unit, dec in SERIES:
            y = self.data[key][i]
            slot = per_axis[ai]
            per_axis[ai] += 1
            dx = 46 if right else -46
            dy = 26 if slot == 0 else -30
            self._notes.append(self.axes[ai].annotate(
                f"{tag}: {y:.{dec}f} {unit}",
                xy=(self.t[i], y), xycoords="data",
                xytext=(dx, dy), textcoords="offset points",
                fontsize=7, color="black",
                ha="left" if right else "right", va="center",
                bbox=dict(boxstyle="round,pad=0.25", fc="#ffffe0",
                          ec=color, lw=0.8, alpha=.95),
                arrowprops=dict(arrowstyle="-", color=color, lw=0.8,
                                shrinkA=0, shrinkB=2),
                annotation_clip=True, zorder=20))
            self._notes.append(self.axes[ai].plot(
                [self.t[i]], [y], marker="o", ms=3.5, color=color, zorder=21)[0])

    def _fill_readout(self, idx):
        ia, ib = idx.get("A"), idx.get("B")
        for _, key, label, unit, dec in SERIES:
            va = f"{self.data[key][ia]:.{dec}f}" if ia is not None else ""
            vb = f"{self.data[key][ib]:.{dec}f}" if ib is not None else ""
            vd = (f"{self.data[key][ib] - self.data[key][ia]:+.{dec}f}"
                  if (ia is not None and ib is not None) else "")
            self.tree.item(key, values=(f"{label} [{unit}]", va, vb, vd))

        if ia is not None and ib is not None:
            dt = self.t[ib] - self.t[ia]
            freq = f"  ({1.0 / abs(dt):.1f} Hz)" if abs(dt) > 1e-9 else ""
            self.delta_lbl.config(
                text=f"A = {self.t[ia]:.4f} s    B = {self.t[ib]:.4f} s    "
                     f"dt = {dt * 1000:+.2f} ms{freq}")
        elif ia is not None:
            self.delta_lbl.config(
                text=f"A = {self.t[ia]:.4f} s    (right click to place B)")
        else:
            self.delta_lbl.config(
                text="left click = cursor A,  right click = cursor B,  "
                     "wheel = zoom X,  shift+wheel = zoom Y")


# ===========================================================================
class CaptureWindow(tk.Toplevel):
    def __init__(self, master, t, data, rows, header, title, on_log, note=""):
        super().__init__(master)
        self.title(title)
        self.geometry("1150x900")
        self.rows = rows
        self.header = header
        self.on_log = on_log
        self.note = note

        top = ttk.Frame(self, padding=4)
        top.pack(fill="x")
        ttk.Button(top, text="Save CSV as...", command=self.save_csv).pack(side="left")
        self.info = ttk.Label(top, text="", font=("Consolas", 9))
        self.info.pack(side="left", padx=10)

        self.pane = PlotPane(self, on_log=on_log)
        self.pane.pack(fill="both", expand=True)
        self.pane.set_data(t, data)
        self._annotate_move(t, data)
        if note:
            self.info.config(text=f"{self.info.cget('text')}    |    {note}")

    def _annotate_move(self, t, data):
        """Mark the last commanded move and how long it took to land inside
        the +-5 deg band - the number the 100 ms requirement is written in."""
        p_cmd, p_act = data["p_cmd"], data["p_act"]
        d = np.abs(np.diff(p_cmd))
        moving = np.flatnonzero(d > 0.02) + 1
        if moving.size == 0:
            self.info.config(text="no commanded move in this capture")
            return
        # Walk back through the last moving block. Tolerate short gaps so a
        # smooth or noisy command ramp is not split into fragments.
        GAP = 25
        i_end = int(moving[-1])
        mset = set(moving.tolist())
        i0 = i_end
        while i0 > 1:
            prev = next((j for j in range(i0 - 1, max(0, i0 - 1 - GAP), -1)
                         if j in mset), None)
            if prev is None:
                break
            i0 = prev
        target = p_cmd[i_end]
        reach = np.flatnonzero(np.abs(p_act[i0:] - target) <= 5.0)
        ax = self.pane.axes[0]
        ax.axvline(t[i0], color="grey", ls=":", lw=1.0)
        if reach.size:
            ir = i0 + int(reach[0])
            ax.axvline(t[ir], color="green", ls=":", lw=1.0)
            dt_ms = (t[ir] - t[i0]) * 1000.0
            txt = (f"last move {p_cmd[i0]:.0f} -> {target:.0f} deg,  "
                   f"first reach +-5 deg in {dt_ms:.0f} ms")
            self.on_log(f"# last move reached +-5 deg in {dt_ms:.1f} ms")
        else:
            txt = "last move never reached +-5 deg in this capture"
        self.info.config(text=txt)
        self.pane.canvas.draw_idle()

    def save_csv(self):
        path = filedialog.asksaveasfilename(
            parent=self, defaultextension=".csv",
            initialfile=f"capture_{stamp()}.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        with open(path, "w", encoding="utf-8", newline="") as fh:
            # Keep the source file's own header, so a re-save of a loaded file
            # is byte-identical rather than silently renaming its columns.
            fh.write(self.header + "\n")
            fh.write("\n".join(self.rows))
            fh.write("\n")
        self.on_log(f"# saved {len(self.rows)} rows -> {path}")


# ===========================================================================
class App(tk.Tk):
    def __init__(self, port=None):
        super().__init__()
        self.title("RS06 brake actuator console")
        self.geometry("1240x820")

        self.link = Link()
        self.t0 = None
        self.hist = {k: collections.deque() for k in
                     ("t", "p_cmd", "p_act", "v_cmd", "v_act",
                      "t_cmd", "t_act", "iq", "vbus", "temp")}
        self.rec_writer = None
        self.rec_file = None
        self.rec_rows = 0

        OUT_DIR.mkdir(exist_ok=True)
        self._build()
        self.after(30, self._pump)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        if port:
            self.port_var.set(port)
            self.connect()

    # ------------------------------------------------------------------ UI
    def _build(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")

        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=26,
                                    values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="Rescan", command=self._rescan).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.conn_btn.pack(side="left", padx=6)
        self.status = ttk.Label(top, text="disconnected", font=("Consolas", 10))
        self.status.pack(side="left", padx=10)

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True)
        side = ttk.Frame(body, padding=6)
        side.pack(side="left", fill="y")

        g = ttk.LabelFrame(side, text="Control", padding=6)
        g.pack(fill="x", pady=3)
        r = ttk.Frame(g); r.pack(fill="x")
        ttk.Button(r, text="START", command=lambda: self.cmd("START")).pack(side="left", expand=True, fill="x")
        ttk.Button(r, text="STOP", command=lambda: self.cmd("STOP")).pack(side="left", expand=True, fill="x")
        ttk.Button(g, text="CLEAR FAULT", command=lambda: self.cmd("CLEARFAULT")).pack(fill="x", pady=2)
        r = ttk.Frame(g); r.pack(fill="x", pady=2)
        ttk.Button(r, text="MODE POS", command=lambda: self.cmd("MODE POS")).pack(side="left", expand=True, fill="x")
        ttk.Button(r, text="MODE TRQ", command=lambda: self.cmd("MODE TRQ")).pack(side="left", expand=True, fill="x")

        r = ttk.Frame(g); r.pack(fill="x", pady=(8, 2))
        ttk.Label(r, text="angle").pack(side="left")
        self.angle_var = tk.StringVar(value="200")
        ttk.Entry(r, textvariable=self.angle_var, width=8).pack(side="left", padx=3)
        ttk.Label(r, text="deg").pack(side="left")
        ttk.Button(g, text="MOVE", command=self.do_move).pack(fill="x")

        r = ttk.Frame(g); r.pack(fill="x", pady=(8, 2))
        ttk.Label(r, text="torque").pack(side="left")
        self.torque_var = tk.StringVar(value="0.0")
        ttk.Entry(r, textvariable=self.torque_var, width=8).pack(side="left", padx=3)
        ttk.Label(r, text="Nm").pack(side="left")
        ttk.Button(g, text="APPLY TORQUE", command=self.do_torque).pack(fill="x")

        r = ttk.Frame(g); r.pack(fill="x", pady=(8, 0))
        ttk.Button(r, text="ZERO soft", command=lambda: self.cmd("ZERO SOFT")).pack(side="left", expand=True, fill="x")
        ttk.Button(r, text="ZERO motor", command=lambda: self.cmd("ZERO MOTOR")).pack(side="left", expand=True, fill="x")
        ttk.Button(g, text="CALIBRATE", command=self.do_calib).pack(fill="x", pady=(6, 0))

        g = ttk.LabelFrame(side, text="1 kHz capture", padding=6)
        g.pack(fill="x", pady=3)
        r = ttk.Frame(g); r.pack(fill="x")
        ttk.Label(r, text="last").pack(side="left")
        self.dump_var = tk.StringVar(value="3")
        ttk.Entry(r, textvariable=self.dump_var, width=5).pack(side="left", padx=3)
        ttk.Label(r, text="s").pack(side="left")
        ttk.Button(g, text="CAPTURE + PLOT", command=self.do_dump).pack(fill="x", pady=2)
        ttk.Button(g, text="save to SD card", command=lambda: self.cmd("SDDUMP")).pack(fill="x")

        g = ttk.LabelFrame(side, text="File", padding=6)
        g.pack(fill="x", pady=3)
        ttk.Button(g, text="Load CSV...", command=self.do_load_csv).pack(fill="x")

        g = ttk.LabelFrame(side, text="Live CSV", padding=6)
        g.pack(fill="x", pady=3)
        self.rec_btn = ttk.Button(g, text="REC start", command=self.toggle_rec)
        self.rec_btn.pack(fill="x")
        self.rec_lbl = ttk.Label(g, text="not recording", font=("Consolas", 8))
        self.rec_lbl.pack(anchor="w")
        ttk.Button(g, text="save buffer as CSV", command=self.save_buffer).pack(fill="x", pady=(4, 0))

        g = ttk.LabelFrame(side, text="Parameter", padding=6)
        g.pack(fill="x", pady=3)
        r = ttk.Frame(g); r.pack(fill="x")
        self.key_var = tk.StringVar(value="kp")
        ttk.Entry(r, textvariable=self.key_var, width=8).pack(side="left")
        self.val_var = tk.StringVar(value="")
        ttk.Entry(r, textvariable=self.val_var, width=10).pack(side="left", padx=3)
        ttk.Button(g, text="SET", command=self.do_set).pack(fill="x", pady=2)
        ttk.Button(g, text="read all PARAMS", command=lambda: self.cmd("PARAMS")).pack(fill="x")

        ttk.Button(side, text="clear plot", command=self.clear_hist).pack(fill="x", pady=6)

        # live plot ---------------------------------------------------------
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)
        self.live = PlotPane(right, on_log=self.log)
        self.live.pack(fill="both", expand=True)

        self.logbox = tk.Text(self, height=6, font=("Consolas", 9), wrap="none")
        self.logbox.pack(fill="x", side="bottom")

    # ------------------------------------------------------------- helpers
    def _ports(self):
        return [f"{p.device}  {p.description}" for p in serial.tools.list_ports.comports()]

    def _rescan(self):
        self.port_cb["values"] = self._ports()

    def log(self, text):
        self.logbox.insert("end", text + "\n")
        self.logbox.see("end")
        if int(self.logbox.index("end-1c").split(".")[0]) > 400:
            self.logbox.delete("1.0", "200.0")

    def cmd(self, line):
        if not self.link.is_open:
            self.log("! not connected")
            return
        self.link.send(line)

    # ----------------------------------------------------------- connection
    def connect(self):
        if self.link.is_open:
            self.cmd("STREAM 0")
            time.sleep(0.1)
            self.link.close()
            self.conn_btn.config(text="Connect")
            self.status.config(text="disconnected")
            return
        port = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not port:
            messagebox.showwarning("RS06", "Pick a serial port first.")
            return
        try:
            self.link.open(port)
        except Exception as exc:
            messagebox.showerror("RS06", f"{port}: {exc}")
            return
        self.conn_btn.config(text="Disconnect")
        self.log(f"# opened {port} @ {BAUD}")
        self.after(400, lambda: (self.cmd("ID"), self.cmd("PARAMS"),
                                 self.cmd(f"STREAM {STREAM_HZ}")))

    def _on_close(self):
        try:
            if self.link.is_open:
                self.link.send("STREAM 0")
                time.sleep(0.1)
        finally:
            self._stop_rec()
            self.link.close()
            self.destroy()

    # -------------------------------------------------------------- actions
    def do_move(self):
        try:
            self.cmd(f"MOVE {float(self.angle_var.get()):.3f}")
        except ValueError:
            self.log("! angle must be a number")

    def do_torque(self):
        try:
            self.cmd(f"TRQ {float(self.torque_var.get()):.3f}")
        except ValueError:
            self.log("! torque must be a number")

    def do_set(self):
        key, val = self.key_var.get().strip(), self.val_var.get().strip()
        if key and val:
            self.cmd(f"SET {key} {val}")
        else:
            self.log("! need key and value")

    def do_calib(self):
        if messagebox.askyesno("RS06", "Run the calibration sequence?\n"
                                       "The axis moves at full effort - keep it clear."):
            self.cmd("CALIB START")

    def do_dump(self):
        try:
            sec = float(self.dump_var.get())
        except ValueError:
            return self.log("! capture length must be a number")
        self.log(f"# requesting {sec:g} s of 1 kHz data...")
        self.cmd(f"DUMP {sec:g}")

    def clear_hist(self):
        for d in self.hist.values():
            d.clear()
        self.t0 = None
        self.live.set_data(np.zeros(0), {k: np.zeros(0) for _, k, _, _, _ in SERIES})

    # ------------------------------------------------------------ live CSV
    def toggle_rec(self):
        if self.rec_file:
            self._stop_rec()
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialdir=str(OUT_DIR),
            initialfile=f"live_{stamp()}.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        self.rec_file = open(path, "w", encoding="utf-8", newline="")
        self.rec_writer = csv.writer(self.rec_file)
        self.rec_writer.writerow(["host_time"] + T_FIELDS)
        self.rec_rows = 0
        self.rec_btn.config(text="REC stop")
        self.rec_lbl.config(text=Path(path).name)
        self.log(f"# recording live stream -> {path}")

    def _stop_rec(self):
        if not self.rec_file:
            return
        self.rec_file.close()
        self.rec_file = None
        self.rec_writer = None
        self.rec_btn.config(text="REC start")
        self.rec_lbl.config(text=f"stopped, {self.rec_rows} rows")
        self.log(f"# recording stopped, {self.rec_rows} rows")

    def save_buffer(self):
        if not self.hist["t"]:
            return self.log("! live buffer is empty")
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialdir=str(OUT_DIR),
            initialfile=f"buffer_{stamp()}.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        keys = ["t", "p_cmd", "p_act", "v_cmd", "v_act",
                "t_cmd", "t_act", "iq", "vbus", "temp"]
        with open(path, "w", encoding="utf-8", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "p_cmd_deg", "p_act_deg", "v_cmd", "v_act",
                        "t_cmd", "t_act", "iq", "vbus", "temp"])
            for row in zip(*(self.hist[k] for k in keys)):
                w.writerow([f"{v:.6g}" for v in row])
        self.log(f"# saved {len(self.hist['t'])} rows -> {path}")

    # ----------------------------------------------------------------- pump
    def _pump(self):
        redraw = False
        try:
            while True:
                kind, payload = self.link.rx.get_nowait()
                if kind == "telem":
                    self._on_telem(payload)
                    redraw = True
                elif kind == "dump":
                    self._on_dump(payload)
                elif kind == "tx":
                    self.log("> " + payload)
                elif kind == "err":
                    self.log("! " + payload)
                else:
                    self.log(payload)
        except queue.Empty:
            pass
        if redraw:
            self._redraw_live()
        self.after(80, self._pump)

    def _on_telem(self, parts):
        d = dict(zip(T_FIELDS, parts))
        if self.rec_writer:
            self.rec_writer.writerow([f"{time.time():.3f}"] + parts)
            self.rec_rows += 1
            if self.rec_rows % 50 == 0:
                self.rec_lbl.config(text=f"recording, {self.rec_rows} rows")

        t_ms = float(d["t_ms"])
        if self.t0 is None:
            self.t0 = t_ms
        t = (t_ms - self.t0) / 1000.0

        h = self.hist
        h["t"].append(t)
        for k in ("p_cmd", "p_act", "v_cmd", "v_act", "t_cmd", "t_act",
                  "iq", "vbus", "temp"):
            h[k].append(float(d[k]))
        while h["t"] and (t - h["t"][0]) > HISTORY_S:
            for dq in h.values():
                dq.popleft()

        latched = int(d["events"])
        active = int(d["active"])
        fault = int(d.get("fault", 0))
        # A latched fault is the headline; with none outstanding, show the live
        # limit instead, marked with "~" so the two never look alike.
        if latched:
            tail = f"[{decode_events(latched)}]"
            if fault:
                tail += f" fault:{decode_faults(fault)}"
        elif active:
            tail = f"~{decode_events(active)}"
        else:
            tail = "[-]"
        self.status.config(
            text=(f"{d['state']:<9} {float(d['vbus']):5.1f}V {float(d['temp']):5.1f}C  "
                  f"ang {float(d['p_act']):8.2f}d  tq {float(d['t_act']):6.2f}Nm  {tail}"))

    def _redraw_live(self):
        if not self.hist["t"]:
            return
        # Preserve cursors while the window scrolls: only refresh the curves.
        t = np.fromiter(self.hist["t"], dtype=float)
        self.live.t = t
        for _, key, _, _, _ in SERIES:
            arr = np.fromiter(self.hist[key], dtype=float)
            self.live.data[key] = arr
            self.live.lines[key].set_data(t, arr)
        if self.live.toolbar.mode == "" and self.live.cursor["A"] is None:
            self.live.axes[0].set_xlim(max(0.0, t[-1] - HISTORY_S),
                                       max(HISTORY_S, t[-1]))
            self.live.autoscale_y()
        else:
            self.live.canvas.draw_idle()

    # -------------------------------------------------------------- load
    def do_load_csv(self):
        path = filedialog.askopenfilename(
            initialdir=str(OUT_DIR) if OUT_DIR.exists() else None,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            t, data, rows, header, note = load_csv(path)
        except Exception as exc:
            self.log(f"! {Path(path).name}: {exc}")
            messagebox.showerror("RS06", "Could not read\n%s\n\n%s" % (path, exc))
            return
        self.log(f"# loaded {Path(path).name}: {note}")
        CaptureWindow(self, t, data, rows, header,
                      f"{Path(path).name}   ({len(t)} samples)", self.log, note)

    # ----------------------------------------------------------- 1 kHz dump
    def _on_dump(self, rows):
        if not rows:
            return self.log("! capture returned no rows")
        cols = {k: [] for k in D_FIELDS}
        # Accept a row that is one column short: captures taken before the
        # events word was split carry no `active` column, and refusing to plot
        # them would throw away the existing test history.
        widths = (len(D_FIELDS), len(D_FIELDS) - 1)
        for line in rows:
            f = line.split(",")
            if len(f) not in widths:
                continue
            try:
                vals = [float(x) for x in f]
            except ValueError:
                continue
            for k, v in zip(D_FIELDS, vals):
                cols[k].append(v)
        if not cols["t_us"]:
            return self.log("! capture unparseable")

        t_us = np.array(cols["t_us"])
        t = (t_us - t_us[0]) / 1e6
        data = {k: np.array(cols[k]) for _, k, _, _, _ in SERIES}

        tag = stamp()
        auto = OUT_DIR / f"capture_{tag}.csv"
        with auto.open("w", encoding="utf-8") as fh:
            fh.write(",".join(D_FIELDS) + "\n")
            fh.write("\n".join(rows) + "\n")
        self.log(f"# {len(t)} samples -> {auto}")

        CaptureWindow(self, t, data, rows, ",".join(D_FIELDS),
                      f"1 kHz capture  {tag}   ({len(t)} samples)", self.log)


def selftest():
    """Check the environment without opening a window - used by the .bat so a
    broken install reports the missing piece instead of flashing a dead GUI."""
    import matplotlib as mpl
    print("rs06_console self-test")
    print("  python     :", sys.version.split()[0], sys.executable)
    print("  pyserial   :", serial.__version__)
    print("  numpy      :", np.__version__)
    print("  matplotlib :", mpl.__version__)
    print("  tkinter    : ok")
    ports = [f"{p.device} ({p.description})" for p in serial.tools.list_ports.comports()]
    print("  serial ports:", ", ".join(ports) if ports else "none found")
    print("OK")
    return 0


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    flags = [a for a in sys.argv[1:] if a.startswith("-")]
    if "--selftest" in flags:
        sys.exit(selftest())
    App(args[0] if args else None).mainloop()
