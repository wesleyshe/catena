"""Control window for the chain sim: buttons instead of shell commands.

Double-click 'Chain Sim.command' in Finder, or run: .venv/bin/python panel.py
It builds chain.xml, opens the MuJoCo viewer (play.py under mjpython) next to it, and sends the viewer commands
over a local socket. The viewer sends live readouts back, shown here and drawn in the viewer.
"""
import glob
import json
import os
import socket
import subprocess
import tkinter as tk
from tkinter import filedialog, simpledialog, ttk

import setup_file
from make_chain import EE_TYPES, N_MODULES, PRESETS, build
from setup_file import SEQ_DIR, SETUP_DIR

HERE = os.path.dirname(os.path.abspath(__file__))
MJPYTHON = os.path.join(HERE, ".venv", "bin", "mjpython")
MODEL = os.path.join(HERE, "chain.xml")
SPIN_AXES = ["vertical (z)", "along chain (y)", "across (x)"]  # the letter before ")" is what the viewer gets


class Panel:
    def __init__(self, root):
        self.root = root
        root.title("Chain Sim")
        root.attributes("-topmost", True)  # stay visible next to the viewer
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.server = socket.socket()
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.server.setblocking(False)
        self.conn, self.buf, self.proc, self.sim_running = None, b"", None, True

        self.preset, self.modules, self.slip = tk.StringVar(value="original14"), tk.IntVar(value=N_MODULES), tk.BooleanVar()
        self.seq, self.move, self.hold = tk.StringVar(), tk.DoubleVar(value=3.0), tk.DoubleVar(value=1.0)
        self.speed, self.gravity, self.g = tk.DoubleVar(value=1.0), tk.BooleanVar(), tk.DoubleVar(value=9.81)
        self.logging, self.status = tk.BooleanVar(), tk.StringVar(value="starting the viewer...")
        self.spin_rate, self.spin_axis = tk.DoubleVar(value=20.0), tk.StringVar(value=SPIN_AXES[0])
        self.air, self.cage = tk.BooleanVar(), tk.BooleanVar()
        self.ee1, self.ee_n = tk.StringVar(value="none"), tk.StringVar(value="none")
        self.setup_name, self.about, self.about_text = tk.StringVar(), tk.StringVar(), ""
        self.spin_at_play = tk.BooleanVar()

        def frame(title):
            f = ttk.LabelFrame(root, text=title, padding=6)
            f.pack(fill="x", padx=8, pady=4)
            return f

        f = frame("Setup (a whole run, from setups/)")
        self.setup_box = ttk.Combobox(f, textvariable=self.setup_name, values=self.setups(), state="readonly", width=24,
                                      postcommand=lambda: self.setup_box.configure(values=self.setups()))
        self.setup_box.grid(row=0, column=0, sticky="we")
        self.setup_box.bind("<<ComboboxSelected>>", lambda e: self.show_about())
        ttk.Button(f, text="Load", command=self.load_setup).grid(row=0, column=1, padx=(6, 0))
        ttk.Button(f, text="Save as...", command=self.save_setup).grid(row=0, column=2, padx=(6, 0))
        ttk.Label(f, textvariable=self.about, wraplength=400, justify="left").grid(row=1, column=0, columnspan=3, sticky="w")
        f.columnconfigure(0, weight=1)
        self.about.set("Pick a setup and click Load (it fills everything below), or set things up by hand.")

        f = frame("Model")
        ttk.Label(f, text="Preset").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.preset, values=list(PRESETS), state="readonly", width=12).grid(row=0, column=1)
        ttk.Label(f, text="Modules").grid(row=0, column=2, padx=(8, 0))
        ttk.Spinbox(f, from_=2, to=40, textvariable=self.modules, width=4).grid(row=0, column=3)
        ttk.Checkbutton(f, text="Slip ring", variable=self.slip).grid(row=0, column=4, padx=8, sticky="w")
        ttk.Checkbutton(f, text="Air drag (cabin air)", variable=self.air).grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(f, text="Net cage (size in spec.toml)", variable=self.cage).grid(row=1, column=2, columnspan=3, sticky="w")
        f.columnconfigure(4, weight=1)

        f = frame("End effectors (a module on each end)")
        for row, (label, var) in enumerate((("Past m1 (on the m1_yaw twist servo)", self.ee1), ("Past the last module", self.ee_n))):
            ttk.Label(f, text=label).grid(row=row, column=0, sticky="w")
            ttk.Combobox(f, textvariable=var, values=EE_TYPES, state="readonly", width=10).grid(row=row, column=1, padx=(8, 0))
        ttk.Button(root, text="Rebuild + open viewer  (applies Model and End effectors)",
                   command=self.open_viewer).pack(fill="x", padx=8, pady=(0, 4))

        f = frame("Fold sequence")
        self.seq_box = ttk.Combobox(f, textvariable=self.seq, values=self.sequences(), width=28,
                                    postcommand=lambda: self.seq_box.configure(values=self.sequences()))
        self.seq.set(next(iter(self.sequences()), ""))
        self.seq_box.grid(row=0, column=0, columnspan=4, sticky="we")
        ttk.Button(f, text="Browse...", command=self.browse).grid(row=0, column=4, sticky="we")
        timing = ttk.Frame(f)
        timing.grid(row=1, column=0, columnspan=5, sticky="w", pady=4)
        for label, var, lo, hi, step in (("Move s", self.move, 0.2, 20, 0.5), ("Hold s", self.hold, 0, 20, 0.5),
                                         ("Speed x", self.speed, 0.1, 10, 0.25)):
            ttk.Label(timing, text=label).pack(side="left", padx=(0, 4))
            ttk.Spinbox(timing, from_=lo, to=hi, increment=step, textvariable=var, width=5,
                        command=self.send_speed if var is self.speed else None).pack(side="left", padx=(0, 14))
        for col, (label, cmd) in enumerate((("Play", self.play), ("Pause", lambda: self.send("pause")),
                                            ("Resume", lambda: self.send("resume")), ("Restart", lambda: self.send("restart")),
                                            ("Stop", lambda: self.send("stop")))):
            ttk.Button(f, text=label, command=cmd, width=7).grid(row=2, column=col, sticky="we")
            f.columnconfigure(col, weight=1)  # buttons share the width evenly

        f = frame("World")
        ttk.Checkbutton(f, text="Gravity", variable=self.gravity, command=self.send_gravity).grid(row=0, column=0)
        ttk.Spinbox(f, from_=0, to=30, increment=0.5, textvariable=self.g, width=5, command=self.send_gravity).grid(row=0, column=1)
        ttk.Label(f, text="m/s²  ").grid(row=0, column=2)
        ttk.Button(f, text="Straighten", command=lambda: self.send("straighten")).grid(row=0, column=3)
        ttk.Button(f, text="Reset pose", command=lambda: self.send("reset")).grid(row=0, column=4, sticky="w")
        self.physics_btn = ttk.Button(f, text="⏸  Pause physics", command=lambda: self.send("physics", run=not self.sim_running))
        self.physics_btn.grid(row=1, column=0, columnspan=5, sticky="we", pady=(6, 0))
        f.columnconfigure(4, weight=1)

        f = frame("Spin (through the centre of mass; also: leftover spin when it's let go)")
        ttk.Spinbox(f, from_=-360, to=360, increment=0.5, textvariable=self.spin_rate, width=6).grid(row=0, column=0)
        ttk.Label(f, text="deg/s about").grid(row=0, column=1, padx=(4, 4))
        ttk.Combobox(f, textvariable=self.spin_axis, values=SPIN_AXES, state="readonly", width=15).grid(row=0, column=2)
        ttk.Button(f, text="Spin", command=lambda: self.send("spin", rate=self.spin_rate.get(),
                                                             axis=self.spin_axis.get()[-2])).grid(row=0, column=3, padx=(6, 0))
        ttk.Button(f, text="Stop spin", command=lambda: self.send("spin", rate=0.0, axis="all")).grid(row=0, column=4, padx=6)
        ttk.Checkbutton(f, text="Also spin like this when Play starts (the spin at release)",
                        variable=self.spin_at_play).grid(row=1, column=0, columnspan=5, sticky="w", pady=(4, 0))

        f = frame("Record")
        ttk.Checkbutton(f, text="Log to logs/", variable=self.logging, command=lambda: self.send("log", on=self.logging.get())).grid(row=0, column=0)
        ttk.Button(f, text="Screenshot", command=lambda: self.send("screenshot")).grid(row=0, column=1, padx=6)
        ttk.Button(f, text="Open folder", command=lambda: subprocess.run(["open", HERE])).grid(row=0, column=2)

        f = frame("Readouts")
        ttk.Label(f, textvariable=self.status, font=("Menlo", 11), justify="left").pack(anchor="w")

        self.open_viewer()
        self.poll()

    def sequences(self):
        return sorted(os.path.basename(p) for p in glob.glob(os.path.join(SEQ_DIR, "*.csv")))

    def setups(self):
        return sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(SETUP_DIR, "*.toml")))

    def show_about(self, note=""):
        path = os.path.join(SETUP_DIR, self.setup_name.get() + ".toml")
        self.about_text = setup_file.about(path) if os.path.exists(path) else ""
        self.about.set("\n".join(x for x in (self.about_text or "(no line saying what it shows)", note) if x))

    def load_setup(self):
        """Fill every field from the picked setup, then rebuild and open the viewer."""
        if not self.setup_name.get():
            self.about.set("Pick a setup first.")
            return
        try:
            s = setup_file.load(os.path.join(SETUP_DIR, self.setup_name.get() + ".toml"))
        except (OSError, ValueError) as e:  # a TOML syntax error is a ValueError too
            self.about.set(f"Can't load it: {e}")
            return
        self.preset.set(s["preset"]), self.modules.set(s["modules"]), self.slip.set(s["slip_ring"])
        self.air.set(s["air"]), self.cage.set(s["cage"]), self.ee1.set(s["ee1"]), self.ee_n.set(s["ee_n"])
        seq = s["sequence"]
        self.seq.set(os.path.basename(seq) if os.path.dirname(seq) == SEQ_DIR else seq)
        self.move.set(s["move_s"]), self.hold.set(s["hold_s"]), self.speed.set(s["speed"])
        self.gravity.set(s["gravity_m_s2"] > 0)
        if s["gravity_m_s2"] > 0:
            self.g.set(s["gravity_m_s2"])
        rate, axis, warning = setup_file.single_axis_spin(s["spin_deg_s"])
        self.spin_at_play.set(rate != 0)
        if rate:
            self.spin_rate.set(rate)
            self.spin_axis.set(next(a for a in SPIN_AXES if a[-2] == axis))
        self.show_about(warning)
        self.open_viewer()

    def save_setup(self):
        """Save every field as a setup file, with a line saying what it shows."""
        path = filedialog.asksaveasfilename(initialdir=SETUP_DIR, defaultextension=".toml", filetypes=[("Setup", "*.toml")])
        if not path:
            return
        about = simpledialog.askstring("Save setup", "One line on what this run shows:", initialvalue=self.about_text,
                                       parent=self.root)
        if about is None:
            return
        try:
            spin = [0.0, 0.0, 0.0]
            if self.spin_at_play.get():
                spin["xyz".index(self.spin_axis.get()[-2])] = float(self.spin_rate.get())
            seq = self.seq.get()
            setup_file.save(path, {**setup_file.DEFAULTS, "sequence": os.path.join(SEQ_DIR, seq) if seq else "",
                                   "move_s": float(self.move.get()), "hold_s": float(self.hold.get()),
                                   "speed": float(self.speed.get()), "spin_deg_s": spin, "air": self.air.get(),
                                   "gravity_m_s2": float(self.g.get()) if self.gravity.get() else 0.0,
                                   "cage": self.cage.get(), "preset": self.preset.get(), "modules": int(self.modules.get()),
                                   "slip_ring": self.slip.get(), "ee1": self.ee1.get(), "ee_n": self.ee_n.get()}, about)
        except (tk.TclError, ValueError, OSError) as e:  # e.g. a number box holding something that isn't a number
            self.about.set(f"Can't save it: {e}")
            return
        if os.path.dirname(os.path.abspath(path)) == SETUP_DIR:
            self.setup_name.set(os.path.splitext(os.path.basename(path))[0])
            self.show_about()
        else:
            self.about.set(f"Saved {path} (outside setups/, so it isn't in the list)")

    def browse(self):
        path = filedialog.askopenfilename(initialdir=SEQ_DIR, filetypes=[("CSV", "*.csv")])
        if path:
            self.seq.set(path)

    def open_viewer(self):
        """Rebuild chain.xml from the Model settings and (re)start the viewer."""
        self.close_viewer()
        g = self.g.get() if self.gravity.get() else 0.0
        with open(MODEL, "w") as fh:
            fh.write(build(self.preset.get(), int(self.modules.get()), self.slip.get(), g, self.air.get(), self.cage.get(),
                           ee1=self.ee1.get(), ee_n=self.ee_n.get()) + "\n")
        self.logging.set(False)
        self.proc = subprocess.Popen([MJPYTHON, os.path.join(HERE, "play.py"), "--model", MODEL, "--log", "",
                                      "--panel", str(self.server.getsockname()[1])], cwd=HERE)
        self.status.set("starting the viewer...")

    def close_viewer(self):
        if self.proc and self.proc.poll() is None:
            self.send("quit")
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
        self.conn, self.buf = None, b""

    def send(self, cmd, **args):
        if not self.conn:
            self.status.set("viewer not connected: click 'Rebuild + open viewer'")
            return
        try:
            self.conn.sendall((json.dumps({"cmd": cmd, **args}) + "\n").encode())
        except OSError:
            self.conn = None

    def play(self):
        path = self.seq.get()
        if not path:
            self.status.set("pick a fold sequence first")
            return
        self.send("speed", value=self.speed.get())
        if self.spin_at_play.get():
            self.send("spin", rate=self.spin_rate.get(), axis=self.spin_axis.get()[-2])
        self.send("play", file=os.path.join(SEQ_DIR, path), move=self.move.get(), hold=self.hold.get())  # abs path stays

    def send_speed(self):
        self.send("speed", value=self.speed.get())

    def send_gravity(self):
        self.send("gravity", g=self.g.get() if self.gravity.get() else 0.0)

    def poll(self):
        """Every 100 ms: accept the viewer's connection, read its readouts, notice if it was closed."""
        if self.conn is None and self.proc and self.proc.poll() is None:
            try:
                self.conn, _ = self.server.accept()
                self.conn.setblocking(False)
            except BlockingIOError:
                pass
        if self.conn:
            try:
                chunk = self.conn.recv(65536)
                self.buf += chunk
                if not chunk:
                    self.conn = None
            except BlockingIOError:
                pass
            lines = self.buf.split(b"\n")
            self.buf = lines[-1]
            if len(lines) > 1:
                info = json.loads(lines[-2])
                self.status.set("\n".join(f"{k:<11}{v}" for k, v in info.items()))
                self.sim_running = info["physics"] == "running"  # also follows the Space key in the viewer
                self.physics_btn.configure(text="⏸  Pause physics" if self.sim_running else "▶  Resume physics")
        if self.proc and self.proc.poll() is not None:
            self.status.set("viewer closed: click 'Rebuild + open viewer'")
            self.proc = None
        self.root.after(100, self.poll)

    def quit(self):
        self.close_viewer()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Panel(root)
    root.mainloop()
