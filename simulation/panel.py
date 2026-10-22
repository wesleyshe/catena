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
from tkinter import filedialog, ttk

from make_chain import N_MODULES, PRESETS, build

HERE = os.path.dirname(os.path.abspath(__file__))
MJPYTHON = os.path.join(HERE, ".venv", "bin", "mjpython")
MODEL = os.path.join(HERE, "chain.xml")
SEQ_DIR = os.path.join(os.path.dirname(HERE), "sequences")  # at the repo root, shared with the robot software


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
        self.spin_rate = tk.DoubleVar(value=20.0)

        def frame(title):
            f = ttk.LabelFrame(root, text=title, padding=6)
            f.pack(fill="x", padx=8, pady=4)
            return f

        f = frame("Model")
        ttk.Label(f, text="Preset").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.preset, values=list(PRESETS), state="readonly", width=12).grid(row=0, column=1)
        ttk.Label(f, text="Modules").grid(row=0, column=2, padx=(8, 0))
        ttk.Spinbox(f, from_=2, to=40, textvariable=self.modules, width=4).grid(row=0, column=3)
        ttk.Checkbutton(f, text="Slip ring", variable=self.slip).grid(row=0, column=4, padx=8, sticky="w")
        ttk.Button(f, text="Rebuild + open viewer", command=self.open_viewer).grid(row=1, column=0, columnspan=5, sticky="we", pady=(6, 0))
        f.columnconfigure(4, weight=1)  # let the button span the whole box

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

        f = frame("Spin (about the vertical, through the centre of mass)")
        ttk.Spinbox(f, from_=-360, to=360, increment=5, textvariable=self.spin_rate, width=6).grid(row=0, column=0)
        ttk.Label(f, text="deg/s").grid(row=0, column=1, padx=(4, 8))
        ttk.Button(f, text="Spin", command=lambda: self.send("spin", rate=self.spin_rate.get())).grid(row=0, column=2)
        ttk.Button(f, text="Stop spin", command=lambda: self.send("spin", rate=0.0)).grid(row=0, column=3, padx=6)

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

    def browse(self):
        path = filedialog.askopenfilename(initialdir=SEQ_DIR, filetypes=[("CSV", "*.csv")])
        if path:
            self.seq.set(path)

    def open_viewer(self):
        """Rebuild chain.xml from the Model settings and (re)start the viewer."""
        self.close_viewer()
        g = self.g.get() if self.gravity.get() else 0.0
        with open(MODEL, "w") as fh:
            fh.write(build(self.preset.get(), int(self.modules.get()), self.slip.get(), g) + "\n")
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
