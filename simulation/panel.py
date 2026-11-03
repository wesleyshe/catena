"""Control window for the chain sim: buttons instead of shell commands.

Double-click 'Chain Sim.command' in Finder, or run: .venv/bin/python panel.py
It lays its settings over the loaded setup, writes that to panel_setup.toml, opens the MuJoCo viewer (play.py --setup,
under mjpython) next to it, and sends the viewer commands over a local socket. The viewer sends live readouts back,
shown here and drawn in the viewer.
"""
import copy
import glob
import os
import subprocess
import tkinter as tk
from tkinter import filedialog, simpledialog, ttk

import setup_file
from make_chain import EE_DEFAULTS, EE_TYPES, FACINGS, N_MODULES, PRESETS
from setup_file import DEMOS, FLIGHT_DEFAULTS, SEQ_DIR, SETUP_DIR, default_strings
from viewer_link import ViewerLink

HERE = os.path.dirname(os.path.abspath(__file__))
PANEL_SETUP = os.path.join(HERE, "panel_setup.toml")  # what the open viewer was built from (git-ignored)
SPIN_AXES = ["vertical (z)", "along chain (y)", "across (x)"]  # the letter before ")" is what the viewer gets
STARTS = ["floating", "on a surface", "on the rod"]
COUNTDOWN_S = 5  # the Countdown s box starts here: Play and Restart show T-5 ... T-1 in the viewer first
ROD = {"from": [0.0, -1.2, 1.0], "to": [0.0, 1.2, 1.0], "diameter_mm": 35.0}  # until a setup says where the rod is
FIRST_SEQ = "fold_sequence.csv"  # picked when the window opens: it plays on the bare chain


def spinbox(parent, var, lo, hi, step, width=6, **kw):
    return ttk.Spinbox(parent, from_=lo, to=hi, increment=step, textvariable=var, width=width, **kw)


class EndBox:
    """One end effector's controls: its type, then that type's settings and, while running, its buttons."""

    def __init__(self, panel, parent, key, title):
        self.panel, self.key = panel, key
        self.type, self.start, self.facing = tk.StringVar(value="none"), tk.StringVar(value=STARTS[0]), tk.StringVar(value="up")
        self.at, self.along, self.roll = [tk.DoubleVar() for _ in range(3)], tk.DoubleVar(value=0.3), tk.DoubleVar()
        self.settings = {k: tk.DoubleVar(value=EE_DEFAULTS[k]) for k in ("pull_N", "cup_mm", "friction", "thrust_N")}
        self.jets = [tk.BooleanVar() for _ in range(5)]
        f = ttk.Frame(parent)
        f.pack(fill="x", pady=(0, 6))
        self.rows = {}

        def row(name, r):
            self.rows[name] = ttk.Frame(f)
            self.rows[name].grid(row=r, column=0, sticky="w", padx=(12 if r else 0, 0))
            return self.rows[name]

        top = row("top", 0)
        ttk.Label(top, text=title, width=22).pack(side="left")
        ttk.Combobox(top, textvariable=self.type, values=EE_TYPES, state="readonly", width=9).pack(side="left")
        self.grip = ttk.Frame(top)
        for text, on in (("Hold", True), ("Let go", False)):
            ttk.Button(self.grip, text=text, width=6, command=lambda on=on: panel.send("grip", end=self.name(), on=on)).pack(side="left")
        self.nozzles = ttk.Frame(top)
        ttk.Label(self.nozzles, text=" nozzles").pack(side="left")
        for k, var in enumerate(self.jets):
            ttk.Checkbutton(self.nozzles, text=f"n{k}", variable=var,
                            command=lambda k=k: panel.send("nozzle", end=self.name(), n=k, on=self.jets[k].get())).pack(side="left")
        place = row("place", 1)
        ttk.Label(place, text="Starts").pack(side="left")
        ttk.Combobox(place, textvariable=self.start, values=STARTS, state="readonly", width=11).pack(side="left", padx=4)
        ttk.Label(place, text="facing").pack(side="left")
        ttk.Combobox(place, textvariable=self.facing, values=list(FACINGS), state="readonly", width=4).pack(side="left", padx=4)
        ttk.Label(place, text="roll°").pack(side="left")
        spinbox(place, self.roll, -180, 180, 15, 5).pack(side="left", padx=4)
        at = row("at", 2)
        for axis, var in zip("xyz", self.at):
            ttk.Label(at, text=f"{axis} m").pack(side="left")
            spinbox(at, var, -5, 5, 0.1, 5).pack(side="left", padx=(2, 8))
        ttk.Label(row("rod", 3), text="along the rod, m").pack(side="left")
        spinbox(self.rows["rod"], self.along, -5, 5, 0.05, 5).pack(side="left", padx=4)
        self.note = ttk.Label(row("note", 4), text="ee1 sets where the chain starts; this end holds what it lands on")
        self.note.pack(side="left")
        cup = row("cup", 5)
        for label, k, lo, hi, step in (("pull N", "pull_N", 1, 200, 5), ("cup mm", "cup_mm", 10, 120, 5),
                                       ("friction", "friction", 0, 2, 0.1)):
            ttk.Label(cup, text=label).pack(side="left")
            spinbox(cup, self.settings[k], lo, hi, step, 5).pack(side="left", padx=(2, 8))
        ttk.Label(row("thrust", 6), text="thrust per nozzle, N").pack(side="left")
        spinbox(self.rows["thrust"], self.settings["thrust_N"], 0, 5, 0.05, 5).pack(side="left", padx=4)
        for var in (self.type, self.start):
            var.trace_add("write", lambda *a: panel.show_ends())

    def name(self):
        return "ee1" if self.key == "ee1" else f"ee{int(self.panel.modules.get())}"

    def placed(self):
        return self.type.get() in ("anchor", "suction") and self.start.get() != STARTS[0]

    def show(self, other_placed=False):
        """Show only what this type has; grey out the placement if the other end (ee1) sets where the chain starts."""
        kind, start = self.type.get(), self.start.get()
        grips, ignored = kind in ("anchor", "suction"), other_placed and self.key == "ee_n"
        self.grip.pack(side="left", padx=(8, 0)) if grips else self.grip.pack_forget()
        self.nozzles.pack(side="left", padx=(8, 0)) if kind == "thruster" else self.nozzles.pack_forget()
        for name, on in (("place", grips and not ignored), ("at", grips and not ignored and start == STARTS[1]),
                         ("rod", grips and not ignored and start == STARTS[2]), ("note", grips and ignored),
                         ("cup", kind == "suction"), ("thrust", kind == "thruster")):
            self.rows[name].grid() if on else self.rows[name].grid_remove()

    def get(self, base):
        """This end's entry for a setup: `base` (what was loaded) with the fields laid over it."""
        e = {**copy.deepcopy(base), "type": self.type.get(), **{k: float(v.get()) for k, v in self.settings.items()}}
        if self.type.get() in ("anchor", "suction"):
            where = {"at": [float(v.get()) for v in self.at]} if self.start.get() == STARTS[1] else \
                    {"rod_m": float(self.along.get())} if self.start.get() == STARTS[2] else {}
            e["attach"] = {**where, "facing": self.facing.get(), "roll_deg": float(self.roll.get())} if where else {}
        return e

    def set(self, e):
        self.type.set(e["type"])
        for k, v in self.settings.items():
            v.set(e[k])
        a = e["attach"]
        self.start.set(STARTS[1] if "at" in a else STARTS[2] if "rod_m" in a else STARTS[0])
        if "at" in a:
            for var, x in zip(self.at, a["at"]):
                var.set(x)
        if "rod_m" in a:
            self.along.set(a["rod_m"])
        self.facing.set(a.get("facing", "up"))
        self.roll.set(a.get("roll_deg", 0.0))
        for var in self.jets:
            var.set(False)


class Panel:
    def __init__(self, root):
        self.root = root
        root.title("Chain Sim")
        root.attributes("-topmost", True)  # stay visible next to the viewer
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.link, self.sim_running = ViewerLink(), True
        self.loaded = setup_file.new()  # the last setup loaded: it keeps whatever has no field here
        self.played = False  # a take has run since the viewer opened or Reset pose: the next Play starts from the start
        self.autoplay = False  # a demo button was clicked: play as soon as the viewer connects
        self.loaded_flight = False  # the open viewer has the flight box (Fly works)

        self.preset, self.modules, self.slip = tk.StringVar(value="original14"), tk.IntVar(value=N_MODULES), tk.BooleanVar()
        self.seq, self.move, self.hold = tk.StringVar(), tk.DoubleVar(value=3.0), tk.DoubleVar(value=1.0)
        self.speed, self.playback = tk.DoubleVar(value=1.0), tk.DoubleVar(value=1.0)
        self.countdown = tk.IntVar(value=COUNTDOWN_S)  # not saved in setups: it stays put while you switch demos
        self.gravity, self.g = tk.BooleanVar(), tk.DoubleVar(value=9.81)
        self.logging, self.status = tk.BooleanVar(), tk.StringVar(value="starting the viewer...")
        self.spin_rate, self.spin_axis = tk.DoubleVar(value=20.0), tk.StringVar(value=SPIN_AXES[0])
        self.air, self.rod, self.rod_mm = tk.BooleanVar(), tk.BooleanVar(), tk.DoubleVar(value=35.0)
        self.box_on, self.box_m = tk.BooleanVar(), [tk.DoubleVar(value=x) for x in FLIGHT_DEFAULTS["box_m"]]
        self.strings_text, self.jitter = tk.StringVar(value=" ".join(map(str, default_strings(N_MODULES)))), tk.BooleanVar(value=True)
        self.release, self.start = tk.DoubleVar(value=FLIGHT_DEFAULTS["release_s"]), tk.DoubleVar(value=FLIGHT_DEFAULTS["start_s"])
        self.hyper = tk.DoubleVar(value=FLIGHT_DEFAULTS["hyper_s"])
        self.setup_name, self.about, self.about_text = tk.StringVar(), tk.StringVar(), ""
        self.seq_about = tk.StringVar()  # what the picked sequence does, and which setups it was written for
        self.spin_at_play = tk.BooleanVar()

        left, right = ttk.Frame(root), ttk.Frame(root)
        left.grid(row=0, column=0, sticky="n")
        right.grid(row=0, column=1, sticky="n")

        def frame(title, side=left):
            f = ttk.LabelFrame(side, text=title, padding=6)
            f.pack(fill="x", padx=8, pady=4)
            return f

        f = frame("The six demos: one click opens and plays it (Play again for another take)")
        for k, (label, name) in enumerate(DEMOS):
            ttk.Button(f, text=label, command=lambda name=name: self.demo(name)).grid(row=k // 3, column=k % 3, sticky="we")
            f.columnconfigure(k % 3, weight=1)

        f = frame("Setup (a whole run, from setups/)")
        self.setup_box = ttk.Combobox(f, textvariable=self.setup_name, values=self.setups(), state="readonly", width=24,
                                      postcommand=lambda: self.setup_box.configure(values=self.setups()))
        self.setup_box.grid(row=0, column=0, sticky="we")
        self.setup_box.bind("<<ComboboxSelected>>", lambda e: self.show_about())
        ttk.Button(f, text="Load", command=self.load_setup).grid(row=0, column=1, padx=(6, 0))
        ttk.Button(f, text="Save as...", command=self.save_setup).grid(row=0, column=2, padx=(6, 0))
        ttk.Label(f, textvariable=self.about, wraplength=430, justify="left").grid(row=1, column=0, columnspan=3, sticky="w")
        f.columnconfigure(0, weight=1)
        self.about.set("Pick a setup and click Load (it fills everything below), then Play.")

        f = frame("Model")
        ttk.Label(f, text="Preset").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.preset, values=list(PRESETS), state="readonly", width=12).grid(row=0, column=1)
        ttk.Label(f, text="Modules").grid(row=0, column=2, padx=(8, 0))
        spinbox(f, self.modules, 2, 40, 1, 4).grid(row=0, column=3)
        ttk.Checkbutton(f, text="Slip ring", variable=self.slip).grid(row=0, column=4, padx=8, sticky="w")
        ttk.Checkbutton(f, text="Air drag (cabin air)", variable=self.air).grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(f, text="Rod to grab (where: in the setup), Ø mm", variable=self.rod).grid(row=2, column=0, columnspan=4, sticky="w")
        spinbox(f, self.rod_mm, 10, 100, 5, 4).grid(row=2, column=4, sticky="w")
        f.columnconfigure(4, weight=1)

        f = frame("End effectors (a module on each end)")
        self.ends = [EndBox(self, f, "ee1", "Past m1 (twist servo)"), EndBox(self, f, "ee_n", "Past the last module")]
        self.show_ends()
        ttk.Button(left, text="Rebuild + open viewer  (applies Model and End effectors)",
                   command=self.open_viewer).pack(fill="x", padx=8, pady=(0, 4))

        f = frame("Fold sequence")
        self.seq_box = ttk.Combobox(f, textvariable=self.seq, values=self.sequences(), width=28,
                                    postcommand=lambda: self.seq_box.configure(values=self.sequences()))
        self.seq.trace_add("write", lambda *a: self.show_seq_about())
        self.seq.set(FIRST_SEQ if FIRST_SEQ in self.sequences() else next(iter(self.sequences()), ""))
        self.seq_box.grid(row=0, column=0, columnspan=4, sticky="we")
        ttk.Button(f, text="Browse...", command=self.browse).grid(row=0, column=4, sticky="we")
        ttk.Label(f, textvariable=self.seq_about, wraplength=430, justify="left").grid(row=1, column=0, columnspan=5, sticky="w")
        timing = ttk.Frame(f)
        timing.grid(row=2, column=0, columnspan=5, sticky="w", pady=4)
        for label, var, lo, hi, step, cmd in (("Move s", self.move, 0.2, 20, 0.5, None), ("Hold s", self.hold, 0, 20, 0.5, None),
                                              ("Speed x", self.speed, 0.1, 10, 0.25, self.send_speed),
                                              ("Playback x", self.playback, 0.25, 8, 0.25, self.send_playback)):
            ttk.Label(timing, text=label).pack(side="left", padx=(0, 4))
            spinbox(timing, var, lo, hi, step, 5, command=cmd).pack(side="left", padx=(0, 10))
        for col, (label, cmd) in enumerate((("Play", self.play), ("Pause", lambda: self.send("pause")),
                                            ("Resume", lambda: self.send("resume")), ("Restart", lambda: self.play(fresh=True)),
                                            ("Stop", lambda: self.send("stop")))):
            ttk.Button(f, text=label, command=cmd, width=7).grid(row=3, column=col, sticky="we")
            f.columnconfigure(col, weight=1)  # buttons share the width evenly
        count = ttk.Frame(f)
        count.grid(row=4, column=0, columnspan=5, sticky="w", pady=(4, 0))
        ttk.Label(count, text="Countdown s").pack(side="left", padx=(0, 4))
        spinbox(count, self.countdown, 0, 60, 1, 5).pack(side="left", padx=(0, 10))
        ttk.Label(count, text="T-5 ... T-1 in the viewer before Play and Restart (0: none)",
                  foreground="gray40").pack(side="left")
        ttk.Label(f, text="Speed x makes the moves faster (the physics changes); Playback x only shows the "
                          "simulation faster than real time.", wraplength=430, foreground="gray40").grid(row=5, column=0, columnspan=5, sticky="w")

        f = frame("World")
        ttk.Checkbutton(f, text="Gravity", variable=self.gravity, command=self.send_gravity).grid(row=0, column=0)
        spinbox(f, self.g, 0, 30, 0.5, 5, command=self.send_gravity).grid(row=0, column=1)
        ttk.Label(f, text="m/s²  ").grid(row=0, column=2)
        ttk.Button(f, text="Straighten", command=lambda: self.send("straighten")).grid(row=0, column=3)
        ttk.Button(f, text="Reset pose", command=self.reset_pose).grid(row=0, column=4, sticky="w")
        self.physics_btn = ttk.Button(f, text="⏸  Pause physics", command=lambda: self.send("physics", run=not self.sim_running))
        self.physics_btn.grid(row=1, column=0, columnspan=5, sticky="we", pady=(6, 0))
        f.columnconfigure(4, weight=1)

        f = frame("Spin (through the centre of mass; also: leftover spin when it's let go)", right)
        spinbox(f, self.spin_rate, -360, 360, 0.5).grid(row=0, column=0)
        ttk.Label(f, text="deg/s about").grid(row=0, column=1, padx=(4, 4))
        ttk.Combobox(f, textvariable=self.spin_axis, values=SPIN_AXES, state="readonly", width=15).grid(row=0, column=2)
        ttk.Button(f, text="Spin", command=lambda: self.send("spin", rate=self.spin_rate.get(),
                                                             axis=self.spin_axis.get()[-2])).grid(row=0, column=3, padx=(6, 0))
        ttk.Button(f, text="Stop spin", command=lambda: self.send("spin", rate=0.0, axis="all")).grid(row=0, column=4, padx=6)
        ttk.Checkbutton(f, text="Also spin like this when Play starts (the spin at release)",
                        variable=self.spin_at_play).grid(row=1, column=0, columnspan=5, sticky="w", pady=(4, 0))

        f = frame("Flight: the box on the plane, and one parabola (Novespace A310)", right)
        box = ttk.Frame(f)
        box.grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(box, text="In the flight box, m:", variable=self.box_on).pack(side="left")
        for label, var in zip(("along", "across", "high"), self.box_m):
            ttk.Label(box, text=label).pack(side="left", padx=(6, 2))
            spinbox(box, var, 0.3, 5, 0.1, 4).pack(side="left")
        held = ttk.Frame(f)
        held.grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(held, text="Strings hold modules").pack(side="left")
        ttk.Entry(held, textvariable=self.strings_text, width=11).pack(side="left", padx=4)
        ttk.Label(held, text="1.8 g phases, s").pack(side="left", padx=(6, 2))
        spinbox(held, self.hyper, 7, 60, 1, 4).pack(side="left")
        ttk.Checkbutton(held, text="g-jitter", variable=self.jitter).pack(side="left", padx=(8, 0))
        times = ttk.Frame(f)
        times.grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(times, text="Let go at T+").pack(side="left")
        spinbox(times, self.release, 0, 20, 0.5, 4).pack(side="left")
        ttk.Label(times, text="s, sequence at T+").pack(side="left", padx=(4, 0))
        spinbox(times, self.start, 0, 20, 0.5, 4).pack(side="left")
        ttk.Label(times, text="s (T+0: zero g begins)").pack(side="left", padx=(4, 0))
        ttk.Button(f, text="✈  Fly a parabola", command=self.fly).grid(row=3, column=0, sticky="we", pady=(6, 0))
        ttk.Label(f, text="The box and the strings need Rebuild; the rest applies at Fly. Fly runs 1 g, the 1.8 g pull-up "
                          "(held by the strings), zero g, the pull-out onto the net.", wraplength=470,
                  foreground="gray40").grid(row=4, column=0, sticky="w")

        f = frame("Record", right)
        ttk.Checkbutton(f, text="Log to logs/", variable=self.logging, command=lambda: self.send("log", on=self.logging.get())).grid(row=0, column=0)
        ttk.Button(f, text="Screenshot", command=lambda: self.send("screenshot")).grid(row=0, column=1, padx=6)
        ttk.Button(f, text="Open folder", command=lambda: subprocess.run(["open", HERE])).grid(row=0, column=2)

        f = frame("Readouts", right)
        ttk.Label(f, textvariable=self.status, font=("Menlo", 11), justify="left").pack(anchor="w")

        self.open_viewer()
        self.poll()

    def sequences(self):
        return sorted(os.path.basename(p) for p in glob.glob(os.path.join(SEQ_DIR, "*.csv")))

    def setups(self):
        return sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(SETUP_DIR, "*.toml")))

    def show_about(self, note=""):
        """The picked setup's line on what it shows, and what's in it (its world, chain and end effectors)."""
        path = os.path.join(SETUP_DIR, self.setup_name.get() + ".toml")
        self.about_text = setup_file.about(path) if os.path.exists(path) else ""
        try:
            inside = "In it: " + setup_file.summary(setup_file.load(path))
        except (OSError, ValueError) as e:
            inside = f"Can't load it: {e}"
        self.about.set("\n".join(x for x in (self.about_text or "(no line saying what it shows)", inside, note) if x))

    def show_seq_about(self):
        """The picked sequence's first line, and the setups that use it: a sequence is only the moves, so one written
        for a setup (the pole climb's pole, the walk's suction cups) needs that setup loaded."""
        path = os.path.join(SEQ_DIR, self.seq.get())
        if not self.seq.get() or not os.path.isfile(path):
            self.seq_about.set("")
            return
        used = setup_file.users(path)
        self.seq_about.set("\n".join(x for x in (setup_file.about(path), f"Used by setups: {', '.join(used)} (Load one for "
                                                  "its world and end effectors)" if used else "No setup uses it.") if x))

    def show_ends(self):
        if hasattr(self, "ends"):
            for box in self.ends:
                box.show(other_placed=self.ends[0].placed())

    def current(self):
        """The loaded setup with every field of this window laid over it."""
        s, seq = copy.deepcopy(self.loaded), self.seq.get()
        spin = [0.0, 0.0, 0.0]
        if self.spin_at_play.get():
            spin["xyz".index(self.spin_axis.get()[-2])] = float(self.spin_rate.get())
        s.update(sequence=os.path.join(SEQ_DIR, seq) if seq else "", move_s=float(self.move.get()),
                 hold_s=float(self.hold.get()), speed=float(self.speed.get()), playback=float(self.playback.get()),
                 gravity_m_s2=float(self.g.get()) if self.gravity.get() else 0.0, spin_deg_s=spin, air=self.air.get(),
                 preset=self.preset.get(), modules=int(self.modules.get()), slip_ring=self.slip.get(),
                 rod={**(s["rod"] or ROD), "diameter_mm": float(self.rod_mm.get())} if self.rod.get() else {},
                 ee1=self.ends[0].get(s["ee1"]), ee_n=self.ends[1].get(s["ee_n"]),
                 flight={**FLIGHT_DEFAULTS, **s["flight"], **self.flight_fields()} if self.box_on.get() else {})
        return s

    def flight_fields(self):
        """The Flight box's fields, as a setup's [flight] entries (raises ValueError on a string list it can't read)."""
        try:
            strings = sorted(int(x) for x in self.strings_text.get().replace(",", " ").split())
        except ValueError:
            raise ValueError(f"Strings hold modules: {self.strings_text.get()!r}: module numbers, e.g. 1 5 8 12") from None
        return dict(box_m=[float(v.get()) for v in self.box_m], strings=strings, release_s=float(self.release.get()),
                    start_s=float(self.start.get()), hyper_s=float(self.hyper.get()), jitter=self.jitter.get())

    def demo(self, name):
        """One of the six demos: load its setup, and play it once the viewer is up."""
        self.setup_name.set(name)
        self.autoplay = self.load_setup()

    def load_setup(self):
        """Fill every field from the picked setup, then rebuild and open the viewer. False if it couldn't."""
        if not self.setup_name.get():
            self.about.set("Pick a setup first.")
            return False
        try:
            s = setup_file.load(os.path.join(SETUP_DIR, self.setup_name.get() + ".toml"))
        except (OSError, ValueError) as e:  # a TOML syntax error is a ValueError too
            self.about.set(f"Can't load it: {e}")
            return False
        self.loaded = s
        self.preset.set(s["preset"]), self.modules.set(s["modules"]), self.slip.set(s["slip_ring"])
        self.air.set(s["air"]), self.rod.set(bool(s["rod"])), self.box_on.set(bool(s["flight"]))
        f = {**FLIGHT_DEFAULTS, "strings": default_strings(s["modules"]), **s["flight"]}
        for var, x in zip(self.box_m, f["box_m"]):
            var.set(x)
        self.strings_text.set(" ".join(map(str, f["strings"]))), self.jitter.set(f["jitter"])
        self.release.set(f["release_s"]), self.start.set(f["start_s"]), self.hyper.set(f["hyper_s"])
        if s["rod"]:
            self.rod_mm.set(s["rod"]["diameter_mm"])
        self.ends[0].set(s["ee1"]), self.ends[1].set(s["ee_n"])
        seq = s["sequence"]
        self.seq.set(os.path.basename(seq) if os.path.dirname(seq) == SEQ_DIR else seq)
        self.move.set(s["move_s"]), self.hold.set(s["hold_s"]), self.speed.set(s["speed"]), self.playback.set(s["playback"])
        self.gravity.set(s["gravity_m_s2"] > 0)
        if s["gravity_m_s2"] > 0:
            self.g.set(s["gravity_m_s2"])
        rate, axis, warning = setup_file.single_axis_spin(s["spin_deg_s"])
        self.spin_at_play.set(rate != 0)
        if rate:
            self.spin_rate.set(rate)
            self.spin_axis.set(next(a for a in SPIN_AXES if a[-2] == axis))
        self.show_about(warning)
        return self.open_viewer()

    def save_setup(self):
        """Save every field (and whatever the loaded setup had that has no field) as a setup file."""
        path = filedialog.asksaveasfilename(initialdir=SETUP_DIR, defaultextension=".toml", filetypes=[("Setup", "*.toml")])
        if not path:
            return
        about = simpledialog.askstring("Save setup", "One line on what this run shows:", initialvalue=self.about_text,
                                       parent=self.root)
        if about is None:
            return
        try:
            setup_file.save(path, self.current(), about)
            setup_file.load(path)  # say so now if Load would refuse the file
        except (tk.TclError, ValueError, OSError) as e:  # e.g. a number box holding something that isn't a number
            self.about.set(f"Saved, but Load would refuse it: {e}" if os.path.exists(path) else f"Can't save it: {e}")
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
        """Write the current settings to panel_setup.toml and (re)start the viewer from it."""
        try:
            setup_file.save(PANEL_SETUP, self.current(), "What the control window last opened the viewer with.")
            built = setup_file.load(PANEL_SETUP)
        except (tk.TclError, ValueError, OSError) as e:
            self.about.set(f"Can't open the viewer with these settings: {e}")
            return False
        self.logging.set(False)
        self.played = False
        self.loaded_flight = bool(built["flight"])  # what the viewer was built with
        self.link.open(PANEL_SETUP)
        self.status.set("starting the viewer...")
        return True

    def send(self, cmd, **args):
        if not self.link.send(cmd, **args):
            self.status.set("viewer not connected: click 'Rebuild + open viewer'")

    def play(self, fresh=False):
        """Play the sequence. A second take (or Restart) starts again from the start pose, so every take is the same."""
        path = self.seq.get()
        if not path:
            self.status.set("pick a fold sequence first")
            return
        if fresh or self.played:
            self.send("reset")
        self.played = True
        self.send("speed", value=self.speed.get())
        self.send_playback()
        try:
            count = self.countdown.get()
        except tk.TclError:  # the box is empty or being typed into
            count = COUNTDOWN_S
        if count > 0:  # the viewer holds the spin and play, and the physics, until T-0
            self.send("countdown", s=count)
        if self.spin_at_play.get():
            self.send("spin", rate=self.spin_rate.get(), axis=self.spin_axis.get()[-2])
        self.send("play", file=os.path.join(SEQ_DIR, path), move=self.move.get(), hold=self.hold.get())  # abs path stays

    def fly(self):
        """Fly one parabola: a fresh start (held by the strings), the countdown, then the viewer runs it all."""
        if not self.loaded_flight:
            self.status.set("the viewer has no flight box: tick 'In the flight box', then 'Rebuild + open viewer'")
            return
        try:
            f = self.flight_fields()
        except (ValueError, tk.TclError) as e:
            self.status.set(str(e))
            return
        self.send("reset")
        self.played = True
        self.send("speed", value=self.speed.get())
        self.send_playback()
        try:
            count = self.countdown.get()
        except tk.TclError:
            count = COUNTDOWN_S
        if count > 0:
            self.send("countdown", s=count)
        seq = self.seq.get()
        self.send("fly", file=os.path.join(SEQ_DIR, seq) if seq else "", move=self.move.get(), hold=self.hold.get(),
                  release=f["release_s"], start=f["start_s"], hyper=f["hyper_s"], jitter=f["jitter"])

    def reset_pose(self):
        self.send("reset")
        self.played = False

    def send_speed(self):
        self.send("speed", value=self.speed.get())

    def send_playback(self):
        try:
            self.send("playback", value=float(self.playback.get()))
        except tk.TclError:  # the box is being typed into
            pass

    def send_gravity(self):
        self.send("gravity", g=self.g.get() if self.gravity.get() else 0.0)

    def poll(self):
        """Every 100 ms: accept the viewer's connection, read its readouts, notice if it was closed."""
        connected, info, closed = self.link.poll()
        if connected:
            self.send_playback()
            if self.autoplay:
                self.autoplay = False
                self.play()
        if info:
            self.status.set("\n".join(f"{k:<11}{v}" for k, v in info.items()))
            self.sim_running = info["physics"] == "running"  # also follows the Space key in the viewer
            self.physics_btn.configure(text="⏸  Pause physics" if self.sim_running else "▶  Resume physics")
        if closed:
            self.status.set("viewer closed: click 'Rebuild + open viewer'")
        self.root.after(100, self.poll)

    def quit(self):
        self.link.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Panel(root)
    root.mainloop()
