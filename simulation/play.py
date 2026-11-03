"""Drive the chain: interactive viewer, fold-sequence playback, and logging.

  mjpython play.py                                  viewer: Control-panel sliders + keyboard
  mjpython play.py --seq ../sequences/fold_sequence.csv       viewer, playing the sequence
  .venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless    no viewer, run and log only
  .venv/bin/python play.py --setup setups/1a_turn.toml --headless   a whole run from a setup file (setups/)
  mjpython play.py --gravity 9.81                   start with Earth gravity: the chain drops onto the floor
  .venv/bin/python play.py --seq ../sequences/reorient_loop.csv --headless --speed 4 --spin 0 0 1 --air
      flight-like: the sequence 4x faster (to fit a parabola), let go with 1 deg/s of spin, cabin air
  .venv/bin/python play.py --setup setups/1d_flight_turn.toml --headless
      a setup with [flight] flies a whole parabola: held by strings through the 1.8 g pull-up, let go at zero g, onto the net
The control window (panel.py, or double-click 'Chain Sim.command') starts this with --panel and sends it commands.

Keyboard (viewer): PageUp / PageDown select module, Up / Down nudge its pitch,
Right / Left nudge its yaw, Space pauses / resumes the physics.
The selected module turns orange.
On a Mac the viewer needs mjpython (it ships in .venv/bin); --headless works with plain python.
"""
import argparse
import csv
import json
import math
import os
import queue
import re
import socket
import sys
import threading
import time

import mujoco
import numpy as np

import setup_file
from make_chain import CHAIN_ROLL, FLIGHT, NOZZLE_RGBA, STRING_RGBA, attach_point, build, leading_end, level_axis

HERE = os.path.dirname(os.path.abspath(__file__))
AXES = dict(x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1))  # world axes: x across the start pose, y along it, z vertical
PARABOLA_S = FLIGHT["parabola_s"]                  # seconds of microgravity per parabola
STEP_DEG = 5.0      # keyboard nudge per press
LOG_DT = 0.01       # seconds between log rows and between sequence target updates
KEYS = dict(next=266, prev=267, pitch_up=265, pitch_down=264, yaw_up=262, yaw_down=263, pause=32)  # GLFW codes
BASE_RGBA, SELECT_RGBA = (0.78, 0.78, 0.80, 1), (1.0, 0.6, 0.2, 1)
CAPTURE = 0.015     # m: an anchor told to hold grabs when its grip face comes this close to something fixed
PULL_IN = 0.2       # s: having grabbed short of the surface, the clamp pulls the face onto it gently for this long
SOFT_SOLREF = (0.05, 1.0)  # the clamp's weld while it pulls in; then it's back to the stiff make_chain.CLAMP_SOLREF
LOAD_TAU = 0.1      # s: end-effector loads are smoothed over this; the stiff clamp's raw force is spiky
CUP_TOUCH = 0.003   # m: a cup this close to a fixed surface counts as on it
EE_COLUMN = re.compile(r"(ee\d+)_(grip|n[0-4])$")  # end-effector columns in a sequence CSV
CALL_OFF = ("pause", "stop", "straighten", "reset", "quit")  # sent during a countdown, these call the take off
TOUCH = 0.001       # m: a shell this close to the box (walls, lid, net) touches it
WINDOW_NAME = "CATENA viewer"  # macOS saves the viewer window's size and place under this name, and reopens it there


def actuator(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def modules(model):
    """How many chain modules: bodies m1, m2, ... (arms and end effectors don't count)."""
    return sum(1 for b in range(model.nbody) if model.body(b).name[:1] == "m" and model.body(b).name[1:].isdigit())


def joint_servos(model):
    """How many joint servos. They come first among the actuators, in CSV column order."""
    return int(np.sum(model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT))


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def read_sequence(path, model):
    """Keyframes from a CSV as joint-servo targets (rad). Columns are degrees in the order m1_yaw, m1_pitch, ...,
    mN_yaw, mN_pitch; an optional header row names them. mN_pitch has no servo, nor has m1_yaw without ee1: ignored.
    End-effector columns (ee1_grip, ...) are read_ee's."""
    names, rows = csv_rows(path, modules(model))
    nj = joint_servos(model)
    keys = []
    for r in rows:
        q = np.zeros(nj)
        for name, val in zip(names, map(float, r)):
            if EE_COLUMN.match(name):
                continue
            a = actuator(model, name)
            if 0 <= a < nj:
                q[a] = math.radians(val)
            elif val != 0:
                print(f"warning: {name} has no servo, ignoring {val:g} deg")
        keys.append(q)
    return np.array(keys)


def csv_rows(path, n):
    """A sequence CSV's column names and rows (strings), skipping # comments. Without a header row the columns are the
    joints of an n-module chain."""
    with open(path) as f:
        rows = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith("#")]
    names = [f"m{i}_{j}" for i in range(1, n + 1) for j in ("yaw", "pitch")]
    try:
        float(rows[0][0])
    except ValueError:
        names, rows = [c.strip() for c in rows[0]], rows[1:]
    for r in rows:
        assert len(r) == len(names), f"{path}: expected {len(names)} columns, got {len(r)}"
    return names, rows


def read_ee(path, model):
    """A sequence's end-effector columns, {"ee1_grip": array per row, ...}: grip 1 = hold, 0 = let go (anchor, suction);
    n0-n4 = nozzle throttle 0-1 (thruster). A column the model can't act on is dropped with one warning."""
    names, rows = csv_rows(path, modules(model))
    out = {}
    for k, name in enumerate(names):
        found = EE_COLUMN.match(name)
        if not found:
            continue
        end, what = found.groups()
        able = (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{end}_clamp") >= 0 or actuator(model, f"{end}_suck") >= 0
                if what == "grip" else actuator(model, f"{end}_{what}") >= 0)
        if able:
            out[name] = np.array([float(r[k]) for r in rows])
        else:
            print(f"warning: {name}: there's no {'anchor or suction cup' if what == 'grip' else 'thruster'} on {end}, ignoring it")
    return out


def coil_axis(pitch, yaw, length):
    """A chain whose every pitch and yaw are the same is a coil (a helix). Its axis in m1's frame: (unit u pointing the
    way the chain winds along it, a point on it, its radius)."""
    R = axis_rotation((1, 0, 0), math.degrees(pitch)) @ axis_rotation((0, 0, 1), math.degrees(yaw))
    t = axis_rotation((1, 0, 0), math.degrees(pitch)) @ np.array([0, 0, length])
    th = math.acos(min(1.0, max(-1.0, (np.trace(R) - 1) / 2)))
    u = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))
    c = np.linalg.lstsq(np.eye(3) - R, t - (u @ t) * u, rcond=None)[0]
    return u * np.sign(u @ t), c, np.linalg.norm(c - (c @ u) * u)


def axis_rotation(axis, deg):
    q, R = np.zeros(4), np.zeros(9)
    mujoco.mju_axisAngle2Quat(q, np.asarray(axis, float), math.radians(deg))
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


def pitch_matrix(theta):
    """The cabin's turn by the plane's pitch: about x (across the cabin), nose (+y) up for theta > 0."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def hermite(u, a, b, ra, rb, span):
    """From a to b as u goes 0 -> 1, leaving at rate ra and arriving at rate rb (per second, over span seconds)."""
    return ((2 * u ** 3 - 3 * u ** 2 + 1) * a + (u ** 3 - 2 * u ** 2 + u) * span * ra + (-2 * u ** 3 + 3 * u ** 2) * b
            + (u ** 3 - u ** 2) * span * rb)


class Parabola:
    """The g the box feels through one parabola (ITF §3.10, numbers in the spec's [flight]): level flight at 1 g, the
    pull-up to 1.8 g, zero g with its leftover jitter, the pull-out, level again. at(t) gives the phase, the g vector
    (the box's frame, in g: x across the cabin, y along it towards the nose, z up), T (the time since zero g began, s)
    and the plane's pitch (rad). In zero g the plane flies a free-fall arc, nose up pitch_deg to nose down pitch_deg, so
    its pitch is atan(tan(pitch_deg) (1 - 2T / parabola_s)); the pull-up and pull-out join that smoothly to level.
    level (g) flies a Moon (0.16) or Mars (0.38) parabola instead: the same length, a flatter arc (it falls at (1 - level)
    g, so tan of the nose-up is (1 - level) tan(pitch_deg)); 1 is level flight between the two 1.8 g phases."""
    LEVEL_S, UP_S, DOWN_S = 3.0, 3.0, 4.0  # level flight before and after; the 1 <-> 1.8 g and 1.8 <-> 0 g ramps

    def __init__(self, hyper_s, jitter, seed, level=0.0):
        g, hold = FLIGHT["hypergravity_g"], hyper_s - self.UP_S - self.DOWN_S
        self.free = "zero g" if level == 0 else f"{level:g} g"  # the phase the experiment runs in
        steps = [("level", self.LEVEL_S, 1), ("pull-up", self.UP_S, g), ("pull-up", hold, g), (f"into {self.free}", self.DOWN_S, level),
                 (self.free, FLIGHT["parabola_s"], level), ("pull-out", self.DOWN_S, g), ("pull-out", hold, g),
                 ("pull-out", self.UP_S, 1), ("level", self.LEVEL_S, 1)]
        self.knots, t = [(0.0, 1.0, "level")], 0.0
        for name, secs, g_end in steps:
            t += secs
            self.knots.append((t, g_end, name))
        self.t0, self.end = self.knots[4][0], t  # zero g begins; the flight ends
        self.top = math.atan((1 - level) * math.tan(math.radians(FLIGHT["pitch_deg"])))
        self.rate = -math.sin(2 * self.top) / FLIGHT["parabola_s"]  # the arc's pitch rate as zero g begins and ends
        rng, self.waves = np.random.default_rng(seed), []  # per axis, three slow waves (2-8 s) adding up to jitter_g at most
        for a in FLIGHT["jitter_g"] if jitter else (0, 0, 0):
            c = rng.uniform(0.3, 1, 3)
            self.waves.append([(a * ci / c.sum(), rng.uniform(2, 8), rng.uniform(0, 2 * math.pi)) for ci in c])

    def at(self, t):
        k = next((k for k in range(1, len(self.knots)) if t < self.knots[k][0]), len(self.knots) - 1)
        (t1, g1, _), (t2, g2, name) = self.knots[k - 1], self.knots[k]
        u = min(1.0, max(0.0, (t - t1) / (t2 - t1)))
        g = g1 + (0.5 - 0.5 * math.cos(math.pi * u)) * (g2 - g1)
        T = t - self.t0  # the jitter runs on T, so every take (whatever its 1.8 g phases) gets the same
        jitter = [sum(c * math.sin(2 * math.pi * T / period + phase) for c, period, phase in axis) for axis in self.waves]
        on = name == self.free
        return name, np.array([jitter[0] * on, jitter[1] * on, -g + jitter[2] * on]), T, self.pitch(t)

    def pitch(self, t):
        up, z, out = self.knots[1][0], self.knots[5][0], self.knots[8][0]  # pull-up starts, zero g ends, level again
        if t <= up or t >= out:
            return 0.0
        if t < self.t0:
            return hermite((t - up) / (self.t0 - up), 0.0, self.top, 0.0, self.rate, self.t0 - up)
        if t <= z:
            return math.atan(math.tan(self.top) * (1 - 2 * (t - self.t0) / FLIGHT["parabola_s"]))
        return hermite((t - z) / (out - z), -self.top, 0.0, self.rate, 0.0, out - z)


class End:
    """An end effector as the session sees it: what it is, what it's been told to do and what it's doing."""

    def __init__(self, model, name):
        find = lambda kind, what: mujoco.mj_name2id(model, kind, what)
        self.name, self.body = name, model.body(name).id
        self.clamp = find(mujoco.mjtObj.mjOBJ_EQUALITY, f"{name}_clamp")
        self.suck = actuator(model, f"{name}_suck")
        self.nozzles = [a for k in range(5) if (a := actuator(model, f"{name}_n{k}")) >= 0]
        self.sites = [model.site(f"{name}_n{k}").id for k in range(len(self.nozzles))]
        self.kind = "anchor" if self.clamp >= 0 else "suction" if self.suck >= 0 else "thruster" if self.nozzles else "plain"
        self.tip = find(mujoco.mjtObj.mjOBJ_GEOM, f"{name}_tip")
        self.mocap = model.body_mocapid[model.body(f"{name}_hold").id] if self.clamp >= 0 else -1
        self.stiff = model.eq_solref[self.clamp].copy() if self.clamp >= 0 else None
        self.cup = find(mujoco.mjtObj.mjOBJ_GEOM, f"{name}_cup")
        self.limit = float(model.actuator_gainprm[self.suck][0]) if self.suck >= 0 else 0.0  # a cup's pull limit (N)
        self.geoms = {g for g in range(model.ngeom) if model.geom_bodyid[g] in (self.body, model.geom_bodyid[self.cup])}
        self.reset()

    def reset(self):
        self.armed = self.clamped = self.held = False
        self.load = self.torque = self.peak = 0.0  # N (clamp force or cup pull), N m (clamp torque), peak load
        self.pops, self.pull_in_until = 0, None    # times it let go without being told to; end of a clamp's pull-in

    def state(self):
        return ("holding" if self.held else "ready to grab" if self.armed else "let go") if self.kind in ("anchor", "suction") \
            else self.kind


def target(keys, t, move, hold):
    """Hold each keyframe for `hold` s, then ease to the next one over `move` s."""
    k = int(t // (move + hold))
    if k >= len(keys) - 1:
        return keys[-1]
    u = min(1.0, max(0.0, (t - k * (move + hold) - hold) / move))
    return keys[k] + (0.5 - 0.5 * math.cos(math.pi * u)) * (keys[k + 1] - keys[k])


class Logger:
    """Writes time, chain COM, total angular momentum about the COM, max |actuator torque|, contacts, and what the
    flight IMU will report: its module's attitude (quaternion, world frame) and rotation rate (rad/s, its own frame)."""

    def __init__(self, path, nj, extra=()):
        self.path, self.nj = path, nj  # nj: joint servos, the first nj actuators. extra: more columns, at the end
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["time", "com_x", "com_y", "com_z", "L_x", "L_y", "L_z", "L_norm", "max_torque", "contacts",
                         "imu_qw", "imu_qx", "imu_qy", "imu_qz", "imu_wx", "imu_wy", "imu_wz", *extra])

    def __call__(self, data, extra=()):
        com, L = data.sensor("com").data, data.sensor("angmom").data
        self.w.writerow([f"{data.time:.3f}", *(f"{x:.7g}" for x in com), *(f"{x:.4g}" for x in L),
                         f"{np.linalg.norm(L):.4g}", f"{np.abs(data.actuator_force[:self.nj]).max():.4g}", data.ncon,
                         *(f"{x:.6g}" for x in data.sensor("imu_quat").data),
                         *(f"{x:.4g}" for x in data.sensor("imu_gyro").data), *extra])

    def close(self):
        self.f.close()


class Session:
    """Simulation state shared by the keyboard, the control window and the main loop."""

    def __init__(self, model, data, log_path=None, setup=None):
        self.m, self.d = model, data
        self.keys, self.seq_name, self.move, self.hold = None, "", 3.0, 1.0
        self.seq_t, self.playing, self.speed = 0.0, False, 1.0
        self.playback = 1.0  # the viewer shows this many seconds of simulation per second; the physics doesn't change
        self.running = True  # False freezes the physics (time, motion, sequence clock, log)
        self.nj = joint_servos(model)  # sequences drive these; other actuators (end effectors) come after them
        self.mid = model.body(f"m{max(1, modules(model) // 2)}").id  # middle module (m6 of 12): its heading = "turned"
        self.end_keys = ("ee1", f"ee{modules(model)}")  # a setup's ee1 and ee_n, by their names in the model
        self.ends = {e: End(model, e) for e in self.end_keys if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, e) >= 0}
        cabin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cabin")  # the flight box turns with the plane
        self.cabin = cabin  # its body; play drives its hinge, cabin_pitch, with the plane's pitch
        if cabin >= 0:
            self.cabin_q, self.cabin_v = model.jnt_qposadr[model.joint("cabin_pitch").id], model.jnt_dofadr[model.joint("cabin_pitch").id]
        self.static = {0, cabin} - {-1}  # bodies that are fixed things: the world, and the cabin with the box on it
        self.fixed = [g for g in range(model.ngeom) if model.geom_bodyid[g] in self.static]  # floor, plates, rod, box
        self.box = [g for g in self.fixed if model.geom(g).name.startswith(("box_", "net"))]  # the flight box and its net
        grips = tuple(e.name for e in self.ends.values() if e.kind in ("anchor", "suction"))  # their shells meet surfaces
        self.shells = [g for g in range(model.ngeom) if model.geom(g).name.endswith("_sphere")  # on purpose: not touches
                       and not model.geom(g).name.startswith(grips or ("-",))]
        self.strings = [t for t in range(model.ntendon) if "_string" in model.tendon(t).name]
        self.net = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "net")
        self.setup, self.row0, self.ee_cols, self.ee_row = None, None, {}, None
        self.flight, self.fly_t = None, None  # the setup's [flight], and the time into a parabola while flying
        self.turn_axis = self.pitch_axis(0.0)  # what turned is about: the pitch axes as the chain starts
        self.log = Logger(log_path, self.nj, self.log_names()) if log_path else None
        self.use_setup(setup) if setup else self.reset()

    def log_names(self):
        """The log's columns after the standard ones: four per fitted end, then the flight's if the box is there."""
        return [f"{e}_{c}" for e in self.ends for c in ("held", "load_N", "torque_Nm", "thrust_N")] + \
            (["g_x", "g_y", "g_z", "box_gap_mm", "string_N"] if self.box else [])

    def log_columns(self):
        ends = [x for e in self.ends.values() for x in (int(e.held), f"{e.load:.4g}", f"{e.torque:.4g}", f"{self.thrust(e):.4g}")]
        flight = [*(f"{x / 9.81:.4g}" for x in self.m.opt.gravity), f"{self.gap * 1000:.1f}", f"{self.tension():.4g}"]
        return ends + (flight if self.box else [])

    def thrust(self, e):
        return float(np.sum(self.d.actuator_force[e.nozzles])) if e.nozzles else 0.0

    def use_setup(self, setup):
        """Start from this setup from now on (and on Reset pose): the chain in its sequence's first row, attached where
        the setup says."""
        self.setup, self.flight = setup, setup["flight"] or None
        self.turn_axis = self.pitch_axis(self.flight["roll_deg"] if self.flight else 0.0)
        self.row0 = read_sequence(setup["sequence"], self.m)[0] if setup["sequence"] else np.zeros(self.nj)
        self.reset()

    def place(self):
        """The setup's start pose: joints (and servo targets) at the sequence's first row; if an end is attached, the
        chain moved so that end sits on its surface, holding on."""
        m, d = self.m, self.d
        for a in range(self.nj):
            d.qpos[m.jnt_qposadr[m.actuator_trnid[a, 0]]] = d.ctrl[a] = self.row0[a]
            if m.actuator_actadr[a] >= 0:
                d.act[m.actuator_actadr[a]] = self.row0[a]
        ends = [self.setup["ee1"], self.setup["ee_n"]]
        lead = leading_end(ends)
        if lead is None and "wrap_m" in self.setup["rod"]:
            self.wrap()
        if lead is not None:
            e = self.ends[self.end_keys[lead]]
            point, normal = attach_point(ends[lead]["attach"], self.setup["rod"])
            n = np.array(normal, float)
            x = axis_rotation(n, ends[lead]["attach"].get("roll_deg", 0.0)) @ np.array(level_axis(normal), float)
            z = n if e.name == "ee1" else -n  # the end's +z points along the chain from ee1, away from it from ee_n
            frame = np.column_stack([x, np.cross(z, x), z])
            centre = np.array(point) + n * m.geom(f"{e.name}_sphere").size[0]
            d.qpos[:3], d.qpos[3:7] = 0, (1, 0, 0, 0)  # m1 at the origin: the end's pose in m1's frame
            mujoco.mj_kinematics(m, d)
            R1 = frame @ d.xmat[e.body].reshape(3, 3).T
            d.qpos[:3] = centre - R1 @ d.xpos[e.body]
            mujoco.mju_mat2Quat(d.qpos[3:7], R1.flatten())
            mujoco.mj_forward(m, d)
            self.grip(e.name, True)
            if e.kind == "anchor":
                self.clamp(e)
            for key, setting in zip(self.end_keys, ends):
                other = self.ends.get(key)
                if other and other is not e and other.kind in ("anchor", "suction") and setting["attach"]:
                    self.grip(key, True)  # attached too: it holds whatever it lands on
        mujoco.mj_forward(m, d)

    def wrap(self):
        """Start wrapped around the rod: the first row is a coil (from m1_pitch and m2_yaw); put its axis on the rod's,
        m1 at wrap_m along it, winding toward the rod's 'to' end."""
        m, d, rod = self.m, self.d, self.setup["rod"]
        q = {m.actuator(a).name: self.row0[a] for a in range(self.nj)}
        u, c, _ = coil_axis(q["m1_pitch"], q["m2_yaw"], np.linalg.norm(m.body_pos[m.body("m2").id]))
        a, b = np.array(rod["from"]), np.array(rod["to"])
        e = (b - a) / np.linalg.norm(b - a)
        k, cos = np.cross(u, e), u @ e  # the rotation taking u to e
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R1 = np.eye(3) + K + K @ K / (1 + cos) if cos > -1 + 1e-9 else axis_rotation(np.cross(u, (1, 0, 0)), 180)
        d.qpos[:3] = a + e * rod["wrap_m"] - R1 @ c
        mujoco.mju_mat2Quat(d.qpos[3:7], R1.flatten())

    def grip(self, name, on):
        """Hold (grab whatever fixed thing the end touches, now or later) or let go."""
        e = self.ends[name]
        e.armed = bool(on)
        if e.kind == "anchor" and not on and e.clamped:
            self.d.eq_active[e.clamp], e.clamped, e.held = 0, False, False
        if e.kind == "suction":
            self.d.ctrl[e.suck] = float(bool(on))

    def nozzle(self, name, k, throttle):
        """Set one nozzle's throttle (0-1); it lights up while it fires."""
        e = self.ends[name]
        if k < len(e.nozzles):
            self.d.ctrl[e.nozzles[k]] = min(1.0, max(0.0, float(throttle)))
            self.m.site_rgba[e.sites[k]] = [float(x) for x in NOZZLE_RGBA[int(self.d.ctrl[e.nozzles[k]] > 0)].split()]

    def nozzles_off(self):
        for e in self.ends.values():
            for k in range(len(e.nozzles)):
                self.nozzle(e.name, k, 0.0)

    def clamp(self, e, shift=np.zeros(3)):
        """Weld the end where it is, moved by `shift` (onto the surface it's about to touch): its hold body goes there.
        With a shift, the weld starts soft and pulls the end in over PULL_IN, like a latch closing."""
        d = self.d
        d.mocap_pos[e.mocap] = d.xpos[e.body] + shift
        d.mocap_quat[e.mocap] = d.xquat[e.body]
        if self.cabin >= 0:  # in the flight box, the hold rides on the cabin: remember it in the box's frame
            R, c = self.box_frame(), d.xpos[self.cabin]
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, (R.T @ d.xmat[e.body].reshape(3, 3)).flatten())
            e.hold_box = (R.T @ (d.mocap_pos[e.mocap] - c), q)
        soft = np.linalg.norm(shift) > 1e-4
        self.m.eq_solref[e.clamp] = SOFT_SOLREF if soft else e.stiff
        e.pull_in_until = d.time + PULL_IN if soft else None
        d.eq_active[e.clamp], e.clamped, e.held = 1, True, True

    def capture(self, e):
        """The shift that puts the end's grip face on the nearest fixed geom within CAPTURE, or None if there's none."""
        best, fromto = None, np.zeros(6)
        for g in self.fixed:
            dist = mujoco.mj_geomDistance(self.m, self.d, e.tip, g, CAPTURE, fromto)
            if dist < CAPTURE and (best is None or dist < best[0]):
                best = (dist, fromto[3:] - fromto[:3] if dist > 0 else np.zeros(3))
        return None if best is None else best[1]

    def update_ends(self):
        """After each step: anchors told to hold grab what they touch; loads, smoothed over LOAD_TAU."""
        m, d, a = self.m, self.d, 1 - math.exp(-LOG_DT / LOAD_TAU)
        for e in self.ends.values():
            force = torque = 0.0
            if e.kind == "anchor":
                if e.armed and not e.clamped and (shift := self.capture(e)) is not None:
                    self.clamp(e, shift)
                if e.pull_in_until is not None and d.time >= e.pull_in_until:
                    m.eq_solref[e.clamp], e.pull_in_until = e.stiff, None
                if e.clamped:
                    rows = (d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (d.efc_id == e.clamp)
                    f = d.efc_force[rows]
                    force, torque = np.linalg.norm(f[:3]), np.linalg.norm(f[3:6])
            elif e.kind == "suction":  # on = the cup is on a surface while sucking; pull = suction - what the surface pushes
                on, push, f6 = False, 0.0, np.zeros(6)
                for i in range(d.ncon):
                    g1, g2 = d.contact.geom1[i], d.contact.geom2[i]
                    if (g1 in e.geoms and m.geom_bodyid[g2] in self.static) or (g2 in e.geoms and m.geom_bodyid[g1] in self.static):
                        on |= e.cup in (g1, g2) and d.contact.dist[i] < CUP_TOUCH
                        if d.contact.efc_address[i] >= 0:
                            mujoco.mj_contactForce(m, d, i, f6)
                            push += f6[0]
                held = e.armed and on
                e.pops += e.held and not held and e.armed  # it came off while still sucking: a pop or a peel
                e.held = held
                force = max(0.0, float(d.actuator_force[e.suck]) - push) if held else 0.0
            e.load += a * (force - e.load)
            e.torque += a * (torque - e.torque)
            e.peak = max(e.peak, e.load)

    def ee_command(self, column, value):
        end, what = EE_COLUMN.match(column).groups()
        if what == "grip":
            self.grip(end, value > 0.5)
        else:
            self.nozzle(end, int(what[1]), value)

    def reset(self):
        """Back to the start pose (the setup's, or floating straight), at rest; stops any sequence."""
        mujoco.mj_resetData(self.m, self.d)
        for e in self.ends.values():
            e.reset()
            if e.clamp >= 0:
                self.m.eq_solref[e.clamp] = e.stiff
            for k in range(len(e.nozzles)):
                self.nozzle(e.name, k, 0.0)
        if self.setup:
            self.place()
        self.end_flight()
        if self.flight and self.strings:  # back at the start, the strings are tied on again, as between parabolas
            self.strings_on(True)
        mujoco.mj_forward(self.m, self.d)
        self.gap = self.box_gap()[0]
        self.keys, self.playing = None, False
        self.com0, self.q0 = self.d.subtree_com[1].copy(), self.d.sensor("imu_quat").data.copy()
        self.com0_box = self.box_com() if self.cabin >= 0 else self.com0  # the start, in the box's frame
        self.turned, self.heading_was, self.done = 0.0, self.heading(), False
        self.turned_space, self.space_was = 0.0, self.heading(in_box=False)
        if self.log:
            self.log(self.d, self.log_columns())

    def box_com(self):
        """The chain's centre of mass in the box's frame (from the box's middle)."""
        return self.box_frame().T @ (self.d.subtree_com[1] - self.d.xpos[self.cabin])

    def box_frame(self):
        """The cabin's orientation (world from box)."""
        return self.d.xmat[self.cabin].reshape(3, 3)

    def strings_on(self, on):
        """Hold the chain where it is (each string as long as it is now) or let go (endless and hidden)."""
        mujoco.mj_forward(self.m, self.d)
        for t in self.strings:
            self.m.tendon_range[t] = (0, self.d.ten_length[t] if on else 100)
            self.m.tendon_rgba[t] = [float(x) for x in STRING_RGBA.split()][:3] + [float(on)]

    def tension(self):
        """The strings' largest pull (N)."""
        rows = (self.d.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON) & np.isin(self.d.efc_id, self.strings)
        return float(np.abs(self.d.efc_force[rows]).max()) if rows.any() else 0.0

    def box_gap(self):
        """The nearest any shell is to the flight box (walls, lid, net), in m, and which part that is."""
        best, fromto = (1.0, ""), np.zeros(6)
        for c in self.shells:
            for b in self.box:
                dist = mujoco.mj_geomDistance(self.m, self.d, c, b, best[0], fromto)
                if dist < best[0]:
                    best = (dist, self.m.geom(b).name)
        return best

    def fly(self, path="", move=3.0, hold=1.0, release_s=None, start_s=None, hyper_s=None, jitter=None, spin=None,
            manual=False):
        """Fly one parabola from here (Reset first for a fresh take): the strings hold the chain through the pull-up and
        let go at T+release_s (T = 0 when zero g begins), adding spin [x, y, z] (deg/s, the setup's spin_deg_s if None)
        to what it keeps from the plane; the sequence starts at T+start_s; the pull-out drops it onto the net. manual:
        no sequence, and the strings hold until let_go(). Missing settings come from the setup's [flight]."""
        f = self.flight
        if not f:
            raise ValueError("this model has no flight box: give the setup a [flight] table (or tick In the flight box) and Rebuild")
        pick = lambda x, key: f[key] if x is None else x
        self.parabola = Parabola(pick(hyper_s, "hyper_s"), pick(jitter, "jitter"), f["seed"], f["level_g"])
        self.release_s, self.start_s, self.pending = pick(release_s, "release_s"), pick(start_s, "start_s"), (path, move, hold)
        self.extra_spin = list(self.setup["spin_deg_s"] if spin is None else spin)
        if manual:
            self.release_s, self.pending = math.inf, None
        self.fly_t, self.released, self.touch, self.gap_min, self.landing, self.dip = 0.0, None, None, 1.0, None, 0.0
        self.fell, self.flown, self.release_spin, self.manual, self.liftoff, self.clear = 0.0, True, 0.0, manual, None, False
        self.strings_on(True)

    def end_flight(self, keep_g=False):
        """Stop flying: the strings let go and, unless the parabola ended by itself (level flight, 1 g), gravity goes back
        to the setup's. Reset and Stop call it too."""
        if self.strings:
            self.strings_on(False)
        if self.flight and not keep_g:
            self.m.opt.gravity[:] = (0, 0, -self.setup["gravity_m_s2"])
        self.fly_t, self.flown = None, False

    def flying(self):
        """Before each step: the parabola's g, letting go, starting the sequence."""
        phase, g, T, theta = self.parabola.at(self.fly_t)
        R = pitch_matrix(theta)
        self.m.opt.gravity[:] = 9.81 * R @ g  # the g in the box, turned with it into the world (which doesn't turn)
        if self.cabin >= 0:  # the cabin's angle and rate, so what rests on it moves with it
            self.d.qpos[self.cabin_q] = theta
            self.d.qvel[self.cabin_v] = (self.parabola.pitch(self.fly_t + 0.005) - self.parabola.pitch(self.fly_t - 0.005)) / 0.01
            c, turn = self.m.body_pos[self.cabin], np.zeros(4)
            mujoco.mju_mat2Quat(turn, R.flatten())
            for e in self.ends.values():  # clamped anchors turn with the box they're clamped to
                if e.clamped:
                    p, q = e.hold_box
                    self.d.mocap_pos[e.mocap] = c + R @ p
                    mujoco.mju_mulQuat(self.d.mocap_quat[e.mocap], turn, q)
        if self.released is None and T >= self.release_s:
            self.let_go()
        if self.pending and T >= self.start_s:
            if self.pending[0]:
                self.play(*self.pending)
            self.pending = None
        self.fly_t += LOG_DT
        return phase, T

    def on_net(self):
        """Is any shell touching the net?"""
        return any(self.net in (self.d.contact.geom1[i], self.d.contact.geom2[i]) for i in range(self.d.ncon))

    def let_go(self):
        """The strings let go (now, or at T+release_s), and any extra spin at release is added."""
        if self.fly_t is None or self.released is not None:
            return
        self.strings_on(False)
        self.released = self.parabola.at(self.fly_t)[2]
        mujoco.mj_forward(self.m, self.d)
        self.release_spin = math.degrees(np.linalg.solve(self.inertia(), self.d.sensor("angmom").data)[0])  # about x
        for axis, rate in zip("xyz", self.extra_spin):
            if rate:
                now = math.degrees(np.linalg.solve(self.inertia(), self.d.sensor("angmom").data) @ AXES[axis])
                self.spin(now + rate, axis)

    def flown_through(self, phase, T):
        """After each step: the gap to the box while free in zero g, then the landing on the net."""
        net = next((b for b in self.box if self.m.geom(b).name == "net"), None)
        if self.flight.get("on_net") and self.liftoff is None and T >= 0 and not self.on_net():  # it floats off the net
            c, w = self.d.xpos[self.cabin], self.d.qvel[self.cabin_v]
            v = self.box_frame().T @ (self.d.sensor("com_vel").data - np.cross((w, 0, 0), self.d.subtree_com[1] - c))
            self.liftoff = (T, v)
        free = self.liftoff is not None if self.flight.get("on_net") else self.released is not None  # nothing holds it
        self.clear = self.clear or (free and self.gap > 10 * TOUCH)  # touches count once it's been clear of everything
        if phase == self.parabola.free and self.clear:
            self.gap_min = min(self.gap_min, self.gap)
            if self.touch is None and self.gap < TOUCH:
                self.touch = (T, self.box_gap()[1])
        if phase == "pull-out" and net is not None:  # in the box's frame: heights above its middle, speed towards the net
            R, c = self.box_frame(), self.d.xpos[self.cabin]
            top = self.m.geom_pos[net][2] + self.m.geom_size[net][2]
            low = min((R.T @ (self.d.geom_xpos[g] - c))[2] - self.m.geom_size[g][0] for g in self.shells)
            if self.landing is None:
                v = self.d.sensor("com_vel").data - np.cross((self.d.qvel[self.cabin_v], 0, 0), self.d.subtree_com[1] - c)
                self.fell = max(self.fell, -float((R.T @ v)[2]))
                if low < top:
                    self.landing = self.fell
            self.dip = max(self.dip, top - low)
        if self.fly_t >= self.parabola.end:
            self.end_flight(keep_g=True)
            self.flown = True
            print(f"flight done: {self.summary()}")

    def inertia(self):
        """Whole chain's inertia tensor about its centre of mass, world frame (the chain: m1 and everything on it)."""
        d, m, c, I = self.d, self.m, self.d.subtree_com[1], np.zeros((3, 3))
        for b in (b for b in range(1, m.nbody) if m.body_rootid[b] == 1):
            R, r = d.ximat[b].reshape(3, 3), d.xipos[b] - c
            I += R @ np.diag(m.body_inertia[b]) @ R.T + m.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
        return I

    def spin_rate(self):
        """Whole-chain spin about the vertical (deg/s): angular momentum / inertia, i.e. the rate if the shape froze."""
        return math.degrees(np.linalg.solve(self.inertia(), self.d.sensor("angmom").data)[2])

    def spin(self, rate, axis="z"):
        """Set the whole chain's spin about world axis x, y or z (through its centre of mass) to `rate` deg/s, keeping
        its shape, its joint motions and its spin about the other two axes. axis="all" sets all three (rate 0 = Stop
        spin). A push from outside: it changes the angular momentum. Also models leftover spin when the chain is let go."""
        d = self.d
        for a in ("x", "y", "z") if axis == "all" else (axis,):
            mujoco.mj_forward(self.m, d)
            n = np.array(AXES[a], float)
            dw = n * (math.radians(rate) - n @ np.linalg.solve(self.inertia(), d.sensor("angmom").data))
            d.qvel[:3] += np.cross(dw, d.xpos[1] - d.subtree_com[1])  # add a rigid rotation about the centre of mass
            d.qvel[3:6] += d.xmat[1].reshape(3, 3).T @ dw
        mujoco.mj_forward(self.m, d)

    def imu_turn(self):
        """How far the IMU's module has turned since the last reset, in 3D (deg), as the flight IMU will see it."""
        return math.degrees(2 * math.acos(min(1.0, abs(float(self.q0 @ self.d.sensor("imu_quat").data)))))

    @staticmethod
    def pitch_axis(roll_deg):
        """The chain's pitch axes at the start, rolled roll_deg about its length (world x for 90, z for 0)."""
        r = math.radians(CHAIN_ROLL + roll_deg)
        return np.array((math.sin(r), 0.0, math.cos(r)))

    def heading(self, in_box=True):
        """Compass direction (rad) the middle module points along the chain, seen from above; None if near vertical.
        In the flight box it's about the chain's starting pitch axis instead (the vertical unless it's rolled), in the
        box's frame (what the cabin's video sees) or, with in_box=False, in space (what the IMU sees)."""
        z = self.d.xmat[self.mid].reshape(3, 3)[:, 2]
        if self.cabin < 0:
            return math.atan2(z[1], z[0]) if np.hypot(z[0], z[1]) > 0.2 else None
        if in_box:
            z = self.box_frame().T @ z
        e1 = np.cross((0.0, 1.0, 0.0), self.turn_axis)
        e1 /= np.linalg.norm(e1)
        p1, p2 = z @ e1, z @ np.cross(self.turn_axis, e1)
        return math.atan2(p2, p1) if math.hypot(p1, p2) > 0.2 else None

    def track_turn(self):
        h = self.heading()
        if h is not None and self.heading_was is not None:
            self.turned += math.degrees((h - self.heading_was + math.pi) % (2 * math.pi) - math.pi)
        self.heading_was = h
        if self.cabin >= 0:
            h = self.heading(in_box=False)
            if h is not None and self.space_was is not None:
                self.turned_space += math.degrees((h - self.space_was + math.pi) % (2 * math.pi) - math.pi)
            self.space_was = h

    def play(self, path, move=3.0, hold=1.0):
        self.keys, self.move, self.hold = read_sequence(path, self.m), move, hold
        self.ee_cols, self.ee_row, self.done = read_ee(path, self.m), None, False
        self.seq_name, self.seq_t, self.playing = os.path.basename(path), 0.0, True

    def duration(self):
        return (len(self.keys) - 1) * (self.move + self.hold) + self.hold

    def advance(self):
        """Simulate LOG_DT seconds. Sequence targets update once per LOG_DT, like commands on a servo bus."""
        flight = self.flying() if self.fly_t is not None else None
        if self.cabin >= 0 and flight is None:
            self.d.qvel[self.cabin_v] = 0  # not flying: the box stays put
        if self.keys is not None and self.playing:
            self.d.ctrl[:self.nj] = target(self.keys, self.seq_t, self.move, self.hold)
            row = min(int(self.seq_t // (self.move + self.hold)), len(self.keys) - 1)  # whose hold has begun
            if row != self.ee_row:
                self.ee_row = row
                for column, values in self.ee_cols.items():
                    self.ee_command(column, values[row])
            self.seq_t += LOG_DT * self.speed
        mujoco.mj_step(self.m, self.d, nstep=max(1, round(LOG_DT / self.m.opt.timestep)))
        mujoco.mj_forward(self.m, self.d)  # sensors for the current state (mj_step leaves them one step behind)
        self.update_ends()
        self.track_turn()
        if self.box:
            self.gap = self.box_gap()[0]
        if flight:
            self.flown_through(*flight)
        if self.log:
            self.log(self.d, self.log_columns())

    def set_log(self, on):
        if self.log:
            self.log.close()
            self.log = None
        if on:
            self.log = Logger(os.path.join(HERE, "logs", f"run_{stamp()}.csv"), self.nj, self.log_names())
            self.log(self.d, self.log_columns())

    def command(self, c):
        """Apply one command from the control window, e.g. {"cmd": "play", "file": ".../sequences/fold_sequence.csv"}."""
        cmd = c["cmd"]
        if cmd == "play":
            self.play(c["file"], c.get("move", 3.0), c.get("hold", 1.0))
        elif cmd == "pause":
            self.playing = False
            self.nozzles_off()  # the timetable stops, and with it the nozzles
        elif cmd == "resume":
            self.playing, self.ee_row = self.keys is not None, None  # the current row's end-effector values again
        elif cmd == "restart":
            self.seq_t, self.playing, self.ee_row = 0.0, self.keys is not None, None
        elif cmd == "fly":
            self.fly(c.get("file", ""), c.get("move", 3.0), c.get("hold", 1.0), c.get("release"), c.get("start"),
                     c.get("hyper"), c.get("jitter"), c.get("spin"), c.get("manual", False))
        elif cmd == "release":
            self.let_go()
        elif cmd in ("stop", "straighten"):
            self.keys, self.playing = None, False
            self.nozzles_off()
            if cmd == "stop" and self.fly_t is not None:
                self.end_flight()
            if cmd == "straighten":
                self.d.ctrl[:self.nj] = 0
        elif cmd == "speed":
            self.speed = float(c["value"])
        elif cmd == "playback":
            self.playback = min(16.0, max(0.1, float(c["value"])))
        elif cmd == "gravity":
            self.m.opt.gravity[:] = (0, 0, -float(c["g"]))
        elif cmd == "physics":
            self.running = bool(c["run"])
        elif cmd == "spin":
            self.spin(float(c["rate"]), c.get("axis", "z"))
        elif cmd == "reset":
            self.reset()
        elif cmd == "log":
            self.set_log(c["on"])
        elif cmd == "grip":
            self.grip(c["end"], c["on"])
        elif cmd == "nozzle":
            self.nozzle(c["end"], int(c["n"]), 1.0 if c["on"] else 0.0)

    def summary(self):
        """One line on how the run went, printed when a headless run ends."""
        drift = np.linalg.norm(self.d.subtree_com[1] - self.com0) * 1000
        line = (f"turned {self.turned:+.1f} deg" + (f" in the box, {self.turned_space:+.1f} deg in space" if self.cabin >= 0 else "")
                + f", IMU turn {self.imu_turn():.1f} deg, spin {self.spin_rate():+.1f} deg/s, COM moved {drift:.3f} mm")
        for e in self.ends.values():
            if e.kind in ("anchor", "suction"):
                line += f"; {e.name} peak load {e.peak:.1f} N, let go untold {e.pops}x"
        if self.setup and self.setup["rod"]:
            u = np.subtract(self.setup["rod"]["to"], self.setup["rod"]["from"])
            moved = (self.box_com() - self.com0_box) if self.cabin >= 0 else self.d.subtree_com[1] - self.com0
            line += f"; along the rod {moved @ u / np.linalg.norm(u) * 1000:+.0f} mm"
        if getattr(self, "parabola", None) and self.flown:
            line += f"; {self.flight_result()}"
        return line

    def flight_result(self):
        off = self.liftoff
        free = (f"started on the net, left it at T+{off[0]:.1f} s ({off[1][2] * 1000:+.0f} mm/s up, {np.linalg.norm(off[1]) * 1000:.0f} mm/s in all), "
                if self.flight.get("on_net") and off else "started on the net, never left it, " if self.flight.get("on_net") else
                "no strings, " if not self.strings else
                f"let go at T+{self.released:.1f} s spinning {self.release_spin:+.1f} deg/s with the plane, "
                if self.released is not None else "never let go, ")
        touch = f"first touched the {self.touch[1]} at T+{self.touch[0]:.1f} s" if self.touch else "touched nothing in zero g"
        held = [e.name for e in self.ends.values() if e.held]
        land = (f"landed on the net at {self.landing:.1f} m/s, sinking {self.dip * 1000:.0f} mm" if self.landing is not None
                else f"still held by {', '.join(held)}" if held else "hasn't landed")
        return f"{free}nearest the box {max(0.0, self.gap_min) * 1000:.0f} mm, {touch}; {land}"

    def readouts(self):
        d, L = self.d, np.linalg.norm(self.d.sensor("angmom").data)
        seq = "none"
        if self.keys is not None:
            done = min(100, 100 * self.seq_t / self.duration())
            seq = f"{self.seq_name} {done:.0f}% ({'playing' if self.playing and done < 100 else 'paused' if not self.playing else 'done'})"
            secs = self.duration() / self.speed
            seq += f", {secs:.1f} s" + ("" if secs <= PARABOLA_S else f": longer than a {PARABOLA_S:g} s parabola")
            if done >= 100 and not self.done:
                self.done = True
                print(f"{self.seq_name} done: {self.summary()}")
        return {"time": f"{d.time:.1f} s", "physics": "running" if self.running else "PAUSED", "sequence": seq,
                "gravity": f"{-self.m.opt.gravity[2]:g} m/s^2",
                "spin": f"{self.spin_rate():+.1f} deg/s", "turned": f"{self.turned:+.1f} deg",
                "IMU turn": f"{self.imu_turn():.1f} deg",
                "COM drift": f"{np.linalg.norm(d.subtree_com[1] - self.com0) * 1000:.3f} mm",
                "|L|": f"{L:.1e} kg m^2/s", "max torque": f"{np.abs(d.actuator_force[:self.nj]).max():.2f} N m",
                "contacts": str(d.ncon), "log": os.path.relpath(self.log.path, HERE) if self.log else "off",
                **({"turned, space": f"{self.turned_space:+.1f} deg (turned is in the box)"} if self.cabin >= 0 else {}),
                **{e.name: self.end_readout(e) for e in self.ends.values()}, **self.flight_readouts()}

    def flight_readouts(self):
        if not self.box:
            return {}
        out = {"box gap": f"{max(0.0, self.gap) * 1000:.0f} mm"}
        if self.fly_t is not None:
            phase, g, T, theta = self.parabola.at(self.fly_t)
            rate = math.degrees(self.parabola.pitch(self.fly_t + 0.05) - self.parabola.pitch(self.fly_t - 0.05)) / 0.1
            out["plane"] = f"pitch {math.degrees(theta):+.0f} deg, turning {rate:+.1f} deg/s"
            out["flight"] = f"{phase}, {-g[2]:.2f} g, T{T:+.1f} s" + (
                f" ({self.parabola.free} in {-T:.1f} s)" if T < 0 else
                f" ({FLIGHT['parabola_s'] - T:.1f} s of {self.parabola.free} left)" if phase == self.parabola.free else "")
            out["strings"] = (f"holding ({len(self.strings)}), pulling up to {self.tension():.1f} N" if self.released is None
                              else f"let go at T+{self.released:.1f} s")
            if self.manual:
                out["by hand"] = ("the strings hold until you click Let go (zero g from T+0)" if self.released is None and self.strings
                                  else "Play runs the sequence now, or drive it with the keys" if self.keys is None
                                  else f"playing {self.seq_name}")
            out["box gap"] += f", least {max(0.0, self.gap_min) * 1000:.0f} mm since let go" if self.released is not None else ""
        elif self.flown:
            out["flight"] = "done: " + self.flight_result()
        else:
            out["flight"] = "not flying (Play flies the parabola)" if self.flight else "no flight timeline in this setup"
        return out

    def banner(self):
        """The big line at the top of the viewer while flying: the g, and T during zero g."""
        if self.fly_t is None:
            return ""
        phase, g, T, _ = self.parabola.at(self.fly_t)
        hint = "  Let go when ready" if self.manual and self.released is None and self.strings else \
               "  Play: the sequence" if self.manual and self.keys is None else ""
        if T < 0:  # counting down to zero g through the level flight and the pull-up
            return f"{-g[2]:.1f} g  {phase}  -  {self.parabola.free} in {math.ceil(-T)}" + (" (by hand)" if self.manual else "")
        return f"{self.parabola.free}  T+{T:.0f}{hint}" if phase == self.parabola.free else f"{-g[2]:.1f} g  {phase}"

    def end_readout(self, e):
        if e.kind == "anchor":
            return f"anchor, {e.state()}, {e.load:.1f} N, {e.torque:.2f} N m"
        if e.kind == "suction":
            return f"suction cup, {e.state()}, pull {e.load:.1f} of {e.limit:g} N, {e.pops} pops"
        if e.kind == "thruster":
            return f"thruster, {'firing' if self.thrust(e) > 0 else 'off'}, {self.thrust(e):.2f} N"
        return e.kind


class Commands:
    """The control window's commands, in the order sent. A countdown {"cmd": "countdown", "s": 5} holds back the ones
    sent after it (a take's spin and play), and the viewer holds the physics, until it ends. Pause, Stop, Reset or
    quit sent during it call the take off."""

    def __init__(self):
        self.waiting, self.until = [], 0.0

    def ready(self, new, now):
        """The commands to run now; new are the ones just received, now is wall-clock time (s)."""
        self.waiting += new
        off = [i for i, c in enumerate(self.waiting) if c["cmd"] in CALL_OFF]
        if now < self.until and off:
            self.waiting, self.until = self.waiting[off[-1]:], 0.0
        out = []
        while self.waiting and now >= self.until:
            c = self.waiting.pop(0)
            if c["cmd"] == "countdown":
                self.until = now + float(c["s"])
            else:
                out.append(c)
        return out

    def left(self, now):
        return max(0.0, self.until - now)


def connect_panel(port):
    """Commands from the control window arrive as JSON lines; readouts go back the same way."""
    sock = socket.create_connection(("127.0.0.1", port))
    inbox = queue.Queue()

    def reader():
        for line in sock.makefile("r"):
            inbox.put(json.loads(line))

    threading.Thread(target=reader, daemon=True).start()
    return sock, inbox


def screenshot(model, data, viewer):
    from PIL import Image

    os.makedirs(os.path.join(HERE, "screenshots"), exist_ok=True)
    path = os.path.join(HERE, "screenshots", f"shot_{stamp()}.png")
    r = mujoco.Renderer(model, 1200, 1600)
    r.update_scene(data, viewer.cam, viewer.opt)
    Image.fromarray(r.render()).save(path)
    r.close()
    print(f"screenshot saved to {os.path.relpath(path, HERE)}")


def run_viewer(s, panel_port=None):
    import mujoco.viewer

    model, data = s.m, s.d
    n = modules(model)
    pressed, sel = [], [2]
    sock, inbox = connect_panel(panel_port) if panel_port else (None, queue.Queue())

    def highlight():
        for i in range(1, n + 1):
            model.geom(f"m{i}_visual").rgba[:] = SELECT_RGBA if i == sel[0] else BASE_RGBA

    def nudge(joint, sign):
        a = actuator(model, f"m{sel[0]}_{joint}")
        if a < 0:
            print(f"module {sel[0]} has no {joint} servo")
            return
        data.ctrl[a] += sign * math.radians(STEP_DEG)
        if model.actuator_ctrllimited[a]:
            data.ctrl[a] = np.clip(data.ctrl[a], *model.actuator_ctrlrange[a])

    def handle(code):
        if code in (KEYS["next"], KEYS["prev"]):
            sel[0] = min(n, max(1, sel[0] + (1 if code == KEYS["next"] else -1)))
            highlight()
        elif code == KEYS["pause"]:
            s.running = not s.running
            print("physics", "running" if s.running else "paused")
            return
        elif code in (KEYS["pitch_up"], KEYS["pitch_down"]):
            nudge("pitch", 1 if code == KEYS["pitch_up"] else -1)
        elif code in (KEYS["yaw_up"], KEYS["yaw_down"]):
            nudge("yaw", 1 if code == KEYS["yaw_up"] else -1)
        else:
            return
        deg = {j: math.degrees(data.ctrl[a]) for j in ("yaw", "pitch") if (a := actuator(model, f"m{sel[0]}_{j}")) >= 0}
        print(f"module {sel[0]}: " + ", ".join(f"{j} {d:+.0f} deg" for j, d in deg.items()))

    highlight()
    frame, next_readout = 2 * LOG_DT, 0.0  # 50 frames per second; readouts 5 times per second
    steps, shown = 0.0, (time.time(), data.time)  # steps owed this frame (playback); when the rate was last measured
    commands, texts, count = Commands(), [], 0  # count: the countdown second on screen (0: none)
    if sys.platform == "darwin":
        import glfw
        glfw.window_hint_string(glfw.COCOA_FRAME_NAME, WINDOW_NAME)
    with mujoco.viewer.launch_passive(model, data, key_callback=pressed.append) as viewer:
        cabin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cabin")

        def camera(on_cabin):
            """The flight box: watch from the cabin (turning with the plane: the box looks still), or from outside, fixed
            in space, where the box pitches with the plane. Esc in the viewer also frees the camera."""
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            if cabin >= 0 and on_cabin:
                viewer.cam.type, viewer.cam.fixedcamid = mujoco.mjtCamera.mjCAMERA_FIXED, cabin
            elif cabin >= 0:
                viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 35, -12, 2.2 * model.stat.extent

        with viewer.lock():  # the passive viewer ignores <visual><global azimuth/elevation>; apply them here
            camera(True)
        while viewer.is_running():
            start = time.time()
            with viewer.lock():
                while pressed:
                    handle(pressed.pop(0))
                new = []
                while not inbox.empty():
                    new.append(inbox.get())
                for c in commands.ready(new, start):
                    if c["cmd"] == "quit":
                        viewer.close()
                    elif c["cmd"] == "camera":
                        camera(bool(c["cabin"]))
                    elif c["cmd"] == "screenshot":
                        screenshot(model, data, viewer)
                    else:
                        try:
                            s.command(c)
                        except Exception as e:  # a bad CSV must not kill the viewer
                            print(f"command {c} failed: {e}")
                left = commands.left(start)  # the physics waits out a countdown, so the take starts at its T-0
                steps += frame / LOG_DT * s.playback if s.running and not left else 0
                for _ in range(int(steps)):
                    s.advance()
                steps -= int(steps)
                info = s.readouts() if start >= next_readout else None
                if info:  # what the playback actually reaches: a slow computer can fall short of a high one
                    rate = (data.time - shown[1]) / max(1e-6, start - shown[0])
                    info["playback"] = f"x{s.playback:g}" + (f" (reaching x{rate:.1f})" if s.running and rate < 0.9 * s.playback else "")
                    shown = (start, data.time)
                    if left:
                        info["sequence"] = f"starts in {math.ceil(left)} s"
            viewer.sync()
            if info:
                texts = [(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "\n".join(info), "\n".join(info.values()))]
            if info or math.ceil(left) != count:  # T-5 ... T-1 in big type, then the take starts; in flight, the g
                count = math.ceil(left)
                title = f"T-{count}" if count else s.banner()
                big = [(mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_TOP, title, "")] if title else []
                viewer.set_texts(texts + big)
            if info:
                next_readout = start + 0.2
                if sock:
                    try:
                        sock.sendall((json.dumps(info) + "\n").encode())
                    except OSError:
                        sock = None
            time.sleep(max(0.0, frame - (time.time() - start)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", help="model XML (default: built fresh from make_chain.py and ../hardware/spec.toml)")
    ap.add_argument("--seq", help="fold sequence CSV (degrees)")
    ap.add_argument("--move", type=float, default=3.0, help="seconds to move between rows")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold each row")
    ap.add_argument("--log", default=os.path.join(HERE, "logs", "run_log.csv"), help='log file; "" for no log')
    ap.add_argument("--headless", action="store_true", help="no viewer; needs --seq")
    ap.add_argument("--gravity", type=float, help="m/s^2 downward, overriding the model (Earth: 9.81)")
    ap.add_argument("--speed", type=float, default=1.0, help="play the sequence this many times faster (e.g. 4)")
    ap.add_argument("--playback", type=float, default=1.0,
                    help="viewer: show the simulation this many times faster than real time; the physics doesn't change")
    ap.add_argument("--spin", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="start spinning at this rate (deg/s) about world x, y, z: the leftover spin when it's let go")
    ap.add_argument("--air", action="store_true", help="cabin air drag on the shells (when building the model)")
    ap.add_argument("--setup", help="setup file (setups/*.toml): the model and the settings above; flags given here win")
    ap.add_argument("--panel", type=int, help=argparse.SUPPRESS)  # port of the control window
    a = ap.parse_args()
    try:
        setup = setup_file.load(a.setup) if a.setup else None
    except (OSError, ValueError) as e:  # say what's wrong with the file, without a traceback
        raise SystemExit(f"can't use that setup: {e}")
    if setup:  # the setup's values become the defaults, so flags on the command line still win
        ap.set_defaults(move=setup["move_s"], hold=setup["hold_s"], speed=setup["speed"], gravity=setup["gravity_m_s2"],
                        air=setup["air"], playback=setup["playback"])
        if not a.panel:  # from the control window, the sequence (and its spin) waits for Play
            ap.set_defaults(seq=setup["sequence"] or None, spin=setup["spin_deg_s"] if any(setup["spin_deg_s"]) else None)
        a = ap.parse_args()

    if a.model:
        model = mujoco.MjModel.from_xml_path(a.model)
    else:  # always the current hardware spec, never a stale chain.xml
        opts = {**setup_file.build_args(setup), "air": a.air} if setup else dict(air=a.air)
        xml = build(**opts).replace('meshdir="meshes"', f'meshdir="{HERE}/meshes"')
        model = mujoco.MjModel.from_xml_string(xml)
    if a.gravity is not None:
        model.opt.gravity[:] = (0, 0, -a.gravity)
    session = Session(model, mujoco.MjData(model), a.log or None, setup if setup and not a.model else None)
    session.speed, session.playback = a.speed, a.playback
    fly = bool(a.headless and setup and setup["flight"] and not a.model)  # a flight setup flies its parabola
    for axis, rate in zip("xyz", () if fly else a.spin or ()):  # in a flight, the spin comes as the strings let go
        session.spin(rate, axis)
    if fly:
        session.fly(a.seq or "", a.move, a.hold)
    elif a.seq:
        session.play(a.seq, a.move, a.hold)
    try:
        if a.headless:
            assert a.seq or fly, "--headless needs --seq"
            if setup:
                print(f"{os.path.basename(a.setup)}: {setup_file.summary(setup)}")
            while session.fly_t is not None if fly else session.seq_t < session.duration() - 1e-9:
                session.advance()
            secs = session.duration() / session.speed if session.keys is not None else 0
            print(f"ran {secs:g} s" + (f" (longer than a {PARABOLA_S:g} s parabola)" if secs > PARABOLA_S else ""))
            if not fly:  # a flight prints its own summary as it lands
                print(session.summary())
        else:
            run_viewer(session, a.panel)
    finally:
        if session.log:
            session.log.close()
            print(f"log written to {os.path.relpath(session.log.path, HERE)}")
