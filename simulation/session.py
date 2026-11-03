"""The run: Session holds one model's state as it's driven (the start pose, playing a sequence, spin and turn, the end
effectors, the flight box, the commands from the control window, the readouts) and Logger writes its log. play.py
(headless or the viewer), verify.py and stall_sweep.py drive a Session; the control window drives it through the viewer.
"""
import csv
import math
import os
import time

import mujoco
import numpy as np

from ends import End
from flight import AXES, FlightBox
from make_chain import CHAIN_ROLL, build
from make_ends import attach_point, leading_end, level_axis
from sequence import EE_COLUMN, LOG_DT, joint_servos, modules, read_ee, read_sequence, target
from spec import FLIGHT

HERE = os.path.dirname(os.path.abspath(__file__))
PARABOLA_S = FLIGHT["parabola_s"]                  # seconds of microgravity per parabola


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def load_model(**build_args):
    """make_chain.build()'s model, compiled, finding the meshes from any folder."""
    return mujoco.MjModel.from_xml_string(build(**build_args).replace('meshdir="meshes"', f'meshdir="{HERE}/meshes"'))


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


class Session(FlightBox):
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
        cabin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cabin")  # the flight box turns with the plane
        self.cabin = cabin  # its body; play drives its hinge, cabin_pitch, with the plane's pitch
        if cabin >= 0:
            self.cabin_q, self.cabin_v = model.jnt_qposadr[model.joint("cabin_pitch").id], model.jnt_dofadr[model.joint("cabin_pitch").id]
        self.static = {0, cabin} - {-1}  # bodies that are fixed things: the world, and the cabin with the box on it
        self.fixed = [g for g in range(model.ngeom) if model.geom_bodyid[g] in self.static]  # floor, plates, rod, box
        self.ends = {e: End(model, data, e, self.fixed, self.static, cabin) for e in self.end_keys
                     if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, e) >= 0}
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
        ends = [x for e in self.ends.values() for x in (int(e.held), f"{e.load:.4g}", f"{e.torque:.4g}", f"{e.thrust():.4g}")]
        flight = [*(f"{x / 9.81:.4g}" for x in self.m.opt.gravity), f"{self.gap * 1000:.1f}", f"{self.tension():.4g}"]
        return ends + (flight if self.box else [])

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
                e.latch()
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
        self.ends[name].grip(on)

    def nozzle(self, name, k, throttle):
        """Set one nozzle's throttle (0-1); it lights up while it fires."""
        self.ends[name].nozzle(k, throttle)

    def nozzles_off(self):
        for e in self.ends.values():
            e.nozzles_off()

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
            e.nozzles_off()
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
        for e in self.ends.values():  # anchors told to hold grab what they touch; cups hold or pop; the loads
            e.update(LOG_DT)
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
                **{e.name: e.readout() for e in self.ends.values()}, **self.flight_readouts()}
