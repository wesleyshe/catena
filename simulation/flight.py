"""The flight: one parabola of the Novespace A310 (Parabola: the g and the plane's pitch over time, numbers in the
spec's [flight]), and Session's methods for the flight box (FlightBox): the strings holding and letting go, the box
turning with the plane, the gap to the box, the landing on the net, and what the readouts and the viewer's big text say.
"""
import math

import mujoco
import numpy as np

from make_box import STRING_RGBA
from sequence import LOG_DT
from spec import FLIGHT

AXES = dict(x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1))  # world axes: x across the start pose, y along it, z vertical
TOUCH = 0.001       # m: a shell this close to the box (walls, lid, net) touches it


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


class FlightBox:
    """Session's methods for the flight box, kept in this file to keep session.py short: Session inherits them, so self
    is the Session. They use its model and data (m, d), what it found in the model (cabin, cabin_q, cabin_v, box, net,
    strings, shells), the setup's [flight] (flight) and its chain (spin, inertia, play, ends, keys, gap)."""

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
            out["strings"] = ("none" + (" (lying on the net)" if self.flight.get("on_net") else "") if not self.strings else
                              f"holding ({len(self.strings)}), pulling up to {self.tension():.1f} N" if self.released is None
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
