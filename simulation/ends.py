"""An end effector at run time (make_ends.py builds them into the model): what it is, what it's been told to do and
what it's doing. An anchor told to hold grabs whatever fixed thing its grip face comes near; a suction cup holds while
it's on a surface and sucking, and pops above its pull limit; a thruster's nozzles push and light up while they fire.
"""
import math

import mujoco
import numpy as np

from make_ends import NOZZLE_RGBA
from sequence import actuator

CAPTURE = 0.015     # m: an anchor told to hold grabs when its grip face comes this close to something fixed
PULL_IN = 0.2       # s: having grabbed short of the surface, the clamp pulls the face onto it gently for this long
SOFT_SOLREF = (0.05, 1.0)  # the clamp's weld while it pulls in; then it's back to the stiff make_ends.CLAMP_SOLREF
LOAD_TAU = 0.1      # s: end-effector loads are smoothed over this; the stiff clamp's raw force is spiky
CUP_TOUCH = 0.003   # m: a cup this close to a fixed surface counts as on it


class End:
    """One end effector, ee1 or ee{N}. fixed: the geoms of fixed things (floor, plates, rod, box); static: their bodies
    (the world, and the cabin if there's a flight box); cabin: the cabin's body id, or -1."""

    def __init__(self, model, data, name, fixed, static, cabin):
        find = lambda kind, what: mujoco.mj_name2id(model, kind, what)
        self.m, self.d, self.fixed, self.static, self.cabin = model, data, fixed, static, cabin
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

    def grip(self, on):
        """Hold (grab whatever fixed thing the end touches, now or later) or let go."""
        self.armed = bool(on)
        if self.kind == "anchor" and not on and self.clamped:
            self.d.eq_active[self.clamp], self.clamped, self.held = 0, False, False
        if self.kind == "suction":
            self.d.ctrl[self.suck] = float(bool(on))

    def nozzle(self, k, throttle):
        """Set one nozzle's throttle (0-1); it lights up while it fires."""
        if k < len(self.nozzles):
            self.d.ctrl[self.nozzles[k]] = min(1.0, max(0.0, float(throttle)))
            self.m.site_rgba[self.sites[k]] = [float(x) for x in NOZZLE_RGBA[int(self.d.ctrl[self.nozzles[k]] > 0)].split()]

    def nozzles_off(self):
        for k in range(len(self.nozzles)):
            self.nozzle(k, 0.0)

    def thrust(self):
        return float(np.sum(self.d.actuator_force[self.nozzles])) if self.nozzles else 0.0

    def latch(self, shift=np.zeros(3)):
        """Weld the end where it is, moved by `shift` (onto the surface it's about to touch): its hold body goes there.
        With a shift, the weld starts soft and pulls the end in over PULL_IN, like a latch closing."""
        d = self.d
        d.mocap_pos[self.mocap] = d.xpos[self.body] + shift
        d.mocap_quat[self.mocap] = d.xquat[self.body]
        if self.cabin >= 0:  # in the flight box, the hold rides on the cabin: remember it in the box's frame
            R, c = d.xmat[self.cabin].reshape(3, 3), d.xpos[self.cabin]
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, (R.T @ d.xmat[self.body].reshape(3, 3)).flatten())
            self.hold_box = (R.T @ (d.mocap_pos[self.mocap] - c), q)
        soft = np.linalg.norm(shift) > 1e-4
        self.m.eq_solref[self.clamp] = SOFT_SOLREF if soft else self.stiff
        self.pull_in_until = d.time + PULL_IN if soft else None
        d.eq_active[self.clamp], self.clamped, self.held = 1, True, True

    def capture(self):
        """The shift that puts the grip face on the nearest fixed geom within CAPTURE, or None if there's none."""
        best, fromto = None, np.zeros(6)
        for g in self.fixed:
            dist = mujoco.mj_geomDistance(self.m, self.d, self.tip, g, CAPTURE, fromto)
            if dist < CAPTURE and (best is None or dist < best[0]):
                best = (dist, fromto[3:] - fromto[:3] if dist > 0 else np.zeros(3))
        return None if best is None else best[1]

    def update(self, dt):
        """After each step of dt seconds: an anchor told to hold grabs what it touches; a cup holds or pops; the load,
        smoothed over LOAD_TAU."""
        m, d, a = self.m, self.d, 1 - math.exp(-dt / LOAD_TAU)
        force = torque = 0.0
        if self.kind == "anchor":
            if self.armed and not self.clamped and (shift := self.capture()) is not None:
                self.latch(shift)
            if self.pull_in_until is not None and d.time >= self.pull_in_until:
                m.eq_solref[self.clamp], self.pull_in_until = self.stiff, None
            if self.clamped:
                rows = (d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (d.efc_id == self.clamp)
                f = d.efc_force[rows]
                force, torque = np.linalg.norm(f[:3]), np.linalg.norm(f[3:6])
        elif self.kind == "suction":  # on = the cup is on a surface while sucking; pull = suction - what the surface pushes
            on, push, f6 = False, 0.0, np.zeros(6)
            for i in range(d.ncon):
                g1, g2 = d.contact.geom1[i], d.contact.geom2[i]
                if (g1 in self.geoms and m.geom_bodyid[g2] in self.static) or (g2 in self.geoms and m.geom_bodyid[g1] in self.static):
                    on |= self.cup in (g1, g2) and d.contact.dist[i] < CUP_TOUCH
                    if d.contact.efc_address[i] >= 0:
                        mujoco.mj_contactForce(m, d, i, f6)
                        push += f6[0]
            held = self.armed and on
            self.pops += self.held and not held and self.armed  # it came off while still sucking: a pop or a peel
            self.held = held
            force = max(0.0, float(d.actuator_force[self.suck]) - push) if held else 0.0
        self.load += a * (force - self.load)
        self.torque += a * (torque - self.torque)
        self.peak = max(self.peak, self.load)

    def readout(self):
        if self.kind == "anchor":
            return f"anchor, {self.state()}, {self.load:.1f} N, {self.torque:.2f} N m"
        if self.kind == "suction":
            return f"suction cup, {self.state()}, pull {self.load:.1f} of {self.limit:g} N, {self.pops} pops"
        if self.kind == "thruster":
            return f"thruster, {'firing' if self.thrust() > 0 else 'off'}, {self.thrust():.2f} N"
        return self.kind
