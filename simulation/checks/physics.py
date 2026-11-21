"""Sections 3, 5, 6 and 13: momentum in zero g, the drop onto the floor, spin and the turn readouts, and the climb round a pole."""
import math
import os

import mujoco
import numpy as np

import setup_file
from make_chain import PRESETS
from session import Session
from checks.common import check, load, random_motion


def check_momentum(T=12.0, amp=90, period=1.5):
    print("3. zero gravity, smooth random joint motion")
    for preset in PRESETS:
        m, d = load(preset)
        drift, L, gross, stable = random_motion(m, d, T, amp, period)
        check(stable and drift < 1e-4 and L < 1e-4 * gross,
              f"{preset}: {T:g} s, COM moved {drift * 1000:.4f} mm, |L| max {L:.1e} vs {gross:.2f} kg m^2/s "
              f"of body momenta ({L / gross:.0e}){'' if stable else ', UNSTABLE'}")


def check_gravity(T=15.0):
    print("5. gravity 9.81 m/s^2: drop onto the floor")
    for preset in PRESETS:
        m, d = load(preset)
        m.opt.gravity[:] = (0, 0, -9.81)
        m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
        mujoco.mj_step(m, d, nstep=round(T / m.opt.timestep))
        mujoco.mj_forward(m, d)
        R = m.geom("m1_sphere").size[0]
        z = np.array([d.xpos[m.body(f"m{i}").id][2] for i in range(1, 13)])
        stable = not any(w.number for w in d.warning)
        check(stable and np.abs(z - R).max() < 1e-3 and d.energy[1] < 1e-3,
              f"{preset}: after {T:g} s all shells rest on the floor (centres {z.min() * 1000:.1f}-{z.max() * 1000:.1f} mm, "
              f"radius {R * 1000:.1f} mm), kinetic energy {d.energy[1] * 1000:.2f} mJ, {d.ncon} contacts")


def check_spin():
    print("6. spin and turn readouts")
    m, _ = load()

    def run(s, joint=None, deg=0.0, sec=4.0):  # ease one joint to deg over 2 s, then hold
        a, t0 = (m.actuator(joint).id, s.d.time) if joint else (None, s.d.time)
        q0 = s.d.ctrl[a] if joint else 0
        while s.d.time < t0 + sec:
            if joint:
                s.d.ctrl[a] = q0 + (math.radians(deg) - q0) * (0.5 - 0.5 * math.cos(math.pi * min(1, (s.d.time - t0) / 2)))
            s.advance()

    s = Session(m, mujoco.MjData(m))
    s.command({"cmd": "spin", "rate": 20})
    L0, w0 = s.d.sensor("angmom").data.copy(), s.spin_rate()
    run(s, sec=2)
    straight, turned = s.spin_rate(), s.turned
    run(s, "m6_pitch", 90)
    folded = s.spin_rate()
    run(s, "m6_pitch", 0)
    back, L1 = s.spin_rate(), s.d.sensor("angmom").data.copy()
    check(abs(w0 - 20) < 0.1 and abs(straight - 20) < 0.1 and abs(turned - 40) < 1,
          f"Spin 20 deg/s: spins at {straight:.1f} deg/s and turns {turned:.1f} deg in 2 s")
    check(folded > 25 and abs(back - 20) < 0.3 and np.allclose(L0, L1, atol=1e-6),
          f"folding into an L spins it up to {folded:.1f} deg/s, unfolding brings it back to {back:.1f} (L unchanged)")
    s.command({"cmd": "spin", "rate": 0, "axis": "all"})
    run(s, sec=1)
    check(np.linalg.norm(s.d.sensor("angmom").data) < 1e-6 and abs(s.spin_rate()) < 0.01, "Stop spin: angular momentum back to 0")
    s = Session(m, mujoco.MjData(m))
    s.spin(10, "x")
    s.spin(20, "z")
    w = np.degrees(np.linalg.solve(s.inertia(), s.d.sensor("angmom").data))
    check(np.allclose(w, (10, 0, 20), atol=0.01), f"Spin about z keeps the spin about x: ({w[0]:.1f}, {w[1]:.1f}, {w[2]:.1f}) deg/s")
    s = Session(m, mujoco.MjData(m))
    run(s, "m11_pitch", 90)
    check(1 < abs(s.turned) < 4 and abs(s.spin_rate()) < 0.01,
          f"from rest, swinging the tip (m11_pitch 90 deg) turns the rest of the chain {s.turned:+.1f} deg, then it stops")


def check_pole_climb():
    print("13. wrapped around a pole: the start pose, the squeeze grip, the climb (14 cm chain, Ø100 mm pole, zero g)")
    setup = setup_file.new(sequence=os.path.join(setup_file.SEQ_DIR, "pole_climb.csv"), move_s=0.7, hold_s=0.3,
                           rod={"from": [0.0, 0.0, 0.15], "to": [0.0, 0.0, 2.6], "diameter_mm": 100.0, "wrap_m": 0.35})
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    a, b = np.array(setup["rod"]["from"]), np.array(setup["rod"]["to"])
    u = (b - a) / np.linalg.norm(b - a)
    want = setup["rod"]["diameter_mm"] / 2000 + m.geom("m1_sphere").size[0]
    def from_axis():
        rel = np.array([d.xpos[m.body(f"m{i}").id] for i in range(1, 13)]) - a
        return np.linalg.norm(rel - np.outer(rel @ u, u), axis=1)
    r0, z0 = from_axis(), (d.subtree_com[1] - a) @ u
    check(np.abs(r0 - want).max() < 1e-3 and (r0 > 0).all() and d.ncon >= 10,
          f"wrapped start: all 12 modules {r0.min() * 1000:.1f}-{r0.max() * 1000:.1f} mm from the pole's axis "
          f"(touching at {want * 1000:.1f}), {d.ncon} shells on the pole")
    s.play(setup["sequence"], setup["move_s"], setup["hold_s"])
    while s.seq_t < s.duration() - 1e-9:
        s.advance()
    climbed, r1 = (d.subtree_com[1] - a) @ u - z0, from_axis()
    check(climbed > 0.3 and r1.max() < want + 0.05, f"the wrap-and-inch climb: {climbed * 1000:.0f} mm up the pole in "
          f"{s.duration():.1f} s, still wrapped (modules {r1.min() * 1000:.0f}-{r1.max() * 1000:.0f} mm from its axis)")
