"""Stall length: the longest chain that can stand straight out level from a wall, at Earth, Mars, Moon and zero gravity.

  .venv/bin/python stall_sweep.py        # about a minute; prints a table

Experiment A's number. Each run is setups/2b_stall_0g.toml (anchored to a wall by ee1, pitch axes level, hanging from
m1_pitch at -90 deg, then told to stand straight out) with the chain length and gravity changed. A chain "stands" if,
at the end, its last module is within LEVEL_DEG of level as seen from m1. The hand estimate is the longest N with
N(N+1)/2 * m * g * pitch <= the servo's stall torque (every module's weight on m1_pitch, straight out).
"""
import math
import os

import mujoco

import setup_file
from make_chain import HW, MASS, SERVO, build
from play import Session, modules

HERE = os.path.dirname(os.path.abspath(__file__))
GRAVITY = dict(Earth=9.81, Mars=3.71, Moon=1.62, zero=0.0)
LENGTHS = range(2, 13)
LEVEL_DEG = 10


def elevation(m, d):
    """Angle (deg) of the line from m1 to the last module above level: 0 = straight out, -90 = hanging."""
    a, b = d.xpos[m.body("m1").id], d.xpos[m.body(f"m{modules(m)}").id]
    return math.degrees(math.atan2(b[2] - a[2], math.hypot(b[0] - a[0], b[1] - a[1])))


def run(n, g):
    setup = setup_file.load(os.path.join(HERE, "setups", "2b_stall_0g.toml"))
    setup.update(modules=n, gravity_m_s2=g)
    xml = build(**setup_file.build_args(setup)).replace('meshdir="meshes"', f'meshdir="{HERE}/meshes"')
    m = mujoco.MjModel.from_xml_string(xml)
    s = Session(m, mujoco.MjData(m), None, setup)
    s.play(setup["sequence"], setup["move_s"], setup["hold_s"])
    while s.seq_t < s.duration() - 1e-9:
        s.advance()
    return elevation(m, s.d)


def estimate(g):
    module = (MASS["base_g"] + MASS["arm_g"]) / 1000
    fits = [n for n in range(1, 41) if g == 0 or n * (n + 1) / 2 * module * g * HW["pitch_mm"] / 1000 <= SERVO["stall_torque_Nm"]]
    return max(fits)


if __name__ == "__main__":
    print(f"chain end's elevation (deg) after standing out; stands = within {LEVEL_DEG} deg of level")
    print("modules " + " ".join(f"{n:>6}" for n in LENGTHS) + "   longest that stands   hand estimate")
    for place, g in GRAVITY.items():
        elev = [run(n, g) for n in LENGTHS]
        stands = [n for n, e in zip(LENGTHS, elev) if e > -LEVEL_DEG]
        longest = max(stands) if stands else "none"
        hand = estimate(g)
        print(f"{place:<7} " + " ".join(f"{e:+6.0f}" for e in elev)
              + f"   {longest!s:>4}{'+' if longest == LENGTHS[-1] else ' '}                {hand if g else 'any'}")
