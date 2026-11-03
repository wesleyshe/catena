"""Writes the pole climb: ../sequences/pole_climb.csv, for the 14 cm chain on a Ø100 mm pole. The angles fit any module
size on a pole scaled the same way (setups/5a_pole_climb flies the 8.5 cm chain on a Ø61 mm pole); verify.py section 13
checks the 14 cm climb.

  .venv/bin/python pole_climb_design.py        # after changing the numbers below

The bare chain wraps a pole and climbs it like a snake, gripping by squeezing. It's three zones: links 1-4 (m1-m5) the
back grip, links 5-7 (m5-m8) the stretchy middle, links 8-11 (m8-m12) the front grip. A zone either grips (its module
centres aimed SQUEEZE inside the pole's contact radius, so the servos press) or is loose (LOOSE outside it).

Each keyframe is designed on the pole's surface: the module centres are placed on cylinders round the pole (one link
apart, rising RISE_* per link), and the joint angles follow from those points. Giving each zone its own coil instead
puts the front off the pole, because the zones' coils don't share an axis.
"""
import math
import os

import numpy as np

import setup_file
from make_chain import PITCH_RANGE, PRESETS, R_SHELL, YAW_RANGE

P = PRESETS["original14"]["pitch"]
POLE_MM = 100                  # pole diameter
SQUEEZE, LOOSE = 0.010, 0.020  # m: gripping zones aimed inside the pole, loose ones clear of it
RISE_SHORT, RISE_LONG = 0.0286, 0.090  # m per link along the pole: coiled (0.16 m per turn), and the middle stretched
MOVE_S, HOLD_S, CYCLES = 0.7, 0.3, 3
ZONE = {i: "A" if i <= 4 else "B" if i <= 7 else "C" for i in range(1, 12)}  # link i joins m{i} and m{i+1}
TOUCH = POLE_MM / 2000 + R_SHELL  # module centres touching the pole
GRIP, FREE = TOUCH - SQUEEZE, TOUCH + LOOSE
STATES = {"touch": dict(A=(TOUCH, RISE_SHORT), B=(TOUCH, RISE_SHORT), C=(TOUCH, RISE_SHORT)),  # the start: the coil
          "hold": dict(A=(GRIP, RISE_SHORT), B=(FREE, RISE_SHORT), C=(GRIP, RISE_SHORT)),
          "front loose": dict(A=(GRIP, RISE_SHORT), B=(FREE, RISE_SHORT), C=(FREE, RISE_SHORT)),
          "front out": dict(A=(GRIP, RISE_SHORT), B=(FREE, RISE_LONG), C=(FREE, RISE_SHORT)),
          "front holds": dict(A=(GRIP, RISE_SHORT), B=(FREE, RISE_LONG), C=(GRIP, RISE_SHORT)),
          "back loose": dict(A=(FREE, RISE_SHORT), B=(FREE, RISE_LONG), C=(GRIP, RISE_SHORT)),
          "back in": dict(A=(FREE, RISE_SHORT), B=(FREE, RISE_SHORT), C=(GRIP, RISE_SHORT))}
CYCLE = ["front loose", "front out", "front holds", "back loose", "back in", "hold"]


def centres(state):
    """Module centres on the cylinders, all winding the same way, plus one virtual module before m1 (index 0)."""
    r = {i: state[ZONE[min(i, 11)]][0] for i in range(1, 13)}
    for i in (5, 8):  # a module where two zones meet sits between them
        r[i] = 0.5 * (state[ZONE[i - 1]][0] + state[ZONE[i]][0])
    th, z, pts = 0.0, 0.0, []
    for i in range(1, 13):
        pts.append((r[i] * math.cos(th), r[i] * math.sin(th), z))
        if i < 12:
            dz = state[ZONE[i]][1]
            th += math.acos((r[i] ** 2 + r[i + 1] ** 2 + dz ** 2 - P ** 2) / (2 * r[i] * r[i + 1]))
            z += dz
    r1, dz1 = state["A"]  # the virtual module continues zone A, so m1 bends like its neighbours
    th0 = -math.acos((2 * r1 ** 2 + dz1 ** 2 - P ** 2) / (2 * r1 * r1))
    return np.array([(r1 * math.cos(th0), r1 * math.sin(th0), -dz1)] + pts)


def joints(pts):
    """Joint angles (deg) that put the modules at pts[1:]: pitch about each module's x bends the chain from its incoming
    direction (the module's z) to its outgoing one; the next yaw turns x onto the next bend axis (whichever sign of it
    needs less twist)."""
    d = [(pts[k + 1] - pts[k]) / np.linalg.norm(pts[k + 1] - pts[k]) for k in range(len(pts) - 1)]
    x, z = np.cross(d[0], d[1]), d[0]
    x /= np.linalg.norm(x)
    pitch, yaw = {}, {}
    for i in range(1, 12):
        pitch[i] = math.degrees(math.atan2(-d[i] @ np.cross(z, x), d[i] @ z))
        if i == 11:
            yaw[12] = yaw[11]
            break
        bend = np.cross(d[i], d[i + 1])
        bend /= np.linalg.norm(bend)
        twist = lambda b: math.degrees(math.atan2(np.cross(x, b) @ d[i], x @ b))
        b = min((bend, -bend), key=lambda b: abs(twist(b)))
        yaw[i + 1], x, z = twist(b), b, d[i]
    return pitch, yaw


if __name__ == "__main__":
    cols = [f"m{i}_{j}" for i in range(1, 13) for j in ("yaw", "pitch")]
    rows = ["touch", "hold"] + CYCLE * CYCLES
    secs = (len(rows) - 1) * (MOVE_S + HOLD_S) + HOLD_S
    lines = ["# Wrapped around a pole (no suction cups, no clamps): climbs it like a snake, gripping by squeezing.",
             "# Three zones: m1-m5 grip, m5-m8 stretch, m8-m12 grip. Each step: the front coil loosens, the middle stretches so",
             "# it slides up the pole, the front squeezes; then the back loosens, the middle shortens and pulls it up, the back",
             f"# squeezes. Written by simulation/pole_climb_design.py for a {POLE_MM} mm pole and 14 cm modules (scale the pole",
             f"# with the modules: setup 5a_pole_climb, 8.5 cm on 61 mm). Row 0 is the coil touching the pole all round.",
             f"# {len(rows)} rows, {secs:.1f} s at move {MOVE_S:g} s, hold {HOLD_S:g} s.",
             ",".join(cols)]
    for name in rows:
        pts = centres(STATES[name])
        pitch, yaw = joints(pts)
        assert max(map(abs, pitch.values())) < PITCH_RANGE and max(map(abs, yaw.values())) < YAW_RANGE, name
        c = pts[1:]
        assert min(np.linalg.norm(c[i] - c[j]) for i in range(12) for j in range(i + 2, 12)) > 2 * R_SHELL, name
        q = {f"m{i}_pitch": v for i, v in pitch.items()} | {f"m{i}_yaw": v for i, v in yaw.items()}
        lines.append(",".join(f"{q.get(c, 0):.2f}" for c in cols))
    with open(os.path.join(setup_file.SEQ_DIR, "pole_climb.csv"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote pole_climb.csv ({len(rows)} rows, {secs:.1f} s)")
