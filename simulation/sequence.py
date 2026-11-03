"""Fold sequences as the sim plays them (the format is in ../sequences/README.md): the joint angles as servo targets,
the end-effector columns, and easing from row to row, with targets updated every LOG_DT like commands on a servo bus.
Also how the model's actuators line up with the CSV's columns.
"""
import csv
import math
import re

import mujoco
import numpy as np

LOG_DT = 0.01       # seconds between log rows and between sequence target updates
EE_COLUMN = re.compile(r"(ee\d+)_(grip|n[0-4])$")  # end-effector columns in a sequence CSV


def actuator(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def modules(model):
    """How many chain modules: bodies m1, m2, ... (arms and end effectors don't count)."""
    return sum(1 for b in range(model.nbody) if model.body(b).name[:1] == "m" and model.body(b).name[1:].isdigit())


def joint_servos(model):
    """How many joint servos. They come first among the actuators, in CSV column order."""
    return int(np.sum(model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT))


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


def target(keys, t, move, hold):
    """Hold each keyframe for `hold` s, then ease to the next one over `move` s."""
    k = int(t // (move + hold))
    if k >= len(keys) - 1:
        return keys[-1]
    u = min(1.0, max(0.0, (t - k * (move + hold) - hold) / move))
    return keys[k] + (0.5 - 0.5 * math.cos(math.pi * u)) * (keys[k + 1] - keys[k])
