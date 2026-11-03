"""Sections 1, 2 and 4: the model compiles from the current CAD, every joint turns about the right axis, and nothing touches."""
import math
import os

import mujoco
import numpy as np
import trimesh

from export_meshes import SRC, STAMP, fingerprint
from make_chain import PRESETS
from sequence import read_sequence, target
from spec import SPEC_FILE
from checks.common import check, load, rot, HERE, FOLD_SEQ


def check_compile():
    print("1. model compiles, from the current hardware")
    cad, cur = os.path.basename(SRC), fingerprint(SRC)
    made = open(STAMP).read().splitlines()[:2] if os.path.exists(STAMP) else ["nothing recorded", ""]
    ok = made == [cad, cur]
    check(ok, f"meshes/ match the current CAD ({cad}, sha256 {cur[:8]})" if ok else
          f"meshes/ came from {made[0]} ({made[1][:8]}), not the current {cad} ({cur[:8]}): run export_meshes.py")
    print(f"  info  masses, sizes, ranges and servo for original14 come from {os.path.relpath(SPEC_FILE, HERE)}")
    for preset in PRESETS:
        for slip in (False, True):
            m, _ = load(preset, slip_ring=slip)
            check(m.nu == 22 and m.njnt == 23, f"{preset}{' --slip-ring' if slip else ''}: {m.nbody - 1} bodies, "
                  f"{m.njnt - 1} hinges, {m.nu} actuators, {sum(m.body_mass):.3f} kg")


def check_axes():
    print("2. joint axes (in module 1's frame; the chain itself floats along world +Y)")
    m, d = load()
    P = PRESETS["original14"]["pitch"]
    mujoco.mj_forward(m, d)
    R1, p1 = d.xmat[1].reshape(3, 3).copy(), d.xpos[1].copy()
    for j in range(1, m.njnt):
        name, i = m.joint(j).name, int(m.joint(j).name[1:].split("_")[0])
        want_axis = (1, 0, 0) if name.endswith("pitch") else (0, 0, 1)
        if not (np.allclose(R1.T @ d.xaxis[j], want_axis) and np.allclose(R1.T @ (d.xanchor[j] - p1), (0, 0, (i - 1) * P))):
            check(False, f"{name}: axis {R1.T @ d.xaxis[j]}, anchor {R1.T @ (d.xanchor[j] - p1)}")
    check(True, "all 22 joint axes/anchors at the zero pose: pitch = +X, yaw = +Z, both through the module centre")

    worst = 0.0
    for j in range(1, m.njnt):
        mujoco.mj_resetData(m, d)
        d.qpos[m.jnt_qposadr[j]] = math.radians(30)
        mujoco.mj_kinematics(m, d)
        b, axis, anchor = m.jnt_bodyid[j], R1.T @ d.xaxis[j], R1.T @ (d.xanchor[j] - p1)
        turn = rot(axis, math.radians(30))
        worst = max(worst, np.abs(R1.T @ d.xmat[b].reshape(3, 3) - turn).max())
        # every body after the joint must have rotated rigidly about the joint axis through its anchor
        for k in range(b, m.nbody):  # bodies are numbered along the chain, so these are all downstream
            rest = np.array([0, 0, (int(m.body(k).name[1:].split("_")[0]) - 1) * P])
            worst = max(worst, np.abs(R1.T @ (d.xpos[k] - p1) - (anchor + turn @ (rest - anchor))).max())
    check(worst < 1e-9, f"30 deg test turn of each joint moves everything after it rigidly about that axis (err {worst:.1e})")

    arm = trimesh.load(f"{HERE}/meshes/arm.stl").vertices * 1000
    discs = arm[(np.abs(arm[:, 0]) > 20.5) & (np.abs(arm[:, 0]) < 24.5) & (np.hypot(arm[:, 1], arm[:, 2]) < 12.5)]
    hub = arm[arm[:, 2] > 84.9]
    dc = (discs[:, 1:].max(0) + discs[:, 1:].min(0)) / 2
    hc = (hub[:, :2].max(0) + hub[:, :2].min(0)) / 2
    check(np.abs(dc).max() < 0.05, f"yoke side discs (horn mounts) centred on the pitch axis: y,z offset {dc.round(3)} mm")
    check(np.abs(hc).max() < 0.05, f"hub flange centred on the yaw axis: x,y offset {hc.round(3)} mm")
    check(abs(arm[:, 2].max() - 88) < 0.01, f"hub top at +{arm[:, 2].max():.2f} mm; + 52 mm to the next yaw horn face = 140 mm pitch")


def check_contacts():
    print("4. contacts")
    m, d = load()
    mujoco.mj_forward(m, d)
    R, P = m.geom("m1_sphere").size[0], PRESETS["original14"]["pitch"]
    check(d.ncon == 0, f"straight chain: {d.ncon} contacts (neighbour gap {(P - 2 * R) * 1000:.1f} mm, "
          f"next-nearest {(2 * P - 2 * R) * 1000:.0f} mm)")
    keys, ncon = read_sequence(FOLD_SEQ, m), 0
    while d.time < (len(keys) - 1) * 4 + 1:
        d.ctrl[:] = target(keys, d.time, 3, 1)
        mujoco.mj_step(m, d, nstep=20)
        ncon = max(ncon, d.ncon)
    check(ncon == 0, f"fold_sequence.csv: at most {ncon} contacts")
