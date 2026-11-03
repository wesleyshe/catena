"""Section 8: offscreen renders into renders/."""
import math
import os

import mujoco

from sequence import read_sequence, target
from checks.common import load, render, HERE, FOLD_SEQ


def renders():
    print("8. renders")
    os.makedirs(f"{HERE}/renders", exist_ok=True)
    ax = "".join(f'<geom type="cylinder" fromto="{a}" size="0.0008" rgba="{c}" contype="0" conaffinity="0"/>'
                 for a, c in (("-0.09 0 0 0.09 0 0", "1 0.1 0.1 1"), ("0 0 -0.09 0 0 0.12", "0.1 0.8 0.1 1")))
    xml = f"""<mujoco><compiler meshdir="{HERE}/meshes"/><visual><global offwidth="1600" offheight="800"/></visual>
      <asset><mesh name="base" file="base_mid.stl" inertia="shell"/><mesh name="arm" file="arm.stl" inertia="shell"/></asset>
      <worldbody><light directional="true" dir="0 0.3 -1"/>{ax}
        <geom type="mesh" mesh="base" rgba="0.78 0.78 0.8 0.35" contype="0" conaffinity="0"/>
        <body><joint name="pitch" axis="1 0 0"/><geom type="mesh" mesh="arm" rgba="0.2 0.45 0.95 1" contype="0" conaffinity="0"/></body>
      </worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    render(m, d, f"{HERE}/renders/module.png", [(90, 0), (0, 0)], (0, 0, 0.01), 0.36)
    d.qpos[0] = math.radians(60)
    mujoco.mj_forward(m, d)
    render(m, d, f"{HERE}/renders/module_pitch60.png", [(0, 0), (130, -25)], (0, 0, 0.01), 0.36)

    m, d = load()
    mujoco.mj_forward(m, d)
    views = [(0, -89), (135, -25)]  # from above (folds lie in the horizontal plane), and oblique
    render(m, d, f"{HERE}/renders/chain12.png", views, d.subtree_com[1], 2.2, (1000, 700))
    keys = read_sequence(FOLD_SEQ, m)
    for t_snap, name in ((4.9, "fold_L"), (12.9, "fold_zigzag")):
        while d.time < t_snap:
            d.ctrl[:] = target(keys, d.time, 3, 1)
            mujoco.mj_step(m, d, nstep=20)
        render(m, d, f"{HERE}/renders/{name}.png", views, d.subtree_com[1], 2.2, (1000, 700))
    m, d = load()
    m.opt.gravity[:] = (0, 0, -9.81)
    mujoco.mj_step(m, d, nstep=round(15 / m.opt.timestep))
    render(m, d, f"{HERE}/renders/gravity_drop.png", [(135, -25), (90, -10)], d.subtree_com[1], 2.2, (1000, 700))
