"""Check the generated model and render PNGs into renders/.

  .venv/bin/python verify.py

1. every preset (with and without --slip-ring) compiles
2. each joint turns about the intended axis: anchors/axes, a 30 deg test turn, and the arm mesh
   (yoke discs centred on the pitch axis, hub centred on the yaw axis, hub top at +88 mm)
3. zero gravity + smooth random joint motion: COM stays still, total angular momentum stays ~0
4. no contacts with the chain straight, and none during ../sequences/fold_sequence.csv
5. gravity on: the chain drops onto the floor and settles there
6. spin: the Spin command, the spin-up when folded (skater effect), Stop spin, and the tip-turn readout
7. offscreen renders: one module, the straight 12-module chain, the L and zig-zag folds, the dropped chain
"""
import math
import os

import mujoco
import numpy as np
import trimesh
from PIL import Image

from make_chain import PRESETS, build
from play import Session, read_sequence, target

HERE = os.path.dirname(os.path.abspath(__file__))
FOLD_SEQ = os.path.join(os.path.dirname(HERE), "sequences", "fold_sequence.csv")
failures = []


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg)
    if not ok:
        failures.append(msg)


def load(preset="original14", n=12, slip_ring=False):
    xml = build(preset, n, slip_ring).replace('meshdir="meshes"', f'meshdir="{HERE}/meshes"')
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    x, y, z = axis
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def gross_momentum(m, d):
    """Sum over bodies of |angular momentum about the chain COM|: the scale that should cancel to ~0."""
    c, total = d.subtree_com[1], 0.0
    for b in range(1, m.nbody):
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b, vel, 0)
        w, v = vel[:3], vel[3:] + np.cross(vel[:3], d.xipos[b] - d.xpos[b])
        R = d.ximat[b].reshape(3, 3)
        total += np.linalg.norm(R @ (m.body_inertia[b] * (R.T @ w)) + m.body_mass[b] * np.cross(d.xipos[b] - c, v))
    return total


def check_compile():
    print("1. model compiles")
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


def check_momentum(T=12.0, amp=90, period=1.5):
    print("3. zero gravity, smooth random joint motion")
    for preset in PRESETS:
        m, d = load(preset)
        rng = np.random.default_rng(1)
        keys = np.radians(rng.uniform(-amp, amp, (int(T / period) + 2, m.nu)))
        keys[0] = 0
        mujoco.mj_forward(m, d)
        c0 = d.subtree_com[1].copy()
        drift = L = gross = 0.0
        while d.time < T:
            k = int(d.time // period)
            u = (d.time - k * period) / period
            d.ctrl[:] = keys[k] + (0.5 - 0.5 * math.cos(math.pi * u)) * (keys[k + 1] - keys[k])
            mujoco.mj_step(m, d, nstep=round(0.01 / m.opt.timestep))
            mujoco.mj_forward(m, d)
            drift = max(drift, np.linalg.norm(d.subtree_com[1] - c0))
            L = max(L, np.linalg.norm(d.sensor("angmom").data))
            gross = max(gross, gross_momentum(m, d))
        stable = not any(w.number for w in d.warning)
        check(stable and drift < 1e-4 and L < 1e-4 * gross,
              f"{preset}: {T:g} s, COM moved {drift * 1000:.4f} mm, |L| max {L:.1e} vs {gross:.2f} kg m^2/s "
              f"of body momenta ({L / gross:.0e}){'' if stable else ', UNSTABLE'}")


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
    s.command({"cmd": "spin", "rate": 0})
    run(s, sec=1)
    check(np.linalg.norm(s.d.sensor("angmom").data) < 1e-6 and abs(s.spin_rate()) < 0.01, "Stop spin: angular momentum back to 0")
    s = Session(m, mujoco.MjData(m))
    run(s, "m11_pitch", 90)
    check(1 < abs(s.turned) < 4 and abs(s.spin_rate()) < 0.01,
          f"from rest, swinging the tip (m11_pitch 90 deg) turns the rest of the chain {s.turned:+.1f} deg, then it stops")


def render(m, d, path, views, lookat, dist, size=(800, 800)):
    r = mujoco.Renderer(m, size[1], size[0])
    opt = mujoco.MjvOption()
    imgs = []
    for az, el in views:
        cam = mujoco.MjvCamera()
        cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, dist, az, el
        r.update_scene(d, cam, opt)
        imgs.append(r.render())
    Image.fromarray(np.concatenate(imgs, 1)).save(path)
    r.close()
    print(f"  wrote {os.path.relpath(path, HERE)}")


def renders():
    print("7. renders")
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


if __name__ == "__main__":
    check_compile()
    check_axes()
    check_momentum()
    check_contacts()
    check_gravity()
    check_spin()
    renders()
    print("\nALL CHECKS PASSED" if not failures else f"\n{len(failures)} CHECK(S) FAILED")
    raise SystemExit(bool(failures))
