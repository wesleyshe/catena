"""Check the generated model and render PNGs into renders/.

  .venv/bin/python verify.py

1. meshes/ were exported from the current CAD (../hardware/cad), and every preset (with and without --slip-ring) compiles
2. each joint turns about the intended axis: anchors/axes, a 30 deg test turn, and the arm mesh
   (yoke discs centred on the pitch axis, hub centred on the yaw axis, hub top at +88 mm)
3. zero gravity + smooth random joint motion: COM stays still, total angular momentum stays ~0
4. no contacts with the chain straight, and none during ../sequences/fold_sequence.csv
5. gravity on: the chain drops onto the floor and settles there
6. spin: the Spin command (one axis at a time), the spin-up when folded (skater effect), Stop spin, the tip-turn readout
7. flight options: the IMU output, weighed modules, air drag and the flight box
8. offscreen renders: one module, the straight 12-module chain, the L and zig-zag folds, the dropped chain
9. end effectors: every type at each end compiles, the modules are unchanged, the m1_yaw twist axis, zero-g momentum
10. anchors: placement, the clamp holds, grab on touch, let go keeps the momentum, the grip column
11. suction cups: hold below the pull limit, pop above it, grab on touch, never stick to the chain
12. thrusters: a = F/m, opposite nozzles cancel, a side nozzle turns the chain but can't roll it
13. wrapped around a pole: the start pose touches all round, and the wrap-and-inch climb gets up it
14. the countdown before a take: the spin and play wait for it, Stop calls it off, Play again starts it afresh
15. a parabola in the flight box (8.5 cm modules): the strings hold it through 1.8 g, let go cleanly at zero g, the turn
    survives the leftover g, the pull-out lands it on the net
"""
import contextlib
import glob
import io
import math
import os
import tempfile

import mujoco
import numpy as np
import trimesh
from PIL import Image

import setup_file
from export_meshes import SRC, STAMP, fingerprint
from make_chain import FLIGHT, held_height
from make_chain import EE_MASS, EE_SERVO_MASS, EE_TYPES, PRESETS, SPEC_FILE, build
from play import LOG_DT, Commands, Session, joint_servos, read_sequence, target
from play import Parabola as Play_parabola

HERE = os.path.dirname(os.path.abspath(__file__))
FOLD_SEQ = os.path.join(os.path.dirname(HERE), "sequences", "fold_sequence.csv")
failures = []


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg)
    if not ok:
        failures.append(msg)


def load(preset="original14", n=12, slip_ring=False, **opts):
    xml = build(preset, n, slip_ring, **opts).replace('meshdir="meshes"', f'meshdir="{HERE}/meshes"')
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


def random_motion(m, d, T=12.0, amp=90, period=1.5):
    """Smooth random joint motion in zero g. Returns the max COM drift (m), max |L|, max gross body momentum, stable.
    Only the joint servos move: other actuators (end effectors) stay off."""
    rng, nj = np.random.default_rng(1), joint_servos(m)
    keys = np.radians(rng.uniform(-amp, amp, (int(T / period) + 2, nj)))
    keys[0] = 0
    mujoco.mj_forward(m, d)
    c0 = d.subtree_com[1].copy()
    drift = L = gross = 0.0
    while d.time < T:
        k = int(d.time // period)
        u = (d.time - k * period) / period
        d.ctrl[:nj] = keys[k] + (0.5 - 0.5 * math.cos(math.pi * u)) * (keys[k + 1] - keys[k])
        mujoco.mj_step(m, d, nstep=round(0.01 / m.opt.timestep))
        mujoco.mj_forward(m, d)
        drift = max(drift, np.linalg.norm(d.subtree_com[1] - c0))
        L = max(L, np.linalg.norm(d.sensor("angmom").data))
        gross = max(gross, gross_momentum(m, d))
    return drift, L, gross, not any(w.number for w in d.warning)


def check_momentum(T=12.0, amp=90, period=1.5):
    print("3. zero gravity, smooth random joint motion")
    for preset in PRESETS:
        m, d = load(preset)
        drift, L, gross, stable = random_motion(m, d, T, amp, period)
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


def check_flight():
    print("7. flight options: IMU output, weighed modules, air drag, the flight box")
    m, d = load()
    s = Session(m, d)
    s.spin(10, "x")
    while d.time < 2 - 1e-9:
        s.advance()
    rate = math.degrees(np.linalg.norm(d.sensor("imu_gyro").data))
    check(abs(rate - 10) < 0.05 and abs(s.imu_turn() - 20) < 0.2,
          f"IMU: spinning at 10 deg/s for 2 s, its gyro reads {rate:.2f} deg/s and it has turned {s.imu_turn():.2f} deg")
    PRESETS["weighed"] = {**PRESETS["original14"], "module_masses": [0.25] * 12}
    m, _ = load("weighed")
    del PRESETS["weighed"]
    check(abs(sum(m.body_mass) - 3.0) < 1e-9, f"12 weighed modules of 250 g make a {sum(m.body_mass):.3f} kg chain")
    after = []
    for air in (False, True):
        m, d = load(air=air)
        s = Session(m, d)
        s.spin(20, "z")
        while d.time < 10 - 1e-9:
            s.advance()
        after.append(s.spin_rate())
    check(abs(after[0] - 20) < 0.01 and 15 < after[1] < 19.8,
          f"air drag: spinning at 20 deg/s, after 10 s it's {after[1]:.2f} deg/s with cabin air, {after[0]:.2f} without")
    m, d = load(flight=dict(box_m=[2.5, 1.3, 1.9], strings=[]))
    s, hit = Session(m, d), None
    start = d.ncon
    m.opt.gravity[:] = (0, 0, -0.0981)
    while hit is None and d.time < 8:
        s.advance()
        hit = d.time if d.ncon else None
    fall = held_height([2.5, 1.3, 1.9]) - m.geom("m1_sphere").size[0] - FLIGHT["net_height_mm"] / 1000
    want = math.sqrt(2 * fall / 0.0981)
    check(not start and hit is not None and abs(hit - want) < 0.2,
          f"flight box: nothing touches at the start in the middle; with 0.01 g left over, the chain reaches the net after "
          f"{f'{hit:.2f} s' if hit else 'never'} (falling {fall:.2f} m: {want:.2f} s)")


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


def check_anchors():
    print("10. anchors: where an end starts attached, the clamp, grab on touch, let go, the grip column")
    P, cols = PRESETS["original14"]["pitch"], [f"m{i}_{j}" for i in range(1, 13) for j in ("yaw", "pitch")]
    zero, fold = [0] * 24, [0] * 24
    fold[cols.index("m6_pitch")] = 90
    with tempfile.TemporaryDirectory() as tmp:
        def csv_file(name, rows, extra=()):
            path = os.path.join(tmp, name)
            with open(path, "w") as f:
                f.write("\n".join([",".join(cols + list(extra))] + [",".join(map(str, r)) for r in rows]) + "\n")
            return path
        seq = csv_file("fold.csv", [zero, fold])

        def session(attach, log=None, sequence=seq):
            setup = setup_file.new(sequence=sequence, ee1=setup_file.end("anchor", attach=attach))
            m, d = load(**setup_file.build_args(setup))
            s = Session(m, d, log)
            s.use_setup(setup)
            return m, d, s

        floor, wall = {"at": [0.0, 0.0, 0.0], "facing": "up", "roll_deg": 0.0}, {"at": [0.0, -0.9, 1.8], "facing": "+y"}
        m, d, s = session(floor)
        R, e1, m1, m12 = m.geom("ee1_sphere").size[0], m.body("ee1").id, m.body("m1").id, m.body("m12").id
        ok = (np.allclose(d.xpos[e1], (0, 0, R)) and np.allclose(d.xpos[m12], (0, 0, R + 12 * P))
              and np.allclose(d.xmat[m1].reshape(3, 3)[:, 0], (1, 0, 0)) and s.ends["ee1"].clamped)
        m, d, s = session({**wall, "roll_deg": 90.0})
        ok = ok and (np.allclose(d.xpos[e1], (0, -0.9 + R, 1.8)) and np.allclose(d.xpos[m12], (0, -0.9 + R + 12 * P, 1.8))
                     and np.allclose(d.xmat[m1].reshape(3, 3)[:, 0], (0, 0, -1)) and m.geom("plate").id >= 0)
        check(ok, "placement: ee1's tip sits on the floor or on a wall plate, the chain leaves along the surface's normal, "
              "pitch axes start level and roll turns them; it starts clamped")

        log = os.path.join(tmp, "log.csv")
        m, d, s = session(floor, log)
        p0, moved = d.xpos[e1].copy(), 0.0
        s.play(seq, move=2, hold=0.5)
        while s.seq_t < s.duration() - 1e-9:
            s.advance()
            moved = max(moved, np.linalg.norm(d.xpos[e1] - p0))
        s.log.close()
        s.log = None
        head = open(log).readline().strip().split(",")
        peak = s.ends["ee1"].peak
        check(moved < 1e-3 and peak > 0.1 and head[-4:] == ["ee1_held", "ee1_load_N", "ee1_torque_Nm", "ee1_thrust_N"],
              f"the clamp holds ee1 within {moved * 1000:.2f} mm while m6 folds 90 deg (peak load {peak:.2f} N); "
              "the log gets 4 columns for the end")

        s.grip("ee1", False)  # let go, lift it 3 cm, hold again and drop it onto the floor
        d.qpos[2] += 0.03
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        s.grip("ee1", True)
        m.opt.gravity[:] = (0, 0, -9.81)
        t0 = d.time
        while d.time < t0 + 1:
            s.advance()
        grabbed = s.ends["ee1"].clamped and abs(d.xpos[e1][2] - R) < 2e-3
        m, d, s = session({})  # floating, lying along +y: dropped on its side, the tip never faces the floor
        s.grip("ee1", True)
        m.opt.gravity[:] = (0, 0, -9.81)
        while d.time < 3:
            s.advance()
        check(grabbed and not s.ends["ee1"].clamped and d.ncon > 0,
              "grab on touch: told to hold, a falling ee1 latches onto the floor tip-first and is pulled onto it; "
              "dropped on its side it doesn't latch")

        m, d, s = session(wall)
        s.play(seq, move=2, hold=0.5)
        while s.seq_t < 1.5:
            s.advance()
        s.grip("ee1", False)
        s.advance()
        L0, v0 = d.sensor("angmom").data.copy(), d.sensor("com_vel").data.copy()
        while s.seq_t < s.duration() - 1e-9:
            s.advance()
        L1, v1 = d.sensor("angmom").data, d.sensor("com_vel").data
        check(np.linalg.norm(L0) > 1e-3 and np.linalg.norm(L1 - L0) < 1e-4 * np.linalg.norm(L0)
              and np.linalg.norm(v0) > 1e-3 and np.linalg.norm(v1 - v0) < 1e-4 * np.linalg.norm(v0),
              f"let go mid-fold: it drifts off at {np.linalg.norm(v0) * 1000:.1f} mm/s, spinning with the momentum the wall "
              f"was taking (|L| {np.linalg.norm(L0):.3f}), both constant after")

        arch = [0] * 24  # an arch whose far end lands on the floor too: ee12 holds whatever it lands on
        arch[cols.index("m4_pitch")] = arch[cols.index("m9_pitch")] = 90
        setup = setup_file.new(sequence=csv_file("arch.csv", [arch]), ee1=setup_file.end("anchor", attach=floor),
                               ee_n=setup_file.end("anchor", attach={"at": [0.0, -5.0, 0.0], "facing": "up"}))
        m, d = load(**setup_file.build_args(setup))
        s = Session(m, d, None, setup)
        for _ in range(20):
            s.advance()
        check(s.ends["ee1"].clamped and s.ends["ee12"].clamped and abs(d.xpos[m.body("ee12").id][2] - R) < 2e-3,
              "with both ends attached, ee1 sets where the chain starts and ee12 holds what it lands on")
        grip = csv_file("grip.csv", [zero + [1, 1], fold + [1, 1], fold + [0, 1]], ("ee1_grip", "ee12_grip"))
        with contextlib.redirect_stdout(io.StringIO()) as said:
            m, d, s = session(floor, sequence=grip)
            s.play(grip, move=1, hold=0.5)
        held = []
        while s.seq_t < s.duration() - 1e-9:
            s.advance()
            if abs(s.seq_t - 2.9) < 0.005 or abs(s.seq_t - 3.1) < 0.005:
                held.append(s.ends["ee1"].clamped)
        check(held == [True, False] and said.getvalue().count("ee12_grip") == 1,
              "the ee1_grip column lets go when row 2's hold starts (3.0 s); ee12_grip without an ee12 is ignored, "
              "with one warning")


def check_suction():
    print("11. suction cups: hold, pop above the pull limit, grab on touch, never the chain")
    with tempfile.TemporaryDirectory() as tmp:
        seq = os.path.join(tmp, "still.csv")
        with open(seq, "w") as f:
            f.write("0" + ",0" * 23 + "\n")
        floor = {"at": [0.0, 0.0, 0.0], "facing": "up", "roll_deg": 0.0}

        def session(attach, **cup):
            setup = setup_file.new(sequence=seq, ee1=setup_file.end("suction", attach=attach, **cup))
            m, d = load(**setup_file.build_args(setup))
            return m, d, Session(m, d, None, setup)

        lifted = {}
        for pull in (15, 25):  # pull m1 straight off the floor, in zero g; the cup's limit is 20 N
            m, d, s = session(floor)
            e1, m1 = m.body("ee1").id, m.body("m1").id
            z0 = d.xpos[e1][2]
            while d.time < 0.5 - 1e-9:
                s.advance()
            d.xfrc_applied[m1, 2] = pull
            while d.time < 1.5 - 1e-9:
                s.advance()
            lifted[pull] = (d.xpos[e1][2] - z0, s.ends["ee1"].held, s.ends["ee1"].pops, s.ends["ee1"].peak)
        ok = lifted[15][0] < 2e-3 and lifted[15][1] and lifted[25][0] > 0.05 and not lifted[25][1] and lifted[25][2] == 1
        check(ok, f"a 20 N cup on the floor holds a 15 N pull (moved {lifted[15][0] * 1000:.1f} mm, pull read "
              f"{lifted[15][3]:.1f} N) and pops at 25 N (counted as 1 pop)")

        m, d, s = session(floor)
        s.grip("ee1", False)
        d.qpos[2] += 0.03
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        s.grip("ee1", True)
        m.opt.gravity[:] = (0, 0, -9.81)
        while d.time < 1:
            s.advance()
        held = s.ends["ee1"].held
        cup = m.geom("ee1_cup").id
        chain = [g for g in range(m.ngeom) if m.geom_bodyid[g] not in (0, m.body("ee1_cup").id) and m.geom_contype[g]]
        never = (m.geom_contype[cup] == m.geom_conaffinity[cup] == 0
                 and all(m.geom_bodyid[g] == 0 for p in range(m.npair) for g in (m.pair_geom1[p], m.pair_geom2[p]) if g != cup))
        check(held and never, "told to hold, a falling cup grabs the floor; the cup can only ever touch fixed things, "
              "never the chain")


def check_thrusters():
    print("12. thrusters: a = F/m, opposite nozzles cancel, side nozzles turn it but can't roll it")
    def run(nozzles, T=2.0):
        m, d = load(ee1="thruster", ee_n="thruster")
        s = Session(m, d)
        for end, k in nozzles:
            s.nozzle(end, k, 1.0)
        c0 = d.subtree_com[1].copy()
        while d.time < T - 1e-9:
            s.advance()
        w = np.linalg.solve(s.inertia(), d.sensor("angmom").data)
        return m, d, s, d.subtree_com[1] - c0, w
    m, d, s, moved, w = run([("ee12", 0)])
    F, M = m.actuator_gainprm[m.actuator("ee12_n0").id][0], sum(m.body_mass)
    want = 0.5 * F / M * 2.0 ** 2
    lit = all(m.site_rgba[m.site(f"ee12_n{k}").id][0] > 0.9 for k in (0,)) and m.site_rgba[m.site("ee12_n1").id][0] < 0.9
    check(abs(-moved[1] - want) < 0.01 * want and np.linalg.norm(moved[[0, 2]]) < 1e-3 * want
          and np.degrees(np.linalg.norm(w)) < 0.1 and lit,
          f"ee12's n0 (0.1 N) on a straight {M:.2f} kg chain: it moves {-moved[1] * 1000:.1f} mm toward ee1 in 2 s "
          f"(F/m gives {want * 1000:.1f}) and turns only {np.degrees(np.linalg.norm(w)):.2f} deg/s; n0 lights up")
    m, d, s, moved, w = run([("ee1", 0), ("ee12", 0)])
    check(np.linalg.norm(moved) < 1e-4 and np.linalg.norm(w) < 1e-6, f"both n0 on a straight chain push against each other: "
          f"it moves {np.linalg.norm(moved) * 1000:.3f} mm")
    def roll_turn(m, d, w):
        axis = d.xmat[m.body("m6").id].reshape(3, 3)[:, 2]  # along the chain
        return np.degrees(abs(w @ axis)), np.degrees(np.linalg.norm(w - (w @ axis) * axis))
    roll, turn = roll_turn(*run([("ee12", 2)])[:2], run([("ee12", 2)])[4])
    check(turn > 5 and roll < 1e-3 * turn, f"a side nozzle (ee12 n2) turns the straight chain end over end "
          f"({turn:.2f} deg/s after 2 s) but can't roll it about its own length ({roll:.4f} deg/s)")
    m, d, s, moved, w = run([("ee12", 1)])
    roll, turn = roll_turn(m, d, w)
    print(f"  info  n1 and n3 do roll it a little ({roll:.1f} deg/s after 2 s): each module's centre of mass sits ~5 mm off "
          "the chain axis, and the chain is easy to roll")
    with tempfile.TemporaryDirectory() as tmp:  # a burst from the sequence; Stop or Pause must cut it, Resume restores it
        seq = os.path.join(tmp, "burst.csv")
        with open(seq, "w") as f:
            f.write(",".join([f"m{i}_{j}" for i in range(1, 13) for j in ("yaw", "pitch")] + ["ee12_n0"]) + "\n"
                    + "0," * 24 + "1\n" + "0," * 24 + "1\n")
        m, d = load(ee1="thruster", ee_n="thruster")
        s = Session(m, d)
        a = m.actuator("ee12_n0").id
        s.play(seq, move=1, hold=1)
        s.advance()
        firing = d.ctrl[a]
        s.command({"cmd": "pause"})
        paused = d.ctrl[a]
        s.command({"cmd": "resume"})
        s.advance()
        resumed = d.ctrl[a]
        s.command({"cmd": "stop"})
        stopped, dim = d.ctrl[a], m.site_rgba[m.site("ee12_n0").id][0] < 0.9
        check(firing == 1 and paused == 0 and resumed == 1 and stopped == 0 and dim,
              "a nozzle the sequence fires goes off on Pause and Stop (and back on with Resume)")


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


def check_flight_parabola():
    print("15. a parabola in the flight box (8.5 cm modules, setups/1a_turn.toml, 1.8 g phases shortened to 7 s)")
    setup = setup_file.load(os.path.join(setup_file.SETUP_DIR, "1a_turn.toml"))
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    start = s.box_com()
    s.fly(hyper_s=7, jitter=False)
    held, pull, top, after, plane = 0.0, 0.0, 0.0, None, None
    with contextlib.redirect_stdout(io.StringIO()):
        while s.fly_t is not None:
            phase, g, T, theta = s.parabola.at(s.fly_t)
            s.advance()
            if T < 0:  # where the chain is in the box, while the plane pitches up
                here = s.box_com()
                held, pull, top = max(held, np.linalg.norm(here - start)), max(pull, s.tension()), max(top, theta)
            if plane is None and s.released is not None:
                plane = math.degrees(s.parabola.pitch(s.fly_t) - s.parabola.pitch(s.fly_t - LOG_DT)) / LOG_DT
                carried = np.cross((math.radians(plane), 0, 0), d.subtree_com[1] - d.xpos[s.cabin])  # the box's speed there
            if after is None and T >= s.release_s + 2:  # 2 s after letting go, nothing pushing it
                after = np.linalg.norm(d.sensor("com_vel").data - carried)
    R, c = s.box_frame(), d.xpos[s.cabin]  # the lowest shell's bottom, above the box floor (in the box's frame)
    low = min((R.T @ (d.geom_xpos[g] - c))[2] - m.geom_size[g][0] for g in s.shells) + setup["flight"]["box_m"][2] / 2
    check(held < 0.005 and pull > 1, f"held by {len(s.strings)} strings through 1 g and 1.8 g as the plane pitches up to "
          f"{math.degrees(top):.0f} deg: the chain stays within {held * 1000:.1f} mm of its place in the box, strings pull up to "
          f"{pull:.1f} N")
    check(after < 0.001 and abs(s.release_spin - plane) < 0.3,
          f"let go at T+{s.released:.1f} s (no jitter): it leaves turning with the plane ({s.release_spin:+.2f} deg/s, the "
          f"plane {plane:+.2f}) and moving with the box where it was held: 2 s later {after * 1000:.2f} mm/s off that")
    check(s.landing is not None and s.dip < FLIGHT["net_height_mm"] / 1000 and low > 0,
          f"the pull-out drops it onto the net at {s.landing:.2f} m/s; it sinks {s.dip * 1000:.0f} mm of the net's "
          f"{FLIGHT['net_height_mm']} mm and never reaches the box floor")
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    s.fly(setup["sequence"], setup["move_s"], setup["hold_s"], hyper_s=7)
    turned = None
    with contextlib.redirect_stdout(io.StringIO()):
        while s.fly_t is not None:
            s.advance()
            if turned is None and s.keys is not None and s.seq_t >= s.duration():
                turned = s.turned  # the turn as the sequence ends, before the pull-out
    check(turned is not None and s.landing is not None, f"with the jitter and the turn sequence, it flies and lands")
    print(f"  info  ... and turns {turned:+.1f} deg about the box's vertical by the sequence's end (-30.3 in perfect zero g, "
          f"let go at rest); {s.flight_result()}")
    setup = setup_file.load(os.path.join(setup_file.SETUP_DIR, "4a_walk.toml"))
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    s.fly(setup["sequence"], setup["move_s"], setup["hold_s"], hyper_s=7, jitter=False)
    with contextlib.redirect_stdout(io.StringIO()):
        while s.keys is None or s.seq_t < s.duration() - 1e-9:
            s.advance()
    walked = -(s.box_com() - s.com0_box)[1]
    check(walked > 0.6 and all(e.pops == 0 for e in s.ends.values()), f"setups/4a_walk: standing on the deck through 1.8 g as "
          f"the box pitches, then walking in zero g on the turning deck: {walked * 1000:.0f} mm, no cup popped")
    p = Play_parabola(7, False, 1, 0.38)
    check(abs(-p.at(p.t0 + 5)[1][2] - 0.38) < 1e-9 and abs(math.degrees(p.top) - math.degrees(math.atan(0.62))) < 1e-9,
          f"a Mars parabola holds 0.38 g and pitches over less: nose up {math.degrees(p.top):.1f} deg (zero g: "
          f"{FLIGHT['pitch_deg']} deg)")
    m, d = load("feetech85")
    s = Session(m, d)
    s.spin(-0.5, "x")
    s.play(os.path.join(os.path.dirname(HERE), "sequences", "reorient_loop.csv"), 1.0, 0.25)
    while s.seq_t < s.duration() - 1e-9:
        s.advance()
    check(abs(s.turned + 20.9) < 0.5, f"why: a free chain let go spinning 0.5 deg/s about the across axis turns "
          f"{s.turned:+.1f} deg, not -30.3 (folding turns that spin into roll about its length), and its IMU turns "
          f"{s.imu_turn():.0f} deg")


def check_countdown():
    print("14. the countdown before a take: Play's commands wait for it, Stop calls it off")
    take = [{"cmd": "reset"}, {"cmd": "countdown", "s": 5}, {"cmd": "spin", "rate": 10}, {"cmd": "play"}]
    c = Commands()
    first, held, then = c.ready(take, 100.0), c.ready([], 104.9), c.ready([], 105.0)
    check([x["cmd"] for x in first] == ["reset"] and held == [] and c.left(102.0) == 3.0
          and [x["cmd"] for x in then] == ["spin", "play"] and c.left(105.0) == 0,
          "reset runs at once; the spin and play wait 5 s, then run together")
    c = Commands()
    c.ready(take, 0.0)
    off, later = c.ready([{"cmd": "stop"}], 2.0), c.ready([], 6.0)
    check([x["cmd"] for x in off] == ["stop"] and later == [] and c.left(2.0) == 0,
          "Stop during the countdown calls the take off: no spin, no play")
    c = Commands()
    c.ready(take, 0.0)
    again = c.ready(take, 3.0)
    check([x["cmd"] for x in again] == ["reset"] and c.left(3.0) == 5 and [x["cmd"] for x in c.ready([], 8.0)] == ["spin", "play"],
          "Play again during the countdown starts a fresh one, and plays once")


def check_end_effectors():
    print("9. end effectors: a module past m1 (on the m1_yaw twist servo) and one past m12")
    m0, _ = load()
    bad = []
    for t1 in EE_TYPES:
        for tn in EE_TYPES:
            m, _ = load(ee1=t1, ee_n=tn)
            fitted = t1 != "none"
            mass = sum(m0.body_mass) + EE_MASS.get(t1, 0) + EE_MASS.get(tn, 0) + (EE_SERVO_MASS if fitted else 0)
            if not (joint_servos(m) == 22 + fitted and m.njnt == 23 + fitted and m.body("m1").id == 1 and m.body("m2").id == 3
                    and abs(sum(m.body_mass) - mass) < 1e-9):
                bad.append(f"{t1}/{tn}")
    check(not bad, f"all {len(EE_TYPES) ** 2} combinations of end types compile: m1_yaw only with ee1, m1 is body 1, "
          "m2 body 3, masses add up" + (f" (wrong: {', '.join(bad)})" if bad else ""))
    for preset in PRESETS:
        if preset != "original14":
            m, _ = load(preset, ee1="anchor", ee_n="thruster")
            check(joint_servos(m) == 23, f"{preset} with an anchor and a thruster: {joint_servos(m)} joint servos, "
                  f"{sum(m.body_mass):.3f} kg")

    m, d = load(ee1="thruster", ee_n="anchor")
    diffs = []
    for b in range(1, m0.nbody):
        name, k = m0.body(b).name, m.body(m0.body(b).name).id
        for f in ("body_mass", "body_inertia", "body_ipos", "body_iquat", "body_quat") + (("body_pos",) if b != 1 else ()):
            if not np.array_equal(getattr(m0, f)[b], getattr(m, f)[k]):
                diffs.append(f"{name} {f}")
    for j in range(1, m0.njnt):
        k = m.joint(m0.joint(j).name).id
        diffs += [f"{m0.joint(j).name} {f}" for f in ("jnt_axis", "jnt_pos", "jnt_range")
                  if not np.array_equal(getattr(m0, f)[j], getattr(m, f)[k])]
    for a in range(m0.nu):
        k = m.actuator(m0.actuator(a).name).id
        diffs += [f"{m0.actuator(a).name} {f}" for f in ("actuator_gainprm", "actuator_biasprm", "actuator_dynprm",
                                                         "actuator_ctrlrange", "actuator_forcerange")
                  if not np.array_equal(getattr(m0, f)[a], getattr(m, f)[k])]
    check(not diffs, "fitting end effectors leaves the 12 modules, their joints and servos exactly as they were"
          + (f": {', '.join(diffs[:5])}" if diffs else ""))

    P = PRESETS["original14"]["pitch"]
    mujoco.mj_forward(m, d)
    R1, p1 = d.xmat[1].reshape(3, 3).copy(), d.xpos[1].copy()
    j, e1, e12 = m.joint("m1_yaw").id, m.body("ee1").id, m.body("ee12").id
    at = lambda b: R1.T @ (d.xpos[b] - p1)
    ok = np.allclose(R1.T @ d.xaxis[j], (0, 0, -1)) and np.allclose(at(e1), (0, 0, -P)) and np.allclose(at(e12), (0, 0, 12 * P))
    d.qpos[m.jnt_qposadr[j]] = math.radians(30)
    mujoco.mj_kinematics(m, d)
    ok = ok and np.allclose(at(e1), (0, 0, -P)) and np.allclose(R1.T @ d.xmat[e1].reshape(3, 3), rot((0, 0, 1), math.radians(-30)))
    check(ok, "m1_yaw twists about the chain axis (-Z in m1's frame, so +30 deg turns the m12 side +30 deg, like every "
          "yaw); ee1 sits one pitch before m1, ee12 one pitch past m12")

    m, d = load(ee1="anchor", ee_n="thruster")
    drift, L, gross, stable = random_motion(m, d)
    check(stable and drift < 1e-4 and L < 1e-4 * gross,
          f"zero g, both ends fitted, 12 s of random motion including m1_yaw: COM moved {drift * 1000:.4f} mm, "
          f"|L| max {L:.1e} vs {gross:.2f} kg m^2/s ({L / gross:.0e}){'' if stable else ', UNSTABLE'}")

    with tempfile.TemporaryDirectory() as tmp:  # a sequence that twists m1_yaw to 20 deg
        seq = os.path.join(tmp, "twist.csv")
        with open(seq, "w") as f:
            f.write(",".join(f"m{i}_{j}" for i in range(1, 13) for j in ("yaw", "pitch")) + "\n20" + ",0" * 23 + "\n")
        m, d = load(ee1="anchor")
        s = Session(m, d)
        keys = read_sequence(seq, m)
        s.play(seq, move=1, hold=1)
        while d.time < 1 - 1e-9:
            s.advance()
        got = math.degrees(d.qpos[m.jnt_qposadr[m.joint("m1_yaw").id]])
        m0, _ = load()
        with contextlib.redirect_stdout(io.StringIO()) as said:
            keys0 = read_sequence(seq, m0)
        check(s.nj == 23 and np.isclose(keys[0, 0], math.radians(20)) and abs(got - 20) < 1 and not keys0.any()
              and "m1_yaw has no servo" in said.getvalue(),
              f"the m1_yaw column drives ee1's twist servo ({got:.1f} deg after 1 s for 20); "
              "without ee1 it's ignored with a warning")

        def refused(text, *words):  # load() must refuse this setup with an error that names the problem
            path = os.path.join(tmp, "bad.toml")
            with open(path, "w") as f:
                f.write(text)
            try:
                setup_file.load(path)
            except ValueError as e:
                return all(w in str(e) for w in words)
            return False
        check(refused('sequence = "fold_sequence.csv"\ngraviy = 9.81\n', "graviy", "bad.toml")
              and refused('sequence = "nowhere.csv"\n', "nowhere.csv")
              and refused('modules = 6\n[ee12]\ntype = "anchor"\n', "[ee6]")
              and refused('[ee1]\ntype = "hook"\n', "hook")
              and refused('speed = 0\n', "speed") and refused('move_s = 0\n', "move_s") and refused('hold_s = -1\n', "hold_s")
              and refused('[ee1]\ntype = "anchor"\nthrust_N = 1\n', "thrust_N") and refused('ee1 = "anchor"\n', "table")
              and refused('[ee1]\ntype = "anchor"\nattach = { rod_m = 0.3 }\n', "[rod]") and refused('speed = \n', "bad.toml")
              and refused('sequence = "."\n', "sequences"),
              "setup files: a misspelt key, a missing sequence, the wrong far-end table, an unknown type, timings that "
              "can't play, settings for another type, an attach without its rod and bad TOML are refused, naming the problem")
        path = os.path.join(tmp, "round.toml")
        rod = {"from": [0.0, -1.0, 1.0], "to": [0.0, 1.0, 1.0], "diameter_mm": 35.0}
        mine = setup_file.new(sequence=seq, move_s=1.5, spin_deg_s=[0.0, 0.0, 0.5], modules=8, rod=rod,
                              flight={**setup_file.FLIGHT_DEFAULTS, "strings": [1, 8], "jitter": False},
                              ee1=setup_file.end("anchor", attach={"at": [0.0, -0.9, 1.8], "facing": "+y", "roll_deg": 90.0}),
                              ee_n=setup_file.end("thruster", thrust_N=0.2))
        setup_file.save(path, mine, "a round trip")
        same = setup_file.load(path) == mine and setup_file.about(path) == "a round trip"
        theirs = setup_file.new(sequence=seq, rod=rod, ee1=setup_file.end("suction", pull_N=15.0,
                                                                          attach={"rod_m": 0.4, "facing": "up", "roll_deg": 0.0}))
        setup_file.save(path, theirs)
        same = same and setup_file.load(path) == theirs
        ours = setup_file.new(sequence=os.path.join(HERE, "..", "sequences", "fold_sequence.csv"))
        setup_file.save(path, ours)
        written = open(path).read()
        same = same and setup_file.load(path)["sequence"] == FOLD_SEQ and written.startswith('sequence = "fold_sequence.csv"')
        check(same and written.count("\n") == 3, "Save as then Load gives back the same setup: placements, rod, settings "
              "per type, and sequences in sequences/ or elsewhere; a file holds only sequence, timing and what isn't default")

    one = setup_file.single_axis_spin
    two = one([0, 2, 0.5])
    check(one([0, 0, 0.5]) == (0.5, "z", "") and two[:2] == (2.0, "y") and "z" in two[2] and one([0, 0, 0]) == (0.0, "z", ""),
          "a setup's spin, for the control window's one-axis Spin box: the largest axis, with a warning if others are dropped")

    names, bad = sorted(glob.glob(os.path.join(setup_file.SETUP_DIR, "*.toml"))), []
    for path in names:
        try:
            setup = setup_file.load(path)
            m, _ = load(**setup_file.build_args(setup))
            read_sequence(setup["sequence"], m)
            assert setup_file.about(path), "no line at the top saying what it shows"
            assert setup_file.summary(setup)
        except Exception as e:
            bad.append(f"{os.path.basename(path)}: {e}")
    check(names and not bad, f"all {len(names)} setups in setups/ load, build, read their sequence and say what they show"
          + (f": {'; '.join(bad)}" if bad else ""))
    missing = [name for _, name in setup_file.DEMOS if not os.path.isfile(os.path.join(setup_file.SETUP_DIR, name + ".toml"))]
    check(setup_file.DEMOS and not missing, f"each of the control window's {len(setup_file.DEMOS)} demo buttons has its setup"
          + (f" (missing: {', '.join(missing)})" if missing else ""))
    csvs = sorted(glob.glob(os.path.join(setup_file.SEQ_DIR, "*.csv")))
    silent = [os.path.basename(p) for p in csvs if not setup_file.about(p)]
    pole = setup_file.summary(setup_file.load(os.path.join(setup_file.SETUP_DIR, "5a_pole_climb.toml")))
    check(not silent and setup_file.users(os.path.join(setup_file.SEQ_DIR, "pole_climb.csv")) == ["5a_pole_climb"]
          and "rod" in pole and "wrapped" in pole, f"all {len(csvs)} sequences say what they do in their first line, and a "
          "sequence knows the setups that use it (pole_climb.csv: 5a_pole_climb, whose pole is in the setup, not the CSV)"
          + (f"; no first line: {', '.join(silent)}" if silent else ""))


if __name__ == "__main__":
    check_compile()
    check_axes()
    check_momentum()
    check_contacts()
    check_gravity()
    check_spin()
    check_flight()
    renders()
    check_end_effectors()
    check_anchors()
    check_suction()
    check_thrusters()
    check_pole_climb()
    check_countdown()
    check_flight_parabola()
    print("\nALL CHECKS PASSED" if not failures else f"\n{len(failures)} CHECK(S) FAILED")
    raise SystemExit(bool(failures))
