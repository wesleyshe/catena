"""Sections 9 to 12: the end effectors in the model, anchors, suction cups and thrusters."""
import contextlib
import io
import math
import os
import tempfile

import mujoco
import numpy as np

import setup_file
from make_chain import PRESETS
from make_ends import EE_MASS, EE_SERVO_MASS, EE_TYPES
from sequence import joint_servos, read_sequence
from session import Session
from checks.common import check, load, rot, random_motion


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
