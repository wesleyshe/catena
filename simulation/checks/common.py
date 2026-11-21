"""Helpers every check uses: check() and the failures it counts, load() for a model, and the motion and momentum tools."""
import math
import os

import mujoco
import numpy as np
from PIL import Image

from sequence import joint_servos
from session import load_model

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # simulation/
FOLD_SEQ = os.path.join(os.path.dirname(HERE), "sequences", "fold_sequence.csv")
failures = []


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg)
    if not ok:
        failures.append(msg)


def load(preset="original14", n=12, slip_ring=False, **opts):
    m = load_model(preset=preset, n=n, slip_ring=slip_ring, **opts)
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
