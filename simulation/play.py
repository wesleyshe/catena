"""Drive the chain: interactive viewer, fold-sequence playback, and logging.

  mjpython play.py                                  viewer: Control-panel sliders + keyboard
  mjpython play.py --seq ../sequences/fold_sequence.csv       viewer, playing the sequence
  .venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless    no viewer, run and log only
  mjpython play.py --gravity 9.81                   start with Earth gravity: the chain drops onto the floor
  .venv/bin/python play.py --seq ../sequences/reorient_loop.csv --headless --speed 4 --spin 0 0 1 --air --cage
      flight-like: the sequence 4x faster (to fit a parabola), let go with 1 deg/s of spin, cabin air, net cage
The control window (panel.py, or double-click 'Chain Sim.command') starts this with --panel and sends it commands.

Keyboard (viewer): PageUp / PageDown select module, Up / Down nudge its pitch,
Right / Left nudge its yaw, Space pauses / resumes the physics.
The selected module turns orange.
On a Mac the viewer needs mjpython (it ships in .venv/bin); --headless works with plain python.
"""
import argparse
import csv
import json
import math
import os
import queue
import socket
import threading
import time

import mujoco
import numpy as np

from make_chain import FLIGHT, build

HERE = os.path.dirname(os.path.abspath(__file__))
AXES = dict(x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1))  # world axes: x across the start pose, y along it, z vertical
PARABOLA_S = FLIGHT["parabola_s"]                  # seconds of microgravity per parabola
STEP_DEG = 5.0      # keyboard nudge per press
LOG_DT = 0.01       # seconds between log rows and between sequence target updates
KEYS = dict(next=266, prev=267, pitch_up=265, pitch_down=264, yaw_up=262, yaw_down=263, pause=32)  # GLFW codes
BASE_RGBA, SELECT_RGBA = (0.78, 0.78, 0.80, 1), (1.0, 0.6, 0.2, 1)


def actuator(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def read_sequence(path, model):
    """Keyframes from a CSV as ctrl vectors (rad). Columns are degrees in the order m1_yaw, m1_pitch, ...,
    mN_yaw, mN_pitch; an optional header row names them. m1_yaw and mN_pitch don't exist and are ignored."""
    with open(path) as f:
        rows = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith("#")]
    n = (model.nu + 2) // 2
    names = [f"m{i}_{j}" for i in range(1, n + 1) for j in ("yaw", "pitch")]
    try:
        float(rows[0][0])
    except ValueError:
        names, rows = [c.strip() for c in rows[0]], rows[1:]
    keys = []
    for r in rows:
        assert len(r) == len(names), f"{path}: expected {len(names)} columns, got {len(r)}"
        q = np.zeros(model.nu)
        for name, val in zip(names, map(float, r)):
            a = actuator(model, name)
            if a >= 0:
                q[a] = math.radians(val)
            elif val != 0:
                print(f"warning: {name} has no servo, ignoring {val:g} deg")
        keys.append(q)
    return np.array(keys)


def target(keys, t, move, hold):
    """Hold each keyframe for `hold` s, then ease to the next one over `move` s."""
    k = int(t // (move + hold))
    if k >= len(keys) - 1:
        return keys[-1]
    u = min(1.0, max(0.0, (t - k * (move + hold) - hold) / move))
    return keys[k] + (0.5 - 0.5 * math.cos(math.pi * u)) * (keys[k + 1] - keys[k])


class Logger:
    """Writes time, chain COM, total angular momentum about the COM, max |actuator torque|, contacts, and what the
    flight IMU will report: its module's attitude (quaternion, world frame) and rotation rate (rad/s, its own frame)."""

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["time", "com_x", "com_y", "com_z", "L_x", "L_y", "L_z", "L_norm", "max_torque", "contacts",
                         "imu_qw", "imu_qx", "imu_qy", "imu_qz", "imu_wx", "imu_wy", "imu_wz"])

    def __call__(self, data):
        com, L = data.sensor("com").data, data.sensor("angmom").data
        self.w.writerow([f"{data.time:.3f}", *(f"{x:.7g}" for x in com), *(f"{x:.4g}" for x in L),
                         f"{np.linalg.norm(L):.4g}", f"{np.abs(data.actuator_force).max():.4g}", data.ncon,
                         *(f"{x:.6g}" for x in data.sensor("imu_quat").data),
                         *(f"{x:.4g}" for x in data.sensor("imu_gyro").data)])

    def close(self):
        self.f.close()


class Session:
    """Simulation state shared by the keyboard, the control window and the main loop."""

    def __init__(self, model, data, log_path=None):
        self.m, self.d = model, data
        self.keys, self.seq_name, self.move, self.hold = None, "", 3.0, 1.0
        self.seq_t, self.playing, self.speed = 0.0, False, 1.0
        self.running = True  # False freezes the physics (time, motion, sequence clock, log)
        self.mid = model.body(f"m{max(1, (model.nu + 2) // 4)}").id  # middle module (m6 of 12): its heading = "turned"
        self.log = Logger(log_path) if log_path else None
        self.reset()

    def reset(self):
        """Back to the floating start pose, straight, at rest; stops any sequence."""
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        self.keys, self.playing = None, False
        self.com0, self.q0 = self.d.subtree_com[1].copy(), self.d.sensor("imu_quat").data.copy()
        self.turned, self.heading_was = 0.0, self.heading()
        if self.log:
            self.log(self.d)

    def inertia(self):
        """Whole chain's inertia tensor about its centre of mass, world frame."""
        d, m, c, I = self.d, self.m, self.d.subtree_com[1], np.zeros((3, 3))
        for b in range(1, m.nbody):
            R, r = d.ximat[b].reshape(3, 3), d.xipos[b] - c
            I += R @ np.diag(m.body_inertia[b]) @ R.T + m.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
        return I

    def spin_rate(self):
        """Whole-chain spin about the vertical (deg/s): angular momentum / inertia, i.e. the rate if the shape froze."""
        return math.degrees(np.linalg.solve(self.inertia(), self.d.sensor("angmom").data)[2])

    def spin(self, rate, axis="z"):
        """Set the whole chain's spin about world axis x, y or z (through its centre of mass) to `rate` deg/s, keeping
        its shape, its joint motions and its spin about the other two axes. axis="all" sets all three (rate 0 = Stop
        spin). A push from outside: it changes the angular momentum. Also models leftover spin when the chain is let go."""
        d = self.d
        for a in ("x", "y", "z") if axis == "all" else (axis,):
            mujoco.mj_forward(self.m, d)
            n = np.array(AXES[a], float)
            dw = n * (math.radians(rate) - n @ np.linalg.solve(self.inertia(), d.sensor("angmom").data))
            d.qvel[:3] += np.cross(dw, d.xpos[1] - d.subtree_com[1])  # add a rigid rotation about the centre of mass
            d.qvel[3:6] += d.xmat[1].reshape(3, 3).T @ dw
        mujoco.mj_forward(self.m, d)

    def imu_turn(self):
        """How far the IMU's module has turned since the last reset, in 3D (deg), as the flight IMU will see it."""
        return math.degrees(2 * math.acos(min(1.0, abs(float(self.q0 @ self.d.sensor("imu_quat").data)))))

    def heading(self):
        """Compass direction (rad) the middle module points along the chain, seen from above; None if near vertical."""
        z = self.d.xmat[self.mid].reshape(3, 3)[:, 2]
        return math.atan2(z[1], z[0]) if np.hypot(z[0], z[1]) > 0.2 else None

    def track_turn(self):
        h = self.heading()
        if h is not None and self.heading_was is not None:
            self.turned += math.degrees((h - self.heading_was + math.pi) % (2 * math.pi) - math.pi)
        self.heading_was = h

    def play(self, path, move=3.0, hold=1.0):
        self.keys, self.move, self.hold = read_sequence(path, self.m), move, hold
        self.seq_name, self.seq_t, self.playing = os.path.basename(path), 0.0, True

    def duration(self):
        return (len(self.keys) - 1) * (self.move + self.hold) + self.hold

    def advance(self):
        """Simulate LOG_DT seconds. Sequence targets update once per LOG_DT, like commands on a servo bus."""
        if self.keys is not None and self.playing:
            self.d.ctrl[:] = target(self.keys, self.seq_t, self.move, self.hold)
            self.seq_t += LOG_DT * self.speed
        mujoco.mj_step(self.m, self.d, nstep=max(1, round(LOG_DT / self.m.opt.timestep)))
        mujoco.mj_forward(self.m, self.d)  # sensors for the current state (mj_step leaves them one step behind)
        self.track_turn()
        if self.log:
            self.log(self.d)

    def set_log(self, on):
        if self.log:
            self.log.close()
            self.log = None
        if on:
            self.log = Logger(os.path.join(HERE, "logs", f"run_{stamp()}.csv"))
            self.log(self.d)

    def command(self, c):
        """Apply one command from the control window, e.g. {"cmd": "play", "file": ".../sequences/fold_sequence.csv"}."""
        cmd = c["cmd"]
        if cmd == "play":
            self.play(c["file"], c.get("move", 3.0), c.get("hold", 1.0))
        elif cmd == "pause":
            self.playing = False
        elif cmd == "resume":
            self.playing = self.keys is not None
        elif cmd == "restart":
            self.seq_t, self.playing = 0.0, self.keys is not None
        elif cmd in ("stop", "straighten"):
            self.keys, self.playing = None, False
            if cmd == "straighten":
                self.d.ctrl[:] = 0
        elif cmd == "speed":
            self.speed = float(c["value"])
        elif cmd == "gravity":
            self.m.opt.gravity[:] = (0, 0, -float(c["g"]))
        elif cmd == "physics":
            self.running = bool(c["run"])
        elif cmd == "spin":
            self.spin(float(c["rate"]), c.get("axis", "z"))
        elif cmd == "reset":
            self.reset()
        elif cmd == "log":
            self.set_log(c["on"])

    def readouts(self):
        d, L = self.d, np.linalg.norm(self.d.sensor("angmom").data)
        seq = "none"
        if self.keys is not None:
            done = min(100, 100 * self.seq_t / self.duration())
            seq = f"{self.seq_name} {done:.0f}% ({'playing' if self.playing and done < 100 else 'paused' if not self.playing else 'done'})"
            secs = self.duration() / self.speed
            seq += f", {secs:.0f} s" + ("" if secs <= PARABOLA_S else f": longer than a {PARABOLA_S:g} s parabola")
        return {"time": f"{d.time:.1f} s", "physics": "running" if self.running else "PAUSED", "sequence": seq,
                "gravity": f"{-self.m.opt.gravity[2]:g} m/s^2",
                "spin": f"{self.spin_rate():+.1f} deg/s", "turned": f"{self.turned:+.1f} deg",
                "IMU turn": f"{self.imu_turn():.1f} deg",
                "COM drift": f"{np.linalg.norm(d.subtree_com[1] - self.com0) * 1000:.3f} mm",
                "|L|": f"{L:.1e} kg m^2/s", "max torque": f"{np.abs(d.actuator_force).max():.2f} N m",
                "contacts": str(d.ncon), "log": os.path.relpath(self.log.path, HERE) if self.log else "off"}


def connect_panel(port):
    """Commands from the control window arrive as JSON lines; readouts go back the same way."""
    sock = socket.create_connection(("127.0.0.1", port))
    inbox = queue.Queue()

    def reader():
        for line in sock.makefile("r"):
            inbox.put(json.loads(line))

    threading.Thread(target=reader, daemon=True).start()
    return sock, inbox


def screenshot(model, data, viewer):
    from PIL import Image

    os.makedirs(os.path.join(HERE, "screenshots"), exist_ok=True)
    path = os.path.join(HERE, "screenshots", f"shot_{stamp()}.png")
    r = mujoco.Renderer(model, 1200, 1600)
    r.update_scene(data, viewer.cam, viewer.opt)
    Image.fromarray(r.render()).save(path)
    r.close()
    print(f"screenshot saved to {os.path.relpath(path, HERE)}")


def run_viewer(s, panel_port=None):
    import mujoco.viewer

    model, data = s.m, s.d
    n = (model.nu + 2) // 2
    pressed, sel = [], [2]
    sock, inbox = connect_panel(panel_port) if panel_port else (None, queue.Queue())

    def highlight():
        for i in range(1, n + 1):
            model.geom(f"m{i}_visual").rgba[:] = SELECT_RGBA if i == sel[0] else BASE_RGBA

    def nudge(joint, sign):
        a = actuator(model, f"m{sel[0]}_{joint}")
        if a < 0:
            print(f"module {sel[0]} has no {joint} servo")
            return
        data.ctrl[a] += sign * math.radians(STEP_DEG)
        if model.actuator_ctrllimited[a]:
            data.ctrl[a] = np.clip(data.ctrl[a], *model.actuator_ctrlrange[a])

    def handle(code):
        if code in (KEYS["next"], KEYS["prev"]):
            sel[0] = min(n, max(1, sel[0] + (1 if code == KEYS["next"] else -1)))
            highlight()
        elif code == KEYS["pause"]:
            s.running = not s.running
            print("physics", "running" if s.running else "paused")
            return
        elif code in (KEYS["pitch_up"], KEYS["pitch_down"]):
            nudge("pitch", 1 if code == KEYS["pitch_up"] else -1)
        elif code in (KEYS["yaw_up"], KEYS["yaw_down"]):
            nudge("yaw", 1 if code == KEYS["yaw_up"] else -1)
        else:
            return
        deg = {j: math.degrees(data.ctrl[a]) for j in ("yaw", "pitch") if (a := actuator(model, f"m{sel[0]}_{j}")) >= 0}
        print(f"module {sel[0]}: " + ", ".join(f"{j} {d:+.0f} deg" for j, d in deg.items()))

    highlight()
    frame, next_readout = 2 * LOG_DT, 0.0  # 50 frames per second; readouts 5 times per second
    with mujoco.viewer.launch_passive(model, data, key_callback=pressed.append) as viewer:
        with viewer.lock():  # the passive viewer ignores <visual><global azimuth/elevation>; apply them here
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        while viewer.is_running():
            start = time.time()
            with viewer.lock():
                while pressed:
                    handle(pressed.pop(0))
                while not inbox.empty():
                    c = inbox.get()
                    if c["cmd"] == "quit":
                        viewer.close()
                    elif c["cmd"] == "screenshot":
                        screenshot(model, data, viewer)
                    else:
                        try:
                            s.command(c)
                        except Exception as e:  # a bad CSV must not kill the viewer
                            print(f"command {c} failed: {e}")
                for _ in range(2 if s.running else 0):
                    s.advance()
                info = s.readouts() if start >= next_readout else None
            viewer.sync()
            if info:
                next_readout = start + 0.2
                viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "\n".join(info), "\n".join(info.values())))
                if sock:
                    try:
                        sock.sendall((json.dumps(info) + "\n").encode())
                    except OSError:
                        sock = None
            time.sleep(max(0.0, frame - (time.time() - start)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", help="model XML (default: built fresh from make_chain.py and ../hardware/spec.toml)")
    ap.add_argument("--seq", help="fold sequence CSV (degrees)")
    ap.add_argument("--move", type=float, default=3.0, help="seconds to move between rows")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold each row")
    ap.add_argument("--log", default=os.path.join(HERE, "logs", "run_log.csv"), help='log file; "" for no log')
    ap.add_argument("--headless", action="store_true", help="no viewer; needs --seq")
    ap.add_argument("--gravity", type=float, help="m/s^2 downward, overriding the model (Earth: 9.81)")
    ap.add_argument("--speed", type=float, default=1.0, help="play the sequence this many times faster (e.g. 4)")
    ap.add_argument("--spin", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="start spinning at this rate (deg/s) about world x, y, z: the leftover spin when it's let go")
    ap.add_argument("--air", action="store_true", help="cabin air drag on the shells (when building the model)")
    ap.add_argument("--cage", action="store_true", help="the net cage from hardware/spec.toml (when building the model)")
    ap.add_argument("--panel", type=int, help=argparse.SUPPRESS)  # port of the control window
    a = ap.parse_args()

    if a.model:
        model = mujoco.MjModel.from_xml_path(a.model)
    else:  # always the current hardware spec, never a stale chain.xml
        xml = build(air=a.air, cage=a.cage).replace('meshdir="meshes"', f'meshdir="{HERE}/meshes"')
        model = mujoco.MjModel.from_xml_string(xml)
    if a.gravity is not None:
        model.opt.gravity[:] = (0, 0, -a.gravity)
    session = Session(model, mujoco.MjData(model), a.log or None)
    session.speed = a.speed
    for axis, rate in zip("xyz", a.spin or ()):
        session.spin(rate, axis)
    if a.seq:
        session.play(a.seq, a.move, a.hold)
    try:
        if a.headless:
            assert a.seq, "--headless needs --seq"
            while session.seq_t < session.duration() - 1e-9:
                session.advance()
            secs = session.duration() / session.speed
            print(f"ran {secs:g} s" + (f" (longer than a {PARABOLA_S:g} s parabola)" if secs > PARABOLA_S else ""))
        else:
            run_viewer(session, a.panel)
    finally:
        if session.log:
            session.log.close()
            print(f"log written to {os.path.relpath(session.log.path, HERE)}")
