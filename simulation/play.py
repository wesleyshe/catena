"""Drive the chain: interactive viewer, fold-sequence playback, and logging.

  mjpython play.py                                  viewer: Control-panel sliders + keyboard
  mjpython play.py --seq ../sequences/fold_sequence.csv       viewer, playing the sequence
  .venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless    no viewer, run and log only
  .venv/bin/python play.py --setup setups/1a_turn.toml --headless   a whole run from a setup file (setups/); one with
      [flight] flies a whole parabola: held by strings through the 1.8 g pull-up, let go at zero g, onto the net
  mjpython play.py --gravity 9.81                   start with Earth gravity: the chain drops onto the floor
  .venv/bin/python play.py --seq ../sequences/reorient_loop.csv --headless --speed 4 --spin 0 0 1 --air
      flight-like: the sequence 4x faster (to fit a parabola), let go with 1 deg/s of spin, cabin air
The control window (panel.py, or double-click 'Chain Sim.command') starts this with --panel and sends it commands.

Keyboard (viewer): PageUp / PageDown select module, Up / Down nudge its pitch,
Right / Left nudge its yaw, Space pauses / resumes the physics.
The selected module turns orange.
On a Mac the viewer needs mjpython (it ships in .venv/bin); --headless works with plain python.
"""
import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time

import mujoco
import numpy as np

import setup_file
from flight import Parabola  # noqa: F401 (play.py's names, for verify.py and anyone's scripts: from play import ...)
from sequence import LOG_DT, actuator, joint_servos, modules, read_sequence, target  # noqa: F401
from session import HERE, PARABOLA_S, Session, load_model, stamp

STEP_DEG = 5.0      # keyboard nudge per press
KEYS = dict(next=266, prev=267, pitch_up=265, pitch_down=264, yaw_up=262, yaw_down=263, pause=32)  # GLFW codes
BASE_RGBA, SELECT_RGBA = (0.78, 0.78, 0.80, 1), (1.0, 0.6, 0.2, 1)
CALL_OFF = ("pause", "stop", "straighten", "reset", "quit")  # sent during a countdown, these call the take off
WINDOW_NAME = "CATENA viewer"  # macOS saves the viewer window's size and place under this name, and reopens it there


class Commands:
    """The control window's commands, in the order sent. A countdown {"cmd": "countdown", "s": 5} holds back the ones
    sent after it (a take's spin and play), and the viewer holds the physics, until it ends. Pause, Stop, Reset or
    quit sent during it call the take off."""

    def __init__(self):
        self.waiting, self.until = [], 0.0

    def ready(self, new, now):
        """The commands to run now; new are the ones just received, now is wall-clock time (s)."""
        self.waiting += new
        off = [i for i, c in enumerate(self.waiting) if c["cmd"] in CALL_OFF]
        if now < self.until and off:
            self.waiting, self.until = self.waiting[off[-1]:], 0.0
        out = []
        while self.waiting and now >= self.until:
            c = self.waiting.pop(0)
            if c["cmd"] == "countdown":
                self.until = now + float(c["s"])
            else:
                out.append(c)
        return out

    def left(self, now):
        return max(0.0, self.until - now)


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
    n = modules(model)
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
    steps, shown = 0.0, (time.time(), data.time)  # steps owed this frame (playback); when the rate was last measured
    commands, texts, count = Commands(), [], 0  # count: the countdown second on screen (0: none)
    if sys.platform == "darwin":
        import glfw
        glfw.window_hint_string(glfw.COCOA_FRAME_NAME, WINDOW_NAME)
    with mujoco.viewer.launch_passive(model, data, key_callback=pressed.append) as viewer:
        cabin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cabin")

        def camera(on_cabin):
            """The flight box: watch from the cabin (turning with the plane: the box looks still), or from outside, fixed
            in space, where the box pitches with the plane. Esc in the viewer also frees the camera."""
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            if cabin >= 0 and on_cabin:
                viewer.cam.type, viewer.cam.fixedcamid = mujoco.mjtCamera.mjCAMERA_FIXED, cabin
            elif cabin >= 0:
                viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 35, -12, 2.2 * model.stat.extent

        with viewer.lock():  # the passive viewer ignores <visual><global azimuth/elevation>; apply them here
            camera(True)
        while viewer.is_running():
            start = time.time()
            with viewer.lock():
                while pressed:
                    handle(pressed.pop(0))
                new = []
                while not inbox.empty():
                    new.append(inbox.get())
                for c in commands.ready(new, start):
                    if c["cmd"] == "quit":
                        viewer.close()
                    elif c["cmd"] == "camera":
                        camera(bool(c["cabin"]))
                    elif c["cmd"] == "screenshot":
                        screenshot(model, data, viewer)
                    else:
                        try:
                            s.command(c)
                        except Exception as e:  # a bad CSV must not kill the viewer
                            print(f"command {c} failed: {e}")
                left = commands.left(start)  # the physics waits out a countdown, so the take starts at its T-0
                steps += frame / LOG_DT * s.playback if s.running and not left else 0
                for _ in range(int(steps)):
                    s.advance()
                steps -= int(steps)
                info = s.readouts() if start >= next_readout else None
                if info:  # what the playback actually reaches: a slow computer can fall short of a high one
                    rate = (data.time - shown[1]) / max(1e-6, start - shown[0])
                    info["playback"] = f"x{s.playback:g}" + (f" (reaching x{rate:.1f})" if s.running and rate < 0.9 * s.playback else "")
                    shown = (start, data.time)
                    if left:
                        info["sequence"] = f"starts in {math.ceil(left)} s"
            viewer.sync()
            if info:
                texts = [(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "\n".join(info), "\n".join(info.values()))]
            if info or math.ceil(left) != count:  # T-5 ... T-1 in big type, then the take starts; in flight, the g
                count = math.ceil(left)
                title = f"T-{count}" if count else s.banner()
                big = [(mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_TOP, title, "")] if title else []
                viewer.set_texts(texts + big)
            if info:
                next_readout = start + 0.2
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
    ap.add_argument("--playback", type=float, default=1.0,
                    help="viewer: show the simulation this many times faster than real time; the physics doesn't change")
    ap.add_argument("--spin", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="start spinning at this rate (deg/s) about world x, y, z: the leftover spin when it's let go")
    ap.add_argument("--air", action="store_true", help="cabin air drag on the shells (when building the model)")
    ap.add_argument("--setup", help="setup file (setups/*.toml): the model and the settings above; flags given here win")
    ap.add_argument("--panel", type=int, help=argparse.SUPPRESS)  # port of the control window
    a = ap.parse_args()
    try:
        setup = setup_file.load(a.setup) if a.setup else None
    except (OSError, ValueError) as e:  # say what's wrong with the file, without a traceback
        raise SystemExit(f"can't use that setup: {e}")
    if setup:  # the setup's values become the defaults, so flags on the command line still win
        ap.set_defaults(move=setup["move_s"], hold=setup["hold_s"], speed=setup["speed"], gravity=setup["gravity_m_s2"],
                        air=setup["air"], playback=setup["playback"])
        if not a.panel:  # from the control window, the sequence (and its spin) waits for Play
            ap.set_defaults(seq=setup["sequence"] or None, spin=setup["spin_deg_s"] if any(setup["spin_deg_s"]) else None)
        a = ap.parse_args()

    if a.model:
        model = mujoco.MjModel.from_xml_path(a.model)
    else:  # always the current hardware spec, never a stale chain.xml
        opts = {**setup_file.build_args(setup), "air": a.air} if setup else dict(air=a.air)
        model = load_model(**opts)
    if a.gravity is not None:
        model.opt.gravity[:] = (0, 0, -a.gravity)
    session = Session(model, mujoco.MjData(model), a.log or None, setup if setup and not a.model else None)
    session.speed, session.playback = a.speed, a.playback
    fly = bool(a.headless and setup and setup["flight"] and not a.model)  # a flight setup flies its parabola
    for axis, rate in zip("xyz", () if fly else a.spin or ()):  # in a flight, the spin comes as the strings let go
        session.spin(rate, axis)
    if fly:
        session.fly(a.seq or "", a.move, a.hold)
    elif a.seq:
        session.play(a.seq, a.move, a.hold)
    try:
        if a.headless:
            assert a.seq or fly, "--headless needs --seq"
            if setup:
                print(f"{os.path.basename(a.setup)}: {setup_file.summary(setup)}")
            while session.fly_t is not None if fly else session.seq_t < session.duration() - 1e-9:
                session.advance()
            secs = session.duration() / session.speed if session.keys is not None else 0
            print(f"ran {secs:g} s" + (f" (longer than a {PARABOLA_S:g} s parabola)" if secs > PARABOLA_S else ""))
            if not fly:  # a flight prints its own summary as it lands
                print(session.summary())
        else:
            run_viewer(session, a.panel)
    finally:
        if session.log:
            session.log.close()
            print(f"log written to {os.path.relpath(session.log.path, HERE)}")
