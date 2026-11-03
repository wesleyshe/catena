"""The viewer window: MuJoCo's passive viewer (under mjpython on a Mac). It steps the Session in real time (or Playback x
faster), takes the keys and the control window's commands (holding a take through its countdown), draws the readouts
and the big text, and sends the readouts back to the control window 5 times a second.

Keyboard: PageUp / PageDown select module, Up / Down nudge its pitch, Right / Left nudge its yaw, Space pauses /
resumes the physics. The selected module turns orange.
"""
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

from sequence import LOG_DT, actuator, modules
from session import HERE, stamp

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

