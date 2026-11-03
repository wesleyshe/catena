"""Press every button of the control window, with its link to the viewer replaced by a recorder, and write down what it
sent: the command lines, and the setup file each (re)opening of the viewer would start from. No viewer opens.

  .venv/bin/python checks/window_recorder.py OUT.txt       # used by same_as_before.py; needs a display (a Mac)
"""
import json
import os
import sys
import tkinter as tk
from tkinter import ttk

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # simulation/
sys.path.insert(0, HERE)
import panel  # noqa: E402
import setup_file  # noqa: E402

lines = []


class Recorder:
    """Stands in for viewer_link.ViewerLink: every open "connects" at the next poll, every send arrives."""

    def __init__(self):
        self.opened = False

    def open(self, setup_path):
        with open(setup_path) as f:
            lines.append("  open: " + f.read().strip().replace("\n", "\n        "))
        self.opened = True

    def close(self):
        pass

    def send(self, cmd, **args):
        lines.append("  send " + json.dumps({"cmd": cmd, **args}))
        return True

    def poll(self):
        connected, self.opened = self.opened, False
        return connected, None, False


panel.ViewerLink = Recorder
root = tk.Tk()
root.withdraw()
p = panel.Panel(root)
p.poll()


def widgets(w):
    for c in w.winfo_children():
        yield c
        yield from widgets(c)


def press(text, prefix=False):
    """Invoke the first button or tick box with this text, then let the window see the viewer connect."""
    for w in widgets(root):
        if isinstance(w, (ttk.Button, ttk.Checkbutton)):
            label = str(w.cget("text"))
            if label == text or prefix and label.startswith(text):
                lines.append(f"press {label}")
                w.invoke()
                p.poll()
                lines.append(f"  status: {p.status.get()}")
                return
    raise SystemExit(f"no button {text!r}")


def load(name):
    lines.append(f"load {name}")
    p.setup_name.set(name)
    p.load_setup()
    p.poll()
    lines.append(f"  about: {p.about.get()}")


lines.insert(0, "startup")
for label, _ in setup_file.DEMOS:
    press(label)
for text in ("Play", "Restart", "Pause", "Resume", "Stop", "Straighten", "Reset pose", "Spin", "Stop spin"):
    press(text)
press("⏸", prefix=True)
press("Also spin like this", prefix=True)
press("Play")
press("Also spin like this", prefix=True)
press("✈", prefix=True)
press("✂", prefix=True)
press("Play")
press("Camera turns", prefix=True)
press("Camera turns", prefix=True)
press("Log to logs/")
press("Log to logs/")
press("Screenshot")
press("Gravity")
press("Gravity")
load("2a_arm")
press("Hold")
press("Let go")
load("3a_U")
press("n0")
press("n0")
for name in p.setups():
    load(name)
    press("Play")
p.quit()
with open(sys.argv[1], "w") as f:
    f.write("\n".join(lines) + "\n")
