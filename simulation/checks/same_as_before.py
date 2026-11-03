"""Is the sim the same as before? For a refactor: save a snapshot, change the code, compare.

  .venv/bin/python checks/same_as_before.py save DIR        # before the change (about 3 minutes)
  .venv/bin/python checks/same_as_before.py compare DIR     # after it: says what differs, exits 1 if anything does
  .venv/bin/python checks/same_as_before.py compare DIR --quick   # only the model XML, the window and stdlib (seconds)

What it records in DIR:
  verify.txt       verify.py's printout
  fold.csv/.txt    the headless fold_sequence.csv log and printout (byte-identical is a contract in AGENTS.md)
  setups/NAME.*    every setup in setups/ flown headless: its log and printout
  xml.txt          SHA-256 of make_chain.build() for every setup, preset, slip ring, end-effector pair and --box
  window.txt       what the control window sends the viewer for each button, and the setup each Load hands it
  stdlib.txt       whether the window's side (panel.py and everything it imports) loads with only the stdlib
Keep DIR on the computer that made it: floating point can differ between computers.
"""
import concurrent.futures
import filecmp
import glob
import hashlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # simulation/
PY = sys.executable
WORKERS = 12
NOT_STDLIB = ("mujoco", "numpy", "trimesh", "PIL", "rhino3dm", "glfw")
STDLIB_CHECK = f"""
import builtins
real = builtins.__import__
def guard(name, *a, **k):
    if name.split('.')[0] in {NOT_STDLIB!r}:
        raise ImportError('not stdlib: ' + name)
    return real(name, *a, **k)
builtins.__import__ = guard
import panel
print('panel.py and everything it imports load with only the stdlib')
"""


def run(args, out):
    """Run a script from simulation/ and save what it printed."""
    p = subprocess.run([PY, *args], cwd=HERE, capture_output=True, text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    with open(out, "w") as f:
        f.write(p.stdout + p.stderr)
    return p.returncode


def xml_hashes():
    sys.path.insert(0, HERE)
    import make_chain
    import setup_file
    h = lambda xml: hashlib.sha256(xml.encode()).hexdigest()
    lines = []
    for path in sorted(glob.glob(os.path.join(setup_file.SETUP_DIR, "*.toml"))):
        lines.append(f"{h(make_chain.build(**setup_file.build_args(setup_file.load(path))))} {os.path.basename(path)}")
    for preset in make_chain.PRESETS:
        for slip in (False, True):
            lines.append(f"{h(make_chain.build(preset, slip_ring=slip))} {preset} slip_ring={slip}")
    for a in make_chain.EE_TYPES:
        for b in make_chain.EE_TYPES:
            lines.append(f"{h(make_chain.build(ee1=a, ee_n=b))} ee1={a} ee_n={b}")
    box = dict(box_m=[x / 1000 for x in make_chain.FLIGHT["box_mm"]], strings=[])
    lines.append(f"{h(make_chain.build(flight=box))} --box")
    lines.append(f"{h(make_chain.build(n=8, gravity=9.81, air=True))} -n 8 --gravity 9.81 --air")
    return "\n".join(lines) + "\n"


def window(out):
    """Every button's commands to the viewer, recorded with the viewer link replaced (no viewer opens)."""
    return run([os.path.join(HERE, "checks", "window_recorder.py"), out], out + ".err")


def snapshot(dest, quick=False):
    os.makedirs(os.path.join(dest, "setups"), exist_ok=True)
    with open(os.path.join(dest, "xml.txt"), "w") as f:
        f.write(xml_hashes())
    run(["-c", STDLIB_CHECK], os.path.join(dest, "stdlib.txt"))
    window(os.path.join(dest, "window.txt"))
    if quick:
        return
    jobs = [(["verify.py"], os.path.join(dest, "verify.txt")),
            (["play.py", "--seq", "../sequences/fold_sequence.csv", "--headless", "--log", os.path.join(dest, "fold.csv")],
             os.path.join(dest, "fold.txt"))]
    for path in sorted(glob.glob(os.path.join(HERE, "setups", "*.toml"))):
        name = os.path.splitext(os.path.basename(path))[0]
        log = os.path.join(dest, "setups", name + ".csv")
        jobs.append((["play.py", "--setup", path, "--headless", "--log", log], os.path.join(dest, "setups", name + ".txt")))
    with concurrent.futures.ThreadPoolExecutor(WORKERS) as pool:
        list(pool.map(lambda job: run(*job), jobs))


def printout(path):
    """A run's printout, without the line naming its log file (in a different folder each time)."""
    return [line for line in open(path).read().splitlines() if not line.startswith("log written to")]


def compare(base, quick=False):
    new = base.rstrip("/") + "_now"
    snapshot(new, quick)
    names = ["xml.txt", "stdlib.txt", "window.txt"] + ([] if quick else ["verify.txt", "fold.csv", "fold.txt"] + sorted(
        os.path.join("setups", os.path.basename(p)) for p in glob.glob(os.path.join(base, "setups", "*"))))
    bad = []
    for name in names:
        a, b = os.path.join(base, name), os.path.join(new, name)
        if not os.path.exists(b):
            bad.append(f"{name}: missing now")
        elif name.endswith(".txt") and name not in ("xml.txt", "window.txt"):
            if printout(a) != printout(b):
                bad.append(f"{name}: printout differs")
        elif not filecmp.cmp(a, b, shallow=False):
            bad.append(f"{name}: differs")
    for line in bad:
        print("  DIFFERS  " + line)
    print(f"{len(names) - len(bad)} of {len(names)} the same as {base}" + (" (quick)" if quick else ""))
    print(f"(this run is in {new}: diff the files there against the saved ones)" if bad else "SAME AS BEFORE")
    return not bad


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("save", "compare"):
        raise SystemExit(__doc__)
    quick = "--quick" in sys.argv
    if sys.argv[1] == "save":
        snapshot(os.path.abspath(sys.argv[2]), quick)
        print(f"saved to {sys.argv[2]}")
    else:
        raise SystemExit(0 if compare(os.path.abspath(sys.argv[2]), quick) else 1)
