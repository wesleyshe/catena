"""Sections 14 and 16: the countdown before a take, and the setup files (and what the control window may import)."""
import ast
import glob
import os
import sys
import tempfile

import setup_file
from flight import Parabola
from sequence import read_sequence
from viewer import Commands
from checks.common import FOLD_SEQ, HERE, check, load


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


def check_setup_files():
    print("16. setup files: what they refuse, Save as then Load, every setup, demo button and sequence; the window's imports")
    with tempfile.TemporaryDirectory() as tmp:  # a sequence for the setups below to name
        seq = os.path.join(tmp, "twist.csv")
        with open(seq, "w") as f:
            f.write(",".join(f"m{i}_{j}" for i in range(1, 13) for j in ("yaw", "pitch")) + "\n20" + ",0" * 23 + "\n")

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

    ramps = Parabola.UP_S + Parabola.DOWN_S
    check(setup_file.RAMPS_S == ramps, f"the shortest 1.8 g phase a setup may ask for ({setup_file.RAMPS_S:g} s) is the "
          f"parabola's two ramps ({Parabola.UP_S:g} + {Parabola.DOWN_S:g} s)")
    side, bad = ("panel", "setup_file", "make_chain", "make_ends", "make_box", "spec", "viewer_link"), []  # the control window and what it imports
    for name in side:
        for n in ast.walk(ast.parse(open(os.path.join(HERE, name + ".py")).read())):
            names = [a.name for a in n.names] if isinstance(n, ast.Import) else [n.module] if isinstance(n, ast.ImportFrom) else []
            bad += [f"{name}.py imports {x}" for x in names if x.split(".")[0] not in sys.stdlib_module_names | set(side)]
    check(not bad, "panel.py and the files it imports use only the stdlib (so the window runs without MuJoCo)"
          + (f": {', '.join(bad)}" if bad else ""))
