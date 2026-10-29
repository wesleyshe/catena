"""Setup files: how a run is set up (everything the sequence CSV doesn't say), so it can be saved, repeated and shared.

A setup is a TOML file in setups/, the same format as ../hardware/spec.toml. Its first line, a # comment, says what the
run shows; the control window displays it. Every key is optional and defaults to DEFAULTS:

  # Turns about 30° with no push.
  sequence = "reorient_loop.csv"   # in ../sequences/, or a full path
  move_s = 1.0                     # seconds to move between rows, and to hold each row
  hold_s = 0.25
  speed = 1.0                      # play the whole sequence this many times faster
  gravity_m_s2 = 0.0
  spin_deg_s = [0.0, 0.0, 0.5]     # spin about world x, y, z when the sequence starts: the leftover spin at release
  air = false                      # cabin air drag, the net cage (numbers in ../hardware/spec.toml)
  cage = false
  preset = "original14"            # make_chain.PRESETS
  modules = 12
  slip_ring = false

  [ee1]                            # the end effector past m1; [ee12] is the one past m12 ([ee{modules}] in general)
  type = "anchor"                  # none, anchor, suction or thruster; a missing table means none

play.py --setup FILE runs one; the control window's Setup box loads and saves them. Stdlib only, like panel.py.
"""
import json
import os
import tomllib

from make_chain import EE_TYPES, N_MODULES, PRESETS

HERE = os.path.dirname(os.path.abspath(__file__))
SETUP_DIR = os.path.join(HERE, "setups")
SEQ_DIR = os.path.join(os.path.dirname(HERE), "sequences")  # at the repo root, shared with the robot software
DEFAULTS = dict(sequence="", move_s=3.0, hold_s=1.0, speed=1.0, gravity_m_s2=0.0, spin_deg_s=[0.0, 0.0, 0.0],
                air=False, cage=False, preset="original14", modules=N_MODULES, slip_ring=False, ee1="none", ee_n="none")
ENDS = ("ee1", "ee_n")  # in a setup dict; in the file they're the tables [ee1] and [ee{modules}]


def fits(val, default):
    """Is a value from the file the same kind as its default? (A whole number is fine where a decimal is expected.)"""
    number = lambda x: isinstance(x, (int, float)) and not isinstance(x, bool)
    if isinstance(default, bool):
        return isinstance(val, bool)
    if isinstance(default, list):
        return isinstance(val, list) and len(val) == 3 and all(map(number, val))
    if isinstance(default, float):
        return number(val)
    return isinstance(val, type(default)) and not isinstance(val, bool)


def load(path):
    """The setup in `path` as a flat dict with DEFAULTS' keys. ee1 and ee_n are end-effector types; sequence is a full
    path, or "" for none. Raises ValueError, naming the file, for anything it doesn't understand."""
    name = os.path.basename(path)
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    setup, n = dict(DEFAULTS), raw.get("modules", N_MODULES)
    for key, val in raw.items():
        if key in ("ee1", f"ee{n}") and isinstance(val, dict):
            if set(val) - {"type"}:
                raise ValueError(f"{name}: [{key}] has unknown keys {', '.join(sorted(set(val) - {'type'}))}")
            if val.get("type", "none") not in EE_TYPES:
                raise ValueError(f"{name}: [{key}] type {val.get('type')!r}: choose one of {', '.join(EE_TYPES)}")
            setup["ee1" if key == "ee1" else "ee_n"] = val.get("type", "none")
        elif key.startswith("ee") and key[2:].isdigit():
            raise ValueError(f"{name}: [{key}]: the ends of a {n}-module chain are [ee1] and [ee{n}]")
        elif key in DEFAULTS and key not in ENDS:
            default = DEFAULTS[key]
            if not fits(val, default):
                raise ValueError(f"{name}: {key} = {val!r}, expected something like {default!r}")
            setup[key] = [float(x) for x in val] if isinstance(default, list) else type(default)(val)
        else:
            keys = ", ".join(k for k in DEFAULTS if k not in ENDS)
            raise ValueError(f"{name}: unknown key {key!r}. Setup keys: {keys}, and tables [ee1] and [ee{n}]")
    for key, ok, rule in (("move_s", setup["move_s"] > 0, "more than 0"), ("hold_s", setup["hold_s"] >= 0, "0 or more"),
                          ("speed", setup["speed"] > 0, "more than 0")):
        if not ok:
            raise ValueError(f"{name}: {key} = {setup[key]:g}, it must be {rule} for the sequence to play")
    if setup["preset"] not in PRESETS:
        raise ValueError(f"{name}: preset {setup['preset']!r}: choose one of {', '.join(PRESETS)}")
    if setup["modules"] < 2:
        raise ValueError(f"{name}: modules = {setup['modules']}, need at least 2")
    if setup["sequence"]:
        setup["sequence"] = os.path.normpath(os.path.join(SEQ_DIR, os.path.expanduser(setup["sequence"])))
        if not os.path.exists(setup["sequence"]):
            raise ValueError(f"{name}: the sequence {setup['sequence']} doesn't exist")
    return setup


def about(path):
    """What the setup shows: its first line, if that's a # comment."""
    with open(path, encoding="utf-8") as f:
        line = f.readline().strip()
    return line.lstrip("#").strip() if line.startswith("#") else ""


def save(path, setup, about=""):
    """Write `setup` (a dict like load's) as TOML, by hand because tomllib only reads. An end that's none gets no table."""
    def toml(x):
        if isinstance(x, bool):
            return "true" if x else "false"
        if isinstance(x, list):
            return "[" + ", ".join(map(toml, x)) + "]"
        return json.dumps(x) if isinstance(x, str) else repr(x)

    seq = setup["sequence"]
    if seq and os.path.dirname(os.path.abspath(seq)) == SEQ_DIR:
        seq = os.path.basename(seq)
    lines = [f"# {' '.join(about.split())}"] if about.strip() else []
    lines += [f"{k} = {toml(seq if k == 'sequence' else setup[k])}" for k in DEFAULTS if k not in ENDS]
    for key, table in zip(ENDS, ("ee1", f"ee{setup['modules']}")):
        if setup[key] != "none":
            lines += ["", f"[{table}]", f"type = {toml(setup[key])}"]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def single_axis_spin(spin):
    """A setup's spin [x, y, z] (deg/s) as (rate, axis letter, warning), for the control window's one-axis Spin box.
    It keeps the largest component; the warning names any others it had to drop."""
    k = max(range(3), key=lambda i: abs(spin[i]))
    dropped = [f"{'xyz'[i]} {spin[i]:g}" for i in range(3) if i != k and spin[i]]
    warning = f"the control window spins about one axis: kept {'xyz'[k]}, dropped {', '.join(dropped)} deg/s" if dropped else ""
    return (float(spin[k]), "xyz"[k], warning) if spin[k] else (0.0, "z", "")


def build_args(setup):
    """make_chain.build()'s keyword arguments for this setup."""
    return dict(preset=setup["preset"], n=setup["modules"], slip_ring=setup["slip_ring"], gravity=setup["gravity_m_s2"],
                air=setup["air"], cage=setup["cage"], ee1=setup["ee1"], ee_n=setup["ee_n"])
