"""Setup files: how a run is set up (everything the sequence CSV doesn't say), so it can be saved, repeated and shared.

A setup is a TOML file in setups/, the same format as ../hardware/spec.toml. Its first line, a # comment, says what the
run shows; the control window displays it. The sequence CSV is only the moves: the world (gravity, a rod or pole), the
end effectors and where everything starts are here. Every key is optional and defaults to DEFAULTS, and save() writes
only sequence, move_s, hold_s and what differs from DEFAULTS, so a setup file shows what's special about its run.
All the keys:

  # Reaches out from a wall and holds.
  sequence = "arm_reach.csv"       # in ../sequences/, or a path relative to it, or a full path
  move_s = 3.0                     # seconds to move between rows, and to hold each row
  hold_s = 1.0
  speed = 1.0                      # play the whole sequence this many times faster (faster motion: different physics)
  playback = 1.0                   # the viewer shows it this many times faster than real time (same physics)
  gravity_m_s2 = 0.0
  spin_deg_s = [0.0, 0.0, 0.5]     # spin about world x, y, z when the sequence starts: the leftover spin at release
  air = false                      # cabin air drag (the air density is in ../hardware/spec.toml)
  preset = "original14"            # make_chain.PRESETS
  modules = 12
  slip_ring = false

  [ee1]                            # the end effector past m1; [ee12] is the one past m12 ([ee{modules}] in general)
  type = "anchor"                  # none, anchor, suction or thruster; a missing table means none
  attach = { at = [0.0, -0.9, 1.8], facing = "+y", roll_deg = 0.0 }   # anchor, suction: where it starts attached
  # attach = { rod_m = 0.3, facing = "up", roll_deg = 0.0 }           # ... or on the rod, this far along it
  # pull_N = 20.0, cup_mm = 40.0, friction = 0.8                      # suction cup only
  # thrust_N = 0.1                                                    # thruster only: per nozzle

  [rod]                            # a fixed rod (a handrail) that anchors can clamp onto; a missing table means none
  from = [0.0, -1.2, 1.0]
  to = [0.0, 1.2, 1.0]
  diameter_mm = 35.0
  # wrap_m = 0.4                   # the bare chain starts wrapped around the rod, m1 this far along it (the
                                   # sequence's first row must be a coil that fits: every pitch the same, every yaw)

  [flight]                         # the box on the parabolic flight; a missing table means none. Every key optional:
  box_m = [2.5, 1.3, 1.9]          # inside: along the chain (the cabin), across, height (default: the spec's box_mm)
  strings = [1, 5, 8, 12]          # modules held still in the middle, each by a string up and one down (default:
                                   # both ends and two in between; [] = none, it floats free in the box)
  on_net = false                   # true: it starts lying on the net instead, no strings, and floats off at zero g
  roll_deg = 0.0                   # the chain rolled about its length: 0 = pitch axes vertical, 90 = across the cabin,
                                   # along the plane's pitch axis (so the plane's spin is about the folds' turn axis)
  level_g = 0.0                    # the parabola's g: 0, a Moon (0.16) or Mars (0.38) parabola, 1 (level flight)
  release_s = 1.0                  # Play (and a headless run) flies the parabola: the strings let go this long after
                                   # zero g begins
  start_s = 1.5                    # ... and the sequence starts this long after
  hyper_s = 23.0                   # each 1.8 g phase, ramps included (the spec's hypergravity_s); at least 7
  jitter = true                    # the leftover g in zero g (the spec's jitter_g), the same every take for one seed
  seed = 1

facing is the surface's outward normal (up = the floor; down, +x, -x, +y, -y), and the chain leaves the surface along
it. On a rod it's the side of the rod the chain is on. If both ends are attached, ee1 sets where the chain starts.

play.py --setup FILE runs one; the control window's Setup box loads and saves them, and its demo buttons (DEMOS) play
one each. Stdlib only, like panel.py.
"""
import copy
import glob
import json
import math
import numbers
import os
import tomllib

from make_chain import N_MODULES, PRESETS, ROD_MM
from make_ends import EE_DEFAULTS, EE_TYPES, FACINGS
from spec import FLIGHT, REPO, SEQ_DIR, SETUP_DIR  # sequences are at the repo root, shared with the robot software

DEFAULTS = dict(sequence="", move_s=3.0, hold_s=1.0, speed=1.0, playback=1.0, gravity_m_s2=0.0, spin_deg_s=[0.0, 0.0, 0.0],
                air=False, preset="original14", modules=N_MODULES, slip_ring=False,
                ee1=dict(EE_DEFAULTS), ee_n=dict(EE_DEFAULTS), rod={}, flight={})
ENDS = ("ee1", "ee_n")  # in a setup dict; in the file they're the tables [ee1] and [ee{modules}]
TABLES = ENDS + ("rod", "flight")
FLIGHT_DEFAULTS = dict(box_m=[x / 1000 for x in FLIGHT["box_mm"]], strings=None, on_net=False, roll_deg=0.0, level_g=0.0, release_s=1.0,
                       start_s=1.5, hyper_s=float(FLIGHT["hypergravity_s"]), jitter=True, seed=1)  # strings None: default_strings
RAMPS_S = 7.0  # the 1 -> 1.8 g and 1.8 -> 0 g ramps (flight.Parabola): a 1.8 g phase can't be shorter
SETTINGS = dict(none=(), anchor=("attach",), suction=("attach", "pull_N", "cup_mm", "friction"), thruster=("thrust_N",))
ALWAYS = ("sequence", "move_s", "hold_s")  # save() writes these; the other keys only where they differ from DEFAULTS
DEMOS = [("1A  Free float (held)", "1a_turn"), ("1B  Free float (from the net)", "1b_from_net"),  # the control window's
         ("2  Anchored", "2a_arm"), ("3  Thrusters", "3a_U"), ("4  Suction walk", "4a_walk"),  # buttons: label, setup
         ("5  Pole climb", "5a_pole_climb"), ("6  Skater", "6a_skater")]
KINDS = dict(anchor="anchor (clamp)", suction="suction cup", thruster="thruster")


def new(**changes):
    """A fresh setup dict: DEFAULTS with these changes."""
    return {**copy.deepcopy(DEFAULTS), **copy.deepcopy(changes)}


def end(kind="none", **settings):
    """An end-effector entry for a setup dict, e.g. end("anchor", attach={"at": [0, 0, 0], "facing": "up"})."""
    return {**copy.deepcopy(EE_DEFAULTS), "type": kind, **copy.deepcopy(settings)}


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


def value(name, key, val, default):
    if not fits(val, default):
        raise ValueError(f"{name}: {key} = {val!r}, expected something like {default!r}")
    return [float(x) for x in val] if isinstance(default, list) else type(default)(val)


def read_attach(name, table, val):
    """{} (floating), {at, facing, roll_deg} or {rod_m, facing, roll_deg}."""
    if not isinstance(val, dict) or len({"at", "rod_m"} & set(val)) != 1 or set(val) - {"at", "rod_m", "facing", "roll_deg"}:
        raise ValueError(f"{name}: [{table}] attach = {val!r}: write {{ at = [x, y, z], facing = \"up\", roll_deg = 0 }} "
                         "or { rod_m = 0.3, facing = \"up\", roll_deg = 0 }")
    out = {"at": value(name, "attach at", val["at"], [0.0] * 3)} if "at" in val else \
          {"rod_m": value(name, "attach rod_m", val["rod_m"], 0.0)}
    out["facing"] = val.get("facing", "up")
    if out["facing"] not in FACINGS:
        raise ValueError(f"{name}: [{table}] facing {out['facing']!r}: choose one of {', '.join(FACINGS)}")
    out["roll_deg"] = value(name, "attach roll_deg", val.get("roll_deg", 0.0), 0.0)
    return out


def read_end(name, table, val):
    if not isinstance(val, dict):
        raise ValueError(f"{name}: {table} = {val!r}: an end effector is a table; write [{table}] and then type = ...")
    kind = val.get("type", "none")
    if kind not in EE_TYPES:
        raise ValueError(f"{name}: [{table}] type {kind!r}: choose one of {', '.join(EE_TYPES)}")
    extra = set(val) - {"type"} - set(SETTINGS[kind])
    if extra:
        raise ValueError(f"{name}: [{table}] a {kind} has no {', '.join(sorted(extra))}"
                         f" (its settings: {', '.join(SETTINGS[kind]) or 'none'})")
    out = end(kind)
    for key in SETTINGS[kind]:
        if key in val:
            out[key] = read_attach(name, table, val[key]) if key == "attach" else value(name, key, val[key], EE_DEFAULTS[key])
    return out


def read_rod(name, val):
    if not isinstance(val, dict) or {"from", "to"} - set(val) or set(val) - {"from", "to", "diameter_mm", "wrap_m"}:
        raise ValueError(f"{name}: [rod] needs from = [x, y, z] and to = [x, y, z], and optionally diameter_mm, wrap_m")
    rod = {"from": value(name, "rod from", val["from"], [0.0] * 3), "to": value(name, "rod to", val["to"], [0.0] * 3),
           "diameter_mm": value(name, "rod diameter_mm", val.get("diameter_mm", float(ROD_MM)), 0.0)}
    if rod["from"] == rod["to"] or rod["diameter_mm"] <= 0:
        raise ValueError(f"{name}: [rod] needs two different end points and a diameter above 0")
    if "wrap_m" in val:
        rod["wrap_m"] = value(name, "rod wrap_m", val["wrap_m"], 0.0)
        if not 0 <= rod["wrap_m"] <= math.dist(rod["from"], rod["to"]):
            raise ValueError(f"{name}: [rod] wrap_m = {rod['wrap_m']:g} is off the rod (0 to {math.dist(rod['from'], rod['to']):.2f} m)")
    return rod


def default_strings(n):
    """Which modules the strings hold if a setup doesn't say: both ends and two in between (1, 5, 8, 12 of 12)."""
    return sorted({1, n // 3 + 1, n - n // 3, n})


def read_flight(name, val, n):
    if not isinstance(val, dict) or set(val) - set(FLIGHT_DEFAULTS):
        raise ValueError(f"{name}: [flight] keys are {', '.join(FLIGHT_DEFAULTS)}")
    out = {}
    for key, default in FLIGHT_DEFAULTS.items():
        if key == "strings":
            got = val.get("strings", default_strings(n))
            if not (isinstance(got, list) and all(isinstance(i, int) and not isinstance(i, bool) and 1 <= i <= n for i in got)
                    and len(set(got)) == len(got)):
                raise ValueError(f"{name}: [flight] strings = {got!r}: a list of different module numbers, 1 to {n}")
            out[key] = sorted(got)
        else:
            out[key] = value(name, f"[flight] {key}", val.get(key, default), default)
    if out["on_net"]:
        out["strings"] = []  # lying on the net, nothing holds it
    if min(out["box_m"]) <= 0 or out["release_s"] < 0 or out["start_s"] < 0 or out["hyper_s"] < RAMPS_S or not 0 <= out["level_g"] <= 1:
        raise ValueError(f"{name}: [flight] needs box_m above 0, release_s and start_s 0 or more, hyper_s {RAMPS_S:g} or more, "
                         "level_g 0 to 1")
    return out


def load(path):
    """The setup in `path` as a dict like DEFAULTS: ee1 / ee_n / rod are dicts, and sequence is a full path or "".
    Raises ValueError, naming the file, for anything it doesn't understand."""
    name = os.path.basename(path)
    try:
        with open(path, "rb") as f:
            raw = tomllib.load(f)
    except tomllib.TOMLDecodeError as e:
        raise ValueError(f"{name}: {e}") from None
    setup, n = new(), raw.get("modules", N_MODULES)
    for key, val in raw.items():
        if key in ("ee1", f"ee{n}"):
            setup["ee1" if key == "ee1" else "ee_n"] = read_end(name, key, val)
        elif key.startswith("ee") and key[2:].isdigit():
            raise ValueError(f"{name}: [{key}]: the ends of a {n}-module chain are [ee1] and [ee{n}]")
        elif key == "rod":
            setup["rod"] = read_rod(name, val)
        elif key == "flight":
            setup["flight"] = read_flight(name, val, n)
        elif key in DEFAULTS and key not in TABLES:
            setup[key] = value(name, key, val, DEFAULTS[key])
        else:
            keys = ", ".join(k for k in DEFAULTS if k not in TABLES)
            raise ValueError(f"{name}: unknown key {key!r}. Setup keys: {keys}, and tables [ee1], [ee{n}], [rod] and [flight]")
    for key, ok, rule in (("move_s", setup["move_s"] > 0, "more than 0"), ("hold_s", setup["hold_s"] >= 0, "0 or more"),
                          ("speed", setup["speed"] > 0, "more than 0"), ("playback", setup["playback"] > 0, "more than 0")):
        if not ok:
            raise ValueError(f"{name}: {key} = {setup[key]:g}, it must be {rule} for the sequence to play")
    if setup["preset"] not in PRESETS:
        raise ValueError(f"{name}: preset {setup['preset']!r}: choose one of {', '.join(PRESETS)}")
    if setup["modules"] < 2:
        raise ValueError(f"{name}: modules = {setup['modules']}, need at least 2")
    for key in ENDS:
        attach = setup[key]["attach"]
        if "rod_m" in attach:
            if not setup["rod"]:
                raise ValueError(f"{name}: an end is attached to the rod, but there's no [rod] table")
            u = [b - a for a, b in zip(setup["rod"]["from"], setup["rod"]["to"])]
            if abs(sum(x * y for x, y in zip(u, FACINGS[attach["facing"]]))) > 1e-9 * math.hypot(*u):
                raise ValueError(f"{name}: facing {attach['facing']!r} isn't across the rod; pick a side of the rod")
    if setup["sequence"]:
        setup["sequence"] = os.path.normpath(os.path.join(SEQ_DIR, os.path.expanduser(setup["sequence"])))
        if not os.path.isfile(setup["sequence"]):
            raise ValueError(f"{name}: the sequence {setup['sequence']} isn't a file (sequences are in {SEQ_DIR})")
    return setup


def about(path):
    """What the setup shows: its first line, if that's a # comment."""
    with open(path, encoding="utf-8") as f:
        line = f.readline().strip()
    return line.lstrip("#").strip() if line.startswith("#") else ""


def toml(x):
    """One TOML value. Only the kinds a setup holds; anything else is a mistake, so it raises."""
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, numbers.Integral):
        return str(int(x))
    if isinstance(x, numbers.Real):
        return repr(float(x))
    if isinstance(x, str):
        return json.dumps(x, ensure_ascii=False)
    if isinstance(x, (list, tuple)):
        return "[" + ", ".join(map(toml, x)) + "]"
    if isinstance(x, dict):
        return "{ " + ", ".join(f"{k} = {toml(v)}" for k, v in x.items()) + " }"
    raise TypeError(f"can't write {x!r} ({type(x).__name__}) to a setup file")


def save(path, setup, about=""):
    """Write `setup` (a dict like load's) as TOML, by hand because tomllib only reads: ALWAYS, then only the keys that
    differ from DEFAULTS. An end that's none, or no rod, gets no table. A sequence inside the repo is saved relative to
    sequences/, so the file works on any computer."""
    seq = setup["sequence"]
    if seq:
        seq = os.path.abspath(seq)
        if os.path.commonpath([seq, REPO]) == REPO:
            seq = os.path.relpath(seq, SEQ_DIR)
    lines = [f"# {' '.join(about.split())}"] if about.strip() else []
    lines += [f"{k} = {toml(seq if k == 'sequence' else setup[k])}" for k in DEFAULTS
              if k not in TABLES and (k in ALWAYS or setup[k] != DEFAULTS[k])]
    for key, table in zip(ENDS, ("ee1", f"ee{setup['modules']}")):
        e = setup[key]
        if e["type"] != "none":
            lines += ["", f"[{table}]", f"type = {toml(e['type'])}"]
            lines += [f"{k} = {toml(e[k])}" for k in SETTINGS[e["type"]] if not (k == "attach" and not e[k])]
    for table in ("rod", "flight"):
        if setup[table]:
            lines += ["", f"[{table}]"] + [f"{k} = {toml(v)}" for k, v in setup[table].items()]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def summary(setup):
    """What's in a setup besides its moves, in one line: the world, the chain, the end effectors, how it starts."""
    parts = [f"gravity {setup['gravity_m_s2']:g} m/s²" if setup["gravity_m_s2"] else "zero g"]
    parts += ["air drag"] if setup["air"] else []
    if setup["flight"]:
        f = setup["flight"]
        parts.append(f"in the flight box ({' x '.join(f'{x:g}' for x in f['box_m'])} m)" +
                     (", lying on the net, no strings" if f["on_net"] else
                      f", held by strings at {', '.join(f'm{i}' for i in f['strings'])} until T+{f['release_s']:g} s"
                      if f["strings"] else ", no strings") +
                     (f", rolled {f['roll_deg']:g} deg (pitch axes across the cabin at 90)" if f["roll_deg"] else "") +
                     (f", a {f['level_g']:g} g parabola" if f["level_g"] else ""))
    if setup["rod"]:
        parts.append(f"a Ø{setup['rod']['diameter_mm']:g} mm rod" + (", starting wrapped round it" if "wrap_m" in setup["rod"] else ""))
    if (setup["preset"], setup["modules"], setup["slip_ring"]) != (DEFAULTS["preset"], DEFAULTS["modules"], False):
        parts.append(f"{setup['preset']}, {setup['modules']} modules" + (", slip ring" if setup["slip_ring"] else ""))
    for key, name in zip(ENDS, ("ee1", f"ee{setup['modules']}")):
        e = setup[key]
        if e["type"] != "none":
            a = e["attach"]
            parts.append(f"{name} {KINDS[e['type']]}" + (f" starting on a surface facing {a['facing']}" if "at" in a else
                                                         " starting on the rod" if "rod_m" in a else ""))
    if all(setup[key]["type"] == "none" for key in ENDS):
        parts.append("no end effectors")
    if any(setup["spin_deg_s"]):
        parts.append("spinning " + ", ".join(f"{r:g}°/s about {a}" for a, r in zip("xyz", setup["spin_deg_s"]) if r)
                     + " from Play")
    return "; ".join(parts)


def users(sequence):
    """The setups in setups/ that play this sequence (a full path), by name."""
    names = []
    for path in sorted(glob.glob(os.path.join(SETUP_DIR, "*.toml"))):
        try:
            if load(path)["sequence"] == os.path.normpath(sequence):
                names.append(os.path.splitext(os.path.basename(path))[0])
        except ValueError:
            pass
    return names


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
                air=setup["air"], ee1=setup["ee1"], ee_n=setup["ee_n"], rod=setup["rod"], flight=setup["flight"] or None)
