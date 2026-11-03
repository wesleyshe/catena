"""The end effectors in the model: an extra module past m1 (ee1, on module 1's twist servo m1_yaw) and/or past the last
module (ee{N}, rigid). Each is none, an anchor (a clamp), a suction cup or a thruster (five nozzles). make_chain.build()
calls bodies() and cup_pairs(); where an end starts (attach_point, leading_end) is also used by setup_file.py and
session.py. Stdlib only, like make_chain.py.
"""
import math
import xml.etree.ElementTree as ET

from spec import v

# End effectors are concepts, not hardware yet, so their placeholder numbers live here, not in spec.toml.
# The 7 cm presets scale the masses by s^3.
EE_TYPES = ("none", "anchor", "suction", "thruster")
EE_MASS = dict(anchor=0.200, suction=0.200, thruster=0.250)  # kg; the thruster also carries nozzles, valves and gas
EE_SERVO_MASS = 0.055   # kg, ee1's twist servo (an AX-12A), carried in ee1
EE_RGBA = dict(anchor="0.80 0.25 0.25 1", suction="0.25 0.70 0.35 1", thruster="0.95 0.70 0.15 1")
# Each end's settings: attach is where an anchor or cup starts attached ({} = floating, see setup_file.py); pull_N, cup_mm
# and friction are the suction cup's pull limit, diameter and friction; thrust_N is each thruster nozzle's push.
EE_DEFAULTS = dict(type="none", attach={}, pull_N=20.0, cup_mm=40.0, friction=0.8, thrust_N=0.1)
TIP = (0.020, 0.004)    # m: an anchor's grip face, a disc on its outer pole (radius, thickness)
CLAMP_SOLREF = "0.002 1"  # the clamp's weld: stiff (the default let a clamped end move 36 mm while the chain folded)
CUP_T = 0.002           # m: half-thickness of a suction cup (its diameter is a setting)
NOZZLES = ((0, 0, 1), (1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0))  # n0-n4 exhausts, in the end's frame (n0's z flips
                        # to point away from the chain at ee1); each pushes the opposite way
NOZZLE_RGBA = ("0.35 0.25 0.10 1", "1 0.55 0.1 1")  # off, firing (ends.py swaps them)
CUP_MARGIN = 0.001      # m: a cup's contact margin: it pulls (and pushes) from this far away. Bigger makes it hover
                        # (MuJoCo 3.16 doesn't let gap switch the push off), so a cup has to be pressed on to stick
FACINGS = {"up": (0, 0, 1), "down": (0, 0, -1), "+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0)}


def level_axis(normal):
    """The pitch axis an attached end starts with, before its roll: level (world x, or world y on an x-facing wall)."""
    return (0, 1, 0) if abs(normal[0]) > 0.5 else (1, 0, 0)


def attach_point(attach, rod):
    """Where an attach setting puts an end's tip: (point on the surface, outward normal). None if it floats."""
    if not attach:
        return None
    normal = FACINGS[attach["facing"]]
    if "at" in attach:
        return tuple(attach["at"]), normal
    a, b = rod["from"], rod["to"]
    length = math.dist(a, b)
    return tuple(a[k] + (b[k] - a[k]) * attach["rod_m"] / length + normal[k] * rod["diameter_mm"] / 2000
                 for k in range(3)), normal


def leading_end(ends):
    """Which end sets where the chain starts: the first anchor or suction cup with an attach setting (0 = ee1, 1 = ee_n)."""
    return next((k for k, e in enumerate(ends) if e["type"] in ("anchor", "suction") and e["attach"]), None)


def bodies(root, world, ends, first, last, n, pitch, r_shell, s, yaw_limits):
    """Add the fitted ends: ee1 one pitch before m1 (body `first`), on hinge m1_yaw, and ee{n} one pitch past the last
    module (body `last`), rigid. ends: ee1's and ee_n's settings (like EE_DEFAULTS). r_shell is the 14 cm module's shell
    radius and s the preset's scale. Returns the end effectors' actuators, (tag, attributes), for build() to write after
    the joint servos."""
    actuators = []
    for name, end, side, e in (("ee1", first, -1, ends[0]), (f"ee{n}", last, 1, ends[1])):
        kind = e["type"]
        if kind == "none":
            continue
        ee = ET.SubElement(end, "body", name=name, pos=v(0, 0, side * pitch))
        if name == "ee1":  # module 1's missing yaw. About -Z so that +angle turns the m12 side +, like every other yaw
            ET.SubElement(ee, "joint", dict(name="m1_yaw", axis="0 0 -1", **yaw_limits))
        mass = (EE_MASS[kind] + (EE_SERVO_MASS if name == "ee1" else 0)) * s ** 3
        ET.SubElement(ee, "geom", {"class": "visual", "type": "sphere", "size": v(r_shell * s), "rgba": EE_RGBA[kind],
                                   "name": f"{name}_visual"})
        ET.SubElement(ee, "geom", {"class": "collision", "name": f"{name}_sphere", "mass": v(mass), "shellinertia": "false"})
        if kind == "anchor":  # its grip face; ends.py clamps when the face is near something fixed and it's told to hold
            ET.SubElement(ee, "geom", {"class": "visual", "type": "cylinder", "name": f"{name}_tip", "rgba": "0.15 0.15 0.15 1",
                                       "size": v(TIP[0] * s, TIP[1] / 2 * s), "pos": v(0, 0, side * (r_shell - TIP[1] / 2) * s)})
            # the clamp: a weld to a fixed (mocap) body that ends.py moves to the end's pose when it grabs
            ET.SubElement(world, "body", name=f"{name}_hold", mocap="true")
            weld = dict(name=f"{name}_clamp", body1=name, body2=f"{name}_hold", relpose="0 0 0 1 0 0 0", active="false",
                        solref=CLAMP_SOLREF)
            if root.find("equality") is None:
                ET.SubElement(root, "equality")
            ET.SubElement(root.find("equality"), "weld", weld)
        if kind == "suction":  # the cup on its own body, so the adhesion (which acts on all of a body's contacts) only
            # pulls where the cup touches. It collides with nothing by itself: the contact pairs below, fixed things only
            cup = ET.SubElement(ee, "body", name=f"{name}_cup", pos=v(0, 0, side * (r_shell * s - CUP_T)))
            ET.SubElement(cup, "geom", name=f"{name}_cup", type="box", size=v(e["cup_mm"] / 2000, e["cup_mm"] / 2000, CUP_T), mass="0",
                          contype="0", conaffinity="0", rgba="0.10 0.35 0.15 1")
            actuators.append(("adhesion", dict(name=f"{name}_suck", body=f"{name}_cup", ctrlrange="0 1",
                                               gain=v(e["pull_N"]), forcelimited="false")))
        if kind == "thruster":  # five nozzles on the shell; a site actuator pushes against each one's exhaust (its z)
            for k, (x, y, z) in enumerate(NOZZLES):
                out = (x, y, z * side)
                ET.SubElement(ee, "site", name=f"{name}_n{k}", type="sphere", size=v(0.012 * s), rgba=NOZZLE_RGBA[0],
                              pos=v(*(c * r_shell * s for c in out)), zaxis=v(*out))
                actuators.append(("general", dict(name=f"{name}_n{k}", site=f"{name}_n{k}", gear="0 0 -1 0 0 0",
                                                  ctrlrange="0 1", gainprm=v(e["thrust_N"]), biastype="none",
                                                  dyntype="none", forcelimited="false")))
    return actuators


def cup_pairs(contact, ends, n, fixed):
    """A suction cup touches only fixed things (the floor, plates, the rod, the flight box): a contact pair with each of
    the `fixed` geoms."""
    for name, e in (("ee1", ends[0]), (f"ee{n}", ends[1])):
        if e["type"] == "suction":
            for g in fixed:
                ET.SubElement(contact, "pair", geom1=f"{name}_cup", geom2=g.get("name"), condim="3", margin=v(CUP_MARGIN),
                              friction=v(e["friction"], e["friction"], 0.005, 0.0001, 0.0001))
