"""Generate chain.xml: a free-floating, zero-gravity MuJoCo model of the self-folding chain.

Usage:  .venv/bin/python make_chain.py [--preset original14|dyna7|feetech7] [-n 12] [--slip-ring] [--gravity 0] [-o chain.xml]

The chain floats horizontally (along world +Y) above a floor, high enough that it can never reach the floor while
folding in zero gravity. Turn gravity on (--gravity 9.81, or the viewer's Physics panel: 0 0 -9.81) to drop it.

Kinematics (measured from 'complete chain.3dm', see ../hardware/AGENTS.md). Per module i:
  m{i}      base: shell + bracket + servos. Module 1 has the free joint.
  m{i}_arm  U-yoke + hub, joint m{i}_pitch about +X through the module centre.
  m{i+1}    next base, one pitch above, joint m{i+1}_yaw about +Z (its yaw horn is bolted to the hub).
Module 1 has no yaw servo and the last module has no pitch servo or arm, so N modules have 2N-2 joints.
End effectors (optional, one module each): ee1 one pitch before m1, on module 1's yaw servo m1_yaw (2N-1 joints), and
ee{N} one pitch past module N, rigid. Each is none, anchor, suction or thruster.
"""
import argparse
import math
import os
import tomllib
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC_FILE = os.path.join(os.path.dirname(HERE), "hardware", "spec.toml")
with open(SPEC_FILE, "rb") as fh:
    SPEC = tomllib.load(fh)  # the chain as built; hardware owns it, and the robot software reads it too
HW, SERVO, MASS, FLIGHT = SPEC["chain"], SPEC["servo"], SPEC["mass"], SPEC["flight"]
WEIGHED = [g / 1000 for g in MASS.get("module_g", [])]  # whole modules (base + arm, kg), once they've been weighed

# original14 is the real chain: its servo, pitch and masses come from the spec. The 7 cm presets are what-ifs.
# Lengths of the meshes and primitives are for the 140 mm AX-12A module; other pitches scale them.
PRESETS = {
    "original14": dict(servo=SERVO["model"], torque=SERVO["stall_torque_Nm"], speed=SERVO["no_load_speed_rad_s"],
                       pitch=HW["pitch_mm"] / 1000, timestep=0.0005, base_mass=MASS["base_g"] / 1000,
                       end_mass=MASS["end_g"] / 1000, arm_mass=MASS["arm_g"] / 1000, module_masses=WEIGHED,
                       shell_frac=0.33, armature=0.002, kv=1.0),
    "dyna7": dict(servo="XL330-M288", torque=0.52, speed=10.8, pitch=0.070, timestep=0.0001,
                  base_mass=0.047, end_mass=0.047, arm_mass=0.004, shell_frac=0.18, armature=0.0002, kv=0.25),
    "feetech7": dict(servo="STS3032", torque=0.44, speed=11.6, pitch=0.070, timestep=0.0001,
                     base_mass=0.051, end_mass=0.051, arm_mass=0.004, shell_frac=0.17, armature=0.0002, kv=0.25),
}
# torque: stall torque (N·m); speed: no-load speed (rad/s); pitch: module centre-to-centre (m).
# base_mass: middle module without its arm (kg); end_mass: modules 1 and N (one servo fewer, plus U2D2 + power hub).
# shell_frac: share of the base mass in the two shell halves (hollow sphere); the rest sits on the core box.
# armature: the motor's inertia as felt at the joint through the gearbox (kg m^2, estimate; AX-12A gearing is 254:1).
# kv: servo braking as the joint nears its target (N m s/rad). Without it the chain rings like a spring after a move;
#   these values make a 60 deg step settle in about 0.5 s with ~1 deg overshoot. Raise kv for stiffer, slower stops.
# timestep: RK4 keeps the free-floating COM still (<0.01 mm in 12 s of random motion; implicitfast drifts ~100 mm),
#   but it is only stable for timestep < ~2.8 * (I_twist + armature) / (4 * (damping + kv)), I_twist = one base's
#   inertia about the chain axis. Lighter modules or more kv need a smaller timestep; run verify.py after changes.

PITCH_RANGE = SPEC["joints"]["pitch_range_deg"]  # deg, from the spec: the hub hits the end of the shell slot
YAW_RANGE = SPEC["joints"]["yaw_range_deg"]      # deg, from the spec: servo range; --slip-ring removes the joint limit
SLIP_RING_TARGETS = 360 # deg, yaw target range with --slip-ring (sets the slider range; the joint itself is free)
KP_SATURATE = 10        # deg of position error at which the actuator reaches stall torque
SERVO_LAG = 0.005       # s, first-order lag on servo targets; smooths 10 ms command steps so RK4 stays accurate
GEAR_FRICTION = 0.07    # Coulomb friction in every joint, as a fraction of stall torque (plastic gearbox, estimate)
CHAIN_ROLL = 0          # deg about the chain's long axis; 0 = pitch axes vertical, so pitch folds lie flat on the floor
ROLLING_RESISTANCE = 0.03  # floor rolling-resistance coefficient (torque = this x shell radius x normal force).
                           # Chosen so a dropped chain settles in ~10 s; below ~0.01 the off-centre modules keep rocking.

R_SHELL = HW["shell_diameter_mm"] / 2000             # shell outer radius (m), from the spec
N_MODULES = HW["modules"]                            # default chain length, from the spec
CORE_BOX = ((0, -0.00975, -0.00575), (0.019, 0.02875, 0.04425))  # bracket + servos: centre, half-size
ARM_BOX = ((0, 0, 0.038), (0.032, 0.012, 0.050))                 # yoke + hub: centre, half-size
BASE_RGBA, ARM_RGBA = "0.78 0.78 0.80 1", "0.20 0.45 0.95 1"
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
CUP_MARGIN = 0.001      # m: a cup's contact margin: it pulls (and pushes) from this far away. Bigger makes it hover
                        # (MuJoCo 3.16 doesn't let gap switch the push off), so a cup has to be pressed on to stick
FACINGS = {"up": (0, 0, 1), "down": (0, 0, -1), "+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0)}
PLATE = (0.5, 0.005)    # m: half-size and half-thickness of the plate an end is attached to when it isn't the floor
ROD_MM = 35             # default rod diameter (mm), about a handrail


def v(*xs):
    return " ".join(f"{x:.6g}" for x in xs)


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


def build(preset="original14", n=N_MODULES, slip_ring=False, gravity=0.0, air=False, cage=False, ee1="none", ee_n="none",
          rod=None):
    """air: cabin air drag on the shells. cage: the net cage around the chain's start position (sizes in the spec).
    ee1, ee_n: the end effector past m1 and past module N (ee12 on 12 modules): a type from EE_TYPES, or a dict like
    EE_DEFAULTS. rod: {"from": [x, y, z], "to": [x, y, z], "diameter_mm": 35} for a fixed rod, or None."""
    ends = [{**EE_DEFAULTS, **({"type": e} if isinstance(e, str) else e)} for e in (ee1, ee_n)]
    for e in ends:
        if e["type"] not in EE_TYPES:
            raise ValueError(f"end effector {e['type']!r}: choose one of {', '.join(EE_TYPES)}")
    lead = leading_end(ends)
    spot = attach_point(ends[lead]["attach"], rod) if lead is not None else None
    p = PRESETS[preset]
    s = p["pitch"] / 0.140                       # geometric scale
    kp = p["torque"] / math.radians(KP_SATURATE)
    damping = p["torque"] / p["speed"]           # servo slows to its no-load speed at stall torque
    span = n + sum(e["type"] != "none" for e in ends)  # modules end to end, counting end effectors
    height = span * p["pitch"] / 2 + 2 * R_SHELL * s  # float height: more than half the chain plus a shell
    roll = math.radians(CHAIN_ROLL)
    weighed = p.get("module_masses") or []
    if weighed and len(weighed) != n:
        print(f"note: spec.toml lists {len(weighed)} module weights for a {n}-module chain; using base_g / end_g instead")
        weighed = []

    root = ET.Element("mujoco", model=f"chain_{preset}_{n}")
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes", autolimits="true")
    # MuJoCo's ellipsoid fluid model on each shell. For a spinning chain it gives about twice the spin loss of a simple
    # hand estimate (quadratic drag, Cd ~0.45), so read it as an upper bound.
    fluid = dict(density=v(FLIGHT["cabin_air_kg_m3"]), viscosity=v(1.8e-5)) if air else {}
    ET.SubElement(root, "option", gravity=v(0, 0, -gravity), timestep=v(p["timestep"]), integrator="RK4", **fluid)
    center = v(0, 0, height) if not spot else v(*(spot[0][k] + spot[1][k] * span * p["pitch"] / 2 for k in range(3)))
    ET.SubElement(root, "statistic", center=center, extent=v(span * p["pitch"]))  # viewer frames the whole chain
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth="1600", offheight="1200", azimuth="0", elevation="-25")  # viewer starts side-on
    ET.SubElement(vis, "headlight", ambient="0.4 0.4 0.4", diffuse="0.6 0.6 0.6")

    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping=v(damping), armature=v(p["armature"]), frictionloss=v(GEAR_FRICTION * p["torque"]))
    # the motor also has to overcome the gear friction, so the net torque at the output at stall is the rated stall torque
    reach = p["torque"] * (1 + GEAR_FRICTION)
    ET.SubElement(default, "position", kp=v(kp), kv=v(p["kv"]), forcerange=v(-reach, reach), timeconst=v(SERVO_LAG))
    in_air = dict(fluidshape="ellipsoid") if air else {}                        # only the shells meet the air
    sheltered = dict(fluidshape="ellipsoid", fluidcoef="0 0 0 0 0") if air else {}  # boxes inside the shells
    for cls, attrs in {"visual": dict(type="mesh", contype="0", conaffinity="0", group="2", mass="0"),
                       "collision": dict(type="sphere", size=v(R_SHELL * s), group="3", shellinertia="true", rgba="1 0.5 0 0.3",
                                         **in_air),
                       "mass": dict(type="box", contype="0", conaffinity="0", group="4", rgba="1 0 0 0.3", **sheltered)}.items():
        ET.SubElement(ET.SubElement(default, "default", {"class": cls}), "geom", attrs)

    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1="0.25 0.26 0.28", rgb2="0.32 0.33 0.35",
                  width="512", height="512")
    ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="5 5", texuniform="true", reflectance="0.1")
    for name in ("base_first", "base_mid", "base_last", "arm"):
        ET.SubElement(asset, "mesh", name=name, file=f"{name}.stl", scale=v(s, s, s), inertia="shell")

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", directional="true", pos="0 0 3", dir="0 0 -1", diffuse="0.5 0.5 0.5")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", material="grid", condim="6",  # 20 cm squares
                  friction=v(1, 0.005, ROLLING_RESISTANCE * R_SHELL * s))
    if cage:  # six thin walls around the chain's start position; a net gives more than a wall, hence the soft solref
        half, t = [x / 2000 for x in (FLIGHT["cage_mm"][1], FLIGHT["cage_mm"][0], FLIGHT["cage_mm"][2])], 0.005
        for k in range(3):
            for sign in (-1, 1):
                pos, size = [0, 0, height], list(half)
                pos[k] += sign * (half[k] + t)
                size[k] = t
                ET.SubElement(world, "geom", name=f"cage{k}{'-+'[sign > 0]}", type="box", pos=v(*pos), size=v(*size),
                              rgba="0.5 0.6 0.7 0.12", solref="0.05 1")
    if spot and "at" in ends[lead]["attach"] and not (ends[lead]["attach"]["facing"] == "up" and spot[0][2] == 0):
        point, normal = spot  # a wall, ceiling or table to be attached to (the floor is already there)
        ET.SubElement(world, "geom", name="plate", type="box", size=v(PLATE[0], PLATE[0], PLATE[1]), zaxis=v(*normal),
                      pos=v(*(point[k] - normal[k] * PLATE[1] for k in range(3))), rgba="0.55 0.60 0.65 1")
    if rod:
        ET.SubElement(world, "geom", name="rod", type="capsule", fromto=v(*rod["from"], *rod["to"]),
                      size=v(rod["diameter_mm"] / 2000), rgba="0.72 0.72 0.76 1")
    imu = min(SPEC["imu"]["module"], n)
    parent = world
    yaw_limits = {} if slip_ring else dict(range=v(-math.radians(YAW_RANGE), math.radians(YAW_RANGE)))
    for i in range(1, n + 1):
        if weighed:  # a weighed module includes its arm (module N has none)
            base_mass = weighed[i - 1] - (p["arm_mass"] if i < n else 0)
        else:
            base_mass = p["end_mass"] if i in (1, n) else p["base_mass"]
        if i == 1:  # module frame: x = pitch axis, z = along the chain -> world +Y, rolled by CHAIN_ROLL
            body = ET.SubElement(parent, "body", name="m1", pos=v(0, -(n - 1) * p["pitch"] / 2, height),
                                 xyaxes=v(math.sin(roll), 0, math.cos(roll), math.cos(roll), 0, -math.sin(roll)))
            ET.SubElement(body, "freejoint", name="root")
            first = body
        else:
            body = ET.SubElement(parent, "body", name=f"m{i}", pos=v(0, 0, p["pitch"]))
            ET.SubElement(body, "joint", dict(name=f"m{i}_yaw", axis="0 0 1", **yaw_limits))
        mesh = "base_first" if i == 1 else "base_last" if i == n else "base_mid"
        ET.SubElement(body, "geom", {"class": "visual", "mesh": mesh, "rgba": BASE_RGBA, "name": f"m{i}_visual"})
        ET.SubElement(body, "geom", {"class": "collision", "name": f"m{i}_sphere", "mass": v(base_mass * p["shell_frac"])})
        ET.SubElement(body, "geom", {"class": "mass", "pos": v(*(c * s for c in CORE_BOX[0])),
                                     "size": v(*(h * s for h in CORE_BOX[1])), "mass": v(base_mass * (1 - p["shell_frac"]))})
        if i == imu:
            ET.SubElement(body, "site", name="imu", size=v(0.01 * s))  # at the module centre, axes = the module frame
        if i == n:
            break
        arm = ET.SubElement(body, "body", name=f"m{i}_arm")
        r = math.radians(PITCH_RANGE)
        ET.SubElement(arm, "joint", name=f"m{i}_pitch", axis="1 0 0", range=v(-r, r))
        ET.SubElement(arm, "geom", {"class": "visual", "mesh": "arm", "rgba": ARM_RGBA, "name": f"m{i}_arm_visual"})
        ET.SubElement(arm, "geom", {"class": "mass", "pos": v(*(c * s for c in ARM_BOX[0])),
                                    "size": v(*(h * s for h in ARM_BOX[1])), "mass": v(p["arm_mass"])})
        parent = arm

    # end effectors go in after the chain, so the chain's body ids don't move (m1 stays body 1)
    ee_actuators = []  # (tag, attributes): after the joint servos
    for name, end, side, e in (("ee1", first, -1, ends[0]), (f"ee{n}", body, 1, ends[1])):
        kind = e["type"]
        if kind == "none":
            continue
        ee = ET.SubElement(end, "body", name=name, pos=v(0, 0, side * p["pitch"]))
        if name == "ee1":  # module 1's missing yaw. About -Z so that +angle turns the m12 side +, like every other yaw
            ET.SubElement(ee, "joint", dict(name="m1_yaw", axis="0 0 -1", **yaw_limits))
        mass = (EE_MASS[kind] + (EE_SERVO_MASS if name == "ee1" else 0)) * s ** 3
        ET.SubElement(ee, "geom", {"class": "visual", "type": "sphere", "size": v(R_SHELL * s), "rgba": EE_RGBA[kind],
                                   "name": f"{name}_visual"})
        ET.SubElement(ee, "geom", {"class": "collision", "name": f"{name}_sphere", "mass": v(mass), "shellinertia": "false"})
        if kind == "anchor":  # its grip face; play.py clamps when the face is near something fixed and it's told to hold
            ET.SubElement(ee, "geom", {"class": "visual", "type": "cylinder", "name": f"{name}_tip", "rgba": "0.15 0.15 0.15 1",
                                       "size": v(TIP[0] * s, TIP[1] / 2 * s), "pos": v(0, 0, side * (R_SHELL - TIP[1] / 2) * s)})
            # the clamp: a weld to a fixed (mocap) body that play.py moves to the end's pose when it grabs
            ET.SubElement(world, "body", name=f"{name}_hold", mocap="true")
            weld = dict(name=f"{name}_clamp", body1=name, body2=f"{name}_hold", relpose="0 0 0 1 0 0 0", active="false",
                        solref=CLAMP_SOLREF)
            if root.find("equality") is None:
                ET.SubElement(root, "equality")
            ET.SubElement(root.find("equality"), "weld", weld)
        if kind == "suction":  # the cup on its own body, so the adhesion (which acts on all of a body's contacts) only
            # pulls where the cup touches. It collides with nothing by itself: the contact pairs below, fixed things only
            cup = ET.SubElement(ee, "body", name=f"{name}_cup", pos=v(0, 0, side * (R_SHELL * s - CUP_T)))
            ET.SubElement(cup, "geom", name=f"{name}_cup", type="cylinder", size=v(e["cup_mm"] / 2000, CUP_T), mass="0",
                          contype="0", conaffinity="0", rgba="0.10 0.35 0.15 1")
            ee_actuators.append(("adhesion", dict(name=f"{name}_suck", body=f"{name}_cup", ctrlrange="0 1",
                                                  gain=v(e["pull_N"]), forcelimited="false")))

    # actuators in CSV order: m1_yaw, m1_pitch, ..., mN_yaw, mN_pitch (skipping mN_pitch, and m1_yaw without ee1)
    act = ET.SubElement(root, "actuator")
    for i in range(1, n + 1):
        for j, lim in ((f"m{i}_yaw", SLIP_RING_TARGETS if slip_ring else YAW_RANGE), (f"m{i}_pitch", PITCH_RANGE)):
            if j != f"m{n}_pitch" and (j != "m1_yaw" or ends[0]["type"] != "none"):
                ET.SubElement(act, "position", name=j, joint=j, ctrlrange=v(-math.radians(lim), math.radians(lim)))
    for tag, attrs in ee_actuators:
        ET.SubElement(act, tag, attrs)

    contact = ET.SubElement(root, "contact")   # neighbours stay one pitch apart, never touch: skip them
    for i in range(1, n):
        ET.SubElement(contact, "exclude", body1=f"m{i}", body2=f"m{i + 1}")
    for name, e in (("ee1", ends[0]), (f"ee{n}", ends[1])):
        if e["type"] == "suction":  # a cup touches only fixed things: the floor, plates, the rod, the cage
            for g in world.findall("geom"):
                ET.SubElement(contact, "pair", geom1=f"{name}_cup", geom2=g.get("name"), condim="3", margin=v(CUP_MARGIN),
                              friction=v(e["friction"], e["friction"], 0.005, 0.0001, 0.0001))

    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "subtreecom", name="com", body="m1")
    ET.SubElement(sensor, "subtreelinvel", name="com_vel", body="m1")
    ET.SubElement(sensor, "subtreeangmom", name="angmom", body="m1")
    ET.SubElement(sensor, "framequat", name="imu_quat", objtype="site", objname="imu")  # what the flight IMU will report:
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu")                          # attitude, and rate in its own frame

    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", default="original14", choices=PRESETS)
    ap.add_argument("-n", "--modules", type=int, default=N_MODULES)
    ap.add_argument("--slip-ring", action="store_true", help="remove the yaw joint limits")
    ap.add_argument("--gravity", type=float, default=0.0, help="m/s^2 downward, towards the floor (Earth: 9.81)")
    ap.add_argument("--air", action="store_true", help="cabin air drag on the shells")
    ap.add_argument("--cage", action="store_true", help="the net cage around the chain (size in hardware/spec.toml)")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "chain.xml"))
    a = ap.parse_args()
    assert a.modules >= 2, "need at least 2 modules"
    with open(a.out, "w") as f:
        f.write(build(a.preset, a.modules, a.slip_ring, a.gravity, a.air, a.cage) + "\n")
    p = PRESETS[a.preset]
    print(f"wrote {a.out}: {a.modules} x {a.preset} ({p['servo']}, {p['torque']} N·m, {p['pitch'] * 100:g} cm pitch)"
          f"{', slip ring' if a.slip_ring else ''}{f', gravity {a.gravity:g} m/s^2' if a.gravity else ''}")
