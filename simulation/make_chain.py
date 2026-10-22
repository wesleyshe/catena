"""Generate chain.xml: a free-floating, zero-gravity MuJoCo model of the self-folding chain.

Usage:  .venv/bin/python make_chain.py [--preset original14|dyna7|feetech7] [-n 12] [--slip-ring] [--gravity 0] [-o chain.xml]

The chain floats horizontally (along world +Y) above a floor, high enough that it can never reach the floor while
folding in zero gravity. Turn gravity on (--gravity 9.81, or the viewer's Physics panel: 0 0 -9.81) to drop it.

Kinematics (measured from 'complete chain.3dm', see ../hardware/AGENTS.md). Per module i:
  m{i}      base: shell + bracket + servos. Module 1 has the free joint.
  m{i}_arm  U-yoke + hub, joint m{i}_pitch about +X through the module centre.
  m{i+1}    next base, one pitch above, joint m{i+1}_yaw about +Z (its yaw horn is bolted to the hub).
Module 1 has no yaw servo and the last module has no pitch servo or arm, so N modules have 2N-2 joints.
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
HW, SERVO, MASS = SPEC["chain"], SPEC["servo"], SPEC["mass"]

# original14 is the real chain: its servo, pitch and masses come from the spec. The 7 cm presets are what-ifs.
# Lengths of the meshes and primitives are for the 140 mm AX-12A module; other pitches scale them.
PRESETS = {
    "original14": dict(servo=SERVO["model"], torque=SERVO["stall_torque_Nm"], speed=SERVO["no_load_speed_rad_s"],
                       pitch=HW["pitch_mm"] / 1000, timestep=0.0005, base_mass=MASS["base_g"] / 1000,
                       end_mass=MASS["end_g"] / 1000, arm_mass=MASS["arm_g"] / 1000, shell_frac=0.33, armature=0.002, kv=1.0),
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


def v(*xs):
    return " ".join(f"{x:.6g}" for x in xs)


def build(preset="original14", n=N_MODULES, slip_ring=False, gravity=0.0):
    p = PRESETS[preset]
    s = p["pitch"] / 0.140                       # geometric scale
    kp = p["torque"] / math.radians(KP_SATURATE)
    damping = p["torque"] / p["speed"]           # servo slows to its no-load speed at stall torque
    height = n * p["pitch"] / 2 + 2 * R_SHELL * s  # float height: more than half the chain plus a shell
    roll = math.radians(CHAIN_ROLL)

    root = ET.Element("mujoco", model=f"chain_{preset}_{n}")
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes", autolimits="true")
    ET.SubElement(root, "option", gravity=v(0, 0, -gravity), timestep=v(p["timestep"]), integrator="RK4")
    ET.SubElement(root, "statistic", center=v(0, 0, height), extent=v(n * p["pitch"]))  # viewer frames the whole chain
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth="1600", offheight="1200", azimuth="0", elevation="-25")  # viewer starts side-on
    ET.SubElement(vis, "headlight", ambient="0.4 0.4 0.4", diffuse="0.6 0.6 0.6")

    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping=v(damping), armature=v(p["armature"]), frictionloss=v(GEAR_FRICTION * p["torque"]))
    ET.SubElement(default, "position", kp=v(kp), kv=v(p["kv"]), forcerange=v(-p["torque"], p["torque"]), timeconst=v(SERVO_LAG))
    for cls, attrs in {"visual": dict(type="mesh", contype="0", conaffinity="0", group="2", mass="0"),
                       "collision": dict(type="sphere", size=v(R_SHELL * s), group="3", shellinertia="true", rgba="1 0.5 0 0.3"),
                       "mass": dict(type="box", contype="0", conaffinity="0", group="4", rgba="1 0 0 0.3")}.items():
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
    parent = world
    for i in range(1, n + 1):
        base_mass = p["end_mass"] if i in (1, n) else p["base_mass"]
        if i == 1:  # module frame: x = pitch axis, z = along the chain -> world +Y, rolled by CHAIN_ROLL
            body = ET.SubElement(parent, "body", name="m1", pos=v(0, -(n - 1) * p["pitch"] / 2, height),
                                 xyaxes=v(math.sin(roll), 0, math.cos(roll), math.cos(roll), 0, -math.sin(roll)))
            ET.SubElement(body, "freejoint", name="root")
        else:
            body = ET.SubElement(parent, "body", name=f"m{i}", pos=v(0, 0, p["pitch"]))
            yaw = dict(name=f"m{i}_yaw", axis="0 0 1")
            if not slip_ring:
                yaw["range"] = v(-math.radians(YAW_RANGE), math.radians(YAW_RANGE))
            ET.SubElement(body, "joint", yaw)
        mesh = "base_first" if i == 1 else "base_last" if i == n else "base_mid"
        ET.SubElement(body, "geom", {"class": "visual", "mesh": mesh, "rgba": BASE_RGBA, "name": f"m{i}_visual"})
        ET.SubElement(body, "geom", {"class": "collision", "name": f"m{i}_sphere", "mass": v(base_mass * p["shell_frac"])})
        ET.SubElement(body, "geom", {"class": "mass", "pos": v(*(c * s for c in CORE_BOX[0])),
                                     "size": v(*(h * s for h in CORE_BOX[1])), "mass": v(base_mass * (1 - p["shell_frac"]))})
        if i == n:
            break
        arm = ET.SubElement(body, "body", name=f"m{i}_arm")
        r = math.radians(PITCH_RANGE)
        ET.SubElement(arm, "joint", name=f"m{i}_pitch", axis="1 0 0", range=v(-r, r))
        ET.SubElement(arm, "geom", {"class": "visual", "mesh": "arm", "rgba": ARM_RGBA, "name": f"m{i}_arm_visual"})
        ET.SubElement(arm, "geom", {"class": "mass", "pos": v(*(c * s for c in ARM_BOX[0])),
                                    "size": v(*(h * s for h in ARM_BOX[1])), "mass": v(p["arm_mass"])})
        parent = arm

    # actuators in CSV order: m1_yaw, m1_pitch, ..., mN_yaw, mN_pitch (skipping the two that don't exist)
    act = ET.SubElement(root, "actuator")
    for i in range(1, n + 1):
        for j, lim in ((f"m{i}_yaw", SLIP_RING_TARGETS if slip_ring else YAW_RANGE), (f"m{i}_pitch", PITCH_RANGE)):
            if j not in ("m1_yaw", f"m{n}_pitch"):
                ET.SubElement(act, "position", name=j, joint=j, ctrlrange=v(-math.radians(lim), math.radians(lim)))

    contact = ET.SubElement(root, "contact")   # neighbours stay one pitch apart, never touch: skip them
    for i in range(1, n):
        ET.SubElement(contact, "exclude", body1=f"m{i}", body2=f"m{i + 1}")

    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "subtreecom", name="com", body="m1")
    ET.SubElement(sensor, "subtreelinvel", name="com_vel", body="m1")
    ET.SubElement(sensor, "subtreeangmom", name="angmom", body="m1")

    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", default="original14", choices=PRESETS)
    ap.add_argument("-n", "--modules", type=int, default=N_MODULES)
    ap.add_argument("--slip-ring", action="store_true", help="remove the yaw joint limits")
    ap.add_argument("--gravity", type=float, default=0.0, help="m/s^2 downward, towards the floor (Earth: 9.81)")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "chain.xml"))
    a = ap.parse_args()
    assert a.modules >= 2, "need at least 2 modules"
    with open(a.out, "w") as f:
        f.write(build(a.preset, a.modules, a.slip_ring, a.gravity) + "\n")
    p = PRESETS[a.preset]
    print(f"wrote {a.out}: {a.modules} x {a.preset} ({p['servo']}, {p['torque']} N·m, {p['pitch'] * 100:g} cm pitch)"
          f"{', slip ring' if a.slip_ring else ''}{f', gravity {a.gravity:g} m/s^2' if a.gravity else ''}")
