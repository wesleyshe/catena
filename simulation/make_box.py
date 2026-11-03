"""The flight box in the model: the acrylic box on the plane with a net across it, and strings that hold the chain still
in the middle until play.py lets them go at zero g. The box, its net, the strings' far ends and a cabin camera are all
on one body, "cabin", on a hinge about x (across the cabin), which play.py drives with the plane's pitch; the world is a
frame moving with the box but not turning, so the chain feels the plane's rotation for itself. make_chain.build() calls
cabin(), tie_ons() and strings(). Stdlib only, like make_chain.py.
"""
import math
import xml.etree.ElementTree as ET

from spec import FLIGHT, v

ACRYLIC_T = 0.006       # m: half-thickness of the flight box's walls and lid
ACRYLIC_RGBA, NET_RGBA, STRING_RGBA = "0.70 0.85 1.00 0.10", "0.92 0.92 0.88 0.04", "1 1 1 1"
# What the box looks like (visual only: no mass, no contacts). Representation, not a design: 40 mm aluminium extrusion on
# every edge and up the middle of the long sides, the net on its own 20 mm frame with a 50 mm mesh, a door on the aisle
# side (-x), and feet on the seat tracks.
FRAME_MM, NET_FRAME_MM, MESH_MM = 40, 20, 50
ALU_RGBA, MESH_RGBA, DOOR_RGBA = "0.66 0.68 0.71 1", "0.95 0.95 0.90 1", "0.30 0.32 0.35 1"
STRING_SOLREF = "0.01 1"  # the strings: stretchy enough to settle, stiff enough to hold the chain within mm at 1.8 g
STRING_FAN = 1.4        # each held module's strings run to the box's top and bottom edges (both sides) at this many
                        # times its distance from the middle along the chain, so together they hold it every way
NET_IMPEDANCE, NET_DAMPING = 0.05, 0.3  # the net: a weak contact (so gravity isn't cancelled) with a spring and a
                                        # damping ratio; nets bounce, hence the low damping


def net_contact(sag_mm):
    """The net's contact: the spring of a net that sags sag_mm under a module at 1 g, so a landing sinks in as far as it
    would on that net. (At rest MuJoCo's soft contact sits a few mm deep, shallower than sag_mm.) MuJoCo scales a direct
    stiffness and damping (negative solref) by the impedance, hence the (1 - d)."""
    k = (1 - NET_IMPEDANCE) * 9.81 / (sag_mm / 1000)
    return dict(solref=v(-k, -2 * NET_DAMPING * math.sqrt(k)), solimp=v(NET_IMPEDANCE, NET_IMPEDANCE, 0.001))


def held_height(box_m):
    """Where the strings hold the chain: in the middle of the room above the net (m above the box floor)."""
    net = FLIGHT["net_height_mm"] / 1000
    return net + (box_m[2] - net) / 2


def string_ends(i, n, pitch, box_m):
    """Module i's four strings: where each ends on the box (on the top and bottom edges of its long sides) and the
    direction to there from the module's centre, with the chain straight where the strings hold it."""
    along, across, tall = box_m
    y, h = (-(n - 1) / 2 + i - 1) * pitch, held_height(box_m)
    out = []
    for x, z in ((-across / 2, tall), (across / 2, tall), (-across / 2, 0.0), (across / 2, 0.0)):
        end = (x, max(-along / 2, min(along / 2, y * STRING_FAN)), z)
        w = [end[0], end[1] - y, end[2] - h]
        out.append((end, [c / math.sqrt(sum(c * c for c in w)) for c in w]))
    return out


def looks(cabin, x, y, h, net):
    """The flight box's look (visual geoms on the cabin): its frame, the net's frame and mesh, the door, the feet.
    x, y, h: half-sizes to the panels' outer faces; net: the net's height in the cabin's frame."""
    look = dict(contype="0", conaffinity="0", mass="0", group="1")
    bar = lambda name, a, b, w, rgba=ALU_RGBA: ET.SubElement(cabin, "geom", name=name, type="box", rgba=rgba, **look,
                                                         pos=v(*((p + q) / 2 for p, q in zip(a, b))),
                                                         size=v(*(max(abs(q - p) / 2, w) for p, q in zip(a, b))))
    f, k = FRAME_MM / 2000, 0
    X, Y, Z = x + f, y + f, h + f  # the extrusion's centre lines, just outside the panels
    for sy in (-1, 1):
        for sz in (-1, 1):
            bar(f"frame{k}", (-X, sy * Y, sz * Z), (X, sy * Y, sz * Z), f); k += 1      # across
            bar(f"frame{k}", (sy * X, -Y, sz * Z), (sy * X, Y, sz * Z), f); k += 1      # along
            bar(f"frame{k}", (sy * X, sz * Y, -Z), (sy * X, sz * Y, Z), f); k += 1      # up the corners
        bar(f"frame{k}", (sy * X, 0, -Z), (sy * X, 0, Z), f); k += 1                    # up the middle of the long sides
    n = NET_FRAME_MM / 2000
    for sy in (-1, 1):  # the net's frame, just inside the panels, and its mesh
        bar(f"net_frame{sy}x", (-x + n, sy * (y - n), net), (x - n, sy * (y - n), net), n)
        bar(f"net_frame{sy}y", (sy * (x - n), -y + n, net), (sy * (x - n), y - n, net), n)
    m = MESH_MM / 1000
    for j in range(1, int(2 * x / m)):
        bar(f"mesh_x{j}", (-x + j * m, -y, net), (-x + j * m, y, net), 0.0012, MESH_RGBA)
    for j in range(1, int(2 * y / m)):
        bar(f"mesh_y{j}", (-x, -y + j * m, net), (x, -y + j * m, net), 0.0012, MESH_RGBA)
    d, top = 0.012, -h + 1.4  # the door in the aisle-side panel: 0.8 m wide, up to 1.4 m, with a handle
    for name, a, b in (("door_left", (-X, -0.4, -h), (-X, -0.4, top)), ("door_right", (-X, 0.4, -h), (-X, 0.4, top)),
                       ("door_top", (-X, -0.4, top), (-X, 0.4, top))):
        bar(name, a, b, d, DOOR_RGBA)
    bar("door_handle", (-X - 0.03, 0.3, -h + 0.75), (-X - 0.03, 0.3, -h + 0.95), d, DOOR_RGBA)
    for sx in (-1, 1):  # feet on the seat tracks (studs 40 mm high)
        for sy in (-1, 1):
            bar(f"foot{sx}{sy}", (sx * X, sy * Y, -Z - 0.04), (sx * X, sy * Y, -Z), 0.03)


def cabin(box_m, rolling):
    """The cabin body, centred on the box, which has its floor at z = 0 while the plane is level: acrylic walls, lid and
    floor, the net, the cabin's own floor, the look and a camera. Not yet in the world: build() adds it after the chain,
    so m1 stays body 1. rolling: the net's rolling friction, the floor's (the lopsided modules lying on it would rock and
    roll sideways without it). The cabin_floor uses build()'s "grid" material."""
    along, across, tall = box_m
    # the net is a soft block from the box floor up to its height, so a chain that lands hard sinks into it and, if it
    # gives all the way, meets the box floor.
    # A real body, not mocap: a mocap body has no velocity, so whatever rested on the box would slide as it turned.
    # It's so heavy the chain can't move it, and play.py sets its angle and rate every step
    body = ET.Element("body", name="cabin", pos=v(0, 0, tall / 2))
    ET.SubElement(body, "inertial", pos="0 0 0", mass="1e4", diaginertia="1e4 1e4 1e4")
    ET.SubElement(body, "joint", name="cabin_pitch", type="hinge", axis="1 0 0", damping="0", armature="0",
                  frictionloss="0")
    x, y, h, t = across / 2, along / 2, tall / 2, ACRYLIC_T
    for name, pos, size in (("box_lid", (0, 0, h + t), (x, y, t)), ("box_floor", (0, 0, -h - t), (x, y, t)),
                            ("box_wall_-x", (-x - t, 0, 0), (t, y, h)), ("box_wall_+x", (x + t, 0, 0), (t, y, h)),
                            ("box_wall_-y", (0, -y - t, 0), (x, t, h)), ("box_wall_+y", (0, y + t, 0), (x, t, h))):
        ET.SubElement(body, "geom", name=name, type="box", pos=v(*pos), size=v(*size), rgba=ACRYLIC_RGBA, mass="0")
    net_z = FLIGHT["net_height_mm"] / 1000
    ET.SubElement(body, "geom", name="net", type="box", pos=v(0, 0, -h + net_z / 2), size=v(x, y, net_z / 2),
                  rgba=NET_RGBA, mass="0", condim="6", friction=v(1, 0.005, rolling),
                  **net_contact(FLIGHT["net_sag_mm"]))  # the cabin is moved, never pushed: no mass
    ET.SubElement(body, "geom", name="cabin_floor", type="box", pos=v(0, 0, -h - 2 * t - 0.045), size="4 4 0.005",
                  material="grid", contype="0", conaffinity="0", mass="0")  # what the cabin camera sees under the box
    looks(body, x + t, y + t, h + t, -h + net_z)
    eye = (x + 1.6, -y - 1.6, 0.5)  # a camera on the cabin, outside the box at one corner, looking at its middle
    back = [c / math.sqrt(sum(e * e for e in eye)) for c in eye]
    right = [-back[1], back[0], 0]
    right = [c / math.hypot(*right) for c in right]
    up = [back[1] * right[2] - back[2] * right[1], back[2] * right[0] - back[0] * right[2], back[0] * right[1] - back[1] * right[0]]
    ET.SubElement(body, "camera", name="cabin", pos=v(*eye), xyaxes=v(*right, *up), fovy="60")
    return body


def tie_ons(module, i, n, pitch, box_m, roll, r_shell, s):
    """Where module i's four strings tie on: sites on its shell (body `module`), each facing its string's far end."""
    for k, (end, w) in enumerate(string_ends(i, n, pitch, box_m)):
        axes = ((math.sin(roll), 0, math.cos(roll)), (math.cos(roll), 0, -math.sin(roll)), (0, 1, 0))  # m1's
        ET.SubElement(module, "site", name=f"m{i}_string{k}", size=v(0.004 * s),
                      pos=v(*(r_shell * s * sum(a * b for a, b in zip(axis, w)) for axis in axes)))


def strings(root, cabin_body, flight, n, pitch):
    """The strings: four spatial tendons per held module, from its tie-ons to sites on the box's edges. play.py sets each
    string's length (holding) or makes it endless (let go)."""
    tendon = ET.SubElement(root, "tendon")
    tall = flight["box_m"][2]
    for i in flight["strings"]:
        for k, (end, _) in enumerate(string_ends(i, n, pitch, flight["box_m"])):
            ET.SubElement(cabin_body, "site", name=f"m{i}_string{k}_end", pos=v(end[0], end[1], end[2] - tall / 2), size=v(0.005))
            string = ET.SubElement(tendon, "spatial", name=f"m{i}_string{k}", limited="true", range="0 100",
                                   width="0.0015", rgba=STRING_RGBA, solreflimit=STRING_SOLREF)
            ET.SubElement(string, "site", site=f"m{i}_string{k}_end")
            ET.SubElement(string, "site", site=f"m{i}_string{k}")
