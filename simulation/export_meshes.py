"""Export one STL per rigid body from '../hardware/cad/7cm module.3dm' into meshes/.

The file is one module; the chain repeats it every pitch (spec.toml) along +Z. Each STL is in its module frame: module
centre (yaw/pitch axis intersection) at the origin, joints at zero, chain along +Z, metres. Bodies:
  base_first  module 1: shell + body + yaw servo + pitch servo (its yaw servo turns ee1 when one is fitted)
  base_mid    modules 2-11: the same
  base_last   module 12: shell + body + yaw servo (no pitch servo, no C-handle)
  arm         the C-handle (modules 1-11), pivots about +X at the module centre
Layers: V6::shell, V6::body and V6::servos make the base, V6::C-handle the arm; V6::axis holds the yaw and pitch axis
lines, which give the module centre. The pitch servo's parts are the servo
parts centred within 6.1 mm of the pitch axis's height (it's 12.1 mm thick, lying along X); the yaw servo's sit below.
Usage: .venv/bin/python export_meshes.py
"""
import hashlib
import os
import numpy as np
import rhino3dm as r3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "hardware", "cad", "7cm module.3dm")  # never modified
OUT = os.path.join(HERE, "meshes")
STAMP = os.path.join(OUT, "SOURCE.txt")  # which CAD file the meshes came from; verify.py checks it still matches
ARM_LAYER, SHELL_LAYER, SERVO_LAYER, AXIS_LAYER = "V6::C-handle", "V6::shell", "V6::servos", "V6::axis"
PITCH_SERVO_Z = 6.1                      # mm: servo parts centred this close to the pitch axis are the pitch servo's
MAX_TRIS = 20000                         # per body; shells are kept at full resolution


def fingerprint(path):
    """SHA-256 of a file: tells whether the CAD has changed since the meshes were exported."""
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def render_mesh(g):
    """Stored render mesh of a Brep/Extrusion/Mesh as a trimesh (mm), or None for curves/points."""
    if isinstance(g, r3.Brep):
        ms = [f.GetMesh(r3.MeshType.Render) for f in g.Faces]
    elif isinstance(g, r3.Extrusion):
        ms = [g.GetMesh(r3.MeshType.Render)]
    elif isinstance(g, r3.Mesh):
        ms = [g]
    else:
        return None
    parts = []
    for mm in ms:
        v = np.array([[p.X, p.Y, p.Z] for p in (mm.Vertices[i] for i in range(len(mm.Vertices)))])
        f = []
        for i in range(len(mm.Faces)):
            a, b, c, d = mm.Faces[i]
            f.append([a, b, c])
            if c != d:
                f.append([a, c, d])
        parts.append(trimesh.Trimesh(v, f, process=False))
    return trimesh.util.concatenate(parts)


def centre(model):
    """The module centre (mm): where the axis layer's pitch line (along X) and yaw line (along Z) cross."""
    ends = {}
    for o in model.Objects:
        if model.Layers[o.Attributes.LayerIndex].FullPath == AXIS_LAYER:
            a, b = (np.array([p.X, p.Y, p.Z]) for p in (o.Geometry.PointAtStart, o.Geometry.PointAtEnd))
            ends["pitch" if abs(b - a)[0] > abs(b - a)[2] else "yaw"] = a
    return np.array([ends["yaw"][0], ends["pitch"][1], ends["pitch"][2]])


def part_of(layer, t):
    """'arm', 'pitch_servo' or 'base' for an object, from its layer and (for servo parts) the height of its centre."""
    if layer == ARM_LAYER:
        return "arm"
    if layer == SERVO_LAYER and abs(t.bounds.mean(0)[2]) <= PITCH_SERVO_Z:
        return "pitch_servo"
    return "base"


def cluster(t, cell):
    """Vertex-clustering decimation: snap vertices to a grid of `cell` mm (error < one cell)."""
    key = np.floor(t.vertices / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    v = np.zeros((inv.max() + 1, 3))
    np.add.at(v, inv, t.vertices)
    v /= np.bincount(inv)[:, None]
    f = inv[t.faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    _, idx = np.unique(np.sort(f, 1), axis=0, return_index=True)
    return trimesh.Trimesh(v, f[np.sort(idx)], process=False)


def decimate(parts):
    """Cluster every non-shell part on the finest grid that keeps the body under MAX_TRIS."""
    keep = [p for p in parts if p.metadata["keep"]]
    rest = [p for p in parts if not p.metadata["keep"]]
    budget = MAX_TRIS - sum(len(p.faces) for p in keep)
    lo, hi = 0.05, 5.0
    for _ in range(25):  # bisect on the grid size (mm)
        cell = (lo + hi) / 2
        lo, hi = (lo, cell) if sum(len(cluster(p, cell).faces) for p in rest) <= budget else (cell, hi)
    print(f"  clustered on a {hi:.2f} mm grid")
    return trimesh.util.concatenate(keep + [cluster(p, hi) for p in rest])


def main():
    model = r3.File3dm.Read(SRC)
    c0 = centre(model)
    print(f"module centre in the file: {c0.round(3)} mm")
    parts = {"base": [], "pitch_servo": [], "arm": []}
    for o in model.Objects:
        t = render_mesh(o.Geometry)
        if t is None:
            continue
        layer = model.Layers[o.Attributes.LayerIndex].FullPath
        if layer == AXIS_LAYER:
            continue
        t.apply_translation(-c0)
        t.merge_vertices()
        t.metadata["keep"] = layer == SHELL_LAYER
        parts[part_of(layer, t)].append(t)
    for name, p in parts.items():
        print(f"{name:11}: {len(p):3d} objects, {sum(len(t.faces) for t in p):6d} tris")

    os.makedirs(OUT, exist_ok=True)
    full = parts["base"] + parts["pitch_servo"]
    pick = {"base_first": full, "base_mid": full, "base_last": parts["base"], "arm": parts["arm"]}
    for fname, body in pick.items():
        n0 = sum(len(p.faces) for p in body)
        t = decimate(body) if n0 > MAX_TRIS else trimesh.util.concatenate(body)
        t.apply_scale(0.001)  # mm -> m
        t.export(f"{OUT}/{fname}.stl")
        lo, hi = t.bounds * 1000
        print(f"{fname:10} {n0:6d} -> {len(t.faces):5d} tris  bounds mm {lo.round(1)} .. {hi.round(1)}")
    with open(STAMP, "w") as f:
        f.write(f"{os.path.basename(SRC)}\n{fingerprint(SRC)}\n")


if __name__ == "__main__":
    main()
