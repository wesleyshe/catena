"""Export one STL per rigid body from '../hardware/cad/complete chain.3dm' into meshes/.

Each STL is in its module frame: module centre (yaw/pitch axis intersection) at the origin,
joints at zero, chain along +Z, metres. Bodies:
  base_first  module 1: shell + pitch servo + U2D2 + power hub (no yaw servo)
  base_mid    modules 2-11: shell + bracket + yaw servo + pitch servo
  base_last   module 12: shell + yaw servo + U2D2 + power hub (no pitch servo, no arm)
  arm         U-yoke + hub (modules 1-11), pivots about +X at the module centre
Usage: .venv/bin/python export_meshes.py
"""
import os
import numpy as np
import rhino3dm as r3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "hardware", "cad", "complete chain.3dm")  # never modified
OUT = os.path.join(HERE, "meshes")
C0 = np.array([-400.70, -399.53, 52.0])  # module 1 centre in the file (mm)
PITCH = 140.0                            # module centre-to-centre (mm), chain runs along +Z
MAX_TRIS = 20000                         # per body; shells are kept at full resolution


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


def body_of(layer, zc):
    """(body, module index) for an object, from its layer and the height of its centre."""
    k = (zc - C0[2]) / PITCH
    if layer.startswith("Link"):      # yoke spans -12..+88 mm about its own module centre
        return "arm", int(np.floor(k + 20 / PITCH))
    return "base", int(round(k))


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
    bodies = {}
    for o in model.Objects:
        t = render_mesh(o.Geometry)
        if t is None:
            continue
        name, k = body_of(model.Layers[o.Attributes.LayerIndex].FullPath, t.bounds.mean(0)[2])
        t.apply_translation(-(C0 + [0, 0, PITCH * k]))
        t.merge_vertices()
        t.metadata["keep"] = model.Layers[o.Attributes.LayerIndex].FullPath == "Shell::Shell"
        bodies.setdefault((name, k), []).append(t)

    for (name, k), parts in sorted(bodies.items()):
        print(f"module {k + 1:2d} {name:4}: {len(parts):3d} objects, {sum(len(p.faces) for p in parts):6d} tris")

    os.makedirs(OUT, exist_ok=True)
    pick = {"base_first": ("base", 0), "base_mid": ("base", 1), "base_last": ("base", 11), "arm": ("arm", 0)}
    for fname, key in pick.items():
        n0 = sum(len(p.faces) for p in bodies[key])
        t = decimate(bodies[key]) if n0 > MAX_TRIS else trimesh.util.concatenate(bodies[key])
        t.apply_scale(0.001)  # mm -> m
        t.export(f"{OUT}/{fname}.stl")
        lo, hi = t.bounds * 1000
        print(f"{fname:10} {n0:6d} -> {len(t.faces):5d} tris  bounds mm {lo.round(1)} .. {hi.round(1)}")


if __name__ == "__main__":
    main()
