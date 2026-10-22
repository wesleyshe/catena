"""Print a summary of a .3dm file: units, layers, objects, colours, geometry types, bounding boxes.
Bounding boxes come from the stored render meshes (tight); Rhino's own quick boxes can be much larger.
Usage: .venv/bin/python inspect_3dm.py ["../hardware/cad/Single module existing.3dm"]
"""
import os
import sys

import rhino3dm as r3

from export_meshes import SRC, render_mesh

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(SRC), "Single module existing.3dm")
m = r3.File3dm.Read(path)
print(f"File: {path}")
print(f"Units: {m.Settings.ModelUnitSystem}  abs tol: {m.Settings.ModelAbsoluteTolerance}")

print(f"\nLayers ({len(m.Layers)}):")
for i, L in enumerate(m.Layers):
    print(f"  [{i}] {L.FullPath!r:40} colour={tuple(L.Color[:3])}")


def colour(o):
    a = o.Attributes
    by_object = a.ColorSource == r3.ObjectColorSource.ColorFromObject
    return tuple((a.ObjectColor if by_object else m.Layers[a.LayerIndex].Color)[:3])


print(f"\nObjects ({len(m.Objects)}):")
print(f"  {'#':>4} {'layer':26} {'name':14} {'type':13} {'colour':16} {'tris':>6}  bbox min -> max (model units)")
for i, o in enumerate(m.Objects):
    g, a = o.Geometry, o.Attributes
    t = render_mesh(g)
    if t is not None:
        lo, hi = t.bounds
    else:
        b = g.GetBoundingBox()
        lo, hi = (b.Min.X, b.Min.Y, b.Min.Z), (b.Max.X, b.Max.Y, b.Max.Z)
    print(f"  {i:4d} {m.Layers[a.LayerIndex].FullPath[:26]:26} {(a.Name or '-')[:14]:14} {type(g).__name__:13} "
          f"{str(colour(o)):16} {len(t.faces) if t is not None else '-':>6}  "
          f"({lo[0]:8.2f},{lo[1]:8.2f},{lo[2]:7.2f}) -> ({hi[0]:8.2f},{hi[1]:8.2f},{hi[2]:7.2f})")
