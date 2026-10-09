#!/usr/bin/env python3
"""s87: per-face 3D extent of the SLEEVE cone-band faces — test the
sub-tolerance-band hypothesis: f41/f155 pockets/bands whose FULL 3D
height < eff_tol (0.0153) weld-collapse vertically regardless of the
triangulation (the s86 catch-22 generalizes to whole faces)."""
from collections import defaultdict
import math

def load(base):
    verts = []
    tris = []
    for line in open(base + ".obj"):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            _, a, b, c = line.split()
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    tf = {}
    for line in open(base + ".fmap"):
        if line.startswith("t "):
            _, ti, fid = line.split()
            tf[int(ti)] = int(fid)
    return verts, tris, tf

verts, tris, tf = load("forensics/s87_final_base/brep2_SHAFT_SLEEVE (BREP#32629)")
# surface type map
stypes = {}
for line in open("forensics/s87_final_base/brep2_SHAFT_SLEEVE (BREP#32629).facemap"):
    parts = line.split()
    if parts[0] == "f":
        stypes[int(parts[1])] = parts[2]

# axis: the cone band — axis along z? compute per-face z-extent and
# radial extent around the best axis (x,y centroid)
face_verts = defaultdict(set)
for ti, t in enumerate(tris):
    f = tf.get(ti, -1)
    for v in t:
        face_verts[f].add(v)

print(f"{'face':>5s} {'type':>8s} {'ntri':>5s} {'nvert':>5s} {'z-ext':>8s} {'rad-ext':>8s} {'axis-ext':>8s}")
for f in sorted(face_verts):
    if stypes.get(f) != "Cone":
        continue
    vs = [verts[i] for i in face_verts[f]]
    zs = [v[2] for v in vs]
    # radial around z-axis at the local centroid x,y
    cx = sum(v[0] for v in vs) / len(vs)
    cy = sum(v[1] for v in vs) / len(vs)
    rs = [math.hypot(v[0] - cx, v[1] - cy) for v in vs]
    zext = max(zs) - min(zs)
    rext = max(rs) - min(rs)
    ntri = sum(1 for ti in range(len(tris)) if tf.get(ti) == f)
    print(f"{f:>5d} {stypes.get(f):>8s} {ntri:>5d} {len(vs):>5d} {zext:>8.4f} {rext:>8.4f} {math.hypot(zext, rext):>8.4f}")
