#!/usr/bin/env python3
"""session-61: localize the (f1, f18/f20) Plane|Cone pairs in the FINAL GEAR mesh.

Uses the final OBJ + fmap: reconstruct the merged mesh, find shared-edge
pairs with >170° dihedral between f1-triangles and f18/f20-triangles,
report their 3D midpoint + which tooth/corner it belongs to.
"""
import math
from collections import defaultdict

def load(path):
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                a, b, c = line.split()[1:]
                tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    return verts, tris

def load_fmap(path):
    fmap = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 3 and parts[0] == "t":
                fmap[int(parts[1])] = int(parts[2])
    return fmap

verts, tris = load("/tmp/gear_final/brep1_GEAR (BREP#16033).obj")
fmap = load_fmap("/tmp/gear_final/brep1_GEAR (BREP#16033).fmap")
print(f"{len(verts)} verts, {len(tris)} tris, fmap {len(fmap)}")

owners = defaultdict(list)
for ti, t in enumerate(tris):
    for k in range(3):
        e = (min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3]))
        owners[e].append(ti)

import numpy as np
pairs = []
for e, os_ in owners.items():
    if len(os_) != 2:
        continue
    (t0, t1) = (tris[os_[0]], tris[os_[1]])
    f0, f1 = fmap.get(os_[0], -1), fmap.get(os_[1], -1)
    if {f0, f1} != {1, 18} and {f0, f1} != {1, 20}:
        continue
    A, B = np.array(verts[e[0]]), np.array(verts[e[1]])
    P0 = np.array(verts[(set(t0) - {e[0], e[1]}).pop()])
    P1 = np.array(verts[(set(t1) - {e[0], e[1]}).pop()])
    n0 = np.cross(B - A, P0 - A); n1 = np.cross(B - A, P1 - A)
    # oriented dihedral: fold-over = apexes same side of the edge in-plane
    dotn = float(np.dot(n0, n1))
    # proper dihedral angle between triangle planes
    nn0 = np.cross(np.array(verts[t0[1]]) - np.array(verts[t0[0]]),
                   np.array(verts[t0[2]]) - np.array(verts[t0[0]]))
    nn1 = np.cross(np.array(verts[t1[1]]) - np.array(verts[t1[0]]),
                   np.array(verts[t1[2]]) - np.array(verts[t1[0]]))
    cosang = float(np.dot(nn0, nn1) / (np.linalg.norm(nn0) * np.linalg.norm(nn1)))
    ang = math.degrees(math.acos(max(-1, min(1, cosang))))
    same_side = dotn > 0
    mid = (A + B) / 2
    pairs.append((f0, f1, ang, same_side, mid, e, os_))

print(f"(f1,f18/f20) shared-edge pairs: {len(pairs)}")
fold = [p for p in pairs if p[3]]
print(f"  same-side (fold): {len(fold)}")
for (f0, f1, ang, ss, mid, e, os_) in fold[:20]:
    az = math.degrees(math.atan2(mid[2], mid[0])) % 360
    print(f"  f{f0}|f{f1} ang={ang:6.1f} mid=({mid[0]:+.3f},{mid[1]:+.4f},{mid[2]:+.3f}) "
          f"az={az:6.1f}° r={math.hypot(mid[0], mid[2]):.4f} edge={e}")
