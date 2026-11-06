#!/usr/bin/env python3
"""s127 п.1b v3: local structure of a conflict patch in the f2 emission.
Print triangles around conflict edges with (u,v) coords to see the lattice
defect pattern."""
import math
from collections import defaultdict

O = (0.0, -1.675, 0.0)
def to_uv(p):
    dx, dy, dz = p[0]-O[0], p[1]-O[1], p[2]-O[2]
    return math.atan2(dx, -dz), -dy

path = "forensics/s126/fm10212/mesh_brep10212_f2_Cylinder_a1.txt"
verts, tris = [], []
for l in open(path):
    p = l.split()
    if not p:
        continue
    if p[0] == "v":
        verts.append(tuple(float(x) for x in p[2:5]))
    elif p[0] == "t":
        tris.append((int(p[2]), int(p[3]), int(p[4])))

used = defaultdict(list)
for ti, t in enumerate(tris):
    for k in range(3):
        a, b = t[k], t[(k+1) % 3]
        used[(a, b)].append(ti)
same = [((a, b), tls) for (a, b), tls in used.items()
        if len(tls) == 2 and (b, a) not in used]
same.sort(key=lambda x: x[1][0])

# take the first 3 conflicts and print involved triangles with uv
for (a, b), tls in same[:3]:
    print(f"--- conflict edge ({a}->{b}) tris={tls}")
    for ti in tls:
        t = tris[ti]
        coords = []
        for v in t:
            u, vv = to_uv(verts[v])
            coords.append(f"v{v}({u:.3f},{vv:.3f})")
        print(f"    t{ti}: {coords[0]} {coords[1]} {coords[2]}")
    # any other triangles using a or b
    for vi in (a, b):
        owners = sorted({ti for (x, y), tl in used.items() for ti in tl if vi in (x, y)})
        print(f"    vert {vi} used by tris: {owners}")

# The lattice: sort interior verts by (u,v) — interior = not on any boundary chain?
# Simplest: histogram of u and v values of ALL verts (quantized) to see the grid
us, vs = [], []
for p in verts:
    u, v = to_uv(p)
    us.append(round(u, 3)); vs.append(round(v, 3))
print("\ngrid check: distinct u values (top 20 by count):")
uc = defaultdict(int)
for u in us: uc[u] += 1
for u, c in sorted(uc.items())[:20]:
    print(f"  u={u:.3f}: {c}")
print("distinct v values (top 20):")
vc = defaultdict(int)
for v in vs: vc[v] += 1
for v, c in sorted(vc.items())[:20]:
    print(f"  v={v:.3f}: {c}")
print(f"total distinct u: {len(uc)}, distinct v: {len(vc)}, verts: {len(verts)}")
