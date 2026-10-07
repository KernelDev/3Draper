#!/usr/bin/env python3
"""s81: anatomy of the (26,26) Nurbs self-fold family in HOUSING_MIRROR
(BREP#62542, step 53616, cps=4x10): 59 pairs (40 REAL). The face's own
triangulation folds against itself. Questions:
  - where are the fold edges (rim row vs interior)?
  - what do the folding triangle pairs look like (sizes, heights)?
  - is there an overlap/region-flip pattern (CDT dropped regions)?
"""
import math
from collections import defaultdict

OBJ = "/tmp/s81_final/brep4_HOUSING_MIRROR (BREP#62542).obj"
FMAP = "/tmp/s81_final/brep4_HOUSING_MIRROR (BREP#62542).fmap"
FOLD = "forensics/s81_drill_full.txt"

verts = []
tris = []
with open(OBJ) as f:
    for line in f:
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:]
            tris.append(tuple(int(v) - 1 for v in (a, b, c)))

fid_by_tri = {}
with open(FMAP) as f:
    for line in f:
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_by_tri[int(ti)] = int(fid)

# fold pairs for (26,26)
pairs = []
with open(FOLD) as f:
    for line in f:
        if "brep_idx=4" not in line or "faces=(26,26)" not in line:
            continue
        subtol = "SUBTOL" in line
        import re
        m = re.search(r"tris=\(\[(\d+), (\d+), (\d+)\],\[(\d+), (\d+), (\d+)\]\)", line)
        if not m:
            continue
        t0 = tuple(int(m.group(i)) for i in (1, 2, 3))
        t1 = tuple(int(m.group(i)) for i in (4, 5, 6))
        m2 = re.search(r"h=\(([\d.]+),([\d.]+)\)", line)
        h = (float(m2.group(1)), float(m2.group(2))) if m2 else (0, 0)
        pairs.append((t0, t1, h, subtol))

print(f"(26,26) fold pairs: {len(pairs)} (REAL {sum(1 for p in pairs if not p[3])})")

def dist(a, b):
    return math.dist(verts[a], verts[b])

def area(a, b, c):
    ax, ay, az = verts[a]; bx, by, bz = verts[b]; cx, cy, cz = verts[c]
    ux, uy, uz = bx-ax, by-ay, bz-az
    vx, vy, vz = cx-ax, cy-ay, cz-az
    cxv = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
    return 0.5*math.sqrt(cxv[0]**2+cxv[1]**2+cxv[2]**2)

# shared edges + fold locations
shared_edges = defaultdict(int)
for t0, t1, h, subtol in pairs:
    e0 = {tuple(sorted((t0[i], t0[(i+1)%3]))) for i in range(3)}
    e1 = {tuple(sorted((t1[i], t1[(i+1)%3]))) for i in range(3)}
    sh = e0 & e1
    for e in sh:
        shared_edges[e] += 1

print(f"distinct shared edges: {len(shared_edges)} (folds per edge: {sorted(shared_edges.values(), reverse=True)[:8]})")

# are the shared edges on the face boundary?
face26 = [ti for ti, t in enumerate(tris) if fid_by_tri.get(ti) == 26]
edge_use = defaultdict(int)
for ti in face26:
    t = tris[ti]
    for i in range(3):
        a, b = t[i], t[(i+1)%3]
        edge_use[tuple(sorted((a, b)))] += 1
bnd_edges = {e for e, c in edge_use.items() if c == 1}
fold_on_bnd = sum(1 for e in shared_edges if e in bnd_edges)
print(f"fold edges on face-26 boundary: {fold_on_bnd}/{len(shared_edges)}")

# geometry of fold pairs: edge length, apex heights, areas
stats = []
for t0, t1, h, subtol in pairs:
    e0 = {tuple(sorted((t0[i], t0[(i+1)%3]))) for i in range(3)}
    e1 = {tuple(sorted((t1[i], t1[(i+1)%3]))) for i in range(3)}
    sh = list(e0 & e1)
    if not sh:
        continue
    e = sh[0]
    elen = dist(*e)
    a0 = area(*t0)
    a1 = area(*t1)
    stats.append((elen, h[0], h[1], a0, a1, subtol))
stats.sort(key=lambda s: -s[0])
print("fold shared-edge lengths (top 10):")
for s in stats[:10]:
    print(f"  edge={s[0]:.4f} h=({s[1]:.4f},{s[2]:.4f}) areas=({s[3]:.5f},{s[4]:.5f}) subtol={s[5]}")
elens = [s[0] for s in stats]
print(f"edge len: min={min(elens):.4f} med={sorted(elens)[len(elens)//2]:.4f} max={max(elens):.4f}")

# mid points of folds (3D cluster)
import re
mids = []
with open(FOLD) as f:
    for line in f:
        if "brep_idx=4" in line and "faces=(26,26)" in line:
            m = re.search(r"mid=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)", line)
            if m:
                mids.append(tuple(float(m.group(i)) for i in (1, 2, 3)))
xs = [m[0] for m in mids]; ys = [m[1] for m in mids]; zs = [m[2] for m in mids]
print(f"fold midpoints bbox: x=[{min(xs):.2f},{max(xs):.2f}] y=[{min(ys):.2f},{max(ys):.2f}] z=[{min(zs):.2f},{max(zs):.2f}]")

# face-26 extent
vv = set()
for ti in face26:
    vv.update(tris[ti])
fx = [verts[v][0] for v in vv]; fy = [verts[v][1] for v in vv]; fz = [verts[v][2] for v in vv]
print(f"face-26 bbox: x=[{min(fx):.2f},{max(fx):.2f}] y=[{min(fy):.2f},{max(fy):.2f}] z=[{min(fz):.2f},{max(fz):.2f}] tris={len(face26)}")

# degree analysis: fan structure inside face 26?
deg = defaultdict(int)
for ti in face26:
    for v in tris[ti]:
        deg[v] += 1
top = sorted(deg.items(), key=lambda kv: -kv[1])[:6]
print(f"face-26 top vertex degrees: {top}")

# do fold triangles share an apex vertex?
apex_count = defaultdict(int)
for t0, t1, h, subtol in pairs:
    e0 = {tuple(sorted((t0[i], t0[(i+1)%3]))) for i in range(3)}
    e1 = {tuple(sorted((t1[i], t1[(i+1)%3]))) for i in range(3)}
    sh = list(e0 & e1)
    if not sh:
        continue
    e = set(sh[0])
    for v in t0:
        if v not in e:
            apex_count[v] += 1
tops = sorted(apex_count.items(), key=lambda kv: -kv[1])[:6]
print(f"fold apex verts: {tops}")
