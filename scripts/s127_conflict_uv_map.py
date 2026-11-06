#!/usr/bin/env python3
"""s127 п.1b: localize face-2 emission winding conflicts in (u,v) space.

Fit cylinder axis from the emission verts (all on the cylinder), compute
(u, v) per vertex, then map the 107 same-direction conflict edges into
the (u,v) band. Question: periodic lattice defect vs localized?
"""
import math
from collections import defaultdict

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
print(f"verts={len(verts)} tris={len(tris)}")

# ── fit cylinder axis via PCA on verts ─────────────────────────────
n = len(verts)
c = tuple(sum(v[k] for v in verts)/n for k in range(3))
# covariance
cov = [[0.0]*3 for _ in range(3)]
for v in verts:
    d = [v[0]-c[0], v[1]-c[1], v[2]-c[2]]
    for i in range(3):
        for j in range(3):
            cov[i][j] += d[i]*d[j]
# power iteration for principal axis
def matvec(m, x):
    return [sum(m[i][j]*x[j] for j in range(3)) for i in range(3)]
ax = [0.0, 0.0, 1.0]
for _ in range(200):
    y = matvec(cov, ax)
    mag = math.sqrt(sum(t*t for t in y))
    ax = [t/mag for t in y]
print(f"axis = ({ax[0]:.6f}, {ax[1]:.6f}, {ax[2]:.6f}), centroid=({c[0]:.4f},{c[1]:.4f},{c[2]:.4f})")

# orthonormal frame
def norm3(v):
    m = math.sqrt(sum(t*t for t in v))
    return [t/m for t in v]
tmp = [1.0, 0.0, 0.0] if abs(ax[0]) < 0.9 else [0.0, 1.0, 0.0]
e1 = norm3([tmp[i] - sum(tmp[k]*ax[k] for k in range(3))*ax[i] for i in range(3)])
e2 = norm3([ax[1]*e1[2]-ax[2]*e1[1], ax[2]*e1[0]-ax[0]*e1[2], ax[0]*e1[1]-ax[1]*e1[0]])

rs = []
uv = []
for v in verts:
    d = [v[k]-c[k] for k in range(3)]
    vv = sum(d[k]*ax[k] for k in range(3))
    px = sum(d[k]*e1[k] for k in range(3))
    py = sum(d[k]*e2[k] for k in range(3))
    r = math.hypot(px, py)
    rs.append(r)
    uv.append((math.atan2(py, px), vv, r))
rmed = sorted(rs)[len(rs)//2]
print(f"radius: median={rmed:.5f} min={min(rs):.5f} max={max(rs):.5f}")

# ── conflict edges ────────────────────────────────────────────────
used = defaultdict(list)
for ti, t in enumerate(tris):
    for k in range(3):
        a, b = t[k], t[(k+1) % 3]
        used[(a, b)].append(ti)
same = []
for (a, b), tls in used.items():
    if len(tls) == 2 and (b, a) not in used:
        same.append(((a, b), tls))
print(f"same-direction conflict edges: {len(same)}")

# where are they in (u,v)?
cells = defaultdict(int)
for (a, b), tls in same:
    ua, va, _ = uv[a]
    ub, vb, _ = uv[b]
    # unwrap
    if ub - ua > math.pi: ub -= 2*math.pi
    if ub - ua < -math.pi: ub += 2*math.pi
    um, vm = (ua+ub)/2, (va+vb)/2
    cells[(round(um, 1), round(vm, 1))] += 1
print("\nconflict-edge (u,v) heatmap (bin=0.1):")
ucols = sorted(set(k[0] for k in cells))
vrows = sorted(set(k[1] for k in cells), reverse=True)
print("      " + "".join(f"{u:5.1f}" for u in ucols))
for vr in vrows:
    row = "".join(f"{cells.get((u, vr), 0):5d}" for u in ucols)
    print(f"v={vr:5.1f} {row}")

# conflict triangles' u-distribution
print("\nu-range of conflict edges:")
us = []
for (a, b), tls in same:
    ua, va, _ = uv[a]
    ub, vb, _ = uv[b]
    if ub - ua > math.pi: ub -= 2*math.pi
    if ub - ua < -math.pi: ub += 2*math.pi
    us.append((ua+ub)/2)
hist = defaultdict(int)
for u in us:
    hist[round(u, 0)] += 1
for k in sorted(hist):
    print(f"  u≈{k:+.0f}: {hist[k]}")
