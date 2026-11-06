#!/usr/bin/env python3
"""s127 п.1b v2: conflict localization with EXACT cylinder params.
Cylinder #75480: r=2.5, origin (0,-1.675,0), zdir (0,-1,0), xdir (0,0,-1).
u = atan2(dx, -dz), v = -(y+1.675).
Also: face 19 (STEP #22580) for comparison.
"""
import math, sys
from collections import defaultdict

O = (0.0, -1.675, 0.0)
ZD = (0.0, -1.0, 0.0)
XD = (0.0, 0.0, -1.0)
R = 2.5

def to_uv(p):
    dx, dy, dz = p[0]-O[0], p[1]-O[1], p[2]-O[2]
    u = math.atan2(dx, -dz)  # atan2(d.xdir, d.(-z of world)) — validated: first dump vert -> 0.851
    v = -(dy)                 # along zdir
    return u, v

def analyze(path, tag, expect_r=R):
    verts, tris = [], []
    for l in open(path):
        p = l.split()
        if not p:
            continue
        if p[0] == "v":
            verts.append(tuple(float(x) for x in p[2:5]))
        elif p[0] == "t":
            tris.append((int(p[2]), int(p[3]), int(p[4])))
    # radius check
    rs = []
    for p in verts:
        dx, dy, dz = p[0]-O[0], p[1]-O[1], p[2]-O[2]
        # distance to axis line: radial component in (x, z) plane around axis dir (0,-1,0)
        # axis through O along (0,-1,0): distance = sqrt(dx^2+dz^2) minus nothing (dy along axis)
        rs.append(math.hypot(dx, dz))
    rs.sort()
    print(f"== {tag}: verts={len(verts)} tris={len(tris)} "
          f"rad-to-axis: min={rs[0]:.4f} med={rs[len(rs)//2]:.4f} max={rs[-1]:.4f} (expect {expect_r})")
    used = defaultdict(list)
    for ti, t in enumerate(tris):
        for k in range(3):
            a, b = t[k], t[(k+1) % 3]
            used[(a, b)].append(ti)
    same = [((a, b), tls) for (a, b), tls in used.items()
            if len(tls) == 2 and (b, a) not in used]
    print(f"   same-dir conflicts: {len(same)}")
    # (u,v) of conflicts
    cells = defaultdict(int)
    us_all, vs_all = [], []
    for (a, b), tls in same:
        ua, va = to_uv(verts[a])
        ub, vb = to_uv(verts[b])
        if ub - ua > math.pi: ub -= 2*math.pi
        if ub - ua < -math.pi: ub += 2*math.pi
        um, vm = (ua+ub)/2, (va+vb)/2
        cells[(round(um, 2), round(vm, 2))] += 1
        us_all.append(um); vs_all.append(vm)
    if us_all:
        print(f"   conflict u-range: [{min(us_all):.3f}, {max(us_all):.3f}]  "
              f"v-range: [{min(vs_all):.3f}, {max(vs_all):.3f}]")
        # fine histogram of u
        hist = defaultdict(int)
        for u in us_all:
            hist[round(u, 1)] += 1
        print("   u-histogram (0.1 bins):")
        for k in sorted(hist):
            bar = "#" * hist[k]
            print(f"     u={k:+.1f}: {hist[k]:3d} {bar}")
    # all-verts uv extent for reference
    au = [to_uv(p) for p in verts]
    print(f"   all verts u: [{min(a[0] for a in au):.3f}, {max(a[0] for a in au):.3f}] "
          f"v: [{min(a[1] for a in au):.3f}, {max(a[1] for a in au):.3f}]")

if __name__ == "__main__":
    analyze("forensics/s126/fm10212/mesh_brep10212_f2_Cylinder_a1.txt", "f2 pass1")
    analyze("forensics/s126/fm10212/mesh_brep10212_f19_Cylinder_a18.txt", "f19 pass1")
