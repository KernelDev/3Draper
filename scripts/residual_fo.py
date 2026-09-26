#!/usr/bin/env python3
"""session-61: localize the 5/9 residual FO pairs on collapsed f18/f20.

The FACEFOLD scan reports fold-over pairs (shared edge, >170°, apexes
same side, real 2D overlap). Reproduce the scan offline on the dumped
pre-merge OBJ and print the pair geometry (edge, apexes, areas).
"""
import math

def load_obj(path):
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

def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def area2(p):
    s = 0.0
    for i in range(3):
        a, b = p[i], p[(i+1) % 3]
        s += a[0]*b[1] - b[0]*a[1]
    return s / 2.0

def analyze(fid):
    verts, tris = load_obj(f"/tmp/gear_collapse/face_{fid}_fo*.obj".replace("*", str(FO[fid])))
    n = len(tris)
    # edge → triangle owners
    from collections import defaultdict
    owners = defaultdict(list)
    for ti, t in enumerate(tris):
        for k in range(3):
            e = (min(t[k], t[(k+1) % 3]), max(t[k], t[(k+1) % 3]))
            owners[e].append(ti)
    print(f"=== f{fid}: {len(verts)} verts, {n} tris")
    fo_pairs = []
    for e, os_ in owners.items():
        if len(os_) != 2:
            continue
        t0, t1 = tris[os_[0]], tris[os_[1]]
        # shared edge endpoints
        a, b = verts[e[0]], verts[e[1]]
        # apexes
        p0 = verts[(set(t0) - {e[0], e[1]}).pop()]
        p1 = verts[(set(t1) - {e[0], e[1]}).pop()]
        # normal-side test: is p1 on the same side of edge (a→b) as p0?
        # project onto plane of t0
        import numpy as np
        A, B, P0, P1 = map(np.array, (a, b, p0, p1))
        n0 = np.cross(B - A, P0 - A)
        n1 = np.cross(B - A, P1 - A)
        dotn = float(np.dot(n0, n1))
        # dihedral angle between the two triangles
        nn0 = np.cross(B - A, P0 - A)
        nn1 = np.cross(B - A, P1 - A)
        cosang = float(np.dot(nn0, nn1) / (np.linalg.norm(nn0) * np.linalg.norm(nn1)))
        ang = math.degrees(math.acos(max(-1, min(1, cosang))))
        if dotn > 0 and ang < 10.0:
            # same-side, nearly-parallel: candidate FOLD-OVER (scan
            # convention: dihedral>170 between oriented normals ==
            # apexes same side; here ang≈0 between my n0/n1)
            fo_pairs.append((e, os_, p0, p1, ang))
    print(f"same-side >170° pairs: {len(fo_pairs)}")
    for e, os_, p0, p1, ang in fo_pairs[:12]:
        a, b = verts[e[0]], verts[e[1]]
        elen = math.dist(a, b)
        d_apex = math.dist(p0, p1)
        print(f"  edge {e} len={elen:.4f} tris {os_} ang={ang:.1f} "
              f"apex-dist={d_apex:.5f}")
        print(f"    a={tuple(round(x,4) for x in a)} b={tuple(round(x,4) for x in b)}")
        print(f"    p0={tuple(round(x,4) for x in p0)} p1={tuple(round(x,4) for x in p1)}")

FO = {18: 5, 20: 9}
for fid in (18, 20):
    analyze(fid)
