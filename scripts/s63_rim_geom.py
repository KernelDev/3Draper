#!/usr/bin/env python3
"""s63 diag: rim geometry of boundary-heavy faces of BREP#1086.

For selected faces from DRAPPER_DUMP_FACE_OBJS: find boundary edges
(exactly 1 triangle), cluster them into connected runs, and fit each
run: is it a circle arc (plane fit + radius), a line, etc. Report
center/radius/normal so we can identify which STEP edge it is and
which neighbor should attach.
"""
import math
import os
import re
import sys
from collections import defaultdict

DIR = "/home/z/my-project/scripts/fobjs1086"

def load(path):
    vs, tris = [], []
    for line in open(path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:4]
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    return vs, tris

def analyze(fname):
    vs, tris = load(os.path.join(DIR, fname))
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append(ti)
    bnd = [e for e, lst in e2t.items() if len(lst) == 1]
    if not bnd:
        return
    # cluster boundary edges into connected runs (shared vertices)
    adj = defaultdict(set)
    for (a, b) in bnd:
        adj[a].add(b)
        adj[b].add(a)
    seen = set()
    runs = []
    for v in adj:
        if v in seen:
            continue
        stack, comp = [v], set()
        while stack:
            u = stack.pop()
            if u in comp:
                continue
            comp.add(u)
            stack.extend(adj[u] - comp)
        seen |= comp
        runs.append(comp)
    print(f"\n=== {fname}: V={len(vs)} T={len(tris)} bnd_edges={len(bnd)} runs={len(runs)}")
    for ri, comp in enumerate(sorted(runs, key=len, reverse=True)):
        pts = [vs[v] for v in comp]
        n = len(pts)
        if n < 3:
            segs = []
            for u in comp:
                for w in adj[u]:
                    if u < w:
                        d = math.dist(vs[u], vs[w])
                        segs.append(f"u{u}-w{w} len={d:.4f}")
            print(f"  run{ri}: {n} pts (short) {'; '.join(segs[:3])}")
            continue
        # plane fit via centroid + SVD-lite (moment)
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        cz = sum(p[2] for p in pts) / n
        # covariance
        cov = [[0.0] * 3 for _ in range(3)]
        for p in pts:
            d = (p[0] - cx, p[1] - cy, p[2] - cz)
            for i in range(3):
                for j in range(3):
                    cov[i][j] += d[i] * d[j]
        # power iteration for normal (smallest eigenvector ~ normal of best plane)
        v = (0.0, 0.0, 1.0)
        # normal = eigenvector of smallest eigenvalue: use inverse power iteration
        import itertools
        def matvec(m, v):
            return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))
        # shift: subtract trace*1.5 to make smallest eigenvalue dominant negative→largest magnitude
        tr = cov[0][0] + cov[1][1] + cov[2][2]
        sh = tr * 1.5
        for _ in range(60):
            m = [row[:] for row in cov]
            for i in range(3):
                m[i][i] -= sh
            nv = matvec(m, v)
            l = math.sqrt(sum(x * x for x in nv)) or 1.0
            v = tuple(x / l for x in nv)
        nrm = v
        # radius about centroid projected in plane
        r2 = 0.0
        for p in pts:
            d = (p[0] - cx, p[1] - cy, p[2] - cz)
            perp = d[0] * nrm[0] + d[1] * nrm[1] + d[2] * nrm[2]
            r2 += sum(x * x for x in d) - perp * perp
        r = math.sqrt(r2 / n)
        # linearity: project onto plane, check collinearity via second axis
        print(f"  run{ri}: {n} pts  c=({cx:.4f},{cy:.4f},{cz:.4f}) n=({nrm[0]:.3f},{nrm[1]:.3f},{nrm[2]:.3f}) R~{r:.4f}")

def main():
    faces = sys.argv[1:] if len(sys.argv) > 1 else [
        "brep1086_f15_s1736_Cone.obj",
        "brep1086_f16_s1737_Plane.obj",
        "brep1086_f17_s1738_Plane.obj",
        "brep1086_f10_s1731_Cone.obj",
    ]
    for f in faces:
        analyze(f)

if __name__ == "__main__":
    main()
