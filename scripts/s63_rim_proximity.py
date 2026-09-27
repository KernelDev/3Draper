#!/usr/bin/env python3
"""s63 diag: point-proximity between pre-merge per-face rims that share
a STEP edge. All faces of one BREP are dumped in the same local frame.

For each boundary vertex of face A, distance to nearest boundary vertex
of face B. Shared-edge rims with matching discretization -> ~0 dists;
mismatched sampling -> bimodal (some 0 at endpoints, rest > tol).
"""
import math
import os
import sys
from collections import defaultdict

DIR = "/home/z/my-project/scripts/fobjs1086"

def load_rim(fname):
    vs, tris = [], []
    for line in open(os.path.join(DIR, fname)):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:4]
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append(ti)
    rim = sorted({v for e in e2t if len(e2t[e]) == 1 for v in e})
    return [vs[v] for v in rim], len(tris), len(vs)

def stats(pa, pb):
    ds = []
    for p in pa:
        best = min(math.dist(p, q) for q in pb)
        ds.append(best)
    ds.sort()
    n = len(ds)
    zero = sum(1 for d in ds if d < 1e-9)
    near = sum(1 for d in ds if d < 1e-6)
    mid = sum(1 for d in ds if d < 1e-3)
    return dict(n=n, exact=zero, near1e6=near, near1e3=mid,
                med=ds[n // 2], max=ds[-1], p90=ds[int(n * 0.9)])

def main():
    pairs = [
        ("brep1086_f16_s1737_Plane.obj", "brep1086_f9_s1730_Cone.obj", "f16 plane vs f9 cone (edge #5577 BSPLINE)"),
        ("brep1086_f16_s1737_Plane.obj", "brep1086_f15_s1736_Cone.obj", "f16 plane vs f15 cone (edge #5585 BSPLINE)"),
        ("brep1086_f16_s1737_Plane.obj", "brep1086_f20_s1741_Plane.obj", "f16 plane vs f20 plane (edge #5589 LINE)"),
        ("brep1086_f9_s1730_Cone.obj", "brep1086_f15_s1736_Cone.obj", "f9 cone vs f15 cone"),
    ]
    for fa, fb, label in pairs:
        pa, ta, va = load_rim(fa)
        pb, tb, vb = load_rim(fb)
        s = stats(pa, pb)
        print(f"{label}")
        print(f"  A: rim={len(pa)} (T={ta})  B: rim={len(pb)} (T={tb})")
        print(f"  nearest-dist: exact={s['exact']} <1e-6={s['near1e6']} <1e-3={s['near1e3']} med={s['med']:.2e} p90={s['p90']:.2e} max={s['max']:.2e}")

if __name__ == "__main__":
    main()
