#!/usr/bin/env python3
"""s63 diag: replicate fold_face_probe's pair-finding on the dumped OBJs.

For each OBJ: build edge->tris by VERTEX INDEX (exactly like the probe),
count edges with exactly 2 tris, dihedral angle distribution, pairs >170.
Also build edge->tris by GEOMETRIC coordinate (rounded) to detect index
splitting (adjacent faces not sharing indices).
"""
import glob
import math
import os
import sys
from collections import defaultdict

def load_obj(path):
    vs, tris = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                _, x, y, z = line.split()
                vs.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                a, b, c = line.split()[1:4]
                tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    return vs, tris

def normal(vs, t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vs[t[0]], vs[t[1]], vs[t[2]]
    ux, uy, uz = bx - ax, by - ay, bz - az
    wx, wy, wz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
    l = math.sqrt(nx * nx + ny * ny + nz * nz)
    if l < 1e-30:
        return None
    return (nx / l, ny / l, nz / l)

def audit(path):
    vs, tris = load_obj(path)
    n_tris = len(tris)
    # index-keyed edges
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append((ti, v0, v1))
    cnt_hist = defaultdict(int)
    for e, lst in e2t.items():
        cnt_hist[len(lst)] += 1
    # geometric-keyed edges (round to 1e-7)
    g2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            k0 = tuple(round(x, 7) for x in vs[v0])
            k1 = tuple(round(x, 7) for x in vs[v1])
            g2t[(min(k0, k1), max(k0, k1))].append((ti, v0, v1))
    g_hist = defaultdict(int)
    for e, lst in g2t.items():
        g_hist[len(lst)] += 1
    # dihedral >170 by index
    normals = [normal(vs, t) for t in tris]
    degenerate = sum(1 for n in normals if n is None)
    pairs170 = 0
    max_ang = 0.0
    over160 = 0
    for e, lst in e2t.items():
        if len(lst) != 2:
            continue
        (t0, _, _), (t1, _, _) = lst
        n0, n1 = normals[t0], normals[t1]
        if n0 is None or n1 is None:
            continue
        dot = max(-1.0, min(1.0, n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]))
        ang = math.degrees(math.acos(dot))
        max_ang = max(max_ang, ang)
        if ang > 170:
            pairs170 += 1
        if ang > 160:
            over160 += 1
    return dict(
        name=os.path.basename(path)[:40],
        V=len(vs), T=n_tris,
        idx_edge_hist=dict(sorted(cnt_hist.items())),
        geo_edge_hist=dict(sorted(g_hist.items())),
        geo_edges_2=g_hist.get(2, 0),
        idx_edges_2=cnt_hist.get(2, 0),
        pairs170=pairs170, over160=over160,
        max_ang=round(max_ang, 2),
        degen=degenerate,
    )

def main():
    paths = sorted(glob.glob("/home/z/my-project/scripts/objs63/*.obj"))
    print(f"{'name':40} {'V':>5} {'T':>5} {'idxE2':>6} {'geoE2':>6} {'p170':>5} {'>160':>5} {'maxA':>7} {'deg':>3}  idx_hist")
    tot170 = 0
    for p in paths:
        r = audit(p)
        tot170 += r["pairs170"]
        print(f"{r['name']:40} {r['V']:5} {r['T']:5} {r['idx_edges_2']:6} {r['geo_edges_2']:6} "
              f"{r['pairs170']:5} {r['over160']:5} {r['max_ang']:7} {r['degen']:3}  {r['idx_edge_hist']}")
    print(f"\nTOTAL pairs>170 by index: {tot170}")

if __name__ == "__main__":
    main()
