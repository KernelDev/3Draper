#!/usr/bin/env python3
"""s63 diag: for each FINAL-mesh boundary point of given faces, list all
final-mesh vertices within search_tol, with face attribution. Shows
whether geometry exists nearby (stitched to wrong partner) or is
genuinely absent (missing coverage).
"""
import math
import sys
from collections import defaultdict, Counter

OBJ = "/home/z/my-project/scripts/objs63/brep23_BEVORRICHTUNG_BNO-002402-STD_A (BREP#1086).obj"
TOL = 0.053  # search_tol from boundary_twin_probe

def load():
    vs, tris = [], []
    for line in open(OBJ):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:4]
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    fids = {}
    for line in open(OBJ.replace(".obj", ".fmap")):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fids[int(ti)] = int(fid)
    return vs, tris, fids

def main():
    vs, tris, fids = load()
    # vertex -> faces
    vfaces = defaultdict(set)
    for ti, t in enumerate(tris):
        f = fids[ti]
        for v in t:
            vfaces[v].add(f)
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append(ti)
    bnd_edges = [e for e, lst in e2t.items() if len(lst) == 1]
    # spatial hash
    cell = TOL
    grid = defaultdict(list)
    for i, p in enumerate(vs):
        grid[(int(p[0] / cell), int(p[1] / cell), int(p[2] / cell))].append(i)

    def near(p):
        out = []
        cx, cy, cz = int(p[0] / cell), int(p[1] / cell), int(p[2] / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    out.extend(grid.get((cx + dx, cy + dy, cz + dz), []))
        return out

    bnd_by_face = defaultdict(list)
    for e in bnd_edges:
        ti = e2t[e][0]
        bnd_by_face[fids[ti]].extend(e)

    for face in (16, 15, 10):
        pts = sorted(set(bnd_by_face[face]))
        print(f"\n=== face {face}: {len(pts)} boundary vertices")
        absent = 0
        partner_hist = Counter()
        for v in pts:
            p = vs[v]
            cand = [(math.dist(p, vs[u]), u) for u in near(p)]
            cand = [(d, u) for d, u in cand if d <= TOL and u != v]
            # exclude vertices that are the SAME point welded (d~0) — those are co-located
            if not cand:
                absent += 1
                continue
            cand.sort()
            # faces of nearest non-self vertex
            d0, u0 = cand[0]
            if d0 < 1e-9:
                partner_hist["co-located:" + ",".join(map(str, sorted(vfaces[u0])))] += 1
            else:
                partner_hist[f"nearby d={d0:.4f}:" + ",".join(map(str, sorted(vfaces[u0])))] += 1
        print(f"  no vertex within {TOL}: {absent}/{len(pts)}")
        for k, c in partner_hist.most_common(8):
            print(f"  {c:4} × {k}")

if __name__ == "__main__":
    main()
