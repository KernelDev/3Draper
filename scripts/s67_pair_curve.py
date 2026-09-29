#!/usr/bin/env python3
"""s67 diag 5: point-to-polyline distance between paired bnd chains.

Hypothesis test: are f125/f214 chains (and Torus pairs 212/127,
158↔204, etc.) two discretizations of the SAME shared STEP curve?
If yes: every vertex of chain A lies ~on chain B (dist ≤ chord sag,
i.e. « step^2). If no (parallel distinct curves): dist ≈ constant
band width (~0.05).
Prints: per-pair dist stats of A-verts to B-polyline (min/med/p90).
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"
PAIRS = [(125, 214), (214, 125), (212, 127), (127, 212),
         (158, 204), (160, 202), (236, 14), (130, 261)]


def pt_seg_dist(p, a, b):
    ab = tuple(b[k] - a[k] for k in range(3))
    ap = tuple(p[k] - a[k] for k in range(3))
    ab2 = sum(x * x for x in ab)
    if ab2 < 1e-18:
        return math.dist(p, a)
    t = max(0.0, min(1.0, sum(ap[k] * ab[k] for k in range(3)) / ab2))
    q = tuple(a[k] + t * ab[k] for k in range(3))
    return math.dist(p, q)


def main():
    verts, tris = [], []
    for line in open(OBJ):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(t) - 1 for t in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(FMAP):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    face_bnd = defaultdict(list)
    for e, ts in edge_tris.items():
        if len(ts) == 1:
            face_bnd[fid_of.get(ts[0], -1)].append(e)

    for fa, fb in PAIRS:
        ea, eb = face_bnd.get(fa, []), face_bnd.get(fb, [])
        if not ea or not eb:
            print(f"pair ({fa},{fb}): MISSING ({len(ea)}/{len(eb)})")
            continue
        # B polyline segments + spatial grid (cell 0.05)
        CELL = 0.05
        grid = defaultdict(list)
        for (u, v) in eb:
            m = tuple((verts[u][k] + verts[v][k]) / 2 for k in range(3))
            grid[(int(m[0]/CELL), int(m[1]/CELL),
                  int(m[2]/CELL))].append((u, v))
        va = set()
        for (u, v) in ea:
            va.add(u); va.add(v)
        ds = []
        for vi in va:
            p = verts[vi]
            cx, cy, cz = (int(p[k]/CELL) for k in range(3))
            best = 1e9
            for r in range(4):
                if best < 1e8:
                    break
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        for dz in range(-r, r + 1):
                            if max(abs(dx), abs(dy), abs(dz)) != r:
                                continue
                            for (u, v) in grid.get(
                                    (cx+dx, cy+dy, cz+dz), ()):
                                d = pt_seg_dist(p, verts[u], verts[v])
                                if d < best:
                                    best = d
            ds.append(best)
        ds.sort()
        print(f"A=f{fa:4d} ({len(ea):4d} bnd, {len(va):4d} verts) "
              f"-> B=f{fb:4d} polyline ({len(eb):4d} segs): "
              f"dist min={ds[0]:.6f} med={ds[len(ds)//2]:.6f} "
              f"p90={ds[int(len(ds)*0.9)]:.6f} max={ds[-1]:.4f}")


if __name__ == "__main__":
    main()
