#!/usr/bin/env python3
"""s67 diag 6: SELF point-to-polyline distance of a face's bnd chain.

If an intra-face unmeshed strip (width ~1 lattice row) exists, the
chain's verts lie ~strip-width from OTHER edges of the SAME chain
(not their own edges). Segment-based exact distance, grid 0.05.
Also measures against the face's NON-boundary (interior) edges to
see which side of the strip IS meshed.
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"
TARGETS = [125, 214, 212, 127, 158, 160, 162, 164, 166, 236, 238, 130]


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
    face_all = defaultdict(list)
    for e, ts in edge_tris.items():
        f = fid_of.get(ts[0], -1)
        face_all[f].append(e)
        if len(ts) == 1:
            face_bnd[f].append(e)

    CELL = 0.05
    for fid in TARGETS:
        eb = face_bnd.get(fid, [])
        if not eb:
            continue
        # grid of the face's OWN bnd segments
        grid = defaultdict(list)
        for (u, v) in eb:
            m = tuple((verts[u][k] + verts[v][k]) / 2 for k in range(3))
            grid[(int(m[0]/CELL), int(m[1]/CELL),
                  int(m[2]/CELL))].append((u, v))
        vs = set()
        for (u, v) in eb:
            vs.add(u); vs.add(v)
        own = defaultdict(set)  # vert -> its own edges' other endpoint
        for (u, v) in eb:
            own[u].add(v); own[v].add(u)
        ds = []
        for vi in vs:
            p = verts[vi]
            cx, cy, cz = (int(p[k]/CELL) for k in range(3))
            best = 1e9
            for r in range(5):
                if best < 1e8:
                    break
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        for dz in range(-r, r + 1):
                            if max(abs(dx), abs(dy), abs(dz)) != r:
                                continue
                            for (u, v) in grid.get(
                                    (cx+dx, cy+dy, cz+dz), ()):
                                if u in own[vi] or v in own[vi]:
                                    continue
                                if u == vi or v == vi:
                                    continue
                                d = pt_seg_dist(p, verts[u], verts[v])
                                if d < best:
                                    best = d
            ds.append(best)
        ds.sort()
        n = len(ds)
        print(f"f{fid:4d} ({len(eb):4d} bnd, {n:4d} verts): self-dist "
              f"min={ds[0]:.5f} med={ds[n//2]:.5f} "
              f"p90={ds[int(n*0.9)]:.5f} max={ds[-1]:.4f} | "
              f"frac<0.03={sum(1 for d in ds if d < 0.03)/n:.2f} "
              f"frac~0.05={sum(1 for d in ds if 0.04 <= d <= 0.06)/n:.2f}")


if __name__ == "__main__":
    main()
