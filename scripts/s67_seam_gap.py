#!/usr/bin/env python3
"""s67 diag 3: are the top bnd chains unwelded seams?

For each top face's bnd edges, find the NEAREST other boundary edge
(midpoint distance) across the whole mesh — restricted to edges NOT in
the same chain. Report: distance distribution (min/med/p90) and which
face owns the nearest counterpart edges. A bimodal ~0 distance = an
unwelded seam with a measurable gap; owner face = the other side.
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP, FMAPF = BASE + ".obj", BASE + ".fmap", BASE + ".facemap"
TARGETS = [236, 238, 212, 127, 166, 160, 162, 158, 164, 130, 125, 214]


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
    stype = {}
    for line in open(FMAPF):
        if line.startswith("f "):
            parts = line.split()
            stype[int(parts[1])] = " ".join(parts[2:-2])

    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    # all bnd edges with owner fid + midpoint
    bnd_all = []
    for e, ts in edge_tris.items():
        if len(ts) == 1:
            fid = fid_of.get(ts[0], -1)
            m = tuple((verts[e[0]][k] + verts[e[1]][k]) / 2
                      for k in range(3))
            bnd_all.append((m, fid, e))
    # grid index for fast NN (cell 0.05)
    CELL = 0.05
    grid = defaultdict(list)
    for i, (m, _, _) in enumerate(bnd_all):
        grid[(int(m[0] / CELL), int(m[1] / CELL),
              int(m[2] / CELL))].append(i)

    def nn(m, skip_fid, skip_set):
        best, bf = 1e9, None
        cx, cy, cz = (int(m[k] / CELL) for k in range(3))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for i in grid.get((cx + dx, cy + dy, cz + dz), ()):
                        m2, f2, e2 = bnd_all[i]
                        if e2 in skip_set:
                            continue
                        d = math.dist(m, m2)
                        if d < best:
                            best, bf = d, f2
        return best, bf

    for fid in TARGETS:
        my = [(m, e) for (m, f, e) in bnd_all if f == fid]
        my_set = set(e for _, e in my)
        if not my:
            continue
        # biggest chain only? use ALL my edges, skip same-face counterparts
        dists, owners = [], defaultdict(int)
        for m, e in my:
            d, f2 = nn(m, fid, my_set)
            dists.append(d)
            owners[f2] += 1
        dists.sort()
        own = sorted(owners.items(), key=lambda kv: -kv[1])[:3]
        print(f"fid={fid:4d} [{stype.get(fid,'?'):12s}] {len(my):4d} bnd | "
              f"NN-dist min={dists[0]:.5f} med={dists[len(dists)//2]:.5f} "
              f"p90={dists[int(len(dists)*0.9)]:.5f} | "
              f"owners: " +
              ", ".join(f"{o[0]}({stype.get(o[0],'?')[:9]}):{o[1]}"
                        for o in own))


if __name__ == "__main__":
    main()
