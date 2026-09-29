#!/usr/bin/env python3
"""s67 diag 4: is the open chain a THIN SLIT (two sides ~step apart,
both in the same face) or a WIDE HOLE boundary?

For the top face chains: for each chain vertex, find nearest OTHER
chain vertex (excluding immediate edge-neighbors along the chain).
If the chain is a thin slit folded onto itself, NN-vertex distance
will be ~local step (0.01-0.02). If it's a wide hole boundary, NN
will be large (~band width).
Also: chain endpoint coordinates and the STEP-style geometry hint.
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"
TARGETS = [236, 212, 127, 160, 130, 125, 214]


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

    for fid in TARGETS:
        edges = face_bnd.get(fid, [])
        adj = defaultdict(set)
        for (u, v) in edges:
            adj[u].add(v); adj[v].add(u)
        # largest connected chain
        seen, best = set(), []
        for s in adj:
            if s in seen:
                continue
            stack, comp = [s], []
            seen.add(s)
            while stack:
                n = stack.pop()
                comp.append(n)
                for m in adj[n]:
                    if m not in seen:
                        seen.add(m); stack.append(m)
            if len(comp) > len(best):
                best = comp
        comp = set(best)
        # NN vertex within chain, excluding 1-2 hop neighbors
        chain_edges = [e for e in edges
                       if e[0] in comp and e[1] in comp]
        hop2 = set()
        for v in comp:
            hop2.add(v)
            for m in adj[v]:
                hop2.add(m)
                for m2 in adj[m]:
                    hop2.add(m2)
        pts = [(v, verts[v]) for v in comp]
        CELL = 0.05
        grid = defaultdict(list)
        for v, p in pts:
            grid[(int(p[0]/CELL), int(p[1]/CELL),
                  int(p[2]/CELL))].append((v, p))
        nn_d = []
        for v, p in pts:
            best_d = 1e9
            cx, cy, cz = (int(p[k]/CELL) for k in range(3))
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    for dz in (-2, -1, 0, 1, 2):
                        for (v2, p2) in grid.get(
                                (cx+dx, cy+dy, cz+dz), ()):
                            if v2 in hop2:
                                continue
                            d = math.dist(p, p2)
                            if d < best_d:
                                best_d = d
            if best_d < 1e8:
                nn_d.append(best_d)
        nn_d.sort()
        # endpoints of chain (deg-1 verts)
        ends = [v for v in comp if len(adj[v]) == 1]
        end_pts = [verts[v] for v in ends]
        print(f"fid={fid:4d}: chain {len(comp)} verts "
              f"{len(chain_edges)} edges | self-NN "
              f"min={nn_d[0]:.5f} med={nn_d[len(nn_d)//2]:.5f} "
              f"p90={nn_d[int(len(nn_d)*0.9)]:.5f} | "
              f"{len(ends)} ends: " +
              " ".join(f"({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})"
                       for p in end_pts[:4]))


if __name__ == "__main__":
    main()
