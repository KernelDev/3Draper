#!/usr/bin/env python3
"""s63 diag: attribute FINAL-mesh boundary edges of a BREP to faces +
geometry. Works on the DRAPPER_DUMP_FINAL_OBJS dump (obj + fmap).
"""
import math
import os
import re
import sys
from collections import defaultdict

def load_pair(obj_path):
    fmap_path = obj_path.replace(".obj", ".fmap")
    vs, tris = [], []
    for line in open(obj_path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:4]
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    fids = {}
    for line in open(fmap_path):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fids[int(ti)] = int(fid)
    return vs, tris, fids

def fit_circle(pts):
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    cz = sum(p[2] for p in pts) / n
    cov = [[0.0] * 3 for _ in range(3)]
    for p in pts:
        d = (p[0] - cx, p[1] - cy, p[2] - cz)
        for i in range(3):
            for j in range(3):
                cov[i][j] += d[i] * d[j]
    tr = cov[0][0] + cov[1][1] + cov[2][2]
    sh = tr * 1.5 + 1e-12
    v = (0.0, 0.1, 0.9)
    for _ in range(80):
        nv = tuple(
            (cov[i][0] - (sh if i == 0 else 0)) * v[0]
            + (cov[i][1] - (sh if i == 1 else 0)) * v[1]
            + (cov[i][2] - (sh if i == 2 else 0)) * v[2]
            for i in range(3)
        )
        l = math.sqrt(sum(x * x for x in nv)) or 1.0
        v = tuple(x / l for x in nv)
    r2 = 0.0
    for p in pts:
        d = (p[0] - cx, p[1] - cy, p[2] - cz)
        perp = d[0] * v[0] + d[1] * v[1] + d[2] * v[2]
        r2 += sum(x * x for x in d) - perp * perp
    return (cx, cy, cz), v, math.sqrt(r2 / n)

def main(obj_path):
    vs, tris, fids = load_pair(obj_path)
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append(ti)
    bnd = [e for e, lst in e2t.items() if len(lst) == 1]
    # attribute boundary edges to the face of their single triangle
    by_face = defaultdict(list)
    for e in bnd:
        ti = e2t[e][0]
        by_face[fids.get(ti, -1)].append(e)
    print(f"{os.path.basename(obj_path)}: V={len(vs)} T={len(tris)} bnd={len(bnd)}")
    for fid, edges in sorted(by_face.items(), key=lambda kv: -len(kv[1])):
        # cluster into runs
        adj = defaultdict(set)
        for (a, b) in edges:
            adj[a].add(b)
            adj[b].add(a)
        seen, runs = set(), []
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
        run_desc = []
        for comp in sorted(runs, key=len, reverse=True)[:4]:
            pts = [vs[v] for v in comp]
            if len(pts) >= 3:
                c, nrm, r = fit_circle(pts)
                run_desc.append(f"{len(pts)}pt R~{r:.3f} c=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) n=({nrm[0]:.2f},{nrm[1]:.2f},{nrm[2]:.2f})")
            else:
                run_desc.append(f"{len(pts)}pt")
        print(f"  face {fid}: {len(edges)} bnd, runs: {' | '.join(run_desc)}")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/z/my-project/scripts/objs63/brep23_BEVORRICHTUNG_BNO-002402-STD_A (BREP#1086).obj")
