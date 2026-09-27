#!/usr/bin/env python3
"""s63 diag: structure of cone f15's pre-merge mesh (brep1086, local).

Questions:
1. Is the apex (VERTEX_LOOP #1371's vertex) present in the mesh?
2. Vertex levels (y-bands): band = rim ring 1 / ring 2 / interior?
3. Are the two 72-pt rim rings hexagonal (6 corners) or circular?
4. Where do the 12 f16-matched points sit (which ring)?
5. Triangle count between levels: band or full cone?
"""
import math
import os
from collections import defaultdict, Counter

DIR = "/home/z/my-project/scripts/fobjs1086"

def load(fname):
    vs, tris = [], []
    for line in open(os.path.join(DIR, fname)):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:4]
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    return vs, tris

def main():
    vs, tris = load("brep1086_f15_s1736_Cone.obj")
    print(f"f15 mesh: V={len(vs)} T={len(tris)}")
    ys = Counter(round(v[1], 3) for v in vs)
    print("y-levels (y: count):", sorted(ys.items()))

    # radius from y-axis per level
    for yl in sorted(ys):
        pts = [v for v in vs if round(v[1], 3) == yl]
        rs = [math.hypot(v[0], v[2]) for v in pts]
        print(f"  y={yl}: n={len(pts)} R=[{min(rs):.4f}..{max(rs):.4f}]")

    # find the two rim rings (boundary loops)
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append(ti)
    bnd = [e for e, lst in e2t.items() if len(lst) == 1]
    adj = defaultdict(set)
    for (a, b) in bnd:
        adj[a].add(b)
        adj[b].add(a)
    seen, rings = set(), []
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
        rings.append(comp)
    print(f"\nboundary rings: {len(rings)}")
    f16_pts = None
    for ri, ring in enumerate(sorted(rings, key=len, reverse=True)):
        pts = [vs[v] for v in ring]
        n = len(pts)
        rs = [math.hypot(p[0], p[2]) for p in pts]
        ys2 = [p[1] for p in pts]
        # corner detection: angle at each point between neighbors
        ring_sorted = sorted(ring, key=lambda v: math.atan2(vs[v][2], vs[v][0]))
        corners = 0
        for i in range(n):
            a = vs[ring_sorted[i]]
            b = vs[ring_sorted[(i - 1) % n]]
            c = vs[ring_sorted[(i + 1) % n]]
            v1 = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v2 = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
            l1 = math.sqrt(sum(x * x for x in v1))
            l2 = math.sqrt(sum(x * x for x in v2))
            ang = math.degrees(math.acos(max(-1, min(1, dot / (l1 * l2))))) if l1 * l2 > 0 else 0
            if ang < 150:
                corners += 1
        print(f"  ring{ri}: {n} pts, y=[{min(ys2):.4f}..{max(ys2):.4f}] R=[{min(rs):.4f}..{max(rs):.4f}] corners(<150°)={corners}")

    # interior vertices (not on any ring)
    rim_all = set().union(*rings) if rings else set()
    interior = [i for i in range(len(vs)) if i not in rim_all]
    if interior:
        pts = [vs[i] for i in interior]
        print(f"\ninterior vertices: {len(interior)}")
        for i in interior[:10]:
            v = vs[i]
            print(f"  v{i}: ({v[0]:.4f},{v[1]:.4f},{v[2]:.4f}) R={math.hypot(v[0], v[2]):.4f}")
    else:
        print("\nNO interior vertices — pure band between rings (apex NOT in mesh!)")

if __name__ == "__main__":
    main()
