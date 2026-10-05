#!/usr/bin/env python3
"""s77: reconstruct f215's earcutr/CDT inputs (two calls) — which one
matches the final mesh; fan structure of the OUTPUT triangles; interior
Steiner placement; identify the fan apex and the unused points.
"""
import sys
from collections import defaultdict

def load(path):
    bnd, holes, inter, tris, mode = [], defaultdict(list), [], [], None
    cur = None
    hi = 0
    for line in open(path):
        if line.startswith("type="):
            hdr = line.strip()
        elif line.startswith("boundary"):
            cur = "b"
        elif line.startswith("hole "):
            hi = int(line.split()[1]); cur = "h"
        elif line.startswith("interior"):
            cur = "i"
        elif line.startswith("tris"):
            cur = "t"
        else:
            p = line.split()
            if not p:
                continue
            if cur == "b" and p[0] == "b":
                bnd.append((float(p[1]), float(p[2])))
            elif cur == "h" and p[0] == "h":
                holes[hi].append((float(p[1]), float(p[2])))
            elif cur == "i" and p[0] == "i":
                inter.append((float(p[1]), float(p[2])))
            elif cur == "t" and p[0] == "t":
                tris.append((int(p[1]), int(p[2]), int(p[3])))
    return hdr, bnd, holes, inter, tris

def fan_analysis(tag, bnd, holes, inter, tris):
    n_b = len(bnd)
    pts = list(bnd)
    for hi in sorted(holes):
        pts += holes[hi]
    off = len(pts)
    pts += inter
    deg = defaultdict(int)
    for t in tris:
        for v in t:
            deg[v] += 1
    top = sorted(deg.items(), key=lambda kv: -kv[1])[:10]
    print(f"-- {tag}: n_pts={len(pts)} (b={n_b}, h={sum(len(v) for v in holes.values())}, i={len(inter)}), tris={len(tris)}")
    print(f"   top-degree: " + ", ".join(f"v{v}:{d}" for v, d in top))
    used_b = {v for v in deg if v < n_b}
    used_i = {v for v in deg if v >= off}
    print(f"   boundary used: {len(used_b)}/{n_b}; interior used: {len(used_i)}/{len(inter)}")
    unused_b = [i for i in range(n_b) if i not in deg]
    unused_i = [i for i in range(off, len(pts)) if i not in deg]
    if unused_b:
        print(f"   UNUSED boundary idx: {unused_b[:20]}{'...' if len(unused_b)>20 else ''} ({len(unused_b)})")
    if unused_i:
        print(f"   UNUSED interior idx(rel): {[i-off for i in unused_i][:20]} ({len(unused_i)})")
    # area share of top apex
    def area2(a, b, c):
        return abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
    tot = sum(area2(*[pts[i] for i in t]) for t in tris) or 1.0
    if top:
        apex = top[0][0]
        ashare = sum(area2(*[pts[i] for i in t]) for t in tris if apex in t) / tot
        print(f"   apex v{apex}: degree {top[0][1]}/{len(tris)} tris touch ({100*top[0][1]//max(1,len(tris))}%), area share {ashare:.1%}")
    # geometry of ring
    us = [p[0] for p in bnd]; vs = [p[1] for p in bnd]
    print(f"   ring UV bbox: u[{min(us):.4f},{max(us):.4f}] v[{min(vs):.4f},{max(vs):.4f}]")
    ius = [p[0] for p in inter]; ivs = [p[1] for p in inter]
    if inter:
        print(f"   interior UV bbox: u[{min(ius):.4f},{max(ius):.4f}] v[{min(ivs):.4f},{max(ivs):.4f}]")
    return pts, tris

def main():
    base = "forensics/s77_dumps/tri_on"
    for f, tag in (("tri_0129_Nurbs.txt", "CALL-A (111 int)"), ("tri_0298_Nurbs.txt", "CALL-B (49 int)")):
        hdr, bnd, holes, inter, tris = load(f"{base}/{f}")
        print(f"\n#### {f}: {hdr}")
        fan_analysis(tag, bnd, holes, inter, tris)

if __name__ == "__main__":
    main()
