#!/usr/bin/env python3
"""s77: anatomy of the Nurbs twin-fan class — face 215 (Nurbs sail,
cps=9x9) vs 216 (Plane wall) in HOUSING_MIRROR (brep#62542), off vs
THIN_STRIP_ZIPPER=1.

For each state: per-face triangle census, fan-degree per vertex
(how many of the face's triangles touch the vertex), area share of the
top apex, sliver count, and the shared-rim structure.
"""
import sys
from collections import defaultdict

def load(base):
    name = "brep4_HOUSING_MIRROR (BREP#62542)"
    verts = {}
    tris = {}
    fmap = {}
    facemap = {}
    vi = 0
    for line in open(f"{base}/{name}.obj"):
        if line.startswith("v "):
            p = line.split()
            vi += 1
            verts[vi] = tuple(float(x) for x in p[1:4])
    for line in open(f"{base}/{name}.fmap"):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fmap[int(ti)] = int(fid)
    for line in open(f"{base}/{name}.facemap"):
        if line.startswith("f "):
            toks = line.split()
            facemap[int(toks[1])] = (" ".join(toks[2:-2]), int(toks[-2]), toks[-1] == "true")
    for line in open(f"{base}/{name}.obj"):
        if line.startswith("f "):
            p = line.split()
            ti = len(tris)
            # OBJ is 1-based; probe/indices elsewhere are 0-based
            tris[ti] = tuple(int(x) - 1 for x in p[1:4])
    return verts, tris, fmap, facemap

def area2(a, b, c):
    return abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))

def face_report(tag, verts, tris, fmap, facemap, want):
    tis = [ti for ti, fid in fmap.items() if fid in want]
    print(f"== {tag}: faces {sorted(want)} — {len(tis)} tris")
    deg = defaultdict(int)
    for ti in tis:
        for v in tris[ti]:
            deg[v] += 1
    n = len(tis)
    top = sorted(deg.items(), key=lambda kv: -kv[1])[:12]
    print(f"   fan-degree top: " + ", ".join(f"v{v}:{d}({100*d//max(1,3*n)}%)" for v, d in top))
    # slivers: tri whose min area2 is near zero relative to face's total
    tot = sum(area2(verts[tris[ti][0]], verts[tris[ti][1]], verts[tris[ti][2]]) for ti in tis) or 1.0
    sliv = 0
    for ti in tis:
        a2 = area2(verts[tris[ti][0]], verts[tris[ti][1]], verts[tris[ti][2]])
        if a2 / tot < 1e-4:
            sliv += 1
    print(f"   slivers(<1e-4 share): {sliv}/{n}")
    # index structure of fan apex triangles: apex + two consecutive ring idx
    apex, cnt = top[0]
    ring = sorted({v for ti in tis for v in tris[ti]} - {apex})
    span = (min(ring), max(ring)) if ring else None
    print(f"   ring around apex v{apex}: {len(ring)} verts, idx-span {span}")
    return apex, n, top

def main():
    for state in ("off", "on"):
        base = f"forensics/s77_dumps/{state}"
        verts, tris, fmap, facemap = load(base)
        print(f"\n#### STATE {state.upper()} (total tris={len(tris)})")
        face_report(state, verts, tris, fmap, facemap, {215})
        face_report(state, verts, tris, fmap, facemap, {216})
        # cross-face: triangles of 215 vs 216 sharing all 3 verts? (merge-dedup
        # survivors would NOT be duplicated across faces at final level)
        t215 = {tris[ti]: ti for ti, fid in fmap.items() if fid == 215}
        t216 = {tris[ti]: ti for ti, fid in fmap.items() if fid == 216}
        dup = set(t215) & set(t216)
        print(f"   cross-face identical tris 215∩216: {len(dup)}")

if __name__ == "__main__":
    main()
