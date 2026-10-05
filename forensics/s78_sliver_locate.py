#!/usr/bin/env python3
"""s78: localize the Cone×Nurbs sliver fold pairs — the probe's
tris=([...],[...]) are VERTEX ids (step indices): two triangles
sharing the edge (894,889) with third vertices 882 (Cone side) and
883 (Nurbs side). Find them in the OBJ, print 3D + UV."""
import sys
from collections import defaultdict

def load(base):
    name = "brep2_SHAFT_SLEEVE (BREP#32629)"
    verts = []
    tris = []
    vt = []
    fmap = {}
    facemap = {}
    with open(f"{base}/{name}.obj") as f:
        lines = f.readlines()
    for line in lines:
        if line.startswith("v "):
            p = line.split()
            verts.append(tuple(float(x) for x in p[1:4]))
        elif line.startswith("vt "):
            p = line.split()
            vt.append((float(p[1]), float(p[2])))
        elif line.startswith("f "):
            p = line.split()
            idx = []
            for tok in p[1:4]:
                parts = tok.split("/")
                vi = int(parts[0]) - 1
                idx.append(vi)
            tris.append(tuple(idx))
    with open(f"{base}/{name}.fmap") as f:
        for line in f:
            if line.startswith("t "):
                _, ti, fid = line.split()
                fmap[int(ti)] = int(fid)
    with open(f"{base}/{name}.facemap") as f:
        for line in f:
            if line.startswith("f "):
                toks = line.split()
                facemap[int(toks[1])] = (toks[2], int(toks[-2]))
    return verts, tris, fmap, facemap, vt

def main():
    base = "/tmp/s78_final"
    verts, tris, fmap, facemap, vt = load(base)
    # probe pairs: (shared edge verts, thirdA, thirdB, coneFace, nurbsFace)
    targets = [
        ((894, 889), 882, 883, 52, 108),
        ((875, 877), 870, 869, 51, 101),
        ((945, 947), 938, 939, 56, 136),
    ]
    for (edge, ta, tb, fc, fn) in targets:
        print(f"\n=== pair ({fc},{fn}): shared edge verts {edge}, thirds {ta}/{tb} ===")
        for ti, t in enumerate(tris):
            if set(edge).issubset(set(t)):
                fid = fmap.get(ti)
                others = [v for v in t if v not in edge]
                print(f"  tri {ti} (face {fid} = {facemap.get(fid)}): verts {t}, third={others}")
                for v in t:
                    print(f"     v{v}: ({verts[v][0]:+.4f},{verts[v][1]:+.4f},{verts[v][2]:+.4f})")
                # area
                a, b, c = (verts[t[0]], verts[t[1]], verts[t[2]])
                cr = ((b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1]),
                      (b[2]-a[2])*(c[0]-a[0]) - (b[0]-a[0])*(c[2]-a[2]),
                      (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
                area = 0.5 * (cr[0]**2 + cr[1]**2 + cr[2]**2) ** 0.5
                print(f"     area={area:.6f}")
    # face tri counts for SAIL faces
    print("\n=== SAIL face census ===")
    cnt = defaultdict(int)
    for ti, fid in fmap.items():
        cnt[fid] += 1
    for f in sorted(cnt):
        t = facemap.get(f, ("?",))[0]
        if f in (86, 94, 101, 108, 115, 122, 129, 136, 143, 308, 315, 322, 329, 336, 343, 350, 357, 364, 51, 52, 56):
            print(f"  f{f}: {cnt[f]} tris, {t}")

if __name__ == "__main__":
    main()
