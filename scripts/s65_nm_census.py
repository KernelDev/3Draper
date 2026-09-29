#!/usr/bin/env python3
"""s65 diag 1: non-manifold edge census over ALL Zentralstaender BREPs +
deep attribution for #1086/#1088 (plan item 1: 189 nm + 6 bnd).

nm edge = mesh edge used by >2 triangles. bnd edge = used by 1.
For #1086/#1088: attribute every nm edge to its face-set, cluster by
(face-set), report edge lengths + a few sample edges with coordinates.
"""
import math
import os
import glob
from collections import defaultdict, Counter

DUMP = "/tmp/s65_objs"

def load(obj_path):
    vs, tris = [], []
    for line in open(obj_path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
            tris.append((a, b, c))
    fids = {}
    fmap = obj_path.replace(".obj", ".fmap")
    for line in open(fmap):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fids[int(ti)] = int(fid)
    return vs, tris, fids

def edge_stats(obj_path):
    vs, tris, fids = load(obj_path)
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            e2t[e].append(ti)
    bnd = [e for e, l in e2t.items() if len(l) == 1]
    nm = [e for e, l in e2t.items() if len(l) > 2]
    return vs, tris, fids, e2t, bnd, nm

def dist(vs, e):
    (x0, y0, z0), (x1, y1, z1) = vs[e[0]], vs[e[1]]
    return math.dist((x0, y0, z0), (x1, y1, z1))

def main():
    print("=== nm/bnd census over all BREPs ===")
    print(f"{'brep':>6} {'name':<46} {'bnd':>6} {'nm':>5}")
    for obj in sorted(glob.glob(os.path.join(DUMP, "*.obj"))):
        base = os.path.basename(obj)
        num = int(base.split("_")[0].replace("brep", ""))
        name = base[6:-4]
        vs, tris, fids, e2t, bnd, nm = edge_stats(obj)
        print(f"{num:>6} {name:<46} {len(bnd):>6} {len(nm):>5}")

    print("\n=== #1086 deep nm attribution ===")
    for tag in ("brep23", "brep24"):
        obj = [p for p in glob.glob(os.path.join(DUMP, f"{tag}_*.obj"))][0]
        vs, tris, fids, e2t, bnd, nm = edge_stats(obj)
        print(f"\n--- {os.path.basename(obj)}: bnd={len(bnd)} nm={len(nm)} ---")
        # group nm edges by (faceset, tri-count)
        groups = defaultdict(list)
        for e in nm:
            fl = tuple(sorted({fids.get(t, -1) for t in e2t[e]}))
            groups[(fl, len(e2t[e]))].append(e)
        for (fl, k), edges in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            lens = sorted(dist(vs, e) for e in edges)
            print(f"  faces={list(fl)} tris/edge={k}: {len(edges)} edges, "
                  f"len [{lens[0]:.4f}..{lens[-1]:.4f}] med={lens[len(lens)//2]:.4f}")
        # bnd attribution too
        bgroups = defaultdict(list)
        for e in bnd:
            fl = tuple(sorted({fids.get(t, -1) for t in e2t[e]}))
            bgroups[fl].append(e)
        print(f"  bnd groups:")
        for fl, edges in sorted(bgroups.items(), key=lambda kv: -len(kv[1])):
            lens = sorted(dist(vs, e) for e in edges)
            print(f"    faces={list(fl)}: {len(edges)} edges, "
                  f"len [{lens[0]:.4f}..{lens[-1]:.4f}]")

if __name__ == "__main__":
    main()
