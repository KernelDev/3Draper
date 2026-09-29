#!/usr/bin/env python3
"""s65 diag 7: compare the lip-chain (#5577) discretization as used by
f9 (lune flap) vs f16 (skirt) — after the shape-guard fix. The chain
should be bit-identical (same STEP entity, edge cache). Where do the
11 unmatched bnd edges per cone come from?
"""
import math
import glob
from collections import defaultdict

OBJ = glob.glob("/tmp/s65_objs/brep23_*.obj")[0]

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
    for line in open(obj_path.replace(".obj", ".fmap")):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fids[int(ti)] = int(fid)
    return vs, tris, fids

def local_coords(p):
    ly = p[0] + 45.0
    lx = -660.6873 - p[1]
    lz = p[2] - 520.5
    return (lx, ly, lz)

def main():
    vs, tris, fids = load(OBJ)

    # vertices used by each face
    fverts = defaultdict(set)
    for ti, f in fids.items():
        for v in tris[ti]:
            fverts[f].add(v)

    # global edge -> faces
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            e2t[e].append(ti)

    # lip chain = boundary verts of f9 with local y < 4.999 (below top plane)
    # and azimuth in [−60°, 0°]
    def az(p):
        return math.degrees(math.atan2(p[2], p[0]))

    lip_f9 = sorted(
        (v for v in fverts[9]
         if local_coords(vs[v])[1] < 4.999),
        key=lambda v: az(local_coords(vs[v])))
    lip_f16 = sorted(
        (v for v in fverts[16]
         if 4.7 < local_coords(vs[v])[1] < 4.999),
        key=lambda v: az(local_coords(vs[v])))

    print(f"f9 lip verts: {len(lip_f9)}   f16 lip verts: {len(lip_f16)}")
    common = set(lip_f9) & set(lip_f16)
    print(f"shared verts: {len(common)}")
    only9 = [v for v in lip_f9 if v not in set(lip_f16)]
    only16 = [v for v in lip_f16 if v not in set(lip_f9)]
    print(f"only in f9: {len(only9)}, only in f16: {len(only16)}")
    for v in only9[:15]:
        lx, ly, lz = local_coords(vs[v])
        print(f"  f9-only v{v}: local=({lx:.4f}, {ly:.4f}, {lz:.4f}) r={math.hypot(lx,lz):.4f} az={math.degrees(math.atan2(lz,lx)):.2f}")
        continue
    for v in only16[:15]:
        lx, ly, lz = local_coords(vs[v])
        print(f"  f16-only v{v}: local=({lx:.4f}, {ly:.4f}, {lz:.4f}) r={math.hypot(lx,lz):.4f} az={math.degrees(math.atan2(lz,lx)):.2f}")

    # global bnd edges of f9 (used by 1 tri): print their geometry
    print("\n=== f9 global bnd edges (unmatched by any other face) ===")
    for e, tl in sorted(e2t.items()):
        if len(tl) != 1 or fids.get(tl[0]) != 9:
            continue
        p0, p1 = local_coords(vs[e[0]]), local_coords(vs[e[1]])
        print(f"  edge {e}: ({p0[0]:.4f},{p0[1]:.4f},{p0[2]:.4f}) -> ({p1[0]:.4f},{p1[1]:.4f},{p1[2]:.4f}) len={math.dist(p0,p1):.4f}")

    print("\n=== f16 bnd edges near the lip (y>4.7) ===")
    for e, tl in sorted(e2t.items()):
        if len(tl) != 1 or fids.get(tl[0]) != 16:
            continue
        p0, p1 = local_coords(vs[e[0]]), local_coords(vs[e[1]])
        if p0[1] > 4.7 and p1[1] > 4.7:
            print(f"  edge {e}: ({p0[0]:.4f},{p0[1]:.4f},{p0[2]:.4f}) -> ({p1[0]:.4f},{p1[1]:.4f},{p1[2]:.4f}) len={math.dist(p0,p1):.4f}")

if __name__ == "__main__":
    main()
