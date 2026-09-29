#!/usr/bin/env python3
"""s67 diag 13: correct-frame UV map of f125: triangle coverage +
bnd-chain path. Reveals the shape of the 'meshed snake'.
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"
FID = 125
POS = (-2.54, 0.375, -4.349967143371)
AXIS = (0.0, 0.0, -1.0)


def to_local(p):
    return (p[2] - 2.325, p[0], p[1] - 5.4)


def uv(p):
    px, py, pz = to_local(p)
    rel = (px - POS[0], py - POS[1], pz - POS[2])
    v = sum(rel[k] * AXIS[k] for k in range(3))
    rad = tuple(rel[k] - v * AXIS[k] for k in range(3))
    return (math.atan2(rad[1], -rad[0]), v)


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

    mytris = [t for t in range(len(tris)) if fid_of.get(t) == FID]
    uvs = {}
    for ti in mytris:
        for vi in tris[ti]:
            if vi not in uvs:
                uvs[vi] = uv(verts[vi])
    edge_tris = defaultdict(list)
    for ti in mytris:
        a, b, c = tris[ti]
        for u, v in ((a, b), (b, c), (c, a)):
            edge_tris[(min(u, v), max(u, v))].append(ti)
    bnd_v = set()
    for e, ts in edge_tris.items():
        if len(ts) == 1:
            bnd_v.add(e[0]); bnd_v.add(e[1])

    NU, NV = 90, 22
    u0, u1 = 0.0, math.pi / 2
    v0, v1 = -1.21, -0.14
    # coverage: mark cells containing triangle VERTS (not bbox)
    grid_v = defaultdict(set)
    grid_b = defaultdict(set)
    for vi, (u, v) in uvs.items():
        i = min(NU - 1, max(0, int((u - u0) / (u1 - u0) * NU)))
        j = min(NV - 1, max(0, int((v - v0) / (v1 - v0) * NV)))
        grid_v[(i, j)].add(vi)
        if vi in bnd_v:
            grid_b[(i, j)].add(vi)
    print(f"f{FID}: {len(mytris)} tris, {len(uvs)} verts, "
          f"{len(bnd_v)} bnd; grid {NU}x{NV}")
    print("map: '#'=cell with verts (all bnd), '+'=mixed, "
          "'o'=cell with interior verts only, '.'=empty")
    print("     u 0° -> 90° (right), v -1.21 (bottom) -> -0.14 (top)")
    for j in range(NV - 1, -1, -1):
        row = []
        for i in range(NU):
            vs_ = grid_v.get((i, j), set())
            bs_ = grid_b.get((i, j), set())
            if not vs_:
                row.append(".")
            elif vs_ == bs_:
                row.append("#")
            else:
                row.append("+")
        print(f"  {''.join(row)}")


if __name__ == "__main__":
    main()
