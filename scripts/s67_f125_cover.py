#!/usr/bin/env python3
"""s67 diag 12: UV coverage map of f125's triangles.

Maps the face's mesh verts to cylinder UV, rasterizes triangle
bboxes into a grid, prints an ASCII coverage map. The slit (missing
region) will show as uncovered cells.
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"
FID = 125
POS = (-2.54, 0.375, -4.349967143371)
AXIS = (0.0, 0.0, -1.0)


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

    def to_local(p):
        return (p[2] - 2.325, p[0], p[1] - 5.4)

    def uv(p):
        px, py, pz = to_local(p)
        rel = (px - POS[0], py - POS[1], pz - POS[2])
        v = sum(rel[k] * AXIS[k] for k in range(3))
        rad = tuple(rel[k] - v * AXIS[k] for k in range(3))
        return (math.atan2(rad[1], rad[0]), v)

    my_tris = [t for t in range(len(tris))
               if fid_of.get(t) == FID]
    print(f"f{FID}: {len(my_tris)} triangles")
    uvs = {}
    us, vs = [], []
    for ti in my_tris:
        for vi in tris[ti]:
            if vi not in uvs:
                uvs[vi] = uv(verts[vi])
                us.append(uvs[vi][0]); vs.append(uvs[vi][1])
    u0, u1 = min(us), max(us)
    v0, v1 = min(vs), max(vs)
    print(f"UV bbox: u[{math.degrees(u0):.2f},{math.degrees(u1):.2f}] "
          f"v[{v0:.4f},{v1:.4f}]")
    # rasterize: NU x NV cells
    NU, NV = 100, 44
    cover = [[0] * NV for _ in range(NU)]
    for ti in my_tris:
        ps = [uvs[v] for v in tris[ti]]
        cu = [p[0] for p in ps]; cvv = [p[1] for p in ps]
        # unwrap u within tri
        for i in range(1, 3):
            while cu[i] - cu[0] > math.pi:
                cu[i] -= 2 * math.pi
            while cu[i] - cu[0] < -math.pi:
                cu[i] += 2 * math.pi
        i0 = max(0, int((min(cu) - u0) / (u1 - u0) * (NU - 1)))
        i1 = min(NU - 1, int((max(cu) - u0) / (u1 - u0) * (NU - 1)))
        j0 = max(0, int((min(cvv) - v0) / (v1 - v0) * (NV - 1)))
        j1 = min(NV - 1, int((max(cvv) - v0) / (v1 - v0) * (NV - 1)))
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                cover[i][j] += 1
    print("coverage map (u → right/top, v ↓): "
          "'.'=0 tris, digits=tri-bbox count per cell")
    for j in range(NV - 1, -1, -1):
        row = "".join(
            "." if cover[i][j] == 0 else
            (str(min(cover[i][j], 9))) for i in range(NU))
        print(f"  {row}")


if __name__ == "__main__":
    main()
