#!/usr/bin/env python3
"""s80: anatomy of the class-B Cylinder faces (HM f148, HOUSING f145).

Questions:
1. What does the cylinder face mesh look like (fan? flat? how many tris)?
2. Are ALL its triangles in the plane z=3.51 (degenerate flat), or only
   the rim-adjacent ones?
3. Where is the fan pole (lens center?) and what are the rim vertices?
4. Compare with the Plane face rim-row.
"""
import sys
from collections import Counter, defaultdict

OBJ = "forensics/s80_objs/brep4_HOUSING_MIRROR (BREP#62542).obj"
FMAP = "forensics/s80_objs/brep4_HOUSING_MIRROR (BREP#62542).fmap"
CYL_FID = 148   # face id of the Cylinder
PLN_FID = 147   # face id of the Plane


def load():
    verts = []
    with open(OBJ) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
    fmap = {}
    with open(FMAP) as f:
        for line in f:
            if line.startswith("t "):
                _, ti, fid = line.split()
                fmap[int(ti)] = int(fid)
    tris = []
    with open(OBJ) as f:
        for line in f:
            if line.startswith("f "):
                a, b, c = line.split()[1:]
                tris.append(tuple(int(v) - 1 for v in (a, b, c)))
    return verts, tris, fmap


def area(v, t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = v[t[0]], v[t[1]], v[t[2]]
    ux, uy, uz = bx - ax, by - ay, bz - az
    wx, wy, wz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
    return 0.5 * (nx * nx + ny * ny + nz * nz) ** 0.5


def main():
    verts, tris, fmap = load()
    print(f"verts={len(verts)} tris={len(tris)}")

    cyl = [i for i in range(len(tris)) if fmap.get(i) == CYL_FID]
    pln = [i for i in range(len(tris)) if fmap.get(i) == PLN_FID]
    print(f"\ncylinder face {CYL_FID}: {len(cyl)} tris; plane face {PLN_FID}: {len(pln)} tris")

    # z stats of cylinder tris
    zs = Counter()
    flat = 0
    for ti in cyl:
        tri = tris[ti]
        tv = [verts[k] for k in tri]
        zmin = min(p[2] for p in tv)
        zmax = max(p[2] for p in tv)
        zs[(round(zmin, 3), round(zmax, 3))] += 1
        if abs(zmax - zmin) < 1e-9:
            flat += 1
    print(f"\ncylinder tris FLAT (dz=0): {flat}/{len(cyl)}")
    print("z-extent histogram (zmin,zmax -> count), top 12:")
    for (zmin, zmax), n in zs.most_common(12):
        print(f"  z [{zmin:.3f},{zmax:.3f}]: {n}")

    # the fan pole: vertex usage count on cylinder face
    cnt = Counter()
    for ti in cyl:
        for v in tris[ti]:
            cnt[v] += 1
    print("\ntop cylinder-face vertex usage (pole = most used):")
    for v, n in cnt.most_common(6):
        print(f"  v{v} used {n}x  xyz={verts[v]}")

    # which cylinder tris touch the pole
    pole = cnt.most_common(1)[0][0]
    pole_tris = [ti for ti in cyl if pole in tris[ti]]
    print(f"\npole v{pole} in {len(pole_tris)} cylinder tris")
    # sample a few, show their other vertices
    for ti in pole_tris[:5]:
        tri = tris[ti]
        others = [k for k in tri if k != pole]
        print(f"  tri#{ti}: pole + {others} xyz={[verts[k] for k in others]}")

    # cylinder tris NOT touching the pole: what are they?
    nonpole = [ti for ti in cyl if pole not in tris[ti]]
    print(f"\ncylinder tris NOT touching pole: {len(nonpole)}")
    znp = Counter()
    for ti in nonpole:
        tv = [verts[k] for k in tris[ti]]
        znp[(round(min(p[2] for p in tv), 3), round(max(p[2] for p in tv), 3))] += 1
    for (zmin, zmax), n in znp.most_common(8):
        print(f"  z [{zmin},{zmax}]: {n}")
    for ti in nonpole[:6]:
        tri = tris[ti]
        print(f"  tri#{tri}: {[verts[k] for k in tri]}")

    # plane face rim-row check: h distribution
    print(f"\nplane face {PLN_FID} sample tris:")
    for ti in pln[:4]:
        tri = tris[ti]
        print(f"  tri#{tri}: {[verts[k] for k in tri]}  area={area(verts, tri):.6f}")


if __name__ == "__main__":
    main()
