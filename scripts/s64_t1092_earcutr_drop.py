#!/usr/bin/env python3
"""s64 diag: reconstruct f29 earcutr I/O, find the dropped triangle.

Surface f29 (local): point_at(u,v) = (17+3 sin u, 40+v, 42+3 cos u).
The missing boundary edge = line#5733: (20,38,42)@(u=pi/2,v=-2) to
(20,40,42)@(u=pi/2,v=0).  Find: (a) which boundary UV indices form this
edge; (b) which earcutr triangles used it; (c) why the triangle was
dropped (position-degenerate: two verts with identical 3D position).
"""
import math
from collections import defaultdict

DUMP = "/tmp/s64_tri/tri_0000_Cylinder.txt"

def point_at(u, v):
    return (17.0 + 3.0 * math.sin(u), 40.0 + v, 42.0 + 3.0 * math.cos(u))

def main():
    bnd, interior, tris = [], [], []
    mode = None
    for line in open(DUMP):
        line = line.strip()
        if line.startswith("type="):
            print(line); continue
        if line == "boundary": mode = "b"; continue
        if line == "interior": mode = "i"; continue
        if line == "tris": mode = "t"; continue
        if not line: continue
        p = line.split()
        if p[0] == "b":
            bnd.append((float(p[1]), float(p[2])))
        elif p[0] == "i":
            interior.append((float(p[1]), float(p[2])))
        elif p[0] == "t":
            tris.append((int(p[1]), int(p[2]), int(p[3])))
    print(f"bnd={len(bnd)} interior={len(interior)} tris={len(tris)}")
    n_b = len(bnd)

    all_uv = bnd + interior
    pts3d = [point_at(u, v) for (u, v) in all_uv]

    # find the missing edge: consecutive bnd pair at u=pi/2, v in {-2, 0}
    target_a = (math.pi / 2, -2.0)
    target_b = (math.pi / 2, 0.0)
    ia = ib = None
    for i, (u, v) in enumerate(bnd):
        if abs(u - math.pi / 2) < 1e-9 and abs(v + 2.0) < 1e-9:
            ia = i
        if abs(u - math.pi / 2) < 1e-9 and abs(v) < 1e-9:
            ib = i
    print(f"\nline#5733 endpoints: bnd_idx v-2={ia} v0={ib}")
    print(f"  pos(ia) = {pts3d[ia] if ia is not None else None}")
    print(f"  pos(ib) = {pts3d[ib] if ib is not None else None}")

    # check: are ia and ib consecutive in the boundary loop?
    if ia is not None and ib is not None:
        diff = (ia - ib) % n_b
        print(f"  index distance (ia-ib) mod {n_b} = {diff}")

    # which triangles contain the edge (ia, ib)?
    if ia is not None and ib is not None:
        users = [t for t in tris if (ia in t and ib in t)]
        print(f"\ntriangles using edge ({ia},{ib}): {len(users)}")
        for t in users:
            third = [x for x in t if x != ia and x != ib][0]
            tp = pts3d[third]
            tu, tv = all_uv[third]
            kind = "bnd" if third < n_b else "int"
            print(f"  tri {t}: third={third} ({kind}) uv=({tu:.6f},{tv:.6f}) pos=({tp[0]:.6f},{tp[1]:.6f},{tp[2]:.6f})")
            # check for position duplicates of the third vertex
            dups = [j for j in range(len(pts3d)) if j != third
                    and abs(pts3d[j][0]-tp[0])<1e-9 and abs(pts3d[j][1]-tp[1])<1e-9
                    and abs(pts3d[j][2]-tp[2])<1e-9]
            if dups:
                for j in dups:
                    ju, jv = all_uv[j]
                    jk = "bnd" if j < n_b else "int"
                    print(f"    ^^^ third's 3D position DUPLICATED by idx {j} ({jk}) uv=({ju:.6f},{jv:.6f})")
            # is third's position == ia or ib's position?
            for name, ref in (("ia", ia), ("ib", ib)):
                if abs(pts3d[ref][0]-tp[0])<1e-9 and abs(pts3d[ref][1]-tp[1])<1e-9 and abs(pts3d[ref][2]-tp[2])<1e-9:
                    print(f"    ^^^ third's position == {name}!")

    # global: position-duplicate census across all_uv
    bypos = defaultdict(list)
    for i, p in enumerate(pts3d):
        bypos[(round(p[0], 9), round(p[1], 9), round(p[2], 9))].append(i)
    dups = {k: v for k, v in bypos.items() if len(v) > 1}
    print(f"\nposition-duplicate groups: {len(dups)}")
    for k, v in dups.items():
        kinds = [("b" if i < n_b else "i") + f"{i}" for i in v]
        print(f"  {k}: {kinds}  uvs={[(round(all_uv[i][0],6), round(all_uv[i][1],6)) for i in v]}")

    # dropped-triangle census: earcutr tris that are position-degenerate
    print("\ndegenerate (position-dup) triangles in earcutr output:")
    ndeg = 0
    for t in tris:
        ps = [pts3d[t[0]], pts3d[t[1]], pts3d[t[2]]]
        if (ps[0] == ps[1]) or (ps[1] == ps[2]) or (ps[0] == ps[2]):
            ndeg += 1
            if ndeg <= 6:
                print(f"  tri {t}: {[tuple(round(x,6) for x in p) for p in ps]}")
    print(f"total degenerate: {ndeg}")

if __name__ == "__main__":
    main()
