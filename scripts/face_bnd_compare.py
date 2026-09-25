#!/usr/bin/env python3
"""Session-56: per-face boundary-edge (usage-1) census on stage meshes —
measures the fill's bnd win on the f198 family under the guard.
"""
import sys
from collections import defaultdict
from pathlib import Path


def load(obj_file):
    verts, tris = [], []
    for line in obj_file.read_text().splitlines():
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
            tris.append((a, b, c))
    fid_of_tri = {}
    for line in obj_file.with_suffix(".fmap").read_text().splitlines():
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of_tri[int(ti)] = int(fid)
    return verts, tris, fid_of_tri


def face_bnd(tris, fid_of_tri, face_ids):
    edge_users = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for e in ((a, b), (b, c), (c, a)):
            edge_users[(min(e), max(e))].append(ti)
    out = {}
    for fid in face_ids:
        n = 0
        for (e, tl) in edge_users.items():
            if len(tl) == 1 and fid_of_tri.get(tl[0]) == fid:
                n += 1
        out[fid] = n
    # whole-mesh totals
    total_bnd = sum(1 for tl in edge_users.values() if len(tl) == 1)
    total_nm = sum(1 for tl in edge_users.values() if len(tl) >= 3)
    return out, total_bnd, total_nm


dirs = [
    ("default-no-fill", Path(sys.argv[1])),
    ("guard-no-fill", Path(sys.argv[2])),
    ("guard-dirty-fill", Path(sys.argv[3])),
]
face_ids = [198, 200, 202, 204, 206]

print(f"{'config':<20}" + "".join(f"{('f'+str(f)):>7}" for f in face_ids) + f"{'HM-bnd':>9}{'HM-nm':>8}")
for name, d in dirs:
    obj = d / "p1_d-after-winding.obj"
    verts, tris, fid = load(obj)
    per, tb, tn = face_bnd(tris, fid, face_ids)
    print(f"{name:<20}" + "".join(f"{per[f]:>7}" for f in face_ids) + f"{tb:>9}{tn:>8}")
