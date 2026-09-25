#!/usr/bin/env python3
"""Session-56: are the f198-family tori CO-LOCATED in 3D (duplicated STEP
faces)? Compute per-face 3D bbox + centroid from a final/stage OBJ+fmap.
Also: for one family member, how many of its vertices are shared with the
other family members?
"""
import sys
from collections import defaultdict
from pathlib import Path

obj_file = Path(sys.argv[1])
face_ids = [int(x) for x in sys.argv[2:]] or [198, 200, 202, 204, 206]

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

fv = defaultdict(set)
for ti, (a, b, c) in enumerate(tris):
    fid = fid_of_tri.get(ti, -1)
    fv[fid].update((a, b, c))

print(f"mesh: {len(verts)} verts, {len(tris)} tris")
sets = {}
for fid in face_ids:
    vs = fv.get(fid, set())
    if not vs:
        print(f"face {fid}: (absent)")
        continue
    xs = [verts[v][0] for v in vs]
    ys = [verts[v][1] for v in vs]
    zs = [verts[v][2] for v in vs]
    cx, cy, cz = sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)
    sets[fid] = vs
    print(f"face {fid}: n={len(vs)} bbox=[{min(xs):.4f},{min(ys):.4f},{min(zs):.4f}]-[{max(xs):.4f},{max(ys):.4f},{max(zs):.4f}] centroid=({cx:.4f},{cy:.4f},{cz:.4f})")

# pairwise vertex-position overlap (by rounded position)
if len(sets) >= 2:
    print("\nposition overlap matrix (shared rounded positions / row face size):")
    pos = {fid: {(round(verts[v][0], 6), round(verts[v][1], 6), round(verts[v][2], 6)) for v in vs}
           for fid, vs in sets.items()}
    ids = sorted(sets)
    hdr = "      " + "".join(f"{f:>10d}" for f in ids)
    print(hdr)
    for a in ids:
        row = f"f{a:<4d} "
        for b in ids:
            row += f"{len(pos[a] & pos[b]):>10d}"
        print(row)
