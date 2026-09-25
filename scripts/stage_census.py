#!/usr/bin/env python3
"""Session-56: per-stage census of the f198 family (drill HM, complement ON).

For each DRAPPER_DUMP_STAGE_OBJS dump, report per target face:
  verts, tris, boundary edges (usage-1), same-face usage>=3 edges,
  and micro-sliver triangles (area < 1e-4 mm^2).
Then attribute the appearance of same-face usage>=3 edges to a stage.
"""
import sys
from collections import defaultdict
from pathlib import Path

stages_dir = Path(sys.argv[1])
face_ids = set(int(x) for x in sys.argv[2:]) or {198, 200, 202, 204, 206}


def load(obj_file):
    verts, tris = [], []
    for line in obj_file.read_text().splitlines():
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
            tris.append((a, b, c))
    fmap = obj_file.with_suffix(".fmap")
    fid_of_tri = {}
    for line in fmap.read_text().splitlines():
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of_tri[int(ti)] = int(fid)
    return verts, tris, fid_of_tri


def tri_area(verts, t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = verts[t[0]], verts[t[1]], verts[t[2]]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    cx_ = uy * vz - uz * vy
    cy_ = uz * vx - ux * vz
    cz_ = ux * vy - uy * vx
    return 0.5 * (cx_ * cx_ + cy_ * cy_ + cz_ * cz_) ** 0.5


def census(verts, tris, fid_of_tri):
    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for e in ((a, b), (b, c), (c, a)):
            key = (min(e), max(e))
            edge_tris[key].append(ti)
    out = {}
    for fid in face_ids:
        ftris = [ti for ti, f in fid_of_tri.items() if f == fid]
        fv = set()
        for ti in ftris:
            fv.update(tris[ti])
        u3 = 0
        for (e, tl) in edge_tris.items():
            if len(tl) >= 3 and all(fid_of_tri.get(t) == fid for t in tl):
                u3 += 1
        slivers = sum(1 for ti in ftris if tri_area(verts, tris[ti]) < 1e-4)
        out[fid] = (len(fv), len(ftris), u3, slivers)
    return out, edge_tris


# ── Pass 1 (final) stages in pipeline order ──────────────────────────
order = ["d-after-merge", "d-after-weld", "d-after-tj", "d-after-gapfill", "d-after-winding"]
print(f"{'stage':<18}" + "".join(f"| f{f}: v/t/u3/sliver " for f in sorted(face_ids)))
results = {}
for st in order:
    obj = stages_dir / f"p1_{st}.obj"
    if not obj.exists():
        continue
    verts, tris, fid = load(obj)
    cen, _ = census(verts, tris, fid)
    results[st] = cen
    row = f"{st:<18}"
    for f in sorted(face_ids):
        v, t, u3, sl = cen[f]
        row += f"| {v:4d}/{t:4d}/{u3:3d}/{sl:4d}      "
    print(row)

# Total row
print(f"{'TOTAL u3':<18}" + "".join(f"| {results[st][f][2]:^17d}" for st in results for f in [0]) if False else "")
for st, cen in results.items():
    print(f"  {st}: total same-face u3 = {sum(c[2] for c in cen.values())}, "
          f"total slivers = {sum(c[3] for c in cen.values())}")
