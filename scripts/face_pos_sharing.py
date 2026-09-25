#!/usr/bin/env python3
"""Session-56: which OTHER faces share 3D vertex positions with the f198
family interior vertices (merged instance mesh, no fill)?

Hypothesis: the STEP models the same fillet region twice (analytic Torus +
NURBS), and the position-dedup at merge welds their triangulations
together, creating the same-face u3/slivers on the torus side.
"""
import sys
from collections import defaultdict
from pathlib import Path

obj_file = Path(sys.argv[1])
target = int(sys.argv[2]) if len(sys.argv) > 2 else 198

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

# position -> set of faces using a vertex there (rounded to 1e-6)
pos_faces = defaultdict(set)
tri_face = []
for ti, (a, b, c) in enumerate(tris):
    fid = fid_of_tri.get(ti, -1)
    tri_face.append(fid)
    for vi in (a, b, c):
        p = (round(verts[vi][0], 6), round(verts[vi][1], 6), round(verts[vi][2], 6))
        pos_faces[p].add(fid)

# target face's vertices
tgt_pos = set()
for ti, (a, b, c) in enumerate(tris):
    if tri_face[ti] == target:
        for vi in (a, b, c):
            tgt_pos.add((round(verts[vi][0], 6), round(verts[vi][1], 6), round(verts[vi][2], 6)))

# how many f198 positions are shared with each other face?
share = defaultdict(int)
for p in tgt_pos:
    for f in pos_faces[p]:
        if f != target:
            share[f] += 1

print(f"face {target}: {len(tgt_pos)} distinct vertex positions")
print("positions shared with other faces (face: count):")
for f, n in sorted(share.items(), key=lambda kv: -kv[1])[:15]:
    print(f"   face {f}: {n}")

# do the u3 edges of the target sit on shared positions?
edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        edge_tris[(min(e), max(e))].append(ti)
u3 = [(e, tl) for e, tl in edge_tris.items()
      if len(tl) >= 3 and all(tri_face[t] == target for t in tl)]
print(f"\nsame-face u3 edges on face {target}: {len(u3)}")
shared_ct = 0
for (e, tl) in u3:
    pa = (round(verts[e[0]][0], 6), round(verts[e[0]][1], 6), round(verts[e[0]][2], 6))
    pb = (round(verts[e[1]][0], 6), round(verts[e[1]][1], 6), round(verts[e[1]][2], 6))
    if len(pos_faces[pa]) > 1 or len(pos_faces[pb]) > 1:
        shared_ct += 1
print(f"  of which incident to a position shared with another face: {shared_ct}")
