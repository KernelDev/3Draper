#!/usr/bin/env python3
"""s80: check whether f148's per-face mesh (post-fix) is still a flat fan
or now a proper curved cushion."""
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "forensics/s80_faceobjs/brep62542_f148_s58085_Cylinder.obj"
verts = []
tris = []
with open(path) as f:
    for line in f:
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:]
            tris.append(tuple(int(v) - 1 for v in (a, b, c)))

print(f"{path}: {len(verts)} verts, {len(tris)} tris")
# cylinder (local): axis? check spread per axis
for i, ax in enumerate("xyz"):
    vals = [v[i] for v in verts]
    print(f"  {ax}: [{min(vals):.4f}, {max(vals):.4f}] span={max(vals)-min(vals):.4f}")
# flat? all z (or which axis) constant?
# distance from cylinder: axis along local Y at (x0=?, z0=?), r=0.5
# find pole: most used vertex
from collections import Counter
cnt = Counter()
for t in tris:
    for v in t:
        cnt[v] += 1
pole, uses = cnt.most_common(1)[0]
print(f"  pole: v{pole} used {uses}x  xyz={verts[pole]}")
print(f"  pole usage fraction: {uses}/{len(tris)} tris")
# if not a fan, show vertex count histogram
print(f"  vertices used >10x: {sum(1 for v, n in cnt.items() if n > 10)}")
