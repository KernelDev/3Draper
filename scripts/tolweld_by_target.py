#!/usr/bin/env python3
"""Session-56 refined: TOLWELD[merge] attribution BY TARGET FACE (q-side,
which survives in the final mesh) + distance histogram per face.
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

obj_file = Path(sys.argv[1])
log_path = Path(sys.argv[2])

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

# position -> faces (exact rounded match at 1e-4)
pos_faces = defaultdict(set)
for ti, (a, b, c) in enumerate(tris):
    fid = fid_of_tri.get(ti, -1)
    for vi in (a, b, c):
        p = (round(verts[vi][0], 4), round(verts[vi][1], 4), round(verts[vi][2], 4))
        pos_faces[p].add(fid)

def faces_at(q, eps=5e-5):
    # log prints q with 5 decimals; round to 4 and look up exact
    for e in (0.0, 5e-5, -5e-5):
        key = (round(q[0] + e, 4), round(q[1] + e, 4), round(q[2] + e, 4))
        if key in pos_faces:
            return pos_faces[key]
    # fallback: nearest within 1e-3
    out = set()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                key = (round(q[0] + dx * 1e-3, 4), round(q[1] + dy * 1e-3, 4), round(q[2] + dz * 1e-3, 4))
                out |= pos_faces.get(key, set())
    return out

pat = re.compile(r"TOLWELD\S*\] ->(\d+) d=([0-9.e+-]+) p=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\) q=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)")
tgt_dist = defaultdict(list)
unattributed = 0
total = 0
for line in log_path.read_text(errors="replace").splitlines():
    m = pat.search(line)
    if not m:
        continue
    total += 1
    d = float(m.group(2))
    q = (float(m.group(6)), float(m.group(7)), float(m.group(8)))
    fs = faces_at(q)
    if not fs:
        unattributed += 1
        continue
    for f in fs:
        tgt_dist[f].append(d)

print(f"total={total}, unattributed q={unattributed}")
print(f"\nper-target-face: events, distance summary")
rows = []
for f, ds in tgt_dist.items():
    ds.sort()
    rows.append((len(ds), f, ds[0], ds[len(ds) // 2], ds[-1]))
rows.sort(reverse=True)
for n, f, lo, med, hi in rows[:20]:
    print(f"   face {f}: {n} events, d [{lo:.2e} .. med {med:.2e} .. {hi:.2e}]")

# f198 family specifically
print("\nf198 family as merge TARGET:")
for f in (198, 200, 202, 204, 206, 199, 226):
    ds = tgt_dist.get(f, [])
    if ds:
        ds.sort()
        print(f"   face {f}: {len(ds)} events, d [{ds[0]:.2e} .. med {ds[len(ds)//2]:.2e} .. {ds[-1]:.2e}]")
    else:
        print(f"   face {f}: 0 events")

# distance bucket 0.00698 cross-tab
print("\nevents at d in [0.00690, 0.00700] by target face:")
for f, ds in tgt_dist.items():
    n = sum(1 for d in ds if 0.00690 <= d <= 0.00700)
    if n > 50:
        print(f"   face {f}: {n}")
