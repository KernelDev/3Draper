#!/usr/bin/env python3
"""Session-56: per-face attribution of TOLWELD[merge] events (the
instance-level merge dedup welds) against the final OFF mesh.

Answers: which faces lose vertices to tolerance merges, at what
distances — and is the f198 family the epicenter?
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

# face -> set of final vertex positions (rounded 1e-4 grid for lookup)
cell = 0.05
grid = defaultdict(set)
for ti, (a, b, c) in enumerate(tris):
    fid = fid_of_tri.get(ti, -1)
    for vi in (a, b, c):
        x, y, z = verts[vi]
        grid[(int(x / cell), int(y / cell), int(z / cell))].add((round(x, 4), round(y, 4), round(z, 4), fid))

def faces_at(px, py, pz, eps=1e-3):
    out = set()
    cx, cy, cz = int(px / cell), int(py / cell), int(pz / cell)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for (x, y, z, fid) in grid.get((cx + dx, cy + dy, cz + dz), ()):
                    if abs(x - px) <= eps and abs(y - py) <= eps and abs(z - pz) <= eps:
                        out.add(fid)
    return out

pat = re.compile(r"TOLWELD\S*\] ->(\d+) d=([0-9.e+-]+) p=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\) q=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)")
events = []
for line in log_path.read_text(errors="replace").splitlines():
    m = pat.search(line)
    if m:
        events.append((float(m.group(2)),
                       (float(m.group(3)), float(m.group(4)), float(m.group(5))),
                       (float(m.group(6)), float(m.group(7)), float(m.group(8)))))

print(f"total TOLWELD[merge] events: {len(events)}")
import statistics
ds = sorted(e[0] for e in events)
print(f"distances: min={ds[0]:.2e} p25={ds[len(ds)//4]:.2e} med={ds[len(ds)//2]:.2e} p75={ds[3*len(ds)//4]:.2e} max={ds[-1]:.2e}")

# distance histogram buckets
hist = defaultdict(int)
for d, _, _ in events:
    b = round(d, 5)
    hist[b] += 1
print("top distance buckets:")
for d, n in sorted(hist.items(), key=lambda kv: -kv[1])[:10]:
    print(f"   d={d:.5f}: {n}")

# per-face attribution (p-side, the merged-away vertex)
face_events = defaultdict(int)
face_events_q = defaultdict(int)
for d, p, q in events:
    fs = faces_at(*p)
    for f in fs:
        face_events[f] += 1
    fsq = faces_at(*q)
    for f in fsq:
        face_events_q[f] += 1

print("\ntop faces by merged-away (p) events:")
for f, n in sorted(face_events.items(), key=lambda kv: -kv[1])[:15]:
    print(f"   face {f}: {n}")
print("top faces by merge-target (q) events:")
for f, n in sorted(face_events_q.items(), key=lambda kv: -kv[1])[:15]:
    print(f"   face {f}: {n}")
