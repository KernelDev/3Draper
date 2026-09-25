#!/usr/bin/env python3
"""Session-56: per-face >170° pair census with side-test robustness.

For two meshes (default vs guard-ON, same instance), compute per face:
  - pairs (usage-2 edges, dihedral > 170°), classified by
    topo-consistency and apex-side, PLUS the side-test margin
    (|s0||s1| vs |s0.s1| — near-collinear apexes make the side sign
    unreliable: margin = |dot| / (|s0||s1|) close to 0 = noise).
  - boundary edges, same-face u3 edges, sliver counts.

Usage: pairs_census.py <obj> <fmap> [--top N]
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

obj = Path(sys.argv[1])
fmap = obj.with_suffix(".fmap")

verts, tris = [], []
for line in obj.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))
fid_of_tri = {}
for line in fmap.read_text().splitlines():
    if line.startswith("t "):
        _, ti, fid = line.split()
        fid_of_tri[int(ti)] = int(fid)


def sub(p, q):
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def cross(p, q):
    return (p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0])


def dot(p, q):
    return p[0] * q[0] + p[1] * q[1] + p[2] * q[2]


def norm(p):
    return math.sqrt(dot(p, p))


def tri_normal(t):
    n = cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))
    l = norm(n)
    return (n[0] / l, n[1] / l, n[2] / l) if l > 1e-30 else None


normals = [tri_normal(t) for t in tris]

edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        edge_tris[(min(e), max(e))].append((ti, e[0], e[1]))

per_face = defaultdict(lambda: defaultdict(int))
angle_bins = defaultdict(lambda: defaultdict(int))  # face -> angle bin -> n
weak_side = defaultdict(int)  # pairs with unreliable side sign
for (e, tl) in edge_tris.items():
    if len(tl) != 2:
        continue
    (t0, a0, b0), (t1, a1, b1) = tl
    n0, n1 = normals[t0], normals[t1]
    if not n0 or not n1:
        continue
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(n0, n1)))))
    if ang <= 170.0:
        continue
    fid = fid_of_tri.get(t0, -1)
    topo_consistent = (a0 == b1) and (b0 == a1)
    # apex side test with margin
    a, b = verts[e[0]], verts[e[1]]
    ed = sub(b, a)

    def apex_side(ti, e0, e1):
        for v in tris[ti]:
            if v != e0 and v != e1:
                return cross(ed, sub(verts[v], a)), verts[v]
        return (0, 0, 0), verts[tris[ti][0]]

    s0, c0 = apex_side(t0, e[0], e[1])
    s1, c1 = apex_side(t1, e[0], e[1])
    d = dot(s0, s1)
    margin = abs(d) / (norm(s0) * norm(s1)) if norm(s0) > 0 and norm(s1) > 0 else 0.0
    same_side = d > 0
    cls = ("FOLD-OVER" if same_side else "CURVED-180") if topo_consistent else (
        "DOUBLE-BROKEN" if same_side else "WINDING-FLIP")
    if margin < 0.05:
        cls += "(weak-side)"
        weak_side[fid] += 1
    per_face[fid][cls] += 1
    angle_bins[fid][int(ang // 1) * 1] += 1

top = int(sys.argv[sys.argv.index("--top") + 1]) if "--top" in sys.argv else 15
rows = sorted(per_face.items(), key=lambda kv: -sum(kv[1].values()))
print(f"{'face':>6} {'total':>7}  classes")
for fid, cls in rows[:top]:
    tot = sum(cls.values())
    parts = " ".join(f"{k}={v}" for k, v in sorted(cls.items(), key=lambda kv: -kv[1]))
    print(f"{fid:>6} {tot:>7}  {parts}")

print("\nangle distribution (top faces, 1° bins >= 179):")
for fid, _ in rows[:6]:
    hi = {a: n for a, n in sorted(angle_bins[fid].items()) if a >= 178}
    lo = sum(n for a, n in angle_bins[fid].items() if a < 178)
    print(f"  face {fid}: <178°: {lo}   {hi}")

print(f"\ntotal pairs: {sum(sum(c.values()) for c in per_face.values())}")
print(f"weak-side pairs: {sum(weak_side.values())}")
