#!/usr/bin/env python3
"""s105 item-2: census of the CHORD (3-on-bnd) fans in the face-8 #2442
uv-audit dump — group by apex (the fixed vertex of each fan tri) and
classify which fan is the rim-zipper (12 REAL seed).

Reads POLY lines (bnd ring) + CHORD lines from the uv-audit dump and:
 1. builds the poly table (idx -> (u, v, xyz));
 2. for each CHORD tri, finds the "apex" = the vertex that repeats
    across the fan (the two other verts are consecutive on one arc);
 3. groups tris by apex, reports per-fan: apex uv, tri count, the
    pair-arc (rim / meridian-uPI / meridian-u2PI / pole), the u/v
    extent, and the max chord length (2D UV) — the fold-over material.
"""
import re
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "forensics/s104/uvaudit_step2442.txt"
poly_re = re.compile(
    r"POLY\s+(\d+) u=\s*([-\d.]+) v=\s*([-\d.]+) xyz=\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)"
)
chord_re = re.compile(
    r"CHORD tri#(\d+) verts=\((\d+),(\d+),(\d+)\) poly=\((\d+),(\d+),(\d+)\) "
    r"uv=\([-\d.]+,[-\d.]+\)\([-\d.]+,[-\d.]+\)\([-\d.]+,[-\d.]+\) cuts=\[\"(.*)\"\]"
)
chord_uv_re = re.compile(
    r"uv=\(([-\d.]+),([-\d.]+)\)\(([-\d.]+),([-\d.]+)\)\(([-\d.]+),([-\d.]+)\)"
)

poly = {}
chords = []
for line in open(path, errors="replace"):
    m = poly_re.match(line)
    if m:
        poly[int(m.group(1))] = (
            float(m.group(2)), float(m.group(3)),
            (float(m.group(4)), float(m.group(5)), float(m.group(6))),
        )
        continue
    m = chord_re.match(line)
    if m:
        uvm = chord_uv_re.search(line)
        uv3 = [(float(uvm.group(1)), float(uvm.group(2))),
               (float(uvm.group(3)), float(uvm.group(4))),
               (float(uvm.group(5)), float(uvm.group(6)))]
        chords.append({
            "tri": int(m.group(1)),
            "poly": (int(m.group(5)), int(m.group(6)), int(m.group(7))),
            "uv": uv3,
            "cuts": m.group(8),
        })

print(f"poly ring: {len(poly)} pts; CHORD tris: {len(chords)}")

# classify each bnd point: rim (v~1.5708) / uPI meridian / u2PI meridian /
# pole-arc (v~0.0334)
def arc_of(idx):
    u, v = poly[idx][0], poly[idx][1]
    if abs(v - 1.5708) < 0.01:
        return "RIM"
    if abs(u - 3.1416) < 0.02:
        return "uPI"
    if abs(u - 6.2832) < 0.02:
        return "u2PI"
    if v < 0.1:
        return "POLE"
    return f"other(u={u:.3f},v={v:.3f})"

# group by apex: the apex is the vertex whose arc differs from the other
# two (fan = pairs on ONE arc + fixed apex elsewhere); when all three on
# the same arc, use the middle poly as apex candidate marker.
groups = defaultdict(list)
for c in chords:
    arcs = [arc_of(p) for p in c["poly"]]
    c["arcs"] = arcs
    # the pair = two consecutive points on the same arc; apex = the third
    # try all assignments, prefer (i,j) pair with |poly_i - poly_j| small
    best = None
    for k in range(3):
        i, j = c["poly"][k], c["poly"][(k + 1) % 3]
        a = c["poly"][3 - k - ((k + 1) % 3 == 2 and 1 or 0)] if False else None
    # simpler: apex = the poly index NOT in the consecutive pair
    # consecutive pair on the ring: idx difference == 1 (ring order)
    p = c["poly"]
    pair_idx = None
    for k in range(3):
        i1, i2 = p[k], p[(k + 1) % 3]
        if abs(i1 - i2) == 1:
            pair_idx = (k, (k + 1) % 3)
            break
    if pair_idx is None:
        # all-same-arc slivers: apex = the middle by ring order
        apex = sorted(p)[1]
        pair = tuple(x for x in p if x != apex)[:2]
    else:
        apex = p[3 - pair_idx[0] - pair_idx[1]]
        pair = (p[pair_idx[0]], p[pair_idx[1]])
    c["apex"] = apex
    c["pair"] = pair
    groups[apex].append(c)

print(f"\nfan groups by apex: {len(groups)}")
for apex in sorted(groups, key=lambda a: -len(groups[a])):
    tris = groups[apex]
    au, av = poly[apex][0], poly[apex][1]
    # fan extent along the pair arc
    us = [c["uv"][0][0] for c in tris] + [c["uv"][1][0] for c in tris] + [c["uv"][2][0] for c in tris]
    vs = [c["uv"][0][1] for c in tris] + [c["uv"][1][1] for c in tris] + [c["uv"][2][1] for c in tris]
    # max 2D chord within a tri (longest edge)
    maxedge = 0.0
    for c in tris:
        pts = c["uv"]
        for a in range(3):
            b = (a + 1) % 3
            d = ((pts[a][0] - pts[b][0]) ** 2 + (pts[a][1] - pts[b][1]) ** 2) ** 0.5
            maxedge = max(maxedge, d)
    # pair-arc class
    arcs = defaultdict(int)
    for c in tris:
        arcs[arc_of(c["pair"][0])] += 1
    print(f"apex poly#{apex} (u={au:.4f}, v={av:.4f}) arc={arc_of(apex)}: "
          f"{len(tris)} tris, pair-arcs={dict(arcs)}, "
          f"extent u[{min(us):.4f},{max(us):.4f}] v[{min(vs):.4f},{max(vs):.4f}], "
          f"max-edge={maxedge:.4f} rad")
    # list the tri ids compactly
    print("   tris:", " ".join(str(c["tri"]) for c in tris[:12]), "..." if len(tris) > 12 else "")
