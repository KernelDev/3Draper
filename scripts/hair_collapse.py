#!/usr/bin/env python3
"""session-61: hair-pattern detection & collapse validation on GEAR f18/f20.

Pattern per tooth (from trace_path):
  base_A --spike(single edge)--> top --flank(55pts)--> base_B
  --jump-chord--> top --flank-retrace(bit-exact)--> base_B --valley--> ...

Collapse: drop [jump-chord + retrace] => simple tooth outline.
Validate: (1) retrace is bit-exact; (2) collapsed ring is simple
(no self-intersections); (3) vertex budget shrink.
"""
import sys

def load(path):
    ring, lat = [], []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) != 3:
                continue
            kind, u, v = parts[0], float(parts[1]), float(parts[2])
            if kind == "ring":
                ring.append((u, v))
            elif kind == "lat":
                lat.append((u, v))
    return ring, lat

def seg_intersect(p1, p2, p3, p4):
    def d(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = d(p3, p4, p1), d(p3, p4, p2)
    d3, d4 = d(p1, p2, p3), d(p1, p2, p4)
    return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
           ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))

def self_ints(poly):
    n = len(poly)
    hits = []
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            if seg_intersect(a1, a2, b1, b2):
                hits.append((i, j))
    return hits

def find_hairs(ring):
    """Find bit-exact retrace runs: ring[s..s+L-1] == ring[s+L+1..s+2L]
    separated by ONE jump vertex (the shared turnaround apex pattern:
    ... top, flank..., base_B, top, flank..., base_B ...)."""
    n = len(ring)
    pos = {}
    for i, p in enumerate(ring):
        pos.setdefault(p, []).append(i)
    hairs = []
    # For each pair of equal points (i, j), check if ring[i+1..k] == ring[j+1..j+k-i]
    for p, idxs in pos.items():
        if len(idxs) < 2:
            continue
        for a in range(len(idxs)):
            for b in range(a + 1, len(idxs)):
                i, j = idxs[a], idxs[b]
                if j == i:
                    continue
                # find max L with ring[i+t] == ring[j+t] for t in 0..L-1 (cyclic)
                L = 0
                while L < n and ring[(i + L) % n] == ring[(j + L) % n]:
                    L += 1
                if L >= 8:  # meaningful retrace run
                    hairs.append((i, j, L))
    return hairs

for fid in (18, 20):
    ring, lat = load(f"/tmp/ring_brep16033_f{fid}_Cone.tsv")
    print(f"=== f{fid}: {len(ring)} pts")
    si = self_ints(ring)
    print(f"  self-intersections: {len(si)}")
    hairs = find_hairs(ring)
    hairs.sort(key=lambda h: -h[2])
    print(f"  retrace runs (L>=8): {len(hairs)}")
    for i, j, L in hairs[:6]:
        print(f"    start {i}↔{j}, L={L} (ring[{i}]={ring[i]}, ring[{j}]={ring[j]})")
    # dedupe overlapping detections: keep maximal runs
    if hairs:
        # collapse: remove the SECOND copy of each maximal retrace run
        drop = set()
        for i, j, L in hairs:
            for t in range(L):
                drop.add((j + t) % len(ring))
        # also the jump chord vertex that precedes run j? The chord is
        # ring[j-1] -> ring[j]; the segment (j-1,j) is the jump itself.
        collapsed = [p for k, p in enumerate(ring) if k not in drop]
        print(f"  collapsed ring: {len(collapsed)} pts (dropped {len(drop)})")
        si2 = self_ints(collapsed)
        print(f"  collapsed self-intersections: {len(si2)}")
        if si2:
            for i, j in si2[:10]:
                print(f"    seg {i} x seg {j}: {collapsed[i]}-{collapsed[(i+1)%len(collapsed)]} x {collapsed[j]}-{collapsed[(j+1)%len(collapsed)]}")
    print()
