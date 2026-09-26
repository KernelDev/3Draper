#!/usr/bin/env python3
"""session-61: numeric tooth-structure analysis of GEAR f18/f20 rings."""
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

def analyze(fid):
    ring, lat = load(f"/tmp/ring_brep16033_f{fid}_Cone.tsv")
    n = len(ring)
    us = [p[0] for p in ring]; vs = [p[1] for p in ring]
    u_lo, u_hi, v_lo, v_hi = min(us), max(us), min(vs), max(vs)
    print(f"=== f{fid}: n={n} u[{u_lo:.6f},{u_hi:.6f}] v[{v_lo:.6f},{v_hi:.6f}]")

    # find the bottom run (contiguous points at v_lo)
    bot = [i for i, p in enumerate(ring) if abs(p[1] - v_lo) < 1e-9]
    print(f"bottom-run points (v=v_lo): {len(bot)}, idx range [{min(bot)}, {max(bot)}]")

    # local extrema of v along the ring (after bottom run) = tooth tops/valleys
    # rotate so we start after the bottom run
    start = max(bot) + 1
    seq = [(ring[(start + k) % n]) for k in range(n)]
    # v-extrema: split by direction changes with tolerance
    tol = (v_hi - v_lo) * 1e-6
    extrema = []
    direction = 0
    for k in range(1, len(seq)):
        dv = seq[k][1] - seq[k-1][1]
        nd = 1 if dv > tol else (-1 if dv < -tol else 0)
        if nd != 0 and direction != 0 and nd != direction:
            extrema.append((k-1, seq[k-1]))
            direction = nd
        elif nd != 0:
            direction = nd
    print(f"v-extrema along top wave: {len(extrema)}")
    # classify extrema
    tops = [e for e in extrema if abs(e[1][1] - v_hi) < 1e-9]
    valleys = [e for e in extrema if e[1][1] < v_hi - 1e-9]
    vvals = sorted(set(round(e[1][1], 6) for e in extrema))
    print(f"extreme v-levels (uniq): {vvals[:15]}")
    print(f"tooth-top count (v=v_hi): {len(tops)}")
    if valleys:
        vv = sorted(set(round(e[1][1], 6) for e in valleys))
        print(f"valley levels: {vv[:15]}")
    # u-monoтohonic segments: does the top wave always advance in u?
    back = 0
    fwd = 0
    for k in range(1, len(seq)):
        du = seq[k][0] - seq[k-1][0]
        if du < -1e-9:
            back += 1
        elif du > 1e-9:
            fwd += 1
    print(f"u-advancing edges: {fwd}, u-backward edges: {back}")
    # segment lengths: max |dv| per edge (tooth flank steepness)
    dvs = sorted((abs(seq[k][1] - seq[k-1][1]) for k in range(1, len(seq))), reverse=True)
    print(f"largest edge |dv|: {dvs[:8]}")
    # points per tooth cycle: count between consecutive tops
    if len(tops) >= 2:
        idxs = [t[0] for t in tops]
        cyc = [b - a for a, b in zip(idxs, idxs[1:])]
        print(f"pts per tooth cycle (between tops): {cyc[:12]}")
        ducyc = [seq[b][0] - seq[a][0] for a, b in zip(idxs, idxs[1:])]
        print(f"u-width per tooth: {[f'{x:.4f}' for x in ducyc[:12]]}")
    print()

for fid in (18, 20):
    analyze(fid)
