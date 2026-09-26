#!/usr/bin/env python3
"""session-61: anatomy of the GEAR cone-band rings (f18/f20).

Reads /tmp/ring_brep16033_f{18,20}_Cone.tsv (ring/hole/lat rows),
classifies each ring point onto the UV bbox frame (4 sides), finds
corners, and prints the chain structure like s60 did for SLEEVE
f93/f152 (top v=..., sides u=..., bottom-meander).
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

def anatomy(name, ring, lat):
    n = len(ring)
    us = [p[0] for p in ring]; vs = [p[1] for p in ring]
    u_lo, u_hi, v_lo, v_hi = min(us), max(us), min(vs), max(vs)
    print(f"== {name}: {n} ring pts, {len(lat)} lattice pts")
    print(f"   u=[{u_lo:.6f}, {u_hi:.6f}] span={u_hi-u_lo:.6f}")
    print(f"   v=[{v_lo:.6f}, {v_hi:.6f}] span={v_hi-v_lo:.6f}")
    if lat:
        lus = sorted(set(round(p[0], 9) for p in lat))
        lvs = sorted(set(round(p[1], 9) for p in lat))
        print(f"   lattice: {len(lus)} u-cols x {len(lvs)} v-rows")
        print(f"   lat u: {[f'{x:.6f}' for x in lus]}")
        print(f"   lat v: {[f'{x:.6f}' for x in lvs]}")
    # frame classification
    eps_u = (u_hi - u_lo) * 1e-9 + 1e-12
    eps_v = (v_hi - v_lo) * 1e-9 + 1e-12
    def side(p):
        s = []
        if abs(p[0] - u_lo) <= eps_u: s.append('L')
        if abs(p[0] - u_hi) <= eps_u: s.append('R')
        if abs(p[1] - v_lo) <= eps_v: s.append('B')
        if abs(p[1] - v_hi) <= eps_v: s.append('T')
        return ''.join(s)
    sides = [side(p) for p in ring]
    off = sum(1 for s in sides if not s)
    print(f"   on-frame: {n - off}/{n}, off-frame: {off}")
    # run-length structure of side classes
    runs = []
    cur, cnt = sides[0], 1
    for s in sides[1:]:
        if s == cur:
            cnt += 1
        else:
            runs.append((cur, cnt)); cur, cnt = s, 1
    if cur == sides[0] and len(runs) > 0:
        runs[0] = (runs[0][0], runs[0][1] + cnt)
    else:
        runs.append((cur, cnt))
    print(f"   side-class runs ({len(runs)}):")
    for i, (s, c) in enumerate(runs[:40]):
        print(f"     [{i}] {s or '-'}: {c}")
    if len(runs) > 40:
        print(f"     ... ({len(runs) - 40} more)")
    # v-histogram on left side to see meander
    lidx = [i for i, s in enumerate(sides) if 'L' in s]
    if lidx:
        lv = [ring[i][1] for i in lidx]
        uniq = sorted(set(round(x, 6) for x in lv))
        print(f"   L-side v values ({len(uniq)} uniq): {uniq[:20]}")
    # v-histogram overall
    vcnt = {}
    for p in ring:
        k = round(p[1], 6)
        vcnt[k] = vcnt.get(k, 0) + 1
    top_v = sorted(vcnt.items(), key=lambda kv: -kv[1])[:12]
    print(f"   top v-values by count: {top_v}")
    return u_lo, u_hi, v_lo, v_hi, sides

for fid in (18, 20):
    path = f"/tmp/ring_brep16033_f{fid}_Cone.tsv"
    ring, lat = load(path)
    anatomy(f"GEAR f{fid}", ring, lat)
    print()
