#!/usr/bin/env python3
"""s69 diag 3: per-face bnd/nm census diff for drill SHAFT_SLEEVE
(BREP#32629) — find the faces behind the +6 bnd / +71 nm regression."""
from collections import defaultdict


def load(base):
    verts, tris = [], []
    for line in open(base + ".obj"):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(t) - 1 for t in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(base + ".fmap"):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    stype = {}
    for line in open(base + ".facemap"):
        if line.startswith("f "):
            parts = line.split()
            stype[int(parts[1])] = " ".join(parts[2:-2])
    return verts, tris, fid_of, stype


def census(base):
    verts, tris, fid_of, stype = load(base)
    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    face_stat = defaultdict(lambda: [0, 0])  # fid -> [bnd, nm]
    for e, ts in edge_tris.items():
        if len(ts) == 1:
            face_stat[fid_of.get(ts[0], -1)][0] += 1
        elif len(ts) > 2:
            for t in ts:
                face_stat[fid_of.get(t, -1)][1] += 1
    return face_stat, stype


OFF = "/home/z/my-project/scripts/s69_objs/brep2_SHAFT_SLEEVE (BREP#32629)"
ON = "/home/z/my-project/scripts/s69_objs_on/brep2_SHAFT_SLEEVE (BREP#32629)"
off_stat, stype = census(OFF)
on_stat, _ = census(ON)
fids = sorted(set(off_stat) | set(on_stat))
print(f"{'fid':>6} {'type':<14} {'off bnd':>8} {'on bnd':>8} {'off nm':>7} {'on nm':>7}")
for fid in fids:
    ob, onm = off_stat.get(fid, [0, 0])
    nb, nn = on_stat.get(fid, [0, 0])
    if ob != nb or onm != nn:
        print(f"{fid:>6} {stype.get(fid, '?'):<14} {ob:>8} {nb:>8} {onm:>7} {nn:>7}")
tot_off_b = sum(v[0] for v in off_stat.values())
tot_on_b = sum(v[0] for v in on_stat.values())
tot_off_n = sum(v[1] for v in off_stat.values())
tot_on_n = sum(v[1] for v in on_stat.values())
print(f"\nTOTAL: bnd {tot_off_b} → {tot_on_b} | nm {tot_off_n} → {tot_on_n}")
