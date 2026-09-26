#!/usr/bin/env python3
"""Session-58: per-face FACEFOLD FO before/after comparison.

Usage: fo_face_compare.py <baseline_scan.txt> <gb_scan.txt>
"""
import re
import sys
from collections import defaultdict

def load(path):
    # face_id -> list of (tris, folds, FO, ovTot) — one entry per merge
    out = defaultdict(list)
    pat = re.compile(
        r"FACEFOLD: face_id=(\d+) tris=(\d+) folds=(\d+) "
        r"\[FO=(\d+) ovTot=([0-9.e+-]+) ovMax=([0-9.e+-]+) \| INV=(\d+)\]")
    for line in open(path):
        m = pat.search(line)
        if m:
            fid = int(m.group(1))
            out[fid].append(tuple(int(m.group(i)) for i in (2, 3, 4, 7))
                            + (float(m.group(5)),))
    return out

base = load(sys.argv[1])
gb = load(sys.argv[2])

fids = sorted(set(base) | set(gb))
tot_b = tot_g = 0
print(f"{'face':>6} | {'base: tris/FO/INV/ovTot':>40} | {'grid-band: tris/FO/INV/ovTot':>40}")
changed = 0
for fid in fids:
    b = base.get(fid, [])
    g = gb.get(fid, [])
    fb = sum(x[1] for x in b); fg = sum(x[1] for x in g)
    tot_b += fb; tot_g += fg
    if not b and not g:
        continue
    # summarize: max FO per side
    def fmt(rows):
        if not rows:
            return "--"
        best = "|".join(f"t{x[0]} FO{x[1]} INV{x[2]} o{x[3]:.1e}" for x in rows[:2])
        return best
    if fmt(b) != fmt(g):
        changed += 1
        print(f"{fid:>6} | {fmt(b):>40} | {fmt(g):>40}")
print(f"\nfaces changed: {changed}")
print(f"FO total: baseline={tot_b}  grid-band={tot_g}  delta={tot_g - tot_b}")
