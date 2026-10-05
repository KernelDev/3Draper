#!/usr/bin/env python3
"""s77: per-pair diff OFF vs THIN_STRIP_ZIPPER=1, all 5 drill BREPs —
which pairs improved, which unmasked, totals raw/REAL."""
import re
from collections import defaultdict
import os

BREPS = {0: "DRILL_SHAFT", 1: "GEAR", 2: "SHAFT_SLEEVE", 3: "HOUSING", 4: "HOUSING_MIRROR"}
PAT = re.compile(r"^\[(\w[\w-]*)\] brep_idx=(\d+) \S+ \(BREP#(\d+)\) BREP#\d+ ang=([\d.]+) faces=\((\d+),(\d+)\)")

def load(path):
    pairs = defaultdict(int)
    cls = defaultdict(int)
    n = 0
    for line in open(path):
        m = PAT.match(line)
        if not m:
            continue
        kind, _, _, ang, fa, fb = m.groups()
        key = (int(fa), int(fb))
        pairs[key] += 1
        cls[(key, kind)] += 1
        n += 1
    return pairs, cls, n

def main():
    tot_off = tot_on = 0
    for b in sorted(BREPS):
        po, co, no = load(f"forensics/s77_dumps/off_brep{b}.txt")
        pn, cn, nn = load(f"forensics/s77_dumps/on_brep{b}.txt")
        tot_off += no; tot_on += nn
        keys = set(po) | set(pn)
        deltas = []
        for k in sorted(keys):
            d = pn.get(k, 0) - po.get(k, 0)
            if d != 0:
                deltas.append((d, k, po.get(k, 0), pn.get(k, 0)))
        deltas.sort(key=lambda x: -abs(x[0]))
        print(f"\n== brep{b} {BREPS[b]}: raw pairs {no} → {nn} (Δ{nn-no:+d}), changed pairs: {len(deltas)}")
        for d, k, o, n_ in deltas[:12]:
            print(f"   ({k[0]},{k[1]}): {o} → {n_} (Δ{d:+d})")
    print(f"\nTOTAL raw: {tot_off} → {tot_on} (Δ{tot_on-tot_off:+d})")

if __name__ == "__main__":
    main()
