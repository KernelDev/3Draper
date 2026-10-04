#!/usr/bin/env python3
"""s73: for pairs ADDED between two runs (per BREP), classify by 3D height h
vs the weld merge tolerance (drill ~0.0153): sub-tol weld artifacts vs real.
"""
import re
import sys
from collections import Counter

MERGE_TOL = 0.0153

def load(path, brep):
    out = {}
    pat = re.compile(
        r"\[(FOLD-\w+|WINDING-\w+)\] brep_idx=\d+ \S+ \(BREP#(\d+)\) BREP#(\d+) "
        r"ang=([\d.]+) faces=\((\d+),(\d+)\).*?areas=\(([\d.e+-]+),([\d.e+-]+)\) "
        r"h=\(([\d.e+-]+),([\d.e+-]+)\)")
    with open(path) as f:
        for line in f:
            m = pat.search(line)
            if not m:
                continue
            kind, _, b2, ang, f1, f2, a1, a2, h1, h2 = m.groups()
            if b2 != brep:
                continue
            key = (kind, int(f1), int(f2))
            # keep the max-h instance per key
            hmax = max(float(h1), float(h2))
            if key not in out or hmax > out[key]:
                out[key] = hmax
    return out

base, new, brep = sys.argv[1], sys.argv[2], sys.argv[3]
a = load(base, brep)
b = load(new, brep)
added = {k: v for k, v in b.items() if k not in a}
removed = {k: v for k, v in a.items() if k not in b}
print(f"BREP#{brep}: added={len(added)} removed={len(removed)}")
sub = [k for k, v in added.items() if v < MERGE_TOL]
print(f"ADDED with max(h) < merge_tol {MERGE_TOL}: {len(sub)}/{len(added)}")
for k, v in sorted(added.items(), key=lambda kv: kv[1]):
    tag = "SUBTOL" if v < MERGE_TOL else "REAL"
    print(f"  {tag} h={v:.4f} {k[0]} faces=({k[1]},{k[2]})")
sub_r = [k for k, v in removed.items() if v < MERGE_TOL]
print(f"REMOVED with max(h) < merge_tol: {len(sub_r)}/{len(removed)}")
