#!/usr/bin/env python3
"""s73: diff fold-pair sets (per BREP) between two fold_face_probe runs."""
import re
import sys
from collections import Counter

def load(path, brep_filter=None):
    pairs = Counter()
    pat = re.compile(r"\[(FOLD-\w+|WINDING-\w+)\] brep_idx=\d+ \S+ \(BREP#(\d+)\) BREP#(\d+) ang=([\d.]+) faces=\((\d+),(\d+)\)")
    with open(path) as f:
        for line in f:
            m = pat.search(line)
            if not m:
                continue
            kind, b1, b2, ang, f1, f2 = m.groups()
            if brep_filter and b2 != brep_filter:
                continue
            key = (kind, int(f1), int(f2))
            pairs[key] += 1
    return pairs

base_path, new_path = sys.argv[1], sys.argv[2]
brep = sys.argv[3] if len(sys.argv) > 3 else None
a = load(base_path, brep)
b = load(new_path, brep)
print(f"baseline total: {sum(a.values())}  fixed total: {sum(b.values())}  delta: {sum(b.values())-sum(a.values()):+d}")
removed = a - b
added = b - a
print(f"\nREMOVED pairs ({sum(removed.values())}):")
for (kind, f1, f2), cnt in sorted(removed.items(), key=lambda kv: -kv[1]):
    print(f"  {kind:20s} faces=({f1},{f2}) x{cnt}")
print(f"\nADDED pairs ({sum(added.values())}):")
for (kind, f1, f2), cnt in sorted(added.items(), key=lambda kv: -kv[1]):
    print(f"  {kind:20s} faces=({f1},{f2}) x{cnt}")
