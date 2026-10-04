#!/usr/bin/env python3
"""s74: full census of one BREP's fold pairs by max(h) vs merge_tol.
Splits SUBTOL weld-artifacts (both apexes below the weld tolerance — the
geometry is physically thinner than what the global weld preserves) from
REAL folds. Groups by class and face-pair.
"""
import re
import sys
from collections import Counter

MERGE_TOL = 0.0153

pat = re.compile(
    r"\[(FOLD-\w+|WINDING-\w+|CURVED-\w+|DOUBLE-\w+)(?: TANGENT-EXEMPT)?\] "
    r"brep_idx=\d+ \S+ \(BREP#(\d+)\) BREP#\d+ ang=([\d.]+) "
    r"faces=\((\d+),(\d+)\) types=\(([^,)]+),([^)]+)\).*?"
    r"areas=\(([\d.e+-]+),([\d.e+-]+)\) h=\(([\d.e+-]+),([\d.e+-]+)\)")

path, brep = sys.argv[1], sys.argv[2]
rows = []
with open(path) as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        kind, b, ang, f1, f2, t1, t2, a1, a2, h1, h2 = m.groups()
        if b != brep:
            continue
        rows.append(dict(kind=kind, ang=float(ang), f1=int(f1), f2=int(f2),
                         t1=t1, t2=t2, a1=float(a1), a2=float(a2),
                         h1=float(h1), h2=float(h2)))

print(f"BREP#{brep}: {len(rows)} pairs")
sub = [r for r in rows if max(r["h1"], r["h2"]) < MERGE_TOL]
real = [r for r in rows if max(r["h1"], r["h2"]) >= MERGE_TOL]
print(f"SUBTOL (max h < {MERGE_TOL}): {len(sub)}")
print(f"REAL   (max h >= {MERGE_TOL}): {len(real)}")

print("\n-- SUBTOL by class:")
for k, v in Counter(r["kind"] for r in sub).most_common():
    print(f"   {k}: {v}")
print("-- SUBTOL by face-pair (types):")
for k, v in Counter((r["f1"], r["f2"], r["t1"][:12], r["t2"][:12]) for r in sub).most_common():
    print(f"   faces=({k[0]},{k[1]}) ({k[2]},{k[3]}): {v}")

print("\n-- REAL by class:")
for k, v in Counter(r["kind"] for r in real).most_common():
    print(f"   {k}: {v}")
print("-- REAL by face-pair (types) top-20:")
for k, v in Counter((r["f1"], r["f2"], r["t1"][:12], r["t2"][:12]) for r in real).most_common(20):
    print(f"   faces=({k[0]},{k[1]}) ({k[2]},{k[3]}): {v}")

print("\n-- SUBTOL detail (h sorted):")
for r in sorted(sub, key=lambda r: max(r["h1"], r["h2"])):
    print(f"   h=({r['h1']:.4f},{r['h2']:.4f}) ang={r['ang']:.2f} "
          f"faces=({r['f1']},{r['f2']}) ({r['t1'][:14]},{r['t2'][:14]}) "
          f"areas=({r['a1']:.5f},{r['a2']:.5f})")

# h histogram of REAL pairs
print("\n-- REAL h histogram:")
bins = Counter()
for r in real:
    h = max(r["h1"], r["h2"])
    bins[round(h, 2)] += 1
for h in sorted(bins):
    print(f"   h~{h:.2f}: {bins[h]}")
