#!/usr/bin/env python3
"""Compare per-face fold-pair counts between two probe stdout logs."""
import sys, re, collections

def face_pairs(path):
    per_brep = collections.defaultdict(collections.Counter)
    brep = None
    with open(path) as fh:
        for line in fh:
            m = re.match(r"^--- brep_idx=(\d+)", line)
            if m:
                brep = int(m.group(1))
                continue
            m = re.search(r"brep_idx=(\d+).*faces=\((\d+),(\d+)\)", line)
            if m and ("FOLD-OVER" in line or "WINDING-FLIP" in line):
                b = int(m.group(1))
                a, c = int(m.group(2)), int(m.group(3))
                f = min(a, c)
                per_brep[b][f] += 1
    return per_brep

a = face_pairs(sys.argv[1])
b = face_pairs(sys.argv[2])
names = {0:"SHAFT",1:"GEAR",2:"SLEEVE",3:"HOUSING",4:"HM"}
print(f"{'brep':6} {'face':>5} {'A':>5} {'B':>5} {'delta':>6}")
tot = 0
rows = []
for br in sorted(set(a) | set(b)):
    for f in sorted(set(a[br]) | set(b[br])):
        pa, pb = a[br].get(f, 0), b[br].get(f, 0)
        d = pb - pa
        if d != 0:
            rows.append((d, br, f, pa, pb))
for d, br, f, pa, pb in sorted(rows, key=lambda r: -abs(r[0]))[:25]:
    print(f"{names.get(br,br):6} {f:>5} {pa:>5} {pb:>5} {d:>+6}")
    tot += d
print("TOTAL delta:", tot)
