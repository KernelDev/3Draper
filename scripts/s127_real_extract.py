#!/usr/bin/env python3
"""s127 п.0: REAL composition extractor + baseline compare.

REAL = census line `[CLASS]` with NO second tag inside brackets
(second tag SUBTOL/THIN-FACE/COIL-TURN = exempt).
Usage: s127_real_extract.py <census_file> [--real-only]
"""
import re, sys

path = sys.argv[1]
real_only = "--real-only" in sys.argv

pat = re.compile(r"^\[([A-Z-]+)(?: ([A-Z-]+))?\]")
out = []
for line in open(path):
    m = pat.match(line)
    if not m:
        continue
    if m.group(2):  # exempt tag present
        if real_only:
            continue
        continue
    # REAL line: summarize (class + brep + faces + types)
    keep = line.rstrip()
    out.append(keep)

if real_only:
    for l in out:
        print(l)
else:
    from collections import Counter
    c = Counter()
    for l in out:
        m2 = re.match(r"\[([A-Z-]+)\] brep_idx=\d+ (\S+) \(BREP#(\d+)\).*?faces=\((\d+),(\d+)\) types=\((\w+),", l)
        if m2:
            c[f"{m2.group(3)} f({m2.group(4)},{m2.group(5)}) {m2.group(6)}"] += 1
        else:
            c[l[:60]] += 1
    print(f"REAL total: {sum(c.values())}")
    for k, v in sorted(c.items()):
        print(f"  {v:3d}  {k}")
