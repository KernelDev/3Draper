#!/usr/bin/env python3
"""s73: bnd/nm census per BREP from DRAPPER_DUMP_FINAL_OBJS dumps.

bnd = edges used exactly once (boundary), nm = edges used >2 (non-manifold).
Compares two dump dirs.
"""
import glob
import os
import sys
from collections import Counter

def census(obj_path):
    ec = Counter()
    with open(obj_path) as f:
        for line in f:
            if not line.startswith("f "):
                continue
            idx = [int(x) for x in line.split()[1:4]]
            for k in range(3):
                x, y = idx[k], idx[(k + 1) % 3]
                if x != y:
                    ec[(min(x, y), max(x, y))] += 1
    bnd = sum(1 for v in ec.values() if v == 1)
    nm = sum(1 for v in ec.values() if v > 2)
    tris = sum(1 for _ in open(obj_path) if _.startswith("f "))
    return tris, bnd, nm

base_dir, fix_dir = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(os.path.join(base_dir, "*.obj")))
print(f"{'file':46s} {'base tris':>9s} {'fix tris':>9s} {'base bnd':>8s} {'fix bnd':>8s} {'d-bnd':>6s} {'base nm':>7s} {'fix nm':>7s} {'d-nm':>6s}")
for bf in files:
 name = os.path.basename(bf)
 ff = os.path.join(fix_dir, name)
 if not os.path.exists(ff):
  continue
 bt, bb, bn = census(bf)
 ft, fb, fn = census(ff)
 print(f"{name:46s} {bt:9d} {ft:9d} {bb:8d} {fb:8d} {fb-bb:+6d} {bn:7d} {fn:7d} {fn-bn:+6d}")
