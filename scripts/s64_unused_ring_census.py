#!/usr/bin/env python3
"""s64 census: count faces whose legacy earcutr pass leaves RING VERTICES
unused (the strip cut-off class), across all DRAPPER_DUMP_TRI_INPUT dumps.

Usage: python3 s64_unused_ring_census.py <dump_dir>
"""
import glob
import os
import sys
from collections import defaultdict

def analyze(path):
    bnd, interior, tris = [], [], []
    mode = None
    header = ""
    holes = []
    for line in open(path):
        line = line.strip()
        if line.startswith("type="):
            header = line
            continue
        if line == "boundary":
            mode = "b"
            continue
        if line == "interior":
            mode = "i"
            continue
        if line == "tris":
            mode = "t"
            continue
        if line.startswith("hole "):
            holes.append(int(line.split()[1]))
            continue
        if not line:
            continue
        p = line.split()
        if p[0] == "b":
            bnd.append((float(p[1]), float(p[2])))
        elif p[0] == "i":
            interior.append((float(p[1]), float(p[2])))
        elif p[0] == "t":
            tris.append((int(p[1]), int(p[2]), int(p[3])))
    n_b = len(bnd)
    # ring verts = 0..n_b (outer) + hole ranges (approx: from each hole start
    # to the next hole start or end of holes section). We know holes only by
    # their start indices; the hole section ends at the first interior point
    # which is index n_b + total_hole_pts. Approximate: ring = 0..(n_b + hole_pts)
    # — we can't separate exactly, but unused OUTER verts (0..n_b) is the
    # primary signal.
    used = set()
    for t in tris:
        used.update(t)
    unused_outer = [i for i in range(n_b) if i not in used]
    unused_interior = [i for i in range(n_b, n_b + len(interior)) if i not in used]
    # hole ring verts: holes are between outer and interior
    hole_region = set()
    for h in holes:
        hole_region.add(h)
    return header, n_b, len(interior), len(tris), unused_outer, unused_interior

def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/s64_tri"
    files = sorted(glob.glob(os.path.join(d, "*.txt")))
    total = 0
    affected = 0
    by_label = defaultdict(list)
    for f in files:
        header, n_b, n_int, n_tris, unused_outer, unused_int = analyze(f)
        total += 1
        if unused_outer:
            affected += 1
            # extract label
            label = ""
            for part in header.split():
                if part.startswith("label="):
                    label = part[6:]
                    break
            by_label[label].append((os.path.basename(f), len(unused_outer),
                                    unused_outer[:5], unused_outer[-1] if unused_outer else None))
    print(f"faces dumped: {total}, with UNUSED OUTER RING verts: {affected}")
    for label, entries in sorted(by_label.items()):
        tot = sum(e[1] for e in entries)
        print(f"  {label}: {len(entries)} dump(s), total unused {tot}")
        for e in entries[:3]:
            print(f"    {e[0]}: {e[1]} unused, first={e[2]}, last={e[3]}")

if __name__ == "__main__":
    main()
