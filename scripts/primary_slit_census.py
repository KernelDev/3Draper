#!/usr/bin/env python3
"""Session-56: primary-triangulation connectivity analysis from TRI_INPUT
dumps — where is the chain slit (usage-1 edges), are all interior Steiner
points used, and what do the complement triangles add on top?

Index space of 't' triples: 0..n_b = boundary, then holes, then interior.
"""
import sys
from collections import defaultdict
from pathlib import Path


def parse(path):
    b, holes, interior, tris = [], [], [], []
    mode = None
    cur_hole = None
    for line in path.read_text().splitlines():
        if line.startswith("type="):
            header = line; continue
        if line == "boundary": mode = "b"; continue
        if line == "interior": mode = "i"; continue
        if line == "tris": mode = "t"; continue
        if line.startswith("hole "):
            cur_hole = []; holes.append(cur_hole); mode = "h"; continue
        if line.startswith("b ") and mode == "b":
            _, u, v = line.split(); b.append((float(u), float(v)))
        elif line.startswith("i ") and mode == "i":
            _, u, v = line.split(); interior.append((float(u), float(v)))
        elif line.startswith("h ") and mode == "h":
            _, u, v = line.split(); cur_hole.append((float(u), float(v)))
        elif line.startswith("t ") and mode == "t":
            _, a, c, d = line.split(); tris.append((int(a), int(c), int(d)))
    return header, b, holes, interior, tris


for fname in sys.argv[1:]:
    path = Path(fname)
    header, b, holes, interior, tris = parse(path)
    n_b = len(b)
    n_i = len(interior)
    i0 = n_b + sum(len(h) for h in holes)  # interior index base

    # usage census over the primary triangles
    usage = defaultdict(int)
    for (a, c, d) in tris:
        for e in ((a, c), (c, d), (d, a)):
            usage[(min(e), max(e))] += 1

    # interior point usage
    used = set()
    for t in tris:
        used.update(t)
    unused_int = [k for k in range(i0, i0 + n_i) if k not in used]

    # usage-1 (slit/boundary) edges involving interior points
    slit_edges = [(e, u) for e, u in usage.items() if u == 1 and (e[0] >= i0 or e[1] >= i0)]
    slit_int_int = [e for e, u in usage.items() if u == 1 and e[0] >= i0 and e[1] >= i0]
    # boundary-ring edges with usage != 1
    ring_edges = []
    n_ring = n_b
    for k in range(n_ring):
        e = (k, (k + 1) % n_ring)
        key = (min(e), max(e))
        ring_edges.append(usage.get(key, 0))

    from collections import Counter
    print(f"\n=== {path.name} ===")
    print(f"  {header}")
    print(f"  interior base={i0}, n_interior={n_i}; unused interior pts: {len(unused_int)}")
    print(f"  tris={len(tris)}; usage hist: {dict(Counter(usage.values()))}")
    print(f"  slit edges (usage-1 touching interior): {len(slit_edges)}")
    print(f"    of which interior-interior: {len(slit_int_int)}")
    print(f"  ring-edge usage hist: {dict(Counter(ring_edges))}")
