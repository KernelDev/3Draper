#!/usr/bin/env python3
"""s64 census 2: classify unused ring verts — ON-EDGE (benign collinear
drop, repairable by edge-split) vs OFF-EDGE (strip cut-off, the f29/f32
class). Runs over DRAPPER_DUMP_TRI_INPUT dumps."""
import glob
import os
import sys
from collections import defaultdict

def analyze(path):
    bnd, holes_pts, interior, tris = [], [], [], []
    mode = None
    header = ""
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
        if line.startswith("hole ") or not line:
            continue
        p = line.split()
        if p[0] == "b":
            bnd.append((float(p[1]), float(p[2])))
        elif p[0] == "h":
            holes_pts.append((float(p[1]), float(p[2])))
        elif p[0] == "i":
            interior.append((float(p[1]), float(p[2])))
        elif p[0] == "t":
            tris.append((int(p[1]), int(p[2]), int(p[3])))
    n_b = len(bnd)
    used = set()
    for t in tris:
        used.update(t)
    unused = [i for i in range(n_b) if i not in used]
    if not unused:
        return None
    # edges
    edges = set()
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            edges.add((min(a, b), max(a, b)))
    all_uv = bnd + holes_pts + interior

    def on_seg(p, a, b, eps=1e-12):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cross) > eps:
            return False
        return (min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
                and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps)

    on_edge = 0
    off_edge = 0
    for k in unused:
        p = bnd[k]
        found = False
        for (a, b) in edges:
            if a == k or b == k:
                continue
            if on_seg(p, all_uv[a], all_uv[b]):
                found = True
                break
        if found:
            on_edge += 1
        else:
            off_edge += 1
    # extract type + label
    stype = ""
    label = ""
    for part in header.split():
        if part.startswith("type="):
            stype = part[5:]
        if part.startswith("label="):
            label = part[6:]
    return label, stype, len(unused), on_edge, off_edge

def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/s64_tri_all"
    files = sorted(glob.glob(os.path.join(d, "*.txt")))
    rows = []
    for f in files:
        r = analyze(f)
        if r:
            rows.append(r)
    # dedupe by (label, stype) keeping first (dumps duplicate on retries)
    seen = {}
    for (label, stype, n, on_e, off_e) in rows:
        key = (label, stype)
        if key not in seen:
            seen[key] = (n, on_e, off_e)
        else:
            prev = seen[key]
            seen[key] = (max(prev[0], n), max(prev[1], on_e), max(prev[2], off_e))
    print(f"{'face':<44} {'type':<10} {'unused':>7} {'on-edge':>8} {'OFF-EDGE':>9}")
    tot_off = 0
    for (label, stype), (n, on_e, off_e) in sorted(seen.items()):
        flag = " <<<" if off_e > 0 else ""
        print(f"{label:<44} {stype:<10} {n:>7} {on_e:>8} {off_e:>9}{flag}")
        tot_off += off_e
    print(f"\ntotal OFF-EDGE (strip-cut class): {tot_off}")
    n_off_faces = sum(1 for v in seen.values() if v[2] > 0)
    print(f"faces with OFF-EDGE > 0: {n_off_faces} of {len(seen)}")

if __name__ == "__main__":
    main()
