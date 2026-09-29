#!/usr/bin/env python3
"""s65 diag 10: prototype the TWO-POINTER MONOTONE STRIP triangulation
for the pinched-crescent lune polygon (f9). Validates: full rim
coverage, zero extra boundary edges, area == polygon area, no overlaps.
"""
import glob
from collections import defaultdict

PATH = sorted(glob.glob("/tmp/cdtfail/brep1086_f9_Cone.cdtfail.txt"))[0]

def load():
    mode = None
    boundary, interior, legacy, cdt = [], [], [], []
    for line in open(PATH):
        line = line.strip()
        if line in ("BOUNDARY", "HOLES", "INTERIOR", "LEGACY", "CDT"):
            mode = line
            continue
        p = line.split()
        if mode == "BOUNDARY":
            boundary.append((float(p[0]), float(p[1])))
        elif mode == "INTERIOR":
            interior.append((float(p[0]), float(p[1])))
        elif mode == "LEGACY":
            legacy.append(tuple(int(x) for x in p))
        elif mode == "CDT":
            cdt.append(tuple(int(x) for x in p))
    return boundary, interior, legacy, cdt

def strip_two_chains(boundary):
    """Find the two-chain split: the polygon is [chain A][chain B] where
    A and B share endpoints in geometry (pinch). Detect the split by
    direction reversal of the u-coordinate walk."""
    n = len(boundary)
    # compute du between consecutive points (cyclic)
    dus = []
    for i in range(n):
        u0 = boundary[i][0]
        u1 = boundary[(i + 1) % n][0]
        dus.append(u1 - u0)
    # find the two extremes: the u-min point and u-max point of the loop
    umin_i = min(range(n), key=lambda i: boundary[i][0])
    umax_i = min(range(n), key=lambda i: -boundary[i][0])
    # chain 1: umin_i -> ... -> umax_i (forward), chain 2: umax_i -> ... -> umin_i
    def walk(a, b):
        out = []
        i = a
        while True:
            out.append(i)
            if i == b:
                break
            i = (i + 1) % n
        return out
    c1 = walk(umin_i, umax_i)
    c2 = walk(umax_i, umin_i)  # includes both endpoints too
    return c1, c2

def tri_area(pts, a, b, c):
    (x0, y0), (x1, y1), (x2, y2) = pts[a], pts[b], pts[c]
    return 0.5 * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))

def main():
    boundary, interior, legacy, cdt = load()
    n = len(boundary)

    c1, c2 = strip_two_chains(boundary)
    print(f"chain1 (u-min→u-max): {len(c1)} pts, first={c1[0]} last={c1[-1]}")
    print(f"chain2 (u-max→u-min): {len(c2)} pts, first={c2[0]} last={c2[-1]}")
    # check monotonicity
    for name, ch in (("c1", c1), ("c2", c2)):
        us = [boundary[i][0] for i in ch]
        mono = all(us[i] <= us[i+1] + 1e-12 for i in range(len(us)-1)) or \
               all(us[i] >= us[i+1] - 1e-12 for i in range(len(us)-1))
        print(f"  {name} u-monotone: {mono}")

    # two-pointer strip between c1 (a-chain) and c2 (b-chain), both from
    # u-min endpoint to u-max endpoint. c2 walks u-max→u-min: reverse it.
    A = c1[:]                      # a0..am, u increasing
    B = c2[::-1]                   # b0..bn, u increasing; b0 == a0 (u-min), bn == am (u-max)
    print(f"A: {len(A)} pts ({A[0]}..{A[-1]}), B: {len(B)} pts ({B[0]}..{B[-1]})")

    tris = []
    i = j = 0
    while i < len(A) - 1 or j < len(B) - 1:
        if i >= len(A) - 1:
            # advance B
            tris.append((A[i], B[j], B[j + 1]))
            j += 1
        elif j >= len(B) - 1:
            tris.append((A[i], A[i + 1], B[j]))
            i += 1
        else:
            # choose by min next-u (standard merge)
            ua = A[i + 1] if i + 1 < len(A) else None
            ub = B[j + 1] if j + 1 < len(B) else None
            # compare the NEXT point u values
            if boundary[A[i + 1]][0] <= boundary[B[j + 1]][0]:
                tris.append((A[i], A[i + 1], B[j]))
                i += 1
            else:
                tris.append((A[i], B[j + 1], B[j]))
                j += 1

    print(f"strip tris: {len(tris)}")

    # validate
    rims = set()
    for i in range(n):
        k = (i + 1) % n
        rims.add((min(i, k), max(i, k)))
    ec = defaultdict(int)
    area = 0.0
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            if a != b:
                ec[(min(a, b), max(a, b))] += 1
        area += tri_area(boundary, *t)
    bnd = [e for e, c in ec.items() if c == 1]
    rim_bnd = [e for e in bnd if e in rims]
    extra = [e for e in bnd if e not in rims]
    missing = [e for e in rims if e not in ec]
    # polygon area (shoelace)
    pa = 0.0
    for i in range(n):
        x0, y0 = boundary[i]
        x1, y1 = boundary[(i + 1) % n]
        pa += x0 * y1 - x1 * y0
    pa = abs(pa / 2)
    print(f"area: strip={abs(area):.6f} polygon={pa:.6f} ratio={abs(area)/pa*100:.2f}%")
    print(f"rim covered: {len(rims) - len(missing)}/{len(rims)}, missing={missing}")
    print(f"extra bnd edges: {len(extra)} {extra[:10]}")
    degenerate = [t for t in tris if abs(tri_area(boundary, *t)) < 1e-12]
    print(f"degenerate tris: {len(degenerate)}")

if __name__ == "__main__":
    main()
