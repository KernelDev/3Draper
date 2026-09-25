#!/usr/bin/env python3
"""Find the first self-intersection in P1 (ring + chain) of a tri-input dump.

Usage: python3 find_p1_crossing.py <dump_file>
P1 = boundary points followed by interior points (append order), closed
implicitly. Reports the first crossing segment pair (excluding shared
endpoints of consecutive segments).
"""
import sys, math

def load(path):
    bnd, inter = [], []
    mode = None
    with open(path) as fh:
        for line in fh:
            if line.startswith('type='):
                hdr = line
                continue
            if line.startswith('boundary'):
                mode = 'b'; continue
            if line.startswith('hole'):
                mode = 'h'; continue
            if line.startswith('interior'):
                mode = 'i'; continue
            if line.startswith('tris'):
                mode = 't'; continue
            p = line.split()
            if len(p) < 3:
                continue
            if p[0] == 'b' and mode == 'b':
                bnd.append((float(p[1]), float(p[2])))
            elif p[0] == 'i' and mode == 'i':
                inter.append((float(p[1]), float(p[2])))
    return hdr, bnd, inter

def seg_int(p1, p2, p3, p4, eps=1e-12):
    d1x, d1y = p2[0]-p1[0], p2[1]-p1[1]
    d2x, d2y = p4[0]-p3[0], p4[1]-p3[1]
    den = d1x*d2y - d1y*d2x
    if abs(den) < eps:
        return None
    t = ((p3[0]-p1[0])*d2y - (p3[1]-p1[1])*d2x) / den
    s = ((p3[0]-p1[0])*d1y - (p3[1]-p1[1])*d1x) / den
    if eps < t < 1-eps and eps < s < 1-eps:
        return (p1[0]+t*d1x, p1[1]+t*d1y)
    return None

def main():
    hdr, bnd, inter = load(sys.argv[1])
    print(hdr.strip())
    print(f"n_boundary={len(bnd)} n_interior={len(inter)}")
    if inter:
        us = sorted(set(round(p[0], 9) for p in inter))
        vs = sorted(set(round(p[1], 9) for p in inter))
        print(f"lat: {len(us)}x{len(vs)} bbox=[{us[0]:.4f},{us[-1]:.4f}]x[{vs[0]:.4f},{vs[-1]:.4f}]")
        print(f"first chain pts: {inter[:3]}")
        print(f"last chain pts: {inter[-3:]}")
    # P1 polygon: boundary + interior, closed
    poly = bnd + inter
    n = len(poly)
    segs = [((i), (i+1) % n) for i in range(n)]
    found = 0
    for a in range(n):
        p1, p2 = poly[segs[a][0]], poly[segs[a][1]]
        for b in range(a+2, n):
            if a == 0 and b == n-1:
                continue
            p3, p4 = poly[segs[b][0]], poly[segs[b][1]]
            hit = seg_int(p1, p2, p3, p4)
            if hit:
                la = 'bnd' if segs[a][0] < len(bnd) else 'chain'
                lb = 'bnd' if segs[b][0] < len(bnd) else 'chain'
                print(f"CROSS: seg#{a}({la}) {p1}->{p2}  X  seg#{b}({lb}) {p3}->{p4}  at ({hit[0]:.6f},{hit[1]:.6f})")
                found += 1
                if found >= 6:
                    return
    if not found:
        print("P1: no crossing found")

main()
