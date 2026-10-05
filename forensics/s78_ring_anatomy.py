#!/usr/bin/env python3
"""s78: full ring anatomy of the f215 sail — exact side split, corner
indices, per-side v/u ranges, wavy-bottom structure, interior lattice
levels. Design input for the SAIL-BAND (constant-v rows on merged
v-partition with s73 wall-splice)."""
import sys
from collections import Counter

def load(path):
    bnd, inter, tris = [], [], []
    cur = None
    for line in open(path):
        if line.startswith("boundary"):
            cur = "b"; continue
        if line.startswith("interior"):
            cur = "i"; continue
        if line.startswith("tris"):
            cur = "t"; continue
        if line.startswith("type") or line.startswith("hole"):
            cur = None; continue
        p = line.split()
        if not p:
            continue
        if cur == "b" and p[0] == "b":
            bnd.append((float(p[1]), float(p[2])))
        elif cur == "i" and p[0] == "i":
            inter.append((float(p[1]), float(p[2])))
        elif cur == "t" and p[0] == "t":
            tris.append((int(p[1]), int(p[2]), int(p[3])))
    return bnd, inter, tris

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/s78_dumps/tri_0129_Nurbs.txt"
    bnd, inter, tris = load(path)
    n = len(bnd)
    print(f"ring n={n}, interior={len(inter)}, tris={len(tris)}")

    # per-edge classification: |du| vs |dv|
    print("\n== EDGE CLASSIFICATION (i: b[i]->b[i+1]) ==")
    kinds = []
    for i in range(n):
        a = bnd[i]; b = bnd[(i+1) % n]
        du, dv = abs(b[0]-a[0]), abs(b[1]-a[1])
        if dv < 1e-7 and du < 1e-7:
            k = "PT"
        elif dv <= 0.05 * du if du > 1e-9 else dv < 1e-7:
            k = "u-run"   # horizontal
        elif du <= 0.05 * dv if dv > 1e-9 else du < 1e-7:
            k = "v-run"   # vertical (u-const)
        else:
            k = "diag"
        kinds.append(k)
    # print runs
    runs = []
    start = 0
    for i in range(1, n+1):
        if i == n or kinds[i] != kinds[start]:
            runs.append((start, i-1, kinds[start], i-start))
            start = i
    for (s, e, k, ln) in runs:
        us = [bnd[j][0] for j in range(s, e+1)]
        vs = [bnd[j][1] for j in range(s, e+1)]
        print(f"  run [{s:3d}..{e:3d}] {k:5s} len={ln:3d} u[{min(us):+.4f},{max(us):+.4f}] v[{min(vs):+.4f},{max(vs):+.4f}]")

    # corners: transition points
    print("\n== SIDE SPLIT (corner detection: v-run side chains) ==")
    # find the two long v-runs (walls)
    walls = [(s, e, ln) for (s, e, k, ln) in runs if k == "v-run" and ln >= 20]
    print(f"  walls (v-run>=20): {[(s,e,l) for s,e,l in walls]}")

    print("\n== PER-POINT DETAIL (first 8 of each run + every 5th) ==")
    for (s, e, k, ln) in runs:
        if ln < 3:
            continue
        step = max(1, ln // 6)
        detail = ", ".join(f"b{i}:({bnd[i][0]:+.3f},{bnd[i][1]:+.3f})" for i in range(s, e+1, step))
        print(f"  [{s:3d}..{e:3d}] {k}: {detail}")

    print("\n== INTERIOR LATTICE ==")
    # cluster by v (levels)
    lvls = Counter(round(p[1], 4) for p in inter)
    print(f"  distinct v levels: {len(lvls)}")
    for v in sorted(lvls):
        row = sorted(p[0] for p in inter if abs(p[1]-v) < 5e-5)
        print(f"  v={v:+.4f}: {len(row)} pts, u[{min(row):+.4f},{max(row):+.4f}] step~{(max(row)-min(row))/max(1,len(row)-1):.4f}")

    # usage in tris
    used = set()
    for t in tris:
        used.update(t)
    unused_b = [i for i in range(n) if i not in used]
    unused_i = [i-n for i in range(n, n+len(inter)) if i not in used]
    print(f"\n== USAGE: boundary unused {len(unused_b)}/{n}: {unused_b[:25]}")
    print(f"   interior unused {len(unused_i)}/{len(inter)}")
    used_i_v = sorted({round(inter[i][1], 4) for i in range(len(inter)) if (i+n) in used})
    print(f"   interior USED v-levels: {used_i_v}")

if __name__ == "__main__":
    main()
