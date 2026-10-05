#!/usr/bin/env python3
"""s77: analyze the f215 sail ring (196 pts) + interior lattice structure
from the TRI_INPUT dump: corner detection, side runs (u-const/v-const/
curved), monotone chain split — the input spec for a 4-sided band grid."""
import sys

def load(path):
    bnd, inter, tris = [], [], []
    cur = None
    for line in open(path):
        if line.startswith("boundary"):
            cur = "b"
        elif line.startswith("interior"):
            cur = "i"
        elif line.startswith("tris"):
            cur = "t"
        else:
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
    path = sys.argv[1] if len(sys.argv) > 1 else "forensics/s77_dumps/tri_on/tri_0129_Nurbs.txt"
    bnd, inter, tris = load(path)
    n = len(bnd)
    us = [p[0] for p in bnd]
    vs = [p[1] for p in bnd]
    print(f"ring n={n}: u[{min(us):.4f},{max(us):.4f}] v[{min(vs):.4f},{max(vs):.4f}]")
    # side classification per edge: |du| vs |dv|
    runs = []
    cur_kind = None
    run_start = 0
    for i in range(n):
        a = bnd[i]
        b = bnd[(i + 1) % n]
        du, dv = abs(b[0] - a[0]), abs(b[1] - a[1])
        if du < 1e-9 and dv < 1e-9:
            kind = "zero"
        elif dv <= 0.05 * du:
            kind = "v-const"
        elif du <= 0.05 * dv:
            kind = "u-const"
        else:
            kind = "diag"
        if kind != cur_kind:
            if cur_kind is not None:
                runs.append((cur_kind, run_start, i))
            cur_kind = kind
            run_start = i
    runs.append((cur_kind, run_start, n))
    print("side runs (kind, start..end, len, span):")
    for kind, s, e in runs:
        seg = bnd[s:e] if e < n else bnd[s:]
        print(f"  {kind:8s} [{s:3d}..{e:3d}) len={e-s:3d} u[{min(p[0] for p in seg):.4f},{max(p[0] for p in seg):.4f}] v[{min(p[1] for p in seg):.4f},{max(p[1] for p in seg):.4f}]")
    # interior lattice: rows by v
    ivs = sorted({round(p[1], 6) for p in inter})
    print(f"interior n={len(inter)}: distinct v levels={len(ivs)}: {[f'{v:.3f}' for v in ivs[:12]]}")
    for v in ivs[:6]:
        row = [p for p in inter if abs(p[1] - v) < 1e-6]
        if row:
            print(f"  v={v:.4f}: {len(row)} pts, u[{min(p[0] for p in row):.4f},{max(p[0] for p in row):.4f}]")
    # distribution of ring v-levels vs interior v-levels
    rvs = sorted({round(p[1], 6) for p in bnd})
    print(f"ring distinct v levels={len(rvs)}: {[f'{v:.3f}' for v in rvs[:15]]}")

if __name__ == "__main__":
    main()
