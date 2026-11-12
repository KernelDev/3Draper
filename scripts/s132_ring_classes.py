#!/usr/bin/env python3
"""s132: edge-class (H/V/D) + corner census + v-run structure for the
5 FAT-heavy #27977 Torus faces (CDT_FAIL dumps)."""
import sys


def parse(path):
    sec = None
    d = {"bnd": [], "int": [], "legacy": [], "cdt": []}
    for ln in open(path):
        ln = ln.rstrip("\n")
        if ln in ("BOUNDARY", "HOLES", "INTERIOR", "LEGACY", "CDT"):
            sec = {"BOUNDARY": "bnd", "HOLES": None, "INTERIOR": "int",
                   "LEGACY": "legacy", "CDT": "cdt"}[ln]
            continue
        if not ln or ln.startswith("n_boundary") or ln.startswith("HOLE"):
            continue
        p = ln.split()
        if sec == "bnd":
            d["bnd"].append((float(p[0]), float(p[1])))
        elif sec == "int":
            d["int"].append((float(p[0]), float(p[1])))
        elif sec in ("legacy", "cdt"):
            d[sec].append((int(p[0]), int(p[1]), int(p[2])))
    return d


def analyze(path, name):
    d = parse(path)
    bnd = d["bnd"]
    n = len(bnd)
    print(f"=== {name}: n={n}")

    # edge classes (custom_cdt edge_class)
    def ecls(i):
        a, b = bnd[i], bnd[(i + 1) % n]
        du = abs(b[0] - a[0])
        dv = abs(b[1] - a[1])
        if du >= 2.0 * dv:
            return 0  # H
        if dv > 2.0 * du:
            return 1  # V
        return 2  # D

    cls = [ecls(i) for i in range(n)]
    from collections import Counter
    cnt = Counter(cls)
    has_d = cnt.get(2, 0) > 0
    corners = [i for i in range(n) if cls[(i + n - 1) % n] != cls[i]]
    print(f"    H={cnt.get(0,0)} V={cnt.get(1,0)} D={cnt.get(2,0)} "
          f"has_d={has_d} corners={len(corners)}")

    # v-sign runs (non-neutral), with sizes in EDGES and v-extent
    vs = [p[1] for p in bnd]
    vmin, vmax = min(vs), max(vs)
    vspan = vmax - vmin
    veps = 1e-9 * vspan

    def sgn(i):
        dv = bnd[(i + 1) % n][1] - bnd[i][1]
        return 1 if dv > veps else (-1 if dv < -veps else 0)

    # runs over ALL edges incl neutral, starting at first non-neutral
    first = next((i for i in range(n) if sgn(i) != 0), 0)
    runs = []  # (sign, n_edges, v_extent, n_pts)
    cur_s, cur_n, cur_lo, cur_hi = sgn(first), 0, vs[first], vs[first]
    for k in range(n):
        i = (first + k) % n
        s = sgn(i)
        if k == 0:
            continue
        if s == cur_s:
            cur_n += 1
            cur_lo = min(cur_lo, vs[i])
            cur_hi = max(cur_hi, vs[i])
        else:
            runs.append((cur_s, cur_n, cur_hi - cur_lo, cur_n + 1))
            cur_s, cur_n, cur_lo, cur_hi = s, 0, vs[i], vs[i]
    runs.append((cur_s, cur_n, cur_hi - cur_lo, cur_n + 1))
    nonneutral = [(k, r) for k, r in enumerate(runs) if r[0] != 0]
    print(f"    total runs={len(runs)} non-neutral={len(nonneutral)}")
    print("    non-neutral runs (sign, edges, v_extent):")
    for k, r in nonneutral:
        print(f"      #{k}: s={r[0]:+d} edges={r[1]:3d} vext={r[2]:.4f}"
              f" ({100*r[2]/vspan:.1f}% of vspan)")
    # neutral runs
    neutrals = [(k, r) for k, r in enumerate(runs) if r[0] == 0]
    for k, r in neutrals:
        if r[1] >= 3:
            print(f"      #{k}: NEUTRAL edges={r[1]:3d}")
    print()


BASE = "/home/z/my-project/scripts/s132/cdtfail"
for nm in ("brep27977_f7_Torus", "brep27977_f65_Torus", "brep27977_f6_Torus",
           "brep27977_f2_Torus"):
    analyze(f"{BASE}/{nm}.cdtfail.txt", nm)
