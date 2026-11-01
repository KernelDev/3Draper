#!/usr/bin/env python3
"""s122 drift analysis for #10220 ribbon faces f25 (fs=135831) / f27 (fs=135837).

Parses EDGEPTS lines (DRAPPER_DUMP_EDGE_PTS=1, chain+zipper mode) and answers:
  1. per-edge: n, u-range, v-median/std, err median/max
  2. rail v-levels (weighted histogram over all points)
  3. raw-array 3D contiguity between consecutive edges: is the chain seed
     (prev raw last -> this raw first) valid, or is the edge REVERSED
     (raw last closer to prev raw last)?
  4. drift classification per edge
"""
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/my-project/forensics_s122/onzip_vulc_edgepts.txt"
FACES = {"135831": "f25", "135837": "f27"}

pts = defaultdict(lambda: defaultdict(list))   # fs -> edge_idx -> [(sid,i,px,py,pz,u,v,err)]
for line in open(path):
    if not line.startswith("EDGEPTS"):
        continue
    f = line.split()
    fs = f[1].split("=")[1]
    if fs not in FACES:
        continue
    ei = int(f[2].split("=")[1])
    sid = f[3].split("=")[1]
    i = int(f[4].split("=")[1])
    p = tuple(float(x) for x in f[5][3:-1].rstrip(")").split(","))
    uv = tuple(float(x) for x in f[6][4:-1].rstrip(")").split(","))
    err = float(f[7].split("=")[1])
    pts[fs][ei].append((sid, i, p, uv, err))

def d3(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

for fs, name in FACES.items():
    edges = pts[fs]
    if not edges:
        print(f"=== {name} (fs={fs}): NO DATA")
        continue
    print(f"\n================ {name} (fs={fs}) — {len(edges)} edges ================")
    # rail v-levels: weighted histogram over per-edge v-medians of FLAT edges
    flat = []
    for ei, lst in sorted(edges.items()):
        vs = [x[3][1] for x in lst]
        vmed = sorted(vs)[len(vs)//2]
        vstd = (sum((v-vmed)**2 for v in vs)/len(vs))**0.5
        if vstd < 0.01 and len(vs) >= 4:
            flat.append((ei, vmed, len(vs)))
    # cluster levels
    levels = []
    for ei, vmed, n in sorted(flat, key=lambda t: t[1]):
        if levels and abs(vmed - levels[-1][0]) < 0.05:
            levels[-1] = ((levels[-1][0]*levels[-1][1] + vmed*n)/(levels[-1][1]+n), levels[-1][1]+n)
        else:
            levels.append((vmed, n))
    print("rail v-levels (v, pts):", [(round(v,4), n) for v, n in levels])

    prev_last = None
    for ei, lst in sorted(edges.items()):
        lst.sort(key=lambda x: x[1])
        n = len(lst)
        us = [x[3][0] for x in lst]
        vs = [x[3][1] for x in lst]
        errs = sorted(x[4] for x in lst)
        vmed = sorted(vs)[len(vs)//2]
        vstd = (sum((v-vmed)**2 for v in vs)/len(vs))**0.5
        sid = lst[0][0]
        p_first, p_last = lst[0][2], lst[-1][2]
        # contiguity vs previous edge's raw last
        cont = ""
        if prev_last is not None:
            df = d3(prev_last, p_first)
            db = d3(prev_last, p_last)
            if df < 0.05:
                cont = "FWD"
            elif db < 0.05:
                cont = "REV!" if db < df else "REV?"
            else:
                cont = f"GAP(f={df:.3f},b={db:.3f})"
        # nearest level + snap delta
        lv = min(levels, key=lambda t: abs(t[0]-vmed)) if levels else (float('nan'), 0)
        dv = vmed - lv[0]
        # classification
        if vstd < 0.01 and n >= 4:
            cls = "RAIL " if abs(dv) < 0.01 else f"RAIL~OFF dv={dv:+.4f}"
        else:
            cls = "SIDE "
        print(f"e{ei:2d} sid={sid} n={n:3d} u=[{min(us):8.3f},{max(us):8.3f}] "
              f"vmed={vmed:.4f} vstd={vstd:.4f} v=[{min(vs):.4f},{max(vs):.4f}] "
              f"err[med={errs[n//2]:.2e},p90={errs[int(n*0.9)]:.2e},max={errs[-1]:.2e}] "
              f"{cls} {cont}")
        prev_last = p_last
