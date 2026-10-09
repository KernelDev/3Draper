#!/usr/bin/env python3
"""s87: anatomy of the f41/f155 seam needle vs the accepted f39/f43 —
replay the castellation run/level detection offline and compare the
seam neighborhoods."""
import sys

def load(path):
    pts = []
    for line in open(path):
        if line.startswith("r "):
            _, u, v = line.split()
            pts.append((float(u), float(v)))
    return pts

def analyze(path, name):
    pts = load(path)
    n = len(pts)
    umin = min(p[0] for p in pts); umax = max(p[0] for p in pts)
    vmin = min(p[1] for p in pts); vmax = max(p[1] for p in pts)
    uspan = umax - umin; vspan = vmax - vmin
    print(f"=== {name}: n={n} u[{umin:.4f},{umax:.4f}] v[{vmin:.4f},{vmax:.4f}] uspan={uspan:.4f} vspan={vspan:.4f}")

    def edge_class(i):
        a = pts[i]; b = pts[(i+1) % n]
        du = abs(b[0]-a[0]); dv = abs(b[1]-a[1])
        if du >= 2*dv: return 0   # H
        if dv > 2*du: return 1    # V
        return 2                   # diag

    # runs (walk from first V edge)
    start = next((i for i in range(n) if edge_class(i) == 1), None)
    if start is None:
        print("  no V edges!"); return
    runs = []
    i = start
    while i < start + n:
        if edge_class(i % n) == 0:
            s = i % n; j = i
            while j < start + n and edge_class(j % n) == 0:
                j += 1
            e = j % n
            runs.append((s, e))
            i = j
        else:
            i += 1
    print(f"  runs={len(runs)}")
    # v-levels
    def run_v(r):
        s, e = r
        ln = (e + n - s) % n + 1
        return sum(pts[(s+k) % n][1] for k in range(ln)) / ln
    lvl_tol = 0.08 * vspan
    sorted_runs = sorted(runs, key=run_v)
    levels = []
    for r in sorted_runs:
        v = run_v(r)
        if levels and abs(v - levels[-1][0]) < lvl_tol:
            levels[-1][1].append(r)
            k = len(levels[-1][1])
            levels[-1][0] = (levels[-1][0]*(k-1) + v) / k
        else:
            levels.append([v, [r]])
    print(f"  levels={len(levels)}: " + ", ".join(
        f"v={lv:.5f} runs={len(rs)}" for lv, rs in levels))
    # per-run detail: length, u-span, spacing stats
    print("  run details (idx_range, npts, v_avg, uspan, du_median, du_min, du_max):")
    for s, e in runs:
        ln = (e + n - s) % n + 1
        us = [pts[(s+k) % n][0] for k in range(ln)]
        dus = [abs(us[k+1]-us[k]) for k in range(ln-1)]
        va = run_v((s, e))
        u_sp = max(us) - min(us)
        if dus:
            dus.sort()
            med = dus[len(dus)//2]
            print(f"    [{s:4d}..{e:4d}] n={ln:3d} v={va:+.5f} uspan={u_sp:.4f} du_med={med:.5f} du_min={dus[0]:.5f} du_max={dus[-1]:.5f}")
        else:
            print(f"    [{s:4d}..{e:4d}] n={ln:3d} v={va:+.5f} uspan={u_sp:.4f} (single pt)")
    # seam neighborhood: find where consecutive u jumps by > uspan/2 (the wrap)
    print("  seam edges (du > uspan/2):")
    for i in range(n):
        a = pts[i]; b = pts[(i+1) % n]
        if abs(b[0]-a[0]) > uspan/2:
            print(f"    edge ({i},{(i+1)%n}): {a} -> {b}")
    # rim edge length census (shortest 20)
    lens = []
    for i in range(n):
        a = pts[i]; b = pts[(i+1) % n]
        lens.append((((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5, i, (i+1) % n))
    lens.sort()
    print("  shortest 12 rim edges:")
    for L, i, j in lens[:12]:
        print(f"    ({i},{j}) len={L:.6f}  {pts[i]} -> {pts[j]}")
    print()

for path, name in [
    ("forensics/s87_castdump/brep32629_f39_Cone_ring.txt", "f39 ACCEPTED"),
    ("forensics/s87_castdump/brep32629_f41_Cone_ring.txt", "f41 REJECT needle"),
    ("forensics/s87_castdump/brep32629_f43_Cone_ring.txt", "f43 ACCEPTED"),
    ("forensics/s87_castdump/brep32629_f155_Cone_ring.txt", "f155 REJECT needle"),
]:
    analyze(path, name)
