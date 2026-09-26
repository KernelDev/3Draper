#!/usr/bin/env python3
"""session-61: verify 3D bit-exactness of the doubled flank passes (f18)."""
def load(path):
    ring, lat = [], []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 3:
                continue
            kind = parts[0]
            if kind == "ring" and len(parts) == 6:
                u, v = float(parts[1]), float(parts[2])
                p3 = (parts[3], parts[4], parts[5])  # keep string form = printed bits
                ring.append((u, v, p3))
            elif kind == "lat":
                ring and None
                lat.append((float(parts[1]), float(parts[2])))
    return ring, lat

ring, lat = load("/tmp/ring_brep16033_f18_Cone.tsv")
n = len(ring)
print(f"ring pts: {n}")
# check the run pairs: 67..122 vs 123..178 (and generalized maximal runs)
def eq(i, j):
    return ring[i % n][2] == ring[j % n][2]  # 3D string equality (9-decimal print)

# find maximal same-direction bit-exact adjacent double runs
runs = []
i = 0
while i < n:
    # try to find L such that ring[i..i+L-1] == ring[i+L..i+2L-1]
    L = 0
    while (i + 2 * (L + 1)) <= 2 * n and eq(i + L, i + L + L + 0) and L < 300:
        L += 1
    # simpler: incremental growth
    L = 0
    while i + 2 * L + 1 < n and ring[i + L][2] == ring[i + L + L][2]:
        L += 1
    if L >= 8:
        runs.append((i, L))
        i += 2 * L
    else:
        i += 1
print(f"maximal double-runs (3D bit-exact, L>=8): {len(runs)}")
for i, L in runs[:8]:
    print(f"  start={i} L={L}: uv {ring[i][0]:.6f},{ring[i][1]:.6f} → {ring[i+L-1][0]:.6f},{ring[i+L-1][1]:.6f}")
# also verify UV equality within a run
bad_uv = 0
for i, L in runs:
    for t in range(L):
        if ring[i + t][:2] != ring[i + L + t][:2]:
            bad_uv += 1
print(f"UV mismatches inside runs: {bad_uv}")
# what's the connector segment (end of run1 → start of run2 = index i+L-1 → i+L)?
if runs:
    i, L = runs[0]
    a, b = ring[i + L - 1], ring[i + L]
    print(f"connector: ({a[0]:.6f},{a[1]:.6f}) → ({b[0]:.6f},{b[1]:.6f})")
    print(f"  3D: {a[2]} → {b[2]}")
    import math
    du, dv = b[0] - a[0], b[1] - a[1]
    print(f"  chord length uv: {math.hypot(du, dv):.6f}")
