#!/usr/bin/env python3
"""session-61: direct 3D check of the known run pairs (67..122 vs 123..178)."""
def load(path):
    ring = []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 6 and parts[0] == "ring":
                ring.append((float(parts[1]), float(parts[2]),
                             (parts[3], parts[4], parts[5])))
    return ring

ring = load("/tmp/ring_brep16033_f18_Cone.tsv")
n = len(ring)
print(f"ring pts: {n}")

# UV-based maximal runs (verified earlier): starts 67, 210, 353... step 143, L=56
total_bad = 0
total_ok = 0
runs = []
start = 67
while start + 2 * 56 <= n:
    L = 56
    # verify UV equality first (should hold)
    uv_ok = all(ring[start + t][:2] == ring[start + L + t][:2] for t in range(L))
    if not uv_ok:
        print(f"start={start}: UV mismatch — stop")
        break
    # 3D equality per point
    bad3d = [t for t in range(L) if ring[start + t][2] != ring[start + L + t][2]]
    print(f"start={start}: UV ok, 3D mismatches: {len(bad3d)}/{L}")
    if bad3d:
        for t in bad3d[:4]:
            print(f"   t={t}: pass1={ring[start+t][2]} pass2={ring[start+L+t][2]}")
    total_bad += len(bad3d)
    total_ok += L - len(bad3d)
    runs.append((start, L, len(bad3d)))
    start += 143
print(f"\ntotal: {total_ok} exact, {total_bad} mismatched 3D pairs over {len(runs)} runs")

# 3D distance stats for a sample mismatch
if runs and runs[0][2] > 0:
    start, L, _ = runs[0]
    bad3d = [t for t in range(L) if ring[start + t][2] != ring[start + L + t][2]]
    import math
    ds = []
    for t in bad3d:
        a = tuple(float(x) for x in ring[start + t][2])
        b = tuple(float(x) for x in ring[start + L + t][2])
        ds.append(math.dist(a, b))
    print(f"3D distances of mismatches: min={min(ds):.3e} max={max(ds):.3e} n={len(ds)}")
