#!/usr/bin/env python3
"""session-61: proper next-occurrence scan for parallel bit-exact double runs."""
def load(path):
    ring = []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 6 and parts[0] == "ring":
                ring.append((float(parts[1]), float(parts[2]),
                             (parts[3], parts[4], parts[5])))
    return ring

def scan(ring, require_uv=True):
    n = len(ring)
    # next-occurrence index: for each i, the next j>i with same 3D string
    nxt = [None] * n
    last = {}
    for i in range(n - 1, -1, -1):
        nxt[i] = last.get(ring[i][2])
        last[ring[i][2]] = i
    runs = []
    i = 0
    while i < n:
        j = nxt[i]
        if j is not None:
            # grow t while ring[i+t] == ring[j+t] (3D AND UV)
            t = 0
            while j + t < n and ring[i + t][2] == ring[j + t][2] and \
                  (not require_uv or ring[i + t][:2] == ring[j + t][:2]):
                t += 1
            if t >= 8:
                runs.append((i, j, t))
                # skip past the whole double-run region
                i = j + t
                continue
        i += 1
    return runs

for fid in (18, 20):
    ring = load(f"/tmp/ring_brep16033_f{fid}_Cone.tsv")
    runs = scan(ring)
    n = len(ring)
    dropped = sum(t for (_, _, t) in runs)
    print(f"f{fid}: {len(runs)} maximal double-runs, would drop {dropped}/{n} pts "
          f"→ {n - dropped}")
    # adjacency check: j == i + t (run2 starts right after run1 ends)?
    adj = sum(1 for (i, j, t) in runs if j == i + t)
    print(f"   adjacent (j==i+t): {adj}/{len(runs)}")
    lens = sorted(set(t for (_, _, t) in runs))
    print(f"   run lengths: {lens}")
    for (i, j, t) in runs[:3] + runs[-2:]:
        print(f"   i={i} j={j} L={t} uv=({ring[i][0]:.6f},{ring[i][1]:.6f})")
