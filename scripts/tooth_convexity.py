#!/usr/bin/env python3
"""session-61: per-tooth convexity analysis on the collapsed f18 ring."""
def load(path):
    ring = []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 6 and parts[0] == "ring":
                ring.append((float(parts[1]), float(parts[2])))
    return ring

def collapse(ring):
    """Reproduce collapse_doubled_rim_passes (UV+3D string equality)."""
    ring3 = []
    # reload with 3D strings
    with open("/tmp/ring_brep16033_f18_Cone.tsv") as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 6 and parts[0] == "ring":
                ring3.append((float(parts[1]), float(parts[2]),
                              (parts[3], parts[4], parts[5])))
    n = len(ring3)
    last = {}
    next_occ = [None] * n
    for i in range(n - 1, -1, -1):
        next_occ[i] = last.get(ring3[i][2])
        last[ring3[i][2]] = i
    drop = set()
    i = 0
    while i < n:
        j = next_occ[i]
        adv = False
        if j is not None:
            t = 0
            while j + t < n and ring3[i + t][2] == ring3[j + t][2] and \
                  ring3[i + t][:2] == ring3[j + t][:2]:
                t += 1
            if t >= 8 and j == i + t:
                for k in range(j, j + t):
                    drop.add(k)
                i = j + t
                adv = True
        if not adv:
            i += 1
    return [p for k, p in enumerate(ring3) if k not in drop]

ring3 = collapse(None)
ring = [(p[0], p[1]) for p in ring3]
n = len(ring)
print(f"collapsed ring: {n} pts")

# structure: v_lo run
v_lo = min(p[1] for p in ring)
v_hi = max(p[1] for p in ring)
u_lo = min(p[0] for p in ring)
u_hi = max(p[0] for p in ring)
eps_v = (v_hi - v_lo) * 1e-6
eps_u = (u_hi - u_lo) * 1e-6
# find longest v_lo run
best = (0, 0)
run_start, run_len = 0, 0
for i in range(2 * n):
    if abs(ring[i % n][1] - v_lo) <= eps_v:
        if run_len == 0:
            run_start = i
        run_len += 1
    else:
        if run_len > best[1]:
            best = (run_start, run_len)
        run_len = 0
if run_len > best[1]:
    best = (run_start, run_len)
rot = [(ring[(best[0] + k) % n]) for k in range(n)]
uv = rot
nb = 0
while nb < n and abs(uv[nb][1] - v_lo) <= eps_v:
    nb += 1
print(f"bottom run: {nb} pts, BL u={uv[0][0]:.6f}, BR u={uv[nb-1][0]:.6f}")

# v_base from first point off u_hi
i = nb
while i < n and abs(uv[i][0] - u_hi) <= eps_u and abs(uv[i][1] - v_lo) > eps_v:
    i += 1
v_base = uv[i][1]
at_base = lambda p: abs(p[1] - v_base) <= eps_v * 10
print(f"v_base = {v_base:.9f}")

# L side
j = n
while j > 0 and abs(uv[j-1][0] - u_lo) <= eps_u and uv[j-1][1] < v_base - eps_v*10:
    j -= 1
l_side_start = j
print(f"l_side_start={l_side_start}, L-valley={uv[l_side_start-1]}")

# teeth
saw = uv[i:l_side_start]
saw_off = i
teeth = []
k = 0
while k < len(saw):
    if at_base(saw[k]):
        k += 1
        continue
    t0 = k
    while k < len(saw) and not at_base(saw[k]):
        k += 1
    teeth.append((t0, k))
print(f"teeth: {len(teeth)}")

def cr2(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

bad = 0
for ti, (t0, t1) in enumerate(teeth):
    poly = [saw[t0-1]] + saw[t0:t1] + [saw[t1]]
    m = len(poly)
    signs = []
    for k in range(m):
        c = cr2(poly[k], poly[(k+1) % m], poly[(k+2) % m])
        signs.append(1 if c > 1e-18 else (-1 if c < -1e-18 else 0))
    nz = [s for s in signs if s != 0]
    convex = len(set(nz)) <= 1
    if not convex:
        bad += 1
        if bad <= 3:
            print(f"\ntooth {ti} NON-CONVEX (t0={t0} t1={t1}, {m} pts)")
            # find the sign flips
            for k in range(m):
                if signs[k] != 0 and signs[(k+1) % m] != 0 and signs[k] != signs[(k+1) % m]:
                    print(f"  flip at poly[{k}]: {poly[k]} → {poly[(k+1)%m]} → {poly[(k+2)%m]}")
            # print the polygon compactly
            print("  poly:", [(round(p[0],5), round(p[1],5)) for p in poly[:8]], "...")
print(f"\nnon-convex teeth: {bad}/{len(teeth)}")
