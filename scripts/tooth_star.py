#!/usr/bin/env python3
"""session-61: fan star-shapedness from base_A for every tooth (f18 collapsed)."""
import math

def load3(path):
    ring3 = []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 6 and parts[0] == "ring":
                ring3.append((float(parts[1]), float(parts[2]),
                              (parts[3], parts[4], parts[5])))
    return ring3

def collapse(ring3):
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

ring3 = load3("/tmp/ring_brep16033_f18_Cone.tsv")
ring3 = collapse(ring3)
ring = [(p[0], p[1]) for p in ring3]
n = len(ring)
v_lo = min(p[1] for p in ring); v_hi = max(p[1] for p in ring)
u_lo = min(p[0] for p in ring); u_hi = max(p[0] for p in ring)
eps_v = (v_hi - v_lo) * 1e-6; eps_u = (u_hi - u_lo) * 1e-6
best = (0, 0); run_start = run_len = 0
for i in range(2 * n):
    if abs(ring[i % n][1] - v_lo) <= eps_v:
        if run_len == 0: run_start = i
        run_len += 1
    else:
        if run_len > best[1]: best = (run_start, run_len)
        run_len = 0
if run_len > best[1]: best = (run_start, run_len)
uv = [(ring[(best[0] + k) % n]) for k in range(n)]
nb = 0
while nb < n and abs(uv[nb][1] - v_lo) <= eps_v: nb += 1
i = nb
while i < n and abs(uv[i][0] - u_hi) <= eps_u and abs(uv[i][1] - v_lo) > eps_v: i += 1
v_base = uv[i][1]
at_base = lambda p: abs(p[1] - v_base) <= eps_v * 10
j = n
while j > 0 and abs(uv[j-1][0] - u_lo) <= eps_u and uv[j-1][1] < v_base - eps_v*10: j -= 1
l_side_start = j
saw = uv[i:l_side_start]; saw_off = i
teeth = []
k = 0
while k < len(saw):
    if at_base(saw[k]): k += 1; continue
    t0 = k
    while k < len(saw) and not at_base(saw[k]): k += 1
    teeth.append((t0, k))

bad = 0
for ti, (t0, t1) in enumerate(teeth):
    base_a = saw[t0 - 1]
    poly = saw[t0:t1] + [saw[t1]]
    angs = [math.atan2(p[1] - base_a[1], p[0] - base_a[0]) for p in poly]
    # allow wrap: rotate angles so strictly increasing mod 2π
    # normalize: shift so angs[0] ≈ 0
    a0 = angs[0]
    angs = [(a - a0) % (2 * math.pi) for a in angs]
    mono = all(angs[w+1] >= angs[w] - 1e-12 for w in range(len(angs) - 1))
    # also check no full wrap overshoot beyond 2π− small
    if not mono or angs[-1] > 2 * math.pi - 1e-9:
        bad += 1
        if bad <= 3:
            # find the violations
            viol = [(w, angs[w], angs[w+1]) for w in range(len(angs)-1)
                    if angs[w+1] < angs[w] - 1e-12]
            print(f"tooth {ti}: NOT star-shaped from base_A (t0={t0})")
            print(f"  first angle {angs[0]:.4f}, last {angs[-1]:.4f}, violations {len(viol)}")
            for w, a, b in viol[:5]:
                print(f"    idx {w}: {a:.6f} → {b:.6f} at {poly[w]} → {poly[w+1]}")
print(f"\nnon-star-shaped teeth: {bad}/{len(teeth)}")
