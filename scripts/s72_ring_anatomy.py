#!/usr/bin/env python3
"""s72: anatomy of a LUNE v-monotone reject ring (SLEEVE f185 family).

Reads the .ring dump (dedup'd, unwrapped u/v) and prints the chain
structure: v-extremes, the two vmin->vmax walks, flat runs, horizontal
extension reach, and WHERE the non-monotonicity lives (which index
range, what u, how deep).
"""
import sys

path = sys.argv[1]
meta = {}
pts = []
with open(path) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        if line.startswith("idx u v"):
            continue
        if "=" in line and " " not in line:
            k, v = line.split("=", 1)
            meta[k] = v
            continue
        if " " in line and not line[0].isdigit():
            # "vspan=... uspan=..." line
            for tok in line.split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    meta[k] = v
            continue
        toks = line.split()
        if len(toks) == 3:
            pts.append((int(toks[0]), float(toks[1]), float(toks[2])))

n = len(pts)
us = [p[1] for p in pts]
vs = [p[2] for p in pts]
vmin = min(vs); vmax = max(vs)
vspan = vmax - vmin
umin = min(us); umax = max(us)
uspan = umax - umin
flat_eps = 1e-7 * max(vspan, 1e-6)
vmin_i = vs.index(vmin); vmax_i = vs.index(vmax)

print(f"label={meta.get('label')} n={n} (raw {meta.get('n_raw')})")
print(f"vspan={vspan:.6f} uspan={uspan:.6f} vmin_i={vmin_i} vmax_i={vmax_i} flat_eps={flat_eps:.2e}")

# chains a (forward) and b (backward), both vmin->vmax
a = []
i = vmin_i
while True:
    a.append(i)
    if i == vmax_i: break
    i = (i + 1) % n
b = []
i = vmin_i
while True:
    b.append(i)
    if i == vmax_i: break
    i = (i + n - 1) % n

def horiz(i0, i1):
    du = abs(us[i1] - us[i0]); dv = abs(vs[i1] - vs[i0])
    return du > 1e-12 and dv <= 0.5 * du

def decompose(chain):
    m = len(chain)
    e1 = 0
    while e1 + 1 < m and vs[chain[e1 + 1]] <= vmin + flat_eps:
        e1 += 1
    # horizontal extension forward
    while e1 + 1 < m and horiz(chain[e1], chain[e1 + 1]):
        nu = us[chain[e1 + 1]]
        if us[chain[0]] <= us[chain[e1]] and nu >= us[chain[e1]] - 1e-12:
            e1 += 1
        elif us[chain[0]] >= us[chain[e1]] and nu <= us[chain[e1]] + 1e-12:
            e1 += 1
        else:
            break
    e2 = m - 1
    while e2 > e1 + 1 and vs[chain[e2 - 1]] >= vmax - flat_eps:
        e2 -= 1
    while e2 > e1 + 1 and horiz(chain[e2], chain[e2 - 1]):
        pu = us[chain[e2 - 1]]
        if us[chain[m - 1]] >= us[chain[e2]] and pu <= us[chain[e2]] + 1e-12:
            e2 -= 1
        elif us[chain[m - 1]] <= us[chain[e2]] and pu >= us[chain[e2]] - 1e-12:
            e2 -= 1
        else:
            break
    return chain[:e1 + 1], chain[e1:e2 + 1], chain[e2:]

for name, chain in (("a", a), ("b", b)):
    pre, mid, suf = decompose(chain)
    print(f"\nchain {name}: {len(chain)} pts (vmin#{chain[0]} -> vmax#{chain[-1]})")
    print(f"  pre (flat@vmin+ext): {len(pre)} pts, u {us[pre[0]]:.4f}->{us[pre[-1]]:.4f}, v {vs[pre[0]]:.5f}->{vs[pre[-1]]:.5f}")
    print(f"  mid (wall):          {len(mid)} pts, u {us[mid[0]]:.4f}->{us[mid[-1]]:.4f}, v {vs[mid[0]]:.5f}->{vs[mid[-1]]:.5f}, idx {mid[0]}..{mid[-1]}")
    print(f"  suf (flat@vmax+ext): {len(suf)} pts, u {us[suf[0]]:.4f}->{us[suf[-1]]:.4f}, v {vs[suf[0]]:.5f}->{vs[suf[-1]]:.5f}")
    # monotonicity of the wall
    dips = [(mid[i], mid[i+1]) for i in range(len(mid)-1) if vs[mid[i+1]] < vs[mid[i]] - flat_eps]
    if dips:
        tot = sum(vs[x] - vs[y] for x, y in dips)
        print(f"  WALL NON-MONO: {len(dips)} dips, total depth {tot:.6f} ({100*tot/vspan:.2f}% of vspan)")
        for x, y in dips[:8]:
            print(f"    #{x}->#{y}: v {vs[x]:.6f}->{vs[y]:.6f} (d={vs[y]-vs[x]:+.6f}), u {us[x]:.4f}->{us[y]:.4f}")
        if len(dips) > 8:
            print(f"    ... +{len(dips)-8} more")
    else:
        print("  wall mono OK")

# Print the raw v-profile around the dip region to see the shape
print("\n--- v-profile of ALL points at u < 0.02 (the u~0 wall region) ---")
low_u = [(i, u, v) for i, u, v in pts if u < 0.02]
print(f"count={len(low_u)}")
for i, u, v in low_u:
    print(f"  #{i:3d} u={u:.6f} v={v:.6f}")
