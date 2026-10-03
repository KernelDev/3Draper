#!/usr/bin/env python3
"""s72: WHERE do the dips live on the wall chain? (foot / head / middle)

For each v-mono reject ring, decompose both chains exactly as the Rust
code does, take the NON-MONO wall, and classify each dip by its position
along the wall walk: foot = first 10% of walk steps, head = last 10%,
middle = the rest. Absorbable = dips only at foot or head.
"""
import sys, glob

def load(path):
    meta = {}
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("idx u v"):
                continue
            if " " not in line and "=" in line:
                k, v = line.split("=", 1)
                meta[k] = v
                continue
            toks = line.split()
            if len(toks) == 3:
                pts.append((int(toks[0]), float(toks[1]), float(toks[2])))
    return meta, pts

def decompose_like_rust(us, vs, vmin, vmax, chain):
    n = len(us)
    vspan = vmax - vmin
    flat_eps = 1e-7 * max(vspan, 1e-6)
    def horiz(i0, i1):
        du = abs(us[i1] - us[i0]); dv = abs(vs[i1] - vs[i0])
        return du > 1e-12 and dv <= 0.5 * du
    m = len(chain)
    e1 = 0
    while e1 + 1 < m and vs[chain[e1 + 1]] <= vmin + flat_eps:
        e1 += 1
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

for path in sorted(glob.glob(sys.argv[1] + "/*.ring")):
    meta, pts = load(path)
    n = len(pts)
    us = [p[1] for p in pts]
    vs = [p[2] for p in pts]
    vmin = min(vs); vmax = max(vs)
    vspan = vmax - vmin
    flat_eps = 1e-7 * max(vspan, 1e-6)
    vmin_i = vs.index(vmin); vmax_i = vs.index(vmax)
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
    label = meta.get("label", path)
    short = label.split("(")[0]
    for name, chain in (("a", a), ("b", b)):
        pre, mid, suf = decompose_like_rust(us, vs, vmin, vmax, chain)
        W = len(mid) - 1  # steps
        dips = [(k, vs[mid[k+1]] - vs[mid[k]]) for k in range(len(mid)-1)
                if vs[mid[k+1]] < vs[mid[k]] - flat_eps]
        if not dips:
            continue
        # position classes
        foot = [d for k, d in dips if k < 0.1 * W]
        head = [d for k, d in dips if k > 0.9 * W]
        middle = [d for k, d in dips if 0.1 * W <= k <= 0.9 * W]
        first_dip_k = min(k for k, _ in dips)
        last_dip_k = max(k for k, _ in dips)
        # mono suffix/prefix reach
        # maximal mono suffix start:
        e2i = len(mid) - 1
        ks = e2i
        while ks > 0 and vs[mid[ks-1]] <= vs[mid[ks]] + flat_eps:
            ks -= 1
        kp = 0
        while kp < len(mid) - 1 and vs[mid[kp+1]] >= vs[mid[kp]] - flat_eps:
            kp += 1
        absorb_foot = ks          # wall[ks..] mono; absorbable steps = ks
        absorb_head = len(mid)-1-kp
        print(f"{short} chain {name}: wall {len(mid)} pts, {len(dips)} dips "
              f"(foot {len(foot)}, mid {len(middle)}, head {len(head)}), "
              f"dip range steps [{first_dip_k}..{last_dip_k}] of {W}; "
              f"mono-suffix starts at step {absorb_foot} (absorb {absorb_foot}), "
              f"mono-prefix ends at {absorb_head}")
