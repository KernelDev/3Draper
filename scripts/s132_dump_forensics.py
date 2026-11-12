#!/usr/bin/env python3
"""s132: why does band_ring_split reject #27977 f7/f65/f2/f6/f68?
Replicates the s114 v-sign-block analysis from custom_cdt.rs on the
CDT_FAIL dumps, plus a used-point census for both triangulations."""
import re, sys, glob, os

DUMPDIR = "/home/z/my-project/scripts/s132/cdtfail"


def parse(path):
    sec = None
    d = {"bnd": [], "holes": [], "int": [], "legacy": [], "cdt": []}
    for ln in open(path):
        ln = ln.rstrip("\n")
        if ln == "BOUNDARY":
            sec = "bnd"; continue
        if ln == "HOLES":
            sec = "holes"; continue
        if ln == "INTERIOR":
            sec = "int"; continue
        if ln == "LEGACY":
            sec = "legacy"; continue
        if ln == "CDT":
            sec = "cdt"; continue
        if ln.startswith("n_boundary") or ln.startswith("HOLE "):
            continue
        if not ln:
            continue
        parts = ln.split()
        if sec in ("bnd", "int"):
            d[sec].append((float(parts[0]), float(parts[1])))
        elif sec in ("legacy", "cdt"):
            d[sec].append((int(parts[0]), int(parts[1]), int(parts[2])))
    return d


def band_split_analysis(bnd, verbose=True):
    """Replica of band_ring_split (s114). Returns the failure reason."""
    n = len(bnd)
    if n < 8:
        return "n<8"
    vs = [p[1] for p in bnd]
    vmin, vmax = min(vs), max(vs)
    vspan = vmax - vmin
    if vspan <= 0:
        return "vspan<=0"
    veps = 1e-9 * vspan

    def sgn(i):
        dv = bnd[(i + 1) % n][1] - bnd[i][1]
        if dv > veps:
            return 1
        if dv < -veps:
            return -1
        return 0

    first = next((i for i in range(n) if sgn(i) != 0), None)
    if first is None:
        return "all-neutral"
    seq = []
    for k in range(n):
        i = (first + k) % n
        s = sgn(i)
        if s != 0:
            seq.append((i, s))
    if len(seq) < 4:
        return f"seq<4 ({len(seq)})"
    # rotate so pos 0 starts a block
    rot_at = 0
    if seq[-1][1] == seq[0][1]:
        last_t = 0
        for k in range(1, len(seq)):
            if seq[k][1] != seq[k - 1][1]:
                last_t = k
        rot_at = last_t
    rot = seq[rot_at:] + seq[:rot_at]
    blocks = []
    s = 0
    for k in range(1, len(rot)):
        if rot[k][1] != rot[k - 1][1]:
            blocks.append((s, k - 1, rot[k - 1][1]))
            s = k
    blocks.append((s, len(rot) - 1, rot[-1][1]))
    if verbose:
        print(f"    v-blocks: {len(blocks)} signs={[b[2] for b in blocks]}")
    if len(blocks) != 2:
        return f"blocks!=2 ({len(blocks)}: signs {[b[2] for b in blocks]})"
    if blocks[0][2] != -blocks[1][2]:
        return f"same-sign blocks ({blocks[0][2]},{blocks[1][2]})"
    # walls + caps
    def chain(f, t):
        c = []
        i = f
        c.append(i)
        while i != t:
            i = (i + 1) % n
            c.append(i)
        return c

    def wall_pts(b):
        e0 = rot[b[0]][0]
        e1 = rot[b[1]][0]
        return (e0, (e1 + 1) % n)

    b0, b1 = blocks
    a0, a1 = wall_pts(b0)
    c0, c1 = wall_pts(b1)
    bridge1 = chain(a1, c0)
    bridge2 = chain(c1, a0)
    if b0[2] == 1:
        right, left_desc, cap_low, cap_high = chain(a0, a1), chain(c0, c1), bridge2, bridge1
    else:
        right, left_desc, cap_low, cap_high = chain(c0, c1), chain(a0, a1), bridge1, bridge2
    left = left_desc
    left = left[::-1]
    if verbose:
        print(f"    right={len(right)} left={len(left_desc)} "
              f"cap_low={len(cap_low)} cap_high={len(cap_high)}")
    if len(right) < 2 or len(left) < 2 or not cap_low or not cap_high:
        return "degenerate wall/cap"
    # u-monotone walls (s83 contract: bottom/top caps u-monotone)?
    def umono(c):
        us = [bnd[i][0] for i in c]
        return all(us[i] <= us[i + 1] + 1e-12 for i in range(len(us) - 1)) or \
               all(us[i] >= us[i + 1] - 1e-12 for i in range(len(us) - 1))
    if verbose:
        print(f"    cap_low u-mono={umono(cap_low)} cap_high u-mono={umono(cap_high)}")
    # v-monotone walls?
    def vmono(c):
        vs2 = [bnd[i][1] for i in c]
        return all(vs2[i] <= vs2[i + 1] + 1e-12 for i in range(len(vs2) - 1)) or \
               all(vs2[i] >= vs2[i + 1] - 1e-12 for i in range(len(vs2) - 1))
    if verbose:
        print(f"    right v-mono={vmono(right)} left v-mono={vmono(left_desc)}")
    return "OK"


def used_census(d):
    nb = len(d["bnd"])
    ni = len(d["int"])
    total = nb + ni
    res = {}
    for k in ("legacy", "cdt"):
        used = set()
        for t in d[k]:
            used.update(t)
        drops = sum(1 for i in range(nb, total) if i not in used)
        unused_bnd = sum(1 for i in range(nb) if i not in used)
        res[k] = (len(d[k]), drops, unused_bnd)
    return res


def row_census(d):
    """Replica of the v-const row clustering."""
    ints = d["int"]
    if not ints:
        return "no interior"
    ivmin = min(p[1] for p in ints)
    ivmax = max(p[1] for p in ints)
    span = ivmax - ivmin
    if span <= 0:
        return "flat interior v"
    tol = 2e-3 * span
    order = sorted(ints, key=lambda p: (p[1], p[0]))
    rows = []
    cur = [order[0]]
    for p in order[1:]:
        if p[1] - cur[-1][1] > tol:
            rows.append(cur)
            cur = [p]
        else:
            cur.append(p)
    rows.append(cur)
    # row stats
    lens = [len(r) for r in rows]
    vs_spread = max(max(p[1] for p in r) - min(p[1] for p in r) for r in rows)
    u_asc = all(
        all(r[i][0] < r[i + 1][0] for i in range(len(r) - 1)) for r in rows
    )
    return (f"rows={len(rows)} lens[{min(lens)}..{max(lens)}] "
            f"max_v_spread={vs_spread:.2e} (tol={tol:.2e}) u_strict_asc={u_asc}")


for path in sorted(glob.glob(f"{DUMPDIR}/*.cdtfail.txt")):
    name = os.path.basename(path).replace(".cdtfail.txt", "")
    d = parse(path)
    print(f"=== {name}: bnd={len(d['bnd'])} int={len(d['int'])} "
          f"holes={len(d['holes'])} legacy={len(d['legacy'])} cdt={len(d['cdt'])}")
    uc = used_census(d)
    print(f"    legacy: tris={uc['legacy'][0]} int_drops={uc['legacy'][1]} "
          f"unused_bnd={uc['legacy'][2]}")
    print(f"    cdt:    tris={uc['cdt'][0]} int_drops={uc['cdt'][1]} "
          f"unused_bnd={uc['cdt'][2]}")
    print(f"    rows: {row_census(d)}")
    reason = band_split_analysis(d["bnd"])
    print(f"    band_ring_split -> {reason}")
    print()
