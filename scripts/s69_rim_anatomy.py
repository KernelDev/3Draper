#!/usr/bin/env python3
"""s69 diag 2: rim segment anatomy of the HOUSING Torus debt faces.

For each face: split the ring at v-extremes into 2 chains; within each
chain classify runs (v-const / u-const / other) to see the fillet rim
structure: 2 v-const arcs + 2 u-const side lines expected.
"""
import math, os, re

TRI_DIR = "/home/z/my-project/scripts/s69_tri"
WANT = ["f127", "f158", "f159", "f196", "f197", "f212", "f215", "f236", "f238"]


def unwrap(xs, period=2 * math.pi):
    out = [xs[0]]
    for x in xs[1:]:
        prev = out[-1]
        while x - prev > period / 2:
            x -= period
        while prev - x > period / 2:
            x += period
        out.append(x)
    return out


def main():
    for fn in sorted(os.listdir(TRI_DIR)):
        m = re.match(r"tri_\d+_Torus\.txt", fn)
        if not m:
            continue
        txt = open(os.path.join(TRI_DIR, fn)).read().splitlines()
        head = txt[0]
        label = head.split("label=")[1]
        face = label.split("_")[1]
        if face not in WANT or not label.startswith("brep47598"):
            continue
        if "_f" not in label:
            continue
        # only first instance (skip LOD dupes: keep first occurrence)
        nb = int(re.search(r"n_boundary=(\d+)", head).group(1))
        bpts = []
        for line in txt:
            if line.startswith("b "):
                _, u, v = line.split()
                bpts.append((float(u), float(v)))
            elif line.startswith(("h ", "i ")):
                break
        if len(bpts) != nb:
            continue
        uu = unwrap([p[0] for p in bpts])
        vv = unwrap([p[1] for p in bpts])
        n = nb
        vmin_i = min(range(n), key=lambda k: vv[k])
        vmax_i = max(range(n), key=lambda k: vv[k])
        a = []
        i = vmin_i
        while True:
            a.append(i)
            if i == vmax_i:
                break
            i = (i + 1) % n
        b = []
        i = vmax_i
        while True:
            b.append(i)
            if i == vmin_i:
                break
            i = (i + 1) % n
        b = b[::-1]
        print(f"\n=== {label} nb={n} vmin@{vmin_i} vmax@{vmax_i} "
              f"chainA={len(a)} chainB={len(b)} ===")
        for nm, ch in (("A", a), ("B", b)):
            # classify runs along the chain
            segs = []
            cur = None
            for k in ch:
                uk, vk = uu[k], vv[k]
                # which coord is ~const over a window? classify by deltas
                segs.append((uk, vk))
            # simple run detection: look at variation of u and v in the chain
            us = [uu[k] for k in ch]
            vs = [vv[k] for k in ch]
            # print compact: first/last of v-const runs and u-const runs
            runs = []
            mode = None
            start = 0
            for k in range(len(ch) - 1):
                du = abs(us[k + 1] - us[k])
                dv = abs(vs[k + 1] - vs[k])
                m2 = "u" if dv < 1e-4 * (du + 1e-9) and du > 1e-6 else ("v" if du < 1e-4 * (dv + 1e-9) and dv > 1e-6 else "?")
                if m2 == "?":
                    m2 = "u" if dv <= 1e-6 else ("v" if du <= 1e-6 else "x")
                if mode is None:
                    mode = m2
                elif m2 != mode:
                    runs.append((mode, start, k, ch[start], ch[k]))
                    start = k
                    mode = m2
            runs.append((mode, start, len(ch) - 1, ch[start], ch[len(ch) - 1]))
            desc = " ".join(
                f"{m3}[{s}:{e}]u[{uu[i0]:.3f}..{uu[i1]:.3f}]v[{vv[i0]:.3f}..{vv[i1]:.3f}]"
                for m3, s, e, i0, i1 in runs)
            print(f"  chain{nm}: {desc}")


if __name__ == "__main__":
    main()
