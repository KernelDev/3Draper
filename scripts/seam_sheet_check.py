#!/usr/bin/env python3
"""Session-56: seam-wrap analysis of the TRI_INPUT dumps for the f198
family (dirty fill) vs f226 (clean fill).

For each dump: load boundary UVs + interior (chain) UVs, check the
v-extent against the torus minor period 2*pi, and test whether the
boundary ring and the interior chain live on CONSISTENT v-sheets
(both wrapped or both unwrapped) around the seam.
"""
import math
import sys
from pathlib import Path

TAU = 2 * math.pi
WRAP_EPS = 1e-6


def parse(path):
    b, h, i, mode = [], [], [], None
    header = ""
    for line in path.read_text().splitlines():
        if line.startswith("type="):
            header = line
            continue
        if line in ("boundary", "interior", "tris") or line.startswith("hole "):
            mode = line
            continue
        if line.startswith("b "):
            _, u, v = line.split()
            b.append((float(u), float(v)))
        elif line.startswith("i "):
            _, u, v = line.split()
            i.append((float(u), float(v)))
    return header, b, i


def analyze(path):
    header, b, i = parse(path)
    if not i:
        return None
    bv = [v for (_, v) in b]
    iv = [v for (_, v) in i]
    b_lo, b_hi = min(bv), max(bv)
    i_lo, i_hi = min(iv), max(iv)
    # A sheet mismatch exists when one set crosses the 2π wrap while the
    # other does not (or they sit on different branches at the seam).
    b_cross = b_lo < TAU - 0.3 and b_hi > TAU + 0.3
    i_cross = i_lo < TAU - 0.3 and i_hi > TAU + 0.3
    # distance of each set from the seam value 2π (mod-wise)
    def near_seam(vs):
        return sum(1 for v in vs if abs(((v + math.pi) % TAU) - math.pi) < 0.5)
    return {
        "header": header,
        "b_v": (b_lo, b_hi), "i_v": (i_lo, i_hi),
        "b_cross": b_cross, "i_cross": i_cross,
        "b_nseam": near_seam(bv), "i_nseam": near_seam(iv),
        "n_b": len(b), "n_i": len(i),
    }


for path in sorted(Path(sys.argv[1]).glob("tri_*.txt")):
    r = analyze(path)
    if r is None:
        continue
    label = [p for p in r["header"].split() if p.startswith("label=")]
    label = label[0][6:] if label else "?"
    if not any(f in label for f in ("f198", "f199", "f226", "f200", "f227")):
        continue
    print(f"{path.name}  {label}")
    print(f"   boundary v: [{r['b_v'][0]:.3f}, {r['b_v'][1]:.3f}]  n={r['n_b']}  crosses2pi={r['b_cross']} near-seam={r['b_nseam']}")
    print(f"   interior v: [{r['i_v'][0]:.3f}, {r['i_v'][1]:.3f}]  n={r['n_i']}  crosses2pi={r['i_cross']} near-seam={r['i_nseam']}")
    mismatch = r["b_cross"] != r["i_cross"]
    print(f"   SHEET MISMATCH: {mismatch}")
