#!/usr/bin/env python3
"""s75 item 1: census of HM REAL 474 / HOUSING REAL 425 large-scale folds.

v2: handles all line classes (FOLD-OVER/WINDING-FLIP x FAT/SLIVER),
SUBTOL/EXEMPT flags, and Nurbs(deg=3/3, cps=9x9) types with nested
parens/commas (s74 lesson 3).
"""
import re
from collections import Counter

SRC = "forensics/s75_drill_full.txt"

RE_HEAD = re.compile(
    r"^\[(?P<tags>[^\]]*)\]\s+brep_idx=(?P<brep>\d+)\s+(?P<name>\S+)")
RE_FACES = re.compile(r"faces=\((\d+),(\d+)\)")
RE_TYPES = re.compile(r"types=\((.*?)\) step=")
RE_STEP = re.compile(r"step=\((\d+),(\d+)\)")
RE_AREAS = re.compile(r"areas=\(([^,]+),([^)]+)\) h=")
RE_H = re.compile(r"\bh=\(([^,]+),([^)]+)\)")
RE_ANG = re.compile(r"ang=([\d.]+)")
RE_MID = re.compile(r"mid=\(([^)]+)\)")

def split_top(s):
    """Split on top-level comma (depth-0)."""
    parts, depth, cur = [], 0, ""
    for ch in s:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    parts.append(cur)
    return parts

def parse_line(line):
    m = RE_HEAD.match(line)
    if not m:
        return None
    tags = m.group("tags")
    subtol = "SUBTOL" in tags
    exempt = "EXEMPT" in tags
    cls = re.sub(r"\s*(SUBTOL|TANGENT-EXEMPT|EXEMPT)\s*", "", tags).strip()
    mf = RE_FACES.search(line)
    mt = RE_TYPES.search(line)
    ms = RE_STEP.search(line)
    ma = RE_AREAS.search(line)
    mh = RE_H.search(line)
    mg = RE_ANG.search(line)
    mm = RE_MID.search(line)
    if not (mf and mt and ms and ma and mh and mg):
        return None
    t1, t2 = (split_top(mt.group(1)) + ["?", "?"])[:2]
    return {
        "brep": int(m.group("brep")),
        "name": m.group("name"),
        "class": cls,
        "subtol": subtol,
        "exempt": exempt,
        "ang": float(mg.group(1)),
        "f1": int(mf.group(1)), "f2": int(mf.group(2)),
        "t1": t1, "t2": t2,
        "s1": int(ms.group(1)), "s2": int(ms.group(2)),
        "a1": float(ma.group(1)), "a2": float(ma.group(2)),
        "h1": float(mh.group(1)), "h2": float(mh.group(2)),
        "mid": mm.group(1) if mm else "?",
    }

def load():
    rows = []
    with open(SRC) as f:
        for line in f:
            r = parse_line(line.rstrip("\n"))
            if r:
                rows.append(r)
    return rows

def hbin(h):
    if h < 0.05: return "h<0.05"
    if h < 0.1: return "0.05-0.1"
    if h < 0.2: return "0.1-0.2"
    if h < 0.5: return "0.2-0.5"
    if h < 1.0: return "0.5-1.0"
    return "h>=1.0"

def census(rows, label):
    real = [r for r in rows if not r["subtol"] and not r["exempt"]]
    print(f"\n===== {label}: REAL {len(real)} =====")
    if not real:
        return
    fp = Counter((r["f1"], r["f2"]) for r in real)
    print(f"-- top face pairs (of {len(fp)} distinct pairs) --")
    for (f1, f2), n in fp.most_common(20):
        ex = next(r for r in real if (r["f1"], r["f2"]) == (f1, f2))
        hm = max(max(r["h1"], r["h2"]) for r in real if (r["f1"], r["f2"]) == (f1, f2))
        am = max(max(r["a1"], r["a2"]) for r in real if (r["f1"], r["f2"]) == (f1, f2))
        print(f"  ({f1},{f2}): {n:4d}  types=({ex['t1']},{ex['t2']}) "
              f"step=({ex['s1']},{ex['s2']}) hmax={hm:.3f} amax={am:.4f}")
    same = sum(1 for r in real if r["f1"] == r["f2"])
    print(f"-- same-face: {same}, cross-face: {len(real)-same}")
    cl = Counter(r["class"] for r in real)
    print("-- classes --")
    for k, n in cl.most_common():
        print(f"  {k}: {n}")
    tp = Counter((r["t1"].split("(")[0], r["t2"].split("(")[0]) for r in real)
    print("-- type pairs (bare) --")
    for (t1, t2), n in tp.most_common(15):
        print(f"  ({t1},{t2}): {n}")
    hb = Counter(hbin(max(r["h1"], r["h2"])) for r in real)
    print("-- max-apex h histogram --")
    for k in ["h<0.05", "0.05-0.1", "0.1-0.2", "0.2-0.5", "0.5-1.0", "h>=1.0"]:
        print(f"  {k}: {hb.get(k, 0)}")
    def abin(a):
        if a < 0.001: return "<1e-3"
        if a < 0.01: return "1e-3..1e-2"
        if a < 0.1: return "1e-2..1e-1"
        return ">=1e-1"
    arb = Counter(abin(max(r["a1"], r["a2"])) for r in real)
    print("-- max-area histogram --")
    for k in ["<1e-3", "1e-3..1e-2", "1e-2..1e-1", ">=1e-1"]:
        print(f"  {k}: {arb.get(k, 0)}")

def main():
    rows = load()
    print(f"total pair lines parsed: {len(rows)} (expect 3796)")
    sub = sum(1 for r in rows if r["subtol"])
    ex = sum(1 for r in rows if r["exempt"])
    print(f"subtol={sub} exempt={ex} real={len(rows)-sub-ex}")
    for brep_idx, name in [(4, "HOUSING_MIRROR"), (3, "HOUSING")]:
        census([r for r in rows if r["brep"] == brep_idx], name)

if __name__ == "__main__":
    main()
