#!/usr/bin/env python3
"""s75 item 1b: spatial structure of the top REAL fold families.

For the top face pairs (HM 105,113 / 57,58 / 147,148 / 26,26 / 3,121
and their HOUSING mirrors), print per-line mid/h/area to see whether
folds line up along a rim (systematic wire issue) or cluster in a
degenerate spot.
"""
import re
import sys
sys.path.insert(0, "forensics")
from s75_hm_housing_census import load

TARGETS = [
    (4, 105, 113), (4, 57, 58), (4, 147, 148), (4, 26, 26), (4, 3, 121),
    (4, 7, 226), (4, 259, 260),
    (3, 224, 247), (3, 49, 178), (3, 144, 145), (3, 227, 228), (3, 4, 4),
]

def main():
    rows = load()
    for brep, f1, f2 in TARGETS:
        sel = [r for r in rows if r["brep"] == brep
               and r["f1"] == f1 and r["f2"] == f2
               and not r["subtol"] and not r["exempt"]]
        if not sel:
            continue
        print(f"\n===== brep={brep} pair ({f1},{f2}): {len(sel)} REAL =====")
        print(f"types=({sel[0]['t1']},{sel[0]['t2']}) class mix="
              f"{sorted(set(r['class'] for r in sel))}")
        # sort by mid x,y,z chain for spatial readability
        for r in sorted(sel, key=lambda r: (r["mid"].split(",")[0],)):
            hm = max(r["h1"], r["h2"])
            am = max(r["a1"], r["a2"])
            print(f"  ang={r['ang']:6.2f} h=({r['h1']:.3f},{r['h2']:.3f}) "
                  f"amax={am:.4f} mid=({r['mid']})")

if __name__ == "__main__":
    main()
