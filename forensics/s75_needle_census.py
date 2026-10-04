#!/usr/bin/env python3
"""s75 item 1c: needle-sidedness census — how many REAL fold pairs have
one degenerate side (min apex height tiny)? Sizes the 'one-side needle'
fraction vs 'both sides fat'.
"""
import sys
sys.path.insert(0, "forensics")
from s75_hm_housing_census import load
from collections import Counter

def mbin(h):
    if h < 0.001: return "<1e-3"
    if h < 0.005: return "1e-3..5e-3"
    if h < 0.02: return "5e-3..2e-2"
    if h < 0.05: return "2e-2..5e-2"
    if h < 0.2: return "5e-2..0.2"
    return ">=0.2"

def main():
    rows = load()
    for brep, name in [(4, "HM"), (3, "HOUSING"), (2, "SLEEVE")]:
        real = [r for r in rows if r["brep"] == brep
                and not r["subtol"] and not r["exempt"]]
        if not real:
            continue
        print(f"\n===== {name} REAL {len(real)}: min-apex-h histogram =====")
        mb = Counter(mbin(min(r["h1"], r["h2"])) for r in real)
        tot = len(real)
        for k in ["<1e-3", "1e-3..5e-3", "5e-3..2e-2", "2e-2..5e-2",
                  "5e-2..0.2", ">=0.2"]:
            n = mb.get(k, 0)
            print(f"  {k}: {n:4d} ({100*n/tot:.0f}%)")
        needle = sum(v for k, v in mb.items() if k in ("<1e-3", "1e-3..5e-3"))
        print(f"  -> one-side needle (<5e-3): {needle} "
              f"({100*needle/tot:.0f}%)")
        # cross-tab: needle-sided by top pairs
        if name == "HM":
            fp = Counter((r["f1"], r["f2"]) for r in real
                         if min(r["h1"], r["h2"]) < 0.005)
            print("  -- needle-sided top pairs --")
            for (f1, f2), n in fp.most_common(10):
                print(f"     ({f1},{f2}): {n}")

if __name__ == "__main__":
    main()
