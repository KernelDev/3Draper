#!/usr/bin/env python3
"""s127 gate: compare angle_check summaries — kill-switch (pre-s127
behavior) vs new (fixes on), per control file. The census lines too."""
import re, sys

FILES = ["brick_thin_round", "drill_top", "transmission_top", "compressor-13920_top"]

def summary(path):
    out = {}
    txt = open(path, errors="replace").read()
    m = re.search(r"Summary:\n(.*?)(?:\n\n|\Z)", txt, re.S)
    if not m:
        return None, 0
    block = m.group(1)
    for line in block.splitlines():
        mm = re.match(r"\s*([A-Za-z][^:]+):\s+(.*)", line)
        if mm:
            out[mm.group(1).strip()] = mm.group(2).strip()
    verdict = None
    for line in txt.splitlines():
        if line.startswith(("✓", "✗")):
            verdict = line.strip()
    return (out, verdict), txt.count("Sub-tol")

def census_lines(path):
    return [l for l in open(path, errors="replace") if re.match(r"^\[[A-Z]", l)]

for f in FILES:
    ks_v, _ = summary(f"forensics/s127/ac_ks_{f}.txt")
    new_v, _ = summary(f"forensics/s127/ac_{f}.txt")
    print(f"===== {f}")
    if ks_v is None or new_v is None:
        print("  MISSING SUMMARY"); continue
    ks_s, ks_verdict = ks_v
    new_s, new_verdict = new_v
    all_keys = list(dict.fromkeys(list(ks_s.keys()) + list(new_s.keys())))
    for k in all_keys:
        a, b = ks_s.get(k, "—"), new_s.get(k, "—")
        mark = "  " if a == b else "**"
        print(f"  {mark} {k:24s} KS: {a[:52]:52s} NEW: {b[:52]}")
    print(f"  {'  ' if ks_verdict == new_verdict else '**'} verdict KS: {ks_verdict}")
    print(f"     verdict NEW: {new_verdict}")
    # census line diff
    ks_c = sorted(census_lines(f"forensics/s127/ac_ks_{f}.txt"))
    new_c = sorted(census_lines(f"forensics/s127/ac_{f}.txt"))
    same = ks_c == new_c
    print(f"  census lines: KS={len(ks_c)} NEW={len(new_c)} {'IDENTICAL' if same else 'DIFFER'}")
    if not same:
        import difflib
        d = list(difflib.unified_diff(ks_c, new_c, lineterm="", n=0))
        print(f"    diff lines: {len(d)}")
        for l in d[2:14]:
            print(f"    {l[:150]}")
