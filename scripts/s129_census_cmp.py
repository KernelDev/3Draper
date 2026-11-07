#!/usr/bin/env python3
"""s129: census comparator — bit-identity of classification lines.

Census pair lines start with '[' + CLASS (FOLD-OVERFAT, WINDING-FLIPFAT,
...); WARN log lines start with '[' + a digit (timestamp) and are
excluded. Per-BREP summaries start with '---'. Pair census lines carry
no timestamp; summary lines neither. Sorted comparison (HashMap pending
order varies run-to-run — s128 lesson: compare metrics, not strings).

Usage: s129_census_cmp.py REF NEW [REF NEW ...]
Exit 0 iff every (ref, new) pair is sorted-identical.
"""
import re, sys


def census_lines(path):
    out = []
    for ln in open(path, errors="replace"):
        ln = ln.rstrip("\n")
        if re.match(r"^\[[A-Z]", ln) or ln.startswith("---"):
            out.append(ln)
    return out


def cmp_pair(ref, new):
    a, b = census_lines(ref), census_lines(new)
    sa, sb = sorted(a), sorted(b)
    tag = "IDENTICAL" if sa == sb else "DIFF"
    print(f"{new}: {len(a)}/{len(b)} lines -> {tag}")
    if sa != sb:
        only_ref = [x for x in sa if x not in sb]
        only_new = [x for x in sb if x not in sa]
        for x in only_ref[:8]:
            print(f"  - REF ONLY: {x[:140]}")
        for x in only_new[:8]:
            print(f"  + NEW ONLY: {x[:140]}")
        return False
    return True


def main():
    args = sys.argv[1:]
    if len(args) < 2 or len(args) % 2:
        sys.exit("usage: s129_census_cmp.py REF NEW [REF NEW ...]")
    ok = True
    for i in range(0, len(args), 2):
        ok = cmp_pair(args[i], args[i + 1]) and ok
    print("RESULT:", "ALL IDENTICAL" if ok else "DIFFERENCES FOUND")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
