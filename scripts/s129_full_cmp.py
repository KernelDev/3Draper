#!/usr/bin/env python3
"""s129: full-output comparator — strips timestamp log lines
("^[YYYY-MM-DD..."), sorts the rest, compares. For angle_check /
all_files / watertight gates where WARN-line count varies with
environment noise but deterministic content must match.

Usage: s129_full_cmp.py REF NEW [REF NEW ...]
"""
import re, sys

TS = re.compile(r"^\[\d{4}-\d{2}-\d{2}")


def load(path):
    return sorted(
        ln.rstrip("\n")
        for ln in open(path, errors="replace")
        if not TS.match(ln) and ln.strip()
    )


def cmp_pair(ref, new):
    a, b = load(ref), load(new)
    tag = "IDENTICAL" if a == b else "DIFF"
    print(f"{new}: {len(a)}/{len(b)} lines -> {tag}")
    if a != b:
        only_ref = [x for x in a if x not in b]
        only_new = [x for x in b if x not in a]
        for x in only_ref[:10]:
            print(f"  - REF ONLY: {x[:150]}")
        for x in only_new[:10]:
            print(f"  + NEW ONLY: {x[:150]}")
        return False
    return True


def main():
    args = sys.argv[1:]
    if len(args) < 2 or len(args) % 2:
        sys.exit("usage: s129_full_cmp.py REF NEW [REF NEW ...]")
    ok = True
    for i in range(0, len(args), 2):
        ok = cmp_pair(args[i], args[i + 1]) and ok
    print("RESULT:", "ALL IDENTICAL" if ok else "DIFFERENCES FOUND")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
