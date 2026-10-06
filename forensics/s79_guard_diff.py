#!/usr/bin/env python3
"""s79: pair-level diff baseline vs fan-guard-on — find regressions."""
import re
from collections import Counter

RE = re.compile(
    r"^\[(?P<tags>[^\]]*)\]\s+brep_idx=(?P<brep>\d+)\s+(?P<name>\S+)"
    r".*faces=\((\d+),(\d+)\)")


def load(path):
    c = Counter()
    with open(path) as f:
        for line in f:
            m = RE.match(line)
            if not m or "FOLD" not in m.group("tags"):
                continue
            subtol = "SUBTOL" in m.group("tags")
            key = (m.group("name"), int(m.group(4)), int(m.group(5)),
                   subtol)
            c[key] += 1
    return c


def main():
    base = load("forensics/s79_drill_full.txt")
    guard = load("/tmp/s79_guard_v5.txt")

    all_keys = set(base) | set(guard)
    deltas = []
    for k in all_keys:
        d = guard.get(k, 0) - base.get(k, 0)
        if d != 0 and not k[3]:  # REAL only
            deltas.append((d, k, base.get(k, 0), guard.get(k, 0)))
    deltas.sort()
    print("=== REAL pair regressions (guard ON worse) ===")
    for d, k, b, g in deltas:
        if d > 0:
            print(f"  {k[0]} faces=({k[1]},{k[2]}): {b} → {g} ({d:+})")
    print("\n=== REAL pair improvements ===")
    for d, k, b, g in reversed(deltas):
        if d < 0:
            print(f"  {k[0]} faces=({k[1]},{k[2]}): {b} → {g} ({d:+})")


if __name__ == "__main__":
    main()
