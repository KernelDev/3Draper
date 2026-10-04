#!/usr/bin/env python3
"""s75: locate the +1/-1 extra point in mismatched shared-rim chains.

Walks both shared sequences (in wire order) and aligns them; reports the
index and coordinates where the 1:1 pairing breaks.
"""
import sys

def load(path):
    a, b = [], []
    for line in open(path):
        parts = line.split()
        if parts and parts[0] == "A":
            a.append((float(parts[2]), float(parts[3]), float(parts[4])))
        elif parts and parts[0] == "B":
            b.append((float(parts[2]), float(parts[3]), float(parts[4])))
    return a, b

def align(a, b, name):
    print(f"\n=== {name}: A {len(a)} pts, B {len(b)} pts ===")
    # try both B directions
    for label, bb in [("fwd", b), ("rev", b[::-1])]:
        # greedy walk: match A[i] to bb[j]; on mismatch, report
        i = j = 0
        mism = []
        while i < len(a) and j < len(bb):
            if a[i] == bb[j]:
                i += 1
                j += 1
            else:
                mism.append((i, j))
                # advance the longer chain heuristically: try skipping one
                # of each and see which restores alignment
                if i + 1 < len(a) and a[i + 1] == bb[j]:
                    print(f"  [{label}] EXTRA A point at A[{i}] = {a[i]} "
                          f"(B continues at {bb[j]})")
                    i += 1
                elif j + 1 < len(bb) and a[i] == bb[j + 1]:
                    print(f"  [{label}] EXTRA B point at B[{j}] = {bb[j]} "
                          f"(A continues at {a[i]})")
                    j += 1
                else:
                    print(f"  [{label}] HARD mismatch A[{i}]={a[i]} "
                          f"vs B[{j}]={bb[j]} — stopping")
                    break
        print(f"  [{label}] aligned: consumed A {i}/{len(a)}, B {j}/{len(bb)}")
        if mism and not (i == len(a) and j == len(bb)):
            continue

for path in sys.argv[1:]:
    a, b = load(path)
    align(a, b, path)
