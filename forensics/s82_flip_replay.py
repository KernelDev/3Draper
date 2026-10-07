#!/usr/bin/env python3
"""s82: replay the lawson_flip log on the earcut base and trace the
moment edge (6,210) becomes non-manifold (count 3)."""
import re
from collections import Counter

# base triangles
tris = []
for line in open('/tmp/s82_stage/A_earcut.tris'):
    a, b, c = map(int, line.split())
    tris.append([a, b, c])

pat = re.compile(
    r"lawson_flip: edge \((\d+),(\d+)\) tris (\d+) \[(\d+), (\d+), (\d+)\] \+ (\d+) \[(\d+), (\d+), (\d+)\] -> \[(\d+), (\d+), (\d+)\] \+ (?:Some\(\[(\d+), (\d+), (\d+)\]\)|None)")

edges = Counter()


def tri_edges(t):
    for k in range(3):
        a, b = t[k], t[(k + 1) % 3]
        if a != b:
            yield (min(a, b), max(a, b))


for t in tris:
    for e in tri_edges(t):
        edges[e] += 1

print(f"start: edge (6,210) count={edges[(6,210)]}")

step = 0
for line in open('/tmp/s82_staged.log'):
    m = pat.search(line)
    if not m:
        continue
    step += 1
    ev1, ev2 = int(m.group(1)), int(m.group(2))
    i, ni = int(m.group(3)), int(m.group(7))
    old_i = [int(m.group(4)), int(m.group(5)), int(m.group(6))]
    old_n = [int(m.group(8)), int(m.group(9)), int(m.group(10))]
    new_i = [int(m.group(11)), int(m.group(12)), int(m.group(13))]
    new_n = [int(m.group(14)), int(m.group(15)), int(m.group(16))] if m.group(14) else None

    # sanity: current triangles match the logged old state?
    if tris[i] != old_i or tris[ni] != old_n:
        print(f"step {step}: STALE i={i} cur={tris[i]} log={old_i} | ni={ni} cur={tris[ni]} log={old_n}")
        break

    for e in tri_edges(old_i):
        edges[e] -= 1
    for e in tri_edges(old_n):
        edges[e] -= 1
    tris[i] = new_i
    if new_n is not None:
        tris[ni] = new_n
    for e in tri_edges(new_i):
        edges[e] += 1
    if new_n is not None:
        for e in tri_edges(new_n):
            edges[e] += 1

    c = edges[(6, 210)]
    if c != 2:
        print(f"step {step}: edge (6,210) count -> {c}  | flip: {line.strip()[:150]}")
        holders = [t for t in tris if 6 in t and 210 in t]
        print(f"   holders: {holders}")
        if c >= 3:
            break
