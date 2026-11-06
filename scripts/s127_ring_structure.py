#!/usr/bin/env python3
"""s127 п.1b: print the ring structure of brep10212 f2 outer polygon —
key transitions, chain segmentation by direction."""
import math, sys

def load_uv(path):
    outer, holes = [], {}
    for l in open(path):
        parts = l.split()
        if not parts or parts[0].startswith("#"):
            continue
        if parts[0] == "o":
            outer.append((float(parts[1]), float(parts[2])))
        elif parts[0].startswith("h"):
            holes.setdefault(int(parts[0][1:]), []).append((float(parts[1]), float(parts[2])))
    return outer, [holes[k] for k in sorted(holes)]

outer, holes = load_uv("forensics/s127/uv10212/uvpoly_1721.txt")
n = len(outer)
print(f"outer n={n}")
# classify each edge: direction in (du, dv)
for i in range(n):
    j = (i+1) % n
    du = outer[j][0] - outer[i][0]
    dv = outer[j][1] - outer[i][1]
    kind = []
    if abs(du) < 0.01: kind.append("V")     # nearly pure v-step
    elif abs(dv) < 0.01: kind.append("U")   # nearly pure u-step
    else: kind.append("D")                   # diagonal
    if abs(du) > 3.0: kind.append("WRAP")
    if abs(dv) > 0.5: kind.append("LONGV")
    if kind == ["V"]:
        continue  # skip boring
    print(f"  e{i:3d}: ({outer[i][0]:7.3f},{outer[i][1]:6.3f}) -> ({outer[j][0]:7.3f},{outer[j][1]:6.3f})  du={du:7.3f} dv={dv:6.3f} {'/'.join(kind)}")

# summarize chain segments
print("\n== segment summary (coarse): walk i, mark runs")
segs = []
cur = None
for i in range(n):
    j = (i+1) % n
    du = outer[j][0] - outer[i][0]
    dv = outer[j][1] - outer[i][1]
    if abs(du) < 0.01: k = "V"
    elif abs(dv) < 0.01: k = "U"
    else: k = "D"
    if cur and cur[0] == k:
        cur[2] = i
        cur[3] += 1
    else:
        if cur: segs.append(cur)
        cur = [k, i, i, 1, du, dv]
if cur: segs.append(cur)
print(f"{'kind':5s} {'from':>5s} {'to':>5s} {'cnt':>4s}  first_du    first_dv")
for k, a, b, c, du, dv in segs:
    print(f"{k:5s} {a:5d} {b:5d} {c:4d}  {du:9.4f} {dv:9.4f}")
