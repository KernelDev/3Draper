#!/usr/bin/env python3
"""session-61: trace the actual path of the GEAR f18 top wave."""
def load(path):
    ring, lat = [], []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) != 3:
                continue
            kind, u, v = parts[0], float(parts[1]), float(parts[2])
            if kind == "ring":
                ring.append((u, v))
            elif kind == "lat":
                lat.append((u, v))
    return ring, lat

ring, lat = load("/tmp/ring_brep16033_f18_Cone.tsv")
n = len(ring)
# bottom run idx 2..33; trace from 34 onward
print("idx  u          v          (du, dv) from prev")
prev = None
for i in range(30, 190):
    u, v = ring[i]
    if prev is not None:
        du, dv = u - prev[0], v - prev[1]
        print(f"{i:4d} {u:9.6f} {v:10.6f}   du={du:+.6f} dv={dv:+.6f}")
    else:
        print(f"{i:4d} {u:9.6f} {v:10.6f}")
    prev = (u, v)
