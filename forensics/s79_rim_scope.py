#!/usr/bin/env python3
"""s79 item 2: measure the scope of the legacy spike-chain rim-edge
loss across all drill faces (TRI_INPUT dumps).

For every triangulation call: the polygon given to earcutr is
[boundary(rim), holes..., interior(steiners)...]. The TRUE rim edges
are (i, (i+1)%n_b) for i in 0..n_b — including the CLOSING edge
(n_b-1, 0). earcutr's polygon instead walks rim then jumps to the
steiner tail, so the closing rim edge can never appear in the output.

We measure per call:
  - n_b, n_interior
  - rim edges covered by output triangles (as UV-index pairs)
  - missing rim edges (count + which)
  - extra boundary edges (polygon edges that are not rim edges)
Also aggregate per label.
"""
import os
import re
from collections import defaultdict

D = "/tmp/s79_tri"
RE_HDR = re.compile(
    r"type=(\S+) forward=(\S+) n_boundary=(\d+) n_holes=(\d+) "
    r"n_interior=(\d+) n_tris=(\d+) label=(.*)")


def parse(path):
    hdr = None
    bnd, holes, inter, tris = [], [], [], []
    mode = None
    hole_idx = -1
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("type="):
                hdr = RE_HDR.match(line)
                continue
            if line == "boundary":
                mode = "b"
                continue
            if line.startswith("hole "):
                hole_idx = int(line.split()[1])
                mode = "h"
                holes.append([])
                continue
            if line == "interior":
                mode = "i"
                continue
            if line.startswith("t "):
                tris.append([int(x) for x in line.split()[1:4]])
                continue
            if line.startswith("b ") and mode == "b":
                _, u, v = line.split()
                bnd.append((float(u), float(v)))
            elif line.startswith("h ") and mode == "h":
                _, u, v = line.split()
                holes[-1].append((float(u), float(v)))
            elif line.startswith("i ") and mode == "i":
                _, u, v = line.split()
                inter.append((float(u), float(v)))
    return hdr, bnd, holes, inter, tris


def main():
    files = sorted(os.listdir(D))
    stats = []
    for fn in files:
        hdr, bnd, holes, inter, tris = parse(os.path.join(D, fn))
        if not hdr:
            continue
        n_b = len(bnd)
        n_h = sum(len(h) for h in holes)
        n_i = len(inter)
        if n_b < 3:
            continue
        # rim edges in polygon-index space: boundary occupies 0..n_b-1
        rim = set()
        for i in range(n_b):
            rim.add((i, (i + 1) % n_b))
        # edges used by output triangles
        used = set()
        for t in tris:
            for k in range(3):
                a, b = t[k], t[(k + 1) % 3]
                used.add((a, b))
                used.add((b, a))
        # hole start offsets
        off = n_b
        hole_starts = []
        for h in holes:
            hole_starts.append(off)
            off += len(h)
        # polygon edges (the boundary earcutr sees): outer ring =
        # [0..n_b) + interior tail (if no holes, interior extends the
        # outer ring; with holes it extends the LAST hole ring)
        outer = list(range(n_b))
        if n_h == 0:
            outer += list(range(n_b, n_b + n_i))
            poly_edges = [(outer[k], outer[(k + 1) % len(outer)])
                          for k in range(len(outer))]
        else:
            poly_edges = [(i, (i + 1) % n_b) for i in range(n_b)]
            for hj, h in enumerate(holes):
                s = hole_starts[hj]
                ring = list(range(s, s + len(h)))
                if hj == len(holes) - 1 and n_i > 0:
                    ring += list(range(n_b + n_h, n_b + n_h + n_i))
                poly_edges += [(ring[k], ring[(k + 1) % len(ring)])
                               for k in range(len(ring))]
        # rim coverage
        missing_rim = [e for e in rim if e not in used]
        # extra boundary edges = polygon edges not rim
        extra = [e for e in poly_edges
                 if e not in rim and e not in used]
        # the lost closing edge specifically
        closing = (n_b - 1, 0)
        closing_lost = closing not in used and n_i > 0 and n_h == 0
        stats.append({
            "file": fn, "type": hdr.group(1),
            "label": hdr.group(7).strip(),
            "n_b": n_b, "n_h": n_h, "n_i": n_i,
            "n_tris": len(tris),
            "missing_rim": len(missing_rim),
            "missing_list": missing_rim[:5],
            "extra_bnd": len(extra),
            "closing_lost": closing_lost,
        })

    with_steiners = [s for s in stats if s["n_i"] > 0]
    print(f"total calls: {len(stats)}, with interior Steiners: "
          f"{len(with_steiners)}")
    lost = [s for s in stats if s["missing_rim"] > 0]
    print(f"calls with MISSING rim edges: {len(lost)}")
    cl = [s for s in stats if s["closing_lost"]]
    print(f"calls losing the CLOSING rim edge (spike-chain, no holes): "
          f"{len(cl)}")
    print()
    # aggregate by label
    by_label = defaultdict(list)
    for s in lost:
        by_label[s["label"]].append(s)
    print("=== calls with missing rim edges, by label ===")
    for lbl, lst in sorted(by_label.items(),
                           key=lambda kv: -len(kv[1]))[:25]:
        tot_miss = sum(x["missing_rim"] for x in lst)
        n_i_sum = sum(x["n_i"] for x in lst) / len(lst)
        print(f"  {lbl}: {len(lst)} calls, {tot_miss} rim edges lost, "
              f"avg n_i={n_i_sum:.0f}")
    print()
    print("=== sample of worst calls ===")
    for s in sorted(lost, key=lambda x: -x["missing_rim"])[:15]:
        print(f"  {s['file']} {s['type']} n_b={s['n_b']} n_i={s['n_i']} "
              f"missing={s['missing_rim']} extra={s['extra_bnd']} "
              f"{s['label'][:60]}")
        print(f"    missing edges: {s['missing_list']}")


if __name__ == "__main__":
    main()
