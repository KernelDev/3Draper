#!/usr/bin/env python3
"""s64 diag: boundary PATH decomposition of f29 earcutr output (junction-aware)."""
from collections import defaultdict, deque

def load(idx):
    bnd, interior, tris = [], [], []
    mode = None
    for line in open(f"/tmp/s64_tri/tri_{idx:04d}_Cylinder.txt"):
        line = line.strip()
        if line in ("boundary", "interior", "tris"):
            mode = line
            continue
        if not line or line.startswith(("type=", "hole")):
            continue
        p = line.split()
        if p[0] == "b":
            bnd.append((float(p[1]), float(p[2])))
        elif p[0] == "i":
            interior.append((float(p[1]), float(p[2])))
        elif p[0] == "t":
            tris.append((int(p[1]), int(p[2]), int(p[3])))
    return bnd, interior, tris

def main():
    bnd, interior, tris = load(0)
    n_b = len(bnd)
    all_uv = bnd + interior
    used = set()
    for t in tris:
        used.update(t)
    ecnt = defaultdict(int)
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            ecnt[(min(a, b), max(a, b))] += 1
    reg_bnd = set(e for e, n in ecnt.items() if n == 1)
    adj = defaultdict(list)
    for (a, b) in reg_bnd:
        adj[a].append(b)
        adj[b].append(a)

    def lbl(i):
        return ("b" + str(i)) if i < n_b else ("i" + str(i - n_b))

    junctions = {v for v in adj if len(adj[v]) != 2}
    print(f"junctions: {[lbl(v) for v in junctions]}")

    # decompose into paths: endpoints = junctions (or cycle if none)
    paths = []
    used_e = set()
    jlist = sorted(junctions)
    for j in jlist:
        for w in adj[j]:
            e = (min(j, w), max(j, w))
            if e in used_e:
                continue
            # walk from j through w until junction or dead end
            path = [j, w]
            used_e.add(e)
            prev, cur = j, w
            while cur not in junctions:
                nxts = [x for x in adj[cur] if x != prev and (min(cur, x), max(cur, x)) not in used_e]
                if not nxts:
                    break
                nxt = nxts[0]
                used_e.add((min(cur, nxt), max(cur, nxt)))
                path.append(nxt)
                prev, cur = cur, nxt
            paths.append(path)
    # cycles (no junctions touched)
    for e in reg_bnd - used_e:
        if e in used_e:
            continue
        a, b = e
        path = [a, b]
        used_e.add(e)
        prev, cur = a, b
        while cur != a:
            nxts = [x for x in adj[cur] if x != prev and (min(cur, x), max(cur, x)) not in used_e]
            if not nxts:
                break
            nxt = nxts[0]
            used_e.add((min(cur, nxt), max(cur, nxt)))
            path.append(nxt)
            prev, cur = cur, nxt
        paths.append(path)

    print(f"boundary paths: {len(paths)}")
    for p in sorted(paths, key=len, reverse=True):
        rims = [i for i in p if i < n_b]
        print(f"  path len={len(p)}: {lbl(p[0])} .. {lbl(p[-1])}, rim {len(rims)}: {sorted(rims)[:40]}")

    # gap fill: find path from b0 to b32 avoiding b1..b31
    start, goal = 0, 32
    q = deque([(start, [start])])
    seen = {start}
    path = None
    while q:
        v, p = q.popleft()
        if v == goal:
            path = p
            break
        for w in adj[v]:
            if w in seen or 1 <= w <= 31:
                continue
            seen.add(w)
            q.append((w, p + [w]))
    if path:
        print(f"\npath b0->b32 avoiding unused rim: len={len(path)}")
        print("  " + " -> ".join(lbl(i) for i in path))
        gap = path + list(range(31, 0, -1))
        pts = [all_uv[i] for i in gap]
        A = 0.0
        for k in range(len(pts)):
            x1, y1 = pts[k]
            x2, y2 = pts[(k + 1) % len(pts)]
            A += x1 * y2 - x2 * y1
        print(f"gap polygon: {len(gap)} verts, signed area={A/2:.6f} (abs {abs(A)/2:.6f})")
    else:
        print("\nno path b0->b32 avoiding unused rim segment")

if __name__ == "__main__":
    main()
