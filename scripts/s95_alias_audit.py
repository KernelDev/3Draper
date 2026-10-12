#!/usr/bin/env python3
"""s95: аудит ошибочных алиасов edge cache для drill SLEEVE.

Сверяет EDGECOL (svp/evp каждого step_id = топологические вершины) с
EDGEKEY (entry_first/entry_last = фактическая геометрия из кэша).
Алиас, у которого entry-геометрия не совпадает с svp/evp (в смысле
"та же пара точек в любом порядке"), возвращает ЧУЖОЙ полилайн.
"""
import re, sys
from collections import defaultdict

edcol = "/home/z/my-project/s95_edgecol_stderr.txt"
edkey = "/home/z/my-project/s95_edgekey_stderr.txt"

# sid -> (svp, evp) из EDGECOL (первое вхождение)
sid_verts = {}
pat_col = re.compile(
    r"EDGECOL: face_step=\d+ edge_idx=\d+ step_id=(\d+) n_pts=\d+ .*?"
    r"svp=\(([-\d.]+),([-\d.]+),([-\d.]+)\) evp=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
for line in open(edcol):
    m = pat_col.search(line)
    if m:
        sid = int(m.group(1))
        if sid not in sid_verts:
            svp = tuple(round(float(m.group(i)), 3) for i in (2, 3, 4))
            evp = tuple(round(float(m.group(i)), 3) for i in (5, 6, 7))
            sid_verts[sid] = (svp, evp)

# EDGEKEY: sid, canon, entry_first, entry_last
pat_key = re.compile(
    r"EDGEKEY: sid=(\d+) canon=(\d+) pr=\(([-\d.]+),([-\d.]+)\) n=(\d+) "
    r"entry_first=\(([-\d.]+),([-\d.]+),([-\d.]+)\) entry_last=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")

bad = defaultdict(set)      # sid -> {canon}
same_geoms = defaultdict(set)
total = 0
aliased = 0
for line in open(edkey):
    m = pat_key.search(line)
    if not m:
        continue
    sid, canon = int(m.group(1)), int(m.group(2))
    ef = tuple(round(float(m.group(i)), 3) for i in (6, 7, 8))
    el = tuple(round(float(m.group(i)), 3) for i in (9, 10, 11))
    total += 1
    if sid != canon:
        aliased += 1
        verts = sid_verts.get(sid)
        if verts is None:
            continue
        svp, evp = verts
        # entry-геометрия должна покрывать те же концы (в любом порядке)
        ok_fwd = (dist(ef, svp) < 1e-9 and dist(el, evp) < 1e-9) if False else None
        # сравнение с округлением 0.001 из печати: допуски 2e-3
        def close(a, b):
            return all(abs(x - y) <= 2e-3 for x, y in zip(a, b))
        match = (close(ef, svp) and close(el, evp)) or (close(ef, evp) and close(el, svp))
        if not match:
            bad[sid].add((canon, ef, el))

def dist(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

print(f"EDGEKEY-записей: {total}, алиасов (sid!=canon): {aliased}")
print(f"step_id с EDGECOL-вершинами: {len(sid_verts)}")
print(f"ОШИБОЧНЫХ алиасов (entry-геометрия != топологические вершины): {len(bad)}")
for sid, entries in sorted(bad.items()):
    verts = sid_verts[sid]
    for (canon, ef, el) in entries:
        print(f"  sid={sid} canon={canon}: svp/evp={verts} entry=({ef}->{el})")

# Сколько уникальных canon задействовано
canons = set(c for entries in bad.values() for (c, _, _) in entries)
print(f"Уникальных canon-жертв: {len(canons)}: {sorted(canons)[:20]}")
