#!/usr/bin/env python3
"""s95: классификация ошибочных алиасов — катастрофические vs near-miss."""
import re
from collections import defaultdict

edcol = "/home/z/my-project/s95_edgecol_stderr.txt"
edkey = "/home/z/my-project/s95_edgekey_stderr.txt"

# sid -> (face_step, svp, evp): первое вхождение EDGECOL
sid_info = {}
pat_col = re.compile(
    r"EDGECOL: face_step=(\d+) edge_idx=\d+ step_id=(\d+) n_pts=\d+ .*?"
    r"svp=\(([-\d.]+),([-\d.]+),([-\d.]+)\) evp=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
for line in open(edcol):
    m = pat_col.search(line)
    if m:
        sid = int(m.group(2))
        if sid not in sid_info:
            svp = tuple(round(float(m.group(i)), 3) for i in (3, 4, 5))
            evp = tuple(round(float(m.group(i)), 3) for i in (6, 7, 8))
            sid_info[sid] = (int(m.group(1)), svp, evp)

pat_key = re.compile(
    r"EDGEKEY: sid=(\d+) canon=(\d+) pr=\(([-\d.]+),([-\d.]+)\) n=(\d+) "
    r"entry_first=\(([-\d.]+),([-\d.]+),([-\d.]+)\) entry_last=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")

def d3(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

catastrophic = []   # чужая геометрия: ни один конец не совпал
near_miss = []      # концы близки (<=0.016), но не идентичны
for line in open(edkey):
    m = pat_key.search(line)
    if not m:
        continue
    sid, canon = int(m.group(1)), int(m.group(2))
    if sid == canon or sid not in sid_info:
        continue
    face, svp, evp = sid_info[sid]
    ef = tuple(round(float(m.group(i)), 3) for i in (6, 7, 8))
    el = tuple(round(float(m.group(i)), 3) for i in (9, 10, 11))
    m_fwd = max(d3(ef, svp), d3(el, evp))
    m_rev = max(d3(ef, evp), d3(el, svp))
    mism = min(m_fwd, m_rev)
    if mism <= 2.2e-3:   # печать с 3 знаками: почти идентично
        continue
    rec = (sid, canon, face, round(mism, 4), svp, evp, ef, el)
    if mism > 0.02:
        catastrophic.append(rec)
    else:
        near_miss.append(rec)

print(f"КАТАСТРОФИЧЕСКИХ (mismatch>0.02, чужая геометрия): {len(catastrophic)}")
by_face_c = defaultdict(int)
for (sid, canon, face, mism, *_ ) in catastrophic:
    by_face_c[face] += 1
print("  по граням:", dict(sorted(by_face_c.items())))
print()
print(f"NEAR-MISS (0.002<mismatch<=0.02): {len(near_miss)}")
by_face_n = defaultdict(int)
for (sid, canon, face, mism, *_) in near_miss:
    by_face_n[face] += 1
top = sorted(by_face_n.items(), key=lambda kv: -kv[1])[:15]
print("  топ граней:", top)
print()
print("Первые 15 катастрофических:")
for rec in catastrophic[:15]:
    sid, canon, face, mism, svp, evp, ef, el = rec
    print(f"  sid={sid} canon={canon} face={face} mism={mism}: svp/evp={svp}->{evp} entry={ef}->{el}")
