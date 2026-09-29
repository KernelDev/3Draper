#!/usr/bin/env python3
"""s65 diag 11: examine the drill seam pairs rejected by the shape guard.
For each pair: curve types, endpoint coords, midpoint distance, and the
5-point sample distance (forward + reversed).
"""
import re
import math

STEP = "/home/z/my-project/3Draper/test/drill_top.stp"

def parse():
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents

def cart(ents, ref):
    t, a = ents[ref]
    nums = []
    for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", a):
        try:
            nums.append(float(x))
        except ValueError:
            pass
    return tuple(nums[-3:]) if len(nums) >= 3 else None

def refs(ent):
    return [int(x) for x in re.findall(r"#(\d+)", ent[1])]

def curve_of(ents, ec_id):
    ec = ents.get(ec_id)
    if not ec:
        return None
    rr = refs(ec)
    geom = ents.get(rr[-1]) if rr else None
    return geom

def sample(ents, ec_id, n=5):
    """Crude sampling: for CIRCLE use param arc; for LINE lerp; for
    B_SPLINE use de Boor-lite (control polygon Bezier if 4 cps)."""
    geom = curve_of(ents, ec_id)
    if not geom:
        return None
    rr = refs(geom)
    t = geom[0]
    if t == "LINE":
        ax = ents[rr[0]]
        aref = refs(ax)
        p0 = cart(ents, aref[0])
        d = ents[aref[1]]
        dv = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", d[1])[-3:]]
        # EDGE_CURVE trimming unknown: use vertex pair for extent
        ec = ents[ec_id]
        ecrr = refs(ec)
        v1 = cart(ents, refs(ents[ecrr[0]])[0])
        v2 = cart(ents, refs(ents[ecrr[1]])[0])
        # project vertices onto line to get t range
        def proj(p):
            return sum((p[i]-p0[i])*dv[i] for i in range(3)) / sum(d*d for d in dv)
        t1, t2 = proj(v1), proj(v2)
        return [tuple(p0[i] + (t1 + (t2-t1)*f/(n-1))*dv[i] for i in range(3))
                for f in range(n)]
    if t == "CIRCLE":
        ax = ents[rr[0]]
        aref = refs(ax)
        c = cart(ents, aref[0])
        d = ents[aref[1]]
        axis = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", d[1])[-3:]]
        r = float(re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", geom[1])[-1])
        # ref dir
        rd = ents[aref[2]] if len(aref) > 2 else None
        # build local frame
        alen = math.sqrt(sum(a*a for a in axis)) or 1.0
        nvec = [a/alen for a in axis]
        # arbitrary perpendicular
        if abs(nvec[0]) < 0.9:
            uvec = [1.0, 0.0, 0.0]
        else:
            uvec = [0.0, 1.0, 0.0]
        # u = uvec - (uvec.n)n
        dot = sum(uvec[i]*nvec[i] for i in range(3))
        u = [uvec[i] - dot*nvec[i] for i in range(3)]
        ul = math.sqrt(sum(x*x for x in u)) or 1.0
        u = [x/ul for x in u]
        v = [nvec[1]*u[2]-nvec[2]*u[1], nvec[2]*u[0]-nvec[0]*u[2], nvec[0]*u[1]-nvec[1]*u[0]]
        ec = ents[ec_id]
        ecrr = refs(ec)
        p1 = cart(ents, refs(ents[ecrr[0]])[0])
        p2 = cart(ents, refs(ents[ecrr[1]])[0])
        def angle_of(p):
            dx = [p[i]-c[i] for i in range(3)]
            return math.atan2(sum(dx[i]*v[i] for i in range(3)),
                              sum(dx[i]*u[i] for i in range(3)))
        a1, a2 = angle_of(p1), angle_of(p2)
        while a2 < a1:
            a2 += 2*math.pi
        return [tuple(c[i] + r*(math.cos(a1+(a2-a1)*f/(n-1))*u[i]
                                 + math.sin(a1+(a2-a1)*f/(n-1))*v[i])
                      for i in range(3)) for f in range(n)]
    # B_SPLINE: control points
    cps = [cart(ents, r) for r in rr]
    if any(c is None for c in cps) or not cps:
        return None
    # bilinear interpolation of control polygon (crude signature)
    out = []
    for f in [0.1, 0.3, 0.5, 0.7, 0.9]:
        # bezier if 4 cps
        if len(cps) == 4:
            pts = []
            for i in range(3):
                b = ( (1-f)**3*cps[0][i] + 3*(1-f)**2*f*cps[1][i]
                     + 3*(1-f)*f**2*cps[2][i] + f**3*cps[3][i] )
                pts.append(b)
            out.append(tuple(pts))
        else:
            # lerp along polygon
            s = f*(len(cps)-1)
            k = min(int(s), len(cps)-2)
            fr = s-k
            out.append(tuple(cps[k][i]*(1-fr)+cps[k+1][i]*fr for i in range(3)))
    return out

def main():
    ents = parse()
    pairs = [(831, 833), (837, 839), (932, 934), (9766, 9535), (9744, 10630), (11864, 9631)]
    for a, b in pairs:
        ga, gb = curve_of(ents, a), curve_of(ents, b)
        ta = ga[0] if ga else "?"
        tb = gb[0] if gb else "?"
        sa = sample(ents, a)
        sb = sample(ents, b)
        if sa and sb:
            fwd = max(math.dist(p, q) for p, q in zip(sa, sb))
            rev = max(math.dist(p, q) for p, q in zip(sa, sb[::-1]))
            mid = math.dist(sa[2], sb[2])
            print(f"#{a}({ta}) vs #{b}({tb}): max_fwd={fwd:.5f} max_rev={rev:.5f} mid_dist={mid:.5f}")
        else:
            print(f"#{a}({ta}) vs #{b}({tb}): sampling failed ({sa is not None},{sb is not None})")

if __name__ == "__main__":
    main()
