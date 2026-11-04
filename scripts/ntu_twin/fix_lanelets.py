#!/usr/bin/env python3
"""Fixes to the NTU twin's Lanelet2 map, agreed by roadmaps 059 and 072 (2026-11-04, the tour).

A. Custom centrelines that run against their lanelet. lanelet2 keeps a `centerline` member as it
   is stored, and Autoware keeps a custom centreline, so on these lanelets its path ran from the
   lanelet's end to its start. The way's node order is reversed (23 ways on the 059 map).
B. Joins that touch but are not linked: a lanelet's end and the next one's start under 0.5 m
   apart, on different nodes, so lanelet2's routing graph has no edge (5 on the 059 map; 3855
   and 3719 could not be reached, 2132 was isolated). The downstream start nodes are replaced,
   in every way that holds them, by the upstream end nodes; a node left unused is removed.
C. A node twice in a row in a way is dropped once: B makes these where a bound already ran
   through the upstream end node (5 ways). The first version of this script (output md5
   97f10f1a) left them, and SSv2 failed on r02 near 3855 ("failed to calculate curve index").
D. Two nodes whose ele came from a DTM spike at map-minus-origin (310.1, 104.9), a canopy the
   old DTM put 6 m over the road: 1014484 15.4770 -> 12.1951 and 1014483 13.2762 -> 12.1078,
   the heights of smooth_dtm.py's fixed DTM there (ground_n11). Set only where a node still
   holds the old value (ELE_FIX).
E. The U-turn 1003951 -> 1004230 at map-minus-origin (81, 312), widened (tour leg 03). Its
   custom centreline arced at R ~3.3 m; the Model 3 turns no tighter than R 3.57 at the rear
   axle (wheel base 3.005 m, 0.7 rad), and its outer front corner then sweeps r ~6.0 m, past
   the outer bounds (5.2-5.6 m from the turn centre). Autoware stopped there ("outside
   drivable area") on CARLA and Isaac. Now, around UTURN_C: the centreline is a quintic S
   from its start into an R UTURN_R arc over 180 deg and a quintic out of it onto the old
   centreline at 1017101; the outer bounds 1003950 / 1004894 follow r UTURN_RO from the
   inner join's ray (1003952) to 200 deg, and the join node 1004044 moves onto that ray.
   The inner bounds and every node another lanelet holds are kept, except that 1004230's left
   bound takes 1008251's nodes south of y 300.2: the two were drawn twice there, crossing
   (2.2 m2 of overlap), and behavior_path_planner cuts its path at a lanelet that overlaps one
   two places back in the route (cutOverlappedLanes), so the path ended at 1003951 however
   wide the turn. The paved junction is
   ~14 m across: the swept footprint stays 0.24 m inside the lanelets, 1.16 m off CARLA's
   props, and grazes the NW grass corner (72.8-73.8, 316.5-317.7) by ~0.1 m. New nodes take
   lat/lon from a local affine fit of their neighbours' local_x/y, ele the median of the
   nearest lanelet nodes. Skipped when 1004044 already sits on its new place.

Deterministic and idempotent: the output is the input's ElementTree round trip (byte-identical
on an unchanged map) with these edits, and a map with nothing to fix is left alone.

    source /opt/autoware/1.5.0/setup.bash
    python3 scripts/ntu_twin/fix_lanelets.py MAP.osm [--out OUT.osm] [--expect-md5 MD5]

Without --out the map is rewritten in place, after a copy to MAP.before_tour_fix.osm. Prints the
input and output md5 and checkValidity and routing-graph counts before and after.
"""
import argparse
import hashlib
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

JOIN = 0.5  # m
# D: node id -> (ele as the spike left it, ele from the fixed DTM)
ELE_FIX = {"1014484": ("15.4770", "12.1951"), "1014483": ("13.2762", "12.1078")}
# E: the U-turn (map-minus-origin frame, m; the map frame is it + ORIGIN)
ORIGIN = np.array([52500.0, 67700.0])
UTURN_C = np.array([78.15, 313.2])  # turn centre
UTURN_R = 4.0  # centreline (rear axle) radius: steer atan(3.005 / 4) = 37 deg of 40
UTURN_RO = 6.8  # outer bound radius: the outer front corner's 6.35 m plus 0.45


def md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()


def load(path):
    root = ET.parse(path).getroot()
    n0 = root.find("node")
    m = lanelet2.io.load(path, UtmProjector(Origin(float(n0.get("lat")), float(n0.get("lon")))))
    tr = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                       lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, tr)


def _quintic(p0, t0, p1, t1, n):
    """Quintic Hermite from p0 (tangent t0) to p1 (t1), zero second derivative at both ends,
    so a straight runs into it with no curvature step."""
    s = np.linspace(0.0, 1.0, n)[:, None]
    h0 = 1 - 10 * s**3 + 15 * s**4 - 6 * s**5
    h1 = s - 6 * s**3 + 8 * s**4 - 3 * s**5
    h3 = -4 * s**3 + 7 * s**4 - 3 * s**5
    h5 = 10 * s**3 - 15 * s**4 + 6 * s**5
    return h0 * p0 + h1 * t0 + h3 * t1 + h5 * p1


def uturn_fix(root, ways, xy):
    """E: widen the U-turn 1003951 -> 1004230 (module doc). Edits `root` in place."""
    nodes = {n.get("id"): n for n in root.findall("node")}
    C = UTURN_C + ORIGIN
    p = {k: xy[k] for k in xy}
    ij = p["1003952"]  # the inner join, kept
    a_j = np.arctan2(ij[1] - C[1], ij[0] - C[0])
    oj = C + UTURN_RO * np.array([np.cos(a_j), np.sin(a_j)])
    if "1004044" not in p or np.linalg.norm(p["1004044"] - oj) < 0.01:
        return {"changed": False, "dropped": set(), "summary": "already widened (or absent)"}

    def at(a_deg, r):
        a = np.radians(a_deg)
        return C + r * np.array([np.cos(a), np.sin(a)])

    # centreline: entry S, the arc, exit S onto the old centreline at 1017101
    s0, e1 = p["1017071"], p["1017101"]
    d1 = (p["1017101"] - p["1017100"]) / np.linalg.norm(p["1017101"] - p["1017100"])
    pin, pout = C + [UTURN_R, 0.0], C + [-UTURN_R, 0.0]
    l1 = np.linalg.norm(pin - s0)
    entry = _quintic(s0, np.array([0.0, 1.0]) * l1, pin, np.array([0.0, 1.0]) * l1, 13)
    l2 = np.linalg.norm(e1 - pout)
    exit_ = _quintic(pout, np.array([0.0, -1.0]) * l2, e1, d1 * l2, 21)
    a_jd = np.degrees(a_j)
    arc1 = [at(a, UTURN_R) for a in np.linspace(0.0, a_jd, 6)[1:]]  # to the join, ~5 deg
    arc2 = [at(a, UTURN_R) for a in np.arange(np.ceil(a_jd / 5) * 5, 180.0, 5.0)] + [pout]
    cl1 = list(entry[1:]) + arc1[:-1]  # interior of 1017081 (its first node kept)
    cl1_end = arc1[-1]
    cl2 = arc2 + list(exit_[1:-1])  # interior of 1017116 up to 1017101
    # outer bounds
    ob1 = [at(a, UTURN_RO) for a in np.linspace(8.0, a_jd, 4)[:-1]]
    ob2 = [at(a, UTURN_RO) for a in np.arange(np.ceil(a_jd / 10) * 10, 200.1, 10.0)]

    # lat/lon from a local affine fit of local_x/y (the nodes within 40 m), ele from neighbours
    ids = [k for k in p if np.linalg.norm(p[k] - C) < 40.0]
    X = np.array([np.r_[p[k], 1.0] for k in ids])
    LL = np.array([[float(nodes[k].get("lat")), float(nodes[k].get("lon"))] for k in ids])
    fit, *_ = np.linalg.lstsq(X, LL, rcond=None)
    eles = np.array([float(nodes[k].find("tag[@k='ele']").get("v")) for k in ids])
    pts = np.array([p[k] for k in ids])
    next_id = [max(int(n.get("id")) for n in nodes.values()) + 1]

    def place(node, q):
        lat, lon = np.r_[q, 1.0] @ fit
        d = np.linalg.norm(pts - q, axis=1)
        near = np.argsort(d)[:6]
        ele = float(np.median(eles[near[d[near] < 6.0]])) if (d[near] < 6.0).any() else 0.0
        node.set("lat", f"{lat:.11f}")
        node.set("lon", f"{lon:.11f}")
        for k, v in (("local_x", f"{q[0]:.4f}"), ("local_y", f"{q[1]:.4f}"), ("ele", f"{ele:.4f}")):
            t = node.find(f"tag[@k='{k}']")
            if t is None:
                t = ET.SubElement(node, "tag", k=k)
            t.set("v", v)
        return node

    def new_node(q):
        n = ET.Element("node", id=str(next_id[0]))
        next_id[0] += 1
        place(n, q)
        root.append(n)
        return n.get("id")

    def set_way(wid, refs):
        w = ways[wid]
        for nd in w.findall("nd"):
            w.remove(nd)
        tags = list(w)
        for t in tags:
            w.remove(t)
        for r in refs:
            ET.SubElement(w, "nd", ref=r)
        for t in tags:
            w.append(t)

    old = {w: [nd.get("ref") for nd in ways[w].findall("nd")] for w in
           ("1003950", "1004894", "1017081", "1017116")}
    place(nodes["1004044"], oj)
    o1 = old["1003950"]
    set_way("1003950", o1[:o1.index("1003939") + 1] + [new_node(q) for q in ob1] + ["1004044"])
    o2 = old["1004894"]
    set_way("1004894", ["1004044"] + [new_node(q) for q in ob2] + o2[o2.index("1004053"):])
    c1 = old["1017081"]
    place(nodes[c1[-1]], cl1_end)
    set_way("1017081", [c1[0]] + [new_node(q) for q in cl1] + [c1[-1]])
    c2 = old["1017116"]
    place(nodes[c2[0]], cl1_end)
    set_way("1017116", [c2[0]] + [new_node(q) for q in cl2] + c2[c2.index("1017101"):])
    # 1004230's left bound and 1008251's (the divider between the two directions) were drawn
    # twice south of y 300.2, crossing by up to 0.6 m: 2.2 m2 of overlap, and behavior_path_
    # planner's cutOverlappedLanes ends the path at the first lanelet overlapping one two places
    # back -- 1004230 after 1008251 -- so the path stopped at 1003951's end whatever its width.
    # 1004230's left bound now runs on 1008251's nodes there (shared, as adjacent lanelets do).
    lb = [nd.get("ref") for nd in ways["1004892"].findall("nd")]
    nb = [nd.get("ref") for nd in ways["1008250"].findall("nd")]
    old["1004892"] = lb
    cut = lb.index("1003968")
    set_way("1004892", lb[:cut] + nb[:nb.index("1008249") + 1][::-1])
    kept = {nd.get("ref") for w in ways.values() for nd in w.findall("nd")}
    dropped = {r for w in old.values() for r in w} - kept
    return {"changed": True, "dropped": dropped,
            "summary": f"centre {UTURN_C.tolist()}, R {UTURN_R}, outer r {UTURN_RO}; "
                       f"{len(ob1) + len(ob2)} outer-bound and {len(cl1) + len(cl2)} centreline "
                       f"nodes new, 1004044 moved to {np.round(oj - ORIGIN, 2).tolist()}, "
                       f"{len(dropped)} nodes dropped"}


def stats(path):
    m, rg = load(path)
    L = list(m.laneletLayer)
    fol = sum(1 for l in L if rg.following(l, False))
    prv = sum(1 for l in L if rg.previous(l, False))
    iso = sorted(l.id for l in L if not rg.following(l, False) and not rg.previous(l, False))
    return (f"lanelets {len(L)}, with followers {fol}, with predecessors {prv}, "
            f"edges {sum(len(rg.following(l, False)) for l in L)}, isolated {iso}, "
            f"checkValidity errors {len(rg.checkValidity())}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("osm")
    ap.add_argument("--out")
    ap.add_argument("--expect-md5")
    a = ap.parse_args()
    h_in = md5(a.osm)
    print(f"input  {h_in}  {a.osm}")
    if a.expect_md5 and h_in != a.expect_md5:
        sys.exit(f"input md5 is not {a.expect_md5}")

    tree = ET.parse(a.osm)
    root = tree.getroot()
    xy = {}
    for n in root.findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        xy[n.get("id")] = np.array([float(t["local_x"]), float(t["local_y"])])
    ways = {w.get("id"): w for w in root.findall("way")}
    m, rg = load(a.osm)
    L = {l.id: l for l in m.laneletLayer}

    # A: centrelines against their lanelet (bounds as lanelet2 orients them)
    reversed_ = []
    for rel in root.findall("relation"):
        cl = [mb.get("ref") for mb in rel.findall("member") if mb.get("role") == "centerline"]
        lid = int(rel.get("id"))
        if not cl or lid not in L:
            continue
        w = ways[cl[0]]
        nds = w.findall("nd")
        l = L[lid]
        s = (xy[str(l.leftBound[0].id)] + xy[str(l.rightBound[0].id)]) / 2
        e = (xy[str(l.leftBound[-1].id)] + xy[str(l.rightBound[-1].id)]) / 2
        c0, c1 = xy[nds[0].get("ref")], xy[nds[-1].get("ref")]
        if np.linalg.norm(c0 - s) + np.linalg.norm(c1 - e) > np.linalg.norm(c0 - e) + np.linalg.norm(c1 - s):
            for nd in nds:
                w.remove(nd)
            tags = list(w)
            for t in tags:
                w.remove(t)
            for nd in reversed(nds):
                w.append(nd)
            for t in tags:
                w.append(t)
            reversed_.append(lid)

    # B: joins that touch, not linked
    replace, joins = {}, []
    for ia in sorted(L):
        la = L[ia]
        fol = {x.id for x in rg.following(la, False)}
        ea = (str(la.leftBound[-1].id), str(la.rightBound[-1].id))
        for ib in sorted(L):
            lb = L[ib]
            if ib == ia or ib in fol:
                continue
            sb = (str(lb.leftBound[0].id), str(lb.rightBound[0].id))
            if max(np.linalg.norm(xy[ea[0]] - xy[sb[0]]), np.linalg.norm(xy[ea[1]] - xy[sb[1]])) < JOIN:
                joins.append((ia, ib))
                for old, new in zip(sb, ea):
                    if old != new:
                        if replace.get(old, new) != new:
                            sys.exit(f"node {old} would join two ends ({replace[old]}, {new})")
                        replace[old] = new
    for w in ways.values():
        for nd in w.findall("nd"):
            if nd.get("ref") in replace:
                nd.set("ref", replace[nd.get("ref")])
    # C: a way holding a node twice in a row (B's replacement makes these where a bound already
    # ran through the upstream end node, 2-17 cm from its own start; SSv2's spline fails on the
    # zero-length segment: "catmull_rom_spline.cpp:331: failed to calculate curve index")
    dups = []
    for wid, w in sorted(ways.items()):
        last = None
        for nd in w.findall("nd"):
            if nd.get("ref") == last:
                w.remove(nd)
                dups.append(wid)
            last = nd.get("ref")
    # D: eles taken from a DTM spike
    eles = []
    for n in root.findall("node"):
        if n.get("id") in ELE_FIX:
            old, new = ELE_FIX[n.get("id")]
            t = n.find("tag[@k='ele']")
            if t is not None and t.get("v") == old:
                t.set("v", new)
                eles.append(n.get("id"))
    uturn = uturn_fix(root, ways, xy)
    used = {nd.get("ref") for w in ways.values() for nd in w.findall("nd")}
    gone = [n for n in root.findall("node") if n.get("id") in replace and n.get("id") not in used]
    gone_e = [n for n in root.findall("node") if n.get("id") in uturn["dropped"]
              and n.get("id") not in used]
    for n in gone + gone_e:
        root.remove(n)

    print(f"A: {len(reversed_)} centrelines reversed {reversed_}")
    print(f"B: {len(joins)} joins linked {joins}, {len(replace)} nodes replaced, {len(gone)} removed")
    print(f"C: {len(dups)} repeated nodes dropped, in ways {dups}")
    print(f"D: {len(eles)} node eles set from the fixed DTM {eles}")
    print(f"E: U-turn 1003951 -> 1004230 {uturn['summary']}")
    print("before:", stats(a.osm))
    if not reversed_ and not joins and not dups and not eles and not uturn["changed"]:
        print("nothing to fix; map unchanged")
        print(f"output {h_in}")
        return

    out = a.out or a.osm
    fd, tmp = tempfile.mkstemp(suffix=".osm", dir=os.path.dirname(os.path.abspath(out)))
    os.close(fd)
    tree.write(tmp, encoding="UTF-8", xml_declaration=True)
    print("after: ", stats(tmp))
    if not a.out:
        backup = a.osm[:-4] + ".before_tour_fix.osm"
        if not os.path.exists(backup):
            shutil.copy2(a.osm, backup)
            print(f"backup {backup}")
    os.replace(tmp, out)
    print(f"output {md5(out)}  {out}")


if __name__ == "__main__":
    main()
