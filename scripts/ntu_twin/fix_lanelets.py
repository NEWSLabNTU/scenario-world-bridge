#!/usr/bin/env python3
"""Two fixes to the NTU twin's Lanelet2 map, agreed by roadmaps 059 and 072 (2026-11-04, the tour).

A. Custom centrelines that run against their lanelet. lanelet2 keeps a `centerline` member as it
   is stored, and Autoware keeps a custom centreline, so on these lanelets its path ran from the
   lanelet's end to its start. The way's node order is reversed (23 ways on the 059 map).
B. Joins that touch but are not linked: a lanelet's end and the next one's start under 0.5 m
   apart, on different nodes, so lanelet2's routing graph has no edge (5 on the 059 map; 3855
   and 3719 could not be reached, 2132 was isolated). The downstream start nodes are replaced,
   in every way that holds them, by the upstream end nodes; a node left unused is removed.

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


def md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()


def load(path):
    root = ET.parse(path).getroot()
    n0 = root.find("node")
    m = lanelet2.io.load(path, UtmProjector(Origin(float(n0.get("lat")), float(n0.get("lon")))))
    tr = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                       lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, tr)


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
    used = {nd.get("ref") for w in ways.values() for nd in w.findall("nd")}
    gone = [n for n in root.findall("node") if n.get("id") in replace and n.get("id") not in used]
    for n in gone:
        root.remove(n)

    print(f"A: {len(reversed_)} centrelines reversed {reversed_}")
    print(f"B: {len(joins)} joins linked {joins}, {len(replace)} nodes replaced, {len(gone)} removed")
    print("before:", stats(a.osm))
    if not reversed_ and not joins:
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
