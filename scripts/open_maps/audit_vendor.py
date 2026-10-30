#!/usr/bin/env python3
"""Audit a generated map's lane centres against the vendor's surveyed lanelets (roadmap 062, O4;
the vendor map is ground truth and never an input).

    scripts/open_maps/audit_vendor.py <generated lanelet2_map.osm> <vendor lanelet2_map.osm>
        [--json out.json] [--shift dx,dy]

The vendor's lanelets (r01, r02: NTU's campus routes) were drawn on their own route point
clouds, which drift up to 0.3 m against the scan (019); they are in 059's map frame M and are
moved to the orthophotos' frame V by +(-0.54, +0.20), where the generated map (true
MGRS-local) is taken to lie. Every metre of a vendor lanelet's centre line is matched to the
nearest generated road lane centre heading the same way (within 30 deg). Reported: how much of
the vendor's network has a generated lane within 5 m (coverage), and the lateral offsets there
(median, 90th percentile, signed mean: a systematic shift of the whole map shows in it).
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree

VENDOR_SHIFT = (-0.54, 0.20)


def tags(e):
    return {t.get("k"): t.get("v") for t in e.findall("tag")}


def centres(path, shift, skip_connectors=False):
    r = ET.parse(path).getroot()
    xy = {}
    for n in r.findall("node"):
        t = tags(n)
        if "local_x" in t:
            xy[n.get("id")] = (float(t["local_x"]) + shift[0], float(t["local_y"]) + shift[1])
    ways = {w.get("id"): [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
            for w in r.findall("way")}
    out = []
    for rel in r.findall("relation"):
        t = tags(rel)
        if t.get("type") != "lanelet" or t.get("subtype", "road") != "road":
            continue
        if skip_connectors and t.get("swb:speed_source") == "connector":
            continue
        m = {x.get("role"): x.get("ref") for x in rel.findall("member") if x.get("type") == "way"}
        l, rr = ways.get(m.get("left"), []), ways.get(m.get("right"), [])
        if len(l) < 2 or len(rr) < 2:
            continue
        L, R = LineString(l), LineString(rr)
        n = max(2, int(max(L.length, R.length)))
        c = [((L.interpolate(i / n, normalized=True).x + R.interpolate(i / n, normalized=True).x) / 2,
              (L.interpolate(i / n, normalized=True).y + R.interpolate(i / n, normalized=True).y) / 2) for i in range(n + 1)]
        out.append((LineString(c), t))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("vendor")
    ap.add_argument("--json")
    ap.add_argument("--shift", default="0,0")
    a = ap.parse_args()
    gen = centres(a.map, tuple(map(float, a.shift.split(","))))
    ven = centres(a.vendor, VENDOR_SHIFT)
    tree = STRtree([g for g, _ in gen])
    total = matched = 0
    offs = []
    for line, _ in ven:
        for i in range(int(line.length)):
            p = line.interpolate(i + 0.5)
            q = line.interpolate(min(line.length, i + 1.0))
            o = line.interpolate(max(0.0, i))
            h = math.atan2(q.y - o.y, q.x - o.x)
            total += 1
            best = None
            for j in tree.query(p.buffer(5.0)):
                g = gen[int(j)][0]
                s = g.project(p)
                a1, b1 = g.interpolate(max(0, s - 0.5)), g.interpolate(min(g.length, s + 0.5))
                hg = math.atan2(b1.y - a1.y, b1.x - a1.x)
                if abs((hg - h + math.pi) % (2 * math.pi) - math.pi) > math.radians(30):
                    continue
                c = g.interpolate(s)
                d = (c.x - p.x) * -math.sin(h) + (c.y - p.y) * math.cos(h)   # + left of vendor
                if abs(d) <= 5.0 and (best is None or abs(d) < abs(best)):
                    best = d
            if best is not None:
                matched += 1
                offs.append(best)
    f = np.array(offs)
    rep = {"vendor_centre_m": total, "with_generated_lane_within_5m": matched,
           "coverage": round(matched / max(total, 1), 3),
           "abs_offset_median_m": round(float(np.median(np.abs(f))), 2) if f.size else None,
           "abs_offset_p90_m": round(float(np.percentile(np.abs(f), 90)), 2) if f.size else None,
           "within_0.5m": round(float((np.abs(f) < 0.5).mean()), 3) if f.size else None,
           "within_1.0m": round(float((np.abs(f) < 1.0).mean()), 3) if f.size else None,
           "signed_mean_m (+: generated left of vendor)": round(float(f.mean()), 2) if f.size else None}
    print(json.dumps(rep, indent=1))
    if a.json:
        Path(a.json).write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
