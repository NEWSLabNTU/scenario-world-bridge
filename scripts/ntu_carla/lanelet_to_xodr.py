#!/usr/bin/env python3
"""The NTU campus's road network for CARLA (roadmap 072 C6): OpenDRIVE 1.4 from the Lanelet2
map that roadmap 059's `register_lanelets.py` wrote.

    lanelet_to_xodr.py LANELET2_OSM OUT.xodr [--origin 52500,67700] [--name NTU]

Each road lanelet becomes one OpenDRIVE road with one driving lane (id -1) whose centre is the
lanelet's centreline: its own `role="centerline"` way where it has one (059 kept those for the
widened curves), otherwise the mean of its bounds resampled at equal arc-length fractions, as
Autoware computes it. The road's reference line is that centreline, a chain of `line`
geometries; a `laneOffset` of half the width puts the lane's centre on it. The width follows the
distance between the bounds, the elevation the centreline's z (CARLA takes its waypoints' and
spawn points' height from the elevation profile; the campus road is ~12 m up).

Links: lanelet A precedes B where A's bounds end on the nodes B's start on (Lanelet2's own rule).
Where lanes split or merge, the lanelets of the split (a predecessor with several successors)
or the merge (a successor with several predecessors) are a junction's connecting roads, grouped
into junctions by the roads they touch; every other lanelet is an ordinary road. Neighbouring
lanes of one carriageway stay separate one-lane roads: CARLA's waypoints and spawn points need
no more, and Autoware routes on the Lanelet2.

Frame: the map frame (MGRS 51RUH local, the Autoware map's) less `--origin`, the same origin as
the meshes, so float32 keeps millimetres; CARLA's world is this frame with y negated. The
geoReference is a transverse Mercator at the origin's latitude and longitude (from the nodes'
lat/lon), which is what CARLA's GNSS reads.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

STEP = 1.0  # m between centreline samples where a lanelet has no centreline of its own


def parse(path):
    root = ET.parse(path).getroot()
    nodes, latlon = {}, {}
    for n in root.iter("node"):
        t = {k.get("k"): k.get("v") for k in n.findall("tag")}
        nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]), float(t.get("ele", 0.0)))
        latlon[n.get("id")] = (float(n.get("lat")), float(n.get("lon")))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.iter("way")}
    lanelets = {}
    for r in root.iter("relation"):
        t = {k.get("k"): k.get("v") for k in r.findall("tag")}
        if t.get("type") != "lanelet" or t.get("subtype", "road") != "road":
            continue
        m = {x.get("role"): x.get("ref") for x in r.findall("member") if x.get("type") == "way"}
        lanelets[r.get("id")] = {"left": list(ways[m["left"]]), "right": list(ways[m["right"]]),
                                 "centre": list(ways[m["centerline"]]) if "centerline" in m else None,
                                 "tags": t}
    return nodes, latlon, lanelets


def resample(p, n):
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]
    t = np.linspace(0.0, d[-1], n)
    return np.stack([np.interp(t, d, p[:, k]) for k in range(p.shape[1])], axis=1)


def geometry(lt, nodes):
    """(centreline Nx3, width N) of a lanelet, the left bound turned to run with the right one
    as Lanelet2 does on load."""
    L = np.array([nodes[i] for i in lt["left"]])
    R = np.array([nodes[i] for i in lt["right"]])
    if np.linalg.norm(L[0, :2] - R[-1, :2]) + np.linalg.norm(L[-1, :2] - R[0, :2]) < \
            np.linalg.norm(L[0, :2] - R[0, :2]) + np.linalg.norm(L[-1, :2] - R[-1, :2]):
        L = L[::-1]
        lt["left"] = lt["left"][::-1]
    length = max(np.linalg.norm(np.diff(L[:, :2], axis=0), axis=1).sum(),
                 np.linalg.norm(np.diff(R[:, :2], axis=0), axis=1).sum())
    n = max(int(math.ceil(length / STEP)) + 1, 2)
    Lr, Rr = resample(L, n), resample(R, n)
    if lt["centre"]:
        C = np.array([nodes[i] for i in lt["centre"]])
        if np.linalg.norm(C[0, :2] - (Lr[0, :2] + Rr[0, :2]) / 2) > np.linalg.norm(C[-1, :2] - (Lr[0, :2] + Rr[0, :2]) / 2):
            C = C[::-1]
        C = resample(C, n)
    else:
        C = (Lr + Rr) / 2
    W = np.linalg.norm(Lr[:, :2] - Rr[:, :2], axis=1)
    # drop repeated points (a zero-length line has no heading)
    keep = np.r_[True, np.linalg.norm(np.diff(C[:, :2], axis=0), axis=1) > 1e-3]
    return C[keep], W[keep]


def links(lanelets):
    """successors and predecessors by shared bound end nodes."""
    starts = defaultdict(list)
    for i, lt in lanelets.items():
        starts[(lt["left"][0], lt["right"][0])].append(i)
    succ, pred = defaultdict(list), defaultdict(list)
    for i, lt in lanelets.items():
        for j in starts.get((lt["left"][-1], lt["right"][-1]), []):
            if j != i:
                succ[i].append(j)
                pred[j].append(i)
    return succ, pred


def junctions(lanelets, succ, pred):
    """{lanelet: junction id} for the lanelets of a split or a merge, grouped by union-find over
    the roads they share."""
    conn = {i for i in lanelets
            if any(len(succ[p]) > 1 for p in pred[i]) or any(len(pred[s]) > 1 for s in succ[i])}
    parent = {i: i for i in conn}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    touch = defaultdict(list)
    for i in conn:
        for r in pred[i] + succ[i]:
            touch[r].append(i)
    for group in touch.values():
        for a, b in zip(group, group[1:]):
            parent[find(a)] = find(b)
    ids, out = {}, {}
    for i in sorted(conn, key=int):
        out[i] = ids.setdefault(find(i), 1000 + len(ids))
    return out


def origin_geo(nodes, latlon, origin):
    """lat/lon of the origin, by a transverse Mercator fitted through the nodes' lat/lon."""
    from pyproj import Transformer

    tr = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)
    ids = list(nodes)[:: max(1, len(nodes) // 200)]
    off = np.median([np.subtract(tr.transform(latlon[i][1], latlon[i][0]), nodes[i][:2]) for i in ids], axis=0)
    lon, lat = tr.transform(origin[0] + off[0], origin[1] + off[1], direction="INVERSE")
    return lat, lon


def write(path, name, lanelets, geo, succ, pred, junc, origin, latlon0):
    ids = {i: n + 1 for n, i in enumerate(sorted(lanelets, key=int))}  # road ids 1..N
    od = ET.Element("OpenDRIVE")
    h = ET.SubElement(od, "header", revMajor="1", revMinor="4", name=name, version="1.00",
                      date="2026-11-03", north="0", south="0", east="0", west="0",
                      vendor="scenario-world-bridge roadmap 072")
    ET.SubElement(h, "geoReference").text = (
        f"+proj=tmerc +lat_0={latlon0[0]:.10f} +lon_0={latlon0[1]:.10f} +k=1 +x_0=0 +y_0=0 "
        "+ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    for i in sorted(lanelets, key=int):
        C, W = geo[i]
        P = C[:, :2] - origin
        seg = np.diff(P, axis=0)
        ds = np.linalg.norm(seg, axis=1)
        s = np.r_[0.0, np.cumsum(ds)]
        road = ET.SubElement(od, "road", name=f"lanelet {i}", length=f"{s[-1]:.4f}",
                             id=str(ids[i]), junction=str(junc.get(i, -1)))
        ln = ET.SubElement(road, "link")
        for kind, nb, contact in (("predecessor", pred[i], "end"), ("successor", succ[i], "start")):
            if not nb:
                continue
            if i in junc:  # a connecting road links to roads
                j = nb[0]
                ET.SubElement(ln, kind, elementType="road", elementId=str(ids[j]), contactPoint=contact)
            elif len(nb) == 1 and nb[0] not in junc:
                ET.SubElement(ln, kind, elementType="road", elementId=str(ids[nb[0]]), contactPoint=contact)
            else:
                ET.SubElement(ln, kind, elementType="junction", elementId=str(junc[nb[0]]))
        tags = lanelets[i]["tags"]
        if tags.get("speed_limit"):
            t = ET.SubElement(road, "type", s="0.0", type="town")
            ET.SubElement(t, "speed", max=tags["speed_limit"].split()[0], unit="km/h")
        pv = ET.SubElement(road, "planView")
        for k in range(len(seg)):
            g = ET.SubElement(pv, "geometry", s=f"{s[k]:.4f}", x=f"{P[k, 0]:.4f}", y=f"{P[k, 1]:.4f}",
                              hdg=f"{math.atan2(seg[k, 1], seg[k, 0]):.6f}", length=f"{ds[k]:.4f}")
            ET.SubElement(g, "line")
        ep = ET.SubElement(road, "elevationProfile")
        z = C[:, 2]
        for k in range(len(seg)):
            ET.SubElement(ep, "elevation", s=f"{s[k]:.4f}", a=f"{z[k]:.4f}",
                          b=f"{(z[k + 1] - z[k]) / ds[k]:.6f}", c="0", d="0")
        ET.SubElement(road, "lateralProfile")
        lanes = ET.SubElement(road, "lanes")
        for k in range(len(seg)):
            ET.SubElement(lanes, "laneOffset", s=f"{s[k]:.4f}", a=f"{W[k] / 2:.4f}",
                          b=f"{(W[k + 1] - W[k]) / 2 / ds[k]:.6f}", c="0", d="0")
        sec = ET.SubElement(lanes, "laneSection", s="0.0")
        ET.SubElement(ET.SubElement(sec, "center"), "lane", id="0", type="none", level="false")
        right = ET.SubElement(sec, "right")
        lane = ET.SubElement(right, "lane", id="-1", type="driving", level="false")
        ll = ET.SubElement(lane, "link")
        if pred[i]:
            ET.SubElement(ll, "predecessor", id="-1")
        if succ[i]:
            ET.SubElement(ll, "successor", id="-1")
        for k in range(len(seg)):
            ET.SubElement(lane, "width", sOffset=f"{s[k]:.4f}", a=f"{W[k]:.4f}",
                          b=f"{(W[k + 1] - W[k]) / ds[k]:.6f}", c="0", d="0")
        ET.SubElement(lane, "roadMark", sOffset="0", type="solid", weight="standard", color="standard",
                      width="0.15", laneChange="none")
    byj = defaultdict(list)
    for i, j in junc.items():
        byj[j].append(i)
    for j, members in sorted(byj.items()):
        je = ET.SubElement(od, "junction", id=str(j), name=f"junction {j}")
        cid = 0
        for i in members:
            for p in pred[i]:
                if p in junc:
                    continue
                c = ET.SubElement(je, "connection", id=str(cid), incomingRoad=str(ids[p]),
                                  connectingRoad=str(ids[i]), contactPoint="start")
                ET.SubElement(c, "laneLink", **{"from": "-1", "to": "-1"})
                cid += 1
    ET.indent(od)
    ET.ElementTree(od).write(path, encoding="UTF-8", xml_declaration=True)
    return ids


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("osm")
    ap.add_argument("out")
    ap.add_argument("--origin", default="52500,67700")
    ap.add_argument("--name", default="NTU")
    ap.add_argument("--route", default="", help="r01 (lanelet ids from 1 000 000) or r02 (below): only its roads")
    a = ap.parse_args(argv)
    origin = np.array([float(v) for v in a.origin.split(",")])
    nodes, latlon, lanelets = parse(a.osm)
    if a.route:
        lanelets = {i: lt for i, lt in lanelets.items() if (a.route == "r01") == (int(i) >= 1000000)}
    geo = {i: geometry(lt, nodes) for i, lt in lanelets.items()}
    succ, pred = links(lanelets)
    junc = junctions(lanelets, succ, pred)
    latlon0 = origin_geo(nodes, latlon, origin)
    write(a.out, a.name, lanelets, geo, succ, pred, junc, origin, latlon0)
    n_j = len(set(junc.values()))
    print(f"{len(lanelets)} lanelets -> {len(lanelets)} roads, {len(junc)} in {n_j} junctions; "
          f"{sum(1 for i in lanelets if not pred[i])} with no predecessor, "
          f"{sum(1 for i in lanelets if not succ[i])} with no successor; "
          f"origin lat/lon {latlon0[0]:.8f}, {latlon0[1]:.8f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
