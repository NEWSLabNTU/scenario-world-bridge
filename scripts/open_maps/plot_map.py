#!/usr/bin/env python3
"""Draw a Lanelet2 map for review: road lanelets coloured by whether they lie in the largest
strongly connected part of the lane graph, junction paths by `turn_direction`, dead ends and
lanelets with no predecessor marked, traffic-light stop lines in red.

    scripts/open_maps/plot_map.py <lanelet2_map.osm> <out.png> [--box x0,y0,x1,y1]
"""
import argparse
import xml.etree.ElementTree as ET
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def tags(e):
    return {t.get("k"): t.get("v") for t in e.findall("tag")}


def load(path):
    r = ET.parse(path).getroot()
    xy = {}
    for n in r.findall("node"):
        t = tags(n)
        xy[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): ([x.get("ref") for x in w.findall("nd")], tags(w)) for w in r.findall("way")}
    L = {}
    for rel in r.findall("relation"):
        t = tags(rel)
        if t.get("type") != "lanelet":
            continue
        m = {x.get("role"): x.get("ref") for x in rel.findall("member") if x.get("type") == "way"}
        L[rel.get("id")] = (t, ways[m["left"]][0], ways[m["right"]][0])
    return xy, ways, L


def scc(succ):
    idx, low, on, st, comps, i = {}, {}, set(), [], [], [0]
    for v0 in succ:
        if v0 in idx:
            continue
        work = [(v0, iter(succ[v0]))]
        idx[v0] = low[v0] = i[0]
        i[0] += 1
        st.append(v0)
        on.add(v0)
        while work:
            v, it = work[-1]
            nxt = next(it, None)
            if nxt is not None:
                if nxt not in idx:
                    idx[nxt] = low[nxt] = i[0]
                    i[0] += 1
                    st.append(nxt)
                    on.add(nxt)
                    work.append((nxt, iter(succ[nxt])))
                elif nxt in on:
                    low[v] = min(low[v], idx[nxt])
            else:
                work.pop()
                if work:
                    low[work[-1][0]] = min(low[work[-1][0]], low[v])
                if low[v] == idx[v]:
                    comp = []
                    while True:
                        w = st.pop()
                        on.discard(w)
                        comp.append(w)
                        if w == v:
                            break
                    comps.append(comp)
    return sorted(comps, key=len, reverse=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("out")
    ap.add_argument("--box")
    a = ap.parse_args()
    xy, ways, L = load(a.map)
    road = {k: v for k, v in L.items() if v[0].get("subtype") == "road"}
    bs = defaultdict(list)
    for k, (t, l, r) in road.items():
        bs[(l[0], r[0])].append(k)
    succ = {k: bs.get((l[-1], r[-1]), []) for k, (t, l, r) in road.items()}
    pred = defaultdict(int)
    for v in succ.values():
        for x in v:
            pred[x] += 1
    big = set(scc(succ)[0])
    fig, ax = plt.subplots(figsize=(22, 16), dpi=110)
    colours = {"left": "tab:blue", "right": "tab:orange", "straight": "tab:green"}
    for k, (t, l, r) in road.items():
        cx = [(xy[p][0] + xy[q][0]) / 2 for p, q in zip(l, r)] if len(l) == len(r) else \
            [xy[p][0] for p in l]
        cy = [(xy[p][1] + xy[q][1]) / 2 for p, q in zip(l, r)] if len(l) == len(r) else \
            [xy[p][1] for p in l]
        td = t.get("turn_direction")
        col = colours.get(td, "0.25" if k in big else "tab:red")
        lw = 0.8 if td else (0.9 if k in big else 1.3)
        for b in (l, r):
            ax.plot([xy[p][0] for p in b], [xy[p][1] for p in b], color=col, lw=lw * 0.6, alpha=0.8)
        if not succ[k]:
            p = xy[l[-1]]
            ax.plot(p[0], p[1], "x", color="crimson", ms=4)
        if not pred[k]:
            p = xy[l[0]]
            ax.plot(p[0], p[1], "o", mfc="none", color="purple", ms=3)
    for wid, (nids, t) in ways.items():
        if t.get("type") == "stop_line":
            ax.plot([xy[p][0] for p in nids], [xy[p][1] for p in nids], color="red", lw=2)
    for k, (t, l, r) in L.items():
        if t.get("subtype") == "crosswalk":
            ax.fill([xy[p][0] for p in l] + [xy[p][0] for p in r[::-1]],
                    [xy[p][1] for p in l] + [xy[p][1] for p in r[::-1]], color="gold", alpha=0.5)
    if a.box:
        x0, y0, x1, y1 = map(float, a.box.split(","))
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_title(f"{a.map}: {len(road)} road lanelets, largest strongly connected part {len(big)} "
                 f"(dark grey; red: outside it); x: dead end, o: no predecessor; "
                 f"paths: left blue, right orange, straight green")
    fig.tight_layout()
    fig.savefig(a.out)


if __name__ == "__main__":
    main()
