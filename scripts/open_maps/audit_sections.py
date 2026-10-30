#!/usr/bin/env python3
"""Audit lane counts and lane line positions road by road against the paint in 059's 2 cm
orthophotos (roadmap 062, O4; the orthophotos are never an input to the map).

    scripts/open_maps/audit_sections.py <lanelet2_map.osm> <ortho dir> [--json out.json]
        [--shift dx,dy] [--min-length 20] [--out-dir <dir for profile plots>]

For each road piece and direction (lanes side by side, joined by the lines they share), the
intensity across the road (+-14 m from its leftmost line) is averaged along it wherever the
ground was observed, in 20 m windows, on the RGB orthophoto's brightness. A painted line
parallel to the road makes a peak in that profile; stray marks wash out. Reported per segment: the painted lines found, our lane
lines, each of ours matched to the nearest paint peak (its offset), and how many of our lines
have paint within 0.3 m and 0.6 m. Only segments with at least `--min-length` m observed are
scored.
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

from PIL import Image

from audit_lines import N, Ortho, PX, tags


class RgbOrtho(Ortho):
    """The RGB orthophoto's brightness: on 新生南路 the paint shows there and hardly at all in
    059's intensity (mean LAS intensity falls with range and saturates)."""

    def tile(self, ix, iy):
        k = (ix, iy)
        if k not in self.cache:
            fi, fm = self.d / f"rgb_{ix}_{iy}.jpg", self.d / f"mask_{ix}_{iy}.png"
            if fi.exists() and fm.exists():
                m = np.asarray(Image.open(fm)) > 127
                m = m.reshape(N // 10, 10, N // 10, 10).any((1, 3)).repeat(10, 0).repeat(10, 1)
                self.cache[k] = (np.asarray(Image.open(fi).convert("L")).astype(np.float32), m)
            else:
                self.cache[k] = None
            if len(self.cache) > 40:
                self.cache.pop(next(iter(self.cache)))
        return self.cache[k]

HALF = 14.0


def segments(path, shift):
    """Groups of side-by-side lanes: road lanelets joined by a shared bound way (a dashed line
    between two lanes of one direction). Each group is one direction of one road piece."""
    r = ET.parse(path).getroot()
    xy = {}
    for n in r.findall("node"):
        t = tags(n)
        if "local_x" in t:
            xy[n.get("id")] = (float(t["local_x"]) + shift[0], float(t["local_y"]) + shift[1])
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in r.findall("way")}
    lan = {}
    for rel in r.findall("relation"):
        t = tags(rel)
        if t.get("type") != "lanelet" or t.get("subtype") != "road" or "swb:osm_way" not in t:
            continue
        m = {x.get("role"): x.get("ref") for x in rel.findall("member") if x.get("type") == "way"}
        lan[rel.get("id")] = (t, m["left"], m["right"])
    by_way = defaultdict(list)
    for k, (t, lw, rw) in lan.items():
        by_way[lw].append(k)
        by_way[rw].append(k)
    parent = {k: k for k in lan}

    def find(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k
    for ks in by_way.values():
        for k in ks[1:]:
            parent[find(k)] = find(ks[0])
    groups = defaultdict(list)
    for k in lan:
        groups[find(k)].append(k)
    for ks in groups.values():
        rights = {lan[k][2] for k in ks}
        left_most = [k for k in ks if lan[k][1] not in rights] or ks
        t0, lw, _ = lan[left_most[0]]
        ref = [xy[i] for i in ways[lw] if i in xy]
        bounds = {w for k in ks for w in lan[k][1:]}
        yield t0, len(ks), ref, [[xy[i] for i in ways[w] if i in xy] for w in bounds]


def peaks(avg, off):
    med = np.median(avg)
    mad = np.median(np.abs(avg - med)) * 1.4826 + 1e-3
    hot = avg > med + 3 * mad
    out, i = [], 0
    while i < len(hot):
        if hot[i]:
            j = i
            while j < len(hot) and hot[j]:
                j += 1
            if 3 <= j - i <= 25:          # 6-50 cm: a painted line, not a kerb or a car
                out.append(float((off[i] + off[j - 1]) / 2))
            i = j
        else:
            i += 1
    return out


def audit(path, ortho, shift, min_length, window=20.0):
    """Per 20 m window along each road piece: the observed cross-road profile, its paint lines,
    and our lines' offsets there."""
    from shapely.geometry import LineString, Point
    off = np.arange(-HALF, HALF + 1e-9, PX)
    rows = []
    for t0, nlanes, ref, bounds in segments(path, shift):
        if len(ref) < 2 or LineString(ref).length < min_length:
            continue
        blines = [LineString(b) for b in bounds if len(b) >= 2]
        refl = LineString(ref)
        s = 0.0
        while s + window <= refl.length + 1e-6:
            acc = np.zeros(len(off))
            cnt = np.zeros(len(off))
            ours_acc = np.zeros(len(blines))
            n = 0
            for k in range(int(window)):
                a = refl.interpolate(s + k + 0.25)
                b = refl.interpolate(min(refl.length, s + k + 0.75))
                h = math.atan2(b.y - a.y, b.x - a.x)
                qx, qy = -math.sin(h), math.cos(h)
                px, py = a.x, a.y
                o = np.array([(lambda q: (q.x - px) * qx + (q.y - py) * qy)(bl.interpolate(bl.project(Point(px, py))))
                              for bl in blines])
                span = (off > o.min() - 0.5) & (off < o.max() + 0.5)
                v, ok = ortho.sample(px + qx * off, py + qy * off)
                if np.isnan(v).all() or ok[span].mean() < 0.6:
                    continue
                n += 1
                ours_acc += o
                acc += np.where(ok, np.nan_to_num(v), 0)
                cnt += ok
            s += window
            if n < window * 0.6:
                continue
            ours = sorted(float(x) for x in ours_acc / n)
            avg = np.where(cnt > 0, acc / np.maximum(cnt, 1), np.nan)
            span = (off > ours[0] - 1.0) & (off < ours[-1] + 1.0)
            prof = np.where(span, np.nan_to_num(avg, nan=float(np.nanmedian(avg[span]))), float(np.nanmedian(avg[span])))
            pk = peaks(prof, off)
            match = [min((abs(p - o), p - o) for p in pk)[1] if pk else None for o in ours]
            rows.append({
                "osm_way": t0.get("swb:osm_way"), "name": t0.get("name", ""), "at_m": round(s - window),
                "lanes": nlanes, "our_lines": [round(x, 2) for x in ours],
                "paint_lines": [round(p, 2) for p in pk],
                "offsets": [None if m is None else round(m, 2) for m in match],
            })
    return rows


def summary(rows):
    """Only windows that show paint inside our span count: a campus road with no paint says
    nothing about our lines."""
    painted = [r for r in rows if r["paint_lines"]]
    offs = np.array([abs(m) for r in painted for m in r["offsets"] if m is not None])
    inner = np.array([abs(m) for r in painted for i, m in enumerate(r["offsets"])
                      if m is not None and 0 < i < len(r["offsets"]) - 1])
    # lane count: paint lines within our span (+-1 m) against our lines
    count_eq = sum(1 for r in painted if len(r["paint_lines"]) == len(r["our_lines"]))
    return {"windows": len(rows), "windows_with_paint": len(painted),
            "lines_scored": int(offs.size),
            "abs_offset_median_m": round(float(np.median(offs)), 2) if offs.size else None,
            "abs_offset_p90_m": round(float(np.percentile(offs, 90)), 2) if offs.size else None,
            "within_0.3m": round(float((offs < 0.3).mean()), 3) if offs.size else None,
            "within_0.6m": round(float((offs < 0.6).mean()), 3) if offs.size else None,
            "inner_lines_abs_offset_median_m": round(float(np.median(inner)), 2) if inner.size else None,
            "paint_line_count_equals_ours": count_eq}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("ortho", type=Path)
    ap.add_argument("--json")
    ap.add_argument("--shift", default="0,0")
    ap.add_argument("--min-length", type=float, default=20.0)
    a = ap.parse_args()
    rows = audit(a.map, RgbOrtho(a.ortho), tuple(map(float, a.shift.split(","))), a.min_length)
    s = summary(rows)
    print(json.dumps(s, indent=1, ensure_ascii=False))
    for r in [r for r in rows if r["paint_lines"]][:30]:
        print(f"{r['name'] or r['osm_way']:14s} {r['osm_way']:>11s} @{r['at_m']:4d} m  lanes {r['lanes']}  "
              f"ours {r['our_lines']}  paint {r['paint_lines']}  offsets {r['offsets']}")
    if a.json:
        Path(a.json).write_text(json.dumps({"summary": s, "segments": rows}, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
