#!/usr/bin/env python3
"""Draw each signalised junction of a generated map over 059's 2 cm orthophoto, for the
junction audit by eye (roadmap 062, O4): what each approach lane may do against the arrows
painted on it. The orthophotos are ground truth and never an input to the map.

    scripts/open_maps/audit_junctions.py <lanelet2_map.osm> <ortho dir> <out dir>
        [--min-observed 0.3] [--half 40] [--all] [--nlsc <site.yaml> <raw dir>]

A junction is the stop lines within 45 m of each other (`--all`: lane ends within 25 m). Per junction, `<out dir>/j<k>.jpg`: the
photo (4 cm a pixel), every lane bound thin, each approach lane's last 25 m shaded, its stop
line red, and at its end its number and movements (L, S, R, U for a U-turn) in the order of the
lanes from the left. `<out dir>/junctions.json` lists the same per approach: road, lane, the
movements and where they came from (`swb:turn_source`: osm, mapillary, rule, with +gap or
+fallback). Junctions whose box the survey observed less than `--min-observed` of are skipped.
`--nlsc` draws on NLSC's open 0.27 m orthophoto (fetch.py's mosaic, resampled into the map
frame at 0.1 m) instead, which covers every junction of the site, not only 059's survey; the
arrows on an arterial's approach read on it where trees and vehicles leave them clear. Then the
ortho dir is not read. The verdicts go in the site's audit file (`sites/<site>_audit.yaml`), which generate.py reads.
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from overlay import PX, TILE, mosaic

LETTER = {"left": "L", "straight": "S", "right": "R", "uturn": "U"}


def tags(e):
    return {t.get("k"): t.get("v") for t in e.findall("tag")}


def load(path):
    r = ET.parse(path).getroot()
    xy = {n.get("id"): (float(tags(n)["local_x"]), float(tags(n)["local_y"])) for n in r.findall("node")}
    ways = {w.get("id"): ([nd.get("ref") for nd in w.findall("nd")], tags(w)) for w in r.findall("way")}
    lanes, regs = {}, {}
    for rel in r.findall("relation"):
        t = tags(rel)
        m = [(x.get("role"), x.get("ref")) for x in rel.findall("member")]
        if t.get("type") == "lanelet" and t.get("subtype") == "road":
            d = dict(m)
            lanes[rel.get("id")] = {"t": t, "l": ways[d["left"]][0], "r": ways[d["right"]][0],
                                    "regs": [ref for role, ref in m if role == "regulatory_element"]}
        elif t.get("subtype") == "traffic_light":
            regs[rel.get("id")] = [ways[ref][0] for role, ref in m if role == "ref_line"][0]
    return xy, ways, lanes, regs


class Nlsc:
    """fetch.py's NLSC mosaic (Web Mercator tiles) read in the map frame."""

    def __init__(self, site, raw):
        import yaml
        from generate import Frame
        Image.MAX_IMAGE_PIXELS = None
        self.frame = Frame(yaml.safe_load(Path(site).read_text())["frame"])
        raw = Path(raw)
        self.meta = json.loads((raw / "nlsc_PHOTO2_z19.json").read_text())
        self.img = np.asarray(Image.open(raw / "nlsc_PHOTO2_z19.png").convert("RGB"))

    def render(self, box, res):
        x0, y0, x1, y1 = box
        return self.sample(*np.meshgrid(np.arange(x0, x1, res), np.arange(y1, y0, -res)))

    def sample(self, X, Y):
        """The photo at map points X, Y (arrays of one shape), bilinear."""
        lon, lat = self.frame.inv.transform(X + self.frame.ox, Y + self.frame.oy)
        m = self.meta
        merc = lambda la: np.log(np.tan(np.pi / 4 + np.radians(la) / 2))
        H, W = self.img.shape[:2]
        u = (np.radians(lon) - math.radians(m["west"])) / (math.radians(m["east"]) - math.radians(m["west"])) * W - 0.5
        v = (merc(lat) - merc(m["north"])) / (merc(m["south"]) - merc(m["north"])) * H - 0.5
        ok = (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
        u, v = np.clip(u, 0, W - 1.001), np.clip(v, 0, H - 1.001)
        i, j = v.astype(int), u.astype(int)
        fu, fv = (u - j)[..., None], (v - i)[..., None]
        a = self.img
        out = (a[i, j] * (1 - fu) * (1 - fv) + a[i, j + 1] * fu * (1 - fv) +
               a[i + 1, j] * (1 - fu) * fv + a[i + 1, j + 1] * fu * fv)
        out[~ok] = 0
        return Image.fromarray(out.astype(np.uint8))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("ortho", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--min-observed", type=float, default=0.3)
    ap.add_argument("--half", type=float, default=40.0)
    ap.add_argument("--all", action="store_true", help="every junction, not only the signalised")
    ap.add_argument("--nlsc", nargs=2, metavar=("SITE", "RAW"), help="NLSC's photo as the background")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    xy, ways, lanes, regs = load(a.map)
    tiles = {(int(p.stem.split("_")[1]), int(p.stem.split("_")[2])) for p in a.ortho.glob("rgb_*.jpg")}
    photo = Nlsc(*a.nlsc) if a.nlsc else None
    succ = defaultdict(list)
    start_of = defaultdict(list)
    for k, ln in lanes.items():
        start_of[(ln["l"][0], ln["r"][0])].append(k)
    for k, ln in lanes.items():
        succ[k] = start_of.get((ln["l"][-1], ln["r"][-1]), [])

    def mid(ids):
        p, q = xy[ids[0]], xy[ids[-1]]
        return ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
    # approaches: lanelets that hold a light (--all: that lead into a junction path), grouped by
    # their stop line (--all: their end); junctions: those 45 m apart at most
    appr = [(k, mid(regs[g])) for k, ln in lanes.items() for g in ln["regs"] if g in regs]
    if a.all:
        appr = [(k, mid([ln["l"][-1], ln["r"][-1]])) for k, ln in lanes.items()
                if "turn_direction" not in ln["t"] and any("turn_direction" in lanes[x]["t"] for x in succ[k])]
    groups = []
    for k, p in appr:
        hit = [g for g in groups if any(math.dist(p, q) < (25 if a.all else 45) for _, q in g)]
        merged = [x for g in hit for x in g] + [(k, p)]
        groups = [g for g in groups if g not in hit] + [merged]
    font = ImageFont.load_default(size=22)
    report = []
    for jn, g in enumerate(sorted(groups, key=lambda g: (round(g[0][1][1], -1), g[0][1][0]))):
        cx = sum(p[0] for _, p in g) / len(g)
        cy = sum(p[1] for _, p in g) / len(g)
        box = (cx - a.half, cy - a.half, cx + a.half, cy + a.half)
        need = {(ix, iy) for ix in range(int(box[0] // TILE), int(math.ceil(box[2] / TILE)))
                for iy in range(int(box[1] // TILE), int(math.ceil(box[3] / TILE)))}
        if photo is None and not need & tiles:
            continue
        step = 5 if photo else 2
        img = photo.render(box, PX * step) if photo else mosaic(a.ortho, box, "rgb", step).convert("RGB")
        seen = (np.asarray(img).sum(-1) > 0).mean()
        if seen < a.min_observed:
            continue
        s = PX * step

        def px(p):
            return ((p[0] - box[0]) / s, (box[3] - p[1]) / s)
        over = Image.new("RGBA", img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(over)
        for k, ln in lanes.items():
            for side in ("l", "r"):
                pts = [xy[i] for i in ln[side]]
                if any(box[0] - 10 < p[0] < box[2] + 10 and box[1] - 10 < p[1] < box[3] + 10 for p in pts):
                    col = (80, 160, 255, 160) if "turn_direction" in ln["t"] else (255, 255, 255, 200)
                    d.line([px(p) for p in pts], fill=col, width=1 if "turn_direction" in ln["t"] else 2)
        entries = []
        approaches = defaultdict(list)   # road lanelets grouped by their stop line
        for k, p in g:
            for gid in lanes[k]["regs"]:
                if gid in regs:
                    approaches[gid].append(k)
            if a.all and not any(gid in regs for gid in lanes[k]["regs"]):
                # no stop line: the lanes side by side are the chain's, ending on one line
                key = next((gk for gk, ks in approaches.items() if gk.startswith("n")
                            and any(lanes[k]["l"][-1] in (lanes[x]["r"][-1], lanes[x]["l"][-1])
                                    or lanes[k]["r"][-1] in (lanes[x]["l"][-1], lanes[x]["r"][-1]) for x in ks)), None)
                approaches[key or f"n{k}"].append(k)
                regs.setdefault(key or f"n{k}", None)
        def line_of(gid, ks):   # the stop line, or one across the lanes' ends where none
            if regs.get(gid):
                return regs[gid]
            ends = [e for k in ks for e in (lanes[k]["l"][-1], lanes[k]["r"][-1])]
            u = (xy[lanes[ks[0]]["r"][-1]][0] - xy[lanes[ks[0]]["l"][-1]][0],
                 xy[lanes[ks[0]]["r"][-1]][1] - xy[lanes[ks[0]]["l"][-1]][1])
            ends.sort(key=lambda e: xy[e][0] * u[0] + xy[e][1] * u[1])
            return [ends[0], ends[-1]]
        for an, (gid, ks) in enumerate(sorted(approaches.items(), key=lambda kv: mid(line_of(*kv)))):
            line = line_of(gid, ks)
            if regs.get(gid):
                d.line([px(xy[i]) for i in line], fill=(255, 30, 30, 255), width=5)
            # lanes from the left: by where their end lies along the stop line
            L0, R0 = xy[line[0]], xy[line[-1]]
            along = lambda k: ((xy[lanes[k]["l"][-1]][0] - L0[0]) * (R0[0] - L0[0]) +
                               (xy[lanes[k]["l"][-1]][1] - L0[1]) * (R0[1] - L0[1]))
            ks = sorted(set(ks), key=along)
            lane_moves = []
            for i, k in enumerate(ks):
                ln = lanes[k]
                poly = [xy[j] for j in ln["l"]] + [xy[j] for j in ln["r"]][::-1]
                d.polygon([px(p) for p in poly], fill=(255, 220, 0, 50))
                mv, src = set(), set()
                for sk in succ[k]:
                    st = lanes[sk]["t"]
                    if "turn_direction" in st:
                        mv.add("uturn" if st.get("swb:uturn") else st["turn_direction"])
                        src.add(st.get("swb:turn_source", ""))
                word = "".join(LETTER[m] for m in ("uturn", "left", "straight", "right") if m in mv)
                e = mid([ln["l"][-1], ln["r"][-1]])
                d.text(px(e), f"{an}.{i} {word}", fill=(255, 255, 0, 255), font=font, stroke_width=3,
                       stroke_fill=(0, 0, 0, 255), anchor="mm")
                lane_moves.append({"lanelet": int(k), "moves": sorted(mv), "source": sorted(src)})
            entries.append({"approach": an, "road": lanes[ks[0]]["t"].get("name", ""), "signal": bool(regs.get(gid)),
                            "stop_line": [round(v, 1) for v in mid(line)],
                            "heading_deg": round(math.degrees(math.atan2(R0[0] - L0[0], -(R0[1] - L0[1])) % 360)),
                            "lanes": lane_moves})
        img = Image.alpha_composite(img.convert("RGBA"), over).convert("RGB")
        ImageDraw.Draw(img).text((10, 10), f"j{jn}  ({cx:.0f}, {cy:.0f})  observed {seen:.0%}", fill=(255, 255, 255),
                                 font=font, stroke_width=3, stroke_fill=(0, 0, 0))
        img.save(a.out / f"j{jn}.jpg", quality=80)
        report.append({"junction": jn, "centre": [round(cx), round(cy)], "observed": round(float(seen), 2),
                       "approaches": entries})
        print(f"j{jn} ({cx:.0f}, {cy:.0f}) observed {seen:.0%}: " + "; ".join(
            f"{e['approach']} {e['road']} " + "|".join("".join(LETTER[m] for m in ("uturn", "left", "straight", "right")
                                                              if m in ln["moves"]) for ln in e["lanes"])
            for e in entries))
    (a.out / "junctions.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
