#!/usr/bin/env python3
"""Draw a generated map's lane bounds over 059's 2 cm orthophoto, for review (roadmap 062, O4).

    scripts/open_maps/overlay.py <lanelet2_map.osm> <ortho dir> <x0,y0,x1,y1> <out.png>
        [--vendor <vendor lanelet2_map.osm>] [--channel rgb|int] [--shift dx,dy]

The box is in the vendor frame V (059's orthos: tile (ix, iy) covers [ix*50, ix*50+50) x
[iy*50, iy*50+50) at 0.02 m, row 0 at the north edge). The generated map is in true
MGRS-local (WGS84), taken as V; `--shift` moves it if they differ. The vendor's lanelets are
in 059's map frame M = V - (-0.54, +0.20), so they are moved by (-0.54, +0.20) back to V.
"""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

TILE, PX = 50.0, 0.02
VENDOR_SHIFT = (-0.54, 0.20)   # V = M + SHIFT


def tags(e):
    return {t.get("k"): t.get("v") for t in e.findall("tag")}


def bounds(path, shift=(0.0, 0.0)):
    """[(points, kind)] of every lanelet bound way: kind solid, dashed, virtual, other."""
    r = ET.parse(path).getroot()
    xy = {}
    for n in r.findall("node"):
        t = tags(n)
        if "local_x" in t:
            xy[n.get("id")] = (float(t["local_x"]) + shift[0], float(t["local_y"]) + shift[1])
    used = set()
    for rel in r.findall("relation"):
        if tags(rel).get("type") == "lanelet":
            for m in rel.findall("member"):
                if m.get("role") in ("left", "right"):
                    used.add(m.get("ref"))
    out = []
    for w in r.findall("way"):
        if w.get("id") not in used:
            continue
        t = tags(w)
        kind = t.get("subtype") if t.get("type") == "line_thin" else t.get("type", "other")
        pts = [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
        if len(pts) >= 2:
            out.append((pts, kind))
    return out


def mosaic(ortho, box, channel, step=1):
    x0, y0, x1, y1 = box
    W = int(round((x1 - x0) / PX)) // step
    H = int(round((y1 - y0) / PX)) // step
    img = np.zeros((H, W, 3), np.uint8)
    for ix in range(int(x0 // TILE), int(math.ceil(x1 / TILE))):
        for iy in range(int(y0 // TILE), int(math.ceil(y1 / TILE))):
            f = ortho / (f"rgb_{ix}_{iy}.jpg" if channel == "rgb" else f"int_{ix}_{iy}.png")
            if not f.exists():
                continue
            a = np.asarray(Image.open(f))
            if channel == "int":
                v = a.astype(np.float32)
                lo, hi = np.percentile(v[v > 0], [2, 99.5]) if (v > 0).any() else (0, 1)
                a = np.clip((v - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
                a = np.stack([a] * 3, -1)
            # tile pixel (r, c) -> box pixel
            tx0, ty1 = ix * TILE, iy * TILE + TILE
            c0 = int(round((tx0 - x0) / PX))
            r0 = int(round((y1 - ty1) / PX))
            a = a[::step, ::step]
            rs, cs = r0 // step, c0 // step
            h, w = a.shape[:2]
            ra, rb = max(0, rs), min(H, rs + h)
            ca, cb = max(0, cs), min(W, cs + w)
            if ra < rb and ca < cb:
                img[ra:rb, ca:cb] = a[ra - rs:rb - rs, ca - cs:cb - cs]
    return Image.fromarray(img)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("ortho", type=Path)
    ap.add_argument("box")
    ap.add_argument("out")
    ap.add_argument("--vendor")
    ap.add_argument("--channel", default="rgb")
    ap.add_argument("--shift", default="0,0")
    ap.add_argument("--step", type=int, default=0, help="pixel decimation (0: fit 2400 px)")
    a = ap.parse_args()
    box = tuple(map(float, a.box.split(",")))
    step = a.step or max(1, int(math.ceil((box[2] - box[0]) / PX / 2400)))
    img = mosaic(a.ortho, box, a.channel, step).convert("RGB")
    d = ImageDraw.Draw(img)
    s = PX * step

    def px(p):
        return ((p[0] - box[0]) / s, (box[3] - p[1]) / s)
    colours = {"solid": (255, 40, 40), "dashed": (255, 160, 0), "virtual": (80, 160, 255)}
    if a.vendor:
        for pts, kind in bounds(a.vendor, VENDOR_SHIFT):
            d.line([px(p) for p in pts], fill=(0, 255, 120), width=2)
    for pts, kind in bounds(a.map, tuple(map(float, a.shift.split(",")))):
        d.line([px(p) for p in pts], fill=colours.get(kind, (255, 0, 255)), width=2)
    img.save(a.out)


if __name__ == "__main__":
    main()
